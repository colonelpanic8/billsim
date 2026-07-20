//! Pooltool-compatible cue impact and cloth-motion equations.

use crate::math::{Vec2, Vec3};
use crate::model::{BallParams, CueSpecs, CueStrike, MotionState};
use crate::table::{CircularCushion, LinearCushion};

const PHYSICS_EPSILON: f64 = f64::EPSILON * 100.0;
const CUSHION_MAX_STEPS: usize = 1_000;
const CUSHION_MAX_STEPS_FLOAT: f64 = 1_000.0;
const CUSHION_DELTA_P: f64 = 0.001;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct KinematicState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub motion: MotionState,
}

impl KinematicState {
    pub(crate) fn stationary(position: Vec2, radius: f64) -> Self {
        Self {
            position: Vec3::new(position.x, position.y, radius),
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            motion: MotionState::Stationary,
        }
    }
}

/// Apply Pooltool's instantaneous-point cue model to a stationary ball.
pub(crate) fn strike(
    state: KinematicState,
    strike: CueStrike,
    ball: BallParams,
    cue: CueSpecs,
) -> KinematicState {
    let theta = strike.theta.to_radians();
    let cue_c = (1.0 - strike.a * strike.a - strike.b * strike.b).sqrt();
    let ball_a = strike.a;
    let ball_c = theta.cos() * cue_c - theta.sin() * strike.b;
    let ball_b = theta.sin() * cue_c + theta.cos() * strike.b;
    let contact = Vec3::new(ball_a, ball_c, ball_b) * ball.radius;

    let inertia_over_mass = (2.0 / 5.0) * ball.radius * ball.radius;
    let temp = contact.x * contact.x
        + (contact.z * theta.cos()).powi(2)
        + (contact.y * theta.sin()).powi(2)
        - 2.0 * contact.z * contact.y * theta.cos() * theta.sin();
    let impulse_speed =
        (2.0 * strike.speed) / (1.0 + ball.mass / cue.mass + temp / inertia_over_mass);

    // Pooltool 0.4.4 is a 2D simulator: cue elevation changes the horizontal
    // speed and spin, but never gives the ball vertical velocity.
    let velocity_ball = Vec3::new(0.0, -impulse_speed * theta.cos(), 0.0);
    let angular_ball = Vec3::new(
        -contact.y * theta.sin() + contact.z * theta.cos(),
        contact.x * theta.sin(),
        -contact.x * theta.cos(),
    ) * (impulse_speed / inertia_over_mass);

    let rotation = strike.phi.to_radians() + std::f64::consts::FRAC_PI_2;
    let mut velocity = velocity_ball.rotate_xy(rotation);
    let angular_velocity = angular_ball.rotate_xy(rotation);

    let squirt = squirt_angle(ball.mass, cue.end_mass, ball_a);
    velocity = velocity.rotate_xy(squirt);

    KinematicState {
        position: state.position,
        velocity,
        angular_velocity,
        motion: MotionState::Sliding,
    }
}

fn squirt_angle(ball_mass: f64, cue_end_mass: f64, side_offset: f64) -> f64 {
    let mass_ratio = ball_mass / cue_end_mass;
    let remaining_radius_squared = 1.0 - side_offset * side_offset;
    let numerator = (5.0 / 2.0) * side_offset * remaining_radius_squared.sqrt();
    let denominator = 1.0 + mass_ratio + (5.0 / 2.0) * remaining_radius_squared;
    -numerator.atan2(denominator)
}

pub(crate) fn transition_time(state: KinematicState, params: BallParams) -> f64 {
    match state.motion {
        MotionState::Stationary | MotionState::Pocketed => f64::INFINITY,
        MotionState::Sliding => slide_time(state, params),
        MotionState::Rolling => roll_time(state, params),
        MotionState::Spinning => spin_time(state, params),
    }
}

/// Apply one already-reached motion transition.
pub(crate) fn resolve_transition(mut state: KinematicState) -> KinematicState {
    state.motion = match state.motion {
        MotionState::Sliding => MotionState::Rolling,
        MotionState::Rolling if state.angular_velocity.z.abs() > PHYSICS_EPSILON => {
            MotionState::Spinning
        }
        MotionState::Rolling | MotionState::Spinning => MotionState::Stationary,
        MotionState::Stationary | MotionState::Pocketed => return state,
    };
    if state.motion == MotionState::Stationary {
        state.velocity = Vec3::ZERO;
        state.angular_velocity = Vec3::ZERO;
    }
    state
}

pub(crate) fn relative_velocity(state: KinematicState, radius: f64) -> Vec3 {
    state.velocity + state.angular_velocity.cross(Vec3::new(0.0, 0.0, -radius))
}

fn slide_time(state: KinematicState, params: BallParams) -> f64 {
    if params.sliding_friction == 0.0 {
        return f64::INFINITY;
    }
    2.0 * relative_velocity(state, params.radius).norm()
        / (7.0 * params.sliding_friction * params.gravity)
}

fn roll_time(state: KinematicState, params: BallParams) -> f64 {
    if params.rolling_friction == 0.0 {
        return f64::INFINITY;
    }
    state.velocity.norm() / (params.rolling_friction * params.gravity)
}

fn spin_time(state: KinematicState, params: BallParams) -> f64 {
    let spin_friction = params.spinning_friction();
    if spin_friction == 0.0 {
        return f64::INFINITY;
    }
    state.angular_velocity.z.abs() * (2.0 / 5.0) * params.radius / (spin_friction * params.gravity)
}

/// Evolve across any number of cloth-motion transitions.
#[cfg(test)]
pub(crate) fn evolve(
    mut state: KinematicState,
    mut time: f64,
    params: BallParams,
) -> KinematicState {
    loop {
        if matches!(
            state.motion,
            MotionState::Stationary | MotionState::Pocketed
        ) {
            return state;
        }

        let until_transition = transition_time(state, params);
        if time < until_transition {
            return evolve_current_motion(state, time, params);
        }

        state = evolve_current_motion(state, until_transition, params);
        time -= until_transition;
        state = resolve_transition(state);
    }
}

fn evolve_current_motion(state: KinematicState, time: f64, params: BallParams) -> KinematicState {
    match state.motion {
        MotionState::Sliding => evolve_sliding(state, time, params),
        MotionState::Rolling => evolve_rolling(state, time, params),
        MotionState::Spinning => evolve_spinning(state, time, params),
        MotionState::Stationary | MotionState::Pocketed => state,
    }
}

/// Evolve within the current motion state without applying a transition.
pub(crate) fn evolve_until_event(
    state: KinematicState,
    time: f64,
    params: BallParams,
) -> KinematicState {
    evolve_current_motion(state, time, params)
}

/// Resolve Pooltool's default frictional/inelastic equal-ball collision.
pub(crate) fn resolve_ball_ball(
    first: KinematicState,
    second: KinematicState,
    params: BallParams,
) -> (KinematicState, KinematicState) {
    let (mut first, mut second) = make_kiss(first, second, params.radius);
    let delta = second.position - first.position;
    let theta = delta.y.atan2(delta.x);

    first.velocity = first.velocity.rotate_xy(-theta);
    first.angular_velocity = first.angular_velocity.rotate_xy(-theta);
    second.velocity = second.velocity.rotate_xy(-theta);
    second.angular_velocity = second.angular_velocity.rotate_xy(-theta);

    let first_normal = 0.5
        * ((1.0 - params.ball_restitution) * first.velocity.x
            + (1.0 + params.ball_restitution) * second.velocity.x);
    let second_normal = 0.5
        * ((1.0 + params.ball_restitution) * first.velocity.x
            + (1.0 - params.ball_restitution) * second.velocity.x);
    let normal_delta = (second_normal - first_normal).abs();

    first.velocity.x = 0.0;
    second.velocity.x = 0.0;
    let mut first_final = first;
    let mut second_final = second;

    let unit_x = Vec3::new(1.0, 0.0, 0.0);
    let first_contact = surface_velocity(first, unit_x, params.radius);
    let second_contact = surface_velocity(second, -unit_x, params.radius);
    let relative_contact = first_contact - second_contact;

    if let Some(relative_hat) = relative_contact.normalized() {
        let friction = alciatore_friction(first, second, params.radius);
        let first_tangent_delta = -relative_hat * (friction * normal_delta);
        let first_angular_delta = unit_x.cross(first_tangent_delta) * (2.5 / params.radius);
        first_final.velocity = first.velocity + first_tangent_delta;
        first_final.angular_velocity = first.angular_velocity + first_angular_delta;
        second_final.velocity = second.velocity - first_tangent_delta;
        second_final.angular_velocity = second.angular_velocity + first_angular_delta;

        let slip_relative = surface_velocity(first_final, unit_x, params.radius)
            - surface_velocity(second_final, -unit_x, params.radius);
        if relative_contact.dot(slip_relative) <= 0.0 {
            (first_final, second_final) = no_slip_collision(first, second, params.radius);
        }
    } else {
        (first_final, second_final) = no_slip_collision(first, second, params.radius);
    }

    first_final.velocity.x = first_normal;
    second_final.velocity.x = second_normal;
    first_final.velocity = first_final.velocity.rotate_xy(theta);
    first_final.angular_velocity = first_final.angular_velocity.rotate_xy(theta);
    second_final.velocity = second_final.velocity.rotate_xy(theta);
    second_final.angular_velocity = second_final.angular_velocity.rotate_xy(theta);
    first_final.velocity.z = 0.0;
    second_final.velocity.z = 0.0;
    first_final.motion = MotionState::Sliding;
    second_final.motion = MotionState::Sliding;
    (first_final, second_final)
}

pub(crate) fn resolve_linear_cushion(
    mut state: KinematicState,
    cushion: LinearCushion,
    params: BallParams,
) -> KinematicState {
    let mut normal = cushion.normal();
    if normal.dot(state.velocity.xy()) <= 0.0 {
        normal = -normal;
    }
    let delta = cushion.p2 - cushion.p1;
    let score = (state.position.xy() - cushion.p1).dot(delta) / delta.norm_squared();
    let closest = cushion.p1 + delta * score;
    let correction = params.radius - (state.position.xy() - closest).norm() + 1e-9;
    let corrected = state.position.xy() - normal * correction;
    state.position.x = corrected.x;
    state.position.y = corrected.y;
    resolve_mathavan(state, normal, cushion.height, params)
}

pub(crate) fn resolve_circular_cushion(
    mut state: KinematicState,
    cushion: CircularCushion,
    params: BallParams,
) -> KinematicState {
    let radial = state.position.xy() - cushion.center;
    let mut normal = radial.normalized().unwrap_or(Vec2::new(1.0, 0.0));
    if normal.dot(state.velocity.xy()) <= 0.0 {
        normal = -normal;
    }
    let correction = params.radius + cushion.radius - radial.norm() - 1e-9;
    let corrected = state.position.xy() + normal * correction;
    state.position.x = corrected.x;
    state.position.y = corrected.y;
    resolve_mathavan(state, normal, cushion.height, params)
}

fn resolve_mathavan(
    mut state: KinematicState,
    normal: Vec2,
    cushion_height: f64,
    params: BallParams,
) -> KinematicState {
    let angle = std::f64::consts::FRAC_PI_2 - normal.y.atan2(normal.x);
    let velocity = state.velocity.rotate_xy(angle);
    let angular = state.angular_velocity.rotate_xy(angle);
    let (vx, vy, wx, wy, wz) = solve_mathavan_components(
        velocity.x,
        velocity.y,
        angular.x,
        angular.y,
        angular.z,
        cushion_height,
        params,
    );
    state.velocity = Vec3::new(vx, vy, 0.0).rotate_xy(-angle);
    state.angular_velocity = Vec3::new(wx, wy, wz).rotate_xy(-angle);
    state.motion = MotionState::Sliding;
    state
}

#[allow(clippy::too_many_arguments)]
#[allow(clippy::similar_names)]
fn solve_mathavan_components(
    mut vx: f64,
    mut vy: f64,
    mut wx: f64,
    mut wy: f64,
    mut wz: f64,
    cushion_height: f64,
    params: BallParams,
) -> (f64, f64, f64, f64, f64) {
    let sin_theta = (cushion_height - params.radius) / params.radius;
    let cos_theta = (1.0 - sin_theta * sin_theta).sqrt();
    let mut work = 0.0;
    let mut steps = 0;
    let delta_p = (params.mass * vy / CUSHION_MAX_STEPS_FLOAT).max(CUSHION_DELTA_P);

    while vy > 0.0 && steps <= 10 * CUSHION_MAX_STEPS {
        let (slip, table_slip) =
            cushion_slip_angles(params.radius, sin_theta, cos_theta, vx, vy, wx, wy, wz);
        let (next_vx, next_vy) = cushion_velocity_step(
            params, sin_theta, cos_theta, vx, vy, slip, table_slip, delta_p,
        );
        if next_vy <= 0.0 {
            let mut refine_delta = delta_p;
            for _ in 0..8 {
                refine_delta /= 2.0;
                let (refine_slip, refine_table_slip) =
                    cushion_slip_angles(params.radius, sin_theta, cos_theta, vx, vy, wx, wy, wz);
                let (test_vx, test_vy) = cushion_velocity_step(
                    params,
                    sin_theta,
                    cos_theta,
                    vx,
                    vy,
                    refine_slip,
                    refine_table_slip,
                    refine_delta,
                );
                if test_vy <= 0.0 {
                    continue;
                }
                vx = test_vx;
                vy = test_vy;
                (wx, wy, wz) = cushion_angular_step(
                    params,
                    sin_theta,
                    cos_theta,
                    wx,
                    wy,
                    wz,
                    refine_slip,
                    refine_table_slip,
                    refine_delta,
                );
                work += refine_delta * vy.abs() * cos_theta;
            }
            break;
        }
        vx = next_vx;
        vy = next_vy;
        (wx, wy, wz) = cushion_angular_step(
            params, sin_theta, cos_theta, wx, wy, wz, slip, table_slip, delta_p,
        );
        work += delta_p * vy.abs() * cos_theta;
        steps += 1;
    }

    let target = params.cushion_restitution.powi(2) * work;
    let mut rebound_work = 0.0;
    steps = 0;
    let delta_p = (target / CUSHION_MAX_STEPS_FLOAT).max(CUSHION_DELTA_P);
    while rebound_work < target && steps <= 10 * CUSHION_MAX_STEPS {
        let (slip, table_slip) =
            cushion_slip_angles(params.radius, sin_theta, cos_theta, vx, vy, wx, wy, wz);
        let next_work = delta_p * vy.abs() * cos_theta;
        if rebound_work + next_work > target {
            let remaining = target - rebound_work;
            let refine_delta = remaining / (vy.abs() * cos_theta);
            (vx, vy) = cushion_velocity_step(
                params,
                sin_theta,
                cos_theta,
                vx,
                vy,
                slip,
                table_slip,
                refine_delta,
            );
            (wx, wy, wz) = cushion_angular_step(
                params,
                sin_theta,
                cos_theta,
                wx,
                wy,
                wz,
                slip,
                table_slip,
                refine_delta,
            );
            break;
        }
        (vx, vy) = cushion_velocity_step(
            params, sin_theta, cos_theta, vx, vy, slip, table_slip, delta_p,
        );
        (wx, wy, wz) = cushion_angular_step(
            params, sin_theta, cos_theta, wx, wy, wz, slip, table_slip, delta_p,
        );
        rebound_work += delta_p * vy.abs() * cos_theta;
        steps += 1;
    }
    (vx, vy, wx, wy, wz)
}

#[allow(clippy::too_many_arguments)]
fn cushion_slip_angles(
    radius: f64,
    sin_theta: f64,
    cos_theta: f64,
    vx: f64,
    vy: f64,
    wx: f64,
    wy: f64,
    wz: f64,
) -> (f64, f64) {
    let cushion_x = vx + wy * radius * sin_theta - wz * radius * cos_theta;
    let cushion_y = -vy * sin_theta + wx * radius;
    let table_x = vx - wy * radius;
    let table_y = vy + wx * radius;
    (
        cushion_y
            .atan2(cushion_x)
            .rem_euclid(2.0 * std::f64::consts::PI),
        table_y
            .atan2(table_x)
            .rem_euclid(2.0 * std::f64::consts::PI),
    )
}

#[allow(clippy::too_many_arguments)]
fn cushion_velocity_step(
    params: BallParams,
    sin_theta: f64,
    cos_theta: f64,
    vx: f64,
    vy: f64,
    slip: f64,
    table_slip: f64,
    delta_p: f64,
) -> (f64, f64) {
    let common = sin_theta + params.cushion_friction * slip.sin() * cos_theta;
    let vx = vx
        - (params.cushion_friction * slip.cos()
            + params.sliding_friction * table_slip.cos() * common)
            * delta_p
            / params.mass;
    let vy = vy
        - (cos_theta - params.cushion_friction * sin_theta * slip.sin()
            + params.sliding_friction * table_slip.sin() * common)
            * delta_p
            / params.mass;
    (vx, vy)
}

#[allow(clippy::too_many_arguments)]
fn cushion_angular_step(
    params: BallParams,
    sin_theta: f64,
    cos_theta: f64,
    wx: f64,
    wy: f64,
    wz: f64,
    slip: f64,
    table_slip: f64,
    delta_p: f64,
) -> (f64, f64, f64) {
    let factor = 5.0 / (2.0 * params.mass * params.radius);
    let common = sin_theta + params.cushion_friction * slip.sin() * cos_theta;
    let wx = wx
        - factor
            * (params.cushion_friction * slip.sin()
                + params.sliding_friction * table_slip.sin() * common)
            * delta_p;
    let wy = wy
        - factor
            * (params.cushion_friction * slip.cos() * sin_theta
                - params.sliding_friction * table_slip.cos() * common)
            * delta_p;
    let wz = wz + factor * params.cushion_friction * slip.cos() * cos_theta * delta_p;
    (wx, wy, wz)
}

fn make_kiss(
    mut first: KinematicState,
    mut second: KinematicState,
    radius: f64,
) -> (KinematicState, KinematicState) {
    let delta = second.position - first.position;
    let distance = delta.norm();
    if let Some(normal) = delta.normalized() {
        let correction = 2.0 * radius - distance + 1e-9;
        second.position += normal * (correction / 2.0);
        first.position -= normal * (correction / 2.0);
    }
    (first, second)
}

fn surface_velocity(state: KinematicState, direction: Vec3, radius: f64) -> Vec3 {
    state.velocity + state.angular_velocity.cross(direction * radius)
}

fn alciatore_friction(first: KinematicState, second: KinematicState, radius: f64) -> f64 {
    let unit_x = Vec3::new(1.0, 0.0, 0.0);
    let first_contact =
        surface_velocity(first, unit_x, radius) - Vec3::new(first.velocity.x, 0.0, 0.0);
    let second_contact =
        surface_velocity(second, -unit_x, radius) - Vec3::new(second.velocity.x, 0.0, 0.0);
    let relative_surface_speed = (first_contact - second_contact).norm();
    0.009_951 + 0.108 * (-1.088 * relative_surface_speed).exp()
}

fn no_slip_collision(
    first: KinematicState,
    second: KinematicState,
    radius: f64,
) -> (KinematicState, KinematicState) {
    let unit_x = Vec3::new(1.0, 0.0, 0.0);
    let tangent_delta = -(first.velocity - second.velocity
        + (first.angular_velocity + second.angular_velocity).cross(unit_x) * radius)
        * (1.0 / 7.0);
    let angular_delta = -(unit_x.cross(first.velocity - second.velocity) / radius
        + first.angular_velocity
        + second.angular_velocity)
        * (5.0 / 14.0);

    let mut first_final = first;
    let mut second_final = second;
    first_final.velocity += tangent_delta;
    first_final.angular_velocity += angular_delta;
    second_final.velocity -= tangent_delta;
    second_final.angular_velocity += angular_delta;
    (first_final, second_final)
}

fn evolve_sliding(mut state: KinematicState, time: f64, params: BallParams) -> KinematicState {
    if time == 0.0 {
        return state;
    }
    let friction_direction = relative_velocity(state, params.radius)
        .normalized()
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let deceleration = params.sliding_friction * params.gravity;

    state.position +=
        state.velocity * time - friction_direction * (0.5 * deceleration * time * time);
    state.velocity -= friction_direction * (deceleration * time);
    state.angular_velocity -= friction_direction.cross(Vec3::new(0.0, 0.0, 1.0))
        * ((5.0 / (2.0 * params.radius)) * deceleration * time);
    state.angular_velocity.z = evolve_spin_component(state.angular_velocity.z, params, time);
    state
}

fn evolve_rolling(mut state: KinematicState, time: f64, params: BallParams) -> KinematicState {
    if time == 0.0 {
        return state;
    }
    let direction = state
        .velocity
        .normalized()
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let deceleration = params.rolling_friction * params.gravity;
    let final_velocity = state.velocity - direction * (deceleration * time);

    state.position += state.velocity * time - direction * (0.5 * deceleration * time * time);
    state.velocity = final_velocity;
    let final_spin_z = evolve_spin_component(state.angular_velocity.z, params, time);
    // Use the same explicit π/2 coordinate rotation as Pooltool. Besides being
    // the rolling constraint, this preserves its floating-point behavior at
    // cardinal angles for differential conformance.
    let rolling_spin = final_velocity.xy().rotate(std::f64::consts::FRAC_PI_2) / params.radius;
    state.angular_velocity = Vec3::new(rolling_spin.x, rolling_spin.y, final_spin_z);
    state
}

fn evolve_spinning(mut state: KinematicState, time: f64, params: BallParams) -> KinematicState {
    state.angular_velocity.z = evolve_spin_component(state.angular_velocity.z, params, time);
    state
}

fn evolve_spin_component(spin_z: f64, params: BallParams, time: f64) -> f64 {
    if time == 0.0 || spin_z.abs() < PHYSICS_EPSILON {
        return spin_z;
    }
    let angular_deceleration =
        5.0 * params.spinning_friction() * params.gravity / (2.0 * params.radius);
    let duration = time.min(spin_z.abs() / angular_deceleration);
    spin_z - spin_z.signum() * angular_deceleration * duration
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::table::{CushionDirection, LinearCushion};

    fn approx(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
    }

    #[test]
    fn center_strike_matches_closed_form_speed() {
        let params = BallParams::default();
        let cue = CueSpecs::default();
        let state = KinematicState::stationary(Vec2::new(0.5, 1.0), params.radius);
        let struck = strike(state, CueStrike::new(2.0, 0.0), params, cue);
        let expected = 4.0 / (1.0 + params.mass / cue.mass);

        approx(struck.velocity.x, expected);
        approx(struck.velocity.y, 0.0);
        approx(struck.angular_velocity.norm(), 0.0);
        assert_eq!(struck.motion, MotionState::Sliding);
    }

    #[test]
    fn strike_phi_rotates_velocity() {
        let params = BallParams::default();
        let state = KinematicState::stationary(Vec2::new(0.5, 1.0), params.radius);
        let struck = strike(
            state,
            CueStrike::new(2.0, 90.0),
            params,
            CueSpecs::default(),
        );

        assert!(struck.velocity.x.abs() < 1e-12);
        assert!(struck.velocity.y > 0.0);
    }

    #[test]
    fn side_spin_produces_spin_and_squirt() {
        let params = BallParams::default();
        let state = KinematicState::stationary(Vec2::new(0.5, 1.0), params.radius);
        let struck = strike(
            state,
            CueStrike {
                speed: 2.0,
                phi: 0.0,
                theta: 0.0,
                a: 0.5,
                b: 0.0,
            },
            params,
            CueSpecs::default(),
        );

        assert!(struck.angular_velocity.z < 0.0);
        assert!(struck.velocity.y < 0.0);
    }

    #[test]
    fn motion_reaches_stationary() {
        let params = BallParams::default();
        let state = KinematicState::stationary(Vec2::new(0.5, 1.0), params.radius);
        let struck = strike(state, CueStrike::new(2.0, 0.0), params, CueSpecs::default());
        let final_state = evolve(struck, 30.0, params);

        assert_eq!(final_state.motion, MotionState::Stationary);
        assert!(final_state.velocity.norm() < 1e-12);
        assert!(final_state.angular_velocity.norm() < 1e-12);
        assert!(final_state.position.x > state.position.x);
    }

    #[test]
    fn mathavan_long_rail_response_matches_pooltool_044() {
        let params = BallParams::default();
        let state = KinematicState {
            position: Vec3::new(1.241_424_999_999_997_3, 0.5, params.radius),
            velocity: Vec3::new(1.058_343_212_670_157, -6.480_483_138_979_17e-17, 0.0),
            angular_velocity: Vec3::new(4.535_771_225_882_184_5e-15, 37.037_382_770_609_17, 0.0),
            motion: MotionState::Rolling,
        };
        let cushion = LinearCushion {
            id: "15",
            p1: Vec2::new(1.27, 0.1),
            p2: Vec2::new(1.27, 1.0),
            direction: CushionDirection::Side1,
            height: 0.036_576,
        };
        let actual = resolve_linear_cushion(state, cushion, params);

        approx(actual.velocity.x, -0.846_881_200_689_680_6);
        approx(actual.velocity.y, -0.000_218_174_225_720_572_38);
        approx(actual.angular_velocity.x, 0.000_817_928_560_368_007_1);
        approx(actual.angular_velocity.y, 8.237_659_430_045_886);
        approx(actual.angular_velocity.z, -0.014_929_340_496_412_431);
    }
}

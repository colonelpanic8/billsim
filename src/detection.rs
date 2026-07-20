//! Analytic event detection.

use roots::{find_roots_quadratic, find_roots_quartic};

use crate::math::Vec2;
use crate::model::{BallParams, MotionState};
use crate::physics::{KinematicState, relative_velocity, transition_time};
use crate::table::{CircularCushion, CushionDirection, LinearCushion, Pocket};

const MIN_EVENT_TIME: f64 = 1e-10;

/// Return the first future time at which two balls touch.
pub(crate) fn ball_ball_collision_time(
    first: KinematicState,
    second: KinematicState,
    params: BallParams,
) -> f64 {
    if matches!(first.motion, MotionState::Pocketed)
        || matches!(second.motion, MotionState::Pocketed)
        || (is_non_translating(first.motion) && is_non_translating(second.motion))
    {
        return f64::INFINITY;
    }

    let separation = second.position.xy() - first.position.xy();
    let diameter = 2.0 * params.radius;
    if separation.norm_squared() < diameter * diameter {
        return f64::INFINITY;
    }

    let (first_accel, first_velocity, first_position) = position_polynomial(first, params);
    let (second_accel, second_velocity, second_position) = position_polynomial(second, params);
    let accel = second_accel - first_accel;
    let velocity = second_velocity - first_velocity;
    let position = second_position - first_position;

    let coefficients = [
        accel.dot(accel),
        2.0 * accel.dot(velocity),
        velocity.dot(velocity) + 2.0 * accel.dot(position),
        2.0 * velocity.dot(position),
        position.dot(position) - diameter * diameter,
    ];
    let roots = find_roots_quartic(
        coefficients[0],
        coefficients[1],
        coefficients[2],
        coefficients[3],
        coefficients[4],
    );
    let valid_until = transition_time(first, params).min(transition_time(second, params));

    roots
        .as_ref()
        .iter()
        .copied()
        .filter(|time| time.is_finite() && *time > MIN_EVENT_TIME)
        .filter(|time| *time <= valid_until + MIN_EVENT_TIME)
        .filter(|time| {
            let delta = accel * (*time * *time) + velocity * *time + position;
            let delta_velocity = accel * (2.0 * *time) + velocity;
            // Keep influx contacts, not a later root where already-touching balls
            // separate again.
            delta.dot(delta_velocity) <= 1e-9
        })
        .fold(f64::INFINITY, f64::min)
}

fn is_non_translating(motion: MotionState) -> bool {
    matches!(
        motion,
        MotionState::Stationary | MotionState::Spinning | MotionState::Pocketed
    )
}

pub(crate) fn linear_cushion_collision_time(
    state: KinematicState,
    cushion: LinearCushion,
    params: BallParams,
) -> f64 {
    if is_non_translating(state.motion) {
        return f64::INFINITY;
    }
    let (accel, velocity, position) = position_polynomial(state, params);
    let delta = cushion.p2 - cushion.p1;
    let (lx, ly, l0) = if delta.x == 0.0 {
        (1.0, 0.0, -cushion.p1.x)
    } else {
        let slope = delta.y / delta.x;
        (-slope, 1.0, slope * cushion.p1.x - cushion.p1.y)
    };
    let normal_scale = (lx * lx + ly * ly).sqrt();
    let signed_radius = match cushion.direction {
        CushionDirection::Side1 => params.radius * normal_scale,
        CushionDirection::Side2 => -params.radius * normal_scale,
    };
    let roots = find_roots_quadratic(
        lx * accel.x + ly * accel.y,
        lx * velocity.x + ly * velocity.y,
        l0 + lx * position.x + ly * position.y + signed_radius,
    );
    let valid_until = transition_time(state, params);
    roots
        .as_ref()
        .iter()
        .copied()
        .filter(|time| time.is_finite() && *time > MIN_EVENT_TIME)
        .filter(|time| *time <= valid_until + MIN_EVENT_TIME)
        .filter(|time| {
            let at_contact = accel * (*time * *time) + velocity * *time + position;
            let score = -(cushion.p1 - at_contact).dot(delta) / delta.norm_squared();
            (0.0..=1.0).contains(&score)
        })
        .fold(f64::INFINITY, f64::min)
}

pub(crate) fn circular_cushion_collision_time(
    state: KinematicState,
    cushion: CircularCushion,
    params: BallParams,
) -> f64 {
    circle_collision_time(
        state,
        cushion.center,
        cushion.radius + params.radius,
        params,
    )
}

pub(crate) fn pocket_collision_time(
    state: KinematicState,
    pocket: Pocket,
    params: BallParams,
) -> f64 {
    circle_collision_time(state, pocket.center, pocket.radius, params)
}

fn circle_collision_time(
    state: KinematicState,
    center: Vec2,
    radius: f64,
    params: BallParams,
) -> f64 {
    if is_non_translating(state.motion) {
        return f64::INFINITY;
    }
    let (accel, velocity, position) = position_polynomial(state, params);
    let offset = position - center;
    let coefficients = [
        accel.dot(accel),
        2.0 * accel.dot(velocity),
        velocity.dot(velocity) + 2.0 * accel.dot(offset),
        2.0 * velocity.dot(offset),
        offset.dot(offset) - radius * radius,
    ];
    let roots = find_roots_quartic(
        coefficients[0],
        coefficients[1],
        coefficients[2],
        coefficients[3],
        coefficients[4],
    );
    let valid_until = transition_time(state, params);
    roots
        .as_ref()
        .iter()
        .copied()
        .filter(|time| time.is_finite() && *time > MIN_EVENT_TIME)
        .filter(|time| *time <= valid_until + MIN_EVENT_TIME)
        .filter(|time| {
            let delta = accel * (*time * *time) + velocity * *time + offset;
            let delta_velocity = accel * (2.0 * *time) + velocity;
            delta.dot(delta_velocity) <= 1e-9
        })
        .fold(f64::INFINITY, f64::min)
}

/// Position is `acceleration * t² + velocity * t + position` until the
/// ball's next motion transition.
fn position_polynomial(state: KinematicState, params: BallParams) -> (Vec2, Vec2, Vec2) {
    let acceleration = match state.motion {
        MotionState::Sliding => {
            let direction = relative_velocity(state, params.radius)
                .xy()
                .normalized()
                .unwrap_or(Vec2::new(1.0, 0.0));
            direction * (-0.5 * params.sliding_friction * params.gravity)
        }
        MotionState::Rolling => {
            state
                .velocity
                .xy()
                .normalized()
                .unwrap_or(Vec2::new(1.0, 0.0))
                * (-0.5 * params.rolling_friction * params.gravity)
        }
        MotionState::Stationary | MotionState::Spinning | MotionState::Pocketed => {
            Vec2::new(0.0, 0.0)
        }
    };
    (acceleration, state.velocity.xy(), state.position.xy())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    fn moving(position: Vec2, velocity: Vec2) -> KinematicState {
        let params = BallParams::default();
        KinematicState {
            position: Vec3::new(position.x, position.y, params.radius),
            velocity: Vec3::new(velocity.x, velocity.y, 0.0),
            angular_velocity: Vec3::new(
                -velocity.y / params.radius,
                velocity.x / params.radius,
                0.0,
            ),
            motion: MotionState::Rolling,
        }
    }

    #[test]
    fn rolling_ball_hits_stationary_ball() {
        let params = BallParams::default();
        let first = moving(Vec2::new(0.2, 0.5), Vec2::new(1.0, 0.0));
        let second = KinematicState::stationary(Vec2::new(0.8, 0.5), params.radius);
        let time = ball_ball_collision_time(first, second, params);

        assert!(time.is_finite());
        assert!(time > 0.5 && time < 0.6);
    }

    #[test]
    fn ball_moving_away_has_no_collision() {
        let params = BallParams::default();
        let first = moving(Vec2::new(0.2, 0.5), Vec2::new(-1.0, 0.0));
        let second = KinematicState::stationary(Vec2::new(0.8, 0.5), params.radius);

        assert!(ball_ball_collision_time(first, second, params).is_infinite());
    }
}

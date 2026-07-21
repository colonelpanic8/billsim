//! Event-based simulation orchestration.

use std::error::Error;
use std::fmt::{Display, Formatter};

use crate::detection::{
    ball_ball_collision_time, circular_cushion_collision_time, linear_cushion_collision_time,
    pocket_collision_time,
};
use crate::math::Vec3;
use crate::model::{
    BallId, BallState, BallTrajectory, BallVelocity, MotionState, ShotProjection, SimulationEvent,
    SimulationEventType, SimulationOptions, SimulationScenario, TrajectoryPoint,
};
use crate::physics::{
    KinematicState, evolve_until_event, resolve_ball_ball, resolve_circular_cushion,
    resolve_linear_cushion, resolve_transition, strike, transition_time,
};
use crate::table::TableGeometry;

#[derive(Clone, Debug, PartialEq)]
pub enum SimulationError {
    NoBalls,
    DuplicateBallId(BallId),
    CueBallNotFound(BallId),
    InvalidValue(&'static str),
    BallOutsideTable(BallId),
    OverlappingBalls(BallId, BallId),
    Unsupported(&'static str),
    MaxTimeExceeded,
    MaxEventsExceeded,
}

impl Display for SimulationError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoBalls => formatter.write_str("the scenario contains no balls"),
            Self::DuplicateBallId(id) => write!(formatter, "duplicate ball id {id}"),
            Self::CueBallNotFound(id) => write!(formatter, "cue ball id {id} was not found"),
            Self::InvalidValue(name) => write!(formatter, "invalid value for {name}"),
            Self::BallOutsideTable(id) => write!(formatter, "ball {id} is outside the table"),
            Self::OverlappingBalls(first, second) => {
                write!(formatter, "balls {first} and {second} overlap")
            }
            Self::Unsupported(feature) => {
                write!(formatter, "unsupported physics feature: {feature}")
            }
            Self::MaxTimeExceeded => formatter.write_str("simulation exceeded max_time"),
            Self::MaxEventsExceeded => formatter.write_str("simulation exceeded max_events"),
        }
    }
}

impl Error for SimulationError {}

#[derive(Clone, Copy, Debug)]
struct SimBall {
    id: BallId,
    state: KinematicState,
}

#[derive(Clone, Debug)]
struct Snapshot {
    time: f64,
    states: Vec<KinematicState>,
}

#[derive(Clone, Copy, Debug)]
enum PendingEvent {
    Transition { ball: usize },
    BallBall { first: usize, second: usize },
    CircularCushion { ball: usize, cushion: usize },
    LinearCushion { ball: usize, cushion: usize },
    Pocket { ball: usize, pocket: usize },
}

/// Simulate a scenario and return trajectories, events, and final state.
///
/// # Errors
///
/// Returns [`SimulationError`] when the scenario or options are invalid, a
/// simulation safety limit is exceeded, or the scenario requires a physics
/// feature that has not reached parity yet.
#[allow(clippy::too_many_lines)]
pub fn simulate(
    scenario: &SimulationScenario,
    options: SimulationOptions,
) -> Result<ShotProjection, SimulationError> {
    validate(scenario, options)?;
    let params = scenario.table.ball;
    let geometry = TableGeometry::new(scenario.table);
    let mut balls: Vec<_> = scenario
        .balls
        .iter()
        .map(|ball| SimBall {
            id: ball.id,
            state: KinematicState::stationary(ball.position, params.radius),
        })
        .collect();
    let cue_index = balls
        .iter()
        .position(|ball| ball.id == scenario.cue_ball_id)
        .ok_or(SimulationError::CueBallNotFound(scenario.cue_ball_id))?;
    balls[cue_index].state = strike(
        balls[cue_index].state,
        scenario.strike,
        params,
        scenario.table.cue,
    );

    let mut elapsed = 0.0;
    let mut events = vec![SimulationEvent {
        event_type: SimulationEventType::StickBall,
        time: 0.0,
        ball_ids: vec![scenario.cue_ball_id],
        position: Some(balls[cue_index].state.position.xy()),
        pocket: None,
        cushion: None,
        velocities_after: vec![velocity_after(&balls[cue_index])],
    }];
    let mut snapshots = vec![snapshot(elapsed, &balls)];
    let mut potted_ball_ids = Vec::new();

    while let Some((duration, pending)) = next_event(&balls, &geometry, params) {
        if events.len() >= options.max_events {
            return Err(SimulationError::MaxEventsExceeded);
        }
        if elapsed + duration > options.max_time {
            return Err(SimulationError::MaxTimeExceeded);
        }

        for ball in &mut balls {
            ball.state = evolve_until_event(ball.state, duration, params);
        }
        elapsed += duration;

        let event = match pending {
            PendingEvent::Transition { ball } => {
                let prior_motion = balls[ball].state.motion;
                balls[ball].state = resolve_transition(balls[ball].state);
                SimulationEvent {
                    event_type: transition_event_type(prior_motion, balls[ball].state.motion),
                    time: elapsed,
                    ball_ids: vec![balls[ball].id],
                    position: Some(balls[ball].state.position.xy()),
                    pocket: None,
                    cushion: None,
                    velocities_after: vec![velocity_after(&balls[ball])],
                }
            }
            PendingEvent::BallBall { first, second } => {
                let (first_state, second_state) =
                    resolve_ball_ball(balls[first].state, balls[second].state, params);
                balls[first].state = first_state;
                balls[second].state = second_state;
                SimulationEvent {
                    event_type: SimulationEventType::BallBall,
                    time: elapsed,
                    ball_ids: vec![balls[first].id, balls[second].id],
                    position: Some(balls[first].state.position.xy()),
                    pocket: None,
                    cushion: None,
                    velocities_after: vec![
                        velocity_after(&balls[first]),
                        velocity_after(&balls[second]),
                    ],
                }
            }
            PendingEvent::CircularCushion { ball, cushion } => {
                let component = geometry.circular[cushion];
                balls[ball].state = resolve_circular_cushion(balls[ball].state, component, params);
                SimulationEvent {
                    event_type: SimulationEventType::BallCushion,
                    time: elapsed,
                    ball_ids: vec![balls[ball].id],
                    position: Some(balls[ball].state.position.xy()),
                    pocket: None,
                    cushion: Some(component.id.to_owned()),
                    velocities_after: vec![velocity_after(&balls[ball])],
                }
            }
            PendingEvent::LinearCushion { ball, cushion } => {
                let component = geometry.linear[cushion];
                balls[ball].state = resolve_linear_cushion(balls[ball].state, component, params);
                SimulationEvent {
                    event_type: SimulationEventType::BallCushion,
                    time: elapsed,
                    ball_ids: vec![balls[ball].id],
                    position: Some(balls[ball].state.position.xy()),
                    pocket: None,
                    cushion: Some(component.id.to_owned()),
                    velocities_after: vec![velocity_after(&balls[ball])],
                }
            }
            PendingEvent::Pocket { ball, pocket } => {
                let component = geometry.pockets[pocket];
                balls[ball].state.position =
                    Vec3::new(component.center.x, component.center.y, -component.depth);
                balls[ball].state.velocity = Vec3::ZERO;
                balls[ball].state.angular_velocity = Vec3::ZERO;
                balls[ball].state.motion = MotionState::Pocketed;
                potted_ball_ids.push(balls[ball].id);
                SimulationEvent {
                    event_type: SimulationEventType::BallPocket,
                    time: elapsed,
                    ball_ids: vec![balls[ball].id],
                    position: Some(component.center),
                    pocket: Some(component.id),
                    cushion: None,
                    velocities_after: vec![velocity_after(&balls[ball])],
                }
            }
        };
        events.push(event);
        snapshots.push(snapshot(elapsed, &balls));
    }

    let trajectories = balls
        .iter()
        .enumerate()
        .map(|(index, ball)| {
            sample_trajectory(
                ball.id,
                index,
                &snapshots,
                elapsed,
                options.trajectory_dt,
                params,
            )
        })
        .collect();
    let final_state = balls
        .iter()
        .map(|ball| BallState {
            ball_id: ball.id,
            position: ball.state.position.xy(),
            motion_state: ball.state.motion,
        })
        .collect();

    Ok(ShotProjection {
        trajectories,
        events,
        final_state,
        potted_ball_ids,
    })
}

fn next_event(
    balls: &[SimBall],
    geometry: &TableGeometry,
    params: crate::model::BallParams,
) -> Option<(f64, PendingEvent)> {
    let mut best_time = f64::INFINITY;
    let mut best_event = None;

    // Match Pooltool 0.4 ordering: transitions win exact ties.
    for (index, ball) in balls.iter().enumerate() {
        let time = transition_time(ball.state, params);
        if time < best_time {
            best_time = time;
            best_event = Some(PendingEvent::Transition { ball: index });
        }
    }
    for first in 0..balls.len() {
        for second in first + 1..balls.len() {
            let time = ball_ball_collision_time(balls[first].state, balls[second].state, params);
            if time < best_time {
                best_time = time;
                best_event = Some(PendingEvent::BallBall { first, second });
            }
        }
    }
    for (ball_index, ball) in balls.iter().enumerate() {
        for (cushion_index, cushion) in geometry.circular.iter().copied().enumerate() {
            let time = circular_cushion_collision_time(ball.state, cushion, params);
            if time < best_time {
                best_time = time;
                best_event = Some(PendingEvent::CircularCushion {
                    ball: ball_index,
                    cushion: cushion_index,
                });
            }
        }
    }
    for (ball_index, ball) in balls.iter().enumerate() {
        for (cushion_index, cushion) in geometry.linear.iter().copied().enumerate() {
            let time = linear_cushion_collision_time(ball.state, cushion, params);
            if time < best_time {
                best_time = time;
                best_event = Some(PendingEvent::LinearCushion {
                    ball: ball_index,
                    cushion: cushion_index,
                });
            }
        }
    }
    for (ball_index, ball) in balls.iter().enumerate() {
        for (pocket_index, pocket) in geometry.pockets.iter().copied().enumerate() {
            let time = pocket_collision_time(ball.state, pocket, params);
            if time < best_time {
                best_time = time;
                best_event = Some(PendingEvent::Pocket {
                    ball: ball_index,
                    pocket: pocket_index,
                });
            }
        }
    }

    best_time.is_finite().then_some((best_time, best_event?))
}

fn transition_event_type(from: MotionState, to: MotionState) -> SimulationEventType {
    match (from, to) {
        (MotionState::Sliding, MotionState::Rolling) => SimulationEventType::SlidingRolling,
        (MotionState::Rolling, MotionState::Spinning) => SimulationEventType::RollingSpinning,
        (MotionState::Rolling, MotionState::Stationary) => SimulationEventType::RollingStationary,
        (MotionState::Spinning, MotionState::Stationary) => SimulationEventType::SpinningStationary,
        _ => unreachable!(),
    }
}

fn snapshot(time: f64, balls: &[SimBall]) -> Snapshot {
    Snapshot {
        time,
        states: balls.iter().map(|ball| ball.state).collect(),
    }
}

fn velocity_after(ball: &SimBall) -> BallVelocity {
    BallVelocity {
        ball_id: ball.id,
        velocity: ball.state.velocity.xy(),
    }
}

fn sample_trajectory(
    ball_id: BallId,
    ball_index: usize,
    snapshots: &[Snapshot],
    final_time: f64,
    dt: f64,
    params: crate::model::BallParams,
) -> BallTrajectory {
    let mut points = Vec::new();
    let mut time = 0.0;
    let mut snapshot_index = 0;
    while time < final_time {
        while snapshot_index + 1 < snapshots.len() && snapshots[snapshot_index + 1].time <= time {
            snapshot_index += 1;
        }
        let base = &snapshots[snapshot_index];
        let state = evolve_until_event(base.states[ball_index], time - base.time, params);
        points.push(TrajectoryPoint {
            time,
            position: state.position.xy(),
            motion_state: state.motion,
        });
        time += dt;
    }

    let final_snapshot = snapshots.last().expect("simulation always has a snapshot");
    let state = final_snapshot.states[ball_index];
    points.push(TrajectoryPoint {
        time: final_time,
        position: state.position.xy(),
        motion_state: state.motion,
    });
    BallTrajectory { ball_id, points }
}

fn validate(
    scenario: &SimulationScenario,
    options: SimulationOptions,
) -> Result<(), SimulationError> {
    if scenario.balls.is_empty() {
        return Err(SimulationError::NoBalls);
    }
    if !scenario.table.length.is_finite() || scenario.table.length <= 0.0 {
        return Err(SimulationError::InvalidValue("table.length"));
    }
    if !scenario.table.width.is_finite() || scenario.table.width <= 0.0 {
        return Err(SimulationError::InvalidValue("table.width"));
    }
    if !scenario.table.ball.radius.is_finite() || scenario.table.ball.radius <= 0.0 {
        return Err(SimulationError::InvalidValue("ball.radius"));
    }
    if !scenario.strike.speed.is_finite() || scenario.strike.speed <= 0.0 {
        return Err(SimulationError::InvalidValue("strike.speed"));
    }
    if !scenario.strike.phi.is_finite()
        || !scenario.strike.theta.is_finite()
        || !scenario.strike.a.is_finite()
        || !scenario.strike.b.is_finite()
    {
        return Err(SimulationError::InvalidValue("strike"));
    }
    if scenario.strike.a * scenario.strike.a + scenario.strike.b * scenario.strike.b > 1.0 {
        return Err(SimulationError::InvalidValue("strike contact offset"));
    }
    if !options.trajectory_dt.is_finite() || options.trajectory_dt <= 0.0 {
        return Err(SimulationError::InvalidValue("trajectory_dt"));
    }
    if !options.max_time.is_finite() || options.max_time <= 0.0 {
        return Err(SimulationError::InvalidValue("max_time"));
    }
    if options.max_events == 0 {
        return Err(SimulationError::InvalidValue("max_events"));
    }

    let mut ids = std::collections::HashSet::with_capacity(scenario.balls.len());
    for ball in &scenario.balls {
        if !ids.insert(ball.id) {
            return Err(SimulationError::DuplicateBallId(ball.id));
        }
        if !ball.position.x.is_finite() || !ball.position.y.is_finite() {
            return Err(SimulationError::InvalidValue("ball.position"));
        }
        let radius = scenario.table.ball.radius;
        if ball.position.x < radius
            || ball.position.x > scenario.table.width - radius
            || ball.position.y < radius
            || ball.position.y > scenario.table.length - radius
        {
            return Err(SimulationError::BallOutsideTable(ball.id));
        }
    }
    if !ids.contains(&scenario.cue_ball_id) {
        return Err(SimulationError::CueBallNotFound(scenario.cue_ball_id));
    }

    let diameter_squared = (2.0 * scenario.table.ball.radius).powi(2);
    for first in 0..scenario.balls.len() {
        for second in first + 1..scenario.balls.len() {
            let delta = scenario.balls[second].position - scenario.balls[first].position;
            if delta.norm_squared() < diameter_squared {
                return Err(SimulationError::OverlappingBalls(
                    scenario.balls[first].id,
                    scenario.balls[second].id,
                ));
            }
        }
    }
    Ok(())
}

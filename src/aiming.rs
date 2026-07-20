//! Jaw-aware geometric and simulation-compensated pot aiming.

use std::error::Error;
use std::fmt::{Display, Formatter};

use crate::math::Vec2;
use crate::model::{BallId, PocketId, SimulationEventType, SimulationOptions, SimulationScenario};
use crate::simulation::{SimulationError, simulate};
use crate::table::{LinearCushion, Pocket, TableGeometry};
use serde::{Deserialize, Serialize};

const MISS_TOLERANCE: f64 = 0.001;
const PHI_TOLERANCE: f64 = 0.02;
const INITIAL_SECANT_STEP: f64 = 0.5;
const MAX_SECANT_STEP: f64 = 6.0;
const MAX_SECANT_ITERATIONS: usize = 12;
const INVALID_STEP_RETRIES: usize = 4;
const CONTACT_SCAN_OFFSETS: [f64; 7] = [1.0, 2.0, 4.0, 8.0, 12.0, 18.0, 24.0];
const MAX_CUT_ANGLE: f64 = 88.0;

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PotAim {
    pub phi: f64,
    pub geometric_phi: f64,
    pub cut_angle: f64,
    pub required_precision: f64,
    pub feasible: bool,
    pub potted: bool,
    pub converged: bool,
    pub occluding_ball_ids: Vec<BallId>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum AimError {
    CueBallIsTarget,
    TargetBallNotFound(BallId),
    InvalidGeometry(&'static str),
    Simulation(SimulationError),
}

impl Display for AimError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CueBallIsTarget => formatter.write_str("target ball must differ from cue ball"),
            Self::TargetBallNotFound(id) => write!(formatter, "target ball {id} was not found"),
            Self::InvalidGeometry(reason) => write!(formatter, "invalid pot geometry: {reason}"),
            Self::Simulation(error) => write!(formatter, "pot probe failed: {error}"),
        }
    }
}

impl Error for AimError {}

impl From<SimulationError> for AimError {
    fn from(value: SimulationError) -> Self {
        Self::Simulation(value)
    }
}

#[derive(Clone, Copy, Debug)]
struct Probe {
    phi: f64,
    signed_miss: Option<f64>,
    potted: bool,
}

#[derive(Clone, Copy)]
struct Jaw {
    left_edge: &'static str,
    left_rail: &'static str,
    right_edge: &'static str,
    right_rail: &'static str,
    corner: bool,
}

/// Geometry-only pot viability: occlusion and cut-angle, no simulations.
///
/// Callers that would otherwise run a simulation search (for example the
/// position sweep) should consult this first — solving aim for an
/// impossible pot degenerates into the solver's worst case on every probe.
#[derive(Clone, Debug, PartialEq)]
pub struct PotFeasibility {
    pub feasible: bool,
    pub occluding_ball_ids: Vec<BallId>,
    pub cut_angle: f64,
}

/// Report whether a direct pot is geometrically on, using no simulations.
///
/// # Errors
///
/// Returns [`AimError`] for an invalid target or degenerate geometry.
pub fn geometric_pot_feasibility(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
) -> Result<PotFeasibility, AimError> {
    let (_, _, setup) = pot_setup(scenario, target_ball_id, pocket_id)?;
    Ok(PotFeasibility {
        feasible: setup.feasible,
        occluding_ball_ids: setup.occluding_ball_ids,
        cut_angle: setup.cut_angle,
    })
}

/// Compute a Pooltool-compatible pot aim in physics-frame degrees.
///
/// The scenario's `strike.phi` is ignored. Speed, elevation, side spin, and
/// follow/draw are retained during the simulation refinement.
///
/// # Errors
///
/// Returns [`AimError`] for an invalid target, degenerate geometry, or a
/// failed simulation probe.
pub fn compute_pot_aim(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
) -> Result<PotAim, AimError> {
    compute_pot_aim_seeded(scenario, target_ball_id, pocket_id, None)
}

struct PotSetup {
    pocket: Pocket,
    potting_point: Vec2,
    geometric_phi: f64,
    cut_angle: f64,
    required_precision: f64,
    occluding_ball_ids: Vec<BallId>,
    feasible: bool,
}

fn pot_setup(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
) -> Result<(Vec2, Vec2, PotSetup), AimError> {
    if target_ball_id == scenario.cue_ball_id {
        return Err(AimError::CueBallIsTarget);
    }
    let cue = scenario
        .balls
        .iter()
        .find(|ball| ball.id == scenario.cue_ball_id)
        .ok_or(AimError::Simulation(SimulationError::CueBallNotFound(
            scenario.cue_ball_id,
        )))?;
    let target = scenario
        .balls
        .iter()
        .find(|ball| ball.id == target_ball_id)
        .ok_or(AimError::TargetBallNotFound(target_ball_id))?;
    let geometry = TableGeometry::new(scenario.table);
    let pocket = *geometry
        .pockets
        .iter()
        .find(|pocket| pocket.id == pocket_id)
        .ok_or(AimError::InvalidGeometry("missing pocket"))?;
    let potting_point = potting_point(
        target.position,
        &geometry,
        pocket,
        scenario.table.ball.radius,
    )?;
    let shadow = shadow_ball_center(target.position, potting_point, scenario.table.ball.radius)?;
    let geometric_phi = direction_degrees(shadow - cue.position);
    let cut_angle = angle_between(shadow - cue.position, potting_point - target.position);
    let required_precision = required_precision(cue.position, target.position, &geometry, pocket)?;
    let occluding_ball_ids = occluding_ball_ids(
        scenario,
        target_ball_id,
        cue.position,
        target.position,
        shadow,
        potting_point,
    );
    let feasible = occluding_ball_ids.is_empty() && cut_angle.abs() <= MAX_CUT_ANGLE;
    Ok((
        cue.position,
        target.position,
        PotSetup {
            pocket,
            potting_point,
            geometric_phi,
            cut_angle,
            required_precision,
            occluding_ball_ids,
            feasible,
        },
    ))
}

/// [`compute_pot_aim`] with an optional warm-start seed for the refinement.
///
/// `seed_phi` (physics-frame degrees) replaces the geometric seed as the
/// starting point of the secant search — useful when solving a family of
/// similar strikes, where the previous solution is a near-answer. The
/// geometric aim is still computed and reported unchanged.
///
/// # Errors
///
/// Returns [`AimError`] for an invalid target, degenerate geometry, or a
/// failed simulation probe.
pub fn compute_pot_aim_seeded(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
    seed_phi: Option<f64>,
) -> Result<PotAim, AimError> {
    let (_, _, setup) = pot_setup(scenario, target_ball_id, pocket_id)?;
    let (solution, converged) = solve(
        scenario,
        target_ball_id,
        setup.pocket,
        setup.potting_point,
        seed_phi.unwrap_or(setup.geometric_phi),
    )?;

    Ok(PotAim {
        phi: solution.phi.rem_euclid(360.0),
        geometric_phi: setup.geometric_phi.rem_euclid(360.0),
        cut_angle: setup.cut_angle,
        required_precision: setup.required_precision,
        feasible: setup.feasible,
        potted: solution.potted,
        converged,
        occluding_ball_ids: setup.occluding_ball_ids,
    })
}

fn solve(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket: Pocket,
    potting_point: Vec2,
    seed_phi: f64,
) -> Result<(Probe, bool), AimError> {
    let evaluate = |phi| probe(scenario, target_ball_id, pocket, potting_point, phi);
    let mut previous = evaluate(seed_phi)?;
    if previous.signed_miss.is_none() {
        'scan: for offset in CONTACT_SCAN_OFFSETS {
            for direction in [1.0, -1.0] {
                let candidate = evaluate(seed_phi + direction * offset)?;
                if candidate.signed_miss.is_some() {
                    previous = candidate;
                    break 'scan;
                }
            }
        }
        if previous.signed_miss.is_none() {
            return Ok((previous, false));
        }
    }

    let mut best = previous;
    let mut current = evaluate(previous.phi + INITIAL_SECANT_STEP)?;
    if current.signed_miss.is_none() {
        current = evaluate(previous.phi - INITIAL_SECANT_STEP)?;
    }
    if current.signed_miss.is_none() {
        return Ok((best, miss(best).abs() <= MISS_TOLERANCE));
    }
    if miss(current).abs() < miss(best).abs() {
        best = current;
    }

    for _ in 0..MAX_SECANT_ITERATIONS {
        if miss(best).abs() <= MISS_TOLERANCE {
            break;
        }
        let denominator = miss(current) - miss(previous);
        if denominator == 0.0 {
            break;
        }
        let mut step = -miss(current) * (current.phi - previous.phi) / denominator;
        step = step.clamp(-MAX_SECANT_STEP, MAX_SECANT_STEP);
        if step.abs() < PHI_TOLERANCE {
            break;
        }

        let mut next = None;
        for _ in 0..INVALID_STEP_RETRIES {
            let candidate = evaluate(current.phi + step)?;
            if candidate.signed_miss.is_some() {
                next = Some(candidate);
                break;
            }
            step /= 2.0;
        }
        let Some(next) = next else { break };
        previous = current;
        current = next;
        if miss(current).abs() < miss(best).abs() {
            best = current;
        }
    }
    Ok((best, miss(best).abs() <= MISS_TOLERANCE))
}

fn miss(probe: Probe) -> f64 {
    probe.signed_miss.expect("only valid probes reach miss()")
}

fn probe(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket: Pocket,
    potting_point: Vec2,
    phi: f64,
) -> Result<Probe, AimError> {
    let mut candidate = scenario.clone();
    candidate.strike.phi = phi.rem_euclid(360.0);
    let projection = simulate(&candidate, SimulationOptions::default())?;
    let sent_velocity =
        direct_sent_velocity(&projection.events, scenario.cue_ball_id, target_ball_id);
    let Some(sent_velocity) = sent_velocity else {
        return Ok(Probe {
            phi,
            signed_miss: None,
            potted: false,
        });
    };
    let target_start = scenario
        .balls
        .iter()
        .find(|ball| ball.id == target_ball_id)
        .expect("target validated before probing")
        .position;
    let to_pocket = potting_point - target_start;
    let angular_error = (sent_velocity.y.atan2(sent_velocity.x) - to_pocket.y.atan2(to_pocket.x))
        .rem_euclid(2.0 * std::f64::consts::PI);
    let angular_error = if angular_error > std::f64::consts::PI {
        angular_error - 2.0 * std::f64::consts::PI
    } else {
        angular_error
    };
    let potted = projection.events.iter().any(|event| {
        event.event_type == SimulationEventType::BallPocket
            && event.ball_ids.contains(&target_ball_id)
            && event.pocket == Some(pocket.id)
    });
    Ok(Probe {
        phi,
        signed_miss: Some(angular_error * to_pocket.norm()),
        potted,
    })
}

fn direct_sent_velocity(
    events: &[crate::model::SimulationEvent],
    cue_ball_id: BallId,
    target_ball_id: BallId,
) -> Option<Vec2> {
    for event in events {
        match event.event_type {
            SimulationEventType::BallBall if event.ball_ids.contains(&cue_ball_id) => {
                if event.ball_ids.len() == 2 && event.ball_ids.contains(&target_ball_id) {
                    return event
                        .velocities_after
                        .iter()
                        .find(|velocity| velocity.ball_id == target_ball_id)
                        .map(|velocity| velocity.velocity)
                        .filter(|velocity| velocity.norm() > 1e-9);
                }
                return None;
            }
            SimulationEventType::BallCushion | SimulationEventType::BallPocket
                if event.ball_ids.contains(&cue_ball_id) =>
            {
                return None;
            }
            _ => {}
        }
    }
    None
}

fn occluding_ball_ids(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    cue: Vec2,
    target: Vec2,
    shadow: Vec2,
    potting_point: Vec2,
) -> Vec<BallId> {
    let mut ids = Vec::new();
    for ball in &scenario.balls {
        if ball.id == scenario.cue_ball_id || ball.id == target_ball_id {
            continue;
        }
        if occludes(cue, shadow, ball.position, scenario.table.ball.radius)
            || occludes(
                target,
                potting_point,
                ball.position,
                scenario.table.ball.radius,
            )
        {
            ids.push(ball.id);
        }
    }
    ids.sort_unstable();
    ids.dedup();
    ids
}

fn occludes(start: Vec2, end: Vec2, ball: Vec2, radius: f64) -> bool {
    let path = end - start;
    if path.norm_squared() <= f64::EPSILON {
        return false;
    }
    let score = (ball - start).dot(path) / path.norm_squared();
    if !(0.0..=1.0).contains(&score) {
        return false;
    }
    (start + path * score - ball).norm() < 2.0 * radius
}

fn shadow_ball_center(target: Vec2, potting_point: Vec2, radius: f64) -> Result<Vec2, AimError> {
    let direction = (potting_point - target)
        .normalized()
        .ok_or(AimError::InvalidGeometry("target is at the potting point"))?;
    Ok(target - direction * (2.0 * radius))
}

fn potting_point(
    ball: Vec2,
    geometry: &TableGeometry,
    pocket: Pocket,
    radius: f64,
) -> Result<Vec2, AimError> {
    let jaw = jaw(pocket.id);
    if !jaw.corner {
        let left = linear(geometry, jaw.left_rail)?;
        let right = linear(geometry, jaw.right_rail)?;
        let mut best = (f64::INFINITY, Vec2::new(0.0, 0.0));
        for left_point in [left.p1, left.p2] {
            for right_point in [right.p1, right.p2] {
                let distance = (left_point - right_point).norm();
                if distance < best.0 {
                    best = (distance, left_point + (right_point - left_point) / 2.0);
                }
            }
        }
        return Ok(best.1);
    }

    let left_rail = linear(geometry, jaw.left_rail)?;
    let right_rail = linear(geometry, jaw.right_rail)?;
    let left_edge = linear(geometry, jaw.left_edge)?;
    let right_edge = linear(geometry, jaw.right_edge)?;
    let left_mouth = line_intersection(left_rail, left_edge)?;
    let right_mouth = line_intersection(right_rail, right_edge)?;
    if same_side(left_mouth, right_mouth, ball, pocket.center) {
        return Ok(pocket.center);
    }

    let intersection = line_intersection(left_rail, right_rail)?;
    let ball_to_intersection = intersection - ball;
    let mut left_unit = (left_rail.p2 - left_rail.p1)
        .normalized()
        .ok_or(AimError::InvalidGeometry("zero-length left rail"))?;
    let mut right_unit = (right_rail.p2 - right_rail.p1)
        .normalized()
        .ok_or(AimError::InvalidGeometry("zero-length right rail"))?;
    if ball_to_intersection.dot(left_unit) < 0.0 {
        left_unit = -left_unit;
    }
    if ball_to_intersection.dot(right_unit) < 0.0 {
        right_unit = -right_unit;
    }
    let left_angle = angle_between(ball_to_intersection, left_unit).abs();
    let right_angle = angle_between(ball_to_intersection, right_unit).abs();
    let (theta, offset_direction) = if left_angle < right_angle {
        (45.0 - left_angle, -right_unit)
    } else {
        (45.0 - right_angle, -left_unit)
    };
    Ok(intersection + offset_direction * ((std::f64::consts::PI / 90.0 * theta).sin() * radius))
}

fn required_precision(
    cue: Vec2,
    ball: Vec2,
    geometry: &TableGeometry,
    pocket: Pocket,
) -> Result<f64, AimError> {
    let jaw = jaw(pocket.id);
    let left_tip = line_intersection(
        linear(geometry, jaw.left_rail)?,
        linear(geometry, jaw.left_edge)?,
    )?;
    let right_tip = line_intersection(
        linear(geometry, jaw.right_rail)?,
        linear(geometry, jaw.right_edge)?,
    )?;
    let left = angle_between(ball - cue, left_tip - ball).abs();
    let right = angle_between(ball - cue, right_tip - ball).abs();
    Ok((left - right).abs())
}

fn linear(geometry: &TableGeometry, id: &'static str) -> Result<LinearCushion, AimError> {
    geometry
        .linear
        .iter()
        .copied()
        .find(|cushion| cushion.id == id)
        .ok_or(AimError::InvalidGeometry("missing cushion segment"))
}

fn line_intersection(first: LinearCushion, second: LinearCushion) -> Result<Vec2, AimError> {
    let first_delta = first.p2 - first.p1;
    let second_delta = second.p2 - second.p1;
    let determinant = first_delta.x * second_delta.y - first_delta.y * second_delta.x;
    if determinant.abs() <= f64::EPSILON {
        return Err(AimError::InvalidGeometry("parallel cushion lines"));
    }
    let offset = second.p1 - first.p1;
    let factor = (offset.x * second_delta.y - offset.y * second_delta.x) / determinant;
    Ok(first.p1 + first_delta * factor)
}

fn same_side(line_start: Vec2, line_end: Vec2, first: Vec2, second: Vec2) -> bool {
    let cross = |point: Vec2| {
        (line_end.x - line_start.x) * (point.y - line_start.y)
            - (line_end.y - line_start.y) * (point.x - line_start.x)
    };
    cross(first) * cross(second) >= 0.0
}

fn angle_between(first: Vec2, second: Vec2) -> f64 {
    let determinant = first.x * second.y - first.y * second.x;
    determinant.atan2(first.dot(second)).to_degrees()
}

fn direction_degrees(direction: Vec2) -> f64 {
    direction.y.atan2(direction.x).to_degrees()
}

const fn jaw(pocket: PocketId) -> Jaw {
    match pocket {
        PocketId::LeftBottom => Jaw {
            left_edge: "1",
            left_rail: "18",
            right_edge: "2",
            right_rail: "3",
            corner: true,
        },
        PocketId::LeftCenter => Jaw {
            left_edge: "4",
            left_rail: "3",
            right_edge: "5",
            right_rail: "6",
            corner: false,
        },
        PocketId::LeftTop => Jaw {
            left_edge: "7",
            left_rail: "6",
            right_edge: "8",
            right_rail: "9",
            corner: true,
        },
        PocketId::RightBottom => Jaw {
            left_edge: "16",
            left_rail: "15",
            right_edge: "17",
            right_rail: "18",
            corner: true,
        },
        PocketId::RightCenter => Jaw {
            left_edge: "13",
            left_rail: "12",
            right_edge: "14",
            right_rail: "15",
            corner: false,
        },
        PocketId::RightTop => Jaw {
            left_edge: "10",
            left_rail: "9",
            right_edge: "11",
            right_rail: "12",
            corner: true,
        },
    }
}

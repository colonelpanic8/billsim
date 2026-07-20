//! Public input and output types.

use crate::math::Vec2;
use serde::{Deserialize, Serialize};

pub type BallId = u32;

/// Pool ball and cloth parameters used by the motion and collision models.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct BallParams {
    pub mass: f64,
    pub radius: f64,
    pub sliding_friction: f64,
    pub rolling_friction: f64,
    /// Spinning friction with radius factored out.
    pub spinning_friction_factor: f64,
    pub ball_ball_friction: f64,
    pub ball_restitution: f64,
    pub cushion_restitution: f64,
    pub cushion_friction: f64,
    pub gravity: f64,
}

impl BallParams {
    #[must_use]
    pub fn spinning_friction(self) -> f64 {
        self.spinning_friction_factor * self.radius
    }
}

impl Default for BallParams {
    fn default() -> Self {
        Self {
            mass: 0.170_097,
            radius: 0.028_575,
            sliding_friction: 0.2,
            rolling_friction: 0.01,
            spinning_friction_factor: 10.0 * 2.0 / 5.0 / 9.0,
            ball_ball_friction: 0.05,
            ball_restitution: 0.95,
            cushion_restitution: 0.85,
            cushion_friction: 0.2,
            gravity: 9.81,
        }
    }
}

/// Cue properties used by the instantaneous-point strike model.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct CueSpecs {
    pub mass: f64,
    pub end_mass: f64,
    pub tip_radius: f64,
}

impl Default for CueSpecs {
    fn default() -> Self {
        Self {
            mass: 0.567,
            end_mass: 0.170_097 / 30.0,
            tip_radius: 0.010_604_5,
        }
    }
}

/// Cue strike in Pooltool's parameterization.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct CueStrike {
    /// Cue impact speed in meters per second.
    pub speed: f64,
    /// Direction in degrees, counterclockwise from positive table x.
    pub phi: f64,
    /// Cue elevation in degrees.
    pub theta: f64,
    /// Side contact offset, normalized by ball radius.
    pub a: f64,
    /// Follow/draw contact offset, normalized by ball radius.
    pub b: f64,
}

impl CueStrike {
    #[must_use]
    pub const fn new(speed: f64, phi: f64) -> Self {
        Self {
            speed,
            phi,
            theta: 0.0,
            a: 0.0,
            b: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct Ball {
    pub id: BallId,
    pub position: Vec2,
}

/// Pooltool 0.4 pocket-table geometry parameters.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct PocketTableParams {
    pub pocket_depth: f64,
    pub cushion_width: f64,
    pub cushion_height: f64,
    pub corner_pocket_width: f64,
    pub corner_pocket_angle: f64,
    pub corner_pocket_depth: f64,
    pub corner_pocket_radius: f64,
    pub corner_jaw_radius: f64,
    pub side_pocket_width: f64,
    pub side_pocket_angle: f64,
    pub side_pocket_depth: f64,
    pub side_pocket_radius: f64,
    pub side_jaw_radius: f64,
}

impl Default for PocketTableParams {
    fn default() -> Self {
        Self {
            pocket_depth: 0.08,
            cushion_width: 0.0508,
            cushion_height: 0.64 * 2.0 * 0.028_575,
            corner_pocket_width: 0.118,
            corner_pocket_angle: 5.3,
            corner_pocket_depth: 0.0398,
            corner_pocket_radius: 0.062,
            corner_jaw_radius: 0.020_95,
            side_pocket_width: 0.137,
            side_pocket_angle: 7.14,
            side_pocket_depth: 0.004_37,
            side_pocket_radius: 0.0645,
            side_jaw_radius: 0.007_95,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
pub enum PocketId {
    LeftBottom,
    LeftCenter,
    LeftTop,
    RightBottom,
    RightCenter,
    RightTop,
}

/// Standard pocket-table playing-surface dimensions and physics parameters.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct TableSpec {
    pub length: f64,
    pub width: f64,
    pub ball: BallParams,
    pub cue: CueSpecs,
    pub pocket_table: PocketTableParams,
}

impl Default for TableSpec {
    fn default() -> Self {
        Self {
            length: 2.54,
            width: 1.27,
            ball: BallParams::default(),
            cue: CueSpecs::default(),
            pocket_table: PocketTableParams::default(),
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SimulationScenario {
    pub balls: Vec<Ball>,
    pub cue_ball_id: BallId,
    pub strike: CueStrike,
    pub table: TableSpec,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct SimulationOptions {
    pub trajectory_dt: f64,
    pub max_time: f64,
    pub max_events: usize,
}

impl Default for SimulationOptions {
    fn default() -> Self {
        Self {
            trajectory_dt: 0.01,
            max_time: 30.0,
            max_events: 1_000,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub enum MotionState {
    Stationary,
    Spinning,
    Sliding,
    Rolling,
    Pocketed,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub enum SimulationEventType {
    StickBall,
    SlidingRolling,
    RollingSpinning,
    RollingStationary,
    SpinningStationary,
    BallBall,
    BallCushion,
    BallPocket,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SimulationEvent {
    pub event_type: SimulationEventType,
    pub time: f64,
    pub ball_ids: Vec<BallId>,
    pub position: Option<Vec2>,
    pub pocket: Option<PocketId>,
    pub cushion: Option<String>,
    pub velocities_after: Vec<BallVelocity>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct BallVelocity {
    pub ball_id: BallId,
    pub velocity: Vec2,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct TrajectoryPoint {
    pub time: f64,
    pub position: Vec2,
    pub motion_state: MotionState,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BallTrajectory {
    pub ball_id: BallId,
    pub points: Vec<TrajectoryPoint>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct BallState {
    pub ball_id: BallId,
    pub position: Vec2,
    pub motion_state: MotionState,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ShotProjection {
    pub trajectories: Vec<BallTrajectory>,
    pub events: Vec<SimulationEvent>,
    pub final_state: Vec<BallState>,
    pub potted_ball_ids: Vec<BallId>,
}

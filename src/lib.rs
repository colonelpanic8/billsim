//! Portable, headless billiards simulation for Railbird.
//!
//! The public model deliberately mirrors the narrow data boundary used by
//! Railbird Shot Lab while keeping all coordinates in the physics frame and SI
//! units. Image-space conversion belongs in consumers of this crate.

pub mod aiming;
mod detection;
pub mod ffi;
pub mod math;
pub mod model;
pub mod physics;
pub mod simulation;
mod table;

pub use aiming::{AimError, PotAim, compute_pot_aim};
pub use ffi::{AimRequest, FfiError, SimulationRequest, compute_pot_aim_json, simulate_json};

#[cfg(feature = "uniffi-bindings")]
uniffi::setup_scaffolding!();
pub use model::{
    Ball, BallId, BallParams, BallState, BallTrajectory, BallVelocity, CueSpecs, CueStrike,
    MotionState, PocketId, PocketTableParams, ShotProjection, SimulationEvent, SimulationEventType,
    SimulationOptions, SimulationScenario, TableSpec, TrajectoryPoint,
};
pub use simulation::{SimulationError, simulate};

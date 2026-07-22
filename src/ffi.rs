//! Serialization-first boundary shared by Python, Swift, and Kotlin.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::aiming::{PotAim, compute_pot_aim};
use crate::math::Vec2;
use crate::model::{BallId, PocketId, ShotProjection, SimulationOptions, SimulationScenario};
use crate::position::{PositionSearchConfig, PositionSuggestion, suggest_position_shot};
use crate::scenario::{ScenarioCase, ScenarioReport, evaluate_scenario};
use crate::simulation::simulate;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SimulationRequest {
    pub scenario: SimulationScenario,
    #[serde(default)]
    pub options: Option<SimulationOptions>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct AimRequest {
    pub scenario: SimulationScenario,
    pub target_ball_id: BallId,
    pub pocket_id: PocketId,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PositionRequest {
    pub scenario: SimulationScenario,
    pub target_ball_id: BallId,
    pub pocket_id: PocketId,
    /// Target-area polygon vertices in physics-frame meters.
    pub target_area: Vec<Vec2>,
    #[serde(default)]
    pub config: Option<PositionSearchConfig>,
}

#[derive(Debug, Error)]
#[cfg_attr(feature = "uniffi-bindings", derive(uniffi::Error))]
pub enum FfiError {
    #[error("invalid request JSON: {0}")]
    InvalidRequest(String),
    #[error("simulation failed: {0}")]
    Simulation(String),
    #[error("aim computation failed: {0}")]
    Aim(String),
    #[error("could not encode response JSON: {0}")]
    ResponseEncoding(String),
}

/// Run a simulation through the cross-language JSON contract.
///
/// # Errors
///
/// Returns [`FfiError`] when the request cannot be decoded, simulation fails,
/// or the response cannot be encoded.
#[cfg_attr(feature = "uniffi-bindings", uniffi::export)]
#[allow(clippy::needless_pass_by_value)] // UniFFI owns exported strings.
pub fn simulate_json(request: String) -> Result<String, FfiError> {
    let request: SimulationRequest = serde_json::from_str(&request)
        .map_err(|error| FfiError::InvalidRequest(error.to_string()))?;
    let projection: ShotProjection =
        simulate(&request.scenario, request.options.unwrap_or_default())
            .map_err(|error| FfiError::Simulation(error.to_string()))?;
    serde_json::to_string(&projection)
        .map_err(|error| FfiError::ResponseEncoding(error.to_string()))
}

/// Compute a compensated pot aim through the cross-language JSON contract.
///
/// # Errors
///
/// Returns [`FfiError`] when the request cannot be decoded, aiming fails, or
/// the response cannot be encoded.
#[cfg_attr(feature = "uniffi-bindings", uniffi::export)]
#[allow(clippy::needless_pass_by_value)] // UniFFI owns exported strings.
pub fn compute_pot_aim_json(request: String) -> Result<String, FfiError> {
    let request: AimRequest = serde_json::from_str(&request)
        .map_err(|error| FfiError::InvalidRequest(error.to_string()))?;
    let aim: PotAim = compute_pot_aim(&request.scenario, request.target_ball_id, request.pocket_id)
        .map_err(|error| FfiError::Aim(error.to_string()))?;
    serde_json::to_string(&aim).map_err(|error| FfiError::ResponseEncoding(error.to_string()))
}

/// Search for a position-play strike through the cross-language JSON
/// contract: pot `target_ball_id` into `pocket_id` and leave the cue ball
/// inside `target_area`.
///
/// # Errors
///
/// Returns [`FfiError`] when the request cannot be decoded, the search
/// fails, or the response cannot be encoded.
#[cfg_attr(feature = "uniffi-bindings", uniffi::export)]
#[allow(clippy::needless_pass_by_value)] // UniFFI owns exported strings.
pub fn suggest_position_shot_json(request: String) -> Result<String, FfiError> {
    let request: PositionRequest = serde_json::from_str(&request)
        .map_err(|error| FfiError::InvalidRequest(error.to_string()))?;
    let suggestion: PositionSuggestion = suggest_position_shot(
        &request.scenario,
        request.target_ball_id,
        request.pocket_id,
        &request.target_area,
        request.config,
    )
    .map_err(|error| FfiError::Aim(error.to_string()))?;
    serde_json::to_string(&suggestion)
        .map_err(|error| FfiError::ResponseEncoding(error.to_string()))
}

/// Grade a scenario test case through the cross-language JSON contract:
/// run its position search and match the result against its expectations.
///
/// # Errors
///
/// Returns [`FfiError`] when the request cannot be decoded, the search
/// fails, or the response cannot be encoded. Expectation mismatches are
/// not errors; they are reported in the returned [`ScenarioReport`].
#[cfg_attr(feature = "uniffi-bindings", uniffi::export)]
#[allow(clippy::needless_pass_by_value)] // UniFFI owns exported strings.
pub fn evaluate_scenario_json(case: String) -> Result<String, FfiError> {
    let case: ScenarioCase =
        serde_json::from_str(&case).map_err(|error| FfiError::InvalidRequest(error.to_string()))?;
    let report: ScenarioReport =
        evaluate_scenario(&case).map_err(|error| FfiError::Aim(error.to_string()))?;
    serde_json::to_string(&report).map_err(|error| FfiError::ResponseEncoding(error.to_string()))
}

#[cfg(feature = "python")]
mod python {
    use pyo3::exceptions::PyValueError;
    use pyo3::prelude::*;

    use super::{
        compute_pot_aim_json, evaluate_scenario_json, simulate_json, suggest_position_shot_json,
    };

    #[pyfunction(name = "simulate_json")]
    fn py_simulate_json(request: String) -> PyResult<String> {
        simulate_json(request).map_err(|error| PyValueError::new_err(error.to_string()))
    }

    #[pyfunction(name = "suggest_position_shot_json")]
    fn py_suggest_position_shot_json(request: String) -> PyResult<String> {
        suggest_position_shot_json(request)
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    #[pyfunction(name = "compute_pot_aim_json")]
    fn py_compute_pot_aim_json(request: String) -> PyResult<String> {
        compute_pot_aim_json(request).map_err(|error| PyValueError::new_err(error.to_string()))
    }

    #[pyfunction(name = "evaluate_scenario_json")]
    fn py_evaluate_scenario_json(case: String) -> PyResult<String> {
        evaluate_scenario_json(case).map_err(|error| PyValueError::new_err(error.to_string()))
    }

    #[pymodule]
    fn _native(module: &Bound<'_, PyModule>) -> PyResult<()> {
        module.add_function(wrap_pyfunction!(py_simulate_json, module)?)?;
        module.add_function(wrap_pyfunction!(py_compute_pot_aim_json, module)?)?;
        module.add_function(wrap_pyfunction!(py_suggest_position_shot_json, module)?)?;
        module.add_function(wrap_pyfunction!(py_evaluate_scenario_json, module)?)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;
    use crate::model::{Ball, CueStrike, TableSpec};

    fn scenario() -> SimulationScenario {
        SimulationScenario {
            balls: vec![Ball {
                id: 0,
                position: Vec2::new(0.5, 0.5),
            }],
            cue_ball_id: 0,
            strike: CueStrike::new(1.0, 0.0),
            table: TableSpec {
                width: 100.0,
                length: 100.0,
                ..TableSpec::default()
            },
        }
    }

    #[test]
    fn simulation_json_round_trip() {
        let request = serde_json::to_string(&SimulationRequest {
            scenario: scenario(),
            options: None,
        })
        .unwrap();
        let response = simulate_json(request).unwrap();
        let projection: ShotProjection = serde_json::from_str(&response).unwrap();
        assert_eq!(projection.final_state.len(), 1);
    }
}

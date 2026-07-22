//! JSON-string WASM surface for billsim, mirroring the PyO3/UniFFI exports.
//!
//! All functions speak the wire format documented in
//! `docs/wire-format.md`: a JSON request string in, a JSON response string
//! out. Errors surface as thrown JS exceptions carrying the FfiError
//! message.

use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn simulate_json(request: &str) -> Result<String, JsError> {
    billsim::simulate_json(request.to_owned()).map_err(|error| JsError::new(&error.to_string()))
}

#[wasm_bindgen]
pub fn compute_pot_aim_json(request: &str) -> Result<String, JsError> {
    billsim::compute_pot_aim_json(request.to_owned())
        .map_err(|error| JsError::new(&error.to_string()))
}

#[wasm_bindgen]
pub fn suggest_position_shot_json(request: &str) -> Result<String, JsError> {
    billsim::suggest_position_shot_json(request.to_owned())
        .map_err(|error| JsError::new(&error.to_string()))
}

#[wasm_bindgen]
pub fn evaluate_scenario_json(case: &str) -> Result<String, JsError> {
    billsim::evaluate_scenario_json(case.to_owned())
        .map_err(|error| JsError::new(&error.to_string()))
}

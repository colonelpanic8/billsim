//! Data-format test cases ("scenarios") for the position search.
//!
//! A scenario pairs a raw [`PositionRequest`] with human-authored
//! expectations about the shots the search should find. Expectations are
//! fuzzy: an expected shot names a rail route and, optionally, a spin
//! family or strike parameters with tolerances, and it is satisfied when
//! any returned candidate matches. The same evaluation runs in `cargo
//! test` (over `tests/scenarios/*.json`) and through the JSON FFI
//! ([`crate::ffi::evaluate_scenario_json`]), so an authoring UI can show
//! exactly what a case asserts alongside what the engine currently finds.

use serde::{Deserialize, Serialize};

use crate::ffi::PositionRequest;
use crate::position::{
    PositionError, PositionShotCandidate, PositionSuggestion, spin_family, suggest_position_shot,
};

/// One scenario file: a search request plus expectations about its result.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ScenarioCase {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Attribution for cases derived from published drills or articles.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    pub request: PositionRequest,
    pub expect: ScenarioExpectations,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ScenarioExpectations {
    /// Whether the search should report the pot-and-park as achievable.
    #[serde(default = "default_true")]
    pub achievable: bool,
    /// Shots the search should surface among `best` + `alternates`.
    #[serde(default)]
    pub shots: Vec<ExpectedShot>,
    /// Minimum number of `shots` that must be found. Defaults to all of
    /// them; lower it to tolerate known-flaky approaches without deleting
    /// their descriptions.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_found: Option<u32>,
}

const fn default_true() -> bool {
    true
}

/// A fuzzy description of one shot the search is expected to find.
///
/// Every field except `label` is optional; only specified fields
/// constrain the match. Numeric fields match within [`ShotTolerances`].
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ExpectedShot {
    pub label: String,
    /// Ordered cue-ball rail route. Labels are the candidate `cue_route`
    /// vocabulary (`bottom`, `top`, `left_low`, `left_high`, `right_low`,
    /// `right_high`); the coarse forms `left` / `right` match either
    /// half. `[]` means a direct, no-cushion leave.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub route: Option<Vec<String>>,
    /// Spin family of the strike: `draw`, `stun`, or `follow`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spin: Option<String>,
    /// Strike speed in m/s.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed: Option<f64>,
    /// Aim direction, physics-frame degrees.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub phi: Option<f64>,
    /// Cue elevation in degrees.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub theta: Option<f64>,
    /// Side contact offset, ball-radius units.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub a: Option<f64>,
    /// Follow/draw contact offset, ball-radius units.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub b: Option<f64>,
    #[serde(default)]
    pub tolerances: ShotTolerances,
}

/// Half-widths of the acceptance window around each specified strike
/// parameter. Defaults are deliberately loose — scenario cases exist to
/// pin down shot families, not exact solver output.
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct ShotTolerances {
    /// m/s.
    pub speed: f64,
    /// Degrees, compared on the circle.
    pub phi: f64,
    /// Degrees.
    pub theta: f64,
    /// Ball-radius units.
    pub a: f64,
    /// Ball-radius units.
    pub b: f64,
}

impl Default for ShotTolerances {
    fn default() -> Self {
        Self {
            speed: 0.35,
            phi: 3.0,
            theta: 5.0,
            a: 0.2,
            b: 0.2,
        }
    }
}

/// Result of matching one [`ExpectedShot`] against the returned candidates.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ExpectedShotOutcome {
    pub label: String,
    pub found: bool,
    /// Index of the first matching candidate: 0 is `best`, then the
    /// `alternates` in order.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub matched_candidate: Option<u32>,
}

/// The evaluation of one [`ScenarioCase`]: pass/fail plus the full
/// suggestion, so authoring tools can show what the engine found next to
/// what the case expected.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ScenarioReport {
    pub name: String,
    pub passed: bool,
    /// Human-readable reasons the case failed; empty when `passed`.
    pub failures: Vec<String>,
    pub outcomes: Vec<ExpectedShotOutcome>,
    pub suggestion: PositionSuggestion,
}

/// Run the position search for `case` and grade the result against its
/// expectations.
///
/// # Errors
///
/// Returns [`PositionError`] when the search itself cannot run (a failure
/// to *match expectations* is reported in the [`ScenarioReport`], not as
/// an error).
pub fn evaluate_scenario(case: &ScenarioCase) -> Result<ScenarioReport, PositionError> {
    let request = &case.request;
    let suggestion = suggest_position_shot(
        &request.scenario,
        request.target_ball_id,
        request.pocket_id,
        &request.target_area,
        request.config.clone(),
    )?;
    let candidates: Vec<&PositionShotCandidate> = suggestion
        .best
        .iter()
        .chain(&suggestion.alternates)
        .collect();

    let mut failures = Vec::new();
    if suggestion.achievable != case.expect.achievable {
        failures.push(format!(
            "expected achievable = {}, search reported {}",
            case.expect.achievable, suggestion.achievable
        ));
    }

    let outcomes: Vec<ExpectedShotOutcome> = case
        .expect
        .shots
        .iter()
        .map(|shot| {
            let matched = candidates
                .iter()
                .position(|candidate| shot_matches(shot, candidate));
            ExpectedShotOutcome {
                label: shot.label.clone(),
                found: matched.is_some(),
                matched_candidate: matched.map(|index| u32::try_from(index).unwrap_or(u32::MAX)),
            }
        })
        .collect();

    let found = outcomes.iter().filter(|outcome| outcome.found).count();
    let required = case
        .expect
        .min_found
        .map_or(case.expect.shots.len(), |minimum| minimum as usize);
    if found < required {
        let returned: Vec<&Vec<String>> = candidates
            .iter()
            .map(|candidate| &candidate.cue_route)
            .collect();
        failures.push(format!(
            "found {found}/{} expected shots (need {required}); returned routes {returned:?}",
            case.expect.shots.len()
        ));
        for outcome in outcomes.iter().filter(|outcome| !outcome.found) {
            failures.push(format!("missing expected shot: {}", outcome.label));
        }
    }

    Ok(ScenarioReport {
        name: case.name.clone(),
        passed: failures.is_empty(),
        failures,
        outcomes,
        suggestion,
    })
}

fn shot_matches(expected: &ExpectedShot, candidate: &PositionShotCandidate) -> bool {
    if let Some(route) = &expected.route
        && !route_matches(route, &candidate.cue_route)
    {
        return false;
    }
    if let Some(spin) = &expected.spin
        && spin != spin_family(candidate.b)
    {
        return false;
    }
    let tolerances = expected.tolerances;
    within(expected.speed, candidate.speed, tolerances.speed)
        && angle_within(expected.phi, candidate.phi, tolerances.phi)
        && within(expected.theta, candidate.theta, tolerances.theta)
        && within(expected.a, candidate.a, tolerances.a)
        && within(expected.b, candidate.b, tolerances.b)
}

fn within(expected: Option<f64>, actual: f64, tolerance: f64) -> bool {
    expected.is_none_or(|value| (value - actual).abs() <= tolerance)
}

fn angle_within(expected: Option<f64>, actual: f64, tolerance: f64) -> bool {
    expected.is_none_or(|value| {
        let difference = (value - actual).abs() % 360.0;
        difference.min(360.0 - difference) <= tolerance
    })
}

const COARSE_LABELS: [&str; 4] = ["bottom", "top", "left", "right"];

fn coarse(label: &str) -> &str {
    label
        .strip_suffix("_low")
        .or_else(|| label.strip_suffix("_high"))
        .unwrap_or(label)
}

/// An expected route written entirely in coarse labels compares against
/// the candidate route with the long-rail halves merged (and consecutive
/// duplicates re-collapsed); a route using any `_low`/`_high` label
/// compares position-by-position, with coarse entries matching either
/// half of their rail.
fn route_matches(expected: &[String], actual: &[String]) -> bool {
    let all_coarse = expected
        .iter()
        .all(|label| COARSE_LABELS.contains(&label.as_str()));
    if all_coarse {
        let mut collapsed: Vec<&str> = Vec::new();
        for label in actual {
            let merged = coarse(label);
            if collapsed.last() != Some(&merged) {
                collapsed.push(merged);
            }
        }
        collapsed.len() == expected.len()
            && collapsed
                .iter()
                .zip(expected)
                .all(|(actual, expected)| *actual == expected)
    } else {
        expected.len() == actual.len()
            && expected
                .iter()
                .zip(actual)
                .all(|(expected, actual)| expected == actual || coarse(actual) == expected)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strings(labels: &[&str]) -> Vec<String> {
        labels.iter().map(|label| (*label).to_owned()).collect()
    }

    #[test]
    fn coarse_route_merges_long_rail_halves() {
        assert!(route_matches(
            &strings(&["bottom", "right"]),
            &strings(&["bottom", "right_low", "right_high"]),
        ));
        assert!(!route_matches(
            &strings(&["bottom", "right"]),
            &strings(&["bottom", "left_low"]),
        ));
    }

    #[test]
    fn fine_route_requires_exact_halves() {
        assert!(route_matches(
            &strings(&["bottom", "right_high"]),
            &strings(&["bottom", "right_high"]),
        ));
        assert!(!route_matches(
            &strings(&["bottom", "right_high"]),
            &strings(&["bottom", "right_low"]),
        ));
    }

    #[test]
    fn mixed_route_lets_coarse_entries_match_either_half() {
        assert!(route_matches(
            &strings(&["right", "top"]),
            &strings(&["right_low", "top"]),
        ));
    }

    #[test]
    fn empty_route_means_no_cushions() {
        assert!(route_matches(&strings(&[]), &strings(&[])));
        assert!(!route_matches(&strings(&[]), &strings(&["bottom"])));
    }

    #[test]
    fn phi_matches_across_the_wraparound() {
        assert!(angle_within(Some(359.0), 1.0, 3.0));
        assert!(!angle_within(Some(350.0), 10.0, 3.0));
    }
}

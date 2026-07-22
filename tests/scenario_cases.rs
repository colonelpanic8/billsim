//! Runs every scenario file in `tests/scenarios/` through the shared
//! evaluator. Authoring docs: `docs/scenario-format.md`.

use std::fs;
use std::path::PathBuf;

use billsim::scenario::{ScenarioCase, evaluate_scenario};

fn load_cases() -> Vec<(String, ScenarioCase)> {
    let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/scenarios");
    let mut paths: Vec<PathBuf> = fs::read_dir(&directory)
        .unwrap_or_else(|error| panic!("could not read {}: {error}", directory.display()))
        .map(|entry| entry.expect("readable directory entry").path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension == "json")
        })
        .collect();
    paths.sort();
    paths
        .into_iter()
        .map(|path| {
            let file = path
                .file_name()
                .expect("file name")
                .to_string_lossy()
                .into_owned();
            let contents = fs::read_to_string(&path)
                .unwrap_or_else(|error| panic!("could not read {file}: {error}"));
            let case: ScenarioCase = serde_json::from_str(&contents)
                .unwrap_or_else(|error| panic!("{file} is not a valid scenario case: {error}"));
            (file, case)
        })
        .collect()
}

#[test]
fn scenario_directory_has_cases_with_unique_names() {
    let cases = load_cases();
    assert!(!cases.is_empty(), "tests/scenarios/ has no scenario files");
    for (file, case) in &cases {
        assert!(
            cases
                .iter()
                .filter(|(_, other)| other.name == case.name)
                .count()
                == 1,
            "{file}: duplicate scenario name {}",
            case.name
        );
    }
}

#[test]
fn every_scenario_case_passes() {
    let mut failures = Vec::new();
    for (file, case) in load_cases() {
        let report = evaluate_scenario(&case)
            .unwrap_or_else(|error| panic!("{file} ({}): search failed: {error}", case.name));
        if !report.passed {
            failures.push(format!(
                "{file} ({}):\n  {}",
                case.name,
                report.failures.join("\n  ")
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "scenario cases failed:\n{}",
        failures.join("\n")
    );
}

//! Source-attributed, behavior-level fixtures for position searches.
//!
//! These fixtures deliberately separate two claims:
//! 1. the simulator can execute each documented position-play approach; and
//! 2. the production search can discover the known approaches.

use billsim::math::Vec2;
use billsim::{
    Ball, CueStrike, PocketId, PositionSearchConfig, PositionShotCandidate, ScoringWeights,
    ShotProjection, SimulationEventType, SimulationOptions, SimulationScenario, TableSpec,
    compute_pot_aim_seeded, simulate, suggest_position_shot,
};
use std::time::Instant;

const ROSS_SOURCE: &str = "Tom Ross, Position Play, Billiards Digest (2002)";
const BU_SOURCE: &str = "Billiard University Exam I - Fundamentals, drills F5 and F8";

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum Rail {
    Bottom,
    Top,
    Left,
    Right,
}

#[derive(Clone, Debug)]
struct ExpectedApproach {
    name: &'static str,
    speed: f64,
    a: f64,
    b: f64,
    route: Vec<Rail>,
}

#[derive(Clone, Debug)]
struct PositionFixture {
    name: &'static str,
    source: &'static str,
    scenario: SimulationScenario,
    target_ball_id: u32,
    pocket: PocketId,
    target_area: Vec<Vec2>,
    expected: Vec<ExpectedApproach>,
}

#[derive(Clone, Debug)]
struct ElongatedTargetFixture {
    name: &'static str,
    scenario: SimulationScenario,
    target_ball_id: u32,
    pocket: PocketId,
    target_area: Vec<Vec2>,
}

fn approach(name: &'static str, speed: f64, a: f64, b: f64, route: &[Rail]) -> ExpectedApproach {
    ExpectedApproach {
        name,
        speed,
        a,
        b,
        route: route.to_vec(),
    }
}

fn scenario(cue: Vec2, object: Vec2, object_id: u32) -> SimulationScenario {
    SimulationScenario {
        balls: vec![
            Ball {
                id: 0,
                position: cue,
            },
            Ball {
                id: object_id,
                position: object,
            },
        ],
        cue_ball_id: 0,
        strike: CueStrike {
            speed: 1.0,
            phi: 0.0,
            theta: 0.0,
            a: 0.0,
            b: 0.0,
        },
        table: TableSpec::default(),
    }
}

fn rectangle(center: Vec2, half_x: f64, half_y: f64) -> Vec<Vec2> {
    vec![
        Vec2::new(center.x - half_x, center.y - half_y),
        Vec2::new(center.x + half_x, center.y - half_y),
        Vec2::new(center.x + half_x, center.y + half_y),
        Vec2::new(center.x - half_x, center.y + half_y),
    ]
}

fn mirrored_x(position: Vec2) -> Vec2 {
    Vec2::new(TableSpec::default().width - position.x, position.y)
}

fn mirrored_area(area: &[Vec2]) -> Vec<Vec2> {
    area.iter().copied().map(mirrored_x).rev().collect()
}

fn mirrored_route(route: &[Rail]) -> Vec<Rail> {
    route
        .iter()
        .map(|rail| match rail {
            Rail::Left => Rail::Right,
            Rail::Right => Rail::Left,
            other => *other,
        })
        .collect()
}

fn ross_fixture() -> PositionFixture {
    PositionFixture {
        name: "ross_four_track_breakout_position",
        source: ROSS_SOURCE,
        scenario: scenario(Vec2::new(0.53, 0.27), Vec2::new(0.3175, 0.0413), 7),
        target_ball_id: 7,
        pocket: PocketId::LeftBottom,
        target_area: rectangle(Vec2::new(1.16, 1.905), 0.08, 0.16),
        expected: vec![
            approach(
                "direct short rail",
                1.288_235,
                0.25,
                -0.106_25,
                &[Rail::Bottom],
            ),
            approach(
                "short-short",
                2.064_706,
                0.0,
                -0.425,
                &[Rail::Bottom, Rail::Top],
            ),
            approach(
                "short-long",
                1.676_471,
                0.5,
                0.0,
                &[Rail::Bottom, Rail::Right],
            ),
            approach(
                "short-long-short",
                2.647_059,
                -0.25,
                0.637_5,
                &[Rail::Bottom, Rail::Left, Rail::Top],
            ),
        ],
    }
}

fn bu_f8_fixture(
    name: &'static str,
    center: Vec2,
    expected: Vec<ExpectedApproach>,
) -> PositionFixture {
    PositionFixture {
        name,
        source: BU_SOURCE,
        scenario: scenario(Vec2::new(0.635, 0.476_25), Vec2::new(0.3175, 0.3175), 1),
        target_ball_id: 1,
        pocket: PocketId::LeftBottom,
        target_area: rectangle(center, 0.108, 0.1395),
        expected,
    }
}

fn bu_f8_fixtures() -> Vec<PositionFixture> {
    vec![
        bu_f8_fixture(
            "bu_f8_target_1",
            Vec2::new(0.14, 0.635),
            vec![
                approach("top long rail", 1.094_118, 0.0, -0.2125, &[Rail::Left]),
                approach("direct", 1.288_235, 0.0, -0.425, &[]),
            ],
        ),
        bu_f8_fixture(
            "bu_f8_target_2",
            Vec2::new(0.14, 1.905),
            vec![
                approach("top long rail", 2.064_706, 0.25, -0.318_75, &[Rail::Left]),
                approach("far short rail", 4.005_882, 0.5, -0.425, &[Rail::Top]),
            ],
        ),
        bu_f8_fixture(
            "bu_f8_target_3",
            Vec2::new(1.13, 1.905),
            vec![
                approach(
                    "two long rails",
                    2.647_059,
                    0.0,
                    0.0,
                    &[Rail::Left, Rail::Right],
                ),
                approach(
                    "long-short",
                    2.647_059,
                    -0.25,
                    -0.106_25,
                    &[Rail::Left, Rail::Top],
                ),
            ],
        ),
        bu_f8_fixture(
            "bu_f8_target_4",
            Vec2::new(0.635, 1.27),
            vec![
                approach("top long rail", 1.870_588, 0.0, -0.106_25, &[Rail::Left]),
                approach("direct", 1.870_588, 0.0, -0.6375, &[]),
            ],
        ),
        bu_f8_fixture(
            "bu_f8_target_5",
            Vec2::new(1.13, 0.635),
            vec![
                approach("top long rail", 1.676_471, 0.0, 0.425, &[Rail::Left]),
                approach(
                    "two long rails",
                    2.258_824,
                    0.0,
                    0.6375,
                    &[Rail::Left, Rail::Right],
                ),
            ],
        ),
    ]
}

fn bu_f5_base() -> SimulationScenario {
    scenario(Vec2::new(0.45, 1.15), Vec2::new(0.692_15, 1.27), 1)
}

fn bu_f5_fixture(
    name: &'static str,
    center: Vec2,
    half_x: f64,
    half_y: f64,
    expected: ExpectedApproach,
) -> PositionFixture {
    PositionFixture {
        name,
        source: BU_SOURCE,
        scenario: bu_f5_base(),
        target_ball_id: 1,
        pocket: PocketId::RightCenter,
        target_area: rectangle(center, half_x, half_y),
        expected: vec![expected],
    }
}

fn bu_f5_fixtures() -> Vec<PositionFixture> {
    let target_1 = Vec2::new(0.635, 1.5875);
    let target_2 = Vec2::new(0.635, 1.905);
    let target_3 = Vec2::new(0.635, 2.2225);
    vec![
        bu_f5_fixture(
            "bu_f5_position_1_direct",
            target_1,
            0.108,
            0.1395,
            approach("direct", 1.094_118, 0.0, -0.425, &[]),
        ),
        bu_f5_fixture(
            "bu_f5_position_2_direct",
            target_2,
            0.108,
            0.1395,
            approach("direct", 1.094_118, 0.25, -0.318_75, &[]),
        ),
        bu_f5_fixture(
            "bu_f5_position_3_direct",
            target_3,
            0.108,
            0.1395,
            approach("direct", 1.094_118, 0.25, -0.2125, &[]),
        ),
        bu_f5_fixture(
            "bu_f5_position_4_end_zone",
            Vec2::new(0.635, 2.432),
            0.1395,
            0.108,
            approach("direct", 1.482_353, -0.5, -0.2125, &[]),
        ),
        bu_f5_fixture(
            "bu_f5_position_5_end_rail",
            target_3,
            0.108,
            0.1395,
            approach("end-rail rebound", 1.482_353, -0.25, -0.2125, &[Rail::Top]),
        ),
        bu_f5_fixture(
            "bu_f5_position_6_end_rail",
            target_2,
            0.108,
            0.1395,
            approach("end-rail rebound", 1.482_353, 0.25, 0.0, &[Rail::Top]),
        ),
        bu_f5_fixture(
            "bu_f5_position_7_end_rail",
            target_1,
            0.108,
            0.1395,
            approach("end-rail rebound", 1.676_471, 0.0, -0.106_25, &[Rail::Top]),
        ),
    ]
}

fn mirrored_bu_f8_fixture(mut fixture: PositionFixture, name: &'static str) -> PositionFixture {
    for ball in &mut fixture.scenario.balls {
        ball.position = mirrored_x(ball.position);
    }
    fixture.name = name;
    fixture.pocket = match fixture.pocket {
        PocketId::LeftBottom => PocketId::RightBottom,
        other => other,
    };
    fixture.target_area = mirrored_area(&fixture.target_area);
    for expected in &mut fixture.expected {
        expected.a = -expected.a;
        expected.route = mirrored_route(&expected.route);
    }
    fixture
}

fn fixtures() -> Vec<PositionFixture> {
    let f8 = bu_f8_fixtures();
    let mut result = vec![ross_fixture()];
    result.extend(f8.clone());
    result.extend(bu_f5_fixtures());
    result.push(mirrored_bu_f8_fixture(
        f8[3].clone(),
        "bu_f8_target_4_mirrored",
    ));
    result.push(mirrored_bu_f8_fixture(
        f8[4].clone(),
        "bu_f8_target_5_mirrored",
    ));
    result
}

fn elongated_target_fixtures() -> Vec<ElongatedTargetFixture> {
    let f8 = scenario(Vec2::new(0.635, 0.476_25), Vec2::new(0.3175, 0.3175), 1);
    let f5 = bu_f5_base();
    let ross = ross_fixture().scenario;
    vec![
        ElongatedTargetFixture {
            name: "bu_f8_center_long_table_axis",
            scenario: f8.clone(),
            target_ball_id: 1,
            pocket: PocketId::LeftBottom,
            target_area: rectangle(Vec2::new(0.635, 1.27), 0.06, 0.40),
        },
        ElongatedTargetFixture {
            name: "bu_f8_center_long_short_axis",
            scenario: f8,
            target_ball_id: 1,
            pocket: PocketId::LeftBottom,
            target_area: rectangle(Vec2::new(0.635, 1.27), 0.40, 0.06),
        },
        ElongatedTargetFixture {
            name: "bu_f5_far_long_table_axis",
            scenario: f5.clone(),
            target_ball_id: 1,
            pocket: PocketId::RightCenter,
            target_area: rectangle(Vec2::new(0.635, 1.905), 0.06, 0.40),
        },
        ElongatedTargetFixture {
            name: "bu_f5_far_long_short_axis",
            scenario: f5,
            target_ball_id: 1,
            pocket: PocketId::RightCenter,
            target_area: rectangle(Vec2::new(0.635, 1.905), 0.40, 0.06),
        },
        ElongatedTargetFixture {
            name: "ross_far_long_table_axis",
            scenario: ross.clone(),
            target_ball_id: 7,
            pocket: PocketId::LeftBottom,
            target_area: rectangle(Vec2::new(1.16, 1.905), 0.06, 0.35),
        },
        ElongatedTargetFixture {
            name: "ross_far_long_short_axis",
            scenario: ross,
            target_ball_id: 7,
            pocket: PocketId::LeftBottom,
            target_area: rectangle(Vec2::new(1.00, 1.905), 0.25, 0.06),
        },
    ]
}

fn rail_at(position: Vec2, table: &TableSpec) -> Rail {
    [
        (position.y, Rail::Bottom),
        (table.length - position.y, Rail::Top),
        (position.x, Rail::Left),
        (table.width - position.x, Rail::Right),
    ]
    .into_iter()
    .min_by(|lhs, rhs| {
        lhs.0
            .partial_cmp(&rhs.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
    .expect("four rails")
    .1
}

fn cue_route(projection: &ShotProjection, table: &TableSpec) -> Vec<Rail> {
    let mut route = Vec::new();
    for event in &projection.events {
        if event.event_type != SimulationEventType::BallCushion || !event.ball_ids.contains(&0) {
            continue;
        }
        let Some(position) = event.position else {
            continue;
        };
        let rail = rail_at(position, table);
        if route.last() != Some(&rail) {
            route.push(rail);
        }
    }
    route
}

fn candidate_route(candidate: &PositionShotCandidate, table: &TableSpec) -> Vec<Rail> {
    candidate
        .projection
        .as_ref()
        .map_or_else(Vec::new, |projection| cue_route(projection, table))
}

fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    #[allow(clippy::cast_precision_loss)]
    let step = (end - start) / (count - 1) as f64;
    #[allow(clippy::cast_precision_loss)]
    (0..count)
        .map(|index| start + step * index as f64)
        .collect()
}

#[derive(Debug)]
struct RecallReport {
    found: usize,
    expected: usize,
    evaluated: usize,
    missing: Vec<String>,
}

fn corpus_recall(config: Option<&PositionSearchConfig>) -> RecallReport {
    let mut found = 0usize;
    let mut expected_count = 0usize;
    let mut evaluated = 0usize;
    let mut missing = Vec::new();
    for fixture in fixtures() {
        let suggestion = suggest_position_shot(
            &fixture.scenario,
            fixture.target_ball_id,
            fixture.pocket,
            &fixture.target_area,
            config.cloned(),
        )
        .unwrap_or_else(|error| panic!("{} search failed: {error}", fixture.name));
        evaluated += suggestion.evaluated_count as usize;
        let routes: Vec<_> = suggestion
            .best
            .iter()
            .chain(&suggestion.alternates)
            .map(|candidate| candidate_route(candidate, &fixture.scenario.table))
            .collect();

        for expected in &fixture.expected {
            expected_count += 1;
            if routes.contains(&expected.route) {
                found += 1;
            } else {
                missing.push(format!(
                    "{} missing {} {:?}; returned {routes:?}",
                    fixture.name, expected.name, expected.route
                ));
            }
        }
    }
    RecallReport {
        found,
        expected: expected_count,
        evaluated,
        missing,
    }
}

fn point_in_fixture_target(point: Vec2, fixture: &PositionFixture) -> bool {
    let min_x = fixture
        .target_area
        .iter()
        .map(|point| point.x)
        .fold(f64::INFINITY, f64::min);
    let max_x = fixture
        .target_area
        .iter()
        .map(|point| point.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = fixture
        .target_area
        .iter()
        .map(|point| point.y)
        .fold(f64::INFINITY, f64::min);
    let max_y = fixture
        .target_area
        .iter()
        .map(|point| point.y)
        .fold(f64::NEG_INFINITY, f64::max);
    (min_x..=max_x).contains(&point.x) && (min_y..=max_y).contains(&point.y)
}

// Tom Ross fixture source:
// https://drdavepoolinfo.com/resource_files/trcd/BILLIRDS%20DIGEST%20PDFs/902%20TR%20Position%20Play.pdf
//
// Billiard University fixtures source (F5 and F8; mirrored drills are
// explicitly allowed by the general Exam I instructions):
// https://billiarduniversity.org/documents/BU_Exam-I_Fundamentals.pdf
#[test]
fn corpus_has_fifteen_source_attributed_scenarios_with_multi_route_priority() {
    let fixtures = fixtures();
    assert_eq!(fixtures.len(), 15);
    assert!(fixtures.iter().all(|fixture| !fixture.source.is_empty()));
    assert!(fixtures.iter().map(|fixture| fixture.name).all(|name| {
        fixtures
            .iter()
            .filter(|fixture| fixture.name == name)
            .count()
            == 1
    }));
    assert!(
        fixtures
            .iter()
            .filter(|fixture| fixture.expected.len() > 1)
            .count()
            >= 8
    );
}

#[test]
fn every_expected_approach_is_supported_by_the_simulator() {
    for fixture in fixtures() {
        for expected in &fixture.expected {
            let mut shot = fixture.scenario.clone();
            shot.strike.speed = expected.speed;
            shot.strike.a = expected.a;
            shot.strike.b = expected.b;
            let aim = compute_pot_aim_seeded(&shot, fixture.target_ball_id, fixture.pocket, None)
                .unwrap_or_else(|error| {
                    panic!(
                        "{} / {} ({}) aim failed: {error}",
                        fixture.name, expected.name, fixture.source
                    )
                });
            assert!(
                aim.potted,
                "{} / {} ({}) did not solve the pot",
                fixture.name, expected.name, fixture.source
            );
            shot.strike.phi = aim.phi;

            let projection =
                simulate(&shot, SimulationOptions::default()).unwrap_or_else(|error| {
                    panic!(
                        "{} / {} ({}) simulation failed: {error}",
                        fixture.name, expected.name, fixture.source
                    )
                });
            assert!(
                projection.potted_ball_ids.contains(&fixture.target_ball_id),
                "{} / {} did not pot the target",
                fixture.name,
                expected.name
            );
            assert!(
                !projection.potted_ball_ids.contains(&shot.cue_ball_id),
                "{} / {} scratched",
                fixture.name,
                expected.name
            );
            let cue_final = projection
                .final_state
                .iter()
                .find(|state| state.ball_id == shot.cue_ball_id)
                .expect("cue ball should remain on the table")
                .position;
            assert!(
                point_in_fixture_target(cue_final, &fixture),
                "{} / {} ended at {cue_final:?}, outside {:?}",
                fixture.name,
                expected.name,
                fixture.target_area
            );
            assert_eq!(
                cue_route(&projection, &shot.table),
                expected.route,
                "{} / {} took the wrong route",
                fixture.name,
                expected.name
            );
        }
    }
}

// Acceptance target for the adaptive production search.
#[test]
fn production_search_finds_every_expected_route_family() {
    let report = corpus_recall(None);
    println!(
        "production search found {}/{} expected approaches in {} cells",
        report.found, report.expected, report.evaluated
    );
    assert!(
        report.missing.is_empty(),
        "production search found {}/{} expected approaches:\n{}",
        report.found,
        report.expected,
        report.missing.join("\n")
    );
}

#[test]
fn elongated_zones_reward_arrivals_along_the_long_axis() {
    let mut f5_table_axis_dwell = None;
    let mut f5_short_axis_dwell = None;
    let mut ross_table_axis_dwell = None;
    let mut ross_short_axis_dwell = None;

    for fixture in elongated_target_fixtures() {
        if !fixture.name.starts_with("bu_f5") && !fixture.name.starts_with("ross") {
            continue;
        }
        let suggestion = suggest_position_shot(
            &fixture.scenario,
            fixture.target_ball_id,
            fixture.pocket,
            &fixture.target_area,
            None,
        )
        .unwrap_or_else(|error| panic!("{} search failed: {error}", fixture.name));
        let best = suggestion
            .best
            .unwrap_or_else(|| panic!("{} should be achievable", fixture.name));
        match fixture.name {
            "bu_f5_far_long_table_axis" => f5_table_axis_dwell = Some(best.dwell),
            "bu_f5_far_long_short_axis" => f5_short_axis_dwell = Some(best.dwell),
            "ross_far_long_table_axis" => ross_table_axis_dwell = Some(best.dwell),
            "ross_far_long_short_axis" => ross_short_axis_dwell = Some(best.dwell),
            _ => {}
        }
    }

    let f5_table = f5_table_axis_dwell.expect("F5 table-axis fixture");
    let f5_short = f5_short_axis_dwell.expect("F5 short-axis fixture");
    let ross_table = ross_table_axis_dwell.expect("Ross table-axis fixture");
    let ross_short = ross_short_axis_dwell.expect("Ross short-axis fixture");
    assert!(
        f5_table > 2.0 * f5_short,
        "F5 along-axis dwell {f5_table:.3} should dominate cross-axis dwell {f5_short:.3}"
    );
    assert!(
        ross_table > 2.0 * ross_short,
        "Ross along-axis dwell {ross_table:.3} should dominate cross-axis dwell {ross_short:.3}"
    );
}

// Opt-in search-design harness. This compares coverage rather than asserting a
// particular grid, keeping the corpus contract independent of implementation.
#[test]
#[ignore = "expensive corpus search experiment"]
fn compare_search_grid_recall() {
    let experiments = [
        ("production-default", None),
        (
            "three-side-spin-slices",
            Some(PositionSearchConfig {
                speed_values: linspace(0.9, 4.2, 12),
                b_values: linspace(-0.85, 0.85, 9),
                a_values: vec![-0.5, 0.0, 0.5],
                fallback_a_values: vec![],
                scoring: ScoringWeights::default(),
            }),
        ),
        (
            "five-side-spin-slices",
            Some(PositionSearchConfig {
                speed_values: linspace(0.9, 4.2, 12),
                b_values: linspace(-0.85, 0.85, 9),
                a_values: linspace(-0.5, 0.5, 5),
                fallback_a_values: vec![],
                scoring: ScoringWeights::default(),
            }),
        ),
        (
            "fine-speed-grid",
            Some(PositionSearchConfig {
                speed_values: linspace(0.9, 4.2, 18),
                b_values: linspace(-0.85, 0.85, 9),
                a_values: linspace(-0.5, 0.5, 5),
                fallback_a_values: vec![],
                scoring: ScoringWeights::default(),
            }),
        ),
        (
            "fine-vertical-spin-grid",
            Some(PositionSearchConfig {
                speed_values: linspace(0.9, 4.2, 12),
                b_values: linspace(-0.85, 0.85, 17),
                a_values: linspace(-0.5, 0.5, 5),
                fallback_a_values: vec![],
                scoring: ScoringWeights::default(),
            }),
        ),
        (
            "medium-grid",
            Some(PositionSearchConfig {
                speed_values: linspace(0.9, 4.2, 15),
                b_values: linspace(-0.85, 0.85, 13),
                a_values: linspace(-0.5, 0.5, 5),
                fallback_a_values: vec![],
                scoring: ScoringWeights::default(),
            }),
        ),
        (
            "dense-grid",
            Some(PositionSearchConfig {
                speed_values: linspace(0.9, 4.2, 18),
                b_values: linspace(-0.85, 0.85, 17),
                a_values: linspace(-0.5, 0.5, 5),
                fallback_a_values: vec![],
                scoring: ScoringWeights::default(),
            }),
        ),
    ];

    for (name, config) in experiments {
        let started = Instant::now();
        let report = corpus_recall(config.as_ref());
        println!(
            "{name}: {}/{} approaches; {} missing; {} cells; {:.2?}",
            report.found,
            report.expected,
            report.missing.len(),
            report.evaluated,
            started.elapsed(),
        );
        for missing in report.missing {
            println!("  {missing}");
        }
    }
}

#[test]
#[ignore = "expensive corpus side-spin experiment"]
fn compare_dense_side_spin_recall() {
    let experiments = [
        ("dense-three-wide", vec![-0.5, 0.0, 0.5]),
        ("dense-three-inner", vec![-0.25, 0.0, 0.25]),
        ("dense-four-negative", vec![-0.5, -0.25, 0.0, 0.25]),
        ("dense-four-positive", vec![-0.25, 0.0, 0.25, 0.5]),
        ("dense-five", linspace(-0.5, 0.5, 5)),
    ];

    for (name, a_values) in experiments {
        let config = PositionSearchConfig {
            speed_values: linspace(0.9, 4.2, 18),
            b_values: linspace(-0.85, 0.85, 17),
            a_values,
            fallback_a_values: vec![],
            scoring: ScoringWeights::default(),
        };
        let started = Instant::now();
        let report = corpus_recall(Some(&config));
        println!(
            "{name}: {}/{} approaches; {} missing; {} cells; {:.2?}",
            report.found,
            report.expected,
            report.missing.len(),
            report.evaluated,
            started.elapsed(),
        );
        for missing in report.missing {
            println!("  {missing}");
        }
    }
}

#[test]
#[ignore = "elongated-zone quality experiment"]
fn inspect_elongated_target_quality() {
    for fixture in elongated_target_fixtures() {
        let suggestion = suggest_position_shot(
            &fixture.scenario,
            fixture.target_ball_id,
            fixture.pocket,
            &fixture.target_area,
            None,
        )
        .unwrap_or_else(|error| panic!("{} search failed: {error}", fixture.name));
        assert!(
            suggestion.achievable,
            "{} should be achievable",
            fixture.name
        );
        let candidates: Vec<_> = suggestion
            .best
            .iter()
            .chain(&suggestion.alternates)
            .collect();
        assert!(
            candidates.len() >= 2,
            "{} should expose competing approaches",
            fixture.name
        );
        println!(
            "{}: {} candidates; {} cells",
            fixture.name,
            candidates.len(),
            suggestion.evaluated_count
        );
        for (index, candidate) in candidates.into_iter().enumerate() {
            println!(
                "  {index}: route={:?} speed={:.3} a={:.3} b={:.3} robustness={} dwell={:.4} score={:.4} dwell_score={:.4} window_score={:.4}",
                candidate_route(candidate, &fixture.scenario.table),
                candidate.speed,
                candidate.a,
                candidate.b,
                candidate.robustness,
                candidate.dwell,
                candidate.score,
                candidate.score_breakdown.dwell,
                candidate.score_breakdown.speed_window,
            );
        }
    }
}

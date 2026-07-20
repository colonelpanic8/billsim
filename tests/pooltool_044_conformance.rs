//! Differential fixtures generated with Pooltool 0.4.4, Railbird's pinned engine line.

use billsim::math::Vec2;
use billsim::{
    Ball, CueStrike, SimulationEventType, SimulationOptions, SimulationScenario, TableSpec,
    simulate,
};

fn run(cue: Vec2, object: Vec2, phi: f64) -> billsim::ShotProjection {
    simulate(
        &SimulationScenario {
            balls: vec![
                Ball {
                    id: 0,
                    position: cue,
                },
                Ball {
                    id: 1,
                    position: object,
                },
            ],
            cue_ball_id: 0,
            strike: CueStrike::new(1.0, phi),
            table: TableSpec::default(),
        },
        SimulationOptions::default(),
    )
    .unwrap()
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.17} differs from Pooltool fixture {expected:.17}"
    );
}

fn event_times(projection: &billsim::ShotProjection, kind: SimulationEventType) -> Vec<f64> {
    projection
        .events
        .iter()
        .filter(|event| event.event_type == kind)
        .map(|event| event.time)
        .collect()
}

#[test]
fn long_rail_fixture_matches_pooltool_044() {
    let projection = run(Vec2::new(0.5, 0.5), Vec2::new(0.8, 2.0), 0.0);
    let cushions = event_times(&projection, SimulationEventType::BallCushion);
    assert_eq!(cushions.len(), 2);
    assert_close(cushions[0], 0.637_517_535_271_306_5, 2e-9);
    assert_close(cushions[1], 3.530_672_678_492_564, 2e-8);

    let final_ball = &projection.final_state[0];
    assert_close(final_ball.position.x, 0.127_714_574_552_051_93, 2e-8);
    assert_close(final_ball.position.y, 0.499_511_769_047_616_96, 2e-8);
}

#[test]
fn corner_pocket_fixture_matches_pooltool_044() {
    let projection = run(Vec2::new(0.3, 0.3), Vec2::new(0.8, 2.0), 225.0);
    let pockets = event_times(&projection, SimulationEventType::BallPocket);
    assert_eq!(pockets.len(), 1);
    assert_close(pockets[0], 0.321_493_172_641_951, 2e-9);
    assert_eq!(projection.potted_ball_ids, vec![0]);

    let final_ball = &projection.final_state[0];
    assert_close(final_ball.position.x, -0.028_142_849_891_224_59, 1e-12);
    assert_close(final_ball.position.y, -0.028_142_849_891_224_59, 1e-12);
}

#[test]
fn ball_ball_fixture_matches_pooltool_044() {
    let projection = run(Vec2::new(0.3, 0.5), Vec2::new(0.8, 0.5), 0.0);
    let ball_ball = event_times(&projection, SimulationEventType::BallBall);
    assert_eq!(ball_ball.len(), 2);
    assert_close(ball_ball[0], 0.358_997_283_252_752_2, 2e-9);
    assert_close(ball_ball[1], 1.452_138_832_386_827_6, 2e-8);

    let cue = projection
        .final_state
        .iter()
        .find(|ball| ball.ball_id == 0)
        .unwrap();
    let object = projection
        .final_state
        .iter()
        .find(|ball| ball.ball_id == 1)
        .unwrap();
    assert_close(cue.position.x, 0.886_571_571_892_022_9, 3e-8);
    assert_close(cue.position.y, 0.500_095_086_609_581_5, 3e-8);
    assert_close(object.position.x, 1.088_638_037_350_454_5, 3e-8);
    assert_close(object.position.y, 0.499_922_279_909_496_6, 3e-8);
}

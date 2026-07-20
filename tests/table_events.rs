use billsim::math::Vec2;
use billsim::{
    Ball, CueStrike, MotionState, PocketId, SimulationEventType, SimulationOptions,
    SimulationScenario, TableSpec, simulate,
};

fn one_ball(position: Vec2, strike: CueStrike) -> SimulationScenario {
    SimulationScenario {
        balls: vec![Ball { id: 0, position }],
        cue_ball_id: 0,
        strike,
        table: TableSpec::default(),
    }
}

#[test]
fn ball_rebounds_from_a_long_rail() {
    let projection = simulate(
        &one_ball(Vec2::new(0.5, 0.5), CueStrike::new(1.0, 0.0)),
        SimulationOptions::default(),
    )
    .expect("rail shot should simulate to rest");

    let cushion_event = projection
        .events
        .iter()
        .find(|event| event.event_type == SimulationEventType::BallCushion)
        .expect("shot should reach the right long rail");
    assert_eq!(cushion_event.cushion.as_deref(), Some("15"));
    assert_eq!(
        projection.final_state[0].motion_state,
        MotionState::Stationary
    );
    assert!(projection.potted_ball_ids.is_empty());
}

#[test]
fn ball_can_be_potted_in_named_corner() {
    let projection = simulate(
        &one_ball(Vec2::new(0.3, 0.3), CueStrike::new(1.0, 225.0)),
        SimulationOptions::default(),
    )
    .expect("corner-pocket shot should simulate");

    let pocket_event = projection
        .events
        .iter()
        .find(|event| event.event_type == SimulationEventType::BallPocket)
        .expect("shot should enter the corner pocket");
    assert_eq!(pocket_event.pocket, Some(PocketId::LeftBottom));
    assert_eq!(
        projection.final_state[0].motion_state,
        MotionState::Pocketed
    );
    assert_eq!(projection.potted_ball_ids, vec![0]);
}

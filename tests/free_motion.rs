use billsim::math::Vec2;
use billsim::{
    Ball, CueStrike, MotionState, SimulationError, SimulationEventType, SimulationOptions,
    SimulationScenario, TableSpec, simulate,
};

fn scenario(strike: CueStrike) -> SimulationScenario {
    SimulationScenario {
        balls: vec![Ball {
            id: 0,
            position: Vec2::new(0.5, 0.5),
        }],
        cue_ball_id: 0,
        strike,
        table: TableSpec {
            length: 200.0,
            width: 200.0,
            ..TableSpec::default()
        },
    }
}

#[test]
fn free_motion_is_an_end_to_end_projection() {
    let projection = simulate(
        &scenario(CueStrike::new(2.0, 0.0)),
        SimulationOptions::default(),
    )
    .expect("free-motion simulation should succeed");

    assert_eq!(projection.trajectories.len(), 1);
    assert!(projection.trajectories[0].points.len() > 100);
    assert_eq!(
        projection.events[0].event_type,
        SimulationEventType::StickBall
    );
    assert_eq!(
        projection.events.last().unwrap().event_type,
        SimulationEventType::RollingStationary
    );
    assert_eq!(
        projection.final_state[0].motion_state,
        MotionState::Stationary
    );
    assert!(projection.final_state[0].position.x > 0.5);
    assert!(projection.potted_ball_ids.is_empty());

    let final_point = projection.trajectories[0].points.last().unwrap();
    assert_eq!(final_point.motion_state, MotionState::Stationary);
    assert!((final_point.position.x - projection.final_state[0].position.x).abs() < 1e-12);
}

#[test]
fn draw_and_follow_produce_different_distances() {
    let follow = simulate(
        &scenario(CueStrike {
            speed: 2.0,
            phi: 0.0,
            theta: 0.0,
            a: 0.0,
            b: 0.6,
        }),
        SimulationOptions::default(),
    )
    .unwrap();
    let draw = simulate(
        &scenario(CueStrike {
            speed: 2.0,
            phi: 0.0,
            theta: 0.0,
            a: 0.0,
            b: -0.6,
        }),
        SimulationOptions::default(),
    )
    .unwrap();

    assert_ne!(follow.final_state[0].position, draw.final_state[0].position);
}

#[test]
fn straight_shot_transfers_motion_to_object_ball() {
    let mut scenario = scenario(CueStrike::new(2.0, 0.0));
    scenario.balls.push(Ball {
        id: 1,
        position: Vec2::new(1.0, 0.5),
    });

    let projection = simulate(&scenario, SimulationOptions::default()).unwrap();

    assert!(
        projection
            .events
            .iter()
            .any(|event| event.event_type == SimulationEventType::BallBall)
    );
    let object = projection
        .final_state
        .iter()
        .find(|ball| ball.ball_id == 1)
        .unwrap();
    assert!(object.position.x > 1.0);
    assert_eq!(object.motion_state, MotionState::Stationary);
}

#[test]
fn invalid_contact_offset_is_rejected() {
    let invalid = scenario(CueStrike {
        speed: 2.0,
        phi: 0.0,
        theta: 0.0,
        a: 0.8,
        b: 0.8,
    });
    assert_eq!(
        simulate(&invalid, SimulationOptions::default()),
        Err(SimulationError::InvalidValue("strike contact offset"))
    );
}

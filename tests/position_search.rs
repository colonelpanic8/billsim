//! Integration tests for the position-play search, in physics meters.
//!
//! Layout mirrors Railbird's Python suite: a cue ball behind an object
//! ball, roughly in line with the `RightTop` corner pocket of a 9-foot
//! table (2.54 x 1.27 m; x spans the short dimension).

use billsim::math::Vec2;
use billsim::{
    Ball, PocketId, PositionSearchConfig, SimulationEventType, SimulationScenario, TableSpec,
    suggest_position_shot,
};

fn scenario(balls: Vec<Ball>) -> SimulationScenario {
    SimulationScenario {
        balls,
        cue_ball_id: 0,
        strike: billsim::CueStrike {
            speed: 1.0,
            phi: 0.0,
            theta: 0.0,
            a: 0.0,
            b: 0.0,
        },
        table: TableSpec::default(),
    }
}

fn straight_pot_balls() -> Vec<Ball> {
    vec![
        Ball {
            id: 0,
            position: Vec2::new(0.4, 1.0),
        },
        Ball {
            id: 1,
            position: Vec2::new(0.6, 1.5),
        },
    ]
}

fn small_config() -> PositionSearchConfig {
    PositionSearchConfig {
        speed_values: vec![1.2, 1.7, 2.2, 2.8, 3.5],
        b_values: vec![-0.8, -0.4, 0.0, 0.4, 0.8],
        a_values: vec![0.0],
        fallback_a_values: vec![],
        scoring: billsim::ScoringWeights::default(),
    }
}

/// Up-table forward zone where rolling non-draw strikes leave the cue
/// ball for this layout (mapped empirically via `examples/landing_map.rs`;
/// draw pulls the cue back down-table, away from it).
fn forward_polygon() -> Vec<Vec2> {
    vec![
        Vec2::new(0.05, 2.00),
        Vec2::new(0.55, 2.00),
        Vec2::new(0.55, 2.52),
        Vec2::new(0.05, 2.52),
    ]
}

#[test]
fn degenerate_polygon_is_rejected() {
    let error = suggest_position_shot(
        &scenario(straight_pot_balls()),
        1,
        PocketId::RightTop,
        &[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)],
        Some(small_config()),
    )
    .unwrap_err();
    assert_eq!(error, billsim::PositionError::DegeneratePolygon);
}

#[test]
fn blocked_pot_short_circuits_without_sweeping() {
    // A blocker sits directly on the cue -> ghost-ball line, so the pot
    // is geometrically infeasible; the search must not sweep at all.
    let mut balls = straight_pot_balls();
    balls.push(Ball {
        id: 2,
        position: Vec2::new(0.5, 1.25),
    });
    let suggestion = suggest_position_shot(
        &scenario(balls),
        1,
        PocketId::RightTop,
        &forward_polygon(),
        Some(small_config()),
    )
    .expect("search should succeed");
    assert!(!suggestion.achievable);
    assert!(suggestion.best.is_none());
    assert!(suggestion.alternates.is_empty());
    assert_eq!(suggestion.evaluated_count, 0);
    assert_eq!(suggestion.successful_count, 0);
}

#[test]
fn forward_position_finds_clean_non_draw_leave() {
    let suggestion = suggest_position_shot(
        &scenario(straight_pot_balls()),
        1,
        PocketId::RightTop,
        &forward_polygon(),
        Some(small_config()),
    )
    .expect("search should succeed");
    assert!(suggestion.achievable);
    assert!(suggestion.successful_count > 0);
    assert_eq!(suggestion.evaluated_count, 25);
    let best = suggestion.best.expect("achievable search returns a best");
    // A forward leave is reached by follow or a rolling non-draw strike;
    // draw pulls the cue back off the contact and never lands here.
    assert!(best.b >= 0.0);
    assert!(best.in_target_area);
    assert!(best.potted);
    assert!(!best.scratched);
    // The b=0 column lands in-zone across the whole speed range here,
    // so the winner should carry a wide speed window.
    assert!(best.robustness >= 3);
    assert!(best.dwell >= 0.0);
    let projection = best.projection.expect("winner carries a projection");
    // Contact-clean: the shot's only ball-ball event is cue -> target.
    let contacts: Vec<_> = projection
        .events
        .iter()
        .filter(|event| event.event_type == SimulationEventType::BallBall)
        .collect();
    assert_eq!(contacts.len(), 1);
    let mut ids = contacts[0].ball_ids.clone();
    ids.sort_unstable();
    assert_eq!(ids, vec![0, 1]);
    // The target must drop and the cue must not.
    assert!(projection.potted_ball_ids.contains(&1));
    assert!(!projection.potted_ball_ids.contains(&0));
}

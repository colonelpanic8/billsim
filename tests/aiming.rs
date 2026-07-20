use billsim::math::Vec2;
use billsim::{Ball, CueStrike, PocketId, SimulationScenario, TableSpec, compute_pot_aim};

fn side_pocket_scenario(side_spin: f64) -> SimulationScenario {
    SimulationScenario {
        balls: vec![
            Ball {
                id: 0,
                position: Vec2::new(0.4, 1.27),
            },
            Ball {
                id: 1,
                position: Vec2::new(0.9, 1.27),
            },
        ],
        cue_ball_id: 0,
        strike: CueStrike {
            speed: 2.0,
            phi: 123.0,
            theta: 0.0,
            a: side_spin,
            b: 0.0,
        },
        table: TableSpec::default(),
    }
}

#[test]
fn straight_side_pot_is_solved_and_verified() {
    let aim = compute_pot_aim(&side_pocket_scenario(0.0), 1, PocketId::RightCenter).unwrap();

    assert!(aim.feasible);
    assert!(aim.converged);
    assert!(aim.potted);
    assert!(aim.occluding_ball_ids.is_empty());
    assert!(aim.cut_angle.abs() < 1e-10);
    assert!(aim.phi.abs() < 0.1 || (aim.phi - 360.0).abs() < 0.1);
}

#[test]
fn side_spin_changes_the_compensated_aim() {
    let center = compute_pot_aim(&side_pocket_scenario(0.0), 1, PocketId::RightCenter).unwrap();
    let english = compute_pot_aim(&side_pocket_scenario(0.2), 1, PocketId::RightCenter).unwrap();

    assert!(english.converged);
    assert!(english.potted);
    assert!((english.phi - center.phi).abs() > 0.05);
}

#[test]
fn reports_an_occluding_ball() {
    let mut scenario = side_pocket_scenario(0.0);
    scenario.balls.push(Ball {
        id: 2,
        position: Vec2::new(0.65, 1.27),
    });

    let aim = compute_pot_aim(&scenario, 1, PocketId::RightCenter).unwrap();

    assert!(!aim.feasible);
    assert_eq!(aim.occluding_ball_ids, vec![2]);
}

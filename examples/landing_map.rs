//! Dev aid: print cue-ball landing spots across a small sweep grid.
use billsim::math::Vec2;
use billsim::{
    Ball, CueStrike, PocketId, SimulationOptions, SimulationScenario, TableSpec,
    compute_pot_aim_seeded, simulate,
};

fn main() {
    let base = SimulationScenario {
        balls: vec![
            Ball {
                id: 0,
                position: Vec2::new(0.4, 1.0),
            },
            Ball {
                id: 1,
                position: Vec2::new(0.6, 1.5),
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
    };
    for speed in [1.2, 1.7, 2.2, 2.8, 3.5] {
        for b in [-0.8, -0.4, 0.0, 0.4, 0.8] {
            let mut scenario = base.clone();
            scenario.strike.speed = speed;
            scenario.strike.b = b;
            let aim = compute_pot_aim_seeded(&scenario, 1, PocketId::RightTop, None).unwrap();
            scenario.strike.phi = aim.phi;
            let projection = simulate(&scenario, SimulationOptions::default()).unwrap();
            let cue = projection
                .final_state
                .iter()
                .find(|s| s.ball_id == 0)
                .unwrap();
            println!(
                "speed={speed} b={b:+.1} potted={} scratch={} cue=({:.2},{:.2})",
                projection.potted_ball_ids.contains(&1),
                projection.potted_ball_ids.contains(&0),
                cue.position.x,
                cue.position.y,
            );
        }
    }
}

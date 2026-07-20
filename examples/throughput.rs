use std::hint::black_box;
use std::time::Instant;

use billsim::math::Vec2;
use billsim::{Ball, CueStrike, SimulationOptions, SimulationScenario, TableSpec, simulate};

fn main() {
    let iterations = std::env::args().nth(1).map_or(10_000, |value| {
        value
            .parse::<u32>()
            .expect("iteration count must be an integer")
    });
    let scenario = SimulationScenario {
        balls: vec![
            Ball {
                id: 0,
                position: Vec2::new(0.3, 0.5),
            },
            Ball {
                id: 1,
                position: Vec2::new(0.8, 0.5),
            },
        ],
        cue_ball_id: 0,
        strike: CueStrike::new(1.0, 0.0),
        table: TableSpec::default(),
    };
    let options = SimulationOptions {
        trajectory_dt: 0.01,
        ..SimulationOptions::default()
    };

    let started = Instant::now();
    for _ in 0..iterations {
        let projection = simulate(black_box(&scenario), options).expect("benchmark shot failed");
        black_box(projection);
    }
    let elapsed = started.elapsed();
    let per_second = f64::from(iterations) / elapsed.as_secs_f64();
    println!("{iterations} shots in {elapsed:.3?}: {per_second:.0} shots/s");
}

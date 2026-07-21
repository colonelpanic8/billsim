//! Position-play suggestion: pot a ball AND leave the cue ball somewhere.
//!
//! Port of Railbird's `railbird.shot_simulation.position` search. Given a
//! scenario, a required pot (target ball + pocket), and a target area
//! (polygon in physics-frame meters) for where the cue ball should come to
//! rest, suggest cue strike parameters (speed, spin, and the solved phi)
//! that pot the ball and leave the cue ball inside the polygon.
//!
//! The search sweeps a (speed, side-spin, follow/draw) grid. At each grid
//! point the aim phi is solved with the throw/squirt-compensated pot solver
//! and the solved strike is forward-simulated once. Successful grid points
//! (potted in the requested pocket, no scratch, cue ball inside the
//! polygon, and the shot's only ball-ball contact being the single
//! cue -> target hit — position play must rely on the cushions, never on
//! secondary contact) are ranked by their *speed window*: human execution
//! error is dominated by speed error, and a speed error moves the landing
//! point along the cue ball's final path, so each success is ranked by how
//! many contiguous speed steps (same spin) also succeed. Ties break on
//! *dwell* — the in-polygon length of the cue ball's final straight
//! approach — then on depth inside the polygon.

use std::collections::{BTreeSet, VecDeque};

use serde::{Deserialize, Serialize};

use crate::aiming::{AimError, compute_pot_aim_seeded, geometric_pot_feasibility};
use crate::math::Vec2;
use crate::model::{
    BallId, PocketId, ShotProjection, SimulationEventType, SimulationOptions, SimulationScenario,
};
use crate::simulation::{SimulationError, simulate};

/// Grid swept by [`suggest_position_shot`].
///
/// `a_values` is the primary side-spin sweep (side spin off by default: it
/// complicates execution for a human without usually helping position).
/// `fallback_a_values` are additional side-spin slices swept only when the
/// primary sweep pots but nothing reaches the polygon.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct PositionSearchConfig {
    pub speed_values: Vec<f64>,
    pub b_values: Vec<f64>,
    pub a_values: Vec<f64>,
    pub fallback_a_values: Vec<f64>,
    pub scoring: ScoringWeights,
}

/// Weights for ranking successful leaves. The score is benefits minus
/// penalties over normalized terms; hard requirements (potted, contact
/// clean, no scratch, inside the polygon) are filters, not weights.
///
/// Rationale: human execution error is dominated by speed, so the speed
/// window (contiguous speed steps that also succeed) and terminal dwell
/// (in-area length of the arrival path — how long the stop position stays
/// viable) are the primary benefits. Everything that makes a shot harder
/// to execute — hitting harder, longer cue travel, spin (side spin
/// costing most, then draw, then follow), and every cushion the cue
/// takes (mildly — cushion routes are normal position play) — counts
/// against it.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct ScoringWeights {
    pub speed_window: f64,
    pub dwell: f64,
    pub depth: f64,
    pub speed: f64,
    pub travel: f64,
    pub side_spin: f64,
    pub follow_spin: f64,
    pub draw_spin: f64,
    pub cushion: f64,
}

impl Default for ScoringWeights {
    fn default() -> Self {
        Self {
            speed_window: 3.0,
            dwell: 1.2,
            depth: 0.4,
            speed: 0.8,
            travel: 0.7,
            side_spin: 0.7,
            follow_spin: 0.3,
            draw_spin: 0.55,
            cushion: 0.25,
        }
    }
}

fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count < 2 {
        return vec![start];
    }
    #[allow(clippy::cast_precision_loss)]
    let step = (end - start) / (count - 1) as f64;
    #[allow(clippy::cast_precision_loss)]
    (0..count)
        .map(|index| start + step * index as f64)
        .collect()
}

impl Default for PositionSearchConfig {
    fn default() -> Self {
        Self {
            speed_values: linspace(0.9, 4.2, 12),
            b_values: linspace(-0.85, 0.85, 9),
            a_values: vec![0.0],
            fallback_a_values: vec![-0.5, 0.5],
            scoring: ScoringWeights::default(),
        }
    }
}

/// One evaluated strike, with the outcome of its forward simulation.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PositionShotCandidate {
    /// Strike speed in m/s.
    pub speed: f64,
    /// Solved aim, physics-frame degrees.
    pub phi: f64,
    pub theta: f64,
    pub a: f64,
    pub b: f64,
    pub cue_final_position: Vec2,
    pub in_target_area: bool,
    /// 0.0 when inside the polygon, else meters from the cue final
    /// position to the polygon boundary.
    pub distance_to_target_area: f64,
    /// Contiguous speed steps at the same spin that also succeed — the
    /// shot's speed tolerance. Includes this cell, so >= 1 for any
    /// successful candidate.
    pub robustness: u32,
    /// In-polygon length (meters) of the cue ball's terminal approach —
    /// the contiguous tail of its path that ends at rest inside the area.
    pub dwell: f64,
    /// Heuristic desirability of this leave (benefits minus penalties;
    /// see [`ScoringWeights`]). Comparable only within one search.
    pub score: f64,
    /// Total distance (meters) the cue ball travels before resting.
    pub cue_travel_distance: f64,
    /// Cushions the cue ball contacts on its way to rest.
    pub cue_cushion_count: u32,
    /// Target ball potted in the requested pocket.
    pub potted: bool,
    /// Cue ball pocketed.
    pub scratched: bool,
    /// Full projection; only populated for returned candidates.
    pub projection: Option<ShotProjection>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PositionSuggestion {
    pub achievable: bool,
    /// `None` only if no grid point pots cleanly.
    pub best: Option<PositionShotCandidate>,
    /// Up to 2, from distinct successful regions.
    pub alternates: Vec<PositionShotCandidate>,
    pub evaluated_count: u32,
    pub successful_count: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PositionError {
    DegeneratePolygon,
    Aim(AimError),
    Simulation(SimulationError),
}

impl std::fmt::Display for PositionError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DegeneratePolygon => {
                formatter.write_str("target_area polygon requires at least 3 vertices")
            }
            Self::Aim(error) => write!(formatter, "aim solve failed: {error}"),
            Self::Simulation(error) => write!(formatter, "sweep simulation failed: {error}"),
        }
    }
}

impl std::error::Error for PositionError {}

impl From<AimError> for PositionError {
    fn from(value: AimError) -> Self {
        Self::Aim(value)
    }
}

impl From<SimulationError> for PositionError {
    fn from(value: SimulationError) -> Self {
        Self::Simulation(value)
    }
}

/// Ray-cast point-in-polygon test (boundary points may go either way).
#[must_use]
pub fn point_in_polygon(point: Vec2, polygon: &[Vec2]) -> bool {
    let mut inside = false;
    let count = polygon.len();
    for index in 0..count {
        let a = polygon[index];
        let b = polygon[(index + 1) % count];
        if (a.y > point.y) != (b.y > point.y) {
            let x_cross = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if point.x < x_cross {
                inside = !inside;
            }
        }
    }
    inside
}

/// Minimum distance from a point to the polygon's boundary.
#[must_use]
pub fn distance_to_polygon_boundary(point: Vec2, polygon: &[Vec2]) -> f64 {
    let count = polygon.len();
    (0..count)
        .map(|index| point_segment_distance(point, polygon[index], polygon[(index + 1) % count]))
        .fold(f64::INFINITY, f64::min)
}

fn point_segment_distance(point: Vec2, start: Vec2, end: Vec2) -> f64 {
    let segment = end - start;
    let length_sq = segment.x * segment.x + segment.y * segment.y;
    if length_sq == 0.0 {
        return (point - start).norm();
    }
    let along = (point - start).x * segment.x + (point - start).y * segment.y;
    let t = (along / length_sq).clamp(0.0, 1.0);
    (point - Vec2::new(start.x + t * segment.x, start.y + t * segment.y)).norm()
}

const DWELL_SAMPLES: u32 = 64;

/// Approximate length of the `start` -> `end` segment inside `polygon` via
/// deterministic midpoint sampling.
#[must_use]
pub fn segment_dwell(start: Vec2, end: Vec2, polygon: &[Vec2]) -> f64 {
    let length = (end - start).norm();
    if length == 0.0 {
        return 0.0;
    }
    let mut inside = 0u32;
    for index in 0..DWELL_SAMPLES {
        let t = (f64::from(index) + 0.5) / f64::from(DWELL_SAMPLES);
        let sample = Vec2::new(
            start.x + t * (end.x - start.x),
            start.y + t * (end.y - start.y),
        );
        if point_in_polygon(sample, polygon) {
            inside += 1;
        }
    }
    length * f64::from(inside) / f64::from(DWELL_SAMPLES)
}

// The outcome flags are the domain shape of a swept cell, not a state
// machine to be refactored away.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Debug)]
struct Cell {
    speed: f64,
    phi: f64,
    theta: f64,
    a: f64,
    b: f64,
    cue_final_position: Vec2,
    potted: bool,
    scratched: bool,
    in_target_area: bool,
    boundary_distance: f64,
    dwell: f64,
    travel: f64,
    cushions: u32,
    /// Any ball-ball contact beyond the single cue -> target hit.
    secondary_contact: bool,
}

impl Cell {
    fn successful(&self) -> bool {
        self.potted && !self.scratched && self.in_target_area && !self.secondary_contact
    }

    fn distance_to_target_area(&self) -> f64 {
        if self.in_target_area {
            0.0
        } else {
            self.boundary_distance
        }
    }
}

type GridKey = (usize, usize);

/// Search for strikes that pot `target_ball_id` and leave the cue ball
/// inside `target_area` (physics-frame meters).
///
/// The scenario's `strike` is ignored except for `theta`; the search
/// sweeps its own (speed, a, b) grid and solves phi per grid point.
///
/// # Errors
///
/// Returns [`PositionError`] for a degenerate polygon or a failed aim
/// solve / simulation inside the sweep.
#[allow(clippy::too_many_lines)] // The sweep/rank/select pipeline reads best unsplit.
pub fn suggest_position_shot(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
    target_area: &[Vec2],
    config: Option<PositionSearchConfig>,
) -> Result<PositionSuggestion, PositionError> {
    if target_area.len() < 3 {
        return Err(PositionError::DegeneratePolygon);
    }
    let config = config.unwrap_or_default();

    // A pot that is geometrically off (path blocked, or cut beyond the
    // playable limit) can never produce a successful cell, and sweeping it
    // anyway hits the aim solver's worst case on every cell. Bail out
    // before simulating anything.
    let feasibility = geometric_pot_feasibility(scenario, target_ball_id, pocket_id)?;
    if !feasibility.feasible {
        return Ok(PositionSuggestion {
            achievable: false,
            best: None,
            alternates: Vec::new(),
            evaluated_count: 0,
            successful_count: 0,
        });
    }

    let mut slices: Vec<(f64, Vec<Vec<Cell>>)> = Vec::new();
    for &a in &config.a_values {
        let cells = evaluate_slice(scenario, target_ball_id, pocket_id, target_area, a, &config)?;
        slices.push((a, cells));
    }

    let any_success = |slices: &[(f64, Vec<Vec<Cell>>)]| {
        slices
            .iter()
            .flat_map(|(_, grid)| grid.iter().flatten())
            .any(Cell::successful)
    };
    let any_pot = |slices: &[(f64, Vec<Vec<Cell>>)]| {
        slices
            .iter()
            .flat_map(|(_, grid)| grid.iter().flatten())
            .any(|cell| cell.potted)
    };

    // Side-spin fallback only helps when the pot itself goes in but no
    // primary-spin leave reaches the polygon.
    if !any_success(&slices) && any_pot(&slices) {
        for &a in &config.fallback_a_values {
            if config.a_values.contains(&a) {
                continue;
            }
            let cells =
                evaluate_slice(scenario, target_ball_id, pocket_id, target_area, a, &config)?;
            slices.push((a, cells));
        }
    }

    let evaluated_count = slices
        .iter()
        .map(|(_, grid)| grid.iter().map(Vec::len).sum::<usize>())
        .sum::<usize>();
    let successful: Vec<(usize, GridKey)> = slices
        .iter()
        .enumerate()
        .flat_map(|(slice_index, (_, grid))| {
            grid.iter().enumerate().flat_map(move |(v, row)| {
                row.iter()
                    .enumerate()
                    .filter_map(move |(b, cell)| cell.successful().then_some((slice_index, (v, b))))
            })
        })
        .collect();

    if successful.is_empty() {
        return Ok(failure_suggestion(
            scenario,
            &slices,
            u32::try_from(evaluated_count).unwrap_or(u32::MAX),
        ));
    }

    // Rank every successful cell by its heuristic leave score (see
    // ScoringWeights); the speed window feeds the score as its dominant
    // benefit. Ties break on (window, dwell) for determinism.
    let scoring_context = ScoringContext::new(&config, scenario.table.length);
    let rank = |slice_index: usize, key: GridKey| -> (f64, u32, f64) {
        let grid = &slices[slice_index].1;
        let cell = &grid[key.0][key.1];
        let window = speed_window(grid, key);
        (scoring_context.score(cell, window), window, cell.dwell)
    };
    let better = |lhs: (f64, u32, f64), rhs: (f64, u32, f64)| -> bool {
        lhs.partial_cmp(&rhs) == Some(std::cmp::Ordering::Greater)
    };

    let mut winner = successful[0];
    let mut winner_rank = rank(winner.0, winner.1);
    for &(slice_index, key) in &successful[1..] {
        let candidate_rank = rank(slice_index, key);
        if better(candidate_rank, winner_rank) {
            winner = (slice_index, key);
            winner_rank = candidate_rank;
        }
    }

    // Alternates come from distinct connected components of the success
    // region (per slice, 8-connectivity), the winner's excluded, so they
    // represent genuinely different ways to play the position.
    let mut alternate_entries: Vec<((f64, u32, f64), usize, GridKey)> = Vec::new();
    for (slice_index, (_, grid)) in slices.iter().enumerate() {
        for component in successful_components(grid) {
            if slice_index == winner.0 && component.contains(&winner.1) {
                continue;
            }
            if let Some(&best_key) = component.iter().max_by(|&&lhs, &&rhs| {
                let lhs_rank = rank(slice_index, lhs);
                let rhs_rank = rank(slice_index, rhs);
                lhs_rank
                    .partial_cmp(&rhs_rank)
                    .unwrap_or(std::cmp::Ordering::Equal)
            }) {
                alternate_entries.push((rank(slice_index, best_key), slice_index, best_key));
            }
        }
    }
    alternate_entries.sort_by(|lhs, rhs| {
        rhs.0
            .partial_cmp(&lhs.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let successful_count = u32::try_from(successful.len()).unwrap_or(u32::MAX);
    let realize =
        |slice_index: usize, key: GridKey| -> Result<PositionShotCandidate, PositionError> {
            let grid = &slices[slice_index].1;
            let cell = &grid[key.0][key.1];
            let window = speed_window(grid, key);
            Ok(to_candidate(
                cell,
                window,
                scoring_context.score(cell, window),
                Some(project(scenario, cell)?),
            ))
        };

    let best = realize(winner.0, winner.1)?;
    let mut alternates = Vec::new();
    for &(_, slice_index, key) in alternate_entries.iter().take(2) {
        alternates.push(realize(slice_index, key)?);
    }

    Ok(PositionSuggestion {
        achievable: true,
        best: Some(best),
        alternates,
        evaluated_count: u32::try_from(evaluated_count).unwrap_or(u32::MAX),
        successful_count,
    })
}

/// Solve + forward-simulate every (speed, b) grid point at side spin `a`.
///
/// The grid is walked serpentine (b reversed on alternate speed rows) so
/// consecutive grid points are physical neighbors, letting each aim solve
/// warm-start from the previous solved phi.
fn evaluate_slice(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
    polygon: &[Vec2],
    a: f64,
    config: &PositionSearchConfig,
) -> Result<Vec<Vec<Cell>>, PositionError> {
    let cue_start = scenario
        .balls
        .iter()
        .find(|ball| ball.id == scenario.cue_ball_id)
        .map(|ball| ball.position)
        .unwrap_or_default();
    let mut grid: Vec<Vec<Option<Cell>>> =
        vec![vec![None; config.b_values.len()]; config.speed_values.len()];
    let mut seed_phi: Option<f64> = None;
    for (speed_index, &speed) in config.speed_values.iter().enumerate() {
        let b_indices: Vec<usize> = if speed_index % 2 == 1 {
            (0..config.b_values.len()).rev().collect()
        } else {
            (0..config.b_values.len()).collect()
        };
        for b_index in b_indices {
            let b = config.b_values[b_index];
            let mut swept = scenario.clone();
            swept.strike.speed = speed;
            swept.strike.a = a;
            swept.strike.b = b;
            let aim = compute_pot_aim_seeded(&swept, target_ball_id, pocket_id, seed_phi)?;
            if aim.potted {
                swept.strike.phi = aim.phi;
                grid[speed_index][b_index] = Some(forward_simulate(
                    &swept,
                    target_ball_id,
                    pocket_id,
                    polygon,
                    a,
                )?);
                // Only propagate seeds that actually solved; a garbage
                // best-effort phi would poison neighboring solves.
                seed_phi = Some(aim.phi);
            } else {
                // The aim solver's verification probe already simulated
                // exactly this strike and it did not pot; the sim is
                // deterministic, so re-running it cannot succeed.
                grid[speed_index][b_index] = Some(Cell {
                    speed,
                    phi: aim.phi,
                    theta: scenario.strike.theta,
                    a,
                    b,
                    cue_final_position: cue_start,
                    potted: false,
                    scratched: false,
                    in_target_area: false,
                    boundary_distance: distance_to_polygon_boundary(cue_start, polygon),
                    dwell: 0.0,
                    travel: 0.0,
                    cushions: 0,
                    secondary_contact: false,
                });
            }
        }
    }
    Ok(grid
        .into_iter()
        .map(|row| {
            row.into_iter()
                .map(|cell| cell.expect("cell filled"))
                .collect()
        })
        .collect())
}

/// Trajectory sampling is skipped during the sweep; only returned
/// candidates get a dense projection.
fn sweep_options() -> SimulationOptions {
    SimulationOptions {
        trajectory_dt: f64::MAX,
        ..SimulationOptions::default()
    }
}

fn forward_simulate(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
    polygon: &[Vec2],
    a: f64,
) -> Result<Cell, PositionError> {
    let projection = simulate(scenario, sweep_options())?;
    let cue_final = projection
        .final_state
        .iter()
        .find(|state| state.ball_id == scenario.cue_ball_id)
        .map(|state| state.position)
        .unwrap_or_default();
    let in_target_area = point_in_polygon(cue_final, polygon);
    let path = cue_path(&projection, scenario, cue_final);
    let travel = path
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum::<f64>();
    let cushions = u32::try_from(
        projection
            .events
            .iter()
            .filter(|event| {
                event.event_type == SimulationEventType::BallCushion
                    && event.ball_ids.contains(&scenario.cue_ball_id)
            })
            .count(),
    )
    .unwrap_or(u32::MAX);
    Ok(Cell {
        speed: scenario.strike.speed,
        phi: scenario.strike.phi,
        theta: scenario.strike.theta,
        a,
        b: scenario.strike.b,
        cue_final_position: cue_final,
        potted: potted_in_pocket(&projection, target_ball_id, pocket_id),
        scratched: ball_pocketed(&projection, scenario.cue_ball_id),
        in_target_area,
        boundary_distance: distance_to_polygon_boundary(cue_final, polygon),
        dwell: terminal_dwell(&path, polygon),
        travel,
        cushions,
        secondary_contact: !sole_contact_is_cue_target(
            &projection,
            scenario.cue_ball_id,
            target_ball_id,
        ),
    })
}

/// The cue ball's path as a polyline: its start position, its position at
/// every event it participates in, and its rest position. Segments
/// between events are treated as straight (curved sliding is slightly
/// under-measured, which is fine for scoring).
fn cue_path(
    projection: &ShotProjection,
    scenario: &SimulationScenario,
    cue_final: Vec2,
) -> Vec<Vec2> {
    let mut path: Vec<Vec2> = Vec::new();
    if let Some(start) = scenario
        .balls
        .iter()
        .find(|ball| ball.id == scenario.cue_ball_id)
    {
        path.push(start.position);
    }
    for event in &projection.events {
        if event.ball_ids.contains(&scenario.cue_ball_id)
            && let Some(position) = event.position
            && path
                .last()
                .is_none_or(|last| (*last - position).norm() > 1e-9)
        {
            path.push(position);
        }
    }
    if path
        .last()
        .is_none_or(|last| (*last - cue_final).norm() > 1e-9)
    {
        path.push(cue_final);
    }
    path
}

/// In-polygon length of the path's terminal tail: walking backward from
/// rest, accumulate arc length while samples stay inside the polygon and
/// stop at the first sample outside. This is the stretch over which a
/// speed error still leaves the cue ball in the target area.
fn terminal_dwell(path: &[Vec2], polygon: &[Vec2]) -> f64 {
    let mut dwell = 0.0;
    'segments: for pair in path.windows(2).rev() {
        let (start, end) = (pair[0], pair[1]);
        let length = (end - start).norm();
        if length == 0.0 {
            continue;
        }
        let step = length / f64::from(DWELL_SAMPLES);
        // Sample from the rest-side end backward toward the segment start.
        for index in 0..DWELL_SAMPLES {
            let t = 1.0 - (f64::from(index) + 0.5) / f64::from(DWELL_SAMPLES);
            let sample = Vec2::new(
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y),
            );
            if point_in_polygon(sample, polygon) {
                dwell += step;
            } else {
                break 'segments;
            }
        }
    }
    dwell
}

fn potted_in_pocket(projection: &ShotProjection, ball_id: BallId, pocket_id: PocketId) -> bool {
    projection.events.iter().any(|event| {
        event.event_type == SimulationEventType::BallPocket
            && event.ball_ids.contains(&ball_id)
            && event.pocket == Some(pocket_id)
    })
}

fn ball_pocketed(projection: &ShotProjection, ball_id: BallId) -> bool {
    projection.events.iter().any(|event| {
        event.event_type == SimulationEventType::BallPocket && event.ball_ids.contains(&ball_id)
    })
}

/// The shot's only ball-ball contact is a single cue -> target hit.
///
/// Position play should rely on the cushions alone: a cue carom off a
/// third ball, the target brushing another ball en route to the pocket, a
/// double-kiss, or any downstream chain all make the outcome depend on
/// secondary contact, so exactly one ball-ball event — the potting contact
/// itself — is permitted.
fn sole_contact_is_cue_target(
    projection: &ShotProjection,
    cue_ball_id: BallId,
    target_ball_id: BallId,
) -> bool {
    let mut contact_count = 0u32;
    for event in &projection.events {
        if event.event_type != SimulationEventType::BallBall {
            continue;
        }
        contact_count += 1;
        if contact_count > 1 {
            return false;
        }
        let ids: BTreeSet<BallId> = event.ball_ids.iter().copied().collect();
        let expected: BTreeSet<BallId> = [cue_ball_id, target_ball_id].into_iter().collect();
        if ids != expected {
            return false;
        }
    }
    contact_count == 1
}

/// Length of the contiguous successful run along the speed axis through
/// `key`, counting `key` itself.
fn speed_window(grid: &[Vec<Cell>], key: GridKey) -> u32 {
    let (speed_index, b_index) = key;
    let mut count = 1u32;
    let mut up = speed_index + 1;
    while up < grid.len() && grid[up][b_index].successful() {
        count += 1;
        up += 1;
    }
    let mut down = speed_index;
    while down > 0 && grid[down - 1][b_index].successful() {
        count += 1;
        down -= 1;
    }
    count
}

/// Normalization context for scoring: saturation scales derived from the
/// sweep configuration and table so every term lands in roughly [0, 1]
/// before weighting.
struct ScoringContext {
    weights: ScoringWeights,
    max_speed: f64,
    max_window: f64,
    table_length: f64,
}

impl ScoringContext {
    fn new(config: &PositionSearchConfig, table_length: f64) -> Self {
        Self {
            weights: config.scoring.clone(),
            max_speed: config
                .speed_values
                .iter()
                .copied()
                .fold(f64::EPSILON, f64::max),
            #[allow(clippy::cast_precision_loss)]
            max_window: config.speed_values.len().max(1) as f64,
            table_length: table_length.max(f64::EPSILON),
        }
    }

    /// Heuristic desirability of a successful leave: benefits (speed
    /// window, terminal dwell, depth in the area) minus penalties (strike
    /// speed, cue travel, spin — side spin weighted heavier than
    /// follow/draw — and cushions). Saturations: dwell at half a table
    /// length, depth at 0.3 m, travel at two table lengths, cushions at 3.
    fn score(&self, cell: &Cell, window: u32) -> f64 {
        let window_n = (f64::from(window) - 1.0).max(0.0) / (self.max_window - 1.0).max(1.0);
        let dwell_n = (cell.dwell / (0.5 * self.table_length)).min(1.0);
        let depth_n = if cell.in_target_area {
            (cell.boundary_distance / 0.3).min(1.0)
        } else {
            0.0
        };
        let speed_n = (cell.speed / self.max_speed).min(1.0);
        let travel_n = (cell.travel / (2.0 * self.table_length)).min(1.0);
        let side_spin_n = (cell.a.abs() / MAX_SPIN_OFFSET).min(1.0);
        let vertical_spin_n = (cell.b.abs() / MAX_SPIN_OFFSET).min(1.0);
        // Draw is harder to execute than follow; each gets its own weight.
        let vertical_weight = if cell.b < 0.0 {
            self.weights.draw_spin
        } else {
            self.weights.follow_spin
        };
        let cushion_n = (f64::from(cell.cushions) / 3.0).min(1.0);
        let weights = &self.weights;
        weights.speed_window * window_n + weights.dwell * dwell_n + weights.depth * depth_n
            - weights.speed * speed_n
            - weights.travel * travel_n
            - weights.side_spin * side_spin_n
            - vertical_weight * vertical_spin_n
            - weights.cushion * cushion_n
    }
}

/// Deepest tip offset the default sweep uses; spin terms normalize
/// against it.
const MAX_SPIN_OFFSET: f64 = 0.85;

/// Connected components (8-connectivity) of successful grid cells.
fn successful_components(grid: &[Vec<Cell>]) -> Vec<BTreeSet<GridKey>> {
    let mut unvisited: BTreeSet<GridKey> = grid
        .iter()
        .enumerate()
        .flat_map(|(v, row)| {
            row.iter()
                .enumerate()
                .filter_map(move |(b, cell)| cell.successful().then_some((v, b)))
        })
        .collect();
    let mut components = Vec::new();
    while let Some(&start) = unvisited.iter().next() {
        unvisited.remove(&start);
        let mut component = BTreeSet::from([start]);
        let mut frontier = VecDeque::from([start]);
        while let Some((v, b)) = frontier.pop_front() {
            for dv in -1i64..=1 {
                for db in -1i64..=1 {
                    if dv == 0 && db == 0 {
                        continue;
                    }
                    let Ok(nv) = usize::try_from(i64::try_from(v).unwrap_or(i64::MAX) + dv) else {
                        continue;
                    };
                    let Ok(nb) = usize::try_from(i64::try_from(b).unwrap_or(i64::MAX) + db) else {
                        continue;
                    };
                    if unvisited.remove(&(nv, nb)) {
                        component.insert((nv, nb));
                        frontier.push_back((nv, nb));
                    }
                }
            }
        }
        components.push(component);
    }
    components
}

/// No grid point succeeded: report the closest clean pot, if any.
///
/// "Clean" holds the best-effort fallback to the same contact standard as
/// a real suggestion — pots with secondary ball contact are not offered
/// even as the consolation shot.
fn failure_suggestion(
    scenario: &SimulationScenario,
    slices: &[(f64, Vec<Vec<Cell>>)],
    evaluated_count: u32,
) -> PositionSuggestion {
    let closest = slices
        .iter()
        .flat_map(|(_, grid)| grid.iter().flatten())
        .filter(|cell| cell.potted && !cell.scratched && !cell.secondary_contact)
        .min_by(|lhs, rhs| {
            lhs.distance_to_target_area()
                .partial_cmp(&rhs.distance_to_target_area())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    let best = closest.map(|cell| to_candidate(cell, 0, 0.0, project(scenario, cell).ok()));
    PositionSuggestion {
        achievable: false,
        best,
        alternates: Vec::new(),
        evaluated_count,
        successful_count: 0,
    }
}

fn to_candidate(
    cell: &Cell,
    robustness: u32,
    score: f64,
    projection: Option<ShotProjection>,
) -> PositionShotCandidate {
    PositionShotCandidate {
        speed: cell.speed,
        phi: cell.phi,
        theta: cell.theta,
        a: cell.a,
        b: cell.b,
        cue_final_position: cell.cue_final_position,
        in_target_area: cell.in_target_area,
        distance_to_target_area: cell.distance_to_target_area(),
        robustness,
        dwell: cell.dwell,
        score,
        cue_travel_distance: cell.travel,
        cue_cushion_count: cell.cushions,
        potted: cell.potted,
        scratched: cell.scratched,
        projection,
    }
}

fn project(scenario: &SimulationScenario, cell: &Cell) -> Result<ShotProjection, PositionError> {
    let mut solved = scenario.clone();
    solved.strike.speed = cell.speed;
    solved.strike.phi = cell.phi;
    solved.strike.a = cell.a;
    solved.strike.b = cell.b;
    Ok(simulate(&solved, SimulationOptions::default())?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(successful: bool) -> Cell {
        Cell {
            speed: 1.0,
            phi: 0.0,
            theta: 0.0,
            a: 0.0,
            b: 0.0,
            cue_final_position: Vec2::new(0.0, 0.0),
            potted: successful,
            scratched: false,
            in_target_area: successful,
            boundary_distance: 1.0,
            dwell: 0.0,
            travel: 0.0,
            cushions: 0,
            secondary_contact: false,
        }
    }

    fn square() -> Vec<Vec2> {
        vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]
    }

    #[test]
    fn point_in_polygon_square() {
        assert!(point_in_polygon(Vec2::new(5.0, 5.0), &square()));
        assert!(!point_in_polygon(Vec2::new(15.0, 5.0), &square()));
        assert!(!point_in_polygon(Vec2::new(5.0, -1.0), &square()));
    }

    #[test]
    fn point_in_polygon_concave() {
        let polygon = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(7.0, 10.0),
            Vec2::new(7.0, 3.0),
            Vec2::new(3.0, 3.0),
            Vec2::new(3.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        assert!(!point_in_polygon(Vec2::new(5.0, 8.0), &polygon));
        assert!(point_in_polygon(Vec2::new(5.0, 1.5), &polygon));
        assert!(point_in_polygon(Vec2::new(1.5, 8.0), &polygon));
        assert!(point_in_polygon(Vec2::new(8.5, 8.0), &polygon));
    }

    #[test]
    fn boundary_distance_measures_depth_inside() {
        let distance = distance_to_polygon_boundary(Vec2::new(1.0, 5.0), &square());
        assert!((distance - 1.0).abs() < 1e-12);
    }

    #[test]
    fn segment_dwell_half_inside() {
        let dwell = segment_dwell(Vec2::new(5.0, 5.0), Vec2::new(15.0, 5.0), &square());
        assert!((dwell - 5.0).abs() <= 10.0 / 64.0);
    }

    #[test]
    fn segment_dwell_degenerate_is_zero() {
        let dwell = segment_dwell(Vec2::new(5.0, 5.0), Vec2::new(5.0, 5.0), &square());
        assert!(dwell.abs() < 1e-12);
    }

    #[test]
    fn speed_window_counts_contiguous_run() {
        // Rows are speed steps at one spin column; success at rows 1..=4.
        let grid: Vec<Vec<Cell>> = (0..6).map(|v| vec![cell((1..=4).contains(&v))]).collect();
        assert_eq!(speed_window(&grid, (2, 0)), 4);
        assert_eq!(speed_window(&grid, (1, 0)), 4);
        assert_eq!(speed_window(&grid, (4, 0)), 4);
    }

    #[test]
    fn speed_window_broken_by_failure() {
        let grid: Vec<Vec<Cell>> = (0..5).map(|v| vec![cell(v != 2)]).collect();
        assert_eq!(speed_window(&grid, (0, 0)), 2);
        assert_eq!(speed_window(&grid, (4, 0)), 2);
    }

    #[test]
    fn secondary_contact_disqualifies() {
        let mut dirty = cell(true);
        dirty.secondary_contact = true;
        assert!(!dirty.successful());
    }

    #[test]
    fn components_split_disconnected_regions() {
        // Two successes separated by a failure column are distinct
        // components under 8-connectivity.
        let mut grid: Vec<Vec<Cell>> = (0..1)
            .map(|_| (0..5).map(|_| cell(false)).collect())
            .collect();
        grid[0][0] = cell(true);
        grid[0][4] = cell(true);
        assert_eq!(successful_components(&grid).len(), 2);
    }

    fn scoring_context() -> ScoringContext {
        ScoringContext::new(&PositionSearchConfig::default(), 2.54)
    }

    #[test]
    fn score_rewards_wider_speed_window() {
        let context = scoring_context();
        let base = cell(true);
        assert!(context.score(&base, 4) > context.score(&base, 1));
    }

    #[test]
    fn score_penalizes_speed_travel_spin_and_cushions() {
        let context = scoring_context();
        let base = cell(true);

        let mut faster = base.clone();
        faster.speed = 4.0;
        assert!(context.score(&faster, 2) < context.score(&base, 2));

        let mut longer = base.clone();
        longer.travel = 4.0;
        assert!(context.score(&longer, 2) < context.score(&base, 2));

        let mut side_spun = base.clone();
        side_spun.a = 0.5;
        let mut followed = base.clone();
        followed.b = 0.5;
        let mut drawn = base.clone();
        drawn.b = -0.5;
        assert!(context.score(&side_spun, 2) < context.score(&base, 2));
        assert!(context.score(&followed, 2) < context.score(&base, 2));
        assert!(context.score(&drawn, 2) < context.score(&base, 2));
        // Execution difficulty ordering: side spin > draw > follow.
        assert!(context.score(&drawn, 2) < context.score(&followed, 2));
        assert!(context.score(&side_spun, 2) < context.score(&drawn, 2));

        let mut banked = base.clone();
        banked.cushions = 3;
        assert!(context.score(&banked, 2) < context.score(&base, 2));
    }

    #[test]
    fn score_rewards_dwell() {
        let context = scoring_context();
        let base = cell(true);
        let mut dwelling = base.clone();
        dwelling.dwell = 0.6;
        assert!(context.score(&dwelling, 2) > context.score(&base, 2));
    }

    #[test]
    fn terminal_dwell_stops_at_first_exit() {
        // Path passes through the square early, leaves, and re-enters to
        // rest inside: only the tail after the last entry counts.
        let square_polygon = square();
        let path = vec![
            Vec2::new(5.0, -10.0),
            Vec2::new(5.0, -1.0), // outside approach
            Vec2::new(5.0, 5.0),  // rest inside
        ];
        let dwell = terminal_dwell(&path, &square_polygon);
        // Inside portion of the last segment is y in [0, 5] => length 5.
        assert!((dwell - 5.0).abs() <= 6.0 / f64::from(DWELL_SAMPLES) * 4.0);
        // A path that never leaves keeps accumulating across segments.
        let staying = vec![
            Vec2::new(1.0, 5.0),
            Vec2::new(4.0, 5.0),
            Vec2::new(8.0, 5.0),
        ];
        let staying_dwell = terminal_dwell(&staying, &square_polygon);
        assert!((staying_dwell - 7.0).abs() <= 0.5);
    }
}

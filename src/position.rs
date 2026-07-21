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

use std::collections::BTreeSet;

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
    pub next_shot: f64,
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
            next_shot: 1.5,
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
    /// The per-factor contributions summing to `score`.
    pub score_breakdown: ScoreBreakdown,
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

/// The per-factor contributions to a leave's [`PositionShotCandidate::score`].
/// Benefits are non-negative, penalties non-positive; `total` is the sum
/// and equals the candidate's `score`. `vertical_spin` is the follow- or
/// draw-weighted contribution depending on the sign of the strike's `b`.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct ScoreBreakdown {
    pub speed_window: f64,
    pub dwell: f64,
    pub depth: f64,
    pub next_shot: f64,
    pub speed: f64,
    pub travel: f64,
    pub side_spin: f64,
    pub vertical_spin: f64,
    pub cushion: f64,
    pub total: f64,
}

impl ScoreBreakdown {
    fn zero() -> Self {
        Self {
            speed_window: 0.0,
            dwell: 0.0,
            depth: 0.0,
            next_shot: 0.0,
            speed: 0.0,
            travel: 0.0,
            side_spin: 0.0,
            vertical_spin: 0.0,
            cushion: 0.0,
            total: 0.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PositionSuggestion {
    pub achievable: bool,
    /// `None` only if no grid point pots cleanly.
    pub best: Option<PositionShotCandidate>,
    /// Up to 3 — the best cell of each distinct route (spin family +
    /// rail sequence) other than the winner's.
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
    /// Quality (0..1) of the best remaining pot from the landing spot —
    /// geometry of the post-shot layout, which is exactly known because
    /// contact-clean shots move nothing but the target.
    next_shot_quality: f64,
    /// The cue ball's route: rail contacts in order (consecutive
    /// duplicates collapsed), in player terms — the six rails of the
    /// table, not pooltool's 30 cushion segments.
    route: Vec<&'static str>,
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
type SearchSlice = (f64, Vec<Vec<Cell>>);
type CellIndex = (usize, GridKey);
type CellRank = (f64, u32, f64);
type RankedCell = (CellRank, usize, GridKey);

/// Search for strikes that pot `target_ball_id` and leave the cue ball
/// inside `target_area` (physics-frame meters).
///
/// The scenario's `strike` is ignored except for `theta`; the search
/// sweeps its own (speed, a, b) grid and solves phi per grid point.
///
/// # Errors
///
/// Returns [`PositionError::DegeneratePolygon`] when `target_area` has fewer
/// than three vertices, or propagates an aim-solving or simulation failure
/// encountered while evaluating the search grid.
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
    // A pot that is geometrically off (path blocked, or cut beyond the
    // playable limit) can never produce a successful cell, and sweeping it
    // anyway hits the aim solver's worst case on every cell. Bail out
    // before simulating anything.
    let feasibility = geometric_pot_feasibility(scenario, target_ball_id, pocket_id)?;
    if !feasibility.feasible {
        return Ok(empty_suggestion());
    }

    let config = config.unwrap_or_default();
    let slices = evaluate_search_slices(scenario, target_ball_id, pocket_id, target_area, &config)?;
    let evaluated_count = count_evaluated(&slices);
    let successful = successful_cells(&slices);

    if successful.is_empty() {
        return Ok(failure_suggestion(
            scenario,
            &slices,
            u32::try_from(evaluated_count).unwrap_or(u32::MAX),
        ));
    }

    let scoring_context = ScoringContext::new(&config, scenario.table.length, target_area);
    let winner = best_cell(&slices, &successful, &scoring_context);
    let alternate_entries = ranked_alternates(&slices, &successful, winner, &scoring_context);
    let successful_count = u32::try_from(successful.len()).unwrap_or(u32::MAX);
    let best = realize_candidate(scenario, &slices, winner, &scoring_context)?;
    let mut alternates = Vec::new();
    for &(_, slice_index, key) in alternate_entries.iter().take(3) {
        alternates.push(realize_candidate(
            scenario,
            &slices,
            (slice_index, key),
            &scoring_context,
        )?);
    }

    Ok(PositionSuggestion {
        achievable: true,
        best: Some(best),
        alternates,
        evaluated_count: u32::try_from(evaluated_count).unwrap_or(u32::MAX),
        successful_count,
    })
}

fn empty_suggestion() -> PositionSuggestion {
    PositionSuggestion {
        achievable: false,
        best: None,
        alternates: Vec::new(),
        evaluated_count: 0,
        successful_count: 0,
    }
}

fn evaluate_search_slices(
    scenario: &SimulationScenario,
    target_ball_id: BallId,
    pocket_id: PocketId,
    target_area: &[Vec2],
    config: &PositionSearchConfig,
) -> Result<Vec<SearchSlice>, PositionError> {
    let mut slices = Vec::new();
    for &a in &config.a_values {
        let cells = evaluate_slice(scenario, target_ball_id, pocket_id, target_area, a, config)?;
        slices.push((a, cells));
    }

    // Side-spin fallback only helps when the pot itself goes in but no
    // primary-spin leave reaches the polygon.
    if !slices_have_success(&slices) && slices_have_pot(&slices) {
        for &a in &config.fallback_a_values {
            if config.a_values.contains(&a) {
                continue;
            }
            let cells =
                evaluate_slice(scenario, target_ball_id, pocket_id, target_area, a, config)?;
            slices.push((a, cells));
        }
    }
    Ok(slices)
}

fn slices_have_success(slices: &[SearchSlice]) -> bool {
    slices
        .iter()
        .flat_map(|(_, grid)| grid.iter().flatten())
        .any(Cell::successful)
}

fn slices_have_pot(slices: &[SearchSlice]) -> bool {
    slices
        .iter()
        .flat_map(|(_, grid)| grid.iter().flatten())
        .any(|cell| cell.potted)
}

fn count_evaluated(slices: &[SearchSlice]) -> usize {
    slices
        .iter()
        .map(|(_, grid)| grid.iter().map(Vec::len).sum::<usize>())
        .sum()
}

fn successful_cells(slices: &[SearchSlice]) -> Vec<CellIndex> {
    slices
        .iter()
        .enumerate()
        .flat_map(|(slice_index, (_, grid))| {
            grid.iter().enumerate().flat_map(move |(speed, row)| {
                row.iter().enumerate().filter_map(move |(spin, cell)| {
                    cell.successful().then_some((slice_index, (speed, spin)))
                })
            })
        })
        .collect()
}

fn rank_cell(
    slices: &[SearchSlice],
    scoring_context: &ScoringContext,
    index: CellIndex,
) -> CellRank {
    let grid = &slices[index.0].1;
    let cell = &grid[index.1.0][index.1.1];
    let window = speed_window(grid, index.1);
    (scoring_context.score(cell, window), window, cell.dwell)
}

fn best_cell(
    slices: &[SearchSlice],
    successful: &[CellIndex],
    scoring_context: &ScoringContext,
) -> CellIndex {
    let mut winner = successful[0];
    let mut winner_rank = rank_cell(slices, scoring_context, winner);
    for &candidate in &successful[1..] {
        let candidate_rank = rank_cell(slices, scoring_context, candidate);
        if candidate_rank.partial_cmp(&winner_rank) == Some(std::cmp::Ordering::Greater) {
            winner = candidate;
            winner_rank = candidate_rank;
        }
    }
    winner
}

fn ranked_alternates(
    slices: &[SearchSlice],
    successful: &[CellIndex],
    winner: CellIndex,
    scoring_context: &ScoringContext,
) -> Vec<RankedCell> {
    let winner_cell = &slices[winner.0].1[winner.1.0][winner.1.1];
    let winner_signature = route_signature(winner_cell);
    let mut best_per_route = std::collections::BTreeMap::new();
    for &index in successful {
        let cell = &slices[index.0].1[index.1.0][index.1.1];
        let signature = route_signature(cell);
        if signature == winner_signature {
            continue;
        }
        let candidate_rank = rank_cell(slices, scoring_context, index);
        let entry = best_per_route
            .entry(signature)
            .or_insert((candidate_rank, index.0, index.1));
        if candidate_rank
            .partial_cmp(&entry.0)
            .is_some_and(std::cmp::Ordering::is_gt)
        {
            *entry = (candidate_rank, index.0, index.1);
        }
    }
    let mut alternates: Vec<RankedCell> = best_per_route.into_values().collect();
    alternates.sort_by(|lhs, rhs| {
        rhs.0
            .partial_cmp(&lhs.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    alternates
}

fn realize_candidate(
    scenario: &SimulationScenario,
    slices: &[SearchSlice],
    index: CellIndex,
    scoring_context: &ScoringContext,
) -> Result<PositionShotCandidate, PositionError> {
    let grid = &slices[index.0].1;
    let cell = &grid[index.1.0][index.1.1];
    let window = speed_window(grid, index.1);
    Ok(to_candidate(
        cell,
        window,
        scoring_context.breakdown(cell, window),
        Some(project(scenario, cell)?),
    ))
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
                    next_shot_quality: 0.0,
                    route: Vec::new(),
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
    let potted = potted_in_pocket(&projection, target_ball_id, pocket_id);
    let scratched = ball_pocketed(&projection, scenario.cue_ball_id);
    let clean = sole_contact_is_cue_target(&projection, scenario.cue_ball_id, target_ball_id);
    // Only cells that can rank need the next-shot geometry.
    let next_shot_quality = if in_target_area && potted && !scratched && clean {
        best_next_shot_quality(scenario, target_ball_id, cue_final)
    } else {
        0.0
    };
    let path = cue_path(&projection, scenario, cue_final);
    let travel = path
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum::<f64>();
    let mut route: Vec<&'static str> = Vec::new();
    let mut cushions = 0u32;
    for event in &projection.events {
        if event.event_type == SimulationEventType::BallCushion
            && event.ball_ids.contains(&scenario.cue_ball_id)
        {
            cushions = cushions.saturating_add(1);
            if let Some(position) = event.position {
                let rail = rail_label(position, &scenario.table);
                // Jaw rattles register several contacts on one rail; a
                // route counts that as one visit.
                if route.last() != Some(&rail) {
                    route.push(rail);
                }
            }
        }
    }
    Ok(Cell {
        speed: scenario.strike.speed,
        phi: scenario.strike.phi,
        theta: scenario.strike.theta,
        a,
        b: scenario.strike.b,
        cue_final_position: cue_final,
        potted,
        scratched,
        in_target_area,
        boundary_distance: distance_to_polygon_boundary(cue_final, polygon),
        dwell: terminal_dwell(&path, polygon),
        travel,
        cushions,
        next_shot_quality,
        route,
        secondary_contact: !clean,
    })
}

/// Classify a cushion contact into the six rails players count: the two
/// short rails and the four long-rail halves either side of the side
/// pockets. Physics frame: x spans the short dimension, y the long one.
fn rail_label(position: Vec2, table: &crate::model::TableSpec) -> &'static str {
    let w = table.width;
    let l = table.length;
    let distances = [
        (position.y, "bottom"),
        (l - position.y, "top"),
        (position.x, "left"),
        (w - position.x, "right"),
    ];
    let side = distances
        .iter()
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
        .map_or("left", |entry| entry.1);
    match side {
        "bottom" => "bottom",
        "top" => "top",
        "left" => {
            if position.y < l / 2.0 {
                "left_low"
            } else {
                "left_high"
            }
        }
        _ => {
            if position.y < l / 2.0 {
                "right_low"
            } else {
                "right_high"
            }
        }
    }
}

/// Spin family in player terms; thresholds match the client's labels.
fn spin_family(b: f64) -> &'static str {
    if b > 0.08 {
        "follow"
    } else if b < -0.08 {
        "draw"
    } else {
        "stun"
    }
}

/// Two cells are the same *shot* when they share a route signature: the
/// spin family plus the ordered rails the cue ball takes. Adjacency in
/// (speed, spin) parameter space says nothing about shot identity — the
/// success region is usually one connected blob — but the route is how a
/// player would name the shot.
fn route_signature(cell: &Cell) -> String {
    format!("{}|{}", spin_family(cell.b), cell.route.join(">"))
}

/// A pot precision window (degrees) at least this wide counts as a fully
/// comfortable next shot.
const NEXT_SHOT_FULL_PRECISION: f64 = 6.0;

/// Quality (0..1) of the best remaining pot from `cue_final` after the
/// target drops. Contact-clean shots move nothing but the target, so the
/// post-shot layout is exact: the original balls minus the target, with
/// the cue at its landing spot. Pure geometry — no simulations.
fn best_next_shot_quality(
    scenario: &SimulationScenario,
    potted_target_id: BallId,
    cue_final: Vec2,
) -> f64 {
    let mut after = scenario.clone();
    after.balls.retain(|ball| ball.id != potted_target_id);
    if let Some(cue) = after
        .balls
        .iter_mut()
        .find(|ball| ball.id == after.cue_ball_id)
    {
        cue.position = cue_final;
    }
    let mut best = 0.0_f64;
    for ball in &after.balls {
        if ball.id == after.cue_ball_id {
            continue;
        }
        for pocket in [
            PocketId::LeftBottom,
            PocketId::LeftCenter,
            PocketId::LeftTop,
            PocketId::RightBottom,
            PocketId::RightCenter,
            PocketId::RightTop,
        ] {
            if let Ok(feasibility) = geometric_pot_feasibility(&after, ball.id, pocket)
                && feasibility.feasible
            {
                best =
                    best.max((feasibility.required_precision / NEXT_SHOT_FULL_PRECISION).min(1.0));
            }
        }
    }
    best
}

/// Polygon area via the shoelace formula (absolute value).
fn polygon_area(polygon: &[Vec2]) -> f64 {
    let count = polygon.len();
    let mut twice_area = 0.0;
    for index in 0..count {
        let a = polygon[index];
        let b = polygon[(index + 1) % count];
        twice_area += a.x * b.y - b.x * a.y;
    }
    (twice_area / 2.0).abs()
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
    /// Depth saturates at this distance from the boundary — half the
    /// zone's own scale (sqrt of area), so "centered" is relative to
    /// what the user drew rather than a fixed absolute margin.
    depth_scale: f64,
}

impl ScoringContext {
    fn new(config: &PositionSearchConfig, table_length: f64, polygon: &[Vec2]) -> Self {
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
            depth_scale: (0.5 * polygon_area(polygon).sqrt()).max(f64::EPSILON),
        }
    }

    /// Heuristic desirability of a successful leave: benefits (speed
    /// window, terminal dwell, depth in the area) minus penalties (strike
    /// speed, cue travel, spin — side spin weighted heavier than
    /// follow/draw — and cushions). Saturations: dwell at half a table
    /// length, depth at 0.3 m, travel at two table lengths, cushions at 3.
    fn score(&self, cell: &Cell, window: u32) -> f64 {
        self.breakdown(cell, window).total
    }

    /// The signed, weighted contribution of every factor, so callers can
    /// show why a leave scored the way it did. Benefits are >= 0,
    /// penalties <= 0, and `total` is their sum.
    fn breakdown(&self, cell: &Cell, window: u32) -> ScoreBreakdown {
        let window_n = (f64::from(window) - 1.0).max(0.0) / (self.max_window - 1.0).max(1.0);
        let dwell_n = (cell.dwell / (0.5 * self.table_length)).min(1.0);
        let depth_n = if cell.in_target_area {
            (cell.boundary_distance / self.depth_scale).min(1.0)
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
        let mut breakdown = ScoreBreakdown {
            speed_window: weights.speed_window * window_n,
            dwell: weights.dwell * dwell_n,
            depth: weights.depth * depth_n,
            next_shot: weights.next_shot * cell.next_shot_quality.clamp(0.0, 1.0),
            speed: -weights.speed * speed_n,
            travel: -weights.travel * travel_n,
            side_spin: -weights.side_spin * side_spin_n,
            vertical_spin: -vertical_weight * vertical_spin_n,
            cushion: -weights.cushion * cushion_n,
            total: 0.0,
        };
        breakdown.total = breakdown.speed_window
            + breakdown.dwell
            + breakdown.depth
            + breakdown.next_shot
            + breakdown.speed
            + breakdown.travel
            + breakdown.side_spin
            + breakdown.vertical_spin
            + breakdown.cushion;
        breakdown
    }
}

/// Deepest tip offset the default sweep uses; spin terms normalize
/// against it.
const MAX_SPIN_OFFSET: f64 = 0.85;

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
    let best = closest.map(|cell| {
        to_candidate(
            cell,
            0,
            ScoreBreakdown::zero(),
            project(scenario, cell).ok(),
        )
    });
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
    breakdown: ScoreBreakdown,
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
        score: breakdown.total,
        score_breakdown: breakdown,
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
            next_shot_quality: 0.0,
            route: Vec::new(),
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
    fn route_signature_distinguishes_routes_and_spin() {
        let mut follow_one_rail = cell(true);
        follow_one_rail.b = 0.4;
        follow_one_rail.route = vec!["top"];
        let mut draw_one_rail = cell(true);
        draw_one_rail.b = -0.4;
        draw_one_rail.route = vec!["top"];
        let mut follow_two_rails = cell(true);
        follow_two_rails.b = 0.4;
        follow_two_rails.route = vec!["top", "right_high"];
        assert_ne!(
            route_signature(&follow_one_rail),
            route_signature(&draw_one_rail)
        );
        assert_ne!(
            route_signature(&follow_one_rail),
            route_signature(&follow_two_rails)
        );
        // Same family + same rails = the same shot, whatever the params.
        let mut same = follow_one_rail.clone();
        same.speed = 3.9;
        assert_eq!(route_signature(&follow_one_rail), route_signature(&same));
    }

    #[test]
    fn rail_label_classifies_the_six_rails() {
        let table = crate::model::TableSpec::default();
        let l = table.length;
        let w = table.width;
        assert_eq!(rail_label(Vec2::new(w / 2.0, 0.01), &table), "bottom");
        assert_eq!(rail_label(Vec2::new(w / 2.0, l - 0.01), &table), "top");
        assert_eq!(rail_label(Vec2::new(0.01, l * 0.25), &table), "left_low");
        assert_eq!(rail_label(Vec2::new(0.01, l * 0.75), &table), "left_high");
        assert_eq!(
            rail_label(Vec2::new(w - 0.01, l * 0.25), &table),
            "right_low"
        );
        assert_eq!(
            rail_label(Vec2::new(w - 0.01, l * 0.75), &table),
            "right_high"
        );
    }

    fn scoring_context() -> ScoringContext {
        ScoringContext::new(&PositionSearchConfig::default(), 2.54, &square())
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
    fn breakdown_sums_to_score_with_correct_signs() {
        let context = scoring_context();
        let mut cell = cell(true);
        cell.speed = 3.0;
        cell.travel = 2.0;
        cell.a = 0.4;
        cell.b = -0.5;
        cell.cushions = 2;
        cell.dwell = 0.4;
        let breakdown = context.breakdown(&cell, 3);
        assert!((breakdown.total - context.score(&cell, 3)).abs() < 1e-12);
        // Benefits non-negative, penalties non-positive.
        for benefit in [
            breakdown.speed_window,
            breakdown.dwell,
            breakdown.depth,
            breakdown.next_shot,
        ] {
            assert!(benefit >= 0.0);
        }
        for penalty in [
            breakdown.speed,
            breakdown.travel,
            breakdown.side_spin,
            breakdown.vertical_spin,
            breakdown.cushion,
        ] {
            assert!(penalty <= 0.0);
        }
        // b < 0 routes the vertical term through the (heavier) draw weight.
        assert!(breakdown.vertical_spin < 0.0);
    }

    #[test]
    fn score_rewards_open_next_shot() {
        let context = scoring_context();
        let base = cell(true);
        let mut open_next = base.clone();
        open_next.next_shot_quality = 1.0;
        assert!(context.score(&open_next, 2) > context.score(&base, 2));
    }

    #[test]
    fn polygon_area_shoelace() {
        assert!((polygon_area(&square()) - 100.0).abs() < 1e-9);
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

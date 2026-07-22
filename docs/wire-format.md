# JSON wire format

All coordinates and physical values use meters, seconds, kilograms, radians
per second, and degrees where a field name says `phi` or `theta`.

## Simulation request

```json
{
  "scenario": {
    "balls": [
      {"id": 0, "position": {"x": 0.4, "y": 1.27}},
      {"id": 1, "position": {"x": 0.9, "y": 1.27}}
    ],
    "cue_ball_id": 0,
    "strike": {
      "speed": 2.0,
      "phi": 0.0,
      "theta": 0.0,
      "a": 0.0,
      "b": 0.0
    },
    "table": {"length": 2.54, "width": 1.27}
  },
  "options": {"trajectory_dt": 0.01}
}
```

`table.ball`, `table.cue`, `table.pocket_table`, and omitted option fields use
Pooltool-compatible defaults. The response is a `ShotProjection`: trajectories,
events, final state, and `potted_ball_ids`. Enum strings use Rust variant names,
for example `"BallCushion"`, `"Rolling"`, and `"Pocketed"`.

## Pot-aim request

```json
{
  "scenario": {"...": "same object as above"},
  "target_ball_id": 1,
  "pocket_id": "RightCenter"
}
```

Pocket IDs are `LeftBottom`, `LeftCenter`, `LeftTop`, `RightBottom`,
`RightCenter`, and `RightTop` in the physics frame. The response contains the
refined and geometric phi, cut angle, precision window, feasibility and
verification flags, and occluding ball IDs.

## Position-search response

`suggest_position_shot_json` returns achievability, counts, and
best/alternate candidates. Each candidate carries the solved strike, the
cue resting position, `robustness` (contiguous speed steps at the same
spin that also succeed), `dwell` (in-area length of the terminal
approach), `cue_travel_distance`, `cue_cushion_count`, `cue_route` (ordered
player-facing rail labels with consecutive duplicates collapsed), and
`score` — the heuristic leave quality (benefits minus penalties; see
`ScoringWeights` on the request's optional `config.scoring`). Scores are
comparable only within a single search.

## Scenario evaluation request

`evaluate_scenario_json` takes a scenario test case — a `PositionRequest`
under `request` plus fuzzy expectations under `expect` — runs the position
search, and returns a `ScenarioReport` grading the result. The case and
report schemas are documented in `scenario-format.md`; the same files live
in `tests/scenarios/` and run under `cargo test`.

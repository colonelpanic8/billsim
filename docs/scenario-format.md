# Scenario test-case format

A scenario is a JSON file pairing a raw position-search request (the wire
format's `PositionRequest`, unchanged) with fuzzy, human-authored
expectations about the shots the search should find. Files live in
`tests/scenarios/*.json` and every file is run by `cargo test` (see
`tests/scenario_cases.rs`). The same evaluation is exported over every
FFI surface as `evaluate_scenario_json`, so authoring tools (e.g. the
Railbird scenario editor) grade a case exactly the way CI will.

Units and conventions match `wire-format.md`: physics-frame meters, y up,
`phi`/`theta` in degrees, pocket ids `LeftBottom` … `RightTop`.

## Shape

```json
{
	"name": "ross_four_track_breakout",
	"description": "Optional prose for humans.",
	"source": "Optional attribution for published drills.",
	"request": {
		"scenario": {
			"balls": [
				{ "id": 0, "position": { "x": 0.53, "y": 0.27 } },
				{ "id": 7, "position": { "x": 0.3175, "y": 0.0413 } }
			],
			"cue_ball_id": 0,
			"strike": { "speed": 1.0, "phi": 0.0, "theta": 0.0, "a": 0.0, "b": 0.0 },
			"table": {}
		},
		"target_ball_id": 7,
		"pocket_id": "LeftBottom",
		"target_area": [
			{ "x": 1.08, "y": 1.745 },
			{ "x": 1.24, "y": 1.745 },
			{ "x": 1.24, "y": 2.065 },
			{ "x": 1.08, "y": 2.065 }
		]
	},
	"expect": {
		"achievable": true,
		"shots": [
			{ "label": "direct short rail", "route": ["bottom"] },
			{ "label": "short-long", "route": ["bottom", "right"], "spin": "stun" }
		],
		"min_found": 2
	}
}
```

- `request` is passed to the search verbatim; `target_area` is an
  arbitrary polygon (any simple polygon with >= 3 vertices, not just a
  rectangle). `scenario.strike` is required by the wire schema but the
  search ignores its values; `"table": {}` selects the default table.
- `expect.achievable` (default `true`) must equal the search's
  `achievable` flag.
- `expect.shots` lists shots the search should surface among `best` and
  `alternates`. `min_found` (default: all of them) lowers the bar for
  known-flaky approaches without deleting their descriptions.

## Expected-shot matching

Every field of an expected shot except `label` is optional, and only the
fields present constrain the match; a shot is found when **any** returned
candidate satisfies all of its constraints.

- `route` — ordered cue-ball cushion route, in the candidate `cue_route`
  vocabulary: `bottom`, `top`, `left_low`, `left_high`, `right_low`,
  `right_high`. The coarse labels `left` / `right` match either half of
  that long rail; a route written entirely in coarse labels also merges
  a candidate's consecutive same-rail contacts (`right_low, right_high`
  matches `["right"]`). `[]` requires a no-cushion leave.
- `spin` — strike spin family: `draw`, `stun`, or `follow` (thresholds
  at |b| = 0.08, matching the client's labels).
- `speed`, `phi`, `theta`, `a`, `b` — strike parameters, each compared
  within `tolerances` (per-field half-widths; `phi` compares on the
  circle). Defaults are deliberately loose: `speed` 0.35 m/s, `phi` 3°,
  `theta` 5°, `a`/`b` 0.2 ball radii. Prefer `route`/`spin` constraints
  where possible — they name shot families the way players do and
  survive solver retuning.

## Evaluation report

`evaluate_scenario` / `evaluate_scenario_json` returns a `ScenarioReport`:

```json
{
	"name": "...",
	"passed": false,
	"failures": ["found 1/2 expected shots (need 2); ...", "missing expected shot: short-long"],
	"outcomes": [{ "label": "direct short rail", "found": true, "matched_candidate": 0 }],
	"suggestion": { "achievable": true, "best": { "...": "full PositionSuggestion" } }
}
```

`matched_candidate` indexes the concatenation of `best` (index 0) and
`alternates` (1..). The full `suggestion` is included so editors can show
what the engine currently finds next to what the case expects.

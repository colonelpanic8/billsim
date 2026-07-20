"""Drop-in Railbird adapters around billsim's native JSON boundary."""

from __future__ import annotations

import json
from typing import Any

from ._native import compute_pot_aim_json, simulate_json


_EVENT_TYPES = {
    "StickBall": "STICK_BALL",
    "BallBall": "BALL_BALL",
    "BallCushion": "BALL_CUSHION",
    "BallPocket": "BALL_POCKET",
    "RollingStationary": "BALL_STOP",
    "SpinningStationary": "BALL_STOP",
}

_PLANE_TO_PHYSICS_POCKET = {
    "top_left": "RightBottom",
    "top_side": "RightCenter",
    "top_right": "RightTop",
    "bottom_left": "LeftBottom",
    "bottom_side": "LeftCenter",
    "bottom_right": "LeftTop",
}


def _scenario_payload(scenario: Any, mapper: Any) -> dict[str, Any]:
    return {
        "balls": [
            {
                "id": ball.ball_id,
                "position": dict(
                    zip(("x", "y"), mapper.plane_to_physics(ball.position), strict=True)
                ),
            }
            for ball in scenario.balls
        ],
        "cue_ball_id": scenario.cue_ball_id,
        "strike": {
            "speed": scenario.strike.v0,
            "phi": mapper.plane_phi_to_physics_phi(scenario.strike.phi),
            "theta": scenario.strike.theta,
            "a": scenario.strike.a,
            "b": scenario.strike.b,
        },
        "table": {
            "length": scenario.table.length_m,
            "width": scenario.table.width_m,
            "ball": {"radius": scenario.table.ball_radius_m},
        },
    }


class RustShotSimulator:
    """Implements Railbird's ``ShotSimulator`` protocol."""

    def simulate(self, scenario: Any, mapper: Any) -> Any:
        from railbird.shot_simulation import datatypes as types

        raw = json.loads(
            simulate_json(
                json.dumps(
                    {
                        "scenario": _scenario_payload(scenario, mapper),
                        "options": {"trajectory_dt": 0.01},
                    }
                )
            )
        )
        trajectories = [
            types.BallTrajectory(
                ball_id=trajectory["ball_id"],
                points=[
                    types.TrajectoryPoint(
                        time=point["time"],
                        position=mapper.physics_to_plane(
                            (point["position"]["x"], point["position"]["y"])
                        ),
                    )
                    for point in trajectory["points"]
                ],
            )
            for trajectory in raw["trajectories"]
        ]
        events = []
        for event in raw["events"]:
            mapped_name = _EVENT_TYPES.get(event["event_type"])
            if mapped_name is None:
                continue
            position = event["position"]
            events.append(
                types.SimulationEvent(
                    event_type=getattr(types.SimulationEventType, mapped_name),
                    time=event["time"],
                    ball_ids=event["ball_ids"],
                    position=(
                        None
                        if position is None
                        else mapper.physics_to_plane((position["x"], position["y"]))
                    ),
                )
            )
        return types.ShotProjection(
            trajectories=trajectories,
            events=events,
            final_state=[
                types.BallState(
                    ball_id=state["ball_id"],
                    position=mapper.physics_to_plane(
                        (state["position"]["x"], state["position"]["y"])
                    ),
                )
                for state in raw["final_state"]
            ],
            potted_ball_ids=raw["potted_ball_ids"],
        )


def compute_pot_aim(
    scenario: Any, target_ball_id: int, pocket_identifier: Any, mapper: Any
) -> Any:
    """Return Railbird's ``PotAim`` while computing entirely in Rust."""
    from railbird.shot_simulation.aiming import PotAim

    pocket_name = getattr(pocket_identifier, "value", str(pocket_identifier))
    raw = json.loads(
        compute_pot_aim_json(
            json.dumps(
                {
                    "scenario": _scenario_payload(scenario, mapper),
                    "target_ball_id": target_ball_id,
                    "pocket_id": _PLANE_TO_PHYSICS_POCKET[pocket_name],
                }
            )
        )
    )
    return PotAim(
        phi=mapper.physics_phi_to_plane_phi(raw["phi"]),
        geometric_phi=mapper.physics_phi_to_plane_phi(raw["geometric_phi"]),
        cut_angle=raw["cut_angle"],
        required_precision=raw["required_precision"],
        feasible=raw["feasible"],
        potted=raw["potted"],
        converged=raw["converged"],
        occluding_ball_ids=raw["occluding_ball_ids"],
    )


BillsimShotSimulator = RustShotSimulator

__all__ = ["BillsimShotSimulator", "RustShotSimulator", "compute_pot_aim"]

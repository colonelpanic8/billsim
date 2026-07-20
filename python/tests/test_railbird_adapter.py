import dataclasses
import enum
import sys
import types
import unittest

from billsim.railbird import RustShotSimulator, compute_pot_aim


class EventType(enum.Enum):
    STICK_BALL = "stick_ball"
    BALL_BALL = "ball_ball"
    BALL_CUSHION = "ball_cushion"
    BALL_POCKET = "ball_pocket"
    BALL_STOP = "ball_stop"


@dataclasses.dataclass
class TrajectoryPoint:
    time: float
    position: tuple[float, float]


@dataclasses.dataclass
class BallTrajectory:
    ball_id: int
    points: list[TrajectoryPoint]


@dataclasses.dataclass
class SimulationEvent:
    event_type: EventType
    time: float
    ball_ids: list[int]
    position: tuple[float, float] | None = None


@dataclasses.dataclass
class BallState:
    ball_id: int
    position: tuple[float, float]


@dataclasses.dataclass
class ShotProjection:
    trajectories: list[BallTrajectory]
    events: list[SimulationEvent]
    final_state: list[BallState]
    potted_ball_ids: list[int]


@dataclasses.dataclass
class PotAim:
    phi: float
    geometric_phi: float
    cut_angle: float
    required_precision: float
    feasible: bool
    potted: bool
    converged: bool
    occluding_ball_ids: list[int]


class IdentityMapper:
    @staticmethod
    def plane_to_physics(position):
        return position

    @staticmethod
    def physics_to_plane(position):
        return position

    @staticmethod
    def plane_phi_to_physics_phi(phi):
        return phi

    @staticmethod
    def physics_phi_to_plane_phi(phi):
        return phi


class RailbirdAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        railbird = types.ModuleType("railbird")
        shot_simulation = types.ModuleType("railbird.shot_simulation")
        datatypes = types.ModuleType("railbird.shot_simulation.datatypes")
        aiming = types.ModuleType("railbird.shot_simulation.aiming")
        for name, value in globals().items():
            if name in {
                "EventType",
                "TrajectoryPoint",
                "BallTrajectory",
                "SimulationEvent",
                "BallState",
                "ShotProjection",
            }:
                setattr(datatypes, "SimulationEventType" if name == "EventType" else name, value)
        shot_simulation.datatypes = datatypes
        aiming.PotAim = PotAim
        shot_simulation.aiming = aiming
        railbird.shot_simulation = shot_simulation
        sys.modules["railbird"] = railbird
        sys.modules["railbird.shot_simulation"] = shot_simulation
        sys.modules["railbird.shot_simulation.datatypes"] = datatypes
        sys.modules["railbird.shot_simulation.aiming"] = aiming

    def test_shot_simulator_protocol_shape(self):
        ball = types.SimpleNamespace(ball_id=0, position=(0.5, 0.5))
        strike = types.SimpleNamespace(v0=1.0, phi=0.0, theta=0.0, a=0.0, b=0.0)
        table = types.SimpleNamespace(length_m=100.0, width_m=100.0, ball_radius_m=0.028575)
        scenario = types.SimpleNamespace(
            balls=[ball], cue_ball_id=0, strike=strike, table=table
        )

        projection = RustShotSimulator().simulate(scenario, IdentityMapper())

        self.assertIsInstance(projection, ShotProjection)
        self.assertEqual(projection.final_state[0].ball_id, 0)
        self.assertGreater(len(projection.trajectories[0].points), 10)
        self.assertEqual(projection.events[0].event_type, EventType.STICK_BALL)

    def test_pot_aim_shape(self):
        balls = [
            types.SimpleNamespace(ball_id=0, position=(0.4, 1.27)),
            types.SimpleNamespace(ball_id=1, position=(0.9, 1.27)),
        ]
        strike = types.SimpleNamespace(v0=2.0, phi=0.0, theta=0.0, a=0.0, b=0.0)
        table = types.SimpleNamespace(length_m=2.54, width_m=1.27, ball_radius_m=0.028575)
        scenario = types.SimpleNamespace(
            balls=balls, cue_ball_id=0, strike=strike, table=table
        )

        aim = compute_pot_aim(
            scenario,
            1,
            types.SimpleNamespace(value="top_side"),
            IdentityMapper(),
        )

        self.assertIsInstance(aim, PotAim)
        self.assertTrue(aim.potted)
        self.assertTrue(aim.converged)


if __name__ == "__main__":
    unittest.main()

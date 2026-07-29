import ctypes
import unittest

from simulation.bindings import (
    DecisionMove,
    DecisionPlan,
    DecisionPoint,
    TrajectoryPlan,
    TrajectoryPose,
    load_library,
)


class BindingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = load_library(force_rebuild=True)

    def test_python_layout_matches_c_layout(self):
        checks = {
            "Simulation_SizeOfDecisionPoint": ctypes.sizeof(DecisionPoint),
            "Simulation_SizeOfDecisionMove": ctypes.sizeof(DecisionMove),
            "Simulation_SizeOfDecisionPlan": ctypes.sizeof(DecisionPlan),
            "Simulation_SizeOfTrajectoryPose": ctypes.sizeof(TrajectoryPose),
            "Simulation_SizeOfTrajectoryPlan": ctypes.sizeof(TrajectoryPlan),
            "Simulation_OffsetOfDecisionPlanMoves": DecisionPlan.moves.offset,
        }
        for name, expected in checks.items():
            function = getattr(self.library, name)
            function.restype = ctypes.c_uint32
            self.assertEqual(expected, function(), name)

    def test_public_algorithms_are_exported(self):
        self.assertTrue(callable(self.library.Decision_Solve))
        self.assertTrue(callable(self.library.Trajectory_Generate))
        self.assertTrue(callable(self.library.Trajectory_Evaluate))


if __name__ == "__main__":
    unittest.main()

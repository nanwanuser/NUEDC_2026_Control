import unittest

import numpy as np

from simulation.scenarios import create_scenario
from simulation.simulator import (
    pose_array,
    run_simulation,
    segment_kinematics,
)


class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = run_simulation(
            create_scenario("general"), sample_period_s=0.01
        )

    def test_all_moves_reach_pick_transit_and_place(self):
        self.assertEqual(4, len(self.result.moves))
        for execution in self.result.moves:
            samples = [
                sample
                for sample in self.result.samples
                if sample.move_index == execution.move_index
            ]
            approach = [sample for sample in samples if sample.phase == "approach"]
            transfer = [sample for sample in samples if sample.phase == "transfer"]

            self.assertTrue(
                np.allclose(
                    approach[-1].pose,
                    pose_array(execution.request.pick),
                    atol=1e-3,
                )
            )
            transit = min(
                transfer,
                key=lambda sample: abs(sample.time_s - execution.transit_time_s),
            )
            self.assertTrue(
                np.allclose(
                    transit.pose,
                    pose_array(execution.request.transit),
                    atol=2e-2,
                )
            )
            self.assertTrue(
                np.allclose(
                    transfer[-1].pose,
                    pose_array(execution.request.place),
                    atol=1e-3,
                )
            )

    def test_grip_sequence_and_move_order(self):
        for execution in self.result.moves:
            samples = [
                sample
                for sample in self.result.samples
                if sample.move_index == execution.move_index
            ]
            self.assertEqual(0, samples[0].grip)
            self.assertTrue(
                any(sample.phase == "hold" and sample.grip == 1 for sample in samples)
            )
            transfer = [sample for sample in samples if sample.phase == "transfer"]
            self.assertTrue(any(sample.grip == 1 for sample in transfer[:-1]))
            self.assertEqual(0, transfer[-1].grip)

        for previous, current in zip(self.result.moves, self.result.moves[1:]):
            self.assertTrue(
                np.allclose(
                    pose_array(current.request.current),
                    pose_array(previous.request.place),
                    atol=1e-6,
                )
            )

    def test_transfer_is_c2_continuous(self):
        for execution in self.result.moves:
            first = execution.plan.transfer[0]
            second = execution.plan.transfer[1]
            p_left, v_left, a_left = segment_kinematics(first, first.duration_s)
            p_right, v_right, a_right = segment_kinematics(second, 0.0)
            self.assertTrue(np.allclose(p_left, p_right, atol=2e-3))
            self.assertTrue(np.allclose(v_left, v_right, atol=0.05))
            self.assertTrue(np.allclose(a_left, a_right, atol=0.05))

    def test_sampled_motion_respects_limits(self):
        limits = self.result.scenario.limits
        self.assertLessEqual(
            max(sample.linear_speed for sample in self.result.samples),
            limits.max_linear_velocity_mm_s * 1.001,
        )
        self.assertLessEqual(
            max(sample.linear_acceleration for sample in self.result.samples),
            limits.max_linear_acceleration_mm_s2 * 1.001,
        )
        self.assertLessEqual(
            max(sample.yaw_speed for sample in self.result.samples),
            limits.max_yaw_velocity_deg_s * 1.001,
        )
        self.assertLessEqual(
            max(sample.yaw_acceleration for sample in self.result.samples),
            limits.max_yaw_acceleration_deg_s2 * 1.001,
        )

    def test_low_height_inter_move_approaches_are_flagged(self):
        first_move_risks = [
            sample.low_height_risk
            for sample in self.result.samples
            if sample.move_index == 0 and sample.phase == "approach"
        ]
        self.assertFalse(any(first_move_risks))
        for move_index in range(1, 4):
            risks = [
                sample.low_height_risk
                for sample in self.result.samples
                if sample.move_index == move_index and sample.phase == "approach"
            ]
            self.assertTrue(all(risks))


if __name__ == "__main__":
    unittest.main()

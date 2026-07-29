import unittest

import numpy as np

from simulation.scenarios import create_scenario, solve_scenario, transform_polygon


class DecisionScenarioTests(unittest.TestCase):
    def test_fixed_mode_places_every_piece_on_its_template(self):
        scenario = create_scenario("fixed")
        plan = solve_scenario(scenario)

        self.assertEqual(4, plan.move_count)
        self.assertEqual(4, len({plan.moves[index].piece_id for index in range(4)}))
        for index in range(4):
            piece = scenario.frame.pieces[index]
            move = plan.moves[index]
            actual = transform_polygon(
                piece.vertices[: piece.vertex_count], move.pick, move.place
            )
            expected = scenario.target_polygons[move.piece_id]
            self.assertTrue(np.allclose(actual, expected, atol=0.05))

    def test_general_mode_builds_target_rectangle(self):
        scenario = create_scenario("general")
        plan = solve_scenario(scenario)
        placed = []

        for index in range(4):
            piece = scenario.frame.pieces[index]
            move = plan.moves[index]
            placed.append(
                transform_polygon(
                    piece.vertices[: piece.vertex_count], move.pick, move.place
                )
            )

        points = np.concatenate(placed)
        extent = points.max(axis=0) - points.min(axis=0)
        center = 0.5 * (points.max(axis=0) + points.min(axis=0))
        self.assertEqual(4, plan.move_count)
        self.assertTrue(np.allclose(extent, [100.0, 60.0], atol=0.1))
        self.assertTrue(np.allclose(center, [105.0, 220.0], atol=0.05))


if __name__ == "__main__":
    unittest.main()

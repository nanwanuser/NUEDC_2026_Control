import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import image as mpimg
import numpy as np

from simulation.scenarios import create_scenario
from simulation.simulator import run_simulation
from simulation.visualization import create_figure


class VisualizationTests(unittest.TestCase):
    def test_both_modes_render_nonblank_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            for mode in ("fixed", "general"):
                result = run_simulation(create_scenario(mode))
                figure, view = create_figure(result)
                path = Path(directory) / f"{mode}.png"
                view.set_time(result.samples[-1].time_s)
                for piece_id, polygon in result.final_polygons.items():
                    self.assertTrue(
                        np.allclose(
                            view._piece_labels[piece_id].get_position(),
                            polygon.mean(axis=0),
                        )
                    )
                risk_lines = [
                    line
                    for line in view.axes["path"].lines
                    if line.get_color() == "#C23B33"
                ]
                self.assertTrue(risk_lines)
                self.assertLess(
                    view.completed_path.get_linewidth(),
                    min(line.get_linewidth() for line in risk_lines),
                )
                view.save_snapshot(path)
                pixels = mpimg.imread(path)
                self.assertGreater(path.stat().st_size, 20_000)
                self.assertGreater(float(pixels.std()), 0.03)
                plt.close(figure)


if __name__ == "__main__":
    unittest.main()

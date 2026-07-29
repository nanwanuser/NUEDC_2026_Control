"""Command-line entry point for the host simulation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize the native decision and Cartesian trajectory algorithms."
    )
    parser.add_argument("--mode", choices=("fixed", "general"), default="general")
    parser.add_argument("--snapshot", type=Path)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.snapshot is not None:
        import matplotlib

        matplotlib.use("Agg")

    try:
        import matplotlib.pyplot as plt

        from simulation.scenarios import create_scenario
        from simulation.simulator import run_simulation
        from simulation.visualization import create_figure

        result = run_simulation(create_scenario(args.mode))
        figure, view = create_figure(result)
        if args.snapshot is not None:
            output = args.snapshot.resolve()
            view.set_time(result.samples[-1].time_s)
            view.save_snapshot(output)
            plt.close(figure)
            print(output)
            return 0

        plt.show()
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(f"Simulation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

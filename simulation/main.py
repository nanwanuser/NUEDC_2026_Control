"""Command-line entry point for the host simulation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


CARD_MAX_SEARCH_NODES = 5000
CARD_MAX_ALIGNMENT_RMS_MM = 3.0
CARD_MAX_TEXTURE_MAE = 45.0


def _card_failure_reason(result) -> str | None:
    validation = result.card_validation
    if validation is None:
        return None
    failures: list[str] = []
    if not validation.topology_ok:
        failures.append("topology mismatch")
    if validation.alignment_rms_mm >= CARD_MAX_ALIGNMENT_RMS_MM:
        failures.append("alignment error exceeds limit")
    if validation.texture_mae >= CARD_MAX_TEXTURE_MAE:
        failures.append("texture error exceeds limit")
    if result.decision_plan.search_nodes >= CARD_MAX_SEARCH_NODES:
        failures.append("search nodes exceed budget")
    return "; ".join(failures) or None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize the native decision and Cartesian trajectory algorithms."
    )
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument(
        "--scenario",
        choices=("general", "card"),
        default="general",
    )
    parser.add_argument("--card-image", type=Path)
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

        result = run_simulation(
            create_scenario(args.scenario, card_image=args.card_image)
        )
        figure, view = create_figure(result)
        if result.card_validation is not None:
            validation = result.card_validation
            print(
                "nodes={} topology={} alignment_rms_mm={:.3f} "
                "texture_mae={:.3f}".format(
                    result.decision_plan.search_nodes,
                    "OK" if validation.topology_ok else "FAIL",
                    validation.alignment_rms_mm,
                    validation.texture_mae,
                )
            )
            failure_reason = _card_failure_reason(result)
            if failure_reason is not None:
                raise RuntimeError(f"card verification failed: {failure_reason}")
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

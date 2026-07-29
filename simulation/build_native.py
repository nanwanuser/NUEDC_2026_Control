"""Build the host DLL from the firmware's pure C algorithms."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


SIMULATION_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SIMULATION_DIR.parent
BUILD_DIR = SIMULATION_DIR / "build"
DLL_PATH = BUILD_DIR / "arm_algorithms.dll"

DECISION_DIR = REPOSITORY_ROOT / "App" / "lib" / "decision"
TRAJECTORY_DIR = REPOSITORY_ROOT / "App" / "lib" / "trajectory"
BUILD_INPUTS = (
    SIMULATION_DIR / "native_bridge.c",
    DECISION_DIR / "decision.c",
    DECISION_DIR / "decision.h",
    TRAJECTORY_DIR / "trajectory.c",
    TRAJECTORY_DIR / "trajectory.h",
)


def _is_current(output: Path) -> bool:
    if not output.exists():
        return False
    output_time = output.stat().st_mtime_ns
    return all(path.stat().st_mtime_ns <= output_time for path in BUILD_INPUTS)


def build_library(force: bool = False) -> Path:
    """Build and return the native algorithm DLL path."""

    gcc = shutil.which("gcc")
    if gcc is None:
        raise RuntimeError("GCC was not found on PATH")

    if not force and _is_current(DLL_PATH):
        return DLL_PATH

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    command = [
        gcc,
        "-std=c11",
        "-O1",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-shared",
        "-Wl,--export-all-symbols",
        "-o",
        str(DLL_PATH),
        str(SIMULATION_DIR / "native_bridge.c"),
        str(DECISION_DIR / "decision.c"),
        str(TRAJECTORY_DIR / "trajectory.c"),
        "-I",
        str(DECISION_DIR),
        "-I",
        str(TRAJECTORY_DIR),
        "-lm",
    ]
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            "Native simulation build failed:\n"
            + completed.stdout
            + completed.stderr
        )
    return DLL_PATH


if __name__ == "__main__":
    print(build_library(force=True))

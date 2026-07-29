# Decision and Trajectory Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python/Matplotlib verification simulator that directly calls the repository's C decision and Cartesian trajectory algorithms for fixed-ID and general puzzle modes.

**Architecture:** A small MinGW-built DLL contains the existing `decision.c` and `trajectory.c` plus an ABI-only bridge. Python `ctypes` bindings construct deterministic four-piece scenes, call the C APIs, sequence each move into a unified sampled timeline, and render an interactive four-panel Matplotlib dashboard or a headless PNG snapshot.

**Tech Stack:** Python 3.13, standard-library `ctypes`/`unittest`/`subprocess`, NumPy 2.1, Matplotlib 3.10, MinGW GCC, existing C11 decision and trajectory modules.

## Global Constraints

- Do not modify `App/lib/decision`, `App/lib/trajectory`, FreeRTOS tasks, or CubeMX files.
- Cover both `DECISION_MODE_FIXED_ID` and `DECISION_MODE_GENERAL` with deterministic built-in scenes only.
- Do not implement inverse kinematics, servo dynamics, joint limits, JSON loading, collision physics, or automatic dependency installation.
- Call the actual C algorithms; Python must not reimplement decision search or quintic trajectory generation.
- First move starts at the built-in home pose; every later move starts at the preceding move's `place` pose.
- Preserve the firmware phase semantics: approach, pick hold with `grip=1`, transfer, place completion with `grip=0`.
- Use English plot labels to avoid host font warnings.
- Keep generated DLLs, PNGs, and caches below `simulation/build` or ignored local cache paths.

---

## File Structure

- Create `simulation/__init__.py`: package marker and public version string.
- Create `simulation/.gitignore`: ignore `build/`, `__pycache__/`, and `tests/__pycache__/`.
- Create `simulation/native_bridge.c`: export C ABI size/offset probes only.
- Create `simulation/build_native.py`: locate GCC and build an up-to-date DLL from repository C sources.
- Create `simulation/bindings.py`: declare C structures/constants and configure DLL functions.
- Create `simulation/scenarios.py`: build fixed-ID and general four-piece scenarios.
- Create `simulation/simulator.py`: solve decisions, generate per-move plans, sample phases, calculate derivative metrics, and reconstruct piece polygons.
- Create `simulation/visualization.py`: render and animate the four-panel dashboard.
- Create `simulation/main.py`: parse `--mode` and `--snapshot`, select backend, and run the app.
- Create `simulation/tests/test_bindings.py`: verify DLL build, ABI, and function loading.
- Create `simulation/tests/test_decision.py`: verify both built-in decision scenes.
- Create `simulation/tests/test_simulator.py`: verify endpoints, phase/grip sequencing, continuity, and constraints.
- Create `simulation/tests/test_visualization.py`: verify both headless snapshots are nonblank.

---

### Task 1: Native Build and ABI-Safe Bindings

**Files:**
- Create: `simulation/__init__.py`
- Create: `simulation/.gitignore`
- Create: `simulation/native_bridge.c`
- Create: `simulation/build_native.py`
- Create: `simulation/bindings.py`
- Create: `simulation/tests/__init__.py`
- Create: `simulation/tests/test_bindings.py`

**Interfaces:**
- Produces: `build_library(force: bool = False) -> pathlib.Path`
- Produces: `load_library(force_rebuild: bool = False) -> ctypes.CDLL`
- Produces: ctypes types matching all public types in `decision.h` and `trajectory.h`.
- Produces: constants `DECISION_MODE_FIXED_ID`, `DECISION_MODE_GENERAL`, `DECISION_RESULT_OK`, `TRAJECTORY_PHASE_APPROACH`, `TRAJECTORY_PHASE_TRANSFER`, `TRAJECTORY_RESULT_OK`, and trajectory state values.

- [ ] **Step 1: Write the failing ABI test**

Create `simulation/tests/test_bindings.py` with a real DLL build and explicit ABI checks:

```python
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
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest simulation.tests.test_bindings -v
```

Expected: import failure because `simulation.bindings` does not exist.

- [ ] **Step 3: Implement the ABI bridge**

Create `simulation/native_bridge.c` with no algorithm behavior:

```c
#include <stddef.h>
#include <stdint.h>

#include "decision.h"
#include "trajectory.h"

#define SIM_EXPORT __declspec(dllexport)

SIM_EXPORT uint32_t Simulation_SizeOfDecisionPoint(void) {
    return (uint32_t)sizeof(DecisionPoint);
}
SIM_EXPORT uint32_t Simulation_SizeOfDecisionMove(void) {
    return (uint32_t)sizeof(DecisionMove);
}
SIM_EXPORT uint32_t Simulation_SizeOfDecisionPlan(void) {
    return (uint32_t)sizeof(DecisionPlan);
}
SIM_EXPORT uint32_t Simulation_SizeOfTrajectoryPose(void) {
    return (uint32_t)sizeof(TrajectoryPose);
}
SIM_EXPORT uint32_t Simulation_SizeOfTrajectoryPlan(void) {
    return (uint32_t)sizeof(TrajectoryPlan);
}
SIM_EXPORT uint32_t Simulation_OffsetOfDecisionPlanMoves(void) {
    return (uint32_t)offsetof(DecisionPlan, moves);
}
```

- [ ] **Step 4: Implement deterministic DLL building**

In `simulation/build_native.py`, resolve paths from `Path(__file__).resolve()`, choose `gcc.exe` with `shutil.which("gcc")`, compare DLL modification time with both C sources, both headers, and the bridge, then run:

```python
command = [
    gcc,
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-shared", "-Wl,--export-all-symbols",
    "-o", str(dll_path),
    str(bridge_path), str(decision_source), str(trajectory_source),
    "-I", str(decision_include), "-I", str(trajectory_include),
    "-lm",
]
completed = subprocess.run(command, capture_output=True, text=True)
if completed.returncode != 0:
    raise RuntimeError(
        "Native simulation build failed:\n"
        + completed.stdout + completed.stderr
    )
```

`build_library()` must create `simulation/build`, skip rebuilding only when the DLL is newer than every input, and raise `RuntimeError("GCC was not found on PATH")` if GCC is unavailable.

- [ ] **Step 5: Implement ctypes declarations and function signatures**

In `simulation/bindings.py`, define `_fields_` in header order using `ctypes.c_float`, `ctypes.c_uint8`, and `ctypes.c_uint32`. Configure the loaded library with these exact signatures:

```python
library.Decision_GetDefaultConfig.argtypes = [ctypes.POINTER(DecisionConfig)]
library.Decision_GetDefaultConfig.restype = None
library.Decision_Solve.argtypes = [
    ctypes.c_int,
    ctypes.POINTER(DecisionVisionFrame),
    ctypes.POINTER(DecisionFixedLayout),
    ctypes.POINTER(DecisionConfig),
    ctypes.POINTER(DecisionPlan),
]
library.Decision_Solve.restype = ctypes.c_int
library.Trajectory_Generate.argtypes = [
    ctypes.POINTER(TrajectoryRequest),
    ctypes.POINTER(TrajectoryPlan),
]
library.Trajectory_Generate.restype = ctypes.c_int
library.Trajectory_Evaluate.argtypes = [
    ctypes.POINTER(TrajectoryPlan),
    ctypes.c_int,
    ctypes.c_float,
    ctypes.POINTER(TrajectoryReference),
]
library.Trajectory_Evaluate.restype = ctypes.c_int
library.Trajectory_GetDuration.argtypes = [
    ctypes.POINTER(TrajectoryPlan), ctypes.c_int
]
library.Trajectory_GetDuration.restype = ctypes.c_float
```

Cache the loaded `ctypes.CDLL` only when `force_rebuild` is false. Add `__version__ = "1.0.0"` to `simulation/__init__.py` and local ignore rules to `simulation/.gitignore`.

- [ ] **Step 6: Run the binding tests and verify GREEN**

Run:

```powershell
python -m unittest simulation.tests.test_bindings -v
```

Expected: 2 tests pass; GCC emits no warnings because warnings are errors.

- [ ] **Step 7: Commit the native binding layer**

```powershell
git add simulation/.gitignore simulation/__init__.py simulation/native_bridge.c simulation/build_native.py simulation/bindings.py simulation/tests/__init__.py simulation/tests/test_bindings.py
git commit -m "feat(simulation): bind native planning algorithms"
```

---

### Task 2: Deterministic Fixed and General Decision Scenarios

**Files:**
- Create: `simulation/scenarios.py`
- Create: `simulation/tests/test_decision.py`

**Interfaces:**
- Consumes: ctypes structures and `load_library()` from Task 1.
- Produces: immutable Python `Scenario` dataclass with `name`, `mode`, `frame`, `fixed_layout`, `config`, `home`, `limits`, and `target_polygons`.
- Produces: `create_scenario(mode: str) -> Scenario` accepting only `"fixed"` and `"general"`.
- Produces: `solve_scenario(scenario: Scenario) -> DecisionPlan`.
- Produces: `transform_polygon(vertices, pick, place) -> numpy.ndarray` for decision-result verification and later drawing.

- [ ] **Step 1: Write failing tests for both decision modes**

Create `simulation/tests/test_decision.py`:

```python
import unittest
import numpy as np

from simulation.scenarios import create_scenario, solve_scenario, transform_polygon


class DecisionScenarioTests(unittest.TestCase):
    def test_fixed_mode_places_every_piece_on_its_template(self):
        scenario = create_scenario("fixed")
        plan = solve_scenario(scenario)
        self.assertEqual(4, plan.move_count)
        self.assertEqual(4, len({plan.moves[i].piece_id for i in range(4)}))
        for index in range(4):
            piece = scenario.frame.pieces[index]
            move = plan.moves[index]
            actual = transform_polygon(piece.vertices[:piece.vertex_count], move.pick, move.place)
            expected = scenario.target_polygons[move.piece_id]
            self.assertTrue(np.allclose(actual, expected, atol=0.05))

    def test_general_mode_builds_target_rectangle(self):
        scenario = create_scenario("general")
        plan = solve_scenario(scenario)
        placed = []
        for index in range(4):
            piece = scenario.frame.pieces[index]
            move = plan.moves[index]
            placed.append(transform_polygon(piece.vertices[:piece.vertex_count], move.pick, move.place))
        points = np.concatenate(placed)
        extent = points.max(axis=0) - points.min(axis=0)
        center = 0.5 * (points.max(axis=0) + points.min(axis=0))
        self.assertEqual(4, plan.move_count)
        self.assertTrue(np.allclose(extent, [100.0, 60.0], atol=0.1))
        self.assertTrue(np.allclose(center, [105.0, 220.0], atol=0.05))
```

- [ ] **Step 2: Run decision tests and verify RED**

Run:

```powershell
python -m unittest simulation.tests.test_decision -v
```

Expected: import failure because `simulation.scenarios` does not exist.

- [ ] **Step 3: Implement the shared four-triangle geometry**

Define target triangles that exactly tile a 100 mm by 60 mm rectangle:

```python
BASE_POLYGONS = (
    ((0.0, 0.0), (100.0, 0.0), (50.0, 30.0)),
    ((100.0, 0.0), (100.0, 60.0), (50.0, 30.0)),
    ((100.0, 60.0), (0.0, 60.0), (50.0, 30.0)),
    ((0.0, 60.0), (0.0, 0.0), (50.0, 30.0)),
)
ANGLES_DEG = (-18.0, 71.0, 143.0, -96.0)
TRANSLATIONS_MM = ((30.0, 20.0), (155.0, 30.0), (45.0, 105.0), (165.0, 115.0))
```

Build each `DecisionPiece` by rotating and translating its base polygon, and set the center to the transformed vertex mean. Assign IDs 10 through 13. Call `Decision_GetDefaultConfig()`, use home pose `(105, 120, 60, 0)`, and limits `(120 mm/s, 300 mm/s^2, 90 deg/s, 240 deg/s^2)`.

For fixed mode, shift every base polygon by `(55, 190)` so the template rectangle center is `(105, 220)`. For general mode, pass an empty fixed layout; the C solver uses the default target center.

- [ ] **Step 4: Implement decision invocation and polygon reconstruction**

`solve_scenario()` must call `Decision_Solve()`, raise `RuntimeError(f"Decision_Solve failed for {scenario.name}: result={result}")` unless the result equals `DECISION_RESULT_OK`, and return the populated plan.

`transform_polygon()` must implement the same grasp-relative transform used by the C tests:

```python
angle = np.deg2rad(place.yaw_deg)
rotation = np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle),  np.cos(angle)]])
relative = np.asarray(vertices, dtype=float) - [pick.x_mm, pick.y_mm]
return relative @ rotation.T + [place.x_mm, place.y_mm]
```

- [ ] **Step 5: Run both decision tests and verify GREEN**

Run:

```powershell
python -m unittest simulation.tests.test_decision -v
```

Expected: 2 tests pass and both C decision modes return `DECISION_RESULT_OK`.

- [ ] **Step 6: Commit the scenarios**

```powershell
git add simulation/scenarios.py simulation/tests/test_decision.py
git commit -m "feat(simulation): add puzzle decision scenarios"
```

---

### Task 3: Multi-Piece Trajectory Sequencing and Metrics

**Files:**
- Create: `simulation/simulator.py`
- Create: `simulation/tests/test_simulator.py`

**Interfaces:**
- Consumes: `Scenario`, `solve_scenario()`, bindings, and C trajectory APIs.
- Produces: `TrajectorySample` dataclass containing `time_s`, `pose`, `grip`, `move_index`, `piece_id`, `phase`, `state`, `linear_speed`, `linear_acceleration`, `yaw_speed`, `yaw_acceleration`, and `low_height_risk`.
- Produces: `MoveExecution` dataclass containing `move_index`, request, plan, global start/end times, `transit_time_s`, hold interval, and transformed final polygon.
- Produces: `SimulationResult` dataclass containing scenario, decision plan, samples, move executions, initial polygons, and final polygons.
- Produces: `run_simulation(scenario: Scenario, sample_period_s: float = 0.02, pick_hold_s: float = 0.6) -> SimulationResult`.
- Produces: `pose_array(pose: TrajectoryPose) -> numpy.ndarray`.

- [ ] **Step 1: Write failing sequencing tests**

Create tests that assert C endpoints and grip semantics:

```python
class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = run_simulation(create_scenario("general"), sample_period_s=0.01)

    def test_all_moves_reach_pick_transit_and_place(self):
        self.assertEqual(4, len(self.result.moves))
        for execution in self.result.moves:
            samples = [s for s in self.result.samples if s.move_index == execution.move_index]
            approach = [s for s in samples if s.phase == "approach"]
            transfer = [s for s in samples if s.phase == "transfer"]
            self.assertTrue(np.allclose(approach[-1].pose, pose_array(execution.request.pick), atol=1e-3))
            transit_index = np.argmin([abs(s.time_s - execution.transit_time_s) for s in transfer])
            self.assertTrue(np.allclose(transfer[transit_index].pose, pose_array(execution.request.transit), atol=2e-2))
            self.assertTrue(np.allclose(transfer[-1].pose, pose_array(execution.request.place), atol=1e-3))

    def test_grip_sequence_and_move_order(self):
        for execution in self.result.moves:
            samples = [s for s in self.result.samples if s.move_index == execution.move_index]
            self.assertEqual(0, samples[0].grip)
            self.assertTrue(any(s.phase == "hold" and s.grip == 1 for s in samples))
            self.assertTrue(any(s.phase == "transfer" and s.grip == 1 for s in samples[:-1]))
            self.assertEqual(0, samples[-1].grip)
```

Add separate tests that check maximum derivative values are within `1.001 * limits`, that transit left/right derivative differences are below `0.05`, and that move 1 through 3 approach samples set `low_height_risk=True`.

- [ ] **Step 2: Run simulator tests and verify RED**

Run:

```powershell
python -m unittest simulation.tests.test_simulator -v
```

Expected: import failure because `simulation.simulator` does not exist.

- [ ] **Step 3: Implement exact polynomial derivatives**

Read the C-generated power coefficients from each `TrajectorySegment`. For normalized time `u=t/T`, compute axis values and real-time derivatives as:

```python
position = sum(c[k] * u**k for k in range(6))
velocity = sum(k * c[k] * u**(k - 1) for k in range(1, 6)) / duration
acceleration = sum(k * (k - 1) * c[k] * u**(k - 2) for k in range(2, 6)) / duration**2
```

Normalize displayed yaw to `[-180, 180)`, but use unwrapped coefficient derivatives for yaw speed and acceleration. Return zeros for a zero-duration segment and for the hold phase.

- [ ] **Step 4: Implement C-backed phase sampling**

For every move:

1. Construct `TrajectoryRequest(current, pick, transit, place, limits)`.
2. Call `Trajectory_Generate()` and raise a result-specific `RuntimeError` on failure.
3. Sample approach from zero through its exact duration with `Trajectory_Evaluate()`.
4. Append a 0.6 s hold whose pose is pick and grip is 1.
5. Sample transfer through both exact segment boundaries and its exact total duration.
6. Set the next request's current pose to the current move's place pose.

Generate each regular sample grid with a helper that always includes both endpoints:

```python
count = max(1, int(np.ceil(duration / sample_period_s)))
local_times = np.linspace(0.0, duration, count + 1)
```

Avoid duplicate global timestamps by omitting the first sample of a new phase when its state is identical to the previous sample, except for the grip transition at pick and place.

- [ ] **Step 5: Implement risks and placed polygons**

Mark an approach as low-height horizontal travel when XY displacement exceeds 1 mm and the maximum of its endpoint Z values is no more than 25% of `config.transit_z_mm`. Apply the flag to every sample in that approach.

Reconstruct the final polygon for each piece with `transform_polygon()`. Store both initial and final polygon arrays in `SimulationResult`, indexed by piece ID, so the renderer never needs to reinterpret C memory.

- [ ] **Step 6: Run simulator tests and verify GREEN**

Run:

```powershell
python -m unittest simulation.tests.test_simulator -v
```

Expected: all endpoint, state, continuity, constraint, and low-height-risk tests pass.

- [ ] **Step 7: Run all non-rendering tests**

Run:

```powershell
python -m unittest simulation.tests.test_bindings simulation.tests.test_decision simulation.tests.test_simulator -v
```

Expected: all tests pass with no warnings or tracebacks.

- [ ] **Step 8: Commit the simulator core**

```powershell
git add simulation/simulator.py simulation/tests/test_simulator.py
git commit -m "feat(simulation): sequence Cartesian puzzle moves"
```

---

### Task 4: Interactive Matplotlib Dashboard and Snapshot CLI

**Files:**
- Create: `simulation/visualization.py`
- Create: `simulation/main.py`
- Create: `simulation/tests/test_visualization.py`

**Interfaces:**
- Consumes: `create_scenario()` and `run_simulation()`.
- Produces: `SimulationView(result: SimulationResult)` with `set_time(time_s)`, `reset()`, `toggle_play()`, `set_mode(mode: str)`, and `save_snapshot(path: Path)`.
- Produces: `create_figure(result: SimulationResult) -> tuple[Figure, SimulationView]`.
- Produces: CLI exit code 0 for successful interactive launch or snapshot render; nonzero with a concise error on dependency, build, decision, trajectory, or output failures.

- [ ] **Step 1: Write failing headless rendering tests**

Create `simulation/tests/test_visualization.py` and force the Agg backend before importing the renderer:

```python
import tempfile
import unittest
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import image as mpimg

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
                view.save_snapshot(path)
                pixels = mpimg.imread(path)
                self.assertGreater(path.stat().st_size, 20_000)
                self.assertGreater(float(pixels.std()), 0.03)
                figure.clear()
```

- [ ] **Step 2: Run rendering test and verify RED**

Run:

```powershell
python -m unittest simulation.tests.test_visualization -v
```

Expected: import failure because `simulation.visualization` does not exist.

- [ ] **Step 3: Implement the four-panel figure**

Use Matplotlib's object-oriented API and `subplot_mosaic`:

```python
figure = plt.figure(figsize=(15, 9), constrained_layout=False)
grid = figure.add_gridspec(2, 2, left=0.06, right=0.98, bottom=0.18, top=0.93,
                           width_ratios=(1.0, 1.15), hspace=0.30, wspace=0.22)
board_ax = figure.add_subplot(grid[0, 0])
path_ax = figure.add_subplot(grid[0, 1], projection="3d")
pose_ax = figure.add_subplot(grid[1, 0])
limits_ax = figure.add_subplot(grid[1, 1])
```

Draw target polygons as dashed outlines, initial/placed/moving pieces with a colorblind-safe `tab10` subset, grasp points as crosses, approach paths in blue, transfer paths in green, and low-height-risk paths in red. Use a stable board aspect ratio and fixed limits computed from all initial/final polygons plus 10% padding.

Plot `x/y/z` on the pose axis and normalized yaw on a twin axis. Plot four constraint ratios and a stepped grip trace on the limits axis. Add a shared vertical time cursor to both time plots.

- [ ] **Step 4: Implement time-dependent piece and end-effector updates**

At a selected sample:

- pieces with lower move indices use their final polygons;
- the active piece remains at its initial polygon during approach;
- while `grip=1`, transform the active initial polygon from its pick pose to the current end pose;
- at the terminal transfer sample, use the final polygon even though `grip=0`;
- later pieces remain at initial polygons.

Update the 3D current point, yaw direction segment, completed-path line, time cursors, state text, and warning text without recreating axes. The warning text is visible only when `low_height_risk` is true or a constraint ratio exceeds 1.0.

- [ ] **Step 5: Implement controls and CLI**

Add Matplotlib `RadioButtons` for fixed/general mode, `Button` widgets for Play/Pause and Reset, and a `Slider` for time. The animation timer advances by 40 ms of wall time at a 1x simulation rate and wraps to zero after completion.

In `simulation/main.py`, parse arguments before importing pyplot-dependent code:

```python
parser.add_argument("--mode", choices=("fixed", "general"), default="general")
parser.add_argument("--snapshot", type=Path)
args = parser.parse_args()
if args.snapshot is not None:
    import matplotlib
    matplotlib.use("Agg")
```

When run as `python simulation/main.py`, insert the repository root into `sys.path` if `__package__` is empty. For snapshot mode, create the parent directory, set time to the final sample, save at 150 DPI with a white background, close the figure, print the absolute output path, and exit.

- [ ] **Step 6: Run visualization tests and verify GREEN**

Run:

```powershell
python -m unittest simulation.tests.test_visualization -v
```

Expected: both mode snapshots exceed the size and pixel-variance thresholds with no Matplotlib warnings.

- [ ] **Step 7: Exercise both CLI snapshot paths**

Run:

```powershell
python simulation/main.py --mode fixed --snapshot simulation/build/fixed.png
python simulation/main.py --mode general --snapshot simulation/build/general.png
```

Expected: both commands exit 0, print absolute PNG paths, and create nonempty images.

- [ ] **Step 8: Commit the dashboard**

```powershell
git add simulation/visualization.py simulation/main.py simulation/tests/test_visualization.py
git commit -m "feat(simulation): visualize decisions and trajectories"
```

---

### Task 5: Full Verification and Repository Hygiene

**Files:**
- Verify only: all files under `simulation/`
- Verify only: existing embedded project sources remain unchanged

**Interfaces:**
- Consumes: all prior task outputs.
- Produces: repeatable passing tests and two verified PNG previews in ignored `simulation/build/`.

- [ ] **Step 1: Run the complete Python suite from a clean native build**

Remove only generated `simulation/build/arm_algorithms.dll`, then run:

```powershell
Remove-Item -LiteralPath simulation/build/arm_algorithms.dll -ErrorAction SilentlyContinue
python -m unittest discover simulation/tests -v
```

Expected: the DLL rebuilds automatically and all tests pass.

- [ ] **Step 2: Generate both final preview images**

```powershell
python simulation/main.py --mode fixed --snapshot simulation/build/fixed.png
python simulation/main.py --mode general --snapshot simulation/build/general.png
```

Expected: both commands exit 0 and both PNG files are nonblank.

- [ ] **Step 3: Inspect both previews**

Open `simulation/build/fixed.png` and `simulation/build/general.png`. Verify all four panels render, labels fit their axes, target outlines are visible, paths are framed, status text does not overlap controls, and warning text is readable.

- [ ] **Step 4: Verify the embedded firmware still builds**

Run:

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

Expected: ARM firmware links successfully with the configured `arm-none-eabi-gcc` toolchain.

- [ ] **Step 5: Check scope and generated-file hygiene**

Run:

```powershell
git status --short
git diff --check
git diff --name-only HEAD~4..HEAD
```

Expected: source changes are confined to `simulation/` plus the two temporary approved design and implementation documents; generated DLL, PNG, and cache files are not tracked.

- [ ] **Step 6: Remove the temporary process documents**

After all implementation and verification commands pass, remove exactly these two files as requested by the user:

```powershell
Remove-Item -LiteralPath docs/superpowers/specs/2026-07-29-decision-trajectory-simulation-design.md
Remove-Item -LiteralPath docs/superpowers/plans/2026-07-29-decision-trajectory-simulation-implementation.md
```

If their parent directories are empty, leave the empty directories alone because Git does not track them. Commit only the two deletions:

```powershell
git add -- docs/superpowers/specs/2026-07-29-decision-trajectory-simulation-design.md docs/superpowers/plans/2026-07-29-decision-trajectory-simulation-implementation.md
git commit -m "chore: remove simulation process documents"
```

- [ ] **Step 7: Record verification completion**

If verification requires no source fix, do not create an empty commit. If a verification-discovered fix was necessary, commit only that focused fix after rerunning the complete suite:

```powershell
git add simulation
git commit -m "fix(simulation): resolve verification findings"
```

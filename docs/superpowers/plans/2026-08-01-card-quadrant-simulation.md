# Card Quadrant Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deterministic host simulation that cuts a real playing-card bitmap into four equal quadrants, derives sparse red/black seam features, calls the native STM32 card solver, and visualizes the reconstructed card.

**Architecture:** Python owns image loading, quadrant texture metadata, and conversion of known cut-line pixels into the same sparse feature structures transmitted by MaixCAM. The existing GCC-built DLL remains the only decision and trajectory implementation. The scenario layer selects `Decision_SolveCard`, and Matplotlib renders the actual quadrant textures at the poses returned by the native solver.

**Tech Stack:** Python 3, ctypes, NumPy, OpenCV, Matplotlib, C11/GCC, native STM32 decision and trajectory sources.

## Global Constraints

- Preserve `general` as the default scenario.
- Use a repository-local PNG by default and accept a local `--card-image` override.
- The card is `100 x 70 mm`, cut into four `50 x 35 mm` rectangles.
- Do not duplicate the layout search or trajectory planner in Python.
- Keep image feature extraction deterministic and independent of the GUI.
- Verify the native layout by topology and reconstructed texture, not by checking private implementation details.

---

### Task 1: Synchronize the Python/C ABI

**Files:**
- Modify: `simulation/bindings.py`
- Modify: `simulation/native_bridge.c`
- Create: `tests/simulation/test_bindings.py`

**Interfaces:**
- Produces `DecisionCardEdgeEvent`, `DecisionCardPrimitive`, `DecisionCardPieceFeatures`, and `DecisionCardFrame` ctypes structures.
- Produces `DecisionPlan.search_nodes` and configured `Decision_SolveCard()` binding.
- Produces `assert_native_layout(library) -> None` that compares C sizes and offsets with ctypes.

- [x] Write a failing test that loads the native library, calls `assert_native_layout`, checks `DecisionPlan.search_nodes`, and verifies a zeroed `DecisionCardFrame` can be passed to `Decision_SolveCard` without corrupting memory.
- [x] Run `python -m pytest tests/simulation/test_bindings.py -q` and confirm failure because card ctypes types or ABI exports are missing.
- [x] Export card structure sizes and `DecisionPlan.search_nodes`/`moves` offsets from `native_bridge.c`; define exact ctypes layouts and bind `Decision_SolveCard`.
- [x] Rebuild the native DLL and rerun the binding test until it passes.

### Task 2: Derive card features from a four-way bitmap cut

**Files:**
- Create: `simulation/card_image.py`
- Create: `simulation/assets/playing_card.png`
- Create: `tests/simulation/test_card_image.py`

**Interfaces:**
- Produces `CardImageData` containing normalized BGR/RGB image, four quadrant arrays, base polygons, paired edge events, and internal primitives.
- Produces `load_card_image(path: Path | None) -> CardImageData`.
- Produces `populate_card_features(card_frame: DecisionCardFrame, image: CardImageData, transforms) -> None`.

- [x] Add a real, repository-local face-card PNG with red and black artwork crossing both centre cuts.
- [x] Write a failing test that loads the asset and asserts all four internal adjacencies receive at least one same-color paired event with hand-checked edge mappings: TL-right/TR-left, TL-bottom/BL-top, TR-bottom/BR-top, BL-right/BR-left.
- [x] Run `python -m pytest tests/simulation/test_card_image.py -q` and confirm failure because `card_image` is absent.
- [x] Implement HSV/gray classification, paired run extraction on the two known cut lines, q8 position conversion, tangent/width/confidence encoding, and bounded connected-component primitives.
- [x] Rerun the image tests and verify invalid/missing images fail with clear `ValueError`/`FileNotFoundError` messages.

### Task 3: Add the native card scenario and feasibility assertions

**Files:**
- Modify: `simulation/scenarios.py`
- Modify: `simulation/simulator.py`
- Create: `tests/simulation/test_card_scenario.py`

**Interfaces:**
- Extends `Scenario` with optional `card_frame`, `card_image`, `base_polygons`, and `expected_adjacencies` data.
- Extends `create_scenario(name="general", card_image=None)` with `name="card"`.
- `solve_scenario` calls `Decision_SolveCard` only when `scenario.card_frame` is present.
- Produces `validate_card_solution(scenario, plan) -> CardValidation` with topology, texture error, and node count.

- [x] Write a failing integration test that constructs the card scene, runs the actual DLL solver, asserts four moves, asserts the two original diagonal pairs remain diagonals, and requires `search_nodes < 5000`.
- [x] Run the test and confirm it fails because the scenario still calls `Decision_Solve` or lacks card data.
- [x] Build four rectangles in canonical TL/TR/BR/BL order, rotate/translate them into the A4 pick half, transform primitive coordinates, attach paired events, and call `Decision_SolveCard`.
- [x] Compute final polygons from native moves and validate all four intended adjacencies using centre distances and shared-edge geometry.
- [x] Reconstruct a canonical final bitmap from piece IDs and orientations; report mean texture error against the source image.
- [x] Rerun card integration and existing decision/protocol tests.

### Task 4: Render textured pieces and expose the CLI

**Files:**
- Modify: `simulation/visualization.py`
- Modify: `simulation/main.py`
- Create: `tests/simulation/test_card_cli.py`

**Interfaces:**
- CLI: `python -m simulation.main --scenario card [--card-image PATH] [--snapshot PATH]`.
- `SimulationView` creates an image artist per card quadrant and updates its affine transform from the current polygon pose.
- Snapshot includes card texture, search nodes, topology status, and seam error.

- [x] Write a failing CLI test that runs the card scenario with `MPLBACKEND=Agg`, requests a temporary snapshot, and asserts exit code zero plus a non-empty PNG.
- [x] Run it and confirm `--scenario` is currently rejected.
- [x] Add CLI arguments and pass them into `create_scenario`.
- [x] Add affine textured image artists for card quadrants while retaining the existing colored polygon path for general scenes.
- [x] Add concise card validation status to the board title/status area.
- [x] Run the CLI test, create `build/card-quadrant-simulation.png`, and inspect the image.

### Task 5: Full verification

**Files:**
- Modify: `progress.md`
- Modify: this plan, marking completed steps.

- [x] Run `python -m pytest tests/simulation -q`.
- [x] Run `python -m simulation.main --scenario card --snapshot build/card-quadrant-simulation.png`.
- [x] Run `ctest --test-dir build/decision-tests --output-on-failure` and `ctest --test-dir build/vision-protocol-tests --output-on-failure`.
- [x] Run `cmake --build --preset Debug -j 8`.
- [x] Run `git diff --check` and inspect both the generated screenshot and final diffs.
- [x] Record that camera lighting, physical cuts, UART, and mechanical placement remain hardware-only validation.

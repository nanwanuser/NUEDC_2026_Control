# Card Pattern Strategy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a separate playing-card puzzle strategy selected by Key2 while preserving the current Key1 geometric strategy.

**Architecture:** MaixCAM extracts calibrated polygon geometry, sparse cut-edge pattern events, and interior red/black primitives. STM32 receives a chunked card feature frame, precomputes edge compatibility, enumerates every geometrically valid layout, ranks complete layouts by pattern consistency, and sends the selected poses through the existing execution state machine.

**Tech Stack:** STM32F407 C11/HAL/FreeRTOS, native CMake tests, MaixPy Python/OpenCV, UART CRC16 protocol.

## Global Constraints

- Key1 behavior remains unchanged.
- Key2 selects an independent card solver.
- No OCR, neural network, or fixed playing-card template in this iteration.
- No production behavior is added without a failing test first.
- Card feature payloads remain split into frames compatible with the current receive buffer.

---

### Task 1: Card decision data model and scoring

**Files:** Modify `App/lib/decision/decision.h`, `App/lib/decision/decision.c`; test `tests/decision/decision_test.c`.

**Interface:** Add `DecisionCardFrame`, `DecisionCardConfig`, `Decision_SolveCard()`, and an ambiguity result. Edge-event matching reverses and offsets the second edge. Complete layouts receive seam and global primitive scores; the normal solver keeps its early exit.

- [x] Add a failing test with four identical rectangular pieces whose correct permutation is defined only by card features.
- [x] Run the decision test and confirm the card API is absent before implementation.
- [x] Implement card structures, complete-layout scoring, evidence ordering, and bounded early exit.
- [x] Add reversed traversal and no-evidence ambiguity tests.
- [x] Run all decision tests.

### Task 2: Chunked card vision protocol

**Files:** Modify `App/lib/vision_protocol/vision_protocol.h`, `App/lib/vision_protocol/vision_protocol.c`, `App/lib/vision_uart/vision_uart.c`; test `tests/vision_protocol/vision_protocol_test.c`.

**Interface:** Add card geometry/event/primitive/commit frame types carrying `layout_id`, chunk index/count, and aggregate checksum. The receiver publishes only a complete self-consistent `DecisionCardFrame`.

- [x] Add failing vectors for valid, missing, reordered, and corrupt chunks.
- [x] Implement parsing and bounded frame assembly without enlarging an individual UART frame beyond its current limit.
- [x] Require repeated identical committed layouts before submission.
- [x] Run protocol and decision tests.

### Task 3: Mission and execution integration

**Files:** Modify `Task/mission/mission.c`, `Task/decision/decision_task.h`, `Task/decision/decision_task.c`; add or extend native task tests.

**Interface:** Add a decision strategy discriminator to `DecisionTaskRequest`. `MISSION_GEOMETRIC` submits the existing vision frame to `Decision_Solve`; `MISSION_CARD_PATTERN` waits for a complete card frame and calls `Decision_SolveCard`. Both produce the existing `DecisionPlan` consumed by the same executor.

- [x] Add a dispatch test proving geometric mode does not require card data and card mode does.
- [x] Implement strategy dispatch; card ambiguity uses the existing no-solution run diagnosis.
- [x] Compile the integrated mission and decision tasks in the target firmware.

### Task 4: MaixCAM feature extraction and encoding

**Files:** Create `D:/Desktop/26-TI-MaixCAM/card_features.py`; modify `vision.py`, `main.py`, `serial_protocol.py`, `vision_uart.py`, `config.py`; create host-side tests under `D:/Desktop/26-TI-MaixCAM/tests`.

**Interface:** `extract_card_features(frame_bgr, measurements, calibration)` returns stable per-piece edge events and interior primitives tied to the same layout ID. `encode_card_feature_frames()` emits protocol-compatible chunks.

- [x] Add failing pure-data tests for chunk encoding and invalid edge references.
- [x] Add synthetic-image tests for a red crossing, white seam, and black primitive.
- [x] Implement feature extraction using the existing refined vertices and calibration.
- [x] Integrate frozen card results with the publisher while retaining geometric frames.
- [x] Run host-side tests and syntax compilation.

### Task 5: End-to-end verification

**Files:** Update protocol and decision READMEs plus `progress.md`.

- [ ] Run all native C tests.
- [x] Run all MaixCAM host-side tests available in the environment.
- [x] Build the STM32 firmware with the repository preset/toolchain.
- [ ] Inspect git diffs in both repositories for unrelated changes.
- [x] Record unverified hardware-only checks: real-card extraction accuracy, MaixCAM processing time, UART transfer time, and total mission time.

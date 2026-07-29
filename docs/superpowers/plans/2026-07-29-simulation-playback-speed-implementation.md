# Simulation 4x Playback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change automatic simulation playback from 1x to a fixed 4x rate while preserving the 40 ms GUI refresh interval.

**Architecture:** Add named playback timing constants to `SimulationView` and advance simulated time by `0.16 s` per timer callback. Exercise the callback directly in the existing Agg-backed visualization test.

**Tech Stack:** Python 3.13, unittest, Matplotlib 3.10.

## Global Constraints

- Do not modify C algorithms, sampled trajectory data, snapshot behavior, or the 40 ms timer interval.
- Do not add a speed control.
- Remove this plan and its design document after verification.

### Task 1: Fixed 4x Playback

**Files:**
- Modify: `simulation/tests/test_visualization.py`
- Modify: `simulation/visualization.py`

**Interfaces:**
- Produces: `PLAYBACK_RATE = 4.0` and `TIMER_INTERVAL_MS = 40`.
- Preserves: `SimulationView._on_timer()` wrapping to zero after total duration.

- [ ] Add a test that sets `playing=True`, calls `_on_timer()`, and checks `1.00 -> 1.16 s`.
- [ ] Add a test that starts within `0.16 s` of the end and checks the next callback wraps to `0 s`.
- [ ] Run the visualization test and verify it fails because playback still advances by `0.04 s`.
- [ ] Define named timing constants and compute the callback increment as `TIMER_INTERVAL_MS / 1000.0 * PLAYBACK_RATE`.
- [ ] Run the visualization test and complete test suite.
- [ ] Generate both mode snapshots and verify the ARM firmware still builds.
- [ ] Delete the temporary speed design and plan documents, then verify a clean worktree.

import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

from simulation.scenarios import (
    create_scenario,
    solve_scenario,
    validate_card_solution,
)


def test_native_card_solver_reconstructs_real_four_quadrant_card():
    scenario = create_scenario("card")
    plan = solve_scenario(scenario)
    validation = validate_card_solution(scenario, plan)

    assert plan.move_count == 4
    assert plan.search_nodes < 5000
    assert validation.topology_ok
    assert validation.alignment_rms_mm < 3.0
    assert validation.texture_mae < 45.0
    assert validation.actual_adjacencies == validation.expected_adjacencies


def test_general_scenario_still_uses_geometric_solver():
    scenario = create_scenario("general")
    plan = solve_scenario(scenario)

    assert scenario.card_frame is None
    assert plan.move_count == 4


def test_wrongly_spaced_layout_is_not_accepted_by_topology_or_texture():
    scenario = create_scenario("card")
    plan = solve_scenario(scenario)
    card_center = np.array((50.0, 35.0))
    for move_index in range(plan.move_count):
        move = plan.moves[move_index]
        piece_center = scenario.base_polygons[int(move.piece_id)].mean(axis=0)
        direction = piece_center - card_center
        direction /= np.linalg.norm(direction)
        move.place.x_mm += float(direction[0] * 25.0)
        move.place.y_mm += float(direction[1] * 25.0)

    validation = validate_card_solution(scenario, plan)

    assert not validation.topology_ok
    assert validation.actual_adjacencies != validation.expected_adjacencies
    assert validation.alignment_rms_mm > 20.0
    assert validation.texture_mae > 45.0


def test_inward_clearance_overlap_is_rejected_for_sparse_card():
    landscape = np.full((280, 400, 3), 255, dtype=np.uint8)
    cv2.rectangle(landscape, (194, 60), (206, 76), (0, 0, 0), -1)
    cv2.rectangle(landscape, (194, 202), (206, 218), (0, 0, 255), -1)
    cv2.rectangle(landscape, (82, 134), (98, 146), (0, 0, 255), -1)
    cv2.rectangle(landscape, (302, 134), (318, 146), (0, 0, 0), -1)
    portrait = cv2.rotate(landscape, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image_path = Path("build/sparse-seam-card.png").resolve()
    image_path.parent.mkdir(parents=True, exist_ok=True)
    assert cv2.imwrite(str(image_path), portrait)

    scenario = create_scenario("card", card_image=image_path)
    plan = solve_scenario(scenario)
    target_center = np.array(
        (scenario.config.target_center.x_mm, scenario.config.target_center.y_mm)
    )
    for move_index in range(plan.move_count):
        move = plan.moves[move_index]
        place = np.array((move.place.x_mm, move.place.y_mm))
        outward = place - target_center
        outward /= np.linalg.norm(outward)
        move.place.x_mm -= float(outward[0] * 4.0)
        move.place.y_mm -= float(outward[1] * 4.0)

    validation = validate_card_solution(scenario, plan)

    assert validation.alignment_rms_mm < 3.0
    assert validation.texture_mae < 45.0
    assert not validation.clearance_ok
    assert not validation.topology_ok


def test_general_scenario_import_does_not_require_opencv():
    script = r'''
import builtins
original_import = builtins.__import__
def guarded_import(name, *args, **kwargs):
    if name == "cv2" or name.startswith("cv2."):
        raise ImportError("cv2 intentionally unavailable")
    return original_import(name, *args, **kwargs)
builtins.__import__ = guarded_import
from simulation.scenarios import create_scenario, solve_scenario
scenario = create_scenario("general")
plan = solve_scenario(scenario)
assert scenario.card_frame is None and plan.move_count == 4
'''
    completed = subprocess.run(
        (sys.executable, "-c", script),
        capture_output=True,
        text=True,
        env=os.environ.copy(),
        timeout=120,
    )

    assert completed.returncode == 0, completed.stderr

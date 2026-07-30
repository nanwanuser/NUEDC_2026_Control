"""Deterministic built-in scene for the decision search."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from simulation.bindings import (
    DECISION_RESULT_OK,
    DecisionConfig,
    DecisionPlan,
    DecisionPoint,
    DecisionVisionFrame,
    TrajectoryLimits,
    TrajectoryPose,
    load_library,
)


BASE_POLYGONS = (
    ((0.0, 0.0), (100.0, 0.0), (50.0, 30.0)),
    ((100.0, 0.0), (100.0, 60.0), (50.0, 30.0)),
    ((100.0, 60.0), (0.0, 60.0), (50.0, 30.0)),
    ((0.0, 60.0), (0.0, 0.0), (50.0, 30.0)),
)
ANGLES_DEG = (-18.0, 71.0, 143.0, -96.0)
TRANSLATIONS_MM = (
    (30.0, 20.0),
    (155.0, 30.0),
    (45.0, 105.0),
    (165.0, 115.0),
)


@dataclass(frozen=True)
class Scenario:
    name: str
    frame: DecisionVisionFrame
    config: DecisionConfig
    home: TrajectoryPose
    limits: TrajectoryLimits
    target_polygons: Mapping[int, np.ndarray]


def _rotate_translate(
    vertices: Sequence[Sequence[float]], angle_deg: float, translation: Sequence[float]
) -> np.ndarray:
    angle = np.deg2rad(angle_deg)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    return np.asarray(vertices, dtype=float) @ rotation.T + np.asarray(translation)


def _fill_point(destination: DecisionPoint, point: Sequence[float]) -> None:
    destination.x_mm = float(point[0])
    destination.y_mm = float(point[1])


def _build_frame() -> DecisionVisionFrame:
    frame = DecisionVisionFrame()
    frame.seq = 20260729
    frame.piece_count = 4

    for index, base_polygon in enumerate(BASE_POLYGONS):
        vertices = _rotate_translate(
            base_polygon, ANGLES_DEG[index], TRANSLATIONS_MM[index]
        )
        piece = frame.pieces[index]
        piece.id = 10 + index
        piece.vertex_count = len(base_polygon)
        _fill_point(piece.center, vertices.mean(axis=0))
        for vertex_index, vertex in enumerate(vertices):
            _fill_point(piece.vertices[vertex_index], vertex)
    return frame


def create_scenario(name: str = "general") -> Scenario:
    """Create the built-in scene. Every contest task assembles pieces the device
    has not seen before, so there is only the edge-matching search to exercise."""

    library = load_library()
    config = DecisionConfig()
    library.Decision_GetDefaultConfig(ctypes.byref(config))

    return Scenario(
        name=name,
        frame=_build_frame(),
        config=config,
        home=TrajectoryPose(105.0, 120.0, 60.0, 0.0),
        limits=TrajectoryLimits(120.0, 300.0, 90.0, 240.0),
        target_polygons={},
    )


def solve_scenario(scenario: Scenario) -> DecisionPlan:
    """Run the native decision algorithm for one built-in scene."""

    plan = DecisionPlan()
    result = load_library().Decision_Solve(
        ctypes.byref(scenario.frame),
        ctypes.byref(scenario.config),
        ctypes.byref(plan),
    )
    if result != DECISION_RESULT_OK:
        raise RuntimeError(
            f"Decision_Solve failed for {scenario.name}: result={result}"
        )
    return plan


def transform_polygon(
    vertices: Sequence[DecisionPoint],
    pick: TrajectoryPose,
    place: TrajectoryPose,
) -> np.ndarray:
    """Apply the grasp-relative rigid transform represented by a move."""

    points = np.array([[point.x_mm, point.y_mm] for point in vertices], dtype=float)
    angle = np.deg2rad(place.yaw_deg)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    relative = points - [pick.x_mm, pick.y_mm]
    return relative @ rotation.T + [place.x_mm, place.y_mm]

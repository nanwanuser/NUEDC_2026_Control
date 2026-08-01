"""Deterministic built-in scene for the decision search."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Mapping, Sequence

import numpy as np

from simulation.bindings import (
    DECISION_CARD_MAX_EDGE_EVENTS_PER_PIECE,
    DECISION_CARD_MAX_PRIMITIVES_PER_PIECE,
    DECISION_RESULT_OK,
    DecisionCardFrame,
    DecisionConfig,
    DecisionPlan,
    DecisionPoint,
    DecisionVisionFrame,
    TrajectoryLimits,
    TrajectoryPose,
    load_library,
)

if TYPE_CHECKING:
    from simulation.card_image import CardImageData


BASE_POLYGONS = (
    ((0.0, 0.0), (100.0, 0.0), (50.0, 30.0)),
    ((100.0, 0.0), (100.0, 60.0), (50.0, 30.0)),
    ((100.0, 60.0), (0.0, 60.0), (50.0, 30.0)),
    ((0.0, 60.0), (0.0, 0.0), (50.0, 30.0)),
)
ANGLES_DEG = (-18.0, 71.0, 143.0, -96.0)
# Every piece centre has to land in the pick half of the sheet, x < 148.5: the
# assembly is built in the other half, and the solver refuses a frame whose pieces
# are measured there rather than trying to assemble on top of them. These are
# chosen to scatter the pieces while keeping each centre well inside that half.
TRANSLATIONS_MM = (
    (30.0, 20.0),
    (95.0, 55.0),
    (45.0, 105.0),
    (100.0, 150.0),
)


@dataclass(frozen=True)
class Scenario:
    name: str
    frame: DecisionVisionFrame
    config: DecisionConfig
    home: TrajectoryPose
    limits: TrajectoryLimits
    target_polygons: Mapping[int, np.ndarray]
    card_frame: DecisionCardFrame | None = None
    card_image: CardImageData | None = None
    base_polygons: Mapping[int, np.ndarray] = field(default_factory=dict)
    expected_adjacencies: frozenset[frozenset[int]] = frozenset()


@dataclass(frozen=True)
class CardValidation:
    topology_ok: bool
    clearance_ok: bool
    expected_adjacencies: frozenset[frozenset[int]]
    actual_adjacencies: frozenset[frozenset[int]]
    alignment_rms_mm: float
    texture_mae: float
    reconstructed_bgr: np.ndarray


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


def _normalize_feature_angle(angle_deg: float) -> int:
    angle = (float(angle_deg) + 90.0) % 180.0 - 90.0
    return int(round(angle))


def _scatter_card_piece(
    points: np.ndarray,
    angle_deg: float,
    center_mm: Sequence[float],
    source_center: Sequence[float] | None = None,
) -> np.ndarray:
    origin = (
        points.mean(axis=0)
        if source_center is None
        else np.asarray(source_center, dtype=float)
    )
    centered = points - origin
    angle = np.deg2rad(angle_deg)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    return centered @ rotation.T + np.asarray(center_mm, dtype=float)


def _build_card_frame(image_data: CardImageData) -> DecisionCardFrame:
    angles = (-18.0, 71.0, 143.0, -96.0)
    centers = ((35.0, 28.0), (105.0, 32.0), (38.0, 105.0), (108.0, 116.0))
    card = DecisionCardFrame()
    card.layout_id = 0x43415244
    card.piece_count = 4
    card.vision.seq = 20260801
    card.vision.piece_count = 4

    for index, piece_id in enumerate((1, 2, 3, 4)):
        angle = angles[index]
        base = image_data.base_polygons[piece_id]
        scattered = _scatter_card_piece(base, angle, centers[index])
        piece = card.vision.pieces[index]
        piece.id = piece_id
        piece.vertex_count = 4
        _fill_point(piece.center, scattered.mean(axis=0))
        for vertex_index, vertex in enumerate(scattered):
            _fill_point(piece.vertices[vertex_index], vertex)

        features = card.pieces[index]
        features.piece_id = piece_id
        source_events = image_data.edge_events[piece_id]
        features.edge_event_count = min(
            len(source_events), DECISION_CARD_MAX_EDGE_EVENTS_PER_PIECE
        )
        for event_index in range(features.edge_event_count):
            source = source_events[event_index]
            target = features.edge_events[event_index]
            target.edge_index = source.edge_index
            target.position_q8 = source.position_q8
            target.color = source.color
            target.tangent_deg = _normalize_feature_angle(
                source.tangent_deg + angle
            )
            target.width_q4_mm = source.width_q4_mm
            target.confidence = source.confidence

        source_primitives = image_data.primitives[piece_id]
        features.primitive_count = min(
            len(source_primitives), DECISION_CARD_MAX_PRIMITIVES_PER_PIECE
        )
        for primitive_index in range(features.primitive_count):
            source = source_primitives[primitive_index]
            target = features.primitives[primitive_index]
            scattered_center = _scatter_card_piece(
                np.asarray((source.center_mm,), dtype=float),
                angle,
                centers[index],
                base.mean(axis=0),
            )[0]
            _fill_point(target.center, scattered_center)
            target.area_mm2 = source.area_mm2
            target.color = source.color
            target.kind = source.kind
            target.angle_deg = _normalize_feature_angle(
                source.angle_deg + angle
            )
            target.confidence = source.confidence
    return card


def create_scenario(
    name: str = "general", card_image: Path | None = None
) -> Scenario:
    """Create a geometric or real-bitmap card scene."""

    library = load_library()
    config = DecisionConfig()
    library.Decision_GetDefaultConfig(ctypes.byref(config))

    if name == "card":
        from simulation.card_image import load_card_image

        image_data = load_card_image(card_image)
        card_frame = _build_card_frame(image_data)
        expected = frozenset(
            frozenset(pair) for pair in ((1, 2), (1, 4), (2, 3), (3, 4))
        )
        return Scenario(
            name=name,
            frame=card_frame.vision,
            config=config,
            home=TrajectoryPose(105.0, 120.0, 60.0, 0.0),
            limits=TrajectoryLimits(120.0, 300.0, 90.0, 240.0),
            target_polygons={},
            card_frame=card_frame,
            card_image=image_data,
            base_polygons=image_data.base_polygons,
            expected_adjacencies=expected,
        )
    if name != "general":
        raise ValueError(f"unknown simulation scenario: {name}")

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
    if scenario.card_frame is None:
        result = load_library().Decision_Solve(
            ctypes.byref(scenario.frame),
            ctypes.byref(scenario.config),
            ctypes.byref(plan),
        )
        entry_point = "Decision_Solve"
    else:
        result = load_library().Decision_SolveCard(
            ctypes.byref(scenario.card_frame),
            ctypes.byref(scenario.config),
            ctypes.byref(plan),
        )
        entry_point = "Decision_SolveCard"
    if result != DECISION_RESULT_OK:
        raise RuntimeError(
            f"{entry_point} failed for {scenario.name}: result={result}"
        )
    return plan


def _final_polygons(
    scenario: Scenario, plan: DecisionPlan
) -> dict[int, np.ndarray]:
    pieces = {
        int(scenario.frame.pieces[index].id): scenario.frame.pieces[index]
        for index in range(scenario.frame.piece_count)
    }
    result: dict[int, np.ndarray] = {}
    for move_index in range(plan.move_count):
        move = plan.moves[move_index]
        piece_id = int(move.piece_id)
        piece = pieces[piece_id]
        result[piece_id] = transform_polygon(
            piece.vertices[: piece.vertex_count], move.pick, move.place
        )
    return result


def _rigid_alignment(
    final_polygons: Mapping[int, np.ndarray],
    base_polygons: Mapping[int, np.ndarray],
) -> tuple[dict[int, np.ndarray], float]:
    final_points = np.concatenate([final_polygons[index] for index in range(1, 5)])
    base_points = np.concatenate([base_polygons[index] for index in range(1, 5)])
    final_center = final_points.mean(axis=0)
    base_center = base_points.mean(axis=0)
    covariance = (final_points - final_center).T @ (base_points - base_center)
    left, _, right = np.linalg.svd(covariance)
    rotation = left @ right
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right
    aligned = {
        piece_id: (polygon - final_center) @ rotation + base_center
        for piece_id, polygon in final_polygons.items()
    }
    residuals = np.concatenate(
        [aligned[index] - base_polygons[index] for index in range(1, 5)]
    )
    return aligned, float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))


def _reconstruct_texture(
    scenario: Scenario, aligned: Mapping[int, np.ndarray]
) -> tuple[np.ndarray, float]:
    import cv2

    assert scenario.card_image is not None
    source = scenario.card_image.image_bgr
    height, width = source.shape[:2]
    canvas = np.full_like(source, 255)
    occupied = np.zeros((height, width), dtype=np.uint8)
    for piece_id in range(1, 5):
        texture = scenario.card_image.quadrants_bgr[piece_id]
        texture_height, texture_width = texture.shape[:2]
        source_corners = np.float32(
            ((0, 0), (texture_width - 1, 0),
             (texture_width - 1, texture_height - 1), (0, texture_height - 1))
        )
        aligned_polygon = np.asarray(aligned[piece_id], dtype=np.float32)
        card_center = np.concatenate(
            tuple(scenario.base_polygons.values())
        ).mean(axis=0)
        base_center = scenario.base_polygons[piece_id].mean(axis=0)
        outward = base_center - card_center
        outward /= np.linalg.norm(outward)
        clearance = float(
            load_library().Simulation_DecisionAssemblyClearanceMm()
        )
        # Invert only the firmware's fixed radial clearance model. Unlike
        # recentering each piece, this preserves every unexpected placement error.
        destination = aligned_polygon - outward.astype(np.float32) * clearance
        destination[:, 0] *= width / 100.0
        destination[:, 1] *= height / 70.0
        matrix = cv2.getPerspectiveTransform(source_corners, destination)
        warped = cv2.warpPerspective(texture, matrix, (width, height))
        mask = cv2.warpPerspective(
            np.full((texture_height, texture_width), 255, dtype=np.uint8),
            matrix,
            (width, height),
        )
        selection = mask > 127
        canvas[selection] = warped[selection]
        occupied[selection] = 255
    difference = np.abs(canvas.astype(np.int16) - source.astype(np.int16))
    missing_penalty = (occupied == 0)[:, :, None] * 255
    error = np.maximum(difference, missing_penalty)
    return canvas, float(error.mean())


def _segments_form_shared_edge(
    first_start: np.ndarray,
    first_end: np.ndarray,
    first_center: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
    second_center: np.ndarray,
    maximum_gap_mm: float,
) -> bool:
    first_vector = first_end - first_start
    second_vector = second_end - second_start
    first_length = float(np.linalg.norm(first_vector))
    second_length = float(np.linalg.norm(second_vector))
    if first_length < 1.0 or second_length < 1.0:
        return False
    tangent = first_vector / first_length
    second_tangent = second_vector / second_length
    if abs(float(np.dot(tangent, second_tangent))) < np.cos(np.deg2rad(3.0)):
        return False

    normal = np.array((-tangent[1], tangent[0]))
    if float(np.dot(second_center - first_center, normal)) < 0.0:
        normal = -normal
    line_gap = float(np.dot(second_start - first_start, normal))
    if line_gap < -0.25 or line_gap > maximum_gap_mm:
        return False

    first_projection = sorted(
        (float(np.dot(first_start, tangent)), float(np.dot(first_end, tangent)))
    )
    second_projection = sorted(
        (float(np.dot(second_start, tangent)), float(np.dot(second_end, tangent)))
    )
    overlap = min(first_projection[1], second_projection[1]) - max(
        first_projection[0], second_projection[0]
    )
    return overlap >= 0.8 * min(first_length, second_length)


def _edge_adjacencies(
    polygons: Mapping[int, np.ndarray], clearance_mm: float
) -> frozenset[frozenset[int]]:
    adjacent: set[frozenset[int]] = set()
    piece_ids = sorted(polygons)
    maximum_gap = 2.0 * clearance_mm + 0.75
    for left_index, left_id in enumerate(piece_ids):
        left = polygons[left_id]
        for right_id in piece_ids[left_index + 1 :]:
            right = polygons[right_id]
            shares_edge = any(
                _segments_form_shared_edge(
                    left[edge],
                    left[(edge + 1) % len(left)],
                    left.mean(axis=0),
                    right[other_edge],
                    right[(other_edge + 1) % len(right)],
                    right.mean(axis=0),
                    maximum_gap,
                )
                for edge in range(len(left))
                for other_edge in range(len(right))
            )
            if shares_edge:
                adjacent.add(frozenset((left_id, right_id)))
    return frozenset(adjacent)


def _clearance_matches_firmware(
    aligned: Mapping[int, np.ndarray],
    base_polygons: Mapping[int, np.ndarray],
    clearance_mm: float,
) -> bool:
    card_center = np.concatenate(tuple(base_polygons.values())).mean(axis=0)
    tolerance_mm = 0.35
    for piece_id, polygon in aligned.items():
        base_center = base_polygons[piece_id].mean(axis=0)
        outward = base_center - card_center
        outward /= np.linalg.norm(outward)
        offset = polygon.mean(axis=0) - base_center
        radial = float(np.dot(offset, outward))
        lateral = float(
            abs(outward[0] * offset[1] - outward[1] * offset[0])
        )
        if (
            abs(radial - clearance_mm) > tolerance_mm
            or lateral > tolerance_mm
        ):
            return False
    return True


def validate_card_solution(
    scenario: Scenario, plan: DecisionPlan
) -> CardValidation:
    """Validate topology, rigid reconstruction, and actual bitmap continuity."""

    if scenario.card_frame is None or scenario.card_image is None:
        raise ValueError("card validation requires a card scenario")
    final_polygons = _final_polygons(scenario, plan)
    clearance = float(load_library().Simulation_DecisionAssemblyClearanceMm())
    actual = _edge_adjacencies(final_polygons, clearance)
    aligned, alignment_rms = _rigid_alignment(
        final_polygons, scenario.base_polygons
    )
    clearance_ok = _clearance_matches_firmware(
        aligned, scenario.base_polygons, clearance
    )
    reconstructed, texture_mae = _reconstruct_texture(scenario, aligned)
    return CardValidation(
        topology_ok=(
            actual == scenario.expected_adjacencies and clearance_ok
        ),
        clearance_ok=clearance_ok,
        expected_adjacencies=scenario.expected_adjacencies,
        actual_adjacencies=actual,
        alignment_rms_mm=alignment_rms,
        texture_mae=texture_mae,
        reconstructed_bgr=reconstructed,
    )


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

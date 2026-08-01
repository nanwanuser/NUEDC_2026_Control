"""Deterministic playing-card bitmap cutting and sparse feature extraction."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from simulation.bindings import (
    DECISION_CARD_COLOR_BLACK,
    DECISION_CARD_COLOR_RED,
    DECISION_CARD_PRIMITIVE_DOT,
    DECISION_CARD_PRIMITIVE_GLYPH,
    DECISION_CARD_PRIMITIVE_LINE,
)


CARD_WIDTH_MM = 100.0
CARD_HEIGHT_MM = 70.0
NORMALIZED_WIDTH_PX = 400
NORMALIZED_HEIGHT_PX = 280
SEAM_BAND_PX = 6
MIN_RUN_PX = 2
MAX_EVENTS_PER_PIECE = 24
MAX_PRIMITIVES_PER_PIECE = 16
DEFAULT_CARD_IMAGE = Path(__file__).resolve().parent / "assets" / "playing_card.png"
INTERNAL_SEAMS = (
    ("TL-right/TR-left", 1, 1, 2, 3),
    ("TL-bottom/BL-top", 1, 2, 4, 0),
    ("TR-bottom/BR-top", 2, 2, 3, 0),
    ("BL-right/BR-left", 4, 1, 3, 3),
)


@dataclass(frozen=True)
class CardEdgeEventData:
    edge_index: int
    position_q8: int
    color: int
    tangent_deg: int
    width_q4_mm: int
    confidence: int


@dataclass(frozen=True)
class CardPrimitiveData:
    center_mm: tuple[float, float]
    area_mm2: float
    color: int
    kind: int
    angle_deg: int
    confidence: int


@dataclass(frozen=True)
class CardImageData:
    source_path: Path
    image_bgr: np.ndarray
    quadrants_bgr: dict[int, np.ndarray]
    base_polygons: dict[int, np.ndarray]
    edge_events: dict[int, tuple[CardEdgeEventData, ...]]
    primitives: dict[int, tuple[CardPrimitiveData, ...]]


def _pattern_masks(image_bgr: np.ndarray) -> dict[int, np.ndarray]:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    red = (
        ((hue <= 12) | (hue >= 165))
        & (saturation >= 80)
        & (value >= 35)
    )
    black = (gray <= 75) & ~red
    return {
        DECISION_CARD_COLOR_RED: red,
        DECISION_CARD_COLOR_BLACK: black,
    }


def _composite_to_bgr(image: np.ndarray, source_path: Path) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.ndim != 3:
        raise ValueError(f"playing-card image has unsupported channels: {source_path}")
    if image.shape[2] == 3:
        return image
    if image.shape[2] != 4:
        raise ValueError(f"playing-card image has unsupported channels: {source_path}")

    alpha = image[:, :, 3:4].astype(np.float32) / 255.0
    foreground = image[:, :, :3].astype(np.float32)
    return np.rint(foreground * alpha + 255.0 * (1.0 - alpha)).astype(np.uint8)


def _validate_internal_seams(
    edge_events: dict[int, tuple[CardEdgeEventData, ...]],
) -> None:
    missing: list[str] = []
    for label, first_id, first_edge, second_id, second_edge in INTERNAL_SEAMS:
        first = tuple(
            event
            for event in edge_events[first_id]
            if event.edge_index == first_edge
        )
        second = tuple(
            event
            for event in edge_events[second_id]
            if event.edge_index == second_edge
        )
        paired = any(
            left.color == right.color
            and abs(left.position_q8 + right.position_q8 - 255) <= 2
            for left in first
            for right in second
        )
        if not paired:
            missing.append(label)
    if missing:
        raise ValueError(
            "playing-card image lacks paired pattern evidence on internal seam(s): "
            + ", ".join(missing)
        )


def _continuous_runs(active: np.ndarray) -> list[tuple[int, int]]:
    indices = np.flatnonzero(active)
    if len(indices) == 0:
        return []
    runs: list[tuple[int, int]] = []
    start = int(indices[0])
    previous = start
    for raw_index in indices[1:]:
        index = int(raw_index)
        if index > previous + 2:
            if previous - start + 1 >= MIN_RUN_PX:
                runs.append((start, previous + 1))
            start = index
        previous = index
    if previous - start + 1 >= MIN_RUN_PX:
        runs.append((start, previous + 1))
    return runs


def _append_pair(
    events: dict[int, list[CardEdgeEventData]],
    first_piece: int,
    first_edge: int,
    second_piece: int,
    second_edge: int,
    fraction: float,
    color: int,
    tangent_deg: int,
    width_mm: float,
) -> None:
    if fraction <= 0.03 or fraction >= 0.97:
        return
    position = max(0, min(255, int(round(fraction * 255.0))))
    reversed_position = 255 - position
    width_q4 = max(1, min(255, int(round(width_mm * 4.0))))
    common = {
        "color": color,
        "tangent_deg": tangent_deg,
        "width_q4_mm": width_q4,
        "confidence": 245,
    }
    events[first_piece].append(
        CardEdgeEventData(
            edge_index=first_edge,
            position_q8=position,
            **common,
        )
    )
    events[second_piece].append(
        CardEdgeEventData(
            edge_index=second_edge,
            position_q8=reversed_position,
            **common,
        )
    )


def _extract_seam_events(
    masks: dict[int, np.ndarray],
) -> dict[int, tuple[CardEdgeEventData, ...]]:
    height, width = next(iter(masks.values())).shape
    middle_x = width // 2
    middle_y = height // 2
    events: dict[int, list[CardEdgeEventData]] = {
        1: [],
        2: [],
        3: [],
        4: [],
    }

    for color, mask in masks.items():
        left_band = mask[:, middle_x - SEAM_BAND_PX : middle_x]
        right_band = mask[:, middle_x : middle_x + SEAM_BAND_PX]
        vertical = left_band.any(axis=1) & right_band.any(axis=1)
        for start, end in _continuous_runs(vertical):
            center = 0.5 * (start + end - 1)
            run_width_mm = (end - start) * CARD_HEIGHT_MM / height
            if center < middle_y:
                _append_pair(
                    events, 1, 1, 2, 3,
                    center / middle_y,
                    color, 0, run_width_mm,
                )
            else:
                _append_pair(
                    events, 4, 1, 3, 3,
                    (center - middle_y) / (height - middle_y),
                    color, 0, run_width_mm,
                )

        top_band = mask[middle_y - SEAM_BAND_PX : middle_y, :]
        bottom_band = mask[middle_y : middle_y + SEAM_BAND_PX, :]
        horizontal = top_band.any(axis=0) & bottom_band.any(axis=0)
        for start, end in _continuous_runs(horizontal):
            center = 0.5 * (start + end - 1)
            run_width_mm = (end - start) * CARD_WIDTH_MM / width
            if center < middle_x:
                _append_pair(
                    events, 4, 0, 1, 2,
                    center / middle_x,
                    color, 90, run_width_mm,
                )
            else:
                _append_pair(
                    events, 3, 0, 2, 2,
                    (center - middle_x) / (width - middle_x),
                    color, 90, run_width_mm,
                )

    return {
        piece_id: tuple(piece_events[:MAX_EVENTS_PER_PIECE])
        for piece_id, piece_events in events.items()
    }


def _primitive_kind(points: np.ndarray, area: int) -> int:
    rectangle = cv2.minAreaRect(points.astype(np.float32).reshape(-1, 1, 2))
    short_side = max(min(rectangle[1]), 1.0)
    if max(rectangle[1]) / short_side >= 3.0:
        return DECISION_CARD_PRIMITIVE_LINE
    perimeter = cv2.arcLength(
        cv2.convexHull(points.astype(np.int32).reshape(-1, 1, 2)), True
    )
    circularity = 0.0 if perimeter <= 0.0 else (
        4.0 * np.pi * area / (perimeter * perimeter)
    )
    return (
        DECISION_CARD_PRIMITIVE_DOT
        if circularity >= 0.65
        else DECISION_CARD_PRIMITIVE_GLYPH
    )


def _extract_primitives(
    masks: dict[int, np.ndarray],
) -> dict[int, tuple[CardPrimitiveData, ...]]:
    height, width = next(iter(masks.values())).shape
    half_width = width // 2
    half_height = height // 2
    origins = {
        1: (0, 0),
        2: (half_width, 0),
        3: (half_width, half_height),
        4: (0, half_height),
    }
    result: dict[int, list[CardPrimitiveData]] = {1: [], 2: [], 3: [], 4: []}
    for piece_id, (origin_x, origin_y) in origins.items():
        for color, mask in masks.items():
            roi = mask[
                origin_y : origin_y + half_height,
                origin_x : origin_x + half_width,
            ].astype(np.uint8) * 255
            count, labels, stats, centroids = cv2.connectedComponentsWithStats(
                roi, 8
            )
            candidates = sorted(
                range(1, count),
                key=lambda label: int(stats[label, cv2.CC_STAT_AREA]),
                reverse=True,
            )
            for label in candidates:
                area_px = int(stats[label, cv2.CC_STAT_AREA])
                if area_px < 20:
                    continue
                ys, xs = np.nonzero(labels == label)
                points = np.column_stack((xs, ys))
                center_x = origin_x + float(centroids[label][0])
                center_y = origin_y + float(centroids[label][1])
                area_mm2 = area_px * CARD_WIDTH_MM * CARD_HEIGHT_MM / (width * height)
                result[piece_id].append(
                    CardPrimitiveData(
                        center_mm=(
                            center_x * CARD_WIDTH_MM / width,
                            center_y * CARD_HEIGHT_MM / height,
                        ),
                        area_mm2=float(area_mm2),
                        color=color,
                        kind=_primitive_kind(points, area_px),
                        angle_deg=0,
                        confidence=min(255, 160 + min(area_px, 95)),
                    )
                )
    return {
        piece_id: tuple(items[:MAX_PRIMITIVES_PER_PIECE])
        for piece_id, items in result.items()
    }


def load_card_image(path: Path | None = None) -> CardImageData:
    """Load, rotate to landscape, cut, and describe a real playing-card image."""

    source_path = Path(path) if path is not None else DEFAULT_CARD_IMAGE
    if not source_path.is_file():
        raise FileNotFoundError(f"playing-card image not found: {source_path}")
    image = cv2.imread(str(source_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"playing-card image could not be decoded: {source_path}")
    if min(image.shape[:2]) < 40:
        raise ValueError(f"playing-card image is too small: {source_path}")
    image = _composite_to_bgr(image, source_path)

    landscape = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    normalized = cv2.resize(
        landscape,
        (NORMALIZED_WIDTH_PX, NORMALIZED_HEIGHT_PX),
        interpolation=cv2.INTER_AREA,
    )
    middle_x = NORMALIZED_WIDTH_PX // 2
    middle_y = NORMALIZED_HEIGHT_PX // 2
    quadrants = {
        1: normalized[:middle_y, :middle_x].copy(),
        2: normalized[:middle_y, middle_x:].copy(),
        3: normalized[middle_y:, middle_x:].copy(),
        4: normalized[middle_y:, :middle_x].copy(),
    }
    base_polygons = {
        1: np.array(((0.0, 0.0), (50.0, 0.0), (50.0, 35.0), (0.0, 35.0))),
        2: np.array(((50.0, 0.0), (100.0, 0.0), (100.0, 35.0), (50.0, 35.0))),
        3: np.array(((50.0, 35.0), (100.0, 35.0), (100.0, 70.0), (50.0, 70.0))),
        4: np.array(((0.0, 35.0), (50.0, 35.0), (50.0, 70.0), (0.0, 70.0))),
    }
    masks = _pattern_masks(normalized)
    edge_events = _extract_seam_events(masks)
    _validate_internal_seams(edge_events)
    return CardImageData(
        source_path=source_path.resolve(),
        image_bgr=normalized,
        quadrants_bgr=quadrants,
        base_polygons=base_polygons,
        edge_events=edge_events,
        primitives=_extract_primitives(masks),
    )

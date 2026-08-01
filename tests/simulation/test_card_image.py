from pathlib import Path

import cv2
import numpy as np
import pytest

from simulation import card_image
from simulation.card_image import load_card_image


ASSET = Path("simulation/assets/playing_card.png")


def _events_on(data, piece_id, edge_index):
    return [
        event
        for event in data.edge_events[piece_id]
        if event.edge_index == edge_index
    ]


@pytest.mark.parametrize(
    ("left_id", "left_edge", "right_id", "right_edge"),
    (
        (1, 1, 2, 3),
        (1, 2, 4, 0),
        (2, 2, 3, 0),
        (4, 1, 3, 3),
    ),
)
def test_real_card_cut_produces_paired_events(
    left_id, left_edge, right_id, right_edge
):
    data = load_card_image(ASSET)
    left = _events_on(data, left_id, left_edge)
    right = _events_on(data, right_id, right_edge)

    assert left
    assert right
    assert any(
        first.color == second.color
        and abs(first.position_q8 + second.position_q8 - 255) <= 2
        for first in left
        for second in right
    )


def test_missing_card_image_reports_the_path():
    missing = Path("simulation/assets/missing-card.png")

    with pytest.raises(FileNotFoundError, match="missing-card.png"):
        load_card_image(missing)


def test_card_without_pattern_evidence_on_every_internal_seam_is_rejected(
    monkeypatch,
):
    image = np.full((314, 226, 3), 255, dtype=np.uint8)
    cv2.circle(image, (45, 45), 14, (0, 0, 0), -1)
    cv2.circle(image, (180, 260), 14, (0, 0, 255), -1)
    monkeypatch.setattr(card_image.cv2, "imread", lambda *_args, **_kwargs: image)

    with pytest.raises(ValueError, match="internal seam"):
        load_card_image(ASSET)


def test_transparent_card_pixels_are_composited_onto_white(monkeypatch):
    landscape = np.full((280, 400, 4), 255, dtype=np.uint8)
    landscape[:12, :12, :3] = 0
    landscape[:12, :12, 3] = 0
    cv2.rectangle(landscape, (192, 50), (208, 90), (0, 0, 0, 255), -1)
    cv2.rectangle(landscape, (192, 190), (208, 230), (0, 0, 0, 255), -1)
    cv2.rectangle(landscape, (70, 132), (130, 148), (0, 0, 0, 255), -1)
    cv2.rectangle(landscape, (270, 132), (330, 148), (0, 0, 0, 255), -1)
    image = cv2.rotate(landscape, cv2.ROTATE_90_COUNTERCLOCKWISE)
    monkeypatch.setattr(card_image.cv2, "imread", lambda *_args, **_kwargs: image)

    data = load_card_image(ASSET)

    assert data.image_bgr.shape[2] == 3
    assert np.all(data.image_bgr[0, 0] >= 250)

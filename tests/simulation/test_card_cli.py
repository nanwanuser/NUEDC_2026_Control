import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from simulation.main import _card_failure_reason


def test_card_cli_writes_snapshot_with_native_result():
    output = Path("build/card-cli-test.png").resolve()
    environment = os.environ.copy()
    environment["MPLBACKEND"] = "Agg"

    completed = subprocess.run(
        (
            sys.executable,
            "-m",
            "simulation.main",
            "--scenario",
            "card",
            "--snapshot",
            str(output),
        ),
        capture_output=True,
        text=True,
        env=environment,
        timeout=120,
    )

    assert completed.returncode == 0, completed.stderr
    assert output.is_file()
    assert output.stat().st_size > 20_000
    assert "nodes=" in completed.stdout
    assert "topology=OK" in completed.stdout


def test_card_cli_failure_reason_covers_invalid_metrics_and_node_budget():
    result = SimpleNamespace(
        decision_plan=SimpleNamespace(search_nodes=5000),
        card_validation=SimpleNamespace(
            topology_ok=False,
            alignment_rms_mm=43.0,
            texture_mae=80.0,
        ),
    )

    reason = _card_failure_reason(result)

    assert "topology" in reason
    assert "alignment" in reason
    assert "texture" in reason
    assert "search nodes" in reason


def test_card_cli_rejects_image_without_internal_seam_evidence():
    image_path = Path("build/card-without-seam-evidence.png").resolve()
    image_path.parent.mkdir(parents=True, exist_ok=True)
    image = np.full((314, 226, 3), 255, dtype=np.uint8)
    cv2.circle(image, (45, 45), 14, (0, 0, 0), -1)
    cv2.circle(image, (180, 260), 14, (0, 0, 255), -1)
    assert cv2.imwrite(str(image_path), image)

    completed = subprocess.run(
        (
            sys.executable,
            "-m",
            "simulation.main",
            "--scenario",
            "card",
            "--card-image",
            str(image_path),
            "--snapshot",
            str(Path("build/invalid-card.png").resolve()),
        ),
        capture_output=True,
        text=True,
        env=os.environ.copy(),
        timeout=120,
    )

    assert completed.returncode == 1
    assert "internal seam" in completed.stderr

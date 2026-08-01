import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MISSION_SOURCE = ROOT / "Core" / "Src" / "freertos.c"
MISSION_STACK_REPORT = (
    ROOT
    / "build"
    / "Debug"
    / "CMakeFiles"
    / "NUEDC_2026_Control.dir"
    / "Task"
    / "mission"
    / "mission.c.su"
)
CALL_MARGIN_BYTES = 256


def _mission_stack_bytes():
    source = MISSION_SOURCE.read_text(encoding="utf-8")
    match = re.search(
        r"Mission_attributes\s*=\s*\{.*?"
        r"\.stack_size\s*=\s*(\d+)\s*\*\s*(\d+)",
        source,
        flags=re.DOTALL,
    )
    assert match is not None, "cannot find Mission task stack size"
    return int(match.group(1)) * int(match.group(2))


def _function_stack_bytes(name):
    assert MISSION_STACK_REPORT.exists(), (
        "build Debug firmware first to generate mission.c.su"
    )
    for line in MISSION_STACK_REPORT.read_text(encoding="utf-8").splitlines():
        columns = line.rsplit("\t", 2)
        if len(columns) == 3 and columns[0].endswith(":" + name):
            return int(columns[1])
    raise AssertionError("cannot find stack usage for " + name)


def test_mission_arm_path_fits_allocated_task_stack():
    required = (
        _function_stack_bytes("Mission_App")
        + _function_stack_bytes("arm_mission")
        + CALL_MARGIN_BYTES
    )

    assert required <= _mission_stack_bytes(), (
        "Mission key path needs at least {} bytes, task has {} bytes".format(
            required, _mission_stack_bytes()
        )
    )

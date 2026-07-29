"""C-backed multi-piece execution sequencing and trajectory metrics."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass

import numpy as np

from simulation.bindings import (
    TRAJECTORY_PHASE_APPROACH,
    TRAJECTORY_PHASE_TRANSFER,
    TRAJECTORY_RESULT_OK,
    TRAJECTORY_STATE_COMPLETE,
    DecisionMove,
    DecisionPlan,
    TrajectoryPhasePlan,
    TrajectoryPlan,
    TrajectoryPose,
    TrajectoryReference,
    TrajectoryRequest,
    TrajectorySegment,
    load_library,
)
from simulation.scenarios import Scenario, solve_scenario, transform_polygon


@dataclass(frozen=True)
class TrajectorySample:
    time_s: float
    pose: np.ndarray
    grip: int
    move_index: int
    piece_id: int
    phase: str
    state: int
    linear_speed: float
    linear_acceleration: float
    yaw_speed: float
    yaw_acceleration: float
    low_height_risk: bool


@dataclass(frozen=True)
class MoveExecution:
    move_index: int
    piece_id: int
    move: DecisionMove
    request: TrajectoryRequest
    plan: TrajectoryPlan
    start_time_s: float
    approach_end_time_s: float
    hold_end_time_s: float
    transit_time_s: float
    end_time_s: float
    final_polygon: np.ndarray


@dataclass(frozen=True)
class SimulationResult:
    scenario: Scenario
    decision_plan: DecisionPlan
    samples: tuple[TrajectorySample, ...]
    moves: tuple[MoveExecution, ...]
    initial_polygons: dict[int, np.ndarray]
    final_polygons: dict[int, np.ndarray]


def pose_array(pose: TrajectoryPose) -> np.ndarray:
    return np.array(
        [pose.x_mm, pose.y_mm, pose.z_mm, pose.yaw_deg], dtype=float
    )


def _copy_pose(pose: TrajectoryPose) -> TrajectoryPose:
    return TrajectoryPose(pose.x_mm, pose.y_mm, pose.z_mm, pose.yaw_deg)


def _normalize_yaw(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def segment_kinematics(
    segment: TrajectorySegment, time_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate position, velocity, and acceleration from C coefficients."""

    duration = float(segment.duration_s)
    if duration <= 0.0:
        normalized_time = 1.0
    else:
        normalized_time = float(np.clip(time_s / duration, 0.0, 1.0))

    coefficients = np.array(
        [
            [segment.coefficient[axis][order] for order in range(6)]
            for axis in range(4)
        ],
        dtype=float,
    )
    position = np.zeros(4, dtype=float)
    for order in range(6):
        position += coefficients[:, order] * normalized_time**order

    if duration <= 0.0:
        velocity = np.zeros(4, dtype=float)
        acceleration = np.zeros(4, dtype=float)
    else:
        velocity = np.zeros(4, dtype=float)
        acceleration = np.zeros(4, dtype=float)
        for order in range(1, 6):
            velocity += (
                order
                * coefficients[:, order]
                * normalized_time ** (order - 1)
                / duration
            )
        for order in range(2, 6):
            acceleration += (
                order
                * (order - 1)
                * coefficients[:, order]
                * normalized_time ** (order - 2)
                / duration**2
            )

    position[3] = _normalize_yaw(position[3])
    return position, velocity, acceleration


def _sample_times(
    duration_s: float, sample_period_s: float, extras: tuple[float, ...] = ()
) -> np.ndarray:
    if duration_s <= 0.0:
        return np.array([0.0])
    count = max(1, int(np.ceil(duration_s / sample_period_s)))
    times = np.linspace(0.0, duration_s, count + 1)
    valid_extras = [value for value in extras if 0.0 <= value <= duration_s]
    if valid_extras:
        times = np.concatenate((times, np.asarray(valid_extras, dtype=float)))
    return np.unique(times)


def _evaluate_reference(
    plan: TrajectoryPlan, phase: int, time_s: float
) -> tuple[TrajectoryReference, int]:
    reference = TrajectoryReference()
    state = load_library().Trajectory_Evaluate(
        ctypes.byref(plan), phase, ctypes.c_float(time_s), ctypes.byref(reference)
    )
    return reference, state


def phase_boundary_times(phase_plan: TrajectoryPhasePlan) -> tuple[float, ...]:
    """Cumulative end time of every segment but the last."""

    boundaries: list[float] = []
    elapsed = 0.0
    for index in range(int(phase_plan.segment_count) - 1):
        elapsed += float(phase_plan.segments[index].duration_s)
        boundaries.append(elapsed)
    return tuple(boundaries)


def _phase_kinematics(
    plan: TrajectoryPlan, phase: int, time_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phase_plan = plan.approach if phase == TRAJECTORY_PHASE_APPROACH else plan.transfer
    segment_count = int(phase_plan.segment_count)
    if segment_count == 0:
        zero = np.zeros(4, dtype=float)
        return zero, zero.copy(), zero.copy()

    # Walk the chain the way the firmware does: the last segment absorbs any
    # overflow so a time past the end clamps to the final pose.
    remaining = time_s
    for index in range(segment_count):
        segment = phase_plan.segments[index]
        if remaining < float(segment.duration_s) or index + 1 == segment_count:
            return segment_kinematics(segment, remaining)
        remaining -= float(segment.duration_s)
    raise AssertionError("unreachable")


def _make_sample(
    *,
    global_time_s: float,
    local_time_s: float,
    plan: TrajectoryPlan,
    phase_enum: int,
    phase_name: str,
    move_index: int,
    piece_id: int,
    low_height_risk: bool,
) -> TrajectorySample:
    reference, state = _evaluate_reference(plan, phase_enum, local_time_s)
    _, velocity, acceleration = _phase_kinematics(plan, phase_enum, local_time_s)
    return TrajectorySample(
        time_s=global_time_s,
        pose=pose_array(reference.pose),
        grip=int(reference.grip),
        move_index=move_index,
        piece_id=piece_id,
        phase=phase_name,
        state=int(state),
        linear_speed=float(np.linalg.norm(velocity[:3])),
        linear_acceleration=float(np.linalg.norm(acceleration[:3])),
        yaw_speed=float(abs(velocity[3])),
        yaw_acceleration=float(abs(acceleration[3])),
        low_height_risk=low_height_risk,
    )


def _initial_polygons(scenario: Scenario) -> dict[int, np.ndarray]:
    polygons: dict[int, np.ndarray] = {}
    for index in range(scenario.frame.piece_count):
        piece = scenario.frame.pieces[index]
        polygons[int(piece.id)] = np.array(
            [
                [piece.vertices[vertex].x_mm, piece.vertices[vertex].y_mm]
                for vertex in range(piece.vertex_count)
            ],
            dtype=float,
        )
    return polygons


def run_simulation(
    scenario: Scenario,
    sample_period_s: float = 0.02,
    pick_hold_s: float = 0.6,
) -> SimulationResult:
    """Solve and sample every move in a built-in scenario."""

    if sample_period_s <= 0.0:
        raise ValueError("sample_period_s must be positive")
    if pick_hold_s < 0.0:
        raise ValueError("pick_hold_s must be non-negative")

    library = load_library()
    decision_plan = solve_scenario(scenario)
    initial_polygons = _initial_polygons(scenario)
    pieces_by_id = {
        int(scenario.frame.pieces[index].id): scenario.frame.pieces[index]
        for index in range(scenario.frame.piece_count)
    }
    samples: list[TrajectorySample] = []
    moves: list[MoveExecution] = []
    final_polygons: dict[int, np.ndarray] = {}
    current = _copy_pose(scenario.home)
    global_time = 0.0

    for move_index in range(decision_plan.move_count):
        move = decision_plan.moves[move_index]
        piece_id = int(move.piece_id)
        # Build the waypoint chains with the firmware itself, so the simulation
        # cannot drift from the paths the STM32 will actually follow.
        request = TrajectoryRequest()
        if not library.Decision_BuildTrajectoryRequest(
            ctypes.byref(move),
            ctypes.byref(current),
            ctypes.byref(scenario.limits),
            ctypes.byref(request),
        ):
            raise RuntimeError(
                f"Decision_BuildTrajectoryRequest failed for piece {piece_id}"
            )
        plan = TrajectoryPlan()
        result = library.Trajectory_Generate(ctypes.byref(request), ctypes.byref(plan))
        if result != TRAJECTORY_RESULT_OK:
            raise RuntimeError(
                f"Trajectory_Generate failed for piece {piece_id}: result={result}"
            )

        approach_duration = float(plan.approach.duration_s)
        # Flag a leg that travels any distance while staying near the board.
        cruise_z = max(0.25 * scenario.config.transit_z_mm, 1e-6)
        low_height_risk = False
        for index in range(int(request.approach.point_count) - 1):
            start = request.approach.points[index]
            end = request.approach.points[index + 1]
            travel = float(
                np.linalg.norm(pose_array(start)[:2] - pose_array(end)[:2])
            )
            if travel > 1.0 and max(start.z_mm, end.z_mm) <= cruise_z:
                low_height_risk = True
                break
        move_start = global_time
        approach_times = _sample_times(
            approach_duration, sample_period_s, phase_boundary_times(plan.approach)
        )
        for local_time in approach_times:
            samples.append(
                _make_sample(
                    global_time_s=move_start + float(local_time),
                    local_time_s=float(local_time),
                    plan=plan,
                    phase_enum=TRAJECTORY_PHASE_APPROACH,
                    phase_name="approach",
                    move_index=move_index,
                    piece_id=piece_id,
                    low_height_risk=low_height_risk,
                )
            )

        approach_end = move_start + approach_duration
        for hold_time in _sample_times(pick_hold_s, sample_period_s):
            samples.append(
                TrajectorySample(
                    time_s=approach_end + float(hold_time),
                    pose=pose_array(move.pick),
                    grip=1,
                    move_index=move_index,
                    piece_id=piece_id,
                    phase="hold",
                    state=TRAJECTORY_STATE_COMPLETE,
                    linear_speed=0.0,
                    linear_acceleration=0.0,
                    yaw_speed=0.0,
                    yaw_acceleration=0.0,
                    low_height_risk=False,
                )
            )

        hold_end = approach_end + pick_hold_s
        transfer_boundaries = phase_boundary_times(plan.transfer)
        first_transfer_duration = (
            transfer_boundaries[0] if transfer_boundaries else 0.0
        )
        transfer_duration = float(plan.transfer.duration_s)
        transfer_times = _sample_times(
            transfer_duration, sample_period_s, transfer_boundaries
        )
        for local_time in transfer_times:
            samples.append(
                _make_sample(
                    global_time_s=hold_end + float(local_time),
                    local_time_s=float(local_time),
                    plan=plan,
                    phase_enum=TRAJECTORY_PHASE_TRANSFER,
                    phase_name="transfer",
                    move_index=move_index,
                    piece_id=piece_id,
                    low_height_risk=False,
                )
            )

        move_end = hold_end + transfer_duration
        piece = pieces_by_id[piece_id]
        final_polygon = transform_polygon(
            piece.vertices[: piece.vertex_count], move.pick, move.place
        )
        final_polygons[piece_id] = final_polygon
        moves.append(
            MoveExecution(
                move_index=move_index,
                piece_id=piece_id,
                move=move,
                request=request,
                plan=plan,
                start_time_s=move_start,
                approach_end_time_s=approach_end,
                hold_end_time_s=hold_end,
                transit_time_s=hold_end + first_transfer_duration,
                end_time_s=move_end,
                final_polygon=final_polygon,
            )
        )
        current = _copy_pose(move.place)
        global_time = move_end

    return SimulationResult(
        scenario=scenario,
        decision_plan=decision_plan,
        samples=tuple(samples),
        moves=tuple(moves),
        initial_polygons=initial_polygons,
        final_polygons=final_polygons,
    )

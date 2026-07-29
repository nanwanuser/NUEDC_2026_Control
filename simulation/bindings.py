"""ctypes declarations for the firmware decision and trajectory APIs."""

from __future__ import annotations

import ctypes

from simulation.build_native import build_library


DECISION_MAX_PIECES = 4
DECISION_MAX_VERTICES = 5
TRAJECTORY_AXIS_COUNT = 4
TRAJECTORY_COEFFICIENT_COUNT = 6
TRAJECTORY_MAX_WAYPOINTS = 6
TRAJECTORY_MAX_SEGMENTS = TRAJECTORY_MAX_WAYPOINTS - 1

DECISION_MODE_FIXED_ID = 0
DECISION_MODE_GENERAL = 1
DECISION_RESULT_OK = 0

TRAJECTORY_PHASE_APPROACH = 0
TRAJECTORY_PHASE_TRANSFER = 1
TRAJECTORY_RESULT_OK = 0
TRAJECTORY_STATE_RUNNING = 0
TRAJECTORY_STATE_COMPLETE = 1
TRAJECTORY_STATE_INVALID_ARGUMENT = 2
TRAJECTORY_STATE_INVALID_PHASE = 3


class TrajectoryPose(ctypes.Structure):
    _fields_ = [
        ("x_mm", ctypes.c_float),
        ("y_mm", ctypes.c_float),
        ("z_mm", ctypes.c_float),
        ("yaw_deg", ctypes.c_float),
    ]


class TrajectoryLimits(ctypes.Structure):
    _fields_ = [
        ("max_linear_velocity_mm_s", ctypes.c_float),
        ("max_linear_acceleration_mm_s2", ctypes.c_float),
        ("max_yaw_velocity_deg_s", ctypes.c_float),
        ("max_yaw_acceleration_deg_s2", ctypes.c_float),
    ]


class TrajectoryPath(ctypes.Structure):
    """Ordered waypoints of one phase; the tool stops only at both ends."""

    _fields_ = [
        ("point_count", ctypes.c_uint8),
        ("points", TrajectoryPose * TRAJECTORY_MAX_WAYPOINTS),
    ]


class TrajectoryRequest(ctypes.Structure):
    _fields_ = [
        ("approach", TrajectoryPath),
        ("transfer", TrajectoryPath),
        ("limits", TrajectoryLimits),
    ]


class TrajectoryReference(ctypes.Structure):
    _fields_ = [("pose", TrajectoryPose), ("grip", ctypes.c_uint8)]


CoefficientRow = ctypes.c_float * TRAJECTORY_COEFFICIENT_COUNT


class TrajectorySegment(ctypes.Structure):
    _fields_ = [
        ("coefficient", CoefficientRow * TRAJECTORY_AXIS_COUNT),
        ("duration_s", ctypes.c_float),
    ]


class TrajectoryPhasePlan(ctypes.Structure):
    _fields_ = [
        ("segment_count", ctypes.c_uint8),
        ("segments", TrajectorySegment * TRAJECTORY_MAX_SEGMENTS),
        ("duration_s", ctypes.c_float),
    ]


class TrajectoryPlan(ctypes.Structure):
    _fields_ = [
        ("approach", TrajectoryPhasePlan),
        ("transfer", TrajectoryPhasePlan),
    ]


class DecisionPoint(ctypes.Structure):
    _fields_ = [("x_mm", ctypes.c_float), ("y_mm", ctypes.c_float)]


class DecisionPiece(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint8),
        ("vertex_count", ctypes.c_uint8),
        ("center", DecisionPoint),
        ("vertices", DecisionPoint * DECISION_MAX_VERTICES),
    ]


class DecisionVisionFrame(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint32),
        ("piece_count", ctypes.c_uint8),
        ("pieces", DecisionPiece * DECISION_MAX_PIECES),
    ]


class DecisionFixedPiece(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint8),
        ("vertex_count", ctypes.c_uint8),
        ("target_vertices", DecisionPoint * DECISION_MAX_VERTICES),
    ]


class DecisionFixedLayout(ctypes.Structure):
    _fields_ = [
        ("piece_count", ctypes.c_uint8),
        ("pieces", DecisionFixedPiece * DECISION_MAX_PIECES),
    ]


class DecisionConfig(ctypes.Structure):
    _fields_ = [
        ("target_center", DecisionPoint),
        ("pick_z_mm", ctypes.c_float),
        ("transit_z_mm", ctypes.c_float),
        ("place_z_mm", ctypes.c_float),
        ("edge_length_tolerance_mm", ctypes.c_float),
        ("boundary_tolerance_mm", ctypes.c_float),
        ("max_fill_error_ratio", ctypes.c_float),
        ("min_short_side_mm", ctypes.c_float),
        ("max_short_side_mm", ctypes.c_float),
        ("min_long_side_mm", ctypes.c_float),
        ("max_long_side_mm", ctypes.c_float),
        ("max_search_nodes", ctypes.c_uint32),
    ]


class DecisionMove(ctypes.Structure):
    """Both lift poses sit at transit_z_mm, directly above pick and place."""

    _fields_ = [
        ("piece_id", ctypes.c_uint8),
        ("pick", TrajectoryPose),
        ("pick_above", TrajectoryPose),
        ("place_above", TrajectoryPose),
        ("place", TrajectoryPose),
    ]


class DecisionPlan(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint32),
        ("move_count", ctypes.c_uint8),
        ("moves", DecisionMove * DECISION_MAX_PIECES),
    ]


_library: ctypes.CDLL | None = None


def _configure_library(library: ctypes.CDLL) -> None:
    library.Decision_GetDefaultConfig.argtypes = [ctypes.POINTER(DecisionConfig)]
    library.Decision_GetDefaultConfig.restype = None
    library.Decision_Solve.argtypes = [
        ctypes.c_int,
        ctypes.POINTER(DecisionVisionFrame),
        ctypes.POINTER(DecisionFixedLayout),
        ctypes.POINTER(DecisionConfig),
        ctypes.POINTER(DecisionPlan),
    ]
    library.Decision_Solve.restype = ctypes.c_int
    library.Decision_BuildTrajectoryRequest.argtypes = [
        ctypes.POINTER(DecisionMove),
        ctypes.POINTER(TrajectoryPose),
        ctypes.POINTER(TrajectoryLimits),
        ctypes.POINTER(TrajectoryRequest),
    ]
    library.Decision_BuildTrajectoryRequest.restype = ctypes.c_uint8
    library.Trajectory_PathReset.argtypes = [ctypes.POINTER(TrajectoryPath)]
    library.Trajectory_PathReset.restype = None
    library.Trajectory_PathAppend.argtypes = [
        ctypes.POINTER(TrajectoryPath),
        ctypes.POINTER(TrajectoryPose),
    ]
    library.Trajectory_PathAppend.restype = ctypes.c_uint8
    library.Trajectory_Generate.argtypes = [
        ctypes.POINTER(TrajectoryRequest),
        ctypes.POINTER(TrajectoryPlan),
    ]
    library.Trajectory_Generate.restype = ctypes.c_int
    library.Trajectory_Evaluate.argtypes = [
        ctypes.POINTER(TrajectoryPlan),
        ctypes.c_int,
        ctypes.c_float,
        ctypes.POINTER(TrajectoryReference),
    ]
    library.Trajectory_Evaluate.restype = ctypes.c_int
    library.Trajectory_GetDuration.argtypes = [
        ctypes.POINTER(TrajectoryPlan),
        ctypes.c_int,
    ]
    library.Trajectory_GetDuration.restype = ctypes.c_float


def load_library(force_rebuild: bool = False) -> ctypes.CDLL:
    """Build, load, and configure the native algorithm library."""

    global _library
    if _library is not None:
        return _library

    path = build_library(force=force_rebuild)
    _library = ctypes.CDLL(str(path))
    _configure_library(_library)
    return _library

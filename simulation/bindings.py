"""ctypes declarations for the firmware decision and trajectory APIs."""

from __future__ import annotations

import ctypes

from simulation.build_native import build_library


DECISION_MAX_PIECES = 4
DECISION_MAX_VERTICES = 5
DECISION_CARD_MAX_EDGE_EVENTS_PER_PIECE = 24
DECISION_CARD_MAX_PRIMITIVES_PER_PIECE = 16
TRAJECTORY_AXIS_COUNT = 4
TRAJECTORY_COEFFICIENT_COUNT = 6
TRAJECTORY_MAX_WAYPOINTS = 6
TRAJECTORY_MAX_SEGMENTS = TRAJECTORY_MAX_WAYPOINTS - 1

DECISION_RESULT_OK = 0
DECISION_RESULT_INVALID_FRAME = 2

DECISION_CARD_COLOR_RED = 1
DECISION_CARD_COLOR_BLACK = 2
DECISION_CARD_PRIMITIVE_UNKNOWN = 0
DECISION_CARD_PRIMITIVE_DOT = 1
DECISION_CARD_PRIMITIVE_LINE = 2
DECISION_CARD_PRIMITIVE_GLYPH = 3

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


class DecisionCardEdgeEvent(ctypes.Structure):
    _fields_ = [
        ("edge_index", ctypes.c_uint8),
        ("position_q8", ctypes.c_uint8),
        ("color", ctypes.c_uint8),
        ("tangent_deg", ctypes.c_int8),
        ("width_q4_mm", ctypes.c_uint8),
        ("confidence", ctypes.c_uint8),
    ]


class DecisionCardPrimitive(ctypes.Structure):
    _fields_ = [
        ("center", DecisionPoint),
        ("area_mm2", ctypes.c_float),
        ("color", ctypes.c_uint8),
        ("kind", ctypes.c_uint8),
        ("angle_deg", ctypes.c_int8),
        ("confidence", ctypes.c_uint8),
    ]


class DecisionCardPieceFeatures(ctypes.Structure):
    _fields_ = [
        ("piece_id", ctypes.c_uint8),
        ("edge_event_count", ctypes.c_uint8),
        (
            "edge_events",
            DecisionCardEdgeEvent * DECISION_CARD_MAX_EDGE_EVENTS_PER_PIECE,
        ),
        ("primitive_count", ctypes.c_uint8),
        (
            "primitives",
            DecisionCardPrimitive * DECISION_CARD_MAX_PRIMITIVES_PER_PIECE,
        ),
    ]


class DecisionCardFrame(ctypes.Structure):
    _fields_ = [
        ("layout_id", ctypes.c_uint32),
        ("vision", DecisionVisionFrame),
        ("piece_count", ctypes.c_uint8),
        ("pieces", DecisionCardPieceFeatures * DECISION_MAX_PIECES),
    ]


class DecisionConfig(ctypes.Structure):
    _fields_ = [
        ("target_center", DecisionPoint),
        # Must match DecisionConfig in decision.h field for field: ctypes maps
        # these by offset, so a missing member does not fail loudly, it silently
        # shifts every field after it onto the wrong bytes.
        ("paper_divider_x_mm", ctypes.c_float),
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
        ("search_nodes", ctypes.c_uint32),
        ("move_count", ctypes.c_uint8),
        ("moves", DecisionMove * DECISION_MAX_PIECES),
    ]


_library: ctypes.CDLL | None = None


def _configure_library(library: ctypes.CDLL) -> None:
    library.Decision_GetDefaultConfig.argtypes = [ctypes.POINTER(DecisionConfig)]
    library.Decision_GetDefaultConfig.restype = None
    library.Decision_Solve.argtypes = [
        ctypes.POINTER(DecisionVisionFrame),
        ctypes.POINTER(DecisionConfig),
        ctypes.POINTER(DecisionPlan),
    ]
    library.Decision_Solve.restype = ctypes.c_int
    library.Decision_SolveCard.argtypes = [
        ctypes.POINTER(DecisionCardFrame),
        ctypes.POINTER(DecisionConfig),
        ctypes.POINTER(DecisionPlan),
    ]
    library.Decision_SolveCard.restype = ctypes.c_int
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

    for export_name in (
        "Simulation_SizeOfDecisionPoint",
        "Simulation_SizeOfDecisionMove",
        "Simulation_SizeOfDecisionPlan",
        "Simulation_SizeOfDecisionCardEdgeEvent",
        "Simulation_SizeOfDecisionCardPrimitive",
        "Simulation_SizeOfDecisionCardPieceFeatures",
        "Simulation_SizeOfDecisionCardFrame",
        "Simulation_SizeOfTrajectoryPose",
        "Simulation_SizeOfTrajectoryPlan",
        "Simulation_OffsetOfDecisionPlanMoves",
        "Simulation_OffsetOfDecisionPlanSearchNodes",
    ):
        getattr(library, export_name).restype = ctypes.c_uint32
    library.Simulation_DecisionAssemblyClearanceMm.restype = ctypes.c_float


def assert_native_layout(library: ctypes.CDLL) -> None:
    """Fail before simulation if ctypes no longer matches the firmware ABI."""

    sizes = (
        (DecisionPoint, library.Simulation_SizeOfDecisionPoint()),
        (DecisionMove, library.Simulation_SizeOfDecisionMove()),
        (DecisionPlan, library.Simulation_SizeOfDecisionPlan()),
        (
            DecisionCardEdgeEvent,
            library.Simulation_SizeOfDecisionCardEdgeEvent(),
        ),
        (
            DecisionCardPrimitive,
            library.Simulation_SizeOfDecisionCardPrimitive(),
        ),
        (
            DecisionCardPieceFeatures,
            library.Simulation_SizeOfDecisionCardPieceFeatures(),
        ),
        (DecisionCardFrame, library.Simulation_SizeOfDecisionCardFrame()),
        (TrajectoryPose, library.Simulation_SizeOfTrajectoryPose()),
        (TrajectoryPlan, library.Simulation_SizeOfTrajectoryPlan()),
    )
    for structure, native_size in sizes:
        python_size = ctypes.sizeof(structure)
        if python_size != native_size:
            raise RuntimeError(
                f"ABI mismatch for {structure.__name__}: "
                f"ctypes={python_size}, native={native_size}"
            )

    offsets = (
        (
            "DecisionPlan.search_nodes",
            DecisionPlan.search_nodes.offset,
            library.Simulation_OffsetOfDecisionPlanSearchNodes(),
        ),
        (
            "DecisionPlan.moves",
            DecisionPlan.moves.offset,
            library.Simulation_OffsetOfDecisionPlanMoves(),
        ),
    )
    for name, python_offset, native_offset in offsets:
        if python_offset != native_offset:
            raise RuntimeError(
                f"ABI offset mismatch for {name}: "
                f"ctypes={python_offset}, native={native_offset}"
            )


def load_library(force_rebuild: bool = False) -> ctypes.CDLL:
    """Build, load, and configure the native algorithm library."""

    global _library
    if _library is not None:
        return _library

    path = build_library(force=force_rebuild)
    _library = ctypes.CDLL(str(path))
    _configure_library(_library)
    assert_native_layout(_library)
    return _library

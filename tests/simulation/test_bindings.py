import ctypes

from simulation.bindings import (
    DECISION_RESULT_INVALID_FRAME,
    DecisionCardFrame,
    DecisionConfig,
    DecisionPlan,
    assert_native_layout,
    load_library,
)


def test_card_ctypes_layout_matches_native_firmware():
    library = load_library(force_rebuild=True)

    assert_native_layout(library)
    assert DecisionPlan.search_nodes.offset == 4
    assert DecisionPlan.moves.offset > DecisionPlan.move_count.offset


def test_zeroed_card_frame_is_rejected_without_abi_corruption():
    library = load_library()
    card = DecisionCardFrame()
    config = DecisionConfig()
    plan = DecisionPlan()
    library.Decision_GetDefaultConfig(ctypes.byref(config))

    result = library.Decision_SolveCard(
        ctypes.byref(card), ctypes.byref(config), ctypes.byref(plan)
    )

    assert result == DECISION_RESULT_INVALID_FRAME

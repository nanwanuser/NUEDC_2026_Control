#ifndef DECISION_TEMPLATE_H
#define DECISION_TEMPLATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "decision.h"

#define DECISION_FIGURE2_WIDTH_MM  100.0f
#define DECISION_FIGURE2_HEIGHT_MM  60.0f

typedef enum {
    DECISION_FIGURE2_PIECE_LARGE_TRIANGLE = 0,
    DECISION_FIGURE2_PIECE_TOP_LEFT,
    DECISION_FIGURE2_PIECE_MIDDLE,
    DECISION_FIGURE2_PIECE_BOTTOM
} DecisionFigure2Piece;

/* Builds the four-piece template from Figure 2, centered in A4 millimetres. */
void DecisionTemplate_GetFigure2Layout(DecisionPoint target_center,
                                       DecisionFixedLayout *layout);

#ifdef __cplusplus
}
#endif

#endif

#ifndef DECISION_H
#define DECISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "trajectory.h"

/* Both from the task: at most four pieces, at most five edges each. */
#define DECISION_MAX_PIECES       4U
#define DECISION_MAX_VERTICES     5U
#define DECISION_DEFAULT_MAX_NODES 50000U

/* The task scores an assembly on one geometric criterion: adjacent pieces'
   corresponding vertices within 2 cm. Every threshold below is therefore a
   search aid, not a correctness requirement, and each one only has to be tight
   enough to keep the search from wandering. Sized against hand-cut pieces,
   whose edges routinely disagree by several millimetres, because a threshold
   that rejects the true layout turns into NO_SOLUTION rather than a lower
   score. */

/* Length disagreement allowed between two edges being joined. Hand-cut mating
   edges differ by millimetres, and the resulting vertex offset stays far inside
   the 2 cm the task allows, so this is deliberately far looser than the cut
   accuracy it tolerates. */
#define DECISION_EDGE_TOLERANCE_MM      8.0f
/* How far a piece's outermost edge may sit from the bounding rectangle and
   still count as lying on it. One degree of cutting error across a 100 mm edge
   is already ~1.7 mm, and the edge itself may be short of the corner. */
#define DECISION_BOUNDARY_TOLERANCE_MM  8.0f
/* Slack between the pieces' total area and their bounding rectangle. Gaps left
   by imperfect cuts count against this, so it has to absorb the sum of all
   inter-piece seams rather than any single one. */
#define DECISION_MAX_FILL_ERROR_RATIO   0.20f
/* How deep one piece may reach into another before the pair counts as
   overlapping. Joining two edges of unequal length aligns their midpoints, so
   the longer piece necessarily pokes into its neighbour by up to half the
   difference: with DECISION_EDGE_TOLERANCE_MM at 8 mm that is 4 mm of
   unavoidable nibble. Judging overlap by depth rather than by any incursion at
   all is what lets the loose edge tolerance actually be used; a piece genuinely
   lying on top of another still reaches far deeper than this. */
#define DECISION_OVERLAP_TOLERANCE_MM   \
    (0.5f * DECISION_EDGE_TOLERANCE_MM + 1.0f)

/* The sheet's midline along its long edge. The task starts the pieces in one half
   and scores the assembly in the other, but which half is which depends on the
   corner the camera calibration calls the origin, so the solver reads it off the
   pieces it was given rather than assuming. */
#define DECISION_PAPER_DIVIDER_X_MM     148.5f

/* Target rectangle envelope, 9x5 cm to 12x9 cm in the task, widened by the
   cutting error that accumulates along each side. The task guarantees the true
   rectangle is inside the stated range; what is measured is not, so clamping to
   the exact range would reject a valid assembly whose edges came out a few
   millimetres long. */
#define DECISION_SIDE_MARGIN_MM         12.0f
#define DECISION_MIN_SHORT_SIDE_MM      (50.0f - DECISION_SIDE_MARGIN_MM)
#define DECISION_MAX_SHORT_SIDE_MM      (90.0f + DECISION_SIDE_MARGIN_MM)
#define DECISION_MIN_LONG_SIDE_MM       (90.0f - DECISION_SIDE_MARGIN_MM)
#define DECISION_MAX_LONG_SIDE_MM       (120.0f + DECISION_SIDE_MARGIN_MM)

typedef struct {
    float x_mm;
    float y_mm;
} DecisionPoint;

typedef struct {
    uint8_t id;
    uint8_t vertex_count;
    DecisionPoint center;
    DecisionPoint vertices[DECISION_MAX_VERTICES];
} DecisionPiece;

/* The vision protocol is decoded into this structure before DecisionTask. */
typedef struct {
    uint32_t seq;
    uint8_t piece_count;
    DecisionPiece pieces[DECISION_MAX_PIECES];
} DecisionVisionFrame;

typedef struct {
    DecisionPoint target_center;
    /* Long-axis coordinate of the line splitting the sheet into the half the
       pieces start in and the half they are assembled in. Set to zero to place
       the rectangle at target_center unconditionally; see
       DECISION_PAPER_DIVIDER_X_MM. */
    float paper_divider_x_mm;
    float pick_z_mm;
    /* Cruise height used above the pick and the place location. */
    float transit_z_mm;
    float place_z_mm;
    float edge_length_tolerance_mm;
    float boundary_tolerance_mm;
    float max_fill_error_ratio;
    float min_short_side_mm;
    float max_short_side_mm;
    float min_long_side_mm;
    float max_long_side_mm;
    uint32_t max_search_nodes;
} DecisionConfig;

/* Both lift poses sit at transit_z_mm, directly above pick and place. */
typedef struct {
    uint8_t piece_id;
    TrajectoryPose pick;
    TrajectoryPose pick_above;
    TrajectoryPose place_above;
    TrajectoryPose place;
} DecisionMove;

typedef struct {
    uint32_t seq;
    uint8_t move_count;
    DecisionMove moves[DECISION_MAX_PIECES];
} DecisionPlan;

typedef enum {
    DECISION_RESULT_OK = 0,
    DECISION_RESULT_INVALID_ARGUMENT,
    DECISION_RESULT_INVALID_FRAME,
    DECISION_RESULT_NO_SOLUTION,
    DECISION_RESULT_SEARCH_LIMIT,
    DECISION_RESULT_NUMERIC_ERROR
} DecisionResult;

void Decision_GetDefaultConfig(DecisionConfig *config);

/* Every scored task assembles pieces the device has not seen before, so the
   solve is always the edge-matching search. */
DecisionResult Decision_Solve(const DecisionVisionFrame *frame,
                              const DecisionConfig *config,
                              DecisionPlan *plan);

DecisionResult Decision_SolveGeneral(const DecisionVisionFrame *frame,
                                     const DecisionConfig *config,
                                     DecisionPlan *plan);

/* Builds current -> lift -> above pick -> pick and pick -> above pick ->
   above place -> place, so no leg of the motion travels near the board. */
uint8_t Decision_BuildTrajectoryRequest(const DecisionMove *move,
                                        const TrajectoryPose *current,
                                        const TrajectoryLimits *limits,
                                        TrajectoryRequest *request);

#ifdef __cplusplus
}
#endif

#endif

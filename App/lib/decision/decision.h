#ifndef DECISION_H
#define DECISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "trajectory.h"

#define DECISION_MAX_PIECES       4U
#define DECISION_MAX_VERTICES     5U
#define DECISION_DEFAULT_MAX_NODES 50000U

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

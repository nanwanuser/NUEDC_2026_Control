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

/* JSON is parsed into this fixed-size structure before entering DecisionTask. */
typedef struct {
    uint32_t seq;
    uint8_t piece_count;
    DecisionPiece pieces[DECISION_MAX_PIECES];
} DecisionVisionFrame;

typedef enum {
    DECISION_MODE_FIXED_ID = 0,
    DECISION_MODE_GENERAL = 1
} DecisionMode;

typedef struct {
    uint8_t id;
    uint8_t vertex_count;
    DecisionPoint target_vertices[DECISION_MAX_VERTICES];
} DecisionFixedPiece;

typedef struct {
    uint8_t piece_count;
    DecisionFixedPiece pieces[DECISION_MAX_PIECES];
} DecisionFixedLayout;

typedef struct {
    DecisionPoint target_center;
    float pick_z_mm;
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

typedef struct {
    uint8_t piece_id;
    TrajectoryPose pick;
    TrajectoryPose transit;
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
    DECISION_RESULT_TEMPLATE_NOT_FOUND,
    DECISION_RESULT_TEMPLATE_MISMATCH,
    DECISION_RESULT_NO_SOLUTION,
    DECISION_RESULT_SEARCH_LIMIT,
    DECISION_RESULT_NUMERIC_ERROR
} DecisionResult;

void Decision_GetDefaultConfig(DecisionConfig *config);

DecisionResult Decision_Solve(DecisionMode mode,
                              const DecisionVisionFrame *frame,
                              const DecisionFixedLayout *fixed_layout,
                              const DecisionConfig *config,
                              DecisionPlan *plan);

DecisionResult Decision_SolveFixed(const DecisionVisionFrame *frame,
                                   const DecisionFixedLayout *layout,
                                   const DecisionConfig *config,
                                   DecisionPlan *plan);

DecisionResult Decision_SolveGeneral(const DecisionVisionFrame *frame,
                                     const DecisionConfig *config,
                                     DecisionPlan *plan);

#ifdef __cplusplus
}
#endif

#endif

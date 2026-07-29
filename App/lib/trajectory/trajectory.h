#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRAJECTORY_AXIS_COUNT              4U
#define TRAJECTORY_COEFFICIENT_COUNT       6U
#define TRAJECTORY_TRANSFER_SEGMENT_COUNT  2U

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float yaw_deg;
} TrajectoryPose;

typedef struct {
    float max_linear_velocity_mm_s;
    float max_linear_acceleration_mm_s2;
    float max_yaw_velocity_deg_s;
    float max_yaw_acceleration_deg_s2;
} TrajectoryLimits;

typedef struct {
    TrajectoryPose current;
    TrajectoryPose pick;
    TrajectoryPose transit;
    TrajectoryPose place;
    TrajectoryLimits limits;
} TrajectoryRequest;

typedef struct {
    TrajectoryPose pose;
    uint8_t grip;
} TrajectoryReference;

typedef enum {
    TRAJECTORY_PHASE_APPROACH = 0,
    TRAJECTORY_PHASE_TRANSFER = 1
} TrajectoryPhase;

typedef enum {
    TRAJECTORY_RESULT_OK = 0,
    TRAJECTORY_RESULT_INVALID_ARGUMENT,
    TRAJECTORY_RESULT_INVALID_LIMIT,
    TRAJECTORY_RESULT_NUMERIC_ERROR
} TrajectoryResult;

typedef enum {
    TRAJECTORY_STATE_RUNNING = 0,
    TRAJECTORY_STATE_COMPLETE,
    TRAJECTORY_STATE_INVALID_ARGUMENT,
    TRAJECTORY_STATE_INVALID_PHASE
} TrajectoryState;

typedef struct {
    float coefficient[TRAJECTORY_AXIS_COUNT][TRAJECTORY_COEFFICIENT_COUNT];
    float duration_s;
} TrajectorySegment;

typedef struct {
    TrajectorySegment approach;
    TrajectorySegment transfer[TRAJECTORY_TRANSFER_SEGMENT_COUNT];
    float transfer_duration_s;
} TrajectoryPlan;

TrajectoryResult Trajectory_Generate(const TrajectoryRequest *request,
                                     TrajectoryPlan *plan);
TrajectoryState Trajectory_Evaluate(const TrajectoryPlan *plan,
                                    TrajectoryPhase phase,
                                    float time_s,
                                    TrajectoryReference *reference);
float Trajectory_GetDuration(const TrajectoryPlan *plan,
                             TrajectoryPhase phase);

#ifdef __cplusplus
}
#endif

#endif

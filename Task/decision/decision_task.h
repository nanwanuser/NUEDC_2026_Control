#ifndef DECISION_TASK_H
#define DECISION_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "decision.h"

typedef struct {
    TrajectoryPose current_pose;
    TrajectoryLimits limits;
    /* Open-loop travel time allowed for the full-speed 90<->180 degree lift
       servo move. The servo has no position feedback. */
    uint32_t lift_travel_ms;
    uint32_t grip_dwell_ms;
    uint32_t release_dwell_ms;
} DecisionExecutionConfig;

typedef enum {
    DECISION_EXECUTION_IDLE = 0,
    DECISION_EXECUTION_WAITING_ROUTE,
    DECISION_EXECUTION_APPROACH,
    DECISION_EXECUTION_LOWER_PICK,
    DECISION_EXECUTION_GRIP_DWELL,
    DECISION_EXECUTION_RAISE_PICK,
    DECISION_EXECUTION_TRANSFER,
    DECISION_EXECUTION_LOWER_PLACE,
    DECISION_EXECUTION_RELEASE_DWELL,
    DECISION_EXECUTION_RAISE_PLACE,
    DECISION_EXECUTION_COMPLETE,
    DECISION_EXECUTION_ERROR
} DecisionExecutionState;

typedef struct {
    DecisionStrategy strategy;
    DecisionVisionFrame vision;
    DecisionCardFrame card;
    DecisionConfig config;
    DecisionExecutionConfig execution;
} DecisionTaskRequest;

typedef struct {
    DecisionResult result;
    DecisionPlan plan;
    DecisionExecutionState execution_state;
    TrajectoryResult trajectory_result;
    uint8_t active_move_index;
    uint32_t active_route_plan_id;
} DecisionTaskOutput;

extern volatile DecisionTaskRequest DecisionTask_Input;
extern volatile uint8_t DecisionTask_RequestPending;
extern volatile DecisionTaskOutput DecisionTask_Output;

void DecisionTask_Init(void);
void DecisionTask_GetDefaultRequest(DecisionTaskRequest *request);
uint8_t DecisionTask_Submit(const DecisionTaskRequest *request);
void Decision_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

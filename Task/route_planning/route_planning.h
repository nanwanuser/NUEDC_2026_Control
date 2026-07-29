#ifndef ROUTE_PLANNING_H
#define ROUTE_PLANNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "trajectory.h"

typedef struct {
    uint32_t plan_id;
    TrajectoryRequest trajectory;
} RoutePlanningRequest;

typedef struct {
    uint32_t plan_id;
    TrajectoryPhase phase;
    TrajectoryState state;
    TrajectoryResult result;
    float elapsed_s;
    TrajectoryReference reference;
} RoutePlanningOutput;

/* Shared variables are the task-to-task interface. */
extern volatile RoutePlanningRequest RoutePlanning_Input;
extern volatile uint8_t RoutePlanning_RequestPending;
extern volatile uint8_t RoutePlanning_ResumeTransferRequested;
extern volatile RoutePlanningOutput RoutePlanning_Output;

void RoutePlanning_Init(void);
uint8_t RoutePlanning_Submit(const RoutePlanningRequest *request);
void RoutePlanning_ResumeTransfer(void);
void Route_planning_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

#ifndef DECISION_TASK_H
#define DECISION_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "decision.h"

typedef struct {
    DecisionMode mode;
    DecisionVisionFrame vision;
    DecisionFixedLayout fixed_layout;
    DecisionConfig config;
} DecisionTaskRequest;

typedef struct {
    DecisionResult result;
    DecisionPlan plan;
} DecisionTaskOutput;

extern volatile DecisionTaskRequest DecisionTask_Input;
extern volatile uint8_t DecisionTask_RequestPending;
extern volatile DecisionTaskOutput DecisionTask_Output;

void DecisionTask_Init(void);
uint8_t DecisionTask_Submit(const DecisionTaskRequest *request);
void Decision_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

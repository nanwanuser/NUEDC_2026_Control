#include "route_planning.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#define ROUTE_PLANNING_TASK_PERIOD_MS 1U

volatile RoutePlanningRequest RoutePlanning_Input;
volatile uint8_t RoutePlanning_RequestPending;
volatile uint8_t RoutePlanning_ResumeTransferRequested;
volatile RoutePlanningOutput RoutePlanning_Output;

static void copy_input(RoutePlanningRequest *request)
{
    taskENTER_CRITICAL();
    *request = RoutePlanning_Input;
    RoutePlanning_RequestPending = 0U;
    taskEXIT_CRITICAL();
}

static uint8_t take_resume_request(void)
{
    uint8_t requested;

    taskENTER_CRITICAL();
    requested = RoutePlanning_ResumeTransferRequested;
    RoutePlanning_ResumeTransferRequested = 0U;
    taskEXIT_CRITICAL();

    return requested;
}

static void publish_output(const RoutePlanningOutput *output)
{
    taskENTER_CRITICAL();
    RoutePlanning_Output = *output;
    taskEXIT_CRITICAL();
}

static float ticks_to_seconds(uint32_t ticks)
{
    const uint32_t tick_frequency = osKernelGetTickFreq();

    if (tick_frequency == 0U) {
        return 0.0f;
    }
    return (float)ticks / (float)tick_frequency;
}

void RoutePlanning_Init(void)
{
    (void)memset((void *)&RoutePlanning_Input, 0, sizeof(RoutePlanning_Input));
    (void)memset((void *)&RoutePlanning_Output, 0, sizeof(RoutePlanning_Output));
    RoutePlanning_Output.state = TRAJECTORY_STATE_INVALID_ARGUMENT;
    RoutePlanning_Output.result = TRAJECTORY_RESULT_INVALID_ARGUMENT;
    RoutePlanning_RequestPending = 0U;
    RoutePlanning_ResumeTransferRequested = 0U;
}

uint8_t RoutePlanning_Submit(const RoutePlanningRequest *request)
{
    if (request == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    RoutePlanning_Input = *request;
    RoutePlanning_RequestPending = 1U;
    RoutePlanning_ResumeTransferRequested = 0U;
    taskEXIT_CRITICAL();

    return 1U;
}

void RoutePlanning_ResumeTransfer(void)
{
    taskENTER_CRITICAL();
    RoutePlanning_ResumeTransferRequested = 1U;
    taskEXIT_CRITICAL();
}

void Route_planning_App(void *argument)
{
    RoutePlanningRequest request;
    RoutePlanningOutput output;
    TrajectoryPlan plan;
    TrajectoryPhase phase = TRAJECTORY_PHASE_APPROACH;
    TrajectoryState state;
    TrajectoryResult result;
    uint32_t active_plan_id = 0U;
    uint32_t phase_start_tick = 0U;
    uint8_t plan_active = 0U;

    (void)argument;

    for (;;) {
        if (RoutePlanning_RequestPending != 0U) {
            copy_input(&request);
            result = Trajectory_Generate(&request.trajectory, &plan);
            active_plan_id = request.plan_id;
            phase = TRAJECTORY_PHASE_APPROACH;
            phase_start_tick = osKernelGetTickCount();
            (void)take_resume_request();

            if (result == TRAJECTORY_RESULT_OK) {
                plan_active = 1U;
            } else {
                plan_active = 0U;
                output.plan_id = active_plan_id;
                output.phase = phase;
                output.state = TRAJECTORY_STATE_INVALID_ARGUMENT;
                output.result = result;
                output.elapsed_s = 0.0f;
                (void)memset(&output.reference, 0, sizeof(output.reference));
                publish_output(&output);
            }
        }

        if (plan_active != 0U) {
            const uint32_t now = osKernelGetTickCount();
            const float elapsed_s = ticks_to_seconds(now - phase_start_tick);

            state = Trajectory_Evaluate(&plan, phase, elapsed_s, &output.reference);
            output.plan_id = active_plan_id;
            output.phase = phase;
            output.state = state;
            output.result = TRAJECTORY_RESULT_OK;
            output.elapsed_s = elapsed_s;
            publish_output(&output);

            if ((phase == TRAJECTORY_PHASE_APPROACH) &&
                (state == TRAJECTORY_STATE_COMPLETE) &&
                (RoutePlanning_ResumeTransferRequested != 0U)) {
                (void)take_resume_request();
                phase = TRAJECTORY_PHASE_TRANSFER;
                phase_start_tick = now;
            }
        }

        osDelay(ROUTE_PLANNING_TASK_PERIOD_MS);
    }
}

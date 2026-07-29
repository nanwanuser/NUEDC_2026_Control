#include "crane_control.h"

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

#define CRANE_CONTROL_TASK_PERIOD_MS 20U

static void snapshot_planner_output(RoutePlanningOutput *output)
{
    taskENTER_CRITICAL();
    *output = RoutePlanning_Output;
    taskEXIT_CRITICAL();
}

/**
 * @brief Strong implementation of the CubeMX Robot_arm_ctrl FreeRTOS task.
 * @param argument Unused task argument.
 */
void Robot_arm_ctrl_App(void *argument)
{
    CraneControlConfig config;
    RoutePlanningOutput output;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_plan_id = UINT32_MAX;
    float last_elapsed_s = -1.0f;

    (void)argument;
    CraneControl_LoadDefaultConfig(&config);
    CraneControl_CustomizeConfig(&config);
    (void)CraneControl_Init(&config);
    for (;;) {
        snapshot_planner_output(&output);
        if (output.plan_id != last_plan_id ||
            output.elapsed_s != last_elapsed_s) {
            if (CraneControl_SubmitPlannerOutput(&output) == CRANE_CONTROL_OK) {
                last_plan_id = output.plan_id;
                last_elapsed_s = output.elapsed_s;
            }
        }
        CraneControl_Update();
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(CRANE_CONTROL_TASK_PERIOD_MS));
    }
}

__weak void CraneControl_CustomizeConfig(CraneControlConfig *config)
{
    (void)config;
}

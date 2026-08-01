#include "crane_control.h"

#include "FreeRTOS.h"
#include "main.h"
#include "mission.h"
#include "task.h"

/* The task period sets how often physical arrival is polled. */
#define CRANE_CONTROL_TASK_PERIOD_MS CRANE_TICK_PERIOD_MS

static void snapshot_planner_output(RoutePlanningOutput *output)
{
    RoutePlanning_GetOutput(output);
}

/**
 * @brief Strong implementation of the CubeMX Robot_arm_ctrl FreeRTOS task.
 * @param argument Unused task argument.
 */
void Robot_arm_ctrl_App(void *argument)
{
    CraneControlConfig config;
    RoutePlanningOutput output;
    CraneControlState state;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_plan_id = UINT32_MAX;
    TrajectoryPhase last_phase = TRAJECTORY_PHASE_APPROACH;
    TrajectoryState last_state = TRAJECTORY_STATE_INVALID_ARGUMENT;
    uint8_t last_waypoint_index = UINT8_MAX;
    uint8_t confirmation_sent = 0U;

    (void)argument;
    CraneControl_LoadDefaultConfig(&config);
    CraneControl_CustomizeConfig(&config);
    if (CraneControl_Init(&config) == CRANE_CONTROL_OK) {
        Mission_SignalCraneReady();
    }
    for (;;) {
        snapshot_planner_output(&output);
        if (output.plan_id != last_plan_id ||
            output.phase != last_phase ||
            output.state != last_state ||
            output.waypoint_index != last_waypoint_index) {
            if (CraneControl_SubmitPlannerOutput(&output) == CRANE_CONTROL_OK) {
                last_plan_id = output.plan_id;
                last_phase = output.phase;
                last_state = output.state;
                last_waypoint_index = output.waypoint_index;
                confirmation_sent = 0U;
            }
        }
        CraneControl_Update();
        CraneControl_GetState(&state);
        if (confirmation_sent == 0U &&
            output.state == TRAJECTORY_STATE_RUNNING &&
            state.status == CRANE_CONTROL_OK &&
            state.axes_at_target != 0U &&
            state.plan_id == output.plan_id &&
            state.phase == output.phase &&
            state.waypoint_index == output.waypoint_index) {
            /* The last waypoint of either phase is directly above the pick or
               place point. Lower in the same control tick that observes both
               drives in position, before the asynchronous planner/decision
               hand-off can delay or lose the lift transition. */
            if ((uint8_t)(output.waypoint_index + 1U) ==
                output.waypoint_count) {
                if (CraneControl_CommandLift(CRANE_LIFT_LOWERED) !=
                    CRANE_CONTROL_OK) {
                    vTaskDelayUntil(&last_wake,
                                    pdMS_TO_TICKS(
                                        CRANE_CONTROL_TASK_PERIOD_MS));
                    continue;
                }
            }
            RoutePlanning_ConfirmWaypoint(output.plan_id, output.phase,
                                          output.waypoint_index);
            confirmation_sent = 1U;
        }
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(CRANE_CONTROL_TASK_PERIOD_MS));
    }
}

__weak void CraneControl_CustomizeConfig(CraneControlConfig *config)
{
    (void)config;
}

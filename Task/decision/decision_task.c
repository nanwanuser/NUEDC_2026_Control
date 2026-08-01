#include "decision_task.h"

#include "crane_control.h"
#include "route_planning.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#define DECISION_TASK_PERIOD_MS 1U

volatile DecisionTaskRequest DecisionTask_Input;
volatile uint8_t DecisionTask_RequestPending;
volatile DecisionTaskOutput DecisionTask_Output;
static uint32_t DecisionTask_NextRoutePlanId = 1U;

static void copy_request(DecisionTaskRequest *request)
{
    taskENTER_CRITICAL();
    *request = DecisionTask_Input;
    DecisionTask_RequestPending = 0U;
    taskEXIT_CRITICAL();
}

static void publish_output(const DecisionTaskOutput *output)
{
    taskENTER_CRITICAL();
    DecisionTask_Output = *output;
    taskEXIT_CRITICAL();
}

static uint32_t next_route_plan_id(void)
{
    uint32_t plan_id = DecisionTask_NextRoutePlanId;

    ++DecisionTask_NextRoutePlanId;
    if (DecisionTask_NextRoutePlanId == 0U) {
        DecisionTask_NextRoutePlanId = 1U;
    }
    return plan_id;
}

static uint32_t elapsed_milliseconds(uint32_t start_tick, uint32_t now_tick)
{
    uint32_t tick_frequency = osKernelGetTickFreq();

    if (tick_frequency == 0U) {
        return 0U;
    }
    return (uint32_t)(((uint64_t)(now_tick - start_tick) * 1000U) /
                      tick_frequency);
}

static uint8_t submit_move(const DecisionPlan *plan,
                           uint8_t move_index,
                           const TrajectoryPose *current_pose,
                           const TrajectoryLimits *limits,
                           uint32_t *route_plan_id)
{
    RoutePlanningRequest route_request;

    if (plan == NULL || current_pose == NULL || limits == NULL ||
        route_plan_id == NULL || move_index >= plan->move_count) {
        return 0U;
    }

    route_request.plan_id = next_route_plan_id();
    if (Decision_BuildTrajectoryRequest(&plan->moves[move_index],
                                        current_pose,
                                        limits,
                                        &route_request.trajectory) == 0U ||
        RoutePlanning_Submit(&route_request) == 0U) {
        return 0U;
    }

    *route_plan_id = route_request.plan_id;
    return 1U;
}

void DecisionTask_GetDefaultRequest(DecisionTaskRequest *request)
{
    if (request == NULL) {
        return;
    }

    (void)memset(request, 0, sizeof(*request));
    request->strategy = DECISION_STRATEGY_GEOMETRIC;
    Decision_GetDefaultConfig(&request->config);
    request->execution.current_pose.x_mm = 0.0f;
    request->execution.current_pose.y_mm = 0.0f;
    request->execution.current_pose.z_mm = request->config.transit_z_mm;
    request->execution.current_pose.yaw_deg = 0.0f;
    request->execution.limits.max_linear_velocity_mm_s = 120.0f;
    request->execution.limits.max_linear_acceleration_mm_s2 = 300.0f;
    request->execution.limits.max_yaw_velocity_deg_s = 90.0f;
    request->execution.limits.max_yaw_acceleration_deg_s2 = 180.0f;
    request->execution.lift_travel_ms = 350U;
    request->execution.grip_dwell_ms = 300U;
    request->execution.release_dwell_ms = 200U;
}

void DecisionTask_Init(void)
{
    DecisionTaskRequest request;

    DecisionTask_GetDefaultRequest(&request);
    DecisionTask_Input = request;
    (void)memset((void *)&DecisionTask_Output, 0, sizeof(DecisionTask_Output));
    DecisionTask_Output.result = DECISION_RESULT_INVALID_ARGUMENT;
    DecisionTask_Output.trajectory_result = TRAJECTORY_RESULT_INVALID_ARGUMENT;
    DecisionTask_Output.execution_state = DECISION_EXECUTION_IDLE;
    DecisionTask_RequestPending = 0U;
    DecisionTask_NextRoutePlanId = 1U;
}

uint8_t DecisionTask_Submit(const DecisionTaskRequest *request)
{
    if (request == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    DecisionTask_Input = *request;
    DecisionTask_RequestPending = 1U;
    /* Retire the previous run's outcome here rather than when this task next
       gets scheduled. The caller starts watching the output as soon as this
       returns, so a terminal COMPLETE or ERROR left over from the last request
       would otherwise be read as this one's result. */
    DecisionTask_Output.execution_state = DECISION_EXECUTION_IDLE;
    DecisionTask_Output.result = DECISION_RESULT_OK;
    DecisionTask_Output.trajectory_result = TRAJECTORY_RESULT_OK;
    DecisionTask_Output.active_move_index = 0U;
    DecisionTask_Output.plan.move_count = 0U;
    taskEXIT_CRITICAL();
    return 1U;
}

void Decision_App(void *argument)
{
    DecisionTaskRequest request;
    DecisionTaskOutput output;
    RoutePlanningOutput route_output;
    TrajectoryPose current_pose;
    uint32_t state_start_tick = 0U;
    uint8_t execution_active = 0U;

    (void)argument;
    (void)memset(&output, 0, sizeof(output));
    for (;;) {
        if (DecisionTask_RequestPending != 0U) {
            copy_request(&request);
            RoutePlanning_Cancel();
            (void)memset(&output, 0, sizeof(output));
            output.result = Decision_SolveStrategy(request.strategy,
                                                   &request.vision,
                                                   &request.card,
                                                   &request.config,
                                                   &output.plan);
            output.trajectory_result = TRAJECTORY_RESULT_INVALID_ARGUMENT;

            if (output.result == DECISION_RESULT_OK &&
                output.plan.move_count > 0U) {
                current_pose = request.execution.current_pose;
                output.active_move_index = 0U;
                if (submit_move(&output.plan,
                                output.active_move_index,
                                &current_pose,
                                &request.execution.limits,
                                &output.active_route_plan_id) != 0U) {
                    output.execution_state = DECISION_EXECUTION_WAITING_ROUTE;
                    execution_active = 1U;
                } else {
                    output.execution_state = DECISION_EXECUTION_ERROR;
                    execution_active = 0U;
                }
            } else {
                output.execution_state = DECISION_EXECUTION_ERROR;
                execution_active = 0U;
            }
            publish_output(&output);
        }

        if (execution_active != 0U) {
            uint32_t now_tick = osKernelGetTickCount();

            RoutePlanning_GetOutput(&route_output);
            if (route_output.plan_id == output.active_route_plan_id) {
                output.trajectory_result = route_output.result;

                if (route_output.result != TRAJECTORY_RESULT_OK) {
                    output.execution_state = DECISION_EXECUTION_ERROR;
                    execution_active = 0U;
                    RoutePlanning_Cancel();
                    publish_output(&output);
                } else if (output.execution_state ==
                           DECISION_EXECUTION_WAITING_ROUTE) {
                    if (route_output.active != 0U) {
                        output.execution_state = DECISION_EXECUTION_APPROACH;
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_APPROACH) {
                    if (route_output.phase == TRAJECTORY_PHASE_APPROACH &&
                        route_output.state == TRAJECTORY_STATE_COMPLETE) {
                        /* CraneControl lowers the lift in the same tick that
                           confirms the final approach waypoint. COMPLETE means
                           the stroke has already started. */
                        output.execution_state =
                            DECISION_EXECUTION_LOWER_PICK;
                        state_start_tick = now_tick;
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_LOWER_PICK) {
                    if (elapsed_milliseconds(state_start_tick, now_tick) >=
                        request.execution.lift_travel_ms) {
                        if (CraneControl_CommandGrip(1U) == CRANE_CONTROL_OK) {
                            output.execution_state =
                                DECISION_EXECUTION_GRIP_DWELL;
                            state_start_tick = now_tick;
                        } else {
                            output.execution_state = DECISION_EXECUTION_ERROR;
                            execution_active = 0U;
                        }
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_GRIP_DWELL) {
                    if (elapsed_milliseconds(state_start_tick, now_tick) >=
                        request.execution.grip_dwell_ms) {
                        if (CraneControl_CommandLift(CRANE_LIFT_RAISED) ==
                            CRANE_CONTROL_OK) {
                            output.execution_state =
                                DECISION_EXECUTION_RAISE_PICK;
                            state_start_tick = now_tick;
                        } else {
                            output.execution_state = DECISION_EXECUTION_ERROR;
                            execution_active = 0U;
                        }
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_RAISE_PICK) {
                    if (elapsed_milliseconds(state_start_tick, now_tick) >=
                        request.execution.lift_travel_ms) {
                        RoutePlanning_ResumeTransfer();
                        output.execution_state = DECISION_EXECUTION_TRANSFER;
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_TRANSFER) {
                    if (route_output.phase == TRAJECTORY_PHASE_TRANSFER &&
                        route_output.state == TRAJECTORY_STATE_COMPLETE &&
                        route_output.active == 0U) {
                        /* The route layer may add a shared wrist bias to keep the
                           carried piece inside the servo travel. Preserve that
                           actual final reference as the next move's empty-tool
                           start instead of reverting to the solver's raw place
                           yaw, which may be outside the wrist workspace. */
                        current_pose = route_output.reference.pose;
                        /* The final transfer waypoint starts the same direct
                           descent before it is confirmed complete. */
                        output.execution_state =
                            DECISION_EXECUTION_LOWER_PLACE;
                        state_start_tick = now_tick;
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_LOWER_PLACE) {
                    if (elapsed_milliseconds(state_start_tick, now_tick) >=
                        request.execution.lift_travel_ms) {
                        if (CraneControl_CommandGrip(0U) == CRANE_CONTROL_OK) {
                            output.execution_state =
                                DECISION_EXECUTION_RELEASE_DWELL;
                            state_start_tick = now_tick;
                        } else {
                            output.execution_state = DECISION_EXECUTION_ERROR;
                            execution_active = 0U;
                        }
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_RELEASE_DWELL) {
                    if (elapsed_milliseconds(state_start_tick, now_tick) >=
                        request.execution.release_dwell_ms) {
                        if (CraneControl_CommandLift(CRANE_LIFT_RAISED) ==
                            CRANE_CONTROL_OK) {
                            output.execution_state =
                                DECISION_EXECUTION_RAISE_PLACE;
                            state_start_tick = now_tick;
                        } else {
                            output.execution_state = DECISION_EXECUTION_ERROR;
                            execution_active = 0U;
                        }
                        publish_output(&output);
                    }
                } else if (output.execution_state ==
                           DECISION_EXECUTION_RAISE_PLACE) {
                    if (elapsed_milliseconds(state_start_tick, now_tick) >=
                        request.execution.lift_travel_ms) {
                        ++output.active_move_index;

                        if (output.active_move_index >= output.plan.move_count) {
                            output.execution_state = DECISION_EXECUTION_COMPLETE;
                            execution_active = 0U;
                        } else if (submit_move(
                                       &output.plan,
                                       output.active_move_index,
                                       &current_pose,
                                       &request.execution.limits,
                                       &output.active_route_plan_id) != 0U) {
                            output.execution_state =
                                DECISION_EXECUTION_WAITING_ROUTE;
                            output.trajectory_result =
                                TRAJECTORY_RESULT_OK;
                        } else {
                            output.execution_state = DECISION_EXECUTION_ERROR;
                            execution_active = 0U;
                        }
                        publish_output(&output);
                    }
                }
            }
        }
        osDelay(DECISION_TASK_PERIOD_MS);
    }
}

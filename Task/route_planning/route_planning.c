#include "route_planning.h"
#include "route_yaw_bias.h"

#include "crane_control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#define ROUTE_PLANNING_TASK_PERIOD_MS 1U

volatile RoutePlanningRequest RoutePlanning_Input;
volatile uint8_t RoutePlanning_RequestPending;
volatile uint8_t RoutePlanning_ResumeTransferRequested;
volatile RoutePlanningOutput RoutePlanning_Output;
static volatile uint8_t RoutePlanning_CancelRequested;
static volatile uint8_t RoutePlanning_WaypointConfirmed;
static volatile uint32_t RoutePlanning_ConfirmedPlanId;
static volatile TrajectoryPhase RoutePlanning_ConfirmedPhase;
static volatile uint8_t RoutePlanning_ConfirmedWaypointIndex;

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

static uint8_t take_cancel_request(void)
{
    uint8_t requested;

    taskENTER_CRITICAL();
    requested = RoutePlanning_CancelRequested;
    RoutePlanning_CancelRequested = 0U;
    taskEXIT_CRITICAL();
    return requested;
}

static uint8_t take_waypoint_confirmation(uint32_t *plan_id,
                                          TrajectoryPhase *phase,
                                          uint8_t *waypoint_index)
{
    uint8_t confirmed;

    taskENTER_CRITICAL();
    confirmed = RoutePlanning_WaypointConfirmed;
    if (confirmed != 0U) {
        *plan_id = RoutePlanning_ConfirmedPlanId;
        *phase = RoutePlanning_ConfirmedPhase;
        *waypoint_index = RoutePlanning_ConfirmedWaypointIndex;
        RoutePlanning_WaypointConfirmed = 0U;
    }
    taskEXIT_CRITICAL();
    return confirmed;
}

static void publish_output(const RoutePlanningOutput *output)
{
    taskENTER_CRITICAL();
    RoutePlanning_Output = *output;
    taskEXIT_CRITICAL();
}

/* The decision layer names only the points it cares about, and the trajectory
   interpolates straight between them. A straight leg between two reachable
   points can still cut inside the crane's minimum reach, because the chord of a
   wide boom sweep passes nearer the column than either end. Bridging poses that
   bulge the leg outwards are inserted here, before generation, so the plan the
   crane executes is one it can follow all the way through.

   Legs are widened in place, which is why the path is rebuilt rather than
   patched: an insertion shifts every later point. */
static uint8_t widen_path(TrajectoryPath *path)
{
    TrajectoryPath widened;
    uint8_t index;

    if (path->point_count == 0U) {
        return 1U;
    }
    if (CraneControl_CheckPose(&path->points[0]) != CRANE_CONTROL_OK) {
        return 0U;
    }
    widened.points[0] = path->points[0];
    widened.point_count = 1U;

    for (index = 1U; index < path->point_count; ++index) {
        const TrajectoryPose *next = &path->points[index];
        /* Room for the bridge, less this point and the ones still to be copied
           after it. Earlier insertions can use the budget up entirely, so the
           subtraction is done signed. */
        const int16_t remaining = (int16_t)(path->point_count - index - 1U);
        const int16_t free_slots = (int16_t)((int16_t)TRAJECTORY_MAX_WAYPOINTS -
                                             (int16_t)widened.point_count -
                                             remaining - 1);
        const uint8_t capacity = free_slots > 0 ? (uint8_t)free_slots : 0U;
        uint8_t inserted = 0U;

        if (CraneControl_CheckPose(next) != CRANE_CONTROL_OK) {
            return 0U;
        }
        if (CraneControl_PlanTransitPoses(
                &widened.points[widened.point_count - 1U], next,
                &widened.points[widened.point_count], capacity,
                &inserted) != CRANE_CONTROL_OK) {
            return 0U;
        }
        widened.point_count = (uint8_t)(widened.point_count + inserted + 1U);
        widened.points[widened.point_count - 1U] = *next;
    }

    *path = widened;
    return 1U;
}

/* The crane refuses out-of-reach references one sample at a time, which during a
   run looks like the arm stopping for no stated reason. Preparing the path up
   front turns that into an immediate planning failure naming the bad plan. */
static uint8_t prepare_path(TrajectoryRequest *trajectory)
{
    return (uint8_t)(widen_path(&trajectory->approach) != 0U &&
                     widen_path(&trajectory->transfer) != 0U);
}

static const TrajectoryPath *phase_path(const TrajectoryRequest *trajectory,
                                        TrajectoryPhase phase)
{
    return phase == TRAJECTORY_PHASE_APPROACH
               ? &trajectory->approach
               : &trajectory->transfer;
}

static void publish_waypoint(RoutePlanningOutput *output,
                             const TrajectoryRequest *trajectory,
                             TrajectoryPhase phase,
                             uint8_t waypoint_index)
{
    const TrajectoryPath *path = phase_path(trajectory, phase);

    output->phase = phase;
    output->state = TRAJECTORY_STATE_RUNNING;
    output->elapsed_s = 0.0f;
    output->reference.pose = path->points[waypoint_index];
    output->reference.grip = phase == TRAJECTORY_PHASE_TRANSFER ? 1U : 0U;
    output->waypoint_index = waypoint_index;
    output->waypoint_count = path->point_count;
    output->active = 1U;
    publish_output(output);
}

void RoutePlanning_Init(void)
{
    (void)memset((void *)&RoutePlanning_Input, 0, sizeof(RoutePlanning_Input));
    (void)memset((void *)&RoutePlanning_Output, 0, sizeof(RoutePlanning_Output));
    RoutePlanning_Output.state = TRAJECTORY_STATE_INVALID_ARGUMENT;
    RoutePlanning_Output.result = TRAJECTORY_RESULT_INVALID_ARGUMENT;
    RoutePlanning_RequestPending = 0U;
    RoutePlanning_ResumeTransferRequested = 0U;
    RoutePlanning_CancelRequested = 0U;
    RoutePlanning_WaypointConfirmed = 0U;
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
    RoutePlanning_CancelRequested = 0U;
    RoutePlanning_WaypointConfirmed = 0U;
    taskEXIT_CRITICAL();

    return 1U;
}

void RoutePlanning_ResumeTransfer(void)
{
    taskENTER_CRITICAL();
    RoutePlanning_ResumeTransferRequested = 1U;
    taskEXIT_CRITICAL();
}

void RoutePlanning_ConfirmWaypoint(uint32_t plan_id,
                                   TrajectoryPhase phase,
                                   uint8_t waypoint_index)
{
    taskENTER_CRITICAL();
    RoutePlanning_ConfirmedPlanId = plan_id;
    RoutePlanning_ConfirmedPhase = phase;
    RoutePlanning_ConfirmedWaypointIndex = waypoint_index;
    RoutePlanning_WaypointConfirmed = 1U;
    taskEXIT_CRITICAL();
}

void RoutePlanning_Cancel(void)
{
    taskENTER_CRITICAL();
    RoutePlanning_RequestPending = 0U;
    RoutePlanning_ResumeTransferRequested = 0U;
    RoutePlanning_WaypointConfirmed = 0U;
    RoutePlanning_CancelRequested = 1U;
    taskEXIT_CRITICAL();
}

void RoutePlanning_GetOutput(RoutePlanningOutput *output)
{
    if (output == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *output = RoutePlanning_Output;
    taskEXIT_CRITICAL();
}

void Route_planning_App(void *argument)
{
    RoutePlanningRequest request;
    RoutePlanningOutput output;
    TrajectoryPlan plan;
    TrajectoryPhase phase = TRAJECTORY_PHASE_APPROACH;
    TrajectoryResult result;
    uint32_t active_plan_id = 0U;
    uint8_t plan_active = 0U;
    uint8_t waypoint_index = 0U;

    (void)argument;
    (void)memset(&output, 0, sizeof(output));
    output.state = TRAJECTORY_STATE_INVALID_ARGUMENT;
    output.result = TRAJECTORY_RESULT_INVALID_ARGUMENT;

    for (;;) {
        if (RoutePlanning_CancelRequested != 0U) {
            (void)take_cancel_request();
            plan_active = 0U;
            output.active = 0U;
            publish_output(&output);
        }

        if (RoutePlanning_RequestPending != 0U) {
            copy_input(&request);
            RoutePlanning_ApplyYawBias(&request.trajectory);
            result = prepare_path(&request.trajectory) != 0U
                         ? Trajectory_Generate(&request.trajectory, &plan)
                         : TRAJECTORY_RESULT_INVALID_ARGUMENT;
            active_plan_id = request.plan_id;
            phase = TRAJECTORY_PHASE_APPROACH;
            (void)take_resume_request();
            RoutePlanning_WaypointConfirmed = 0U;

            if (result == TRAJECTORY_RESULT_OK) {
                plan_active = 1U;
                output.plan_id = active_plan_id;
                output.result = result;
                waypoint_index = 1U;
                publish_waypoint(&output, &request.trajectory, phase,
                                 waypoint_index);
            } else {
                plan_active = 0U;
                output.plan_id = active_plan_id;
                output.phase = phase;
                output.state = TRAJECTORY_STATE_INVALID_ARGUMENT;
                output.result = result;
                output.elapsed_s = 0.0f;
                (void)memset(&output.reference, 0, sizeof(output.reference));
                output.waypoint_index = 0U;
                output.waypoint_count = 0U;
                output.active = 0U;
                publish_output(&output);
            }
        }

        if (plan_active != 0U) {
            uint32_t confirmed_plan_id;
            TrajectoryPhase confirmed_phase;
            uint8_t confirmed_waypoint_index;

            if (take_waypoint_confirmation(&confirmed_plan_id,
                                           &confirmed_phase,
                                           &confirmed_waypoint_index) != 0U &&
                output.state == TRAJECTORY_STATE_RUNNING &&
                confirmed_plan_id == active_plan_id &&
                confirmed_phase == phase &&
                confirmed_waypoint_index == waypoint_index) {
                const TrajectoryPath *path = phase_path(&request.trajectory,
                                                        phase);

                if ((uint8_t)(waypoint_index + 1U) < path->point_count) {
                    ++waypoint_index;
                    publish_waypoint(&output, &request.trajectory, phase,
                                     waypoint_index);
                } else {
                    output.state = TRAJECTORY_STATE_COMPLETE;
                    output.active = phase == TRAJECTORY_PHASE_APPROACH ? 1U : 0U;
                    publish_output(&output);
                    if (phase == TRAJECTORY_PHASE_TRANSFER) {
                        plan_active = 0U;
                    }
                }
            }

            if (phase == TRAJECTORY_PHASE_APPROACH &&
                output.state == TRAJECTORY_STATE_COMPLETE &&
                RoutePlanning_ResumeTransferRequested != 0U) {
                (void)take_resume_request();
                phase = TRAJECTORY_PHASE_TRANSFER;
                waypoint_index = 1U;
                publish_waypoint(&output, &request.trajectory, phase,
                                 waypoint_index);
            }
        }

        osDelay(ROUTE_PLANNING_TASK_PERIOD_MS);
    }
}

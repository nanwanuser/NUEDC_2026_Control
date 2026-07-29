#include "trajectory.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TRAJECTORY_YAW_PERIOD_DEG          360.0f
#define TRAJECTORY_YAW_HALF_PERIOD_DEG     180.0f
#define TRAJECTORY_QUINTIC_MAX_VELOCITY    1.875f
#define TRAJECTORY_QUINTIC_MAX_ACCELERATION 5.773503f

enum {
    TRAJECTORY_AXIS_X = 0,
    TRAJECTORY_AXIS_Y,
    TRAJECTORY_AXIS_Z,
    TRAJECTORY_AXIS_YAW
};

static float max_float(float left, float right)
{
    return left > right ? left : right;
}

static float normalize_yaw(float yaw_deg)
{
    float normalized = fmodf(yaw_deg + TRAJECTORY_YAW_HALF_PERIOD_DEG,
                             TRAJECTORY_YAW_PERIOD_DEG);

    if (normalized < 0.0f) {
        normalized += TRAJECTORY_YAW_PERIOD_DEG;
    }

    return normalized - TRAJECTORY_YAW_HALF_PERIOD_DEG;
}

static float shortest_yaw_delta(float from_deg, float to_deg)
{
    return normalize_yaw(to_deg - from_deg);
}

static uint8_t pose_is_finite(const TrajectoryPose *pose)
{
    return (uint8_t)(isfinite(pose->x_mm) &&
                     isfinite(pose->y_mm) &&
                     isfinite(pose->z_mm) &&
                     isfinite(pose->yaw_deg));
}

static uint8_t limits_are_finite(const TrajectoryLimits *limits)
{
    return (uint8_t)(isfinite(limits->max_linear_velocity_mm_s) &&
                     isfinite(limits->max_linear_acceleration_mm_s2) &&
                     isfinite(limits->max_yaw_velocity_deg_s) &&
                     isfinite(limits->max_yaw_acceleration_deg_s2));
}

static uint8_t limits_are_positive(const TrajectoryLimits *limits)
{
    return (uint8_t)(limits->max_linear_velocity_mm_s > 0.0f &&
                     limits->max_linear_acceleration_mm_s2 > 0.0f &&
                     limits->max_yaw_velocity_deg_s > 0.0f &&
                     limits->max_yaw_acceleration_deg_s2 > 0.0f);
}

static void pose_to_axes(const TrajectoryPose *pose, float axes[TRAJECTORY_AXIS_COUNT])
{
    axes[TRAJECTORY_AXIS_X] = pose->x_mm;
    axes[TRAJECTORY_AXIS_Y] = pose->y_mm;
    axes[TRAJECTORY_AXIS_Z] = pose->z_mm;
    axes[TRAJECTORY_AXIS_YAW] = pose->yaw_deg;
}

static void axes_to_pose(const float axes[TRAJECTORY_AXIS_COUNT], TrajectoryPose *pose)
{
    pose->x_mm = axes[TRAJECTORY_AXIS_X];
    pose->y_mm = axes[TRAJECTORY_AXIS_Y];
    pose->z_mm = axes[TRAJECTORY_AXIS_Z];
    pose->yaw_deg = normalize_yaw(axes[TRAJECTORY_AXIS_YAW]);
}

static float approach_duration(const TrajectoryRequest *request)
{
    float dx = request->pick.x_mm - request->current.x_mm;
    float dy = request->pick.y_mm - request->current.y_mm;
    float dz = request->pick.z_mm - request->current.z_mm;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    float yaw_distance = fabsf(shortest_yaw_delta(request->current.yaw_deg,
                                                  request->pick.yaw_deg));
    float duration = 0.0f;

    duration = max_float(duration,
                         TRAJECTORY_QUINTIC_MAX_VELOCITY * distance /
                         request->limits.max_linear_velocity_mm_s);
    duration = max_float(duration,
                         sqrtf(TRAJECTORY_QUINTIC_MAX_ACCELERATION * distance /
                               request->limits.max_linear_acceleration_mm_s2));
    duration = max_float(duration,
                         TRAJECTORY_QUINTIC_MAX_VELOCITY * yaw_distance /
                         request->limits.max_yaw_velocity_deg_s);
    duration = max_float(duration,
                         sqrtf(TRAJECTORY_QUINTIC_MAX_ACCELERATION * yaw_distance /
                               request->limits.max_yaw_acceleration_deg_s2));

    return duration;
}

static void generate_rest_to_rest(const TrajectoryPose *start,
                                  const TrajectoryPose *end,
                                  float duration_s,
                                  TrajectorySegment *segment)
{
    float start_axes[TRAJECTORY_AXIS_COUNT];
    float end_axes[TRAJECTORY_AXIS_COUNT];
    uint32_t axis;

    pose_to_axes(start, start_axes);
    pose_to_axes(end, end_axes);
    end_axes[TRAJECTORY_AXIS_YAW] =
        start_axes[TRAJECTORY_AXIS_YAW] +
        shortest_yaw_delta(start_axes[TRAJECTORY_AXIS_YAW],
                           end_axes[TRAJECTORY_AXIS_YAW]);

    memset(segment, 0, sizeof(*segment));
    segment->duration_s = duration_s;

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        float delta = end_axes[axis] - start_axes[axis];

        segment->coefficient[axis][0] = start_axes[axis];
        segment->coefficient[axis][3] = 10.0f * delta;
        segment->coefficient[axis][4] = -15.0f * delta;
        segment->coefficient[axis][5] = 6.0f * delta;
    }
}

static float evaluate_polynomial(const float coefficient[TRAJECTORY_COEFFICIENT_COUNT],
                                 float u)
{
    float value = coefficient[TRAJECTORY_COEFFICIENT_COUNT - 1U];
    int32_t index;

    for (index = (int32_t)TRAJECTORY_COEFFICIENT_COUNT - 2; index >= 0; --index) {
        value = value * u + coefficient[index];
    }

    return value;
}

static void evaluate_segment(const TrajectorySegment *segment,
                             float time_s,
                             TrajectoryPose *pose)
{
    float axes[TRAJECTORY_AXIS_COUNT];
    float u;
    uint32_t axis;

    if (segment->duration_s <= 0.0f || time_s >= segment->duration_s) {
        u = 1.0f;
    } else if (time_s <= 0.0f) {
        u = 0.0f;
    } else {
        u = time_s / segment->duration_s;
    }

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        axes[axis] = evaluate_polynomial(segment->coefficient[axis], u);
    }

    axes_to_pose(axes, pose);
}

TrajectoryResult Trajectory_Generate(const TrajectoryRequest *request,
                                     TrajectoryPlan *plan)
{
    float duration_s;

    if (request == NULL || plan == NULL) {
        return TRAJECTORY_RESULT_INVALID_ARGUMENT;
    }

    if (!pose_is_finite(&request->current) ||
        !pose_is_finite(&request->pick) ||
        !pose_is_finite(&request->transit) ||
        !pose_is_finite(&request->place) ||
        !limits_are_finite(&request->limits)) {
        return TRAJECTORY_RESULT_INVALID_ARGUMENT;
    }

    if (!limits_are_positive(&request->limits)) {
        return TRAJECTORY_RESULT_INVALID_LIMIT;
    }

    memset(plan, 0, sizeof(*plan));
    duration_s = approach_duration(request);
    if (!isfinite(duration_s)) {
        return TRAJECTORY_RESULT_NUMERIC_ERROR;
    }

    generate_rest_to_rest(&request->current,
                          &request->pick,
                          duration_s,
                          &plan->approach);
    generate_rest_to_rest(&request->pick,
                          &request->transit,
                          0.0f,
                          &plan->transfer[0]);
    generate_rest_to_rest(&request->transit,
                          &request->place,
                          0.0f,
                          &plan->transfer[1]);

    return TRAJECTORY_RESULT_OK;
}

TrajectoryState Trajectory_Evaluate(const TrajectoryPlan *plan,
                                    TrajectoryPhase phase,
                                    float time_s,
                                    TrajectoryReference *reference)
{
    if (plan == NULL || reference == NULL || !isfinite(time_s)) {
        return TRAJECTORY_STATE_INVALID_ARGUMENT;
    }

    if (phase != TRAJECTORY_PHASE_APPROACH &&
        phase != TRAJECTORY_PHASE_TRANSFER) {
        return TRAJECTORY_STATE_INVALID_PHASE;
    }

    if (phase == TRAJECTORY_PHASE_APPROACH) {
        evaluate_segment(&plan->approach, time_s, &reference->pose);
        if (plan->approach.duration_s <= 0.0f ||
            time_s >= plan->approach.duration_s) {
            reference->grip = 1U;
            return TRAJECTORY_STATE_COMPLETE;
        }

        reference->grip = 0U;
        return TRAJECTORY_STATE_RUNNING;
    }

    evaluate_segment(&plan->transfer[1], time_s, &reference->pose);
    reference->grip = 0U;
    return TRAJECTORY_STATE_COMPLETE;
}

float Trajectory_GetDuration(const TrajectoryPlan *plan,
                             TrajectoryPhase phase)
{
    if (plan == NULL) {
        return 0.0f;
    }

    if (phase == TRAJECTORY_PHASE_APPROACH) {
        return plan->approach.duration_s;
    }
    if (phase == TRAJECTORY_PHASE_TRANSFER) {
        return plan->transfer_duration_s;
    }

    return 0.0f;
}

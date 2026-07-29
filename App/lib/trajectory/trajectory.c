#include "trajectory.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TRAJECTORY_YAW_PERIOD_DEG          360.0f
#define TRAJECTORY_YAW_HALF_PERIOD_DEG     180.0f
#define TRAJECTORY_QUINTIC_MAX_VELOCITY    1.875f
#define TRAJECTORY_QUINTIC_MAX_ACCELERATION 5.773503f
#define TRAJECTORY_DURATION_SCALE_MARGIN   1.0001f

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

static float clamp_abs(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
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

static void pose_to_unwrapped_axes(const TrajectoryPose *pose,
                                   float previous_yaw_deg,
                                   float axes[TRAJECTORY_AXIS_COUNT])
{
    pose_to_axes(pose, axes);
    axes[TRAJECTORY_AXIS_YAW] =
        previous_yaw_deg + shortest_yaw_delta(previous_yaw_deg, pose->yaw_deg);
}

static float motion_duration(const float start[TRAJECTORY_AXIS_COUNT],
                             const float end[TRAJECTORY_AXIS_COUNT],
                             const TrajectoryLimits *limits)
{
    float dx = end[TRAJECTORY_AXIS_X] - start[TRAJECTORY_AXIS_X];
    float dy = end[TRAJECTORY_AXIS_Y] - start[TRAJECTORY_AXIS_Y];
    float dz = end[TRAJECTORY_AXIS_Z] - start[TRAJECTORY_AXIS_Z];
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    float yaw_distance = fabsf(end[TRAJECTORY_AXIS_YAW] -
                               start[TRAJECTORY_AXIS_YAW]);
    float duration = 0.0f;

    duration = max_float(duration,
                         TRAJECTORY_QUINTIC_MAX_VELOCITY * distance /
                         limits->max_linear_velocity_mm_s);
    duration = max_float(duration,
                         sqrtf(TRAJECTORY_QUINTIC_MAX_ACCELERATION * distance /
                               limits->max_linear_acceleration_mm_s2));
    duration = max_float(duration,
                         TRAJECTORY_QUINTIC_MAX_VELOCITY * yaw_distance /
                         limits->max_yaw_velocity_deg_s);
    duration = max_float(duration,
                         sqrtf(TRAJECTORY_QUINTIC_MAX_ACCELERATION * yaw_distance /
                               limits->max_yaw_acceleration_deg_s2));

    return duration;
}

static void generate_quintic_segment(
    const float start[TRAJECTORY_AXIS_COUNT],
    const float end[TRAJECTORY_AXIS_COUNT],
    const float start_velocity[TRAJECTORY_AXIS_COUNT],
    const float end_velocity[TRAJECTORY_AXIS_COUNT],
    const float start_acceleration[TRAJECTORY_AXIS_COUNT],
    const float end_acceleration[TRAJECTORY_AXIS_COUNT],
    float duration_s,
    TrajectorySegment *segment)
{
    uint32_t axis;

    memset(segment, 0, sizeof(*segment));
    segment->duration_s = duration_s;

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        float duration_squared = duration_s * duration_s;
        float c0 = start[axis];
        float c1 = start_velocity[axis] * duration_s;
        float c2 = 0.5f * start_acceleration[axis] * duration_squared;
        float position_residual = end[axis] - (c0 + c1 + c2);
        float velocity_residual = end_velocity[axis] * duration_s -
                                  (c1 + 2.0f * c2);
        float acceleration_residual = end_acceleration[axis] * duration_squared -
                                      2.0f * c2;

        segment->coefficient[axis][0] = c0;
        segment->coefficient[axis][1] = c1;
        segment->coefficient[axis][2] = c2;
        segment->coefficient[axis][3] = 10.0f * position_residual -
                                        4.0f * velocity_residual +
                                        0.5f * acceleration_residual;
        segment->coefficient[axis][4] = -15.0f * position_residual +
                                        7.0f * velocity_residual -
                                        acceleration_residual;
        segment->coefficient[axis][5] = 6.0f * position_residual -
                                        3.0f * velocity_residual +
                                        0.5f * acceleration_residual;
    }
}

static float limited_transit_velocity(float start,
                                      float transit,
                                      float end,
                                      float first_duration_s,
                                      float second_duration_s)
{
    float first_secant;
    float second_secant;
    float velocity;
    float limit;

    if (first_duration_s <= 0.0f || second_duration_s <= 0.0f) {
        return 0.0f;
    }

    first_secant = (transit - start) / first_duration_s;
    second_secant = (end - transit) / second_duration_s;

    if (first_secant * second_secant <= 0.0f) {
        return 0.0f;
    }

    velocity = 2.0f * first_secant * second_secant /
               (first_secant + second_secant);
    limit = 3.0f * fminf(fabsf(first_secant), fabsf(second_secant));
    return clamp_abs(velocity, limit);
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

static void power_to_bezier(
    const float coefficient[TRAJECTORY_COEFFICIENT_COUNT],
    float bezier[TRAJECTORY_COEFFICIENT_COUNT])
{
    bezier[0] = coefficient[0];
    bezier[1] = coefficient[0] + coefficient[1] / 5.0f;
    bezier[2] = coefficient[0] + 2.0f * coefficient[1] / 5.0f +
                coefficient[2] / 10.0f;
    bezier[3] = coefficient[0] + 3.0f * coefficient[1] / 5.0f +
                3.0f * coefficient[2] / 10.0f + coefficient[3] / 10.0f;
    bezier[4] = coefficient[0] + 4.0f * coefficient[1] / 5.0f +
                3.0f * coefficient[2] / 5.0f +
                2.0f * coefficient[3] / 5.0f + coefficient[4] / 5.0f;
    bezier[5] = coefficient[0] + coefficient[1] + coefficient[2] +
                coefficient[3] + coefficient[4] + coefficient[5];
}

static float segment_required_scale(const TrajectorySegment *segment,
                                    const TrajectoryLimits *limits)
{
    float bezier[TRAJECTORY_AXIS_COUNT][TRAJECTORY_COEFFICIENT_COUNT];
    float first[TRAJECTORY_AXIS_COUNT][5];
    float second[TRAJECTORY_AXIS_COUNT][4];
    float max_linear_velocity = 0.0f;
    float max_linear_acceleration = 0.0f;
    float max_yaw_velocity = 0.0f;
    float max_yaw_acceleration = 0.0f;
    float duration_squared;
    float scale = 1.0f;
    uint32_t axis;
    uint32_t index;

    if (segment->duration_s <= 0.0f) {
        return 1.0f;
    }

    duration_squared = segment->duration_s * segment->duration_s;
    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        power_to_bezier(segment->coefficient[axis], bezier[axis]);
        for (index = 0U; index < 5U; ++index) {
            first[axis][index] = 5.0f *
                (bezier[axis][index + 1U] - bezier[axis][index]);
        }
        for (index = 0U; index < 4U; ++index) {
            second[axis][index] = 4.0f *
                (first[axis][index + 1U] - first[axis][index]);
        }
    }

    for (index = 0U; index < 5U; ++index) {
        float linear_velocity = sqrtf(
            first[TRAJECTORY_AXIS_X][index] * first[TRAJECTORY_AXIS_X][index] +
            first[TRAJECTORY_AXIS_Y][index] * first[TRAJECTORY_AXIS_Y][index] +
            first[TRAJECTORY_AXIS_Z][index] * first[TRAJECTORY_AXIS_Z][index]) /
            segment->duration_s;
        float yaw_velocity =
            fabsf(first[TRAJECTORY_AXIS_YAW][index]) / segment->duration_s;

        max_linear_velocity = max_float(max_linear_velocity, linear_velocity);
        max_yaw_velocity = max_float(max_yaw_velocity, yaw_velocity);
    }

    for (index = 0U; index < 4U; ++index) {
        float linear_acceleration = sqrtf(
            second[TRAJECTORY_AXIS_X][index] * second[TRAJECTORY_AXIS_X][index] +
            second[TRAJECTORY_AXIS_Y][index] * second[TRAJECTORY_AXIS_Y][index] +
            second[TRAJECTORY_AXIS_Z][index] * second[TRAJECTORY_AXIS_Z][index]) /
            duration_squared;
        float yaw_acceleration =
            fabsf(second[TRAJECTORY_AXIS_YAW][index]) / duration_squared;

        max_linear_acceleration = max_float(max_linear_acceleration,
                                            linear_acceleration);
        max_yaw_acceleration = max_float(max_yaw_acceleration,
                                         yaw_acceleration);
    }

    scale = max_float(scale,
                      max_linear_velocity /
                      limits->max_linear_velocity_mm_s);
    scale = max_float(scale,
                      sqrtf(max_linear_acceleration /
                            limits->max_linear_acceleration_mm_s2));
    scale = max_float(scale,
                      max_yaw_velocity /
                      limits->max_yaw_velocity_deg_s);
    scale = max_float(scale,
                      sqrtf(max_yaw_acceleration /
                            limits->max_yaw_acceleration_deg_s2));

    return scale;
}

static uint8_t scale_plan_durations(TrajectoryPlan *plan,
                                    const TrajectoryLimits *limits)
{
    float approach_scale = segment_required_scale(&plan->approach, limits);
    float transfer_scale = max_float(
        segment_required_scale(&plan->transfer[0], limits),
        segment_required_scale(&plan->transfer[1], limits));

    if (!isfinite(approach_scale) || !isfinite(transfer_scale)) {
        return 0U;
    }

    if (approach_scale > 1.0f) {
        plan->approach.duration_s *=
            approach_scale * TRAJECTORY_DURATION_SCALE_MARGIN;
    }

    if (transfer_scale > 1.0f) {
        float scale = transfer_scale * TRAJECTORY_DURATION_SCALE_MARGIN;

        plan->transfer[0].duration_s *= scale;
        plan->transfer[1].duration_s *= scale;
    }

    plan->transfer_duration_s = plan->transfer[0].duration_s +
                                plan->transfer[1].duration_s;
    return (uint8_t)(isfinite(plan->approach.duration_s) &&
                     isfinite(plan->transfer_duration_s));
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
    float approach_start[TRAJECTORY_AXIS_COUNT];
    float approach_end[TRAJECTORY_AXIS_COUNT];
    float transfer_start[TRAJECTORY_AXIS_COUNT];
    float transfer_transit[TRAJECTORY_AXIS_COUNT];
    float transfer_end[TRAJECTORY_AXIS_COUNT];
    float zero[TRAJECTORY_AXIS_COUNT] = {0.0f};
    float transit_velocity[TRAJECTORY_AXIS_COUNT];
    float approach_duration_s;
    float first_duration_s;
    float second_duration_s;
    uint32_t axis;

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
    pose_to_axes(&request->current, approach_start);
    pose_to_unwrapped_axes(&request->pick,
                           approach_start[TRAJECTORY_AXIS_YAW],
                           approach_end);
    approach_duration_s = motion_duration(approach_start,
                                          approach_end,
                                          &request->limits);

    pose_to_axes(&request->pick, transfer_start);
    pose_to_unwrapped_axes(&request->transit,
                           transfer_start[TRAJECTORY_AXIS_YAW],
                           transfer_transit);
    pose_to_unwrapped_axes(&request->place,
                           transfer_transit[TRAJECTORY_AXIS_YAW],
                           transfer_end);
    first_duration_s = motion_duration(transfer_start,
                                       transfer_transit,
                                       &request->limits);
    second_duration_s = motion_duration(transfer_transit,
                                        transfer_end,
                                        &request->limits);

    if (!isfinite(approach_duration_s) ||
        !isfinite(first_duration_s) ||
        !isfinite(second_duration_s)) {
        return TRAJECTORY_RESULT_NUMERIC_ERROR;
    }

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        transit_velocity[axis] = limited_transit_velocity(
            transfer_start[axis],
            transfer_transit[axis],
            transfer_end[axis],
            first_duration_s,
            second_duration_s);
    }

    generate_quintic_segment(approach_start,
                              approach_end,
                              zero,
                              zero,
                              zero,
                              zero,
                              approach_duration_s,
                              &plan->approach);
    generate_quintic_segment(transfer_start,
                              transfer_transit,
                              zero,
                              transit_velocity,
                              zero,
                              zero,
                              first_duration_s,
                              &plan->transfer[0]);
    generate_quintic_segment(transfer_transit,
                              transfer_end,
                              transit_velocity,
                              zero,
                              zero,
                              zero,
                              second_duration_s,
                              &plan->transfer[1]);
    plan->transfer_duration_s = first_duration_s + second_duration_s;

    if (!isfinite(plan->transfer_duration_s) ||
        !scale_plan_durations(plan, &request->limits)) {
        return TRAJECTORY_RESULT_NUMERIC_ERROR;
    }

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

    if (plan->transfer_duration_s <= 0.0f) {
        evaluate_segment(&plan->transfer[1], 0.0f, &reference->pose);
        reference->grip = 0U;
        return TRAJECTORY_STATE_COMPLETE;
    }

    if (time_s >= plan->transfer_duration_s) {
        evaluate_segment(&plan->transfer[1],
                         plan->transfer[1].duration_s,
                         &reference->pose);
        reference->grip = 0U;
        return TRAJECTORY_STATE_COMPLETE;
    }

    if (plan->transfer[0].duration_s > 0.0f &&
        time_s < plan->transfer[0].duration_s) {
        evaluate_segment(&plan->transfer[0], time_s, &reference->pose);
    } else {
        evaluate_segment(&plan->transfer[1],
                         time_s - plan->transfer[0].duration_s,
                         &reference->pose);
    }

    reference->grip = 1U;
    return TRAJECTORY_STATE_RUNNING;
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

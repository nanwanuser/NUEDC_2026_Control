#include "trajectory.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define ASSERT_TRUE(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            fprintf(stderr, "%s:%d assertion failed: %s\n",                    \
                    __FILE__, __LINE__, #condition);                              \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

#define ASSERT_EQ_INT(expected, actual)                                          \
    do {                                                                         \
        int expected_value = (int)(expected);                                     \
        int actual_value = (int)(actual);                                         \
        if (expected_value != actual_value) {                                     \
            fprintf(stderr, "%s:%d expected %d, got %d\n",                     \
                    __FILE__, __LINE__, expected_value, actual_value);             \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

#define ASSERT_NEAR(expected, actual, tolerance)                                 \
    do {                                                                         \
        float expected_value = (float)(expected);                                 \
        float actual_value = (float)(actual);                                     \
        float tolerance_value = (float)(tolerance);                               \
        if (fabsf(expected_value - actual_value) > tolerance_value) {             \
            fprintf(stderr, "%s:%d expected %.7g, got %.7g (tol %.7g)\n",       \
                    __FILE__, __LINE__, expected_value, actual_value,              \
                    tolerance_value);                                              \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

static TrajectoryRequest valid_request(void)
{
    const TrajectoryRequest request = {
        .current = {20.0f, 30.0f, 45.0f, 170.0f},
        .pick = {80.0f, 60.0f, 5.0f, -170.0f},
        .transit = {120.0f, 130.0f, 50.0f, -120.0f},
        .place = {160.0f, 200.0f, 5.0f, -90.0f},
        .limits = {120.0f, 300.0f, 90.0f, 180.0f}
    };

    return request;
}

static void assert_pose_near(TrajectoryPose expected,
                             TrajectoryPose actual,
                             float tolerance)
{
    ASSERT_NEAR(expected.x_mm, actual.x_mm, tolerance);
    ASSERT_NEAR(expected.y_mm, actual.y_mm, tolerance);
    ASSERT_NEAR(expected.z_mm, actual.z_mm, tolerance);
    ASSERT_NEAR(expected.yaw_deg, actual.yaw_deg, tolerance);
}

static float evaluate_axis_derivative(const TrajectorySegment *segment,
                                      uint32_t axis,
                                      float local_time_s,
                                      uint32_t order)
{
    float u;
    float value = 0.0f;
    uint32_t coefficient_index;

    ASSERT_TRUE(segment != NULL);
    ASSERT_TRUE(axis < TRAJECTORY_AXIS_COUNT);
    ASSERT_TRUE(order == 1U || order == 2U);

    if (segment->duration_s <= 0.0f) {
        return 0.0f;
    }

    u = local_time_s / segment->duration_s;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;

    for (coefficient_index = order;
         coefficient_index < TRAJECTORY_COEFFICIENT_COUNT;
         ++coefficient_index) {
        float multiplier = (float)coefficient_index;
        uint32_t power = coefficient_index - order;

        if (order == 2U) {
            multiplier *= (float)(coefficient_index - 1U);
        }

        value += multiplier *
                 segment->coefficient[axis][coefficient_index] *
                 powf(u, (float)power);
    }

    if (order == 1U) {
        return value / segment->duration_s;
    }

    return value / (segment->duration_s * segment->duration_s);
}

static void test_approach_pose_and_grip_output(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    TrajectoryReference reference;
    float duration;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    duration = Trajectory_GetDuration(&plan, TRAJECTORY_PHASE_APPROACH);
    ASSERT_TRUE(duration > 0.0f);

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      0.0f, &reference));
    assert_pose_near(request.current, reference.pose, 1.0e-5f);
    ASSERT_EQ_INT(0, reference.grip);

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      duration * 0.5f, &reference));
    ASSERT_NEAR(50.0f, reference.pose.x_mm, 1.0e-4f);
    ASSERT_NEAR(45.0f, reference.pose.y_mm, 1.0e-4f);
    ASSERT_NEAR(25.0f, reference.pose.z_mm, 1.0e-4f);
    ASSERT_NEAR(-180.0f, reference.pose.yaw_deg, 1.0e-3f);
    ASSERT_EQ_INT(0, reference.grip);

    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      duration, &reference));
    assert_pose_near(request.pick, reference.pose, 1.0e-4f);
    ASSERT_EQ_INT(1, reference.grip);

    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      duration + 1.0f, &reference));
    ASSERT_EQ_INT(1, reference.grip);
}

static void test_invalid_request_is_rejected(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_ARGUMENT,
                  Trajectory_Generate(NULL, &plan));
    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_ARGUMENT,
                  Trajectory_Generate(&request, NULL));

    request.current.x_mm = NAN;
    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_ARGUMENT,
                  Trajectory_Generate(&request, &plan));

    request = valid_request();
    request.limits.max_linear_velocity_mm_s = 0.0f;
    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_LIMIT,
                  Trajectory_Generate(&request, &plan));
}

static void test_transfer_passes_transit_and_controls_grip(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    TrajectoryReference reference;
    float transit_time;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    transit_time = plan.transfer[0].duration_s;
    ASSERT_TRUE(transit_time > 0.0f);
    ASSERT_TRUE(plan.transfer_duration_s > transit_time);

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      0.0f, &reference));
    assert_pose_near(request.pick, reference.pose, 1.0e-4f);
    ASSERT_EQ_INT(1, reference.grip);

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      transit_time, &reference));
    assert_pose_near(request.transit, reference.pose, 1.0e-3f);
    ASSERT_EQ_INT(1, reference.grip);

    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      plan.transfer_duration_s, &reference));
    assert_pose_near(request.place, reference.pose, 1.0e-3f);
    ASSERT_EQ_INT(0, reference.grip);

    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      plan.transfer_duration_s + 1.0f,
                                      &reference));
    ASSERT_EQ_INT(0, reference.grip);
}

static void test_transfer_is_c2_and_does_not_stop_at_transit(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    float linear_speed_squared = 0.0f;
    uint32_t axis;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        float left_velocity = evaluate_axis_derivative(
            &plan.transfer[0], axis, plan.transfer[0].duration_s, 1U);
        float right_velocity = evaluate_axis_derivative(
            &plan.transfer[1], axis, 0.0f, 1U);
        float left_acceleration = evaluate_axis_derivative(
            &plan.transfer[0], axis, plan.transfer[0].duration_s, 2U);
        float right_acceleration = evaluate_axis_derivative(
            &plan.transfer[1], axis, 0.0f, 2U);

        ASSERT_NEAR(left_velocity, right_velocity, 1.0e-3f);
        ASSERT_NEAR(left_acceleration, right_acceleration, 1.0e-2f);

        if (axis < 3U) {
            linear_speed_squared += left_velocity * left_velocity;
        }
    }

    ASSERT_TRUE(sqrtf(linear_speed_squared) > 1.0e-3f);
}

static void assert_segment_limits(const TrajectorySegment *segment,
                                  const TrajectoryLimits *limits)
{
    uint32_t sample;

    if (segment->duration_s <= 0.0f) {
        return;
    }

    for (sample = 0U; sample <= 1000U; ++sample) {
        float time_s = segment->duration_s * (float)sample / 1000.0f;
        float vx = evaluate_axis_derivative(segment, 0U, time_s, 1U);
        float vy = evaluate_axis_derivative(segment, 1U, time_s, 1U);
        float vz = evaluate_axis_derivative(segment, 2U, time_s, 1U);
        float ax = evaluate_axis_derivative(segment, 0U, time_s, 2U);
        float ay = evaluate_axis_derivative(segment, 1U, time_s, 2U);
        float az = evaluate_axis_derivative(segment, 2U, time_s, 2U);
        float yaw_velocity = evaluate_axis_derivative(segment, 3U, time_s, 1U);
        float yaw_acceleration = evaluate_axis_derivative(segment, 3U, time_s, 2U);
        float linear_speed = sqrtf(vx * vx + vy * vy + vz * vz);
        float linear_acceleration = sqrtf(ax * ax + ay * ay + az * az);

        ASSERT_TRUE(linear_speed <=
                    limits->max_linear_velocity_mm_s + 1.0e-3f);
        ASSERT_TRUE(linear_acceleration <=
                    limits->max_linear_acceleration_mm_s2 + 1.0e-2f);
        ASSERT_TRUE(fabsf(yaw_velocity) <=
                    limits->max_yaw_velocity_deg_s + 1.0e-3f);
        ASSERT_TRUE(fabsf(yaw_acceleration) <=
                    limits->max_yaw_acceleration_deg_s2 + 1.0e-2f);
    }
}

static void power_to_bezier(const float coefficient[TRAJECTORY_COEFFICIENT_COUNT],
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

static void assert_segment_bezier_bounds(const TrajectorySegment *segment,
                                         const TrajectoryLimits *limits)
{
    float bezier[TRAJECTORY_AXIS_COUNT][TRAJECTORY_COEFFICIENT_COUNT];
    float first[TRAJECTORY_AXIS_COUNT][5];
    float second[TRAJECTORY_AXIS_COUNT][4];
    float max_linear_velocity = 0.0f;
    float max_linear_acceleration = 0.0f;
    float max_yaw_velocity = 0.0f;
    float max_yaw_acceleration = 0.0f;
    uint32_t axis;
    uint32_t index;

    if (segment->duration_s <= 0.0f) {
        return;
    }

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
        float linear = sqrtf(first[0][index] * first[0][index] +
                             first[1][index] * first[1][index] +
                             first[2][index] * first[2][index]) /
                       segment->duration_s;
        float yaw = fabsf(first[3][index]) / segment->duration_s;

        if (linear > max_linear_velocity) max_linear_velocity = linear;
        if (yaw > max_yaw_velocity) max_yaw_velocity = yaw;
    }

    for (index = 0U; index < 4U; ++index) {
        float duration_squared = segment->duration_s * segment->duration_s;
        float linear = sqrtf(second[0][index] * second[0][index] +
                             second[1][index] * second[1][index] +
                             second[2][index] * second[2][index]) /
                       duration_squared;
        float yaw = fabsf(second[3][index]) / duration_squared;

        if (linear > max_linear_acceleration) max_linear_acceleration = linear;
        if (yaw > max_yaw_acceleration) max_yaw_acceleration = yaw;
    }

    ASSERT_TRUE(max_linear_velocity <=
                limits->max_linear_velocity_mm_s + 1.0e-3f);
    ASSERT_TRUE(max_linear_acceleration <=
                limits->max_linear_acceleration_mm_s2 + 1.0e-2f);
    ASSERT_TRUE(max_yaw_velocity <=
                limits->max_yaw_velocity_deg_s + 1.0e-3f);
    ASSERT_TRUE(max_yaw_acceleration <=
                limits->max_yaw_acceleration_deg_s2 + 1.0e-2f);
}

static void test_all_motion_limits_are_respected(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;

    request.limits.max_linear_velocity_mm_s = 35.0f;
    request.limits.max_linear_acceleration_mm_s2 = 70.0f;
    request.limits.max_yaw_velocity_deg_s = 25.0f;
    request.limits.max_yaw_acceleration_deg_s2 = 50.0f;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    assert_segment_limits(&plan.approach, &request.limits);
    assert_segment_limits(&plan.transfer[0], &request.limits);
    assert_segment_limits(&plan.transfer[1], &request.limits);
    assert_segment_bezier_bounds(&plan.approach, &request.limits);
    assert_segment_bezier_bounds(&plan.transfer[0], &request.limits);
    assert_segment_bezier_bounds(&plan.transfer[1], &request.limits);
}

static void test_zero_distance_and_repeated_waypoints(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    TrajectoryReference reference;

    request.pick = request.current;
    request.transit = request.current;
    request.place = request.current;
    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    ASSERT_NEAR(0.0f, plan.approach.duration_s, 1.0e-7f);
    ASSERT_NEAR(0.0f, plan.transfer_duration_s, 1.0e-7f);
    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      0.0f, &reference));
    ASSERT_EQ_INT(1, reference.grip);
    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      0.0f, &reference));
    ASSERT_EQ_INT(0, reference.grip);
    assert_pose_near(request.current, reference.pose, 1.0e-5f);

    request = valid_request();
    request.transit = request.pick;
    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    ASSERT_NEAR(0.0f, plan.transfer[0].duration_s, 1.0e-7f);
    ASSERT_TRUE(plan.transfer[1].duration_s > 0.0f);
    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      0.0f, &reference));
    assert_pose_near(request.pick, reference.pose, 1.0e-4f);
    ASSERT_EQ_INT(1, reference.grip);
}

static void test_reversing_transit_uses_safe_stop(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    uint32_t axis;

    request.pick = (TrajectoryPose){0.0f, 0.0f, 0.0f, 0.0f};
    request.transit = (TrajectoryPose){20.0f, 30.0f, 40.0f, 45.0f};
    request.place = request.pick;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        ASSERT_NEAR(0.0f,
                    evaluate_axis_derivative(&plan.transfer[0], axis,
                                             plan.transfer[0].duration_s, 1U),
                    1.0e-3f);
        ASSERT_NEAR(0.0f,
                    evaluate_axis_derivative(&plan.transfer[1], axis,
                                             0.0f, 1U),
                    1.0e-3f);
    }
}

static void test_evaluate_rejects_invalid_inputs(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    TrajectoryReference reference;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    ASSERT_EQ_INT(TRAJECTORY_STATE_INVALID_ARGUMENT,
                  Trajectory_Evaluate(NULL, TRAJECTORY_PHASE_APPROACH,
                                      0.0f, &reference));
    ASSERT_EQ_INT(TRAJECTORY_STATE_INVALID_ARGUMENT,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      NAN, &reference));
    ASSERT_EQ_INT(TRAJECTORY_STATE_INVALID_ARGUMENT,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      0.0f, NULL));
    ASSERT_EQ_INT(TRAJECTORY_STATE_INVALID_PHASE,
                  Trajectory_Evaluate(&plan, (TrajectoryPhase)99,
                                      0.0f, &reference));
}

int main(void)
{
    test_approach_pose_and_grip_output();
    test_invalid_request_is_rejected();
    test_transfer_passes_transit_and_controls_grip();
    test_transfer_is_c2_and_does_not_stop_at_transit();
    test_all_motion_limits_are_respected();
    test_zero_distance_and_repeated_waypoints();
    test_reversing_transit_uses_safe_stop();
    test_evaluate_rejects_invalid_inputs();

    puts("trajectory tests passed");
    return EXIT_SUCCESS;
}

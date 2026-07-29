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

int main(void)
{
    test_approach_pose_and_grip_output();
    test_invalid_request_is_rejected();

    puts("trajectory tests passed");
    return EXIT_SUCCESS;
}

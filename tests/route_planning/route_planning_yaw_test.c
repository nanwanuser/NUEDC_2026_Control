#include "route_yaw_bias.h"

#include "crane_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static TrajectoryPose captured_poses[TRAJECTORY_MAX_WAYPOINTS];
static uint8_t captured_count;

CraneControlStatus CraneControl_ChooseYawBias(const TrajectoryPose *poses,
                                              uint8_t count,
                                              float *bias_deg)
{
    captured_count = count;
    (void)memcpy(captured_poses, poses, count * sizeof(*poses));
    *bias_deg = 25.0f;
    return CRANE_CONTROL_OK;
}

static int yaw_equals(float actual, float expected)
{
    return fabsf(actual - expected) <= 0.001f;
}

static int test_bias_uses_only_carried_piece_poses(void)
{
    TrajectoryRequest trajectory;
    uint8_t index;

    (void)memset(&trajectory, 0, sizeof(trajectory));
    trajectory.approach.point_count = 4U;
    trajectory.approach.points[0].yaw_deg = -180.0f;
    trajectory.approach.points[1].yaw_deg = -180.0f;
    trajectory.approach.points[2].yaw_deg = 0.0f;
    trajectory.approach.points[3].yaw_deg = 0.0f;

    trajectory.transfer.point_count = 4U;
    trajectory.transfer.points[0].yaw_deg = 0.0f;
    trajectory.transfer.points[1].yaw_deg = 0.0f;
    trajectory.transfer.points[2].yaw_deg = -131.536621f;
    trajectory.transfer.points[3].yaw_deg = -131.536621f;

    RoutePlanning_ApplyYawBias(&trajectory);

    if (captured_count != 4U) {
        fprintf(stderr, "bias chooser received %u poses, expected 4 transfer poses\n",
                (unsigned)captured_count);
        return 1;
    }
    for (index = 0U; index < captured_count; ++index) {
        if (!yaw_equals(captured_poses[index].yaw_deg,
                        index < 2U ? 0.0f : -131.536621f)) {
            fputs("bias chooser did not receive the unmodified transfer path\n",
                  stderr);
            return 1;
        }
    }
    if (!yaw_equals(trajectory.approach.points[0].yaw_deg, -180.0f) ||
        !yaw_equals(trajectory.approach.points[1].yaw_deg, -180.0f)) {
        fputs("empty-tool approach poses were biased\n", stderr);
        return 1;
    }
    if (!yaw_equals(trajectory.approach.points[2].yaw_deg, 25.0f) ||
        !yaw_equals(trajectory.approach.points[3].yaw_deg, 25.0f)) {
        fputs("pick approach poses did not receive the transfer bias\n", stderr);
        return 1;
    }
    for (index = 0U; index < trajectory.transfer.point_count; ++index) {
        const float expected = index < 2U ? 25.0f : -106.536621f;

        if (!yaw_equals(trajectory.transfer.points[index].yaw_deg, expected)) {
            fputs("transfer pose did not receive the shared bias\n", stderr);
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    if (test_bias_uses_only_carried_piece_poses() != 0) {
        return 1;
    }
    puts("route planning yaw tests passed");
    return 0;
}

#include "route_yaw_bias.h"

#include "crane_control.h"

#include <stddef.h>

void RoutePlanning_ApplyYawBias(TrajectoryRequest *trajectory)
{
    uint8_t approach_bias_start;
    uint8_t index;
    float bias_deg = 0.0f;

    if (trajectory == NULL || trajectory->transfer.point_count == 0U) {
        return;
    }

    /* Only the transfer path carries the piece. Its shared bias preserves the
       required pick-to-place rotation without making the empty-tool start pose
       consume part of the wrist's 180 degree travel. */
    (void)CraneControl_ChooseYawBias(trajectory->transfer.points,
                                    trajectory->transfer.point_count,
                                    &bias_deg);

    /* Decision_BuildTrajectoryRequest() ends the approach at pick-above. Keep
       the actual start and optional vertical clearance pose as-is, then join the
       biased pick-above pose continuously to the transfer path. */
    approach_bias_start = trajectory->approach.point_count > 1U
                              ? (uint8_t)(trajectory->approach.point_count - 1U)
                              : 0U;
    for (index = approach_bias_start;
         index < trajectory->approach.point_count;
         ++index) {
        trajectory->approach.points[index].yaw_deg += bias_deg;
    }
    for (index = 0U; index < trajectory->transfer.point_count; ++index) {
        trajectory->transfer.points[index].yaw_deg += bias_deg;
    }
}

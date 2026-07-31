#ifndef ROUTE_YAW_BIAS_H
#define ROUTE_YAW_BIAS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "trajectory.h"

void RoutePlanning_ApplyYawBias(TrajectoryRequest *trajectory);

#ifdef __cplusplus
}
#endif

#endif

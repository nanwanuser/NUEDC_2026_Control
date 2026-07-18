#ifndef APP_IMU_H
#define APP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "icm42688p.h"
#include "imu_fusion.h"

void AppIMU_Init(void);
void AppIMU_Process(void);
uint8_t AppIMU_IsReady(void);
icm42688_status_t AppIMU_GetInitStatus(void);
const imu_euler_t *AppIMU_GetEulerDeg(void);
const imu_quat_t *AppIMU_GetQuaternion(void);

#ifdef __cplusplus
}
#endif

#endif

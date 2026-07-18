#ifndef APP_IMU_H
#define APP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "icm42688p.h"
#include "imu_fusion.h"

typedef struct
{
    imu_vec3_t accel_g;
    imu_vec3_t gyro_dps;
    imu_euler_t euler_deg;
    float temperature_c;
} app_imu_telemetry_t;

void AppIMU_Init(void);
void AppIMU_Process(void);
uint8_t AppIMU_IsReady(void);
icm42688_status_t AppIMU_GetInitStatus(void);
const imu_euler_t *AppIMU_GetEulerDeg(void);
const imu_quat_t *AppIMU_GetQuaternion(void);
uint8_t AppIMU_GetTelemetry(app_imu_telemetry_t *telemetry);

#ifdef __cplusplus
}
#endif

#endif

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "icm42688p.h"

typedef struct
{
    float x;
    float y;
    float z;
} imu_vec3_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} imu_quat_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} imu_euler_t;

typedef struct
{
    uint16_t gyro_calibration_samples;
    float gyro_static_threshold_dps;
    float accel_static_threshold_g;
    float accel_lpf_alpha;
    float bias_alpha;
    float mahony_kp;
    float mahony_ki;
} imu_fusion_config_t;

typedef struct
{
    uint8_t initialized;
    uint8_t gyro_bias_ready;
    uint16_t calibration_count;

    imu_vec3_t gyro_bias_dps;
    imu_vec3_t gyro_integral;
    imu_vec3_t accel_filtered_g;
    imu_vec3_t last_accel_g;

    imu_quat_t quaternion;
    imu_euler_t euler_deg;

    float last_temperature_c;
    float last_dt_s;
} imu_fusion_state_t;

void IMU_FusionGetDefaultConfig(imu_fusion_config_t *config);
void IMU_FusionInit(imu_fusion_state_t *state, const imu_fusion_config_t *config);
uint8_t IMU_FusionGyroBiasReady(const imu_fusion_state_t *state);
void IMU_FusionResetYaw(imu_fusion_state_t *state);
icm42688_status_t IMU_FusionUpdate(imu_fusion_state_t *state,
                                   const imu_fusion_config_t *config,
                                   float dt_s);

const imu_quat_t *IMU_FusionGetQuaternion(const imu_fusion_state_t *state);
const imu_euler_t *IMU_FusionGetEulerDeg(const imu_fusion_state_t *state);
imu_vec3_t IMU_FusionGetGyroBiasDps(const imu_fusion_state_t *state);

#ifdef __cplusplus
}
#endif

#endif

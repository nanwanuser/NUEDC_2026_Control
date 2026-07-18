#include "imu_fusion.h"

#include <math.h>
#include <string.h>

#define IMU_UPDATE_MIN_DT_S (0.0005f)

static imu_vec3_t IMU_Vec3Lpf(imu_vec3_t previous, imu_vec3_t current, float alpha)
{
    imu_vec3_t result;

    result.x = previous.x + (alpha * (current.x - previous.x));
    result.y = previous.y + (alpha * (current.y - previous.y));
    result.z = previous.z + (alpha * (current.z - previous.z));
    return result;
}

static void IMU_UpdateOutputs(imu_fusion_state_t *state)
{
    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&state->ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);

    state->quaternion.w = quaternion.element.w;
    state->quaternion.x = quaternion.element.x;
    state->quaternion.y = quaternion.element.y;
    state->quaternion.z = quaternion.element.z;

    state->euler_deg.roll_deg = euler.angle.roll;
    state->euler_deg.pitch_deg = euler.angle.pitch;
    state->euler_deg.yaw_deg = euler.angle.yaw;
}

void IMU_FusionGetDefaultConfig(imu_fusion_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->sample_rate_hz = 1000.0f;
    config->gyro_static_threshold_dps = 3.0f;
    config->bias_stationary_period_s = 3.0f;
    config->accel_lpf_alpha = 0.45f;
    config->fusion_gain = 0.5f;
    config->gyroscope_range_dps = 500.0f;
    config->acceleration_rejection_deg = 10.0f;
    config->recovery_trigger_period_s = 5.0f;
}

void IMU_FusionInit(imu_fusion_state_t *state, const imu_fusion_config_t *config)
{
    FusionAhrsSettings ahrs_settings;
    FusionBiasSettings bias_settings;

    if ((state == NULL) || (config == NULL) ||
        (!isfinite(config->sample_rate_hz)) ||
        (!isfinite(config->gyro_static_threshold_dps)) ||
        (!isfinite(config->bias_stationary_period_s)) ||
        (!isfinite(config->accel_lpf_alpha)) ||
        (!isfinite(config->fusion_gain)) ||
        (!isfinite(config->gyroscope_range_dps)) ||
        (!isfinite(config->acceleration_rejection_deg)) ||
        (!isfinite(config->recovery_trigger_period_s)) ||
        (config->sample_rate_hz <= 0.0f) ||
        (config->gyro_static_threshold_dps < 0.0f) ||
        (config->bias_stationary_period_s < 0.0f) ||
        (config->accel_lpf_alpha < 0.0f) ||
        (config->accel_lpf_alpha > 1.0f) ||
        (config->fusion_gain < 0.0f) ||
        (config->gyroscope_range_dps < 0.0f) ||
        (config->acceleration_rejection_deg < 0.0f) ||
        (config->recovery_trigger_period_s < 0.0f))
    {
        return;
    }

    memset(state, 0, sizeof(*state));

    FusionAhrsInitialise(&state->ahrs);
    ahrs_settings = fusionAhrsDefaultSettings;
    ahrs_settings.sampleRate = config->sample_rate_hz;
    ahrs_settings.convention = FusionConventionNwu;
    ahrs_settings.gain = config->fusion_gain;
    ahrs_settings.gyroscopeRange = config->gyroscope_range_dps;
    ahrs_settings.accelerationRejection = config->acceleration_rejection_deg;
    ahrs_settings.magneticRejection = 0.0f;
    ahrs_settings.recoveryTriggerPeriod = config->recovery_trigger_period_s;
    FusionAhrsSetSettings(&state->ahrs, &ahrs_settings);

    FusionBiasInitialise(&state->bias);
    bias_settings = fusionBiasDefaultSettings;
    bias_settings.sampleRate = config->sample_rate_hz;
    bias_settings.stationaryThreshold = config->gyro_static_threshold_dps;
    bias_settings.stationaryPeriod = config->bias_stationary_period_s;
    FusionBiasSetSettings(&state->bias, &bias_settings);

    state->quaternion.w = 1.0f;
    state->accel_filtered_g.z = 1.0f;
    state->initialized = 1U;
}

uint8_t IMU_FusionGyroBiasReady(const imu_fusion_state_t *state)
{
    if (state == NULL)
    {
        return 0U;
    }

    return state->gyro_bias_ready;
}

void IMU_FusionResetYaw(imu_fusion_state_t *state)
{
    if ((state == NULL) || (state->initialized == 0U))
    {
        return;
    }

    FusionAhrsSetHeading(&state->ahrs, 0.0f);
    IMU_UpdateOutputs(state);
}

icm42688_status_t IMU_FusionUpdate(imu_fusion_state_t *state,
                                   const imu_fusion_config_t *config,
                                   float dt_s)
{
    icm42688_scaled_t sensor;
    icm42688_status_t status;
    imu_vec3_t accel_g;
    FusionVector gyroscope;
    FusionVector accelerometer;
    FusionVector gyro_offset;

    if ((state == NULL) || (config == NULL) || (state->initialized == 0U))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    if ((!isfinite(dt_s)) || (dt_s < IMU_UPDATE_MIN_DT_S))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    status = ICM42688_ReadScaled(&sensor);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    accel_g.x = sensor.accel_x_g;
    accel_g.y = sensor.accel_y_g;
    accel_g.z = sensor.accel_z_g;

    if (state->sample_count == 0U)
    {
        state->accel_filtered_g = accel_g;
    }
    else
    {
        state->accel_filtered_g = IMU_Vec3Lpf(state->accel_filtered_g,
                                              accel_g,
                                              config->accel_lpf_alpha);
    }

    gyroscope = (FusionVector){.axis = {
        .x = sensor.gyro_x_dps,
        .y = sensor.gyro_y_dps,
        .z = sensor.gyro_z_dps,
    }};
    accelerometer = (FusionVector){.axis = {
        .x = state->accel_filtered_g.x,
        .y = state->accel_filtered_g.y,
        .z = state->accel_filtered_g.z,
    }};

    gyroscope = FusionBiasUpdate(&state->bias, gyroscope);
    FusionAhrsSetSamplePeriod(&state->ahrs, dt_s);
    FusionAhrsUpdateNoMagnetometer(&state->ahrs, gyroscope, accelerometer);

    gyro_offset = FusionBiasGetOffset(&state->bias);
    state->gyro_bias_dps.x = gyro_offset.axis.x;
    state->gyro_bias_dps.y = gyro_offset.axis.y;
    state->gyro_bias_dps.z = gyro_offset.axis.z;
    state->gyro_bias_ready = (state->bias.timer >= state->bias.timeout) ? 1U : 0U;
    if (state->sample_count < UINT32_MAX)
    {
        state->sample_count++;
    }
    state->last_accel_g = accel_g;
    state->last_gyro_dps.x = sensor.gyro_x_dps;
    state->last_gyro_dps.y = sensor.gyro_y_dps;
    state->last_gyro_dps.z = sensor.gyro_z_dps;
    state->last_temperature_c = sensor.temperature_c;
    state->last_dt_s = dt_s;
    IMU_UpdateOutputs(state);

    return ICM42688_STATUS_OK;
}

const imu_quat_t *IMU_FusionGetQuaternion(const imu_fusion_state_t *state)
{
    if (state == NULL)
    {
        return NULL;
    }

    return &state->quaternion;
}

const imu_euler_t *IMU_FusionGetEulerDeg(const imu_fusion_state_t *state)
{
    if (state == NULL)
    {
        return NULL;
    }

    return &state->euler_deg;
}

imu_vec3_t IMU_FusionGetGyroBiasDps(const imu_fusion_state_t *state)
{
    const imu_vec3_t zero = {0.0f, 0.0f, 0.0f};

    if (state == NULL)
    {
        return zero;
    }

    return state->gyro_bias_dps;
}

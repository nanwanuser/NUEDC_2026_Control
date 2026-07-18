#include "imu_fusion.h"

#include <math.h>
#include <string.h>

#define IMU_RAD_PER_DEG (0.01745329251994329577f)
#define IMU_DEG_PER_RAD (57.2957795130823208768f)
#define IMU_GRAVITY_NORM_MIN (0.80f)
#define IMU_GRAVITY_NORM_MAX (1.20f)
#define IMU_UPDATE_MIN_DT_S (0.0005f)

static float IMU_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float IMU_InvSqrt(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(x);
}

static float IMU_Vec3Norm(const imu_vec3_t *vec)
{
    return sqrtf((vec->x * vec->x) + (vec->y * vec->y) + (vec->z * vec->z));
}

static imu_vec3_t IMU_Vec3Sub(imu_vec3_t a, imu_vec3_t b)
{
    imu_vec3_t result;

    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}

static imu_vec3_t IMU_Vec3Scale(imu_vec3_t vec, float scale)
{
    imu_vec3_t result;

    result.x = vec.x * scale;
    result.y = vec.y * scale;
    result.z = vec.z * scale;

    return result;
}

static imu_vec3_t IMU_Vec3Cross(imu_vec3_t a, imu_vec3_t b)
{
    imu_vec3_t result;

    result.x = (a.y * b.z) - (a.z * b.y);
    result.y = (a.z * b.x) - (a.x * b.z);
    result.z = (a.x * b.y) - (a.y * b.x);

    return result;
}

static imu_vec3_t IMU_Vec3Lpf(imu_vec3_t previous, imu_vec3_t current, float alpha)
{
    imu_vec3_t result;

    /* First-order low-pass filter for accel data before fusion. */
    result.x = previous.x + (alpha * (current.x - previous.x));
    result.y = previous.y + (alpha * (current.y - previous.y));
    result.z = previous.z + (alpha * (current.z - previous.z));

    return result;
}

static void IMU_QuatNormalize(imu_quat_t *quat)
{
    float inv_norm;

    inv_norm = IMU_InvSqrt((quat->w * quat->w) +
                           (quat->x * quat->x) +
                           (quat->y * quat->y) +
                           (quat->z * quat->z));
    if (inv_norm == 0.0f)
    {
        quat->w = 1.0f;
        quat->x = 0.0f;
        quat->y = 0.0f;
        quat->z = 0.0f;
        return;
    }

    quat->w *= inv_norm;
    quat->x *= inv_norm;
    quat->y *= inv_norm;
    quat->z *= inv_norm;
}

static void IMU_UpdateEulerFromQuaternion(imu_fusion_state_t *state)
{
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    /* Internal state stays in quaternion form; Euler is output only. */
    sinr_cosp = 2.0f * ((state->quaternion.w * state->quaternion.x) +
                        (state->quaternion.y * state->quaternion.z));
    cosr_cosp = 1.0f - 2.0f * ((state->quaternion.x * state->quaternion.x) +
                               (state->quaternion.y * state->quaternion.y));

    sinp = 2.0f * ((state->quaternion.w * state->quaternion.y) -
                   (state->quaternion.z * state->quaternion.x));
    sinp = IMU_Clamp(sinp, -1.0f, 1.0f);

    siny_cosp = 2.0f * ((state->quaternion.w * state->quaternion.z) +
                        (state->quaternion.x * state->quaternion.y));
    cosy_cosp = 1.0f - 2.0f * ((state->quaternion.y * state->quaternion.y) +
                               (state->quaternion.z * state->quaternion.z));

    state->euler_deg.roll_deg = atan2f(sinr_cosp, cosr_cosp) * IMU_DEG_PER_RAD;
    state->euler_deg.pitch_deg = asinf(sinp) * IMU_DEG_PER_RAD;
    state->euler_deg.yaw_deg = atan2f(siny_cosp, cosy_cosp) * IMU_DEG_PER_RAD;
}

static void IMU_SetQuaternionFromEuler(imu_fusion_state_t *state,
                                       float roll_deg,
                                       float pitch_deg,
                                       float yaw_deg)
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;

    roll_rad = roll_deg * IMU_RAD_PER_DEG * 0.5f;
    pitch_rad = pitch_deg * IMU_RAD_PER_DEG * 0.5f;
    yaw_rad = yaw_deg * IMU_RAD_PER_DEG * 0.5f;

    cr = cosf(roll_rad);
    sr = sinf(roll_rad);
    cp = cosf(pitch_rad);
    sp = sinf(pitch_rad);
    cy = cosf(yaw_rad);
    sy = sinf(yaw_rad);

    state->quaternion.w = (cr * cp * cy) + (sr * sp * sy);
    state->quaternion.x = (sr * cp * cy) - (cr * sp * sy);
    state->quaternion.y = (cr * sp * cy) + (sr * cp * sy);
    state->quaternion.z = (cr * cp * sy) - (sr * sp * cy);

    IMU_QuatNormalize(&state->quaternion);
    IMU_UpdateEulerFromQuaternion(state);
}

static void IMU_UpdateGyroBias(imu_fusion_state_t *state,
                               const imu_fusion_config_t *config,
                               imu_vec3_t gyro_dps,
                               imu_vec3_t accel_g)
{
    float gyro_norm;
    float accel_norm;
    float accel_error;

    gyro_norm = IMU_Vec3Norm(&gyro_dps);
    accel_norm = IMU_Vec3Norm(&accel_g);
    accel_error = fabsf(accel_norm - 1.0f);

    /* Update bias only while the board is judged to be nearly static. */
    if ((gyro_norm < config->gyro_static_threshold_dps) &&
        (accel_error < config->accel_static_threshold_g))
    {
        if (state->gyro_bias_ready == 0U)
        {
            /* Recursive mean for initial gyro bias estimation. */
            state->calibration_count++;
            state->gyro_bias_dps.x += (gyro_dps.x - state->gyro_bias_dps.x) / (float)state->calibration_count;
            state->gyro_bias_dps.y += (gyro_dps.y - state->gyro_bias_dps.y) / (float)state->calibration_count;
            state->gyro_bias_dps.z += (gyro_dps.z - state->gyro_bias_dps.z) / (float)state->calibration_count;

            if (state->calibration_count >= config->gyro_calibration_samples)
            {
                state->gyro_bias_ready = 1U;
                state->gyro_integral.x = 0.0f;
                state->gyro_integral.y = 0.0f;
                state->gyro_integral.z = 0.0f;
            }
        }
        else
        {
            /* Slow online tracking for thermal drift after startup. */
            state->gyro_bias_dps = IMU_Vec3Lpf(state->gyro_bias_dps, gyro_dps, config->bias_alpha);
        }
    }
}

static void IMU_MahonyUpdate(imu_fusion_state_t *state,
                             const imu_fusion_config_t *config,
                             imu_vec3_t gyro_rad,
                             imu_vec3_t accel_g,
                             float dt_s)
{
    float accel_norm;
    imu_vec3_t accel_unit;
    imu_vec3_t gravity_est;
    imu_vec3_t error;
    float half_dt;
    float q0;
    float q1;
    float q2;
    float q3;

    accel_norm = IMU_Vec3Norm(&accel_g);
    /* Ignore accel correction when its magnitude is far away from 1 g. */
    if ((accel_norm > IMU_GRAVITY_NORM_MIN) && (accel_norm < IMU_GRAVITY_NORM_MAX))
    {
        accel_unit = IMU_Vec3Scale(accel_g, 1.0f / accel_norm);

        gravity_est.x = 2.0f * ((state->quaternion.x * state->quaternion.z) -
                                (state->quaternion.w * state->quaternion.y));
        gravity_est.y = 2.0f * ((state->quaternion.w * state->quaternion.x) +
                                (state->quaternion.y * state->quaternion.z));
        gravity_est.z = 1.0f - 2.0f * ((state->quaternion.x * state->quaternion.x) +
                                       (state->quaternion.y * state->quaternion.y));

        error = IMU_Vec3Cross(gravity_est, accel_unit);

        state->gyro_integral.x += config->mahony_ki * error.x * dt_s;
        state->gyro_integral.y += config->mahony_ki * error.y * dt_s;
        state->gyro_integral.z += config->mahony_ki * error.z * dt_s;

        gyro_rad.x += (config->mahony_kp * error.x) + state->gyro_integral.x;
        gyro_rad.y += (config->mahony_kp * error.y) + state->gyro_integral.y;
        gyro_rad.z += (config->mahony_kp * error.z) + state->gyro_integral.z;
    }

    half_dt = 0.5f * dt_s;
    q0 = state->quaternion.w;
    q1 = state->quaternion.x;
    q2 = state->quaternion.y;
    q3 = state->quaternion.z;

    state->quaternion.w += (-q1 * gyro_rad.x - q2 * gyro_rad.y - q3 * gyro_rad.z) * half_dt;
    state->quaternion.x += ( q0 * gyro_rad.x + q2 * gyro_rad.z - q3 * gyro_rad.y) * half_dt;
    state->quaternion.y += ( q0 * gyro_rad.y - q1 * gyro_rad.z + q3 * gyro_rad.x) * half_dt;
    state->quaternion.z += ( q0 * gyro_rad.z + q1 * gyro_rad.y - q2 * gyro_rad.x) * half_dt;

    IMU_QuatNormalize(&state->quaternion);
    IMU_UpdateEulerFromQuaternion(state);
}

void IMU_FusionGetDefaultConfig(imu_fusion_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->gyro_calibration_samples = 500U;
    config->gyro_static_threshold_dps = 1.5f;
    config->accel_static_threshold_g = 0.08f;
    config->accel_lpf_alpha = 0.45f;
    config->bias_alpha = 0.001f;
    config->mahony_kp = 5.5f;
    config->mahony_ki = 0.02f;
}

void IMU_FusionInit(imu_fusion_state_t *state, const imu_fusion_config_t *config)
{
    if ((state == NULL) || (config == NULL))
    {
        return;
    }

    memset(state, 0, sizeof(*state));
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
    if (state == NULL)
    {
        return;
    }

    state->gyro_integral.z = 0.0f;
    IMU_SetQuaternionFromEuler(state,
                               state->euler_deg.roll_deg,
                               state->euler_deg.pitch_deg,
                               0.0f);
}

icm42688_status_t IMU_FusionUpdate(imu_fusion_state_t *state,
                                   const imu_fusion_config_t *config,
                                   float dt_s)
{
    icm42688_scaled_t sensor;
    icm42688_status_t status;
    imu_vec3_t accel_g;
    imu_vec3_t gyro_dps;
    imu_vec3_t gyro_rad;

    if ((state == NULL) || (config == NULL) || (state->initialized == 0U))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    if (dt_s < IMU_UPDATE_MIN_DT_S)
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

    gyro_dps.x = sensor.gyro_x_dps;
    gyro_dps.y = sensor.gyro_y_dps;
    gyro_dps.z = sensor.gyro_z_dps;

    if (state->calibration_count == 0U)
    {
        state->accel_filtered_g = accel_g;
    }
    else
    {
        state->accel_filtered_g = IMU_Vec3Lpf(state->accel_filtered_g,
                                              accel_g,
                                              config->accel_lpf_alpha);
    }

    state->last_accel_g = accel_g;
    state->last_temperature_c = sensor.temperature_c;
    state->last_dt_s = dt_s;

    IMU_UpdateGyroBias(state, config, gyro_dps, state->accel_filtered_g);

    gyro_dps = IMU_Vec3Sub(gyro_dps, state->gyro_bias_dps);
    gyro_rad = IMU_Vec3Scale(gyro_dps, IMU_RAD_PER_DEG);

    IMU_MahonyUpdate(state, config, gyro_rad, state->accel_filtered_g, dt_s);

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
    imu_vec3_t bias = {0.0f, 0.0f, 0.0f};

    if (state == NULL)
    {
        return bias;
    }

    return state->gyro_bias_dps;
}

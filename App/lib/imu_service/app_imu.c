#include "app_imu.h"

/*
 * IMU service layer:
 * 1. EXTI interrupt only sets the data-ready flag and never reads SPI in IRQ context.
 * 2. AppIMU_Process() runs in the main loop and owns sensor reading plus quaternion update.
 * 3. If INT1 is not wired correctly yet, a 1 ms polling fallback keeps the demo alive.
 */
#define APP_IMU_SAMPLE_DT_S (0.001f)
#define APP_IMU_POLL_PERIOD_MS (1U)

static imu_fusion_state_t s_imu_state;
static imu_fusion_config_t s_fusion_config;
static icm42688_config_t s_icm_config;
static icm42688_status_t s_init_status = ICM42688_STATUS_ERROR;
static uint8_t s_ready = 0U;
static uint32_t s_last_poll_ms = 0U;

void AppIMU_Init(void)
{
    IMU_FusionGetDefaultConfig(&s_fusion_config);
    ICM42688_GetDefaultConfig(&s_icm_config);

    /* 1 kHz ODR + 500 dps/4 g is a practical baseline for an attitude demo. */
    s_icm_config.gyro_fs = ICM42688_GYRO_FS_500DPS;
    s_icm_config.accel_fs = ICM42688_ACCEL_FS_4G;
    s_icm_config.gyro_odr = ICM42688_ODR_1KHZ;
    s_icm_config.accel_odr = ICM42688_ODR_1KHZ;

    s_init_status = ICM42688_Init(NULL, &s_icm_config);
    if (s_init_status != ICM42688_STATUS_OK)
    {
        s_ready = 0U;
        return;
    }

    IMU_FusionInit(&s_imu_state, &s_fusion_config);
    s_last_poll_ms = HAL_GetTick();
    s_ready = 1U;
}

void AppIMU_Process(void)
{
    uint32_t now_ms;
    uint8_t data_ready;

    if (s_ready == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    data_ready = ICM42688_GetDataReadyFlag();
    if ((data_ready == 0U) && ((now_ms - s_last_poll_ms) < APP_IMU_POLL_PERIOD_MS))
    {
        return;
    }

    s_last_poll_ms = now_ms;
    if (data_ready != 0U)
    {
        ICM42688_ClearDataReadyFlag();
    }

    (void)IMU_FusionUpdate(&s_imu_state, &s_fusion_config, APP_IMU_SAMPLE_DT_S);
}

uint8_t AppIMU_IsReady(void)
{
    return s_ready;
}

icm42688_status_t AppIMU_GetInitStatus(void)
{
    return s_init_status;
}

const imu_euler_t *AppIMU_GetEulerDeg(void)
{
    if (s_ready == 0U)
    {
        return NULL;
    }

    return IMU_FusionGetEulerDeg(&s_imu_state);
}

const imu_quat_t *AppIMU_GetQuaternion(void)
{
    if (s_ready == 0U)
    {
        return NULL;
    }

    return IMU_FusionGetQuaternion(&s_imu_state);
}

#include "IMU_Task.h"

#include "Gimbal_ctrl_Motion.h"
#include "app_imu.h"
#include "cmsis_os.h"

#include <stddef.h>

#define IMU_TASK_SAMPLE_PERIOD_MS (10U)
#define IMU_TASK_INIT_RETRY_MS (1000U)
#define IMU_TASK_YAW_WINDOW_MS (2000U)
#define IMU_TASK_YAW_THRESHOLD_DEG (60.0f)
#define IMU_TASK_YAW_HISTORY_COUNT \
    ((IMU_TASK_YAW_WINDOW_MS / IMU_TASK_SAMPLE_PERIOD_MS) + 1U)

typedef struct {
    float previous_yaw_deg;
    float unwrapped_yaw_deg;
    float history_deg[IMU_TASK_YAW_HISTORY_COUNT];
    uint16_t history_count;
    uint16_t next_index;
    uint8_t initialized;
} imu_yaw_monitor_t;

static imu_yaw_monitor_t s_yaw_monitor;

static float imu_task_abs_f32(float value)
{
    return value < 0.0f ? -value : value;
}

static float imu_task_unwrap_delta(float delta_deg)
{
    if (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    } else if (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

static void imu_task_reset_yaw_monitor(float yaw_deg)
{
    s_yaw_monitor.previous_yaw_deg = yaw_deg;
    s_yaw_monitor.unwrapped_yaw_deg = 0.0f;
    s_yaw_monitor.history_deg[0] = 0.0f;
    s_yaw_monitor.history_count = 1U;
    s_yaw_monitor.next_index = 1U;
    s_yaw_monitor.initialized = 1U;
}

static float imu_task_get_window_delta(void)
{
    uint16_t oldest_index = s_yaw_monitor.history_count <
                                    IMU_TASK_YAW_HISTORY_COUNT
                                ? 0U : s_yaw_monitor.next_index;
    return s_yaw_monitor.unwrapped_yaw_deg -
           s_yaw_monitor.history_deg[oldest_index];
}

static int32_t imu_task_round_tenths(float angle_deg)
{
    float scaled = angle_deg * 10.0f;
    scaled += scaled >= 0.0f ? 0.5f : -0.5f;
    return (int32_t)scaled;
}

static uint8_t imu_task_get_yaw_compensation(float yaw_deg,
                                              int32_t *compensation_tenths)
{
    float sample_delta;
    float window_delta;

    if (s_yaw_monitor.initialized == 0U) {
        imu_task_reset_yaw_monitor(yaw_deg);
        return 0U;
    }
    sample_delta = imu_task_unwrap_delta(
        yaw_deg - s_yaw_monitor.previous_yaw_deg);
    s_yaw_monitor.previous_yaw_deg = yaw_deg;
    s_yaw_monitor.unwrapped_yaw_deg += sample_delta;
    s_yaw_monitor.history_deg[s_yaw_monitor.next_index] =
        s_yaw_monitor.unwrapped_yaw_deg;
    s_yaw_monitor.next_index = (uint16_t)(
        (s_yaw_monitor.next_index + 1U) % IMU_TASK_YAW_HISTORY_COUNT);
    if (s_yaw_monitor.history_count < IMU_TASK_YAW_HISTORY_COUNT) {
        s_yaw_monitor.history_count++;
    }
    window_delta = imu_task_get_window_delta();
    if (imu_task_abs_f32(window_delta) < IMU_TASK_YAW_THRESHOLD_DEG) {
        return 0U;
    }
    *compensation_tenths = imu_task_round_tenths(-window_delta);
    imu_task_reset_yaw_monitor(yaw_deg);
    return 1U;
}

void IMU_Task_App(void *argument)
{
    const imu_euler_t *euler;
    int32_t compensation_tenths;
    uint32_t next_wake_tick;

    (void)argument;
    while (AppIMU_IsReady() == 0U) {
        AppIMU_Init();
        if (AppIMU_IsReady() == 0U) {
            osDelay(IMU_TASK_INIT_RETRY_MS);
        }
    }
    next_wake_tick = osKernelGetTickCount();
    for (;;) {
        AppIMU_Process();
        euler = AppIMU_GetEulerDeg();
        if (AppIMU_IsCalibrated() != 0U && euler != NULL) {
            if (imu_task_get_yaw_compensation(
                    euler->yaw_deg, &compensation_tenths) != 0U) {
                gimbal_motion_request_yaw_compensation(compensation_tenths);
            }
        } else {
            s_yaw_monitor.initialized = 0U;
        }
        next_wake_tick += IMU_TASK_SAMPLE_PERIOD_MS;
        osDelayUntil(next_wake_tick);
    }
}

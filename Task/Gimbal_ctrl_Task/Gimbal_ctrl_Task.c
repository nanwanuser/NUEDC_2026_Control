#include "Gimbal_ctrl_Task.h"

#include "Gimbal_ctrl_Motion.h"
#include "Gimbal_ctrl_Vision.h"
#include "cmsis_os.h"

#include <stddef.h>

typedef struct {
    int16_t x;
    int16_t y;
    uint32_t sequence;
    bool received;
    bool lost;
} gimbal_vision_snapshot_t;

typedef struct {
    volatile int16_t x;
    volatile int16_t y;
    volatile uint32_t sequence;
    volatile bool received;
    volatile bool lost;
} gimbal_vision_state_t;

typedef struct {
    int16_t yaw_delta;
    int16_t pitch_delta;
    uint16_t yaw_speed;
    uint16_t pitch_speed;
} gimbal_vector_motion_t;

static gimbal_vision_state_t s_vision;
static uint32_t s_corrected_sequence;
static bool s_initialized;

static int32_t gimbal_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int16_t gimbal_clamp_angle(int32_t angle_tenths)
{
    if (angle_tenths < GIMBAL_ANGLE_MIN_TENTHS) {
        return GIMBAL_ANGLE_MIN_TENTHS;
    }
    if (angle_tenths > GIMBAL_ANGLE_MAX_TENTHS) {
        return GIMBAL_ANGLE_MAX_TENTHS;
    }
    return (int16_t)angle_tenths;
}

static void gimbal_reset_vision(void)
{
    s_vision.sequence++;
    s_vision.x = 0;
    s_vision.y = 0;
    s_vision.received = false;
    s_vision.lost = true;
    s_vision.sequence++;
}

static void gimbal_read_vision(gimbal_vision_snapshot_t *snapshot)
{
    uint32_t sequence_before;
    do {
        sequence_before = s_vision.sequence;
        snapshot->x = s_vision.x;
        snapshot->y = s_vision.y;
        snapshot->received = s_vision.received;
        snapshot->lost = s_vision.lost;
        snapshot->sequence = s_vision.sequence;
    } while ((sequence_before & 1U) != 0U ||
             sequence_before != snapshot->sequence);
}

static bool gimbal_target_visible(void)
{
    gimbal_vision_snapshot_t snapshot;
    (void)gimbal_vision_receive(GIMBAL_VISION_POLL_TIMEOUT_MS);
    gimbal_read_vision(&snapshot);
    return snapshot.received && !snapshot.lost &&
           snapshot.sequence != s_corrected_sequence;
}

static uint32_t gimbal_vector_error(const gimbal_vision_snapshot_t *vector)
{
    return (uint32_t)gimbal_abs_i32(vector->x) +
           (uint32_t)gimbal_abs_i32(vector->y);
}

static int16_t gimbal_scale_vector_component(int16_t component,
                                             int32_t vector_max)
{
    int32_t numerator = (int32_t)component * GIMBAL_VISION_STEP_MAX_TENTHS;
    if (numerator > 0) {
        numerator += vector_max / 2;
    } else if (numerator < 0) {
        numerator -= vector_max / 2;
    }
    return (int16_t)(numerator / vector_max);
}

static uint16_t gimbal_scale_vector_speed(int16_t step, int32_t step_max)
{
    uint32_t speed;
    if (step == 0) {
        return 1U;
    }
    speed = (uint32_t)GIMBAL_MOTION_SPEED_RPM *
            (uint32_t)gimbal_abs_i32(step);
    speed = (speed + (uint32_t)step_max - 1U) / (uint32_t)step_max;
    return (uint16_t)(speed == 0U ? 1U : speed);
}

static bool gimbal_prepare_vector_motion(
    const gimbal_vision_snapshot_t *vector, gimbal_vector_motion_t *motion)
{
    int32_t vector_max = gimbal_abs_i32(vector->x);
    int32_t step_max;
    if (gimbal_abs_i32(vector->y) > vector_max) {
        vector_max = gimbal_abs_i32(vector->y);
    }
    if (vector_max == 0) {
        return false;
    }
    motion->yaw_delta = gimbal_scale_vector_component(vector->x, vector_max);
    motion->pitch_delta = gimbal_scale_vector_component(vector->y, vector_max);
    step_max = gimbal_abs_i32(motion->yaw_delta);
    if (gimbal_abs_i32(motion->pitch_delta) > step_max) {
        step_max = gimbal_abs_i32(motion->pitch_delta);
    }
    motion->yaw_speed = gimbal_scale_vector_speed(
        motion->yaw_delta, step_max);
    motion->pitch_speed = gimbal_scale_vector_speed(
        motion->pitch_delta, step_max);
    return true;
}

static HAL_StatusTypeDef gimbal_start_axis_step(
    gimbal_motion_axis_t axis, int16_t delta, uint16_t speed,
    uint32_t *motion_time_ms)
{
    int16_t target = gimbal_clamp_angle(
        (int32_t)gimbal_motion_get_angle(axis) + delta);
    return gimbal_motion_start_absolute(
        axis, target, speed, GIMBAL_MOTION_ACCELERATION, motion_time_ms);
}

static HAL_StatusTypeDef gimbal_move_vector_step(
    const gimbal_vision_snapshot_t *vector)
{
    gimbal_vector_motion_t motion;
    uint32_t yaw_time = 0U;
    uint32_t pitch_time = 0U;
    HAL_StatusTypeDef status;
    if (!gimbal_prepare_vector_motion(vector, &motion)) {
        return HAL_OK;
    }
    status = gimbal_start_axis_step(
        GIMBAL_MOTION_AXIS_YAW, motion.yaw_delta,
        motion.yaw_speed, &yaw_time);
    if (status != HAL_OK) {
        return status;
    }
    status = gimbal_start_axis_step(
        GIMBAL_MOTION_AXIS_PITCH, motion.pitch_delta,
        motion.pitch_speed, &pitch_time);
    if (yaw_time > 0U || pitch_time > 0U) {
        osDelay(yaw_time > pitch_time ? yaw_time : pitch_time);
    }
    return status;
}

static HAL_StatusTypeDef gimbal_wait_new_vector(
    uint32_t previous_sequence, gimbal_vision_snapshot_t *snapshot)
{
    uint32_t start_tick = HAL_GetTick();
    HAL_StatusTypeDef receive_status;
    while ((HAL_GetTick() - start_tick) < GIMBAL_VISION_WAIT_TIMEOUT_MS) {
        receive_status = gimbal_vision_receive(GIMBAL_VISION_POLL_TIMEOUT_MS);
        if (receive_status == HAL_ERROR || receive_status == HAL_BUSY) {
            return HAL_ERROR;
        }
        gimbal_read_vision(snapshot);
        if (snapshot->sequence != previous_sequence && snapshot->received) {
            return snapshot->lost ? HAL_ERROR : HAL_OK;
        }
        osDelay(1U);
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef gimbal_scan_yaw_row(int16_t target_yaw)
{
    int16_t current_yaw = gimbal_motion_get_angle(GIMBAL_MOTION_AXIS_YAW);
    int16_t step = target_yaw > current_yaw ? GIMBAL_SCAN_YAW_STEP_TENTHS
                                            : -GIMBAL_SCAN_YAW_STEP_TENTHS;
    while (current_yaw != target_yaw) {
        current_yaw = gimbal_clamp_angle((int32_t)current_yaw + step);
        if ((step > 0 && current_yaw > target_yaw) ||
            (step < 0 && current_yaw < target_yaw)) {
            current_yaw = target_yaw;
        }
        if (gimbal_motion_move_absolute(
                GIMBAL_MOTION_AXIS_YAW, current_yaw,
                GIMBAL_MOTION_SPEED_RPM,
                GIMBAL_MOTION_ACCELERATION) != HAL_OK) {
            return HAL_ERROR;
        }
        if (gimbal_target_visible()) {
            return HAL_OK;
        }
    }
    return HAL_BUSY;
}

HAL_StatusTypeDef gimbal_ctrl_yaw_relative(int16_t angle_tenths,
                                           uint16_t speed_rpm,
                                           uint8_t acceleration)
{
    if (!s_initialized) {
        return HAL_ERROR;
    }
    return gimbal_motion_move_relative(
        GIMBAL_MOTION_AXIS_YAW, angle_tenths, speed_rpm, acceleration);
}

HAL_StatusTypeDef gimbal_ctrl_yaw_absolute(int16_t angle_tenths,
                                           uint16_t speed_rpm,
                                           uint8_t acceleration)
{
    if (!s_initialized) {
        return HAL_ERROR;
    }
    return gimbal_motion_move_absolute(
        GIMBAL_MOTION_AXIS_YAW, gimbal_clamp_angle(angle_tenths),
        speed_rpm, acceleration);
}

HAL_StatusTypeDef gimbal_ctrl_pitch_relative(int16_t angle_tenths,
                                             uint16_t speed_rpm,
                                             uint8_t acceleration)
{
    if (!s_initialized) {
        return HAL_ERROR;
    }
    return gimbal_motion_move_relative(
        GIMBAL_MOTION_AXIS_PITCH, angle_tenths, speed_rpm, acceleration);
}

HAL_StatusTypeDef gimbal_ctrl_pitch_absolute(int16_t angle_tenths,
                                             uint16_t speed_rpm,
                                             uint8_t acceleration)
{
    if (!s_initialized) {
        return HAL_ERROR;
    }
    return gimbal_motion_move_absolute(
        GIMBAL_MOTION_AXIS_PITCH, gimbal_clamp_angle(angle_tenths),
        speed_rpm, acceleration);
}

void gimbal_ctrl_vision_input(uint8_t status, int16_t x, int16_t y)
{
    if (status != GIMBAL_VISION_STATUS_VALID &&
        status != GIMBAL_VISION_STATUS_LOST) {
        return;
    }
    s_vision.sequence++;
    s_vision.x = x;
    s_vision.y = y;
    s_vision.lost = status == GIMBAL_VISION_STATUS_LOST;
    s_vision.received = true;
    s_vision.sequence++;
}

void gimbal_ctrl_get_state(gimbal_ctrl_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->yaw_angle_tenths =
        gimbal_motion_get_angle(GIMBAL_MOTION_AXIS_YAW);
    state->pitch_angle_tenths =
        gimbal_motion_get_angle(GIMBAL_MOTION_AXIS_PITCH);
    state->initialized = s_initialized;
}

HAL_StatusTypeDef gimbal_ctrl_initialize(void)
{
    HAL_StatusTypeDef status;
    s_initialized = false;
    s_corrected_sequence = 0U;
    gimbal_reset_vision();
    gimbal_motion_reset();
    status = gimbal_motion_home_pitch();
    if (status == HAL_OK) {
        gimbal_motion_reset();
        s_initialized = true;
    }
    return status;
}

HAL_StatusTypeDef gimbal_ctrl_scan(void)
{
    int16_t pitch;
    int16_t current_yaw;
    int16_t target_yaw;
    HAL_StatusTypeDef row_status;
    if (!s_initialized) {
        return HAL_ERROR;
    }
    if (gimbal_target_visible()) {
        return HAL_OK;
    }
    current_yaw = gimbal_motion_get_angle(GIMBAL_MOTION_AXIS_YAW);
    target_yaw = current_yaw >= 0 ? GIMBAL_ANGLE_MAX_TENTHS
                                  : GIMBAL_ANGLE_MIN_TENTHS;
    row_status = gimbal_scan_yaw_row(target_yaw);
    if (row_status != HAL_BUSY) {
        return row_status;
    }
    target_yaw = -target_yaw;
    for (;;) {
        row_status = gimbal_scan_yaw_row(target_yaw);
        if (row_status != HAL_BUSY) {
            return row_status;
        }
        pitch = gimbal_motion_get_angle(GIMBAL_MOTION_AXIS_PITCH);
        if (pitch == GIMBAL_ANGLE_MAX_TENTHS) {
            return HAL_TIMEOUT;
        }
        pitch = gimbal_clamp_angle(
            (int32_t)pitch + GIMBAL_SCAN_PITCH_STEP_TENTHS);
        row_status = gimbal_ctrl_pitch_absolute(
            pitch, GIMBAL_MOTION_SPEED_RPM, GIMBAL_MOTION_ACCELERATION);
        if (row_status != HAL_OK || gimbal_target_visible()) {
            return row_status;
        }
        target_yaw = -target_yaw;
    }
}

HAL_StatusTypeDef gimbal_ctrl_correct(void)
{
    gimbal_vision_snapshot_t vector;
    HAL_StatusTypeDef status;
    if (!s_initialized) {
        return HAL_ERROR;
    }
    gimbal_read_vision(&vector);
    if (vector.received && vector.lost) {
        return HAL_ERROR;
    }
    if (!vector.received || vector.sequence == s_corrected_sequence) {
        status = gimbal_wait_new_vector(vector.sequence, &vector);
        if (status != HAL_OK) {
            return status;
        }
    }
    s_corrected_sequence = vector.sequence;
    if (gimbal_vector_error(&vector) < GIMBAL_VISION_CONVERGENCE_PIXELS) {
        return HAL_OK;
    }
    return gimbal_move_vector_step(&vector);
}

void Gimbal_ctrl_App(void *argument)
{
    HAL_StatusTypeDef status;
    (void)argument;
    gimbal_ctrl_status = HAL_ERROR;
    if (gimbal_ctrl_initialize() != HAL_OK) {
        osThreadExit();
        return;
    }
    osDelay(GIMBAL_SCAN_START_DELAY_MS);
    for (;;) {
        status = gimbal_ctrl_scan();
        if (status != HAL_OK) {
            gimbal_ctrl_status = HAL_ERROR;
            osDelay(1U);
            continue;
        }
        gimbal_ctrl_status = HAL_OK;
        while (gimbal_ctrl_correct() == HAL_OK) {
            gimbal_ctrl_status = HAL_OK;
        }
        gimbal_ctrl_status = HAL_ERROR;
    }
}

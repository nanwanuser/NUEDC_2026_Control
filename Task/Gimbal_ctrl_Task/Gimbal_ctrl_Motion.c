#include "Gimbal_ctrl_Motion.h"

#include "Gimbal_ctrl_Task.h"
#include "cmsis_os.h"
#include "pd42s1.h"

#include <stddef.h>

#define GIMBAL_FULL_REVOLUTION_TENTHS 3600L
#define GIMBAL_MILLISECONDS_PER_MINUTE 60000UL

typedef struct {
    uint8_t motor_id;
    int16_t angle_tenths;
    int32_t motor_units;
} gimbal_axis_state_t;

static gimbal_axis_state_t s_yaw = {.motor_id = PD42S1_MOTOR_1_ID};
static gimbal_axis_state_t s_pitch = {.motor_id = PD42S1_MOTOR_2_ID};

static int32_t gimbal_motion_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int16_t gimbal_motion_clamp_angle(int32_t angle_tenths)
{
    if (angle_tenths < GIMBAL_ANGLE_MIN_TENTHS) {
        return GIMBAL_ANGLE_MIN_TENTHS;
    }
    if (angle_tenths > GIMBAL_ANGLE_MAX_TENTHS) {
        return GIMBAL_ANGLE_MAX_TENTHS;
    }
    return (int16_t)angle_tenths;
}

static HAL_StatusTypeDef gimbal_motion_from_max485(max485_status_t status)
{
    if (status == MAX485_STATUS_OK) {
        return HAL_OK;
    }
    if (status == MAX485_STATUS_TIMEOUT) {
        return HAL_TIMEOUT;
    }
    return status == MAX485_STATUS_UART_BUSY ? HAL_BUSY : HAL_ERROR;
}

static HAL_StatusTypeDef gimbal_motion_receive_success(
    uint8_t motor_id, pd42s1_command_t command)
{
    pd42s1_result_t result;
    max485_status_t status = pd42s1_receive_response(
        motor_id, command, &result, PD42S1_UART_TIMEOUT_MS);

    if (status != MAX485_STATUS_OK) {
        return gimbal_motion_from_max485(status);
    }
    return result == PD42S1_RESULT_SUCCESS ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef gimbal_motion_send_torque(
    pd42s1_direction_t direction, uint16_t current_ma)
{
    max485_status_t status = pd42s1_set_torque(
        PD42S1_MOTOR_2_ID, direction, current_ma);

    if (status != MAX485_STATUS_OK) {
        return gimbal_motion_from_max485(status);
    }
    return gimbal_motion_receive_success(PD42S1_MOTOR_2_ID,
                                         PD42S1_COMMAND_TORQUE);
}

static int32_t gimbal_motion_angle_to_units(int16_t angle_tenths)
{
    int64_t scaled = (int64_t)angle_tenths *
                     PD42S1_POSITION_UNITS_PER_REVOLUTION;

    scaled += scaled >= 0 ? GIMBAL_FULL_REVOLUTION_TENTHS / 2L
                          : -GIMBAL_FULL_REVOLUTION_TENTHS / 2L;
    return -(int32_t)(scaled / GIMBAL_FULL_REVOLUTION_TENTHS);
}

static uint32_t gimbal_motion_calculate_time(int32_t delta_tenths,
                                             uint16_t speed_rpm)
{
    uint64_t numerator = (uint64_t)gimbal_motion_abs_i32(delta_tenths) *
                         GIMBAL_MILLISECONDS_PER_MINUTE;
    uint32_t denominator = (uint32_t)speed_rpm *
                           GIMBAL_FULL_REVOLUTION_TENTHS;

    return (uint32_t)((numerator + denominator - 1U) / denominator) +
           GIMBAL_MOTION_SETTLE_MS;
}

static gimbal_axis_state_t *gimbal_motion_get_axis(gimbal_motion_axis_t axis)
{
    if (axis == GIMBAL_MOTION_AXIS_YAW) {
        return &s_yaw;
    }
    if (axis == GIMBAL_MOTION_AXIS_PITCH) {
        return &s_pitch;
    }
    return NULL;
}

static bool gimbal_motion_request_is_valid(const gimbal_axis_state_t *axis,
                                           uint16_t speed_rpm,
                                           uint8_t acceleration)
{
    return axis != NULL && speed_rpm > 0U &&
           speed_rpm <= PD42S1_MAX_SPEED_RPM &&
           acceleration <= PD42S1_MAX_ACCELERATION;
}

static HAL_StatusTypeDef gimbal_motion_send_position(
    const gimbal_axis_state_t *axis, pd42s1_command_t command,
    int32_t signed_units, uint16_t speed_rpm, uint8_t acceleration)
{
    pd42s1_direction_t direction = signed_units < 0
                                       ? PD42S1_DIRECTION_REVERSE
                                       : PD42S1_DIRECTION_FORWARD;
    uint32_t units = (uint32_t)gimbal_motion_abs_i32(signed_units);
    max485_status_t status;

    if (command == PD42S1_COMMAND_ABSOLUTE_POSITION) {
        status = pd42s1_move_absolute(axis->motor_id, direction, acceleration,
                                     speed_rpm, units);
    } else {
        status = pd42s1_move_relative(axis->motor_id, direction, acceleration,
                                     speed_rpm, units);
    }
    if (status != MAX485_STATUS_OK) {
        return gimbal_motion_from_max485(status);
    }
    return gimbal_motion_receive_success(axis->motor_id, command);
}

static HAL_StatusTypeDef gimbal_motion_clear_position(uint8_t motor_id)
{
    max485_status_t status = pd42s1_clear_position(motor_id);

    if (status != MAX485_STATUS_OK) {
        return gimbal_motion_from_max485(status);
    }
    return gimbal_motion_receive_success(
        motor_id, PD42S1_COMMAND_CLEAR_POSITION);
}

static HAL_StatusTypeDef gimbal_motion_clear_all_positions(void)
{
    HAL_StatusTypeDef status = gimbal_motion_clear_position(
        PD42S1_MOTOR_1_ID);

    if (status != HAL_OK) {
        return status;
    }
    return gimbal_motion_clear_position(PD42S1_MOTOR_2_ID);
}

static HAL_StatusTypeDef gimbal_motion_move_pitch_home(void)
{
    uint32_t reverse_units = (uint32_t)gimbal_motion_abs_i32(
        gimbal_motion_angle_to_units(GIMBAL_HOMING_REVERSE_ANGLE_TENTHS));
    max485_status_t max485_status = pd42s1_move_relative(
        PD42S1_MOTOR_2_ID, PD42S1_DIRECTION_REVERSE,
        GIMBAL_MOTION_ACCELERATION, GIMBAL_MOTION_SPEED_RPM, reverse_units);
    HAL_StatusTypeDef status;

    if (max485_status != MAX485_STATUS_OK) {
        return gimbal_motion_from_max485(max485_status);
    }
    status = gimbal_motion_receive_success(
        PD42S1_MOTOR_2_ID, PD42S1_COMMAND_RELATIVE_POSITION);
    if (status == HAL_OK) {
        osDelay(gimbal_motion_calculate_time(
            GIMBAL_HOMING_REVERSE_ANGLE_TENTHS, GIMBAL_MOTION_SPEED_RPM));
    }
    return status;
}

void gimbal_motion_reset(void)
{
    s_yaw.angle_tenths = 0;
    s_yaw.motor_units = 0;
    s_pitch.angle_tenths = 0;
    s_pitch.motor_units = 0;
}

HAL_StatusTypeDef gimbal_motion_home_pitch(void)
{
    HAL_StatusTypeDef status = gimbal_motion_send_torque(
        PD42S1_DIRECTION_FORWARD, GIMBAL_HOMING_TORQUE_MA);

    if (status != HAL_OK) {
        (void)gimbal_motion_send_torque(PD42S1_DIRECTION_FORWARD, 0U);
        return status;
    }
    osDelay(GIMBAL_HOMING_TORQUE_DURATION_MS);
    status = gimbal_motion_send_torque(PD42S1_DIRECTION_FORWARD, 0U);
    if (status != HAL_OK) {
        return status;
    }
    status = gimbal_motion_move_pitch_home();
    return status == HAL_OK ? gimbal_motion_clear_all_positions() : status;
}

HAL_StatusTypeDef gimbal_motion_start_absolute(
    gimbal_motion_axis_t axis_id, int16_t target_angle_tenths,
    uint16_t speed_rpm, uint8_t acceleration, uint32_t *motion_time_ms)
{
    gimbal_axis_state_t *axis = gimbal_motion_get_axis(axis_id);
    int32_t target_units;
    int32_t delta_angle;
    HAL_StatusTypeDef status;

    if (!gimbal_motion_request_is_valid(axis, speed_rpm, acceleration) ||
        motion_time_ms == NULL) {
        return HAL_ERROR;
    }
    target_angle_tenths = gimbal_motion_clamp_angle(target_angle_tenths);
    target_units = gimbal_motion_angle_to_units(target_angle_tenths);
    delta_angle = (int32_t)target_angle_tenths - axis->angle_tenths;
    *motion_time_ms = 0U;
    if (target_units == axis->motor_units) {
        axis->angle_tenths = target_angle_tenths;
        return HAL_OK;
    }
    status = gimbal_motion_send_position(
        axis, PD42S1_COMMAND_ABSOLUTE_POSITION, target_units,
        speed_rpm, acceleration);
    if (status != HAL_OK) {
        return status;
    }
    axis->angle_tenths = target_angle_tenths;
    axis->motor_units = target_units;
    *motion_time_ms = gimbal_motion_calculate_time(delta_angle, speed_rpm);
    return HAL_OK;
}

HAL_StatusTypeDef gimbal_motion_move_relative(
    gimbal_motion_axis_t axis_id, int16_t delta_angle_tenths,
    uint16_t speed_rpm, uint8_t acceleration)
{
    gimbal_axis_state_t *axis = gimbal_motion_get_axis(axis_id);
    int16_t target_angle;
    int32_t target_units;
    int32_t delta_units;
    int32_t actual_delta;
    HAL_StatusTypeDef status;

    if (!gimbal_motion_request_is_valid(axis, speed_rpm, acceleration)) {
        return HAL_ERROR;
    }
    target_angle = gimbal_motion_clamp_angle(
        (int32_t)axis->angle_tenths + delta_angle_tenths);
    target_units = gimbal_motion_angle_to_units(target_angle);
    delta_units = target_units - axis->motor_units;
    actual_delta = (int32_t)target_angle - axis->angle_tenths;
    if (delta_units == 0) {
        axis->angle_tenths = target_angle;
        return HAL_OK;
    }
    status = gimbal_motion_send_position(
        axis, PD42S1_COMMAND_RELATIVE_POSITION, delta_units,
        speed_rpm, acceleration);
    if (status != HAL_OK) {
        return status;
    }
    axis->angle_tenths = target_angle;
    axis->motor_units = target_units;
    osDelay(gimbal_motion_calculate_time(actual_delta, speed_rpm));
    return HAL_OK;
}

HAL_StatusTypeDef gimbal_motion_move_absolute(
    gimbal_motion_axis_t axis, int16_t target_angle_tenths,
    uint16_t speed_rpm, uint8_t acceleration)
{
    uint32_t motion_time_ms;
    HAL_StatusTypeDef status = gimbal_motion_start_absolute(
        axis, target_angle_tenths, speed_rpm, acceleration, &motion_time_ms);

    if (status == HAL_OK && motion_time_ms > 0U) {
        osDelay(motion_time_ms);
    }
    return status;
}

int16_t gimbal_motion_get_angle(gimbal_motion_axis_t axis)
{
    gimbal_axis_state_t *axis_state = gimbal_motion_get_axis(axis);
    return axis_state == NULL ? 0 : axis_state->angle_tenths;
}
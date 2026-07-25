#include "pd42s1.h"

#include "usart.h"

#define PD42S1_TORQUE_PAYLOAD_LENGTH 3U
#define PD42S1_POSITION_PAYLOAD_LENGTH 8U

static bool pd42s1_direction_is_valid(pd42s1_direction_t direction)
{
    return direction == PD42S1_DIRECTION_FORWARD ||
           direction == PD42S1_DIRECTION_REVERSE;
}

static bool pd42s1_command_is_supported(pd42s1_command_t command)
{
    return command == PD42S1_COMMAND_TORQUE ||
           command == PD42S1_COMMAND_ABSOLUTE_POSITION ||
           command == PD42S1_COMMAND_RELATIVE_POSITION ||
           command == PD42S1_COMMAND_CLEAR_POSITION;
}

static bool pd42s1_result_is_valid(uint8_t result)
{
    return result == PD42S1_RESULT_SUCCESS ||
           (result >= PD42S1_RESULT_FRAME_TOO_SHORT &&
            result <= PD42S1_RESULT_INVALID_DATA);
}

static void pd42s1_write_u16_be(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value >> 8U);
    output[1] = (uint8_t)value;
}

static void pd42s1_write_u32_be(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value >> 24U);
    output[1] = (uint8_t)(value >> 16U);
    output[2] = (uint8_t)(value >> 8U);
    output[3] = (uint8_t)value;
}

static max485_status_t pd42s1_send_position(uint8_t motor_id,
                                            pd42s1_command_t command,
                                            pd42s1_direction_t direction,
                                            uint8_t acceleration,
                                            uint16_t speed_rpm,
                                            uint32_t position_units)
{
    uint8_t payload[PD42S1_POSITION_PAYLOAD_LENGTH];

    if (!pd42s1_is_supported_motor(motor_id) ||
        !pd42s1_direction_is_valid(direction) ||
        acceleration > PD42S1_MAX_ACCELERATION ||
        speed_rpm > PD42S1_MAX_SPEED_RPM) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }

    payload[0] = (uint8_t)direction;
    payload[1] = acceleration;
    pd42s1_write_u16_be(&payload[2], speed_rpm);
    pd42s1_write_u32_be(&payload[4], position_units);
    return max485_send_frame(motor_id, (uint8_t)command, payload,
                             sizeof(payload), PD42S1_UART_TIMEOUT_MS);
}

void pd42s1_init(void)
{
    max485_init(&huart2, EN485_GPIO_Port, EN485_Pin);
}

bool pd42s1_is_supported_motor(uint8_t motor_id)
{
    return motor_id == PD42S1_MOTOR_1_ID || motor_id == PD42S1_MOTOR_2_ID;
}

max485_status_t pd42s1_set_torque(uint8_t motor_id,
                                 pd42s1_direction_t direction,
                                 uint16_t current_ma)
{
    uint8_t payload[PD42S1_TORQUE_PAYLOAD_LENGTH];

    if (!pd42s1_is_supported_motor(motor_id) ||
        !pd42s1_direction_is_valid(direction) ||
        current_ma > PD42S1_MAX_TORQUE_CURRENT_MA) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }

    payload[0] = (uint8_t)direction;
    pd42s1_write_u16_be(&payload[1], current_ma);
    return max485_send_frame(motor_id, PD42S1_COMMAND_TORQUE, payload,
                             sizeof(payload), PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_move_absolute(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units)
{
    return pd42s1_send_position(motor_id, PD42S1_COMMAND_ABSOLUTE_POSITION,
                                direction, acceleration, speed_rpm,
                                position_units);
}

max485_status_t pd42s1_move_relative(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units)
{
    return pd42s1_send_position(motor_id, PD42S1_COMMAND_RELATIVE_POSITION,
                                direction, acceleration, speed_rpm,
                                position_units);
}

max485_status_t pd42s1_clear_position(uint8_t motor_id)
{
    if (!pd42s1_is_supported_motor(motor_id)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    return max485_send_frame(motor_id, PD42S1_COMMAND_CLEAR_POSITION,
                             NULL, 0U, PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_receive_response(uint8_t motor_id,
                                       pd42s1_command_t command,
                                       pd42s1_result_t *result,
                                       uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status;

    if (!pd42s1_is_supported_motor(motor_id) ||
        !pd42s1_command_is_supported(command) || result == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    status = max485_receive_frame(&frame, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id || frame.function != (uint8_t)command ||
        frame.payload_length != 1U ||
        !pd42s1_result_is_valid(frame.payload[0])) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }

    *result = (pd42s1_result_t)frame.payload[0];
    return MAX485_STATUS_OK;
}

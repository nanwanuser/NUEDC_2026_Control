#include "pd42s1.h"

#include "usart.h"

#define PD42S1_TORQUE_PAYLOAD_LENGTH 3U
#define PD42S1_POSITION_PAYLOAD_LENGTH 8U
#define PD42S1_DRIVER_PARAMETERS_MIN_PAYLOAD_LENGTH 2U
/* Mode, direction, four speed bytes, two current bytes. */
#define PD42S1_HOME_PARAMETERS_PAYLOAD_LENGTH 8U
/* One int32 position. */
#define PD42S1_LIMIT_ORIGIN_PAYLOAD_LENGTH 4U

static bool pd42s1_direction_is_valid(pd42s1_direction_t direction)
{
    return direction == PD42S1_DIRECTION_FORWARD ||
           direction == PD42S1_DIRECTION_REVERSE;
}

/* The control commands all answer with a single result byte. The homing commands
   echo their parameters back as well, so they consume their own replies through
   pd42s1_receive_home_reply() rather than going through here. */
static bool pd42s1_command_is_supported(pd42s1_command_t command)
{
    return command == PD42S1_COMMAND_TORQUE ||
           command == PD42S1_COMMAND_ABSOLUTE_POSITION ||
           command == PD42S1_COMMAND_RELATIVE_POSITION ||
           command == PD42S1_COMMAND_CLEAR_POSITION ||
           command == PD42S1_COMMAND_RELEASE_STALL_PROTECTION ||
           command == PD42S1_COMMAND_CLEAR_STATE;
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

static int32_t pd42s1_read_i32_be(const uint8_t *input)
{
    const uint32_t raw = ((uint32_t)input[0] << 24U) |
                         ((uint32_t)input[1] << 16U) |
                         ((uint32_t)input[2] << 8U) |
                         (uint32_t)input[3];

    return raw <= (uint32_t)INT32_MAX
               ? (int32_t)raw
               : (int32_t)((int64_t)raw - 0x100000000LL);
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

max485_status_t pd42s1_receive_home_reply(uint8_t motor_id,
                                        pd42s1_command_t command,
                                        uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status = max485_receive_frame(&frame, timeout_ms);

    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id || frame.function != (uint8_t)command ||
        frame.payload_length == 0U ||
        !pd42s1_result_is_valid(frame.payload[0])) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }
    return frame.payload[0] == PD42S1_RESULT_SUCCESS
               ? MAX485_STATUS_OK
               : MAX485_STATUS_UNEXPECTED_FRAME;
}

max485_status_t pd42s1_set_left_limit_origin(uint8_t motor_id,
                                          int32_t position_units)
{
    uint8_t payload[PD42S1_LIMIT_ORIGIN_PAYLOAD_LENGTH];

    if (!pd42s1_is_supported_motor(motor_id)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    /* The field is int32 on the wire, so the cast is the two's-complement
       encoding the drive expects, not a range change. */
    pd42s1_write_u32_be(payload, (uint32_t)position_units);
    return max485_send_frame(motor_id, PD42S1_COMMAND_SET_LEFT_LIMIT_ORIGIN,
                             payload, sizeof(payload), PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_set_home_parameters(uint8_t motor_id,
                                         pd42s1_home_mode_t mode,
                                         pd42s1_direction_t direction,
                                         uint16_t speed_rpm,
                                         uint16_t limit_current_ma)
{
    uint8_t payload[PD42S1_HOME_PARAMETERS_PAYLOAD_LENGTH];

    if (!pd42s1_is_supported_motor(motor_id) ||
        !pd42s1_direction_is_valid(direction) ||
        mode > PD42S1_HOME_MODE_RIGHT_LIMIT ||
        speed_rpm > PD42S1_MAX_SPEED_RPM ||
        limit_current_ma > PD42S1_MAX_TORQUE_CURRENT_MA) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }

    payload[0] = (uint8_t)mode;
    payload[1] = (uint8_t)direction;
    /* The speed field is four bytes wide even though the range fits in two. */
    pd42s1_write_u32_be(&payload[2], speed_rpm);
    pd42s1_write_u16_be(&payload[6], limit_current_ma);
    return max485_send_frame(motor_id, PD42S1_COMMAND_SET_HOME_PARAMETERS,
                             payload, sizeof(payload), PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_trigger_home(uint8_t motor_id,
                                   pd42s1_home_trigger_t trigger)
{
    uint8_t payload[1];

    if (!pd42s1_is_supported_motor(motor_id) ||
        trigger > PD42S1_HOME_TRIGGER_MULTI_TURN) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    payload[0] = (uint8_t)trigger;
    return max485_send_frame(motor_id, PD42S1_COMMAND_TRIGGER_HOME,
                             payload, sizeof(payload), PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_abort_home(uint8_t motor_id)
{
    if (!pd42s1_is_supported_motor(motor_id)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    return max485_send_frame(motor_id, PD42S1_COMMAND_ABORT_HOME,
                             NULL, 0U, PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_read_home_state(uint8_t motor_id,
                                      pd42s1_home_state_t *state,
                                      uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status;

    if (!pd42s1_is_supported_motor(motor_id) || state == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    status = max485_send_frame(motor_id, PD42S1_COMMAND_READ_HOME_STATE,
                              NULL, 0U, PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    status = max485_receive_frame(&frame, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id ||
        frame.function != (uint8_t)PD42S1_COMMAND_READ_HOME_STATE ||
        frame.payload_length < 2U ||
        frame.payload[0] != PD42S1_RESULT_SUCCESS ||
        frame.payload[1] > (uint8_t)PD42S1_HOME_STATE_NOT_FOUND) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }
    *state = (pd42s1_home_state_t)frame.payload[1];
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_read_realtime_position(uint8_t motor_id,
                                             int32_t *position_units,
                                             uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status;

    if (!pd42s1_is_supported_motor(motor_id) || position_units == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    status = max485_send_frame(motor_id,
                               PD42S1_COMMAND_READ_REALTIME_POSITION,
                               NULL, 0U, PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    status = max485_receive_frame(&frame, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id ||
        frame.function != (uint8_t)PD42S1_COMMAND_READ_REALTIME_POSITION ||
        frame.payload_length != 5U ||
        frame.payload[0] != PD42S1_RESULT_SUCCESS) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }
    *position_units = pd42s1_read_i32_be(&frame.payload[1]);
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_read_position_error(uint8_t motor_id,
                                          int32_t *position_error_units,
                                          uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status;

    if (!pd42s1_is_supported_motor(motor_id) ||
        position_error_units == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    status = max485_send_frame(motor_id,
                               PD42S1_COMMAND_READ_POSITION_ERROR,
                               NULL, 0U, PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    status = max485_receive_frame(&frame, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id ||
        frame.function != (uint8_t)PD42S1_COMMAND_READ_POSITION_ERROR ||
        frame.payload_length != 5U ||
        frame.payload[0] != PD42S1_RESULT_SUCCESS) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }
    *position_error_units = pd42s1_read_i32_be(&frame.payload[1]);
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_read_arrival_flag(uint8_t motor_id,
                                        pd42s1_arrival_t *arrival,
                                        uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status;

    if (!pd42s1_is_supported_motor(motor_id) || arrival == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    status = max485_send_frame(motor_id, PD42S1_COMMAND_READ_ARRIVAL,
                               NULL, 0U, PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    status = max485_receive_frame(&frame, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id ||
        frame.function != (uint8_t)PD42S1_COMMAND_READ_ARRIVAL ||
        frame.payload_length != 2U ||
        frame.payload[0] != PD42S1_RESULT_SUCCESS ||
        frame.payload[1] > (uint8_t)PD42S1_ARRIVAL_REACHED) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }
    *arrival = (pd42s1_arrival_t)frame.payload[1];
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_read_work_mode(uint8_t motor_id,
                                     pd42s1_work_mode_t *mode,
                                     uint32_t timeout_ms)
{
    max485_frame_t frame;
    max485_status_t status;

    if (!pd42s1_is_supported_motor(motor_id) || mode == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    status = max485_send_frame(motor_id,
                               PD42S1_COMMAND_READ_DRIVER_PARAMETERS,
                               NULL, 0U, PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    status = max485_receive_frame(&frame, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    if (frame.address != motor_id ||
        frame.function != (uint8_t)PD42S1_COMMAND_READ_DRIVER_PARAMETERS ||
        frame.payload_length < PD42S1_DRIVER_PARAMETERS_MIN_PAYLOAD_LENGTH ||
        frame.payload[0] != PD42S1_RESULT_SUCCESS ||
        frame.payload[1] > (uint8_t)PD42S1_WORK_MODE_HOME) {
        return MAX485_STATUS_UNEXPECTED_FRAME;
    }
    *mode = (pd42s1_work_mode_t)frame.payload[1];
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_clear_position(uint8_t motor_id)
{
    if (!pd42s1_is_supported_motor(motor_id)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    return max485_send_frame(motor_id, PD42S1_COMMAND_CLEAR_POSITION,
                             NULL, 0U, PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_release_stall_protection(uint8_t motor_id)
{
    if (!pd42s1_is_supported_motor(motor_id)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    return max485_send_frame(motor_id,
                             PD42S1_COMMAND_RELEASE_STALL_PROTECTION,
                             NULL, 0U, PD42S1_UART_TIMEOUT_MS);
}

max485_status_t pd42s1_clear_state(uint8_t motor_id)
{
    if (!pd42s1_is_supported_motor(motor_id)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    return max485_send_frame(motor_id, PD42S1_COMMAND_CLEAR_STATE,
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

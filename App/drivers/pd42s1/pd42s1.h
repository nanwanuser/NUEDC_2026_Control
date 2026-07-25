#ifndef PD42S1_H
#define PD42S1_H

#include "max485.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PD42S1_MOTOR_1_ID 1U
#define PD42S1_MOTOR_2_ID 2U
#define PD42S1_POSITION_UNITS_PER_REVOLUTION 51200U
#define PD42S1_MAX_ACCELERATION 200U
#define PD42S1_MAX_SPEED_RPM 6000U
#define PD42S1_MAX_TORQUE_CURRENT_MA 3000U
#define PD42S1_UART_TIMEOUT_MS 20U

typedef enum {
    PD42S1_DIRECTION_FORWARD = 0,
    PD42S1_DIRECTION_REVERSE = 1,
} pd42s1_direction_t;

typedef enum {
    PD42S1_COMMAND_TORQUE = 0xF0,
    PD42S1_COMMAND_ABSOLUTE_POSITION = 0xF2,
    PD42S1_COMMAND_RELATIVE_POSITION = 0xF3,
    PD42S1_COMMAND_CLEAR_POSITION = 0xF8,
} pd42s1_command_t;

typedef enum {
    PD42S1_RESULT_SUCCESS = 0x01,
    PD42S1_RESULT_FRAME_TOO_SHORT = 0xE1,
    PD42S1_RESULT_INVALID_HEADER = 0xE2,
    PD42S1_RESULT_INVALID_TAIL = 0xE3,
    PD42S1_RESULT_CHECKSUM_ERROR = 0xE4,
    PD42S1_RESULT_UNSUPPORTED_FUNCTION = 0xE5,
    PD42S1_RESULT_INVALID_DATA = 0xE6,
} pd42s1_result_t;

/**
 * @brief Initialize the PD42S1 bus on USART2 and PE8 (EN485).
 * @note Call after MX_GPIO_Init() and MX_USART2_UART_Init().
 */
void pd42s1_init(void);

/**
 * @brief Return whether the address is one of this project's motors (1 or 2).
 */
bool pd42s1_is_supported_motor(uint8_t motor_id);

/**
 * @brief Send a closed-loop torque-mode command.
 * @param current_ma Target Iq current from 0 to 3000 mA.
 * @note When drive response is enabled, call pd42s1_receive_response()
 *       immediately after a successful send and before sending another command.
 */
max485_status_t pd42s1_set_torque(uint8_t motor_id,
                                 pd42s1_direction_t direction,
                                 uint16_t current_ma);

/**
 * @brief Send a closed-loop absolute-position command.
 * @param acceleration Ramp value from 0 to 200; zero means direct start.
 * @param speed_rpm Target speed from 0 to 6000 RPM.
 * @param position_units Position magnitude; 51200 units equal one revolution.
 * @note When drive response is enabled, call pd42s1_receive_response()
 *       immediately after a successful send and before sending another command.
 */
max485_status_t pd42s1_move_absolute(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units);

/**
 * @brief Send a closed-loop relative-position command.
 * @param acceleration Ramp value from 0 to 200; zero means direct start.
 * @param speed_rpm Target speed from 0 to 6000 RPM.
 * @param position_units Offset magnitude; 51200 units equal one revolution.
 * @note When drive response is enabled, call pd42s1_receive_response()
 *       immediately after a successful send and before sending another command.
 */
max485_status_t pd42s1_move_relative(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units);

/**
 * @brief Clear the motor's current angle, position error, and pulse count.
 * @note The command has no payload. Receive its response before another send.
 */
max485_status_t pd42s1_clear_position(uint8_t motor_id);

/**
 * @brief Receive and validate the response to a PD42S1 control command.
 * @param result Receives the drive's first response byte (0x01 or 0xE1-0xE6).
 */
max485_status_t pd42s1_receive_response(uint8_t motor_id,
                                       pd42s1_command_t command,
                                       pd42s1_result_t *result,
                                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif

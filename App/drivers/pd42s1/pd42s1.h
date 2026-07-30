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
    PD42S1_COMMAND_SET_HOME_PARAMETERS = 0x91,
    PD42S1_COMMAND_TRIGGER_HOME = 0x92,
    PD42S1_COMMAND_ABORT_HOME = 0x93,
    PD42S1_COMMAND_READ_HOME_STATE = 0x96,
    PD42S1_COMMAND_TORQUE = 0xF0,
    PD42S1_COMMAND_ABSOLUTE_POSITION = 0xF2,
    PD42S1_COMMAND_RELATIVE_POSITION = 0xF3,
    PD42S1_COMMAND_CLEAR_POSITION = 0xF8,
} pd42s1_command_t;

/* Which end the drive seeks and whether a switch marks it. The two "no limit"
   modes detect the end by stall current instead, which needs no switch but drives
   the mechanism into its hard stop to find it. */
typedef enum {
    PD42S1_HOME_MODE_LEFT_NO_LIMIT = 0,
    PD42S1_HOME_MODE_RIGHT_NO_LIMIT = 1,
    PD42S1_HOME_MODE_LEFT_LIMIT = 2,
    PD42S1_HOME_MODE_RIGHT_LIMIT = 3,
} pd42s1_home_mode_t;

/* How the drive travels to the origin once it knows where it is. */
typedef enum {
    PD42S1_HOME_TRIGGER_SINGLE_TURN = 0,
    PD42S1_HOME_TRIGGER_NEAREST = 1,
    PD42S1_HOME_TRIGGER_MULTI_TURN = 2,
} pd42s1_home_trigger_t;

typedef enum {
    PD42S1_HOME_STATE_IDLE = 0,
    PD42S1_HOME_STATE_RUNNING = 1,
    PD42S1_HOME_STATE_COMPLETE = 2,
    PD42S1_HOME_STATE_NOT_FOUND = 3,
} pd42s1_home_state_t;

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
 * @brief Set how the drive looks for its origin (0x91).
 * @param mode Which end to seek and whether a limit switch marks it.
 * @param speed_rpm Seek speed from 0 to 6000 RPM.
 * @param limit_current_ma Stall threshold from 0 to 3000 mA, used to detect the
 *        end in the two no-limit modes and ignored in the switch modes.
 * @note Persisted by the drive, so it only has to be sent when it changes.
 */
max485_status_t pd42s1_set_home_parameters(uint8_t motor_id,
                                         pd42s1_home_mode_t mode,
                                         pd42s1_direction_t direction,
                                         uint16_t speed_rpm,
                                         uint16_t limit_current_ma);

/**
 * @brief Start a homing run with the stored parameters (0x92).
 * @note Returns as soon as the drive accepts the command; the run itself takes
 *       seconds, so poll pd42s1_read_home_state() for the outcome.
 */
max485_status_t pd42s1_trigger_home(uint8_t motor_id,
                                   pd42s1_home_trigger_t trigger);

/**
 * @brief Interrupt a homing run in progress (0x93).
 */
max485_status_t pd42s1_abort_home(uint8_t motor_id);

/**
 * @brief Consume the reply to a homing command (0x91, 0x92, or 0x93).
 * @return MAX485_STATUS_OK only when the drive reported success.
 * @note Separate from pd42s1_receive_response() because these replies echo their
 *       parameters after the result byte, so the payload length varies.
 */
max485_status_t pd42s1_receive_home_reply(uint8_t motor_id,
                                        pd42s1_command_t command,
                                        uint32_t timeout_ms);

/**
 * @brief Read whether a homing run is idle, running, finished, or failed (0x96).
 * @param state Receives the drive's homing state on success.
 * @note Sends the query and consumes the reply, so unlike the commands above it
 *       needs no separate pd42s1_receive_response() call.
 */
max485_status_t pd42s1_read_home_state(uint8_t motor_id,
                                      pd42s1_home_state_t *state,
                                      uint32_t timeout_ms);

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

#ifndef TEST_PD42S1_H
#define TEST_PD42S1_H

#include <stdint.h>

#define PD42S1_MOTOR_1_ID 1U
#define PD42S1_MOTOR_2_ID 2U
#define PD42S1_POSITION_UNITS_PER_REVOLUTION 51200U
#define PD42S1_MAX_ACCELERATION 200U
#define PD42S1_MAX_SPEED_RPM 6000U
#define PD42S1_MAX_TORQUE_CURRENT_MA 3000U
#define PD42S1_UART_TIMEOUT_MS 20U

typedef enum {
    MAX485_STATUS_OK = 0,
    MAX485_STATUS_INVALID_ARGUMENT
} max485_status_t;

typedef enum {
    PD42S1_DIRECTION_FORWARD = 0,
    PD42S1_DIRECTION_REVERSE = 1
} pd42s1_direction_t;

typedef enum {
    PD42S1_COMMAND_TORQUE = 0xF0,
    PD42S1_COMMAND_ABSOLUTE_POSITION = 0xF2,
    PD42S1_COMMAND_RELATIVE_POSITION = 0xF3,
    PD42S1_COMMAND_CLEAR_POSITION = 0xF8,
    PD42S1_COMMAND_RELEASE_STALL_PROTECTION = 0xF9,
    PD42S1_COMMAND_CLEAR_STATE = 0xFB
} pd42s1_command_t;

typedef enum {
    PD42S1_RESULT_SUCCESS = 0x01
} pd42s1_result_t;

typedef enum {
    PD42S1_ARRIVAL_NOT_REACHED = 0,
    PD42S1_ARRIVAL_REACHED = 1,
} pd42s1_arrival_t;

void pd42s1_init(void);
max485_status_t pd42s1_set_torque(uint8_t motor_id,
                                 pd42s1_direction_t direction,
                                 uint16_t current_ma);
max485_status_t pd42s1_move_absolute(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units);
max485_status_t pd42s1_move_relative(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units);
max485_status_t pd42s1_clear_position(uint8_t motor_id);
max485_status_t pd42s1_release_stall_protection(uint8_t motor_id);
max485_status_t pd42s1_clear_state(uint8_t motor_id);
max485_status_t pd42s1_receive_response(uint8_t motor_id,
                                       pd42s1_command_t command,
                                       pd42s1_result_t *result,
                                       uint32_t timeout_ms);
max485_status_t pd42s1_read_arrival_flag(uint8_t motor_id,
                                        pd42s1_arrival_t *arrival,
                                        uint32_t timeout_ms);
max485_status_t pd42s1_read_position_error(uint8_t motor_id,
                                           int32_t *position_units,
                                           uint32_t timeout_ms);
max485_status_t pd42s1_read_realtime_position(uint8_t motor_id,
                                              int32_t *position_units,
                                              uint32_t timeout_ms);

#endif

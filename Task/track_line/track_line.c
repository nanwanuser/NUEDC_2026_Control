#include "track_line.h"

#include "Gray_Sensor.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#if CHASSIS_LEFT_MOTOR_INDEX >= MOTOR_COUNT
#error "CHASSIS_LEFT_MOTOR_INDEX must be smaller than MOTOR_COUNT"
#endif

#if CHASSIS_RIGHT_MOTOR_INDEX >= MOTOR_COUNT
#error "CHASSIS_RIGHT_MOTOR_INDEX must be smaller than MOTOR_COUNT"
#endif

#if CHASSIS_LEFT_MOTOR_INDEX == CHASSIS_RIGHT_MOTOR_INDEX
#error "Chassis left and right motor indexes must be different"
#endif

#if TRACK_LINE_SENSOR_INDEX >= gray_sensor_count
#error "TRACK_LINE_SENSOR_INDEX must be smaller than gray_sensor_count"
#endif

#if CHASSIS_TASK_PERIOD_MS == 0U
#error "CHASSIS_TASK_PERIOD_MS must be greater than zero"
#endif

gray_sensor_config Gray_Sensor_Config[gray_sensor_count] = {
    {
        .ad0_port = GRAY_AD0_GPIO_Port,
        .ad0_pin = GRAY_AD0_Pin,
        .ad1_port = GRAY_AD1_GPIO_Port,
        .ad1_pin = GRAY_AD1_Pin,
        .ad2_port = GRAY_AD2_GPIO_Port,
        .ad2_pin = GRAY_AD2_Pin,
        .out_port = GRAY_OUT_GPIO_Port,
        .out_pin = GRAY_OUT_Pin,
        .active_level = GRAY_SENSOR_ACTIVE_HIGH,
    },
};

typedef enum {
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_DRIVE,
    CHASSIS_MODE_BRAKE,
} chassis_mode;

typedef struct {
    float left_speed;
    float right_speed;
    chassis_mode mode;
} chassis_command;

static chassis_command s_chassis_command;
static uint8_t s_track_line_enabled;
static uint8_t s_sensor_data;
static uint8_t s_line_detected;
static int8_t s_search_direction;
static float s_line_error;
static uint32_t s_lost_time_ms;

static float chassis_abs(float value) {
    if (value < 0.0f) {
        return -value;
    }

    return value;
}

static float chassis_limit_speed(float speed) {
    if (speed < MOTOR_SPEED_COMMAND_MIN) {
        return MOTOR_SPEED_COMMAND_MIN;
    }
    if (speed > MOTOR_SPEED_COMMAND_MAX) {
        return MOTOR_SPEED_COMMAND_MAX;
    }

    return speed;
}

static uint8_t track_line_sensor_is_valid(void) {
    const gray_sensor_config *config = &Gray_Sensor_Config[TRACK_LINE_SENSOR_INDEX];

    return config->ad0_port != NULL &&
           config->ad1_port != NULL &&
           config->ad2_port != NULL &&
           config->out_port != NULL;
}

static uint8_t track_line_calculate_error(uint8_t sensor_data,
                                          float *line_error) {
    uint8_t channel;
    uint8_t active_count = 0U;
    float position_sum = 0.0f;

    if (line_error == NULL) {
        return 0U;
    }

    for (channel = 0U; channel < 8U; channel++) {
        float channel_position;

        if ((sensor_data & (uint8_t)(1U << channel)) == 0U) {
            continue;
        }

        channel_position = 3.5f - (float)channel;
        if (TRACK_LINE_CHANNEL_0_IS_LEFT == 0U) {
            channel_position = -channel_position;
        }

        position_sum += channel_position;
        active_count++;
    }

    if (active_count == 0U) {
        return 0U;
    }

    *line_error = position_sum / (float)active_count;
    return 1U;
}

static void track_line_apply_steering(float line_error) {
    float error_magnitude = chassis_abs(line_error);
    float inner_speed;
    float outer_speed;

    if (line_error == 0.0f) {
        chassis_set_wheel_speed(TRACK_LINE_BASE_SPEED,
                                TRACK_LINE_BASE_SPEED);
        return;
    }

    if (error_magnitude >= TRACK_LINE_SHARP_ERROR_THRESHOLD) {
        inner_speed = TRACK_LINE_SHARP_INNER_SPEED;
        outer_speed = TRACK_LINE_SHARP_OUTER_SPEED;
    } else if (error_magnitude >= TRACK_LINE_NORMAL_ERROR_THRESHOLD) {
        inner_speed = TRACK_LINE_NORMAL_INNER_SPEED;
        outer_speed = TRACK_LINE_NORMAL_OUTER_SPEED;
    } else {
        inner_speed = TRACK_LINE_SLIGHT_INNER_SPEED;
        outer_speed = TRACK_LINE_SLIGHT_OUTER_SPEED;
    }

    if (line_error > 0.0f) {
        chassis_set_wheel_speed(inner_speed, outer_speed);
    } else {
        chassis_set_wheel_speed(outer_speed, inner_speed);
    }
}

static void track_line_handle_lost(void) {
    if (s_lost_time_ms < TRACK_LINE_LOST_STOP_MS) {
        s_lost_time_ms += CHASSIS_TASK_PERIOD_MS;
    }

    if (s_search_direction == 0 ||
        s_lost_time_ms >= TRACK_LINE_LOST_STOP_MS) {
        chassis_stop();
        return;
    }

    if (s_search_direction > 0) {
        chassis_set_wheel_speed(0.0f, TRACK_LINE_LOST_SEARCH_SPEED);
    } else {
        chassis_set_wheel_speed(TRACK_LINE_LOST_SEARCH_SPEED, 0.0f);
    }
}

static void chassis_save_command(chassis_mode mode,
                                 float left_speed,
                                 float right_speed) {
    taskENTER_CRITICAL();
    s_chassis_command.left_speed = left_speed;
    s_chassis_command.right_speed = right_speed;
    s_chassis_command.mode = mode;
    taskEXIT_CRITICAL();
}

static chassis_command chassis_get_command(void) {
    chassis_command command;

    taskENTER_CRITICAL();
    command = s_chassis_command;
    taskEXIT_CRITICAL();

    return command;
}

void chassis_init(void) {
    chassis_save_command(CHASSIS_MODE_STOP, 0.0f, 0.0f);
    chassis_process();
}

void chassis_set_wheel_speed(float left_speed, float right_speed) {
    left_speed = chassis_limit_speed(left_speed);
    right_speed = chassis_limit_speed(right_speed);

    chassis_save_command(CHASSIS_MODE_DRIVE, left_speed, right_speed);
}

void chassis_set_motion(float forward_speed, float turn_speed) {
    float left_speed;
    float right_speed;
    float maximum_speed;
    float scale;

    forward_speed = chassis_limit_speed(forward_speed);
    turn_speed = chassis_limit_speed(turn_speed);

    left_speed = forward_speed - turn_speed;
    right_speed = forward_speed + turn_speed;
    maximum_speed = chassis_abs(left_speed);
    if (chassis_abs(right_speed) > maximum_speed) {
        maximum_speed = chassis_abs(right_speed);
    }

    if (maximum_speed > MOTOR_SPEED_COMMAND_MAX) {
        scale = MOTOR_SPEED_COMMAND_MAX / maximum_speed;
        left_speed *= scale;
        right_speed *= scale;
    }

    chassis_set_wheel_speed(left_speed, right_speed);
}

void chassis_forward(float speed) {
    speed = chassis_abs(chassis_limit_speed(speed));
    chassis_set_wheel_speed(speed, speed);
}

void chassis_backward(float speed) {
    speed = chassis_abs(chassis_limit_speed(speed));
    chassis_set_wheel_speed(-speed, -speed);
}

void chassis_turn_left(float speed) {
    speed = chassis_abs(chassis_limit_speed(speed));
    chassis_set_wheel_speed(-speed, speed);
}

void chassis_turn_right(float speed) {
    speed = chassis_abs(chassis_limit_speed(speed));
    chassis_set_wheel_speed(speed, -speed);
}

void chassis_stop(void) {
    chassis_save_command(CHASSIS_MODE_STOP, 0.0f, 0.0f);
}

void chassis_brake(void) {
    chassis_save_command(CHASSIS_MODE_BRAKE, 0.0f, 0.0f);
}

void chassis_process(void) {
    chassis_command command = chassis_get_command();

    switch (command.mode) {
        case CHASSIS_MODE_DRIVE:
            motor_set_speed(Motor_Config[CHASSIS_LEFT_MOTOR_INDEX],
                            command.left_speed);
            motor_set_speed(Motor_Config[CHASSIS_RIGHT_MOTOR_INDEX],
                            command.right_speed);
            break;
        case CHASSIS_MODE_BRAKE:
            motor_brake(Motor_Config[CHASSIS_LEFT_MOTOR_INDEX]);
            motor_brake(Motor_Config[CHASSIS_RIGHT_MOTOR_INDEX]);
            break;
        case CHASSIS_MODE_STOP:
        default:
            motor_stop(Motor_Config[CHASSIS_LEFT_MOTOR_INDEX]);
            motor_stop(Motor_Config[CHASSIS_RIGHT_MOTOR_INDEX]);
            break;
    }
}

void track_line_init(void) {
    s_sensor_data = 0U;
    s_line_detected = 0U;
    s_search_direction = 0;
    s_line_error = 0.0f;
    s_lost_time_ms = 0U;
    s_track_line_enabled = track_line_sensor_is_valid();

    gray_sensor_init();
    chassis_init();
}

void track_line_process(void) {
    float current_error;

    if (s_track_line_enabled == 0U) {
        return;
    }
    if (!track_line_sensor_is_valid()) {
        s_line_detected = 0U;
        chassis_stop();
        return;
    }

    s_sensor_data = gray_sensor_read_byte(
        Gray_Sensor_Config[TRACK_LINE_SENSOR_INDEX]);
    s_line_detected = track_line_calculate_error(s_sensor_data,
                                                  &current_error);
    if (s_line_detected == 0U) {
        track_line_handle_lost();
        return;
    }

    s_lost_time_ms = 0U;
    s_line_error = current_error;

    if (current_error > 0.25f) {
        s_search_direction = 1;
    } else if (current_error < -0.25f) {
        s_search_direction = -1;
    }

    track_line_apply_steering(current_error);
}

void track_line_enable(void) {
    s_line_detected = 0U;
    s_search_direction = 0;
    s_line_error = 0.0f;
    s_lost_time_ms = 0U;
    s_track_line_enabled = track_line_sensor_is_valid();
}

void track_line_disable(void) {
    s_track_line_enabled = 0U;
    s_line_detected = 0U;
    chassis_stop();
}

uint8_t track_line_get_sensor_data(void) {
    return s_sensor_data;
}

float track_line_get_error(void) {
    return s_line_error;
}

uint8_t track_line_is_detected(void) {
    return s_line_detected;
}

void Track_line_App(void *argument) {
    uint32_t next_wake_tick = osKernelGetTickCount();

    (void)argument;
    track_line_init();

    for (;;) {
        track_line_process();
        chassis_process();
        next_wake_tick += CHASSIS_TASK_PERIOD_MS;
        osDelayUntil(next_wake_tick);
    }
}

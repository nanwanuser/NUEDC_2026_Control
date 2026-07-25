#include "motor_speed_control.h"

#include "FreeRTOS.h"
#include "task.h"

#if MOTOR_COUNT != 2U
#error "motor_speed_control currently requires exactly two motors"
#endif

typedef struct {
    float target_rpm;
    motor_speed_control_mode mode;
} motor_speed_control_request;

static motor_speed_control_state s_motor_speed_state[MOTOR_COUNT];
static volatile motor_speed_control_request s_motor_speed_request[MOTOR_COUNT];

static const float s_encoder_forward_sign[MOTOR_COUNT] = {
    MOTOR1_ENCODER_FORWARD_SIGN,
    MOTOR2_ENCODER_FORWARD_SIGN,
};

static float motor_speed_limit(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }

    return value;
}

static uint8_t motor_speed_sign_changed(float old_target,
                                        float new_target) {
    return (old_target > 0.0f && new_target < 0.0f) ||
           (old_target < 0.0f && new_target > 0.0f);
}

static void motor_speed_reset_pid(uint32_t motor_index) {
    s_motor_speed_state[motor_index].pwm_command = 0.0f;
    s_motor_speed_state[motor_index].integrator = 0.0f;
    s_motor_speed_state[motor_index].filter_state = 0.0f;
}

static void motor_speed_save_request(uint32_t motor_index,
                                     motor_speed_control_mode mode,
                                     float target_rpm) {
    if (motor_index >= MOTOR_COUNT) {
        return;
    }

    taskENTER_CRITICAL();
    s_motor_speed_request[motor_index].target_rpm = target_rpm;
    s_motor_speed_request[motor_index].mode = mode;
    taskEXIT_CRITICAL();
}

static void motor_speed_get_requests(
    motor_speed_control_request requests[MOTOR_COUNT]) {
    uint32_t i;

    taskENTER_CRITICAL();
    for (i = 0U; i < MOTOR_COUNT; i++) {
        requests[i].target_rpm = s_motor_speed_request[i].target_rpm;
        requests[i].mode = s_motor_speed_request[i].mode;
    }
    taskEXIT_CRITICAL();
}

void motor_speed_control_init(void) {
    uint32_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        s_motor_speed_state[i] = (motor_speed_control_state){0};
        s_motor_speed_request[i].target_rpm = 0.0f;
        s_motor_speed_request[i].mode = MOTOR_SPEED_MODE_STOP;
        motor_stop(Motor_Config[i]);
    }
}

void motor_speed_control_set_target(uint32_t motor_index, float target_rpm) {
    target_rpm = motor_speed_limit(target_rpm,
                                   MOTOR_SPEED_TARGET_RPM_MIN,
                                   MOTOR_SPEED_TARGET_RPM_MAX);

    if (target_rpm == 0.0f) {
        motor_speed_control_stop(motor_index);
        return;
    }

    motor_speed_save_request(motor_index,
                             MOTOR_SPEED_MODE_DRIVE,
                             target_rpm);
}

void motor_speed_control_stop(uint32_t motor_index) {
    motor_speed_save_request(motor_index, MOTOR_SPEED_MODE_STOP, 0.0f);
}

void motor_speed_control_brake(uint32_t motor_index) {
    motor_speed_save_request(motor_index, MOTOR_SPEED_MODE_BRAKE, 0.0f);
}

void motor_speed_control_stop_all(void) {
    uint32_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_speed_control_stop(i);
    }
}

void motor_speed_control_brake_all(void) {
    uint32_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_speed_control_brake(i);
    }
}

void motor_speed_control_process(void) {
    motor_speed_control_request requests[MOTOR_COUNT];
    uint32_t i;

    motor_speed_get_requests(requests);
    encoder_update_motion(MOTOR_SPEED_CONTROL_PERIOD_S);

    for (i = 0U; i < MOTOR_COUNT; i++) {
        const encoder_motion_data *motion = encoder_get_motion_data(i);
        motor_speed_control_state *state = &s_motor_speed_state[i];
        float error_rpm;
        float filter_output;
        float pwm_command;
        float integrator_step;

        if (motion == NULL) {
            requests[i].mode = MOTOR_SPEED_MODE_STOP;
        } else {
            state->measured_rpm = motion->rotational_speed_rpm *
                                  s_encoder_forward_sign[i];
        }

        if (requests[i].mode != MOTOR_SPEED_MODE_DRIVE) {
            state->target_rpm = 0.0f;
            state->mode = requests[i].mode;
            motor_speed_reset_pid(i);

            if (requests[i].mode == MOTOR_SPEED_MODE_BRAKE) {
                motor_brake(Motor_Config[i]);
            } else {
                motor_stop(Motor_Config[i]);
            }
            continue;
        }

        if (state->mode != MOTOR_SPEED_MODE_DRIVE ||
            motor_speed_sign_changed(state->target_rpm,
                                     requests[i].target_rpm) != 0U) {
            motor_speed_reset_pid(i);
        }

        state->mode = MOTOR_SPEED_MODE_DRIVE;
        state->target_rpm = requests[i].target_rpm;
        error_rpm = state->target_rpm - state->measured_rpm;
        filter_output = MOTOR_SPEED_PID_FILTER_N *
                        (MOTOR_SPEED_PID_KD * error_rpm -
                         state->filter_state);
        pwm_command = MOTOR_SPEED_PID_KP * error_rpm +
                      state->integrator + filter_output;

        integrator_step = MOTOR_SPEED_PID_KI * error_rpm *
                          MOTOR_SPEED_CONTROL_PERIOD_S;
        if (!((pwm_command >= MOTOR_SPEED_COMMAND_MAX &&
               integrator_step > 0.0f) ||
              (pwm_command <= MOTOR_SPEED_COMMAND_MIN &&
               integrator_step < 0.0f))) {
            state->integrator += integrator_step;
            state->integrator = motor_speed_limit(
                state->integrator,
                MOTOR_SPEED_COMMAND_MIN,
                MOTOR_SPEED_COMMAND_MAX);
        }

        state->filter_state += filter_output *
                               MOTOR_SPEED_CONTROL_PERIOD_S;
        state->pwm_command = motor_speed_limit(
            pwm_command,
            MOTOR_SPEED_COMMAND_MIN,
            MOTOR_SPEED_COMMAND_MAX);
        motor_set_speed(Motor_Config[i], state->pwm_command);
    }
}

const motor_speed_control_state *motor_speed_control_get_state(
    uint32_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return NULL;
    }

    return &s_motor_speed_state[motor_index];
}

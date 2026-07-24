#include "motor_speed_control.h"

#include "simulink_pid.h"
#include "usart.h"

#include <stdio.h>

#define SPEED_REPORT_SCALE 100.0f
#define SPEED_REPORT_BUFFER_SIZE 48U

#if MOTOR_SPEED_CONTROL_INDEX >= MOTOR_COUNT
#error "MOTOR_SPEED_CONTROL_INDEX must be smaller than MOTOR_COUNT"
#endif

static float s_target_rpm;
static float s_current_rpm;
static float s_ramp_direction = 1.0f;
static uint32_t s_report_elapsed_ms;
static char s_report_buffer[SPEED_REPORT_BUFFER_SIZE];

static int32_t speed_to_centi_rpm(float speed_rpm) {
    float scaled = speed_rpm * SPEED_REPORT_SCALE;

    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }

    return (int32_t)(scaled - 0.5f);
}

static uint32_t speed_abs_centi_rpm(int32_t speed_centi_rpm) {
    if (speed_centi_rpm < 0) {
        return (uint32_t)(-(int64_t)speed_centi_rpm);
    }

    return (uint32_t)speed_centi_rpm;
}

static void motor_speed_update_target(void) {
    float step_rpm = MOTOR_SPEED_TARGET_RAMP_RPM_PER_S *
                     MOTOR_SPEED_CONTROL_PERIOD_S;

    s_target_rpm += s_ramp_direction * step_rpm;

    if (s_target_rpm >= MOTOR_SPEED_TARGET_MAX_RPM) {
        s_target_rpm = MOTOR_SPEED_TARGET_MAX_RPM;
        s_ramp_direction = -1.0f;
    } else if (s_target_rpm <= -MOTOR_SPEED_TARGET_MAX_RPM) {
        s_target_rpm = -MOTOR_SPEED_TARGET_MAX_RPM;
        s_ramp_direction = 1.0f;
    }
}

static void motor_speed_send_report(void) {
    int32_t target_centi_rpm;
    int32_t current_centi_rpm;
    int32_t encoder_total_count;
    uint32_t target_abs;
    uint32_t current_abs;
    int written;

    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
        return;
    }

    target_centi_rpm = speed_to_centi_rpm(s_target_rpm);
    current_centi_rpm = speed_to_centi_rpm(s_current_rpm);
    encoder_total_count = Encoder_Config[MOTOR_SPEED_CONTROL_INDEX].total_count;
    target_abs = speed_abs_centi_rpm(target_centi_rpm);
    current_abs = speed_abs_centi_rpm(current_centi_rpm);

    written = snprintf(s_report_buffer,
                       sizeof(s_report_buffer),
                       "%s%lu.%02lu,%s%lu.%02lu,%ld\n",
                       target_centi_rpm < 0 ? "-" : "",
                       (unsigned long)(target_abs / 100U),
                       (unsigned long)(target_abs % 100U),
                       current_centi_rpm < 0 ? "-" : "",
                       (unsigned long)(current_abs / 100U),
                       (unsigned long)(current_abs % 100U),
                       (long)encoder_total_count);
    if (written <= 0 || (size_t)written >= sizeof(s_report_buffer)) {
        return;
    }

    (void)HAL_UART_Transmit_IT(&huart1,
                               (uint8_t *)s_report_buffer,
                               (uint16_t)written);
}

void motor_speed_control_init(void) {
    s_target_rpm = 0.0f;
    s_current_rpm = 0.0f;
    s_ramp_direction = 1.0f;
    s_report_elapsed_ms = 0U;

    encoder_clear_count(&Encoder_Config[MOTOR_SPEED_CONTROL_INDEX]);
    simulink_pid_initialize();
    motor_stop(Motor_Config[MOTOR_SPEED_CONTROL_INDEX]);
}

void motor_speed_control_process(void) {
    const encoder_motion_data *motion;
    float speed_error_rpm;
    float pwm_command;
    real_T integrator_before_step;

    motor_speed_update_target();
    encoder_update_motion(MOTOR_SPEED_CONTROL_PERIOD_S);

    motion = encoder_get_motion_data(MOTOR_SPEED_CONTROL_INDEX);
    if (motion == NULL) {
        motor_stop(Motor_Config[MOTOR_SPEED_CONTROL_INDEX]);
        return;
    }

    s_current_rpm = motion->rotational_speed_rpm;
    speed_error_rpm = s_target_rpm - s_current_rpm;

    /* 模型内部误差为 700-rpm_now，此处令其等价于 target-current。 */
    simulink_pid_U.rpm_now = SIMULINK_PID_MODEL_REFERENCE_RPM -
                             (real_T)speed_error_rpm;
    integrator_before_step = simulink_pid_X.Integrator_CSTATE;
    simulink_pid_step();

    pwm_command = SIMULINK_PID_MOTOR_OUTPUT_SIGN *
                  (float)simulink_pid_Y.pwm_ctrl;

    // 输出饱和且误差仍推动积分增大时，撤销本周期积分，抑制换向时的积分饱和。
    if ((pwm_command > MOTOR_SPEED_COMMAND_MAX && speed_error_rpm > 0.0f) ||
        (pwm_command < MOTOR_SPEED_COMMAND_MIN && speed_error_rpm < 0.0f)) {
        simulink_pid_X.Integrator_CSTATE = integrator_before_step;
    }

    motor_set_speed(Motor_Config[MOTOR_SPEED_CONTROL_INDEX], pwm_command);

    s_report_elapsed_ms += MOTOR_SPEED_CONTROL_PERIOD_MS;
    if (s_report_elapsed_ms >= MOTOR_SPEED_REPORT_PERIOD_MS) {
        s_report_elapsed_ms = 0U;
        motor_speed_send_report();
    }
}

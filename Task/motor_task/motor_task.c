#include "motor_task.h"

#include "Encoder.h"
#include "cmsis_os.h"
#include "usart.h"

#include <stdio.h>

#define MOTOR_DEBUG_STATUS_BUFFER_SIZE 96U
#define MOTOR_DEBUG_TICK_TO_SECONDS 0.001f

static osThreadId_t s_motor_debug_task_handle;
static volatile int16_t s_manual_pwm_command;
static char s_status_buffer[MOTOR_DEBUG_STATUS_BUFFER_SIZE];

#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_STEP
static float s_current_duty_percent = MOTOR_DEBUG_MIN_DUTY_PERCENT;
static int8_t s_step_direction = 1;
#endif

#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_MANUAL_PWM
static int8_t s_manual_pwm_sign;
#endif

static const osThreadAttr_t s_motor_debug_task_attributes = {
    .name = "motorDebugTask",
    .stack_size = MOTOR_DEBUG_TASK_STACK_SIZE_BYTES,
    .priority = (osPriority_t)osPriorityNormal,
};

static void motor_debug_send_text(int written) {
    if (written <= 0 || (size_t)written >= sizeof(s_status_buffer)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)s_status_buffer,
                            (uint16_t)written,
                            MOTOR_DEBUG_UART_TIMEOUT_MS);
}

#if MOTOR_DEBUG_MODE != MOTOR_DEBUG_MODE_ENCODER_COUNT
static Motor_Direction motor_debug_get_forward_direction(void) {
    if (MOTOR_DEBUG_MOTOR_INDEX == 0U) {
        return MOTOR_DEBUG_MOTOR_1_DIRECTION;
    }

    return MOTOR_DEBUG_MOTOR_2_DIRECTION;
}
#endif

#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_STEP
static void motor_debug_apply_duty(float duty_percent) {
    motor_set_duty_percent(Motor_Config[MOTOR_DEBUG_MOTOR_INDEX],
                           duty_percent);
}

static void motor_debug_update_step(void) {
    float next_duty = s_current_duty_percent +
                      (float)s_step_direction * MOTOR_DEBUG_DUTY_STEP_PERCENT;

    if (next_duty >= MOTOR_DEBUG_MAX_DUTY_PERCENT) {
        next_duty = MOTOR_DEBUG_MAX_DUTY_PERCENT;
        s_step_direction = -1;
    } else if (next_duty <= MOTOR_DEBUG_MIN_DUTY_PERCENT) {
        next_duty = MOTOR_DEBUG_MIN_DUTY_PERCENT;
        s_step_direction = 1;
    }

    s_current_duty_percent = next_duty;
    motor_debug_apply_duty(s_current_duty_percent);
}

static void motor_debug_send_step_report(void) {
    int32_t duty_tenths = (int32_t)(s_current_duty_percent * 10.0f + 0.5f);
    int written = snprintf(s_status_buffer, sizeof(s_status_buffer),
                           "Motor%lu: Duty=%ld.%01ld%%, trend=%s\r\n",
                           (unsigned long)MOTOR_DEBUG_MOTOR_NUMBER,
                           (long)(duty_tenths / 10),
                           (long)(duty_tenths % 10),
                           s_step_direction > 0 ? "UP" : "DOWN");

    motor_debug_send_text(written);
    (void)encoder_send_motor_motion_report(&huart1,
                                           MOTOR_DEBUG_MOTOR_INDEX);
}

static void motor_debug_run_step_mode(void) {
    uint32_t next_wake = osKernelGetTickCount();
    uint32_t last_sample = HAL_GetTick();
    uint32_t last_report = last_sample;
    uint32_t last_step = last_sample;

    set_direction(Motor_Config[MOTOR_DEBUG_MOTOR_INDEX],
                  motor_debug_get_forward_direction());
    motor_debug_apply_duty(s_current_duty_percent);

    for (;;) {
        uint32_t now;
        uint32_t elapsed_ms;

        next_wake += MOTOR_DEBUG_CONTROL_PERIOD_MS;
        (void)osDelayUntil(next_wake);
        now = HAL_GetTick();
        elapsed_ms = now - last_sample;
        if (elapsed_ms > 0U) {
            encoder_update_motion((float)elapsed_ms *
                                  MOTOR_DEBUG_TICK_TO_SECONDS);
            last_sample = now;
        }
        if ((now - last_step) >= MOTOR_DEBUG_STEP_PERIOD_MS) {
            last_step = now;
            motor_debug_update_step();
        }
        if ((now - last_report) >= MOTOR_DEBUG_REPORT_PERIOD_MS) {
            last_report = now;
            motor_debug_send_step_report();
        }
    }
}
#endif

#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_ENCODER_COUNT
static void motor_debug_run_count_mode(void) {
    uint32_t next_wake = osKernelGetTickCount();
    uint32_t last_report = HAL_GetTick();
    int32_t total_count = 0;

    for (;;) {
        uint32_t now;

        next_wake += MOTOR_DEBUG_COUNT_SAMPLE_PERIOD_MS;
        (void)osDelayUntil(next_wake);
        total_count = encoder_get_total_count(
            &Encoder_Config[MOTOR_DEBUG_MOTOR_INDEX]);
        now = HAL_GetTick();
        if ((now - last_report) >= MOTOR_DEBUG_COUNT_REPORT_PERIOD_MS) {
            int written;

            last_report = now;
            written = snprintf(s_status_buffer, sizeof(s_status_buffer),
                               "COUNT=%ld\r\n", (long)total_count);
            motor_debug_send_text(written);
        }
    }
}
#endif

#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_MANUAL_PWM
static int32_t motor_debug_round_rpm(float rpm) {
    if (rpm >= 0.0f) {
        return (int32_t)(rpm + 0.5f);
    }

    return (int32_t)(rpm - 0.5f);
}

static void motor_debug_apply_signed_pwm(int16_t pwm_command) {
    motor_config motor = Motor_Config[MOTOR_DEBUG_MOTOR_INDEX];
    int8_t sign = pwm_command > 0 ? 1 : (pwm_command < 0 ? -1 : 0);
    float magnitude;

    if (sign == 0) {
        motor_stop(motor);
        s_manual_pwm_sign = 0;
        return;
    }
    if (sign != s_manual_pwm_sign) {
        motor_set_speed(motor, 0.0f);
        set_direction(motor,
                      sign > 0 ? motor_debug_get_forward_direction() :
                                 (motor_debug_get_forward_direction() == CW ?
                                      CCW : CW));
        s_manual_pwm_sign = sign;
    }

    magnitude = pwm_command > 0 ? (float)pwm_command :
                                  (float)(-(int32_t)pwm_command);
    motor_set_speed(motor, magnitude);
}

static void motor_debug_send_manual_report(int16_t pwm_command) {
    const encoder_motion_data *motion =
        encoder_get_motion_data(MOTOR_DEBUG_MOTOR_INDEX);
    int32_t rpm = motion == NULL ? 0 :
                  motor_debug_round_rpm(motion->rotational_speed_rpm);
    int written = snprintf(s_status_buffer, sizeof(s_status_buffer),
                           "PWM=%d RPM=%ldRPM\r\n",
                           (int)pwm_command, (long)rpm);

    motor_debug_send_text(written);
}

static void motor_debug_run_manual_mode(void) {
    uint32_t next_wake = osKernelGetTickCount();
    uint32_t last_sample = HAL_GetTick();

    for (;;) {
        uint32_t now;
        uint32_t elapsed_ms;
        int16_t pwm_command;

        next_wake += MOTOR_DEBUG_MANUAL_PERIOD_MS;
        (void)osDelayUntil(next_wake);
        pwm_command = s_manual_pwm_command;
        motor_debug_apply_signed_pwm(pwm_command);
        now = HAL_GetTick();
        elapsed_ms = now - last_sample;
        if (elapsed_ms > 0U) {
            encoder_update_motion((float)elapsed_ms *
                                  MOTOR_DEBUG_TICK_TO_SECONDS);
            last_sample = now;
        }
        motor_debug_send_manual_report(pwm_command);
    }
}
#endif

static void motor_debug_task(void *argument) {
    (void)argument;

#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_STEP
    motor_debug_run_step_mode();
#elif MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_ENCODER_COUNT
    motor_debug_run_count_mode();
#else
    motor_debug_run_manual_mode();
#endif
}

void motor_debug_set_pwm(int16_t pwm_command) {
    if (pwm_command < MOTOR_DEBUG_PWM_COMMAND_MIN) {
        pwm_command = MOTOR_DEBUG_PWM_COMMAND_MIN;
    } else if (pwm_command > MOTOR_DEBUG_PWM_COMMAND_MAX) {
        pwm_command = MOTOR_DEBUG_PWM_COMMAND_MAX;
    }

    s_manual_pwm_command = pwm_command;
}

bool motor_debug_start(void) {
    if (s_motor_debug_task_handle != NULL) {
        return true;
    }

#if MOTOR_DEBUG_MODE != MOTOR_DEBUG_MODE_ENCODER_COUNT
    motor_init();
#endif
    encoder_init();
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_STEP
    s_current_duty_percent = MOTOR_DEBUG_MIN_DUTY_PERCENT;
    s_step_direction = 1;
#elif MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_MANUAL_PWM
    s_manual_pwm_command = 0;
    s_manual_pwm_sign = 0;
#endif
    s_motor_debug_task_handle =
        osThreadNew(motor_debug_task, NULL, &s_motor_debug_task_attributes);
    return s_motor_debug_task_handle != NULL;
}

//
// Created by m1816 on 26-7-14.
//

#include "Encoder.h"

#include <stdio.h>

#define ENCODER_PI_F 3.14159265358979323846f
#define ENCODER_MILLI_SCALE 1000.0f
#define ENCODER_REPORT_BUFFER_SIZE 192U
#define ENCODER_COUNTER_REPORT_BUFFER_SIZE 64U

// 未提供板级强配置时，驱动保持安全空配置。
__weak encoder_config Encoder_Config[encoder_count] = {0};

encoder_motion_data Encoder_Motion[encoder_count] = {0};

static uint32_t s_last_report_tick;
static uint32_t s_last_counter_report_tick;
static char s_report_buffer[ENCODER_REPORT_BUFFER_SIZE];
static char s_counter_report_buffer[ENCODER_COUNTER_REPORT_BUFFER_SIZE];

static int32_t encoder_float_to_milli(float value) {
    float scaled = value * ENCODER_MILLI_SCALE;

    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }

    return (int32_t)(scaled - 0.5f);
}

static uint32_t encoder_abs_milli(int32_t value) {
    if (value < 0) {
        return (uint32_t)(-(int64_t)value);
    }

    return (uint32_t)value;
}

static int encoder_format_report_line(char *buffer,
                                      size_t buffer_size,
                                      uint32_t motor_index,
                                      const encoder_motion_data *motion) {
    int32_t rpm_milli = encoder_float_to_milli(motion->rotational_speed_rpm);
    int32_t speed_milli = encoder_float_to_milli(motion->linear_speed_m_s);
    uint32_t rpm_abs = encoder_abs_milli(rpm_milli);
    uint32_t speed_abs = encoder_abs_milli(speed_milli);

    return snprintf(buffer, buffer_size,
                    "Motor%lu: RPM=%s%lu.%03lu rpm, speed=%s%lu.%03lu m/s\r\n",
                    (unsigned long)(motor_index + 1U),
                    rpm_milli < 0 ? "-" : "",
                    (unsigned long)(rpm_abs / 1000U),
                    (unsigned long)(rpm_abs % 1000U),
                    speed_milli < 0 ? "-" : "",
                    (unsigned long)(speed_abs / 1000U),
                    (unsigned long)(speed_abs % 1000U));
}

static int32_t encoder_find_index(const encoder_config *config) {
    uint32_t i;

    for (i = 0; i < encoder_count; i++) {
        if (config == &Encoder_Config[i]) {
            return (int32_t)i;
        }
    }

    return -1;
}

void encoder_init(void) {
    uint32_t i;

    for (i = 0; i < encoder_count; i++) {
        Encoder_Motion[i] = (encoder_motion_data){0};
        if (Encoder_Config[i].htim == NULL) {
            continue;
        }

        __HAL_TIM_SET_COUNTER(Encoder_Config[i].htim, 0);
        Encoder_Config[i].total_count = 0;
        Encoder_Config[i].last_count = 0;
        HAL_TIM_Encoder_Start(Encoder_Config[i].htim,
                              TIM_CHANNEL_1 | TIM_CHANNEL_2);
    }

    s_last_report_tick = HAL_GetTick();
    s_last_counter_report_tick = s_last_report_tick;
}

int16_t encoder_get_delta_count(encoder_config *config) {
    uint16_t current_count;
    int16_t delta_count;

    if (config == NULL || config->htim == NULL) {
        return 0;
    }

    current_count = (uint16_t)__HAL_TIM_GET_COUNTER(config->htim);
    delta_count = (int16_t)(current_count - config->last_count);

    config->last_count = current_count;
    config->total_count += delta_count;

    return delta_count;
}

int32_t encoder_get_total_count(encoder_config *config) {
    if (config == NULL) {
        return 0;
    }

    encoder_get_delta_count(config);
    return config->total_count;
}

uint16_t encoder_get_now_counter(encoder_config config) {
    if (config.htim == NULL) {
        return 0;
    }

    return (uint16_t)__HAL_TIM_GET_COUNTER(config.htim);
}

void encoder_clear_count(encoder_config *config) {
    int32_t index;

    if (config == NULL || config->htim == NULL) {
        return;
    }

    __HAL_TIM_SET_COUNTER(config->htim, 0);
    config->total_count = 0;
    config->last_count = 0;

    index = encoder_find_index(config);
    if (index >= 0) {
        Encoder_Motion[index] = (encoder_motion_data){0};
    }
}

void encoder_update_motion(float sample_period_s) {
    uint32_t i;
    float counts_per_revolution = ENCODER_COUNTS_PER_WHEEL_REV;

    if (sample_period_s <= 0.0f || counts_per_revolution <= 0.0f) {
        return;
    }

    for (i = 0; i < encoder_count; i++) {
        int16_t delta_count = encoder_get_delta_count(&Encoder_Config[i]);

        Encoder_Motion[i].rotational_speed_rpm =
            (float)delta_count * 60.0f /
            (counts_per_revolution * sample_period_s);
        Encoder_Motion[i].linear_speed_m_s =
            Encoder_Motion[i].rotational_speed_rpm * ENCODER_PI_F *
            ENCODER_WHEEL_DIAMETER_M / 60.0f;
        Encoder_Motion[i].distance_m =
            (float)Encoder_Config[i].total_count * ENCODER_PI_F *
            ENCODER_WHEEL_DIAMETER_M / counts_per_revolution;
    }
}

const encoder_motion_data *encoder_get_motion_data(uint32_t motor_index) {
    if (motor_index >= encoder_count) {
        return NULL;
    }

    return &Encoder_Motion[motor_index];
}

HAL_StatusTypeDef encoder_send_motion_report(UART_HandleTypeDef *huart) {
    uint32_t i;
    size_t used = 0U;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    for (i = 0; i < encoder_count; i++) {
        size_t remaining = sizeof(s_report_buffer) - used;
        int written = encoder_format_report_line(&s_report_buffer[used],
                                                 remaining,
                                                 i,
                                                 &Encoder_Motion[i]);
        if (written < 0 || (size_t)written >= remaining) {
            return HAL_ERROR;
        }
        used += (size_t)written;
    }

    return HAL_UART_Transmit(huart,
                             (uint8_t *)s_report_buffer,
                             (uint16_t)used,
                             ENCODER_UART_TIMEOUT_MS);
}

void
encoder_motion_report_process(void) {
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms = current_tick - s_last_report_tick;

    if (elapsed_ms < ENCODER_REPORT_PERIOD_MS) {
        return;
    }

    s_last_report_tick = current_tick;
    encoder_update_motion((float)elapsed_ms / 1000.0f);
    (void)encoder_send_motion_report(ENCODER_REPORT_UART_HANDLE);
}


HAL_StatusTypeDef encoder_send_counter_report(UART_HandleTypeDef *huart) {
    uint16_t tim2_count;
    uint16_t tim4_count;
    int written;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    tim2_count = encoder_get_now_counter(Encoder_Config[0]);
    tim4_count = encoder_get_now_counter(Encoder_Config[1]);
    written = snprintf(s_counter_report_buffer,
                       sizeof(s_counter_report_buffer),
                       "TIM2_CNT=%lu, TIM4_CNT=%lu\r\n",
                       (unsigned long)tim2_count,
                       (unsigned long)tim4_count);
    if (written < 0 || (size_t)written >= sizeof(s_counter_report_buffer)) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart,
                             (uint8_t *)s_counter_report_buffer,
                             (uint16_t)written,
                             ENCODER_UART_TIMEOUT_MS);
}

void encoder_counter_report_process(void) {
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms = current_tick - s_last_counter_report_tick;

    if (elapsed_ms < ENCODER_COUNTER_REPORT_PERIOD_MS) {
        return;
    }

    s_last_counter_report_tick = current_tick;
    (void)encoder_send_counter_report(ENCODER_REPORT_UART_HANDLE);
}

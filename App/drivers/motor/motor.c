//
// Created by m1816 on 26-7-24.
//

#include "motor.h"

#include <stdio.h>

/*
 * ========================== 电机硬件配置区 ==========================
 *
 * 电机 1：
 *   PWM      -> TIM9_CH1 / PE5  / PWMA1
 *   TB6612   -> AIN1: PA11，AIN2: PA10
 *   编码器 A -> TIM2_CH1 / PA15 / H1A
 *   编码器 B -> TIM2_CH2 / PB3  / H1B
 *   正速度   -> CCW
 *
 * 电机 2：
 *   PWM      -> TIM9_CH2 / PE6  / PWMB1
 *   TB6612   -> BIN1: PA9，BIN2: PA8
 *   编码器 A -> TIM4_CH1 / PD12 / H2A
 *   编码器 B -> TIM4_CH2 / PD13 / H2B
 *   正速度   -> CW
 *
 * CubeMX 必须保持以下配置：
 *   TIM9  -> PWM Generation CH1/CH2，ARR = 9999
 *   TIM2  -> Encoder Mode TI12，ARR = 65535
 *   TIM4  -> Encoder Mode TI12，ARR = 65535
 *   AIN1/AIN2/BIN1/BIN2 -> GPIO Output，默认低电平
 *
 * 更换端口时，先在 CubeMX 中修改并重新生成工程，再修改下面两个数组。
 * ====================================================================
 */

#define ENCODER_PI_F 3.14159265358979323846f
#define ENCODER_MILLI_SCALE 1000.0f
#define ENCODER_REPORT_BUFFER_SIZE 256U
#define ENCODER_COUNTER_REPORT_BUFFER_SIZE 64U

motor_config Motor_Config[MOTOR_COUNT] = {
    {
        .htim = &htim9,
        .channel = TIM_CHANNEL_1,
        .in1_port = AIN1_GPIO_Port,
        .in1_pin = AIN1_Pin,
        .in2_port = AIN2_GPIO_Port,
        .in2_pin = AIN2_Pin,
        .positive_direction = CCW,
    },
    {
        .htim = &htim9,
        .channel = TIM_CHANNEL_2,
        .in1_port = BIN1_GPIO_Port,
        .in1_pin = BIN1_Pin,
        .in2_port = BIN2_GPIO_Port,
        .in2_pin = BIN2_Pin,
        .positive_direction = CW,
    },
};

encoder_config Encoder_Config[MOTOR_COUNT] = {
    {
        .htim = &htim2,
        .total_count = 0,
        .last_count = 0,
    },
    {
        .htim = &htim4,
        .total_count = 0,
        .last_count = 0,
    },
};

encoder_motion_data Encoder_Motion[MOTOR_COUNT] = {0};

static uint32_t s_last_report_tick;
static uint32_t s_last_counter_report_tick;
static char s_report_buffer[ENCODER_REPORT_BUFFER_SIZE];
static char s_counter_report_buffer[ENCODER_COUNTER_REPORT_BUFFER_SIZE];

static uint8_t motor_config_is_valid(motor_config config) {
    return config.htim != NULL &&
           config.in1_port != NULL &&
           config.in2_port != NULL;
}

static void motor_write_pwm(motor_config config, float speed_magnitude) {
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(config.htim) + 1U;
    uint32_t compare = (uint32_t)(speed_magnitude * (float)period /
                                  MOTOR_SPEED_COMMAND_MAX + 0.5f);

    __HAL_TIM_SET_COMPARE(config.htim, config.channel, compare);
}

static void motor_set_direction(motor_config config,
                                Motor_Direction direction) {
    switch (direction) {
        case CW:
            HAL_GPIO_WritePin(config.in1_port, config.in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config.in2_port, config.in2_pin, GPIO_PIN_RESET);
            break;
        case CCW:
            HAL_GPIO_WritePin(config.in1_port, config.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config.in2_port, config.in2_pin, GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(config.in1_port, config.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config.in2_port, config.in2_pin, GPIO_PIN_RESET);
            break;
    }
}

static Motor_Direction motor_get_reverse_direction(Motor_Direction direction) {
    if (direction == CW) {
        return CCW;
    }

    return CW;
}

static int32_t encoder_find_index(const encoder_config *config) {
    uint32_t i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        if (config == &Encoder_Config[i]) {
            return (int32_t)i;
        }
    }

    return -1;
}

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
    int32_t distance_milli = encoder_float_to_milli(motion->distance_m);
    uint32_t rpm_abs = encoder_abs_milli(rpm_milli);
    uint32_t speed_abs = encoder_abs_milli(speed_milli);
    uint32_t distance_abs = encoder_abs_milli(distance_milli);

    return snprintf(buffer, buffer_size,
                    "Motor%lu: RPM=%s%lu.%03lu rpm, speed=%s%lu.%03lu m/s, "
                    "distance=%s%lu.%03lu m\r\n",
                    (unsigned long)(motor_index + 1U),
                    rpm_milli < 0 ? "-" : "",
                    (unsigned long)(rpm_abs / 1000U),
                    (unsigned long)(rpm_abs % 1000U),
                    speed_milli < 0 ? "-" : "",
                    (unsigned long)(speed_abs / 1000U),
                    (unsigned long)(speed_abs % 1000U),
                    distance_milli < 0 ? "-" : "",
                    (unsigned long)(distance_abs / 1000U),
                    (unsigned long)(distance_abs % 1000U));
}

void motor_init(void) {
    uint32_t i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        if (!motor_config_is_valid(Motor_Config[i])) {
            continue;
        }

        HAL_TIM_PWM_Start(Motor_Config[i].htim, Motor_Config[i].channel);
        motor_stop(Motor_Config[i]);
    }

    encoder_init();
}

void motor_set_speed(motor_config config, float speed) {
    float speed_magnitude;
    Motor_Direction direction;

    if (!motor_config_is_valid(config)) {
        return;
    }

    if (speed < MOTOR_SPEED_COMMAND_MIN) {
        speed = MOTOR_SPEED_COMMAND_MIN;
    } else if (speed > MOTOR_SPEED_COMMAND_MAX) {
        speed = MOTOR_SPEED_COMMAND_MAX;
    }

    if (speed == 0.0f) {
        motor_stop(config);
        return;
    }

    if (speed > 0.0f) {
        speed_magnitude = speed;
        direction = config.positive_direction;
    } else {
        speed_magnitude = -speed;
        direction = motor_get_reverse_direction(config.positive_direction);
    }

    // 改变 H 桥方向前先关闭 PWM，避免带负载直接换向。
    motor_write_pwm(config, 0.0f);
    motor_set_direction(config, direction);
    motor_write_pwm(config, speed_magnitude);
}

void motor_stop(motor_config config) {
    if (!motor_config_is_valid(config)) {
        return;
    }

    motor_write_pwm(config, 0.0f);
    HAL_GPIO_WritePin(config.in1_port, config.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config.in2_port, config.in2_pin, GPIO_PIN_RESET);
}

void motor_brake(motor_config config) {
    if (!motor_config_is_valid(config)) {
        return;
    }

    motor_write_pwm(config, 0.0f);
    HAL_GPIO_WritePin(config.in1_port, config.in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(config.in2_port, config.in2_pin, GPIO_PIN_SET);
}

void encoder_init(void) {
    uint32_t i;

    for (i = 0; i < MOTOR_COUNT; i++) {
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

    for (i = 0; i < MOTOR_COUNT; i++) {
        int16_t delta_count = encoder_get_delta_count(&Encoder_Config[i]);
        float raw_speed_rpm = (float)delta_count * 60.0f /
                              (counts_per_revolution * sample_period_s);

        Encoder_Motion[i].rotational_speed_rpm = raw_speed_rpm;
        Encoder_Motion[i].linear_speed_m_s =
            raw_speed_rpm * ENCODER_PI_F * ENCODER_WHEEL_DIAMETER_M / 60.0f;
        Encoder_Motion[i].distance_m =
            (float)Encoder_Config[i].total_count * ENCODER_PI_F *
            ENCODER_WHEEL_DIAMETER_M / counts_per_revolution;
    }
}

const encoder_motion_data *encoder_get_motion_data(uint32_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return NULL;
    }

    return &Encoder_Motion[motor_index];
}

HAL_StatusTypeDef encoder_send_motor_motion_report(UART_HandleTypeDef *huart,
                                                   uint32_t motor_index) {
    int written;

    if (huart == NULL || motor_index >= MOTOR_COUNT) {
        return HAL_ERROR;
    }

    written = encoder_format_report_line(s_report_buffer,
                                         sizeof(s_report_buffer),
                                         motor_index,
                                         &Encoder_Motion[motor_index]);
    if (written < 0 || (size_t)written >= sizeof(s_report_buffer)) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart,
                             (uint8_t *)s_report_buffer,
                             (uint16_t)written,
                             ENCODER_UART_TIMEOUT_MS);
}

HAL_StatusTypeDef encoder_send_motion_report(UART_HandleTypeDef *huart) {
    uint32_t i;
    size_t used = 0U;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    for (i = 0; i < MOTOR_COUNT; i++) {
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

void encoder_motion_report_process(void) {
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

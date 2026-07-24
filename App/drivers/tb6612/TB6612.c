//
// Created by m1816 on 26-7-10.
//

#include "TB6612.h"
#include "main.h"

//电机的配置结构体
//根据电机的数量，分别配置每个电机的定时器，通道，两个GPIO端口和引脚；
//若需修改电机配置，直接修改该结构体，调用电机配置，也可直接调用该结构体；
__weak motor_config Motor_Config[motor_count] = {0};

static uint8_t motor_config_is_valid(motor_config config) {
    return config.htim != NULL &&
           config.in1_port != NULL &&
           config.in2_port != NULL;
}

void motor_init(void) {
    uint32_t i;

    for (i = 0; i < motor_count; i++) {
        if (!motor_config_is_valid(Motor_Config[i])) {
            continue;
        }

        HAL_TIM_PWM_Start(Motor_Config[i].htim,Motor_Config[i].channel);
        motor_stop(Motor_Config[i]);
    }
}

void motor_set_speed(motor_config Motor_Config,float speed) {
    uint32_t compare;
    if (!motor_config_is_valid(Motor_Config)) {
        return;
    }

    if (speed < MOTOR_SPEED_COMMAND_MIN) {
        speed = MOTOR_SPEED_COMMAND_MIN;
    } else if (speed > MOTOR_SPEED_COMMAND_MAX) {
        speed = MOTOR_SPEED_COMMAND_MAX;
    }
    compare = (uint32_t)(speed * 10.0f);
    __HAL_TIM_SET_COMPARE(Motor_Config.htim,Motor_Config.channel,compare);
}

void motor_set_duty_percent(motor_config Motor_Config, float duty_percent) {
    uint32_t period;
    uint32_t compare;

    if (!motor_config_is_valid(Motor_Config)) {
        return;
    }
    if (duty_percent < MOTOR_DUTY_PERCENT_MIN) {
        duty_percent = MOTOR_DUTY_PERCENT_MIN;
    } else if (duty_percent > MOTOR_DUTY_PERCENT_MAX) {
        duty_percent = MOTOR_DUTY_PERCENT_MAX;
    }

    period = __HAL_TIM_GET_AUTORELOAD(Motor_Config.htim) + 1U;
    compare = (uint32_t)(duty_percent * (float)period / 100.0f + 0.5f);
    __HAL_TIM_SET_COMPARE(Motor_Config.htim, Motor_Config.channel, compare);
}

void set_direction(motor_config Motor_Config,Motor_Direction direction) {
    if (!motor_config_is_valid(Motor_Config)) {
        return;
    }

    switch(direction) {
        case CW:
            HAL_GPIO_WritePin(Motor_Config.in1_port, Motor_Config.in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Motor_Config.in2_port, Motor_Config.in2_pin, GPIO_PIN_RESET);
            break;
        case CCW:
            HAL_GPIO_WritePin(Motor_Config.in1_port, Motor_Config.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_Config.in2_port, Motor_Config.in2_pin, GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(Motor_Config.in1_port, Motor_Config.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor_Config.in2_port, Motor_Config.in2_pin, GPIO_PIN_RESET);
            break;
    }
}

void motor_stop(motor_config Motor_Config) {
    if (!motor_config_is_valid(Motor_Config)) {
        return;
    }

    motor_set_speed(Motor_Config,0.0f);
    HAL_GPIO_WritePin(Motor_Config.in1_port, Motor_Config.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_Config.in2_port, Motor_Config.in2_pin, GPIO_PIN_RESET);
}

void motor_brake(motor_config Motor_Config) {
    if (!motor_config_is_valid(Motor_Config)) {
        return;
    }

    motor_set_speed(Motor_Config,0.0f);
    HAL_GPIO_WritePin(Motor_Config.in1_port, Motor_Config.in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_Config.in2_port, Motor_Config.in2_pin, GPIO_PIN_SET);
}

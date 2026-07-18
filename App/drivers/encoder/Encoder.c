//
// Created by m1816 on 26-7-14.
//

#include "Encoder.h"

//编码器的配置结构体
//根据编码器数量，分别配置每个编码器对应的定时器；
//若需新增编码器，直接扩展该结构体数组即可；
__weak encoder_config Encoder_Config[encoder_count] = {0};

void encoder_init(void) {
    uint32_t i;

    for (i = 0; i < encoder_count; i++) {
        if (Encoder_Config[i].htim == NULL) {
            continue;
        }

        __HAL_TIM_SET_COUNTER(Encoder_Config[i].htim,0);
        Encoder_Config[i].total_count = 0;
        Encoder_Config[i].last_count = 0;
        HAL_TIM_Encoder_Start(Encoder_Config[i].htim,TIM_CHANNEL_1 | TIM_CHANNEL_2);
    }
}

int16_t encoder_get_delta_count(encoder_config *Encoder_Config) {
    uint16_t current_count;
    int16_t delta_count;

    if (Encoder_Config == NULL || Encoder_Config->htim == NULL) {
        return 0;
    }

    current_count = (uint16_t)__HAL_TIM_GET_COUNTER(Encoder_Config->htim);
    delta_count = (int16_t)(current_count - Encoder_Config->last_count);

    Encoder_Config->last_count = current_count;
    Encoder_Config->total_count += delta_count;

    return delta_count;
}

int32_t encoder_get_total_count(encoder_config *Encoder_Config) {
    if (Encoder_Config == NULL) {
        return 0;
    }

    encoder_get_delta_count(Encoder_Config);
    return Encoder_Config->total_count;
}

uint16_t encoder_get_now_counter(encoder_config Encoder_Config) {
    if (Encoder_Config.htim == NULL) {
        return 0;
    }

    return (uint16_t)__HAL_TIM_GET_COUNTER(Encoder_Config.htim);
}

void encoder_clear_count(encoder_config *Encoder_Config) {
    if (Encoder_Config == NULL || Encoder_Config->htim == NULL) {
        return;
    }

    __HAL_TIM_SET_COUNTER(Encoder_Config->htim,0);
    Encoder_Config->total_count = 0;
    Encoder_Config->last_count = 0;
}

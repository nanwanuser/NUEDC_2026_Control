//
// Created by m1816 on 26-7-17.
//

#include "Gray_Sensor.h"

//灰度模块配置结构体
//根据模块数量，分别配置AD0、AD1、AD2和OUT对应的端口与引脚；
//若需修改引脚，直接修改该结构体数组即可；
__weak gray_sensor_config Gray_Sensor_Config[gray_sensor_count] = {0};

static void gray_sensor_write_address(gray_sensor_config Gray_Sensor_Config,uint8_t channel) {
    HAL_GPIO_WritePin(Gray_Sensor_Config.ad0_port,
                      Gray_Sensor_Config.ad0_pin,
                      (channel & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Gray_Sensor_Config.ad1_port,
                      Gray_Sensor_Config.ad1_pin,
                      (channel & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Gray_Sensor_Config.ad2_port,
                      Gray_Sensor_Config.ad2_pin,
                      (channel & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t gray_sensor_pin_state_to_value(gray_sensor_config Gray_Sensor_Config,GPIO_PinState pin_state) {
    if (Gray_Sensor_Config.active_level == GRAY_SENSOR_ACTIVE_HIGH) {
        return (pin_state == GPIO_PIN_SET) ? 1U : 0U;
    }

    return (pin_state == GPIO_PIN_RESET) ? 1U : 0U;
}

static void gray_sensor_wait_data_ready(void) {
    volatile uint32_t i;

    for (i = 0; i < 64U; i++) {
        __NOP();
    }
}

void gray_sensor_init(void) {
    uint32_t i;

    for (i = 0; i < gray_sensor_count; i++) {
        if (Gray_Sensor_Config[i].ad0_port == NULL ||
            Gray_Sensor_Config[i].ad1_port == NULL ||
            Gray_Sensor_Config[i].ad2_port == NULL ||
            Gray_Sensor_Config[i].out_port == NULL) {
            continue;
        }

        gray_sensor_set_channel(Gray_Sensor_Config[i],0);
    }
}

void gray_sensor_set_channel(gray_sensor_config Gray_Sensor_Config,uint8_t channel) {
    if (Gray_Sensor_Config.ad0_port == NULL ||
        Gray_Sensor_Config.ad1_port == NULL ||
        Gray_Sensor_Config.ad2_port == NULL) {
        return;
    }

    if (channel > 7U) {
        channel = 7U;
    }

    gray_sensor_write_address(Gray_Sensor_Config,channel);
    gray_sensor_wait_data_ready();
}

uint8_t gray_sensor_read_channel(gray_sensor_config Gray_Sensor_Config,uint8_t channel) {
    gray_sensor_set_channel(Gray_Sensor_Config,channel);
    return gray_sensor_read_now(Gray_Sensor_Config);
}

uint8_t gray_sensor_read_now(gray_sensor_config Gray_Sensor_Config) {
    GPIO_PinState pin_state;

    if (Gray_Sensor_Config.out_port == NULL) {
        return 0;
    }

    pin_state = HAL_GPIO_ReadPin(Gray_Sensor_Config.out_port,Gray_Sensor_Config.out_pin);
    return gray_sensor_pin_state_to_value(Gray_Sensor_Config,pin_state);
}

void gray_sensor_read_all(gray_sensor_config Gray_Sensor_Config,uint8_t *buffer) {
    uint8_t i;

    if (buffer == NULL) {
        return;
    }

    for (i = 0; i < 8U; i++) {
        buffer[i] = gray_sensor_read_channel(Gray_Sensor_Config,i);
    }
}

uint8_t gray_sensor_read_byte(gray_sensor_config Gray_Sensor_Config) {
    uint8_t i;
    uint8_t sensor_data;

    sensor_data = 0;

    for (i = 0; i < 8U; i++) {
        if (gray_sensor_read_channel(Gray_Sensor_Config,i) != 0U) {
            sensor_data |= (uint8_t)(1U << i);
        }
    }

    return sensor_data;
}

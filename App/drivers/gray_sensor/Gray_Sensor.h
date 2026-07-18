//
// Created by m1816 on 26-7-17.
//

#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "main.h"

//灰度模块数量
#define gray_sensor_count 1U

//OUT口有效电平定义
typedef enum {
    GRAY_SENSOR_ACTIVE_LOW = 0,
    GRAY_SENSOR_ACTIVE_HIGH = 1,
}gray_sensor_active_level_t;

//单个灰度模块配置结构体
typedef struct {
    GPIO_TypeDef *ad0_port;
    uint16_t ad0_pin;
    GPIO_TypeDef *ad1_port;
    uint16_t ad1_pin;
    GPIO_TypeDef *ad2_port;
    uint16_t ad2_pin;
    GPIO_TypeDef *out_port;
    uint16_t out_pin;
    gray_sensor_active_level_t active_level;
}gray_sensor_config;

extern gray_sensor_config Gray_Sensor_Config[gray_sensor_count];

//灰度模块初始化函数
//初始化所有灰度模块地址线为0通道
void gray_sensor_init(void);

//灰度模块通道选择函数
//@Gray_Sensor_Config 灰度模块配置结构体
//@channel 通道号 0~7，超出范围会自动限幅到7
void gray_sensor_set_channel(gray_sensor_config Gray_Sensor_Config,uint8_t channel);

//灰度模块单路读取函数
//@Gray_Sensor_Config 灰度模块配置结构体
//@channel 通道号 0~7
//@return 对应通道的逻辑状态 0或1
uint8_t gray_sensor_read_channel(gray_sensor_config Gray_Sensor_Config,uint8_t channel);

//灰度模块原始OUT口读取函数
//@Gray_Sensor_Config 灰度模块配置结构体
//@return OUT口当前原始电平 0或1
uint8_t gray_sensor_read_now(gray_sensor_config Gray_Sensor_Config);

//灰度模块8路读取函数
//@Gray_Sensor_Config 灰度模块配置结构体
//@buffer 长度至少为8的数组，用于保存8路结果
void gray_sensor_read_all(gray_sensor_config Gray_Sensor_Config,uint8_t *buffer);

//灰度模块8路打包读取函数
//@Gray_Sensor_Config 灰度模块配置结构体
//@return bit0~bit7分别对应0~7通道
uint8_t gray_sensor_read_byte(gray_sensor_config Gray_Sensor_Config);

#endif //GRAY_SENSOR_H

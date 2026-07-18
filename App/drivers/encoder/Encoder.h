//
// Created by m1816 on 26-7-14.
//

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"

//编码器数量
#define encoder_count 1U

//单个编码器配置结构体
typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t total_count;
    uint16_t last_count;
}encoder_config;

extern encoder_config Encoder_Config[encoder_count];

//编码器初始化函数
//初始化所有编码器接口并清零计数
void encoder_init(void);

//编码器增量读取函数
//@Encoder_Config 编码器配置结构体指针
//@return 距离上次读取后的增量计数
int16_t encoder_get_delta_count(encoder_config *Encoder_Config);

//编码器累计计数读取函数
//@Encoder_Config 编码器配置结构体指针
//@return 从清零开始累计的总计数
int32_t encoder_get_total_count(encoder_config *Encoder_Config);

//编码器当前计数器值读取函数
//@Encoder_Config 编码器配置结构体
//@return 当前定时器CNT寄存器值
uint16_t encoder_get_now_counter(encoder_config Encoder_Config);

//编码器清零函数
//@Encoder_Config 编码器配置结构体指针
void encoder_clear_count(encoder_config *Encoder_Config);

#endif //ENCODER_H

//
// Created by m1816 on 26-7-10.
//

#ifndef TB6612_H
#define TB6612_H



//驱动点击数量
#define motor_count 2U

#include "stm32f407xx.h"
//#include "stm32f4xx_hal_tim.h"
#include "tim.h"

//电机运动状态枚举
typedef enum {
    CW = 0,
    CCW = 1,
}Motor_Direction;


//单个点击配置结构体
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
}motor_config;


extern motor_config Motor_Config[motor_count];

//电机初始化函数
//初始化所有电机的PWM输出和默认停止状态
void motor_init(void);

//电机速度设置函数
//@Motor_Config 电机配置结构体
//@speed 设置新的速度 0.0~1000.0，超出范围会自动限幅
//建议ARR配置为9999
void motor_set_speed(motor_config Motor_Config,float speed) ;

//电机方向设置函数
//@Motor_Config 电机配置结构体
//@direction 电机转动方向
void set_direction(motor_config Motor_Config,Motor_Direction direction);

//电机停止函数
//@Motor_Config 电机配置结构体
void motor_stop(motor_config Motor_Config);

//电机刹车函数
//@Motor_Config 电机配置结构体
void motor_brake(motor_config Motor_Config);

#endif //TB8812_H

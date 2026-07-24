//
// Created by m1816 on 26-7-24.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include "usart.h"

// 当前工程使用两路直流电机，每路电机对应一路 PWM 和一路正交编码器。
#define MOTOR_COUNT 2U

// 保留原驱动中的数量宏，避免已有调用代码失效。
#define motor_count MOTOR_COUNT
#define encoder_count MOTOR_COUNT

// motor_set_speed() 使用的命令范围。
#define MOTOR_SPEED_COMMAND_MIN -1000.0f
#define MOTOR_SPEED_COMMAND_MAX 1000.0f

// MG310 + MGR 编码器和车轮参数。
#ifndef ENCODER_PULSES_PER_MOTOR_REV
#define ENCODER_PULSES_PER_MOTOR_REV 500.0f
#endif

#ifndef ENCODER_GEAR_RATIO
#define ENCODER_GEAR_RATIO 20.409f
#endif

#ifndef ENCODER_QUADRATURE_MULTIPLIER
#define ENCODER_QUADRATURE_MULTIPLIER 4.0f
#endif

#ifndef ENCODER_COUNTS_PER_WHEEL_REV
#define ENCODER_COUNTS_PER_WHEEL_REV \
    (ENCODER_PULSES_PER_MOTOR_REV * ENCODER_GEAR_RATIO * \
     ENCODER_QUADRATURE_MULTIPLIER)
#endif

#ifndef ENCODER_WHEEL_DIAMETER_M
#define ENCODER_WHEEL_DIAMETER_M 0.038f
#endif

// 编码器数据上报配置。
#ifndef ENCODER_REPORT_PERIOD_MS
#define ENCODER_REPORT_PERIOD_MS 100U
#endif

#ifndef ENCODER_COUNTER_REPORT_PERIOD_MS
#define ENCODER_COUNTER_REPORT_PERIOD_MS 50U
#endif

#ifndef ENCODER_REPORT_UART_HANDLE
#define ENCODER_REPORT_UART_HANDLE (&huart1)
#endif

#ifndef ENCODER_UART_TIMEOUT_MS
#define ENCODER_UART_TIMEOUT_MS 20U
#endif

// 电机转动方向；实际正反方向与电机接线有关。
typedef enum {
    CW = 0,
    CCW = 1,
} Motor_Direction;

// 单路 TB6612 电机控制配置。
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    Motor_Direction positive_direction;
} motor_config;

// 单路正交编码器配置和累计计数状态。
typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t total_count;
    uint16_t last_count;
} encoder_config;

// 单路电机最近一次计算得到的运动数据。
typedef struct {
    float rotational_speed_rpm;
    float linear_speed_m_s;
    float distance_m;
} encoder_motion_data;

extern motor_config Motor_Config[MOTOR_COUNT];
extern encoder_config Encoder_Config[MOTOR_COUNT];
extern encoder_motion_data Encoder_Motion[MOTOR_COUNT];

/**
 * @brief 初始化全部 PWM 输出和编码器，初始化后电机处于停止状态。
 * @note 调用前必须完成 GPIO、TIM9、TIM2 和 TIM4 的 CubeMX 初始化。
 */
void motor_init(void);

/**
 * @brief 设置单路电机的速度命令。
 * @param config 目标电机配置，一般传入 Motor_Config[index]。
 * @param speed 速度命令，范围为 -1000.0 到 1000.0，超出范围自动限幅。
 *              正负号控制方向，绝对值控制 PWM，0 表示停止。
 */
void motor_set_speed(motor_config config, float speed);

/**
 * @brief 将 PWM 清零并把两个方向引脚拉低，使电机自然停止。
 */
void motor_stop(motor_config config);

/**
 * @brief 将 PWM 清零并把两个方向引脚拉高，使 TB6612 主动刹车。
 */
void motor_brake(motor_config config);

/**
 * @brief 单独初始化全部编码器并清零计数状态。
 * @note motor_init() 已经调用此函数，一般不需要再次调用。
 */
void encoder_init(void);

/**
 * @brief 读取距离上次采样后的编码器增量。
 * @return 有符号增量计数；配置无效时返回 0。
 */
int16_t encoder_get_delta_count(encoder_config *config);

/**
 * @brief 更新并返回编码器累计计数。
 */
int32_t encoder_get_total_count(encoder_config *config);

/**
 * @brief 读取编码器定时器当前原始计数值，不修改累计状态。
 */
uint16_t encoder_get_now_counter(encoder_config config);

/**
 * @brief 清零指定编码器的硬件计数、累计计数和运动数据。
 */
void encoder_clear_count(encoder_config *config);

/**
 * @brief 根据实际采样周期更新全部电机的转速、线速度和距离。
 * @param sample_period_s 实际采样周期，单位为秒，必须大于 0。
 */
void encoder_update_motion(float sample_period_s);

/**
 * @brief 获取指定电机最近一次计算得到的运动数据。
 * @return 只读数据指针；索引无效时返回 NULL。
 */
const encoder_motion_data *encoder_get_motion_data(uint32_t motor_index);

/**
 * @brief 通过指定串口发送单路电机运动数据。
 */
HAL_StatusTypeDef encoder_send_motor_motion_report(UART_HandleTypeDef *huart,
                                                   uint32_t motor_index);

/**
 * @brief 通过指定串口发送全部电机运动数据。
 */
HAL_StatusTypeDef encoder_send_motion_report(UART_HandleTypeDef *huart);

/**
 * @brief 按 ENCODER_REPORT_PERIOD_MS 更新运动数据并通过默认串口发送。
 */
void encoder_motion_report_process(void);

/**
 * @brief 通过指定串口发送两路编码器当前原始计数器值。
 */
HAL_StatusTypeDef encoder_send_counter_report(UART_HandleTypeDef *huart);

/**
 * @brief 按 ENCODER_COUNTER_REPORT_PERIOD_MS 发送原始计数器值。
 */
void encoder_counter_report_process(void);

#endif // MOTOR_H

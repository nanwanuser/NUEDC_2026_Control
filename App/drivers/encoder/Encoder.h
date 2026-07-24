//
// Created by m1816 on 26-7-14.
//

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"
#include "usart.h"

// 编码器数量
#define encoder_count 2U

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

// 单个编码器配置结构体
typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t total_count;
    uint16_t last_count;
} encoder_config;

// 单个电机运动数据
typedef struct {
    float rotational_speed_rpm;
    float linear_speed_m_s;
    float distance_m;
} encoder_motion_data;

extern encoder_config Encoder_Config[encoder_count];
extern encoder_motion_data Encoder_Motion[encoder_count];

/**
 * @brief 初始化全部编码器并清零计数和运动数据。
 */
void encoder_init(void);

/**
 * @brief 读取距离上次采样后的编码器增量。
 * @param config 编码器配置指针。
 * @return 有符号增量计数；配置无效时返回 0。
 */
int16_t encoder_get_delta_count(encoder_config *config);

/**
 * @brief 更新并读取编码器累计计数。
 * @param config 编码器配置指针。
 * @return 从上次清零开始的累计计数。
 */
int32_t encoder_get_total_count(encoder_config *config);

/**
 * @brief 读取编码器定时器当前计数值。
 * @param config 编码器配置。
 * @return 当前 16 位计数值。
 */
uint16_t encoder_get_now_counter(encoder_config config);

/**
 * @brief 清零指定编码器的硬件计数、累计计数和运动数据。
 * @param config 编码器配置指针。
 */
void encoder_clear_count(encoder_config *config);

/**
 * @brief 根据采样周期更新全部电机的转速、线速度和累计距离。
 * @param sample_period_s 实际采样周期，单位为秒，必须大于 0。
 */
void encoder_update_motion(float sample_period_s);

/**
 * @brief 获取指定电机最近一次计算的运动数据。
 * @param motor_index 电机索引，范围为 0 到 encoder_count - 1。
 * @return 运动数据只读指针；索引无效时返回 NULL。
 */
const encoder_motion_data *encoder_get_motion_data(uint32_t motor_index);

/**
 * @brief 将指定电机的运动数据通过指定串口发送。
 * @param huart UART 句柄；传入 NULL 时返回 HAL_ERROR。
 * @param motor_index 电机索引，范围为 0 到 encoder_count - 1。
 * @return HAL 串口发送状态；参数无效时返回 HAL_ERROR。
 */
HAL_StatusTypeDef encoder_send_motor_motion_report(UART_HandleTypeDef *huart,
                                                   uint32_t motor_index);

/**
 * @brief 将两路电机运动数据通过指定串口发送。
 * @param huart UART 句柄；传入 NULL 时返回 HAL_ERROR。
 * @return HAL 串口发送状态。
 */
HAL_StatusTypeDef encoder_send_motion_report(UART_HandleTypeDef *huart);

/**
 * @brief 按 ENCODER_REPORT_PERIOD_MS 周期更新数据并通过默认串口发送。
 * @note 需要在 FreeRTOS 任务或主循环中高频调用。
 */
void encoder_motion_report_process(void);

/**
 * @brief 通过指定串口发送 TIM2 和 TIM4 当前 CNT 值。
 * @param huart UART 句柄；传入 NULL 时返回 HAL_ERROR。
 * @return HAL 串口发送状态。
 */
HAL_StatusTypeDef encoder_send_counter_report(UART_HandleTypeDef *huart);

/**
 * @brief 按 ENCODER_COUNTER_REPORT_PERIOD_MS 周期发送两个定时器 CNT 值。
 * @note 需要在主循环中高频调用。
 */
void encoder_counter_report_process(void);

#endif // ENCODER_H

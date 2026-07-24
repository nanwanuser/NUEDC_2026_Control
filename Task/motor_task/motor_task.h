#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "TB6612.h"

// 在此处选择一键调试模式，默认模式 1。
#define MOTOR_DEBUG_MODE_STEP 1U
#define MOTOR_DEBUG_MODE_ENCODER_COUNT 2U
#define MOTOR_DEBUG_MODE_MANUAL_PWM 3U

#ifndef MOTOR_DEBUG_MODE
#define MOTOR_DEBUG_MODE MOTOR_DEBUG_MODE_MANUAL_PWM
#endif

// 使用 1 或 2 选择调试电机，默认电机 1。
#define MOTOR_DEBUG_MOTOR_NUMBER 1U
#define MOTOR_DEBUG_MOTOR_INDEX (MOTOR_DEBUG_MOTOR_NUMBER - 1U)

#if MOTOR_DEBUG_MOTOR_NUMBER < 1U || MOTOR_DEBUG_MOTOR_NUMBER > motor_count
#error "MOTOR_DEBUG_MOTOR_NUMBER must be between 1 and motor_count"
#endif

#if MOTOR_DEBUG_MODE < MOTOR_DEBUG_MODE_STEP || \
    MOTOR_DEBUG_MODE > MOTOR_DEBUG_MODE_MANUAL_PWM
#error "MOTOR_DEBUG_MODE must be 1, 2 or 3"
#endif

// 模式 1：占空比阶跃，50 Hz 采样，10 Hz 输出。
#define MOTOR_DEBUG_CONTROL_PERIOD_MS 20U
#define MOTOR_DEBUG_REPORT_PERIOD_MS 100U
#define MOTOR_DEBUG_STEP_PERIOD_MS 5000U
#define MOTOR_DEBUG_MIN_DUTY_PERCENT 20.0f
#define MOTOR_DEBUG_MAX_DUTY_PERCENT 70.0f
#define MOTOR_DEBUG_DUTY_STEP_PERCENT 5.0f

// 模式 2：100 Hz 累计编码器计数，1 Hz 输出。
#define MOTOR_DEBUG_COUNT_SAMPLE_PERIOD_MS 10U
#define MOTOR_DEBUG_COUNT_REPORT_PERIOD_MS 1000U

// 模式 3：有符号 PWM 控制和 RPM 输出，频率 100 Hz。
#define MOTOR_DEBUG_MANUAL_PERIOD_MS 10U
#define MOTOR_DEBUG_PWM_COMMAND_MIN (-1000)
#define MOTOR_DEBUG_PWM_COMMAND_MAX 1000

#define MOTOR_DEBUG_MOTOR_1_DIRECTION CCW
#define MOTOR_DEBUG_MOTOR_2_DIRECTION CW
#define MOTOR_DEBUG_UART_TIMEOUT_MS 20U
#define MOTOR_DEBUG_TASK_STACK_SIZE_BYTES 768U

/**
 * @brief 一键初始化当前模式需要的外设并创建电机调试任务。
 * @return 创建成功或任务已存在时返回 true，否则返回 false。
 * @note 必须在 osKernelInitialize() 之后调用；模式 2 不启动 PWM。
 */
bool motor_debug_start(void);

/**
 * @brief 设置模式 3 使用的有符号 PWM 指令。
 * @param pwm_command PWM 指令，范围 -1000 到 1000，超出时自动限幅。
 * @note 正负号控制方向，0 表示停止；仅模式 3 会将该值输出到电机。
 */
void motor_debug_set_pwm(int16_t pwm_command);

#endif // MOTOR_TASK_H

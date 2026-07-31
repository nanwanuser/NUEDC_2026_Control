/**
 * @file Servo.h
 * @brief MG996R 双通道 PWM 舵机驱动。
 *
 * 硬件映射：
 * - Z 轴升降：TIM1_CH3，PE13；
 * - 末端 Yaw：TIM1_CH4，PE14。
 *
 * TIM1 以 1 MHz 计数并输出 50 Hz PWM。Z 轴根据实际安装方向直接映射为：
 * - 0 deg   -> 2500 us，最高；
 * - 90 deg  -> 1500 us，中间；
 * - 180 deg -> 500 us，最低。
 *
 * 末端 Yaw 保持标准正向映射：0~180 deg 对应 500~2500 us。
 */
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#include <stdint.h>

#define SERVO_COUNT                         2U

#define SERVO_LIFT_PWM_CHANNEL              TIM_CHANNEL_3
#define SERVO_END_YAW_PWM_CHANNEL           TIM_CHANNEL_4

#define SERVO_MIN_ANGLE_DEG                 0.0f
#define SERVO_MAX_ANGLE_DEG                 180.0f
#define SERVO_CENTER_ANGLE_DEG              90.0f
#define SERVO_ANGLE_RESOLUTION_DEG          0.1f

/**
 * Z 轴只需要调这两个角度：
 * - INIT 同时是上电位置和任务抬起位置；
 * - LOWERED 是吸取和放置时的下降位置。
 *
 * 调试时应逐步增大 LOWERED，避免连杆到达机械限位后继续堵转。
 */
#define SERVO_LIFT_INIT_ANGLE_DEG           40.0f
#define SERVO_LIFT_LOWERED_ANGLE_DEG        125.0f
#define SERVO_LIFT_RAISED_ANGLE_DEG         SERVO_LIFT_INIT_ANGLE_DEG

#define SERVO_END_YAW_INIT_ANGLE_DEG        SERVO_CENTER_ANGLE_DEG

#define SERVO_MIN_PULSE_US                  500U
#define SERVO_MAX_PULSE_US                  2500U
#define SERVO_PWM_FREQUENCY_HZ              50U

#define SERVO_TIMER_COUNTER_HZ              1000000UL
#define SERVO_TIMER_PERIOD_COUNTS           20000UL

typedef enum
{
    SERVO_LIFT = 0,
    SERVO_END_YAW,
} Servo_Id_t;

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint32_t min_compare;
    uint32_t max_compare;
} Servo_Config_t;

/**
 * 本驱动没有舵机位置反馈。current_angle_deg 表示已经写入 PWM 的指令角度，
 * 不是舵机输出轴的实测角度。
 */
typedef struct
{
    float target_angle_deg;
    float current_angle_deg;
    uint8_t initialized;
} Servo_State_t;

/** 初始化两个通道并立即输出各自的初始化角度。 */
HAL_StatusTypeDef Servo_Init(void);

/** 立即把角度线性映射为 PWM 并写入对应通道。 */
HAL_StatusTypeDef Servo_SetAngle(Servo_Id_t servo_id, float angle_deg);

/** 与 Servo_SetAngle() 相同，保留该接口以兼容现有任务层。 */
HAL_StatusTypeDef Servo_SetAngleImmediate(Servo_Id_t servo_id, float angle_deg);

/** 依次立即设置 Z 轴和末端 Yaw 角度。 */
HAL_StatusTypeDef Servo_SetAngles(float lift_angle_deg,
                                  float end_yaw_angle_deg);

/** 直接 PWM 模式不需要周期更新；保留空接口以兼容现有任务。 */
void Servo_Update(void);

/** 保持当前 PWM 输出。 */
void Servo_Stop(Servo_Id_t servo_id);
void Servo_StopAll(void);

float Servo_GetCurrentAngle(Servo_Id_t servo_id);
float Servo_GetTargetAngle(Servo_Id_t servo_id);
uint8_t Servo_IsAtTarget(Servo_Id_t servo_id);
const Servo_State_t *Servo_GetState(Servo_Id_t servo_id);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */

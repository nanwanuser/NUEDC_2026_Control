/**
 * @file Servo.h
 * @brief MG996R 双舵机 PWM 驱动公共接口。
 *
 * 硬件固定映射：
 * - 吊臂升降：TIM1_CH3，PE13；
 * - 末端电磁铁 Yaw：TIM1_CH4，PE14。
 *
 * 定时器参数为 PSC=167、ARR=19999，TIM1 输入时钟为 168 MHz，
 * 因而计数频率为 1 MHz、PWM 周期为 20 ms（50 Hz）。
 * 舵机脉宽 500~2500 us 对应机械角 0~180°。
 */
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"
#include "KalmanFilter.h"

#include <stdint.h>

/** 本驱动管理的舵机数量。 */
#define SERVO_COUNT                         2U

/**
 * 逻辑轴到实际 PWM 通道的唯一映射点。
 * 舵机1控制吊臂升降，舵机2控制末端电磁铁 Yaw。
 */
#define SERVO_LIFT_PWM_CHANNEL              TIM_CHANNEL_3
#define SERVO_END_YAW_PWM_CHANNEL           TIM_CHANNEL_4

/** MG996R 允许的软件机械角范围及中心位置。 */
#define SERVO_MIN_ANGLE_DEG                 0.0f
#define SERVO_MAX_ANGLE_DEG                 180.0f
#define SERVO_CENTER_ANGLE_DEG              90.0f

/**
 * 舵机安装方向配置：1 表示最终 PWM 方向反转，0 表示正常方向。
 * 反转只作用于“软件机械角 -> CCR”换算，不改变上层保存的目标角、当前角和滤波逻辑，
 * 机械安装完成后可分别调整两个宏，不需要改上层角度定义。
 */
#define SERVO_LIFT_REVERSED                  0U
#define SERVO_END_YAW_REVERSED               0U

/**
 * 机械零位补偿采用来源工程中的实际宏值。补偿只改变最终 CCR，
 * 不改变上层保存的目标角、当前角和滤波状态。更换舵机臂或重新安装后，
 * 应重新实机测量并调整这两个值。
 */
#define SERVO_LIFT_ZERO_TRIM_DEG             (0.0f)
#define SERVO_END_YAW_ZERO_TRIM_DEG          (0.0f)

/**
 * 软件角度分辨率为 0.1°。当前定时器每个计数为 1 us，
 * 500~2500 us 对应 CCR 500~2500，硬件理论角度分辨率为 0.09°/count。
 * 驱动仍按 0.1° 量化软件指令，再四舍五入映射到 CCR。
 */
#define SERVO_ANGLE_RESOLUTION_DEG          0.1f

/** MG996R 的目标控制脉宽和工作频率。端点应结合实物校准。 */
#define SERVO_MIN_PULSE_US                  500U
#define SERVO_MAX_PULSE_US                  2500U
#define SERVO_PWM_FREQUENCY_HZ              50U

/**
 * TIM1 时钟换算：168 MHz / (PSC + 1) = 1 MHz；
 * PWM 周期计数：ARR + 1 = 20000；1000000 / 20000 = 50 Hz。
 */
#define SERVO_TIMER_COUNTER_HZ              1000000UL
#define SERVO_TIMER_PERIOD_COUNTS           20000UL

/**
 * 角度指令滤波参数。
 * 一阶低通先抑制目标突变，随后一维卡尔曼滤波进一步平滑输出。
 * 参数与 Servo_Update() 的调用周期有关，推荐以固定 10 ms 周期调用。
 */
#define SERVO_FIRST_ORDER_ALPHA             0.35f
#define SERVO_KALMAN_PROCESS_NOISE          0.02f
#define SERVO_KALMAN_MEASUREMENT_NOISE      0.08f
#define SERVO_KALMAN_INITIAL_COVARIANCE     1.0f

/** 舵机逻辑编号，同时定义其在云台中的用途。 */
typedef enum
{
    SERVO_LIFT = 0,      /**< 舵机1：吊臂升降，TIM1_CH3/PE13。 */
    SERVO_END_YAW,       /**< 舵机2：末端电磁铁 Yaw，TIM1_CH4/PE14。 */
} Servo_Id_t;

/** 单个舵机的只读硬件配置。 */
typedef struct
{
    TIM_HandleTypeDef *timer;  /**< HAL 定时器句柄，本工程固定为 htim1。 */
    uint32_t channel;          /**< HAL PWM 通道，例如 TIM_CHANNEL_3。 */
    uint32_t min_compare;      /**< 0°/500 us 对应的 CCR 比较值。 */
    uint32_t max_compare;      /**< 180°/2500 us 对应的 CCR 比较值。 */
} Servo_Config_t;

/**
 * @brief 单个舵机的软件控制状态。
 *
 * 本工程没有舵机编码器反馈，因此 current_angle_deg 表示当前写入 PWM 的
 * 软件角度，而不是舵机输出轴的实测机械角度。
 */
typedef struct
{
    float target_angle_deg;    /**< 用户最终目标角，范围 0~180°。 */
    float low_pass_angle_deg;  /**< 一阶低通滤波器的内部状态。 */
    float current_angle_deg;   /**< 卡尔曼滤波并量化后的当前 PWM 软件角。 */
    KalmanFilter_t kalman;     /**< 本通道独立的一维卡尔曼滤波器。 */
    uint8_t initialized;       /**< 非 0 表示该通道已经完成初始化。 */
} Servo_State_t;

/**
 * @brief 初始化两个 PWM 通道，并将逻辑角置于 90°，输出时自动叠加机械零位补偿。
 * @return HAL_OK 表示成功；HAL_ERROR 表示定时器或 PWM 通道启动失败。
 * @note 调用前必须已经执行 MX_TIM1_Init()。
 */
HAL_StatusTypeDef Servo_Init(void);

/**
 * @brief 设置一个舵机的最终目标角度。
 * @param servo_id 舵机编号。
 * @param angle_deg 目标机械角；超出 0~180° 时自动限幅，并量化到 0.1°。
 * @note 本函数只更新目标，必须周期调用 Servo_Update() 才会逐步输出。
 */
HAL_StatusTypeDef Servo_SetAngle(Servo_Id_t servo_id, float angle_deg);

/** 同时设置吊臂升降和末端 Yaw 舵机的最终目标机械角。 */
HAL_StatusTypeDef Servo_SetAngles(float lift_angle_deg,
                                  float end_yaw_angle_deg);

/**
 * @brief 执行一次两通道滤波和 PWM 更新。
 * @note 推荐由裸机主循环按固定 5~10 ms 周期调用，不应在中断中执行。
 */
void Servo_Update(void);

/** 立即冻结指定通道的目标和滤波状态，并保持当前 PWM 角度。 */
void Servo_Stop(Servo_Id_t servo_id);

/** 立即冻结两个舵机，并继续输出 PWM 保持当前位置。 */
void Servo_StopAll(void);

/** 获取当前写入 PWM 的软件机械角。非法编号返回 90°。 */
float Servo_GetCurrentAngle(Servo_Id_t servo_id);

/** 获取当前最终目标机械角。非法编号返回 90°。 */
float Servo_GetTargetAngle(Servo_Id_t servo_id);

/** 判断指定通道的当前软件角是否已经到达最终目标。 */
uint8_t Servo_IsAtTarget(Servo_Id_t servo_id);

/** 获取指定通道的只读状态指针；非法编号返回 NULL。 */
const Servo_State_t *Servo_GetState(Servo_Id_t servo_id);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */

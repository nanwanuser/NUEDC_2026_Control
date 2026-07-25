#ifndef MOTOR_SPEED_CONTROL_H
#define MOTOR_SPEED_CONTROL_H

#include "motor.h"

// Simulink 模型按 1 ms 固定步长完成整定，速度环必须保持相同周期。
#define MOTOR_SPEED_CONTROL_PERIOD_MS 1U
#define MOTOR_SPEED_CONTROL_PERIOD_S 0.001f

// 底盘接口使用目标车轮转速，单位为 rpm。
#ifndef MOTOR_SPEED_TARGET_RPM_MIN
#define MOTOR_SPEED_TARGET_RPM_MIN -600.0f
#endif

#ifndef MOTOR_SPEED_TARGET_RPM_MAX
#define MOTOR_SPEED_TARGET_RPM_MAX 600.0f
#endif

// 从 simulink_pid 模型提取的 PIDF 参数，符号已转换为正向速度反馈形式。
#ifndef MOTOR_SPEED_PID_KP
#define MOTOR_SPEED_PID_KP 1.76391701674895f
#endif

#ifndef MOTOR_SPEED_PID_KI
#define MOTOR_SPEED_PID_KI 48.3950240418852f
#endif

#ifndef MOTOR_SPEED_PID_KD
#define MOTOR_SPEED_PID_KD 0.00485109028407955f
#endif

#ifndef MOTOR_SPEED_PID_FILTER_N
#define MOTOR_SPEED_PID_FILTER_N 61.765222360892f
#endif

// 车轮前进时编码器 RPM 应为正；若实车反馈方向相反，只修改对应符号。
#ifndef MOTOR1_ENCODER_FORWARD_SIGN
#define MOTOR1_ENCODER_FORWARD_SIGN -1.0f
#endif

#ifndef MOTOR2_ENCODER_FORWARD_SIGN
#define MOTOR2_ENCODER_FORWARD_SIGN 1.0f
#endif

typedef enum {
    MOTOR_SPEED_MODE_STOP = 0,
    MOTOR_SPEED_MODE_DRIVE,
    MOTOR_SPEED_MODE_BRAKE,
} motor_speed_control_mode;

typedef struct {
    float target_rpm;
    float measured_rpm;
    float pwm_command;
    float integrator;
    float filter_state;
    motor_speed_control_mode mode;
} motor_speed_control_state;

/** @brief 初始化两路速度环请求和 PID 状态。 */
void motor_speed_control_init(void);

/** @brief 设置单路电机目标车轮转速，超出范围时自动限幅。 */
void motor_speed_control_set_target(uint32_t motor_index, float target_rpm);

/** @brief 请求单路电机自然停止。 */
void motor_speed_control_stop(uint32_t motor_index);

/** @brief 请求单路电机主动刹车。 */
void motor_speed_control_brake(uint32_t motor_index);

/** @brief 请求两路电机自然停止。 */
void motor_speed_control_stop_all(void);

/** @brief 请求两路电机主动刹车。 */
void motor_speed_control_brake_all(void);

/** @brief 执行一次两路编码器采样和 PID 计算，每 1 ms 调用一次。 */
void motor_speed_control_process(void);

/** @brief 获取单路速度环状态，索引无效时返回 NULL。 */
const motor_speed_control_state *motor_speed_control_get_state(
    uint32_t motor_index);

#endif // MOTOR_SPEED_CONTROL_H

#ifndef MOTOR_SPEED_CONTROL_H
#define MOTOR_SPEED_CONTROL_H

#include "motor.h"

// 当前闭环控制电机 1。修改为 1U 可切换到电机 2。
#ifndef MOTOR_SPEED_CONTROL_INDEX
#define MOTOR_SPEED_CONTROL_INDEX 0U
#endif

// Simulink 模型固定步长为 1 ms，任务调用周期必须保持一致。
#define MOTOR_SPEED_CONTROL_PERIOD_MS 1U
#define MOTOR_SPEED_CONTROL_PERIOD_S 0.001f

// 目标速度按 100 rpm/s 在 +600 rpm 和 -600 rpm 之间往返。
#ifndef MOTOR_SPEED_TARGET_MAX_RPM
#define MOTOR_SPEED_TARGET_MAX_RPM 600.0f
#endif

#ifndef MOTOR_SPEED_TARGET_RAMP_RPM_PER_S
#define MOTOR_SPEED_TARGET_RAMP_RPM_PER_S 100.0f
#endif

// 生成模型内部仍使用 700 rpm 常量，此值用于构造等价的目标速度误差。
#ifndef SIMULINK_PID_MODEL_REFERENCE_RPM
#define SIMULINK_PID_MODEL_REFERENCE_RPM 700.0f
#endif

// 模型已按负增益电机整定，生成的 PID 输出直接传给电机驱动。
#ifndef SIMULINK_PID_MOTOR_OUTPUT_SIGN
#define SIMULINK_PID_MOTOR_OUTPUT_SIGN 1.0f
#endif

// FireWater 波形数据通过 USART1 每 20 ms 输出一次。
#ifndef MOTOR_SPEED_REPORT_PERIOD_MS
#define MOTOR_SPEED_REPORT_PERIOD_MS 20U
#endif

void motor_speed_control_init(void);
void motor_speed_control_process(void);

#endif // MOTOR_SPEED_CONTROL_H

#ifndef TRACK_LINE_H
#define TRACK_LINE_H

#include "motor_speed_control.h"

// 电机 1（Motor_Config[0]）对应左轮，电机 2（Motor_Config[1]）对应右轮。
#ifndef CHASSIS_LEFT_MOTOR_INDEX
#define CHASSIS_LEFT_MOTOR_INDEX 0U
#endif

#ifndef CHASSIS_RIGHT_MOTOR_INDEX
#define CHASSIS_RIGHT_MOTOR_INDEX 1U
#endif

// Track_line_App 的底盘输出周期。
#ifndef CHASSIS_TASK_PERIOD_MS
#define CHASSIS_TASK_PERIOD_MS 5U
#endif

// 巡线使用第一个八路灰度模块。
#ifndef TRACK_LINE_SENSOR_INDEX
#define TRACK_LINE_SENSOR_INDEX 0U
#endif

// 通道 0 位于车体左侧时保持为 1，物理方向相反时改为 0。
#ifndef TRACK_LINE_CHANNEL_0_IS_LEFT
#define TRACK_LINE_CHANNEL_0_IS_LEFT 1U
#endif

// 巡线速度分级，单位为车轮目标转速 rpm。
#ifndef TRACK_LINE_BASE_SPEED
#define TRACK_LINE_BASE_SPEED 80.0f
#endif

#ifndef TRACK_LINE_SLIGHT_INNER_SPEED
#define TRACK_LINE_SLIGHT_INNER_SPEED 68.0f
#endif

#ifndef TRACK_LINE_SLIGHT_OUTER_SPEED
#define TRACK_LINE_SLIGHT_OUTER_SPEED 88.0f
#endif

#ifndef TRACK_LINE_NORMAL_INNER_SPEED
#define TRACK_LINE_NORMAL_INNER_SPEED 64.0f
#endif

#ifndef TRACK_LINE_NORMAL_OUTER_SPEED
#define TRACK_LINE_NORMAL_OUTER_SPEED 90.0f
#endif

#ifndef TRACK_LINE_SHARP_INNER_SPEED
#define TRACK_LINE_SHARP_INNER_SPEED -80.0f
#endif

#ifndef TRACK_LINE_SHARP_OUTER_SPEED
#define TRACK_LINE_SHARP_OUTER_SPEED 200.0f
#endif

#ifndef TRACK_LINE_NORMAL_ERROR_THRESHOLD
#define TRACK_LINE_NORMAL_ERROR_THRESHOLD 1.0f
#endif

#ifndef TRACK_LINE_SHARP_ERROR_THRESHOLD
#define TRACK_LINE_SHARP_ERROR_THRESHOLD 3.0f
#endif

// 丢线持续超过该时间后，对重新捕获的轨迹进行稳定确认。
#ifndef TRACK_LINE_LOST_CONFIRM_MS
#define TRACK_LINE_LOST_CONFIRM_MS 15U
#endif

// 丢线后立即原地左转找线时，两侧车轮的目标转速绝对值。
#ifndef TRACK_LINE_SEARCH_TURN_SPEED
#define TRACK_LINE_SEARCH_TURN_SPEED 60.0f
#endif

// 搜索时必须连续检测到轨迹达到该时间，才恢复正常巡线。
#ifndef TRACK_LINE_REACQUIRE_CONFIRM_MS
#define TRACK_LINE_REACQUIRE_CONFIRM_MS 10U
#endif

// 从丢线时刻开始计时，超过该时间后自然停止。
#ifndef TRACK_LINE_LOST_STOP_MS
#define TRACK_LINE_LOST_STOP_MS 600U
#endif

/**
 * @brief 初始化底盘命令，初始化后两路电机自然停止。
 * @note motor_init() 已在 main.c 中完成，此函数不重复初始化底层电机。
 */
void chassis_init(void);

/**
 * @brief 设置左右轮速度命令。
 * @param left_speed 左轮目标转速，单位 rpm。
 * @param right_speed 右轮目标转速，单位 rpm。
 */
void chassis_set_wheel_speed(float left_speed, float right_speed);

/**
 * @brief 设置底盘前进和转向命令。
 * @param forward_speed 正数前进，负数后退。
 * @param turn_speed 正数左转，负数右转。
 * @note 混控结果超限时会按比例缩放，保持左右轮速度比例。
 */
void chassis_set_motion(float forward_speed, float turn_speed);

/** @brief 底盘直线前进，speed 的负号会被忽略。 */
void chassis_forward(float speed);

/** @brief 底盘直线后退，speed 的负号会被忽略。 */
void chassis_backward(float speed);

/** @brief 底盘原地左转，speed 的负号会被忽略。 */
void chassis_turn_left(float speed);

/** @brief 底盘原地右转，speed 的负号会被忽略。 */
void chassis_turn_right(float speed);

/** @brief 两路电机自然停止。 */
void chassis_stop(void);

/** @brief 两路电机主动刹车。 */
void chassis_brake(void);

/**
 * @brief 把最近一次底盘命令更新到两路电机速度环。
 */
void chassis_process(void);

/** @brief 初始化灰度传感器和巡线状态，默认启用巡线。 */
void track_line_init(void);

/** @brief 读取八路灰度数据并更新底盘目标。 */
void track_line_process(void);

/** @brief 启用巡线控制。 */
void track_line_enable(void);

/** @brief 停止巡线并让底盘自然停止。 */
void track_line_disable(void);

/** @brief 获取最近一次八路灰度打包数据。 */
uint8_t track_line_get_sensor_data(void);

/** @brief 获取最近一次有效巡线位置误差，正数表示线在左侧。 */
float track_line_get_error(void);

/** @brief 最近一次采样是否检测到线。 */
uint8_t track_line_is_detected(void);

/**
 * @brief CubeMX Track_line 弱任务的强定义。
 */
void Track_line_App(void *argument);

#endif // TRACK_LINE_H

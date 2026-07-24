#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

/**
 * @brief 速度环 PID 参数。
 * @note 输出单位由调用方决定；本工程建议使用占空比百分数。
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float derivative_cutoff_hz;
} speed_pid_params;

/**
 * @brief 速度环 PID 控制器运行状态。
 */
typedef struct {
    speed_pid_params params;
    float integral;
    float previous_measurement;
    float filtered_derivative;
    uint8_t initialized;
} speed_pid_controller;

/**
 * @brief 初始化速度环 PID 控制器。
 * @param controller 控制器指针。
 * @param params PID 参数指针。
 */
void speed_pid_init(speed_pid_controller *controller,
                    const speed_pid_params *params);

/**
 * @brief 清空积分项和微分状态，但保留 PID 参数。
 * @param controller 控制器指针。
 */
void speed_pid_reset(speed_pid_controller *controller);

/**
 * @brief 执行一次位置式速度 PID 计算。
 * @param controller 控制器指针。
 * @param target_rpm 目标输出轴转速，单位 rpm。
 * @param measured_rpm 实测输出轴转速，单位 rpm。
 * @param sample_period_s 实际采样周期，单位 s，必须大于 0。
 * @return 限幅后的控制输出；参数无效时返回 0。
 */
float speed_pid_update(speed_pid_controller *controller,
                       float target_rpm,
                       float measured_rpm,
                       float sample_period_s);

#endif // SPEED_PID_H

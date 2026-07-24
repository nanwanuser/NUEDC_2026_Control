# L1 自适应电机速度控制器

## 1. 模块定位

本模块是与 HAL、FreeRTOS 和具体电机驱动解耦的纯 C 控制器：

- 输入：目标速度和实测速度，单位均为 RPM
- 输出：带符号的 PWM 指令，默认范围为 `-1000 ~ 1000`
- 默认调用周期：`0.01 s`，即 `100 Hz`
- 正输出和负输出分别表示两个方向，方向与电机接线的对应关系由上层处理

控制器包含一阶理想参考模型、状态预测器、带投影边界的自适应律，以及只让低频补偿进入控制量的一阶低通滤波器。`reference_model_norm` 是理想模型速度，实测速度应在暂态后跟随它。

不辨识电机参数并不等于没有假设。本实现适合对增益、负载和摩擦变化做工程补偿，但在没有对象增益符号、输入增益下界和未建模动态上界时，不能宣称满足严格的 L1 鲁棒稳定性证明。首次上车必须架空车轮、限制目标速度并观察电流和振荡。

## 2. 默认参数

`L1Adaptive_GetDefaultConfig()` 返回以下初值：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `sample_time_s` | `0.01` | 控制和测速周期，100 Hz |
| `speed_scale_rpm` | `500` | 归一化速度，目标会限制到 +/-500 RPM |
| `output_limit` | `1000` | PWM 指令绝对值上限 |
| `reference_bandwidth_rad_s` | `2*pi*5` | 理想模型带宽 |
| `control_filter_bandwidth_rad_s` | `2*pi*10` | 自适应补偿低通带宽 |
| `adaptation_gain` | `20` | 自适应估计更新增益 |
| `uncertainty_limit` | `1.5` | 归一化不确定性估计边界 |

这些值是 100 Hz 下的保守起点，不是 MG310 的已辨识参数。

## 3. 500 PPR GMR 编码器测速

购买页面给出的减速比为 `20.409:1`。若 `500 PPR` 表示电机轴每相每转 500 个脉冲，并且 STM32 定时器使用正交四倍频，则减速箱输出轴每转计数为：

```text
counts_per_output_rev = 500 * 4 * 20.409 = 40818
rpm = delta_count * 60 / (counts_per_output_rev * 0.01)
    = delta_count * 0.146994...
```

厂家对 PPR/CPR 的口径可能不同。上车前手动让输出轴准确转一圈并读取累计计数：若约为 `40818`，上式成立；若约为 `10205`，则厂家给出的 500 已包含四倍频计数，此时不要再乘 4。

测速、状态估计和控制更新必须由同一个固定 10 ms 周期触发。不要用普通 `osDelay(10)` 累积调度漂移；应使用 `vTaskDelayUntil()` 或等效的绝对周期调度。

## 4. 使用示例

```c
#include "Encoder.h"
#include "TB6612.h"
#include "l1_adaptive.h"

#include <math.h>

#define CONTROL_PERIOD_S              0.01f
#define COUNTS_PER_OUTPUT_REV         40818.0f

static l1_adaptive_controller_t speed_controller;

void speed_control_init(void)
{
    l1_adaptive_config_t config;

    L1Adaptive_GetDefaultConfig(&config);
    (void)L1Adaptive_Init(&speed_controller, &config);
    L1Adaptive_Reset(&speed_controller, 0.0f);
}

void speed_control_step(float target_rpm)
{
    const int16_t delta_count =
        encoder_get_delta_count(&Encoder_Config[0]);
    const float measured_rpm =
        (float)delta_count * 60.0f /
        (COUNTS_PER_OUTPUT_REV * CONTROL_PERIOD_S);
    const float signed_pwm =
        L1Adaptive_Update(&speed_controller, target_rpm, measured_rpm);

    set_direction(Motor_Config[0], signed_pwm >= 0.0f ? CW : CCW);
    motor_set_speed(Motor_Config[0], fabsf(signed_pwm));
}
```

正常改变目标速度时直接继续调用 `L1Adaptive_Update()`，不需要清空已经学习到的补偿。编码器状态重建、控制任务重新启动，或故障解除后重新使能控制时，调用 `L1Adaptive_Reset()`。堵转保护应由上层根据电流和速度另行实现，不能依赖控制器输出限幅代替。

## 5. 调参顺序

1. 先确认编码器正方向与 PWM 正方向一致，并校准每圈计数。
2. 架空车轮，从 `50 RPM` 目标和较小的 `output_limit` 开始。
3. 若响应太慢，先小幅提高 `reference_bandwidth_rad_s`；若噪声或振荡明显，则降低它。
4. 若恒定负载下偏差消除太慢，小幅提高 `adaptation_gain`；若估计量快速撞到边界或振荡，则降低它。
5. `control_filter_bandwidth_rad_s` 越高补偿越快，但也越容易把噪声和未建模高频动态送入 PWM，应保守调整。

每次只改一个参数，并记录 `measured_speed_rpm`、`reference_model_norm * speed_scale_rpm`、`uncertainty_estimate`、`output` 和 `output_saturated`。

# NUEDC 2026 Control

STM32F407 电赛控制工程。当前电机部分用于 MG310 电机、MGR 编码器和 TB6612 驱动，提供单电机阶跃、编码器计数和手动 PWM 三种调试模式，并为后续速度闭环 PID 整定采集数据。

## 1. 当前电机硬件配置

| 功能 | 当前配置 |
| --- | --- |
| 主控 | STM32F407 |
| 电机 | MG310 |
| 编码器 | MGR |
| 电机驱动 | TB6612 |
| PWM 定时器 | TIM9 CH1 / CH2 |
| 编码器定时器 | TIM2 / TIM4 |
| 调试串口 | USART1，115200-8-N-1 |
| 轮子直径 | 38 mm，即 `0.038 m` |

板级电机和编码器绑定位于 `Core/Src/main.c`：

```text
Motor1: TIM9 CH1 + TIM2 Encoder
Motor2: TIM9 CH2 + TIM4 Encoder
```

## 2. 单电机一键调试模式

一键启动接口：

```c
bool motor_debug_start(void);
```

`MX_FREERTOS_Init()` 已自动调用该接口。调试任务默认只使用电机 1，模式在 `Task/motor_task/motor_task.h` 中选择：

```c
#define MOTOR_DEBUG_MODE_STEP 1U
#define MOTOR_DEBUG_MODE_ENCODER_COUNT 2U
#define MOTOR_DEBUG_MODE_MANUAL_PWM 3U

#define MOTOR_DEBUG_MODE MOTOR_DEBUG_MODE_STEP
#define MOTOR_DEBUG_MOTOR_NUMBER 1U
```

| 模式 | 功能 | 电机输出 | USART1 输出频率 |
| --- | --- | --- | --- |
| 模式 1，默认 | 自动占空比阶跃 | 启动默认电机 1 | 10 Hz |
| 模式 2 | 手动转动一圈并读取累计编码器计数 | 不初始化 TB6612，不启动 PWM | 1 Hz |
| 模式 3 | 接口设置有符号 PWM，同时读取原始 RPM | `-1000~1000`，正负号控制方向 | 100 Hz |

要切换模式，只修改 `MOTOR_DEBUG_MODE`，例如：

```c
#define MOTOR_DEBUG_MODE MOTOR_DEBUG_MODE_ENCODER_COUNT
```

### 2.1 模式 1：自动阶跃

默认参数：

```c
#define MOTOR_DEBUG_CONTROL_PERIOD_MS 20U
#define MOTOR_DEBUG_REPORT_PERIOD_MS 100U
#define MOTOR_DEBUG_STEP_PERIOD_MS 5000U
#define MOTOR_DEBUG_MIN_DUTY_PERCENT 20.0f
#define MOTOR_DEBUG_MAX_DUTY_PERCENT 70.0f
#define MOTOR_DEBUG_DUTY_STEP_PERCENT 5.0f
```

占空比序列为：

```text
20% → 25% → 30% → ... → 70% → 65% → ... → 20%
```

每级保持 5 秒，一个完整上升和下降周期约 100 秒。

### 2.2 模式 2：单圈编码器计数

模式 2 只初始化编码器，不调用 `motor_init()`，因此不会启动 PWM。任务内部以 10 ms 周期累计计数，以 1 Hz 输出：

```text
COUNT=40818
```

使用时先启动系统，再手动将电机 1 输出轴或轮子准确转动一圈，记录转动前后的 `COUNT` 差值。该差值用于确认 `ENCODER_COUNTS_PER_WHEEL_REV` 是否正确。

### 2.3 模式 3：接口设置有符号 PWM

模式 3 使用以下接口修改 PWM 指令：

```c
void motor_debug_set_pwm(int16_t pwm_command);
```

示例：

```c
motor_debug_set_pwm(-102);
```

参数超出 `-1000~1000` 时自动限幅；正负号控制方向，`0` 停止。任务以 100 Hz 应用最新指令并输出原始 RPM。方向切换前会先将 PWM 置零。

## 3. 编码器与原始转速

参数位于 `App/drivers/encoder/Encoder.h`：

```c
#define ENCODER_PULSES_PER_MOTOR_REV 500.0f
#define ENCODER_GEAR_RATIO 20.409f
#define ENCODER_QUADRATURE_MULTIPLIER 4.0f
#define ENCODER_WHEEL_DIAMETER_M 0.038f
```

当前编码器转速滤波已取消，`Encoder_Motion[].rotational_speed_rpm` 直接保存每个采样周期计算出的原始 RPM，不再经过一阶低通滤波：

```text
raw_rpm = delta_count * 60 / (counts_per_revolution * sample_period_s)
```

轮子线速度和累计距离按以下公式计算：

```text
wheel_counts = encoder_pulses * gear_ratio * quadrature_multiplier
wheel_circumference = pi * wheel_diameter
speed = raw_rpm * wheel_circumference / 60
distance = total_count * wheel_circumference / wheel_counts
```

> 必须确认 MGR 标称的 500 脉冲是单通道脉冲数、AB 两相边沿总数，还是已经包含四倍频。定义不一致会导致 RPM 和距离产生 2 倍或 4 倍误差。

## 4. USART1 输出

所有调试模式默认使用 USART1，且只处理默认电机 1。

模式 1 以 10 Hz 输出占空比、原始 RPM、线速度和累计距离：

```text
Motor1: Duty=20.0%, trend=UP
Motor1: RPM=123.456 rpm, speed=0.246 m/s, distance=0.105 m
```

模式 2 以 1 Hz 只输出累计编码器计数：

```text
COUNT=40818
```

模式 3 以 100 Hz 输出 PWM 指令和原始 RPM，不输出电机编号：

```text
PWM=-102 RPM=200RPM
```

模式 3 的 RPM 四舍五入为整数，以满足上述固定输出格式。

## 5. 速度环 PID

PID 模块位于：

```text
App/lib/speed_pid/speed_pid.h
App/lib/speed_pid/speed_pid.c
App/lib/speed_pid/README.md
```

主要接口：

```c
void speed_pid_init(speed_pid_controller *controller,
                    const speed_pid_params *params);

void speed_pid_reset(speed_pid_controller *controller);

float speed_pid_update(speed_pid_controller *controller,
                       float target_rpm,
                       float measured_rpm,
                       float sample_period_s);
```

当前 PID 模块尚未接管 PWM，阶跃任务仍为开环输出。应先用阶跃数据确认电机响应，再生成第一版 PID 参数。

控制器使用位置式 PID、测量值微分、一阶微分滤波、积分限幅和条件积分抗饱和：

```text
e(k) = r(k) - y(k)
I(k) = clamp(I(k-1) + Ki*Ts*e(k), Imin, Imax)
D_raw(k) = -(y(k)-y(k-1))/Ts
u(k) = clamp(Kp*e(k) + I(k) + Kd*D(k), Umin, Umax)
```

## 6. 生成和调整 PID 需要的数据

### 6.1 必需参数

请记录并提供：

```text
电机额定电压：
调试时实测电源电压：
电源限流值：

编码器标称脉冲数：
脉冲数定义：单通道 / AB边沿 / 已含四倍频
减速比：
编码器位置：电机轴 / 减速箱输出轴
轮径：38 mm

调试电机：Motor1 / Motor2
运行方向：CW / CCW
负载状态：架空 / 落地空载 / 实际负载

常用目标转速：
最大目标转速：
允许超调：
期望稳定时间：
是否需要正反转：
```

### 6.2 阶跃日志

至少保存一次完整的默认阶跃周期，约 100 秒：

```text
20% → 25% → ... → 70% → ... → 20%
```

为了更准确地辨识动态模型，建议额外测试：

```text
20% → 40%
30% → 50%
40% → 60%
60% → 40%
50% → 30%
```

每个阶跃保持 5～8 秒，并记录完整串口数据。

### 6.3 推荐日志格式

当前文本日志可以用于初步整定。若需要自动分析，推荐后续扩展为 CSV：

```text
time_ms,motor,duty,raw_rpm,speed_m_s,distance_m
1000,1,30.0,82.315,0.164,0.201
1100,1,30.0,81.732,0.163,0.217
```

拿到这些数据后可以计算：

- 电机启动死区。
- 占空比到稳态 RPM 的映射。
- 上升时间、下降时间、时间常数和延迟。
- 第一版 `Kp`、`Ki`、`Kd`。
- PID 输出限幅和积分限幅。
- 合适的速度环采样周期。
- 转速前馈关系 `duty_ff = a*target_rpm + b`。

## 7. 推荐调试流程

1. 架空轮子并设置电源限流。
2. 默认选择电机 1，确认电机和编码器方向。
3. 手动转动输出轴一圈，核对编码器换算参数。
4. 运行完整占空比阶跃并保存 USART1 日志。
5. 根据阶跃数据生成第一版 PI/PID 参数。
6. 先架空测试闭环，再进行落地空载测试。
7. 最后在实际比赛负载下修正积分、输出限幅和前馈参数。

对于减速直流电机，建议先使用 PI 控制，即 `Kd=0`。只有在确实需要改善动态响应且编码器噪声足够小时，再加入少量微分项。

## 8. 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

生成文件：

```text
build/Debug/NUEDC_2026_Control.elf
```

## 9. 相关文档

- `App/drivers/tb6612/TB6612_README.md`：TB6612 驱动说明。
- `App/drivers/encoder/Encoder_README.md`：编码器驱动说明。
- `App/lib/speed_pid/README.md`：PID 数学公式和整定说明。
- `Task/motor_task/README.md`：单电机三模式调试任务说明。

## 10. 安全注意事项

- 首次调试必须架空车轮。
- 使用电源限流，并观察电机和 TB6612 温升。
- 确认 TB6612 使能连接正确。
- 确认未选中的电机保持停止。
- 不要在机械结构可能夹伤人员时直接运行自动阶跃任务。
- PID 参数必须先从较保守的数值开始，避免直接输出满占空比。

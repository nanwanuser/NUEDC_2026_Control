# 单电机三模式调试任务

`motor_debug_start()` 是一键入口。`MX_FREERTOS_Init()` 已调用该函数，因此系统启动后会按编译期配置自动初始化所需外设并创建 `motorDebugTask`。

## 模式选择

在 `motor_task.h` 中修改：

```c
#define MOTOR_DEBUG_MODE_STEP 1U
#define MOTOR_DEBUG_MODE_ENCODER_COUNT 2U
#define MOTOR_DEBUG_MODE_MANUAL_PWM 3U

#ifndef MOTOR_DEBUG_MODE
#define MOTOR_DEBUG_MODE MOTOR_DEBUG_MODE_STEP
#endif
```

默认模式为 `MOTOR_DEBUG_MODE_STEP`，默认电机为电机 1：

```c
#define MOTOR_DEBUG_MOTOR_NUMBER 1U
```

## 模式 1：自动阶跃

- 初始化 TB6612 和编码器。
- 只驱动默认电机 1，另一路电机保持停止。
- 以 50 Hz 更新编码器运动数据。
- 以 10 Hz 通过 USART1 输出占空比、原始 RPM、线速度和累计距离。
- 默认每 5 秒改变 5% 占空比，并在 20%～70% 之间往返。

```text
20% → 25% → ... → 70% → 65% → ... → 20%
```

## 模式 2：单圈编码器计数

- 只初始化编码器，不调用 `motor_init()`，不会启动 PWM。
- 默认读取电机 1 的编码器。
- 内部以 10 ms 周期累计计数，避免低频读取时跨越 16 位计数范围造成歧义。
- USART1 以 1 Hz 输出累计计数。

```text
COUNT=40818
```

测试时记录手动转动一圈前后的 `COUNT`，两者差值就是实测每圈编码器计数。注意保持转动方向一致，并明确测试的是电机轴、减速箱输出轴还是轮子。

## 模式 3：有符号 PWM 与 RPM

使用公共接口设置 PWM：

```c
void motor_debug_set_pwm(int16_t pwm_command);
```

例如：

```c
motor_debug_set_pwm(-102);
```

- 输入范围为 `-1000~1000`，超出时自动限幅。
- 正负号控制方向，`0` 停止。
- 方向切换前先将 PWM 置零。
- 任务以 100 Hz 应用最新 PWM，并以 100 Hz 通过 USART1 输出。
- RPM 为未滤波的原始值，并四舍五入为整数。
- 不输出电机编号，默认使用电机 1。

固定格式：

```text
PWM=-102 RPM=200RPM
```

## 编码器参数

轮径位于 `App/drivers/encoder/Encoder.h`，当前为 38 mm：

```c
#define ENCODER_WHEEL_DIAMETER_M 0.038f
```

当前编码器速度滤波已取消，模式 1 和模式 3 均输出原始 RPM。

## 安全提示

- 模式 1 和模式 3 首次调试时必须架空车轮，并设置电源限流。
- 模式 2 不会主动启动电机，但仍应避免手动转动时夹伤手指。
- 修改模式后需要重新编译并烧录。

# Servo 驱动使用说明

本目录提供基于 STM32 HAL 的舵机 PWM 驱动，支持直接设置角度，也支持一阶滤波平滑转动。

## 文件说明

- `Servo.h`：对外接口、舵机参数和定时器通道配置
- `Servo.c`：角度到 PWM 脉宽的转换、直接控制、平滑运动控制

## 当前硬件配置

当前驱动默认使用：

- 定时器：`TIM1`
- 通道：`TIM_CHANNEL_1`
- 输出引脚：`PE9`
- PWM 周期：20 ms，适合常见 50 Hz 舵机
- 脉宽范围：`500` 到 `2500`
- 角度范围：`0.0f` 到 `180.0f`

这些配置在 `Servo.h` 中定义：

```c
#define SERVO_TIM_HANDLE      &htim1
#define SERVO_TIM_CHANNEL     TIM_CHANNEL_1
#define SERVO_MIN_ANGLE       0.0f
#define SERVO_MAX_ANGLE       180.0f
#define SERVO_MIN_PULSE       500
#define SERVO_MAX_PULSE       2500
```

如果更换舵机输出通道，需要同步修改 `SERVO_TIM_HANDLE`、`SERVO_TIM_CHANNEL`，并在 CubeMX 中配置对应的 PWM 引脚。

## 初始化要求

使用前需要先完成 HAL、时钟、GPIO 和 TIM 初始化，然后调用 `Servo_Init()`。

典型顺序：

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_TIM1_Init();

Servo_Init();
```

`Servo_Init()` 会完成以下工作：

- 初始化全局舵机状态 `g_servo`
- 将舵机输出设置到初始角度 `0.0f`
- 启动 `TIM1 CH1` 的 PWM 输出

## 常用接口

### `Servo_SetAngle_Direct(float angle)`

立即设置舵机角度。

```c
Servo_SetAngle_Direct(90.0f);
```

角度会被限制在 `SERVO_MIN_ANGLE` 到 `SERVO_MAX_ANGLE` 之间。

### `Servo_SetAngle_Smooth(float target_angle, float max_speed, float accel)`

设置目标角度，并按最大速度和加速度参数平滑运动。

```c
Servo_SetAngle_Smooth(180.0f, 60.0f, 120.0f);
```

参数含义：

- `target_angle`：目标角度，单位为度
- `max_speed`：最大速度，单位为度/秒
- `accel`：参考加速度，单位为度/秒^2

如果 `max_speed <= 0`，函数会退化为直接设置角度。

### `Servo_Loop_Process(void)`

平滑运动处理函数，需要高频调用。

```c
while (1)
{
    Servo_Loop_Process();
}
```

如果调用 `Servo_SetAngle_Smooth()` 后没有持续调用 `Servo_Loop_Process()`，舵机不会继续平滑移动到目标位置。

### `Servo_Stop(void)`

停止当前平滑运动，并将目标角度设置为当前角度。

```c
Servo_Stop();
```

## 使用示例

直接转到 90 度：

```c
Servo_Init();
Servo_SetAngle_Direct(90.0f);
```

平滑转到 180 度：

```c
Servo_Init();
Servo_SetAngle_Smooth(180.0f, 50.0f, 100.0f);

while (1)
{
    Servo_Loop_Process();
}
```

## 注意事项

- 舵机供电电流通常较大，建议使用独立 5 V 电源，并与 STM32 共地。
- 当前 `TIM1 CH1` 也可能被其他驱动占用，使用前请确认没有多个模块同时控制同一个 PWM 通道。
- `SERVO_MIN_PULSE` 和 `SERVO_MAX_PULSE` 需要根据实际舵机调整。常见范围是 500 us 到 2500 us，也有舵机使用 1000 us 到 2000 us。
- 平滑运动依赖 `HAL_GetTick()`，系统 tick 必须正常工作。
- `Servo_Loop_Process()` 可以放在主循环中，也可以放在固定周期任务中；调用越稳定，运动越平滑。

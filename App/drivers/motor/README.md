# Motor 电机与编码器驱动

该模块把原 `tb6612` 和 `encoder` 两套驱动合并到一起，统一管理两路直流电机的 PWM、方向和编码器数据。

## 文件

- `motor.h`：数据类型、参数和公共接口
- `motor.c`：硬件配置、TB6612 控制和编码器处理

## 当前硬件配置

| 通道 | PWM | TB6612 方向脚 | 编码器定时器 | 编码器引脚 | 正速度方向 |
| --- | --- | --- | --- | --- | --- |
| 电机 1 | TIM9_CH1 / PE5 | AIN1 PA11，AIN2 PA10 | TIM2 Encoder TI12 | CH1 PA15，CH2 PB3 | CCW |
| 电机 2 | TIM9_CH2 / PE6 | BIN1 PA9，BIN2 PA8 | TIM4 Encoder TI12 | CH1 PD12，CH2 PD13 | CW |

端口和定时器句柄集中在 `motor.c` 文件开头的 `Motor_Config[]` 和 `Encoder_Config[]` 中。修改硬件连接时，需要同时修改 CubeMX 配置和这两个数组。

CubeMX 当前要求：

- TIM9 CH1、CH2 配置为 PWM，ARR 为 `9999`
- TIM2、TIM4 配置为 `Encoder Mode TI12`，ARR 为 `65535`
- AIN1、AIN2、BIN1、BIN2 配置为推挽输出，默认低电平
- PE5、PE6、PA15、PB3、PD12、PD13 配置为对应定时器复用功能

## 机械参数

`motor.h` 中包含当前 MG310 + MGR 的参数：

```c
#define ENCODER_PULSES_PER_MOTOR_REV 500.0f
#define ENCODER_GEAR_RATIO 20.409f
#define ENCODER_QUADRATURE_MULTIPLIER 4.0f
#define ENCODER_WHEEL_DIAMETER_M 0.038f
```

每个车轮一圈的有效计数为：

```text
500 x 20.409 x 4 = 40818 counts
```

更换电机、减速箱或车轮后，需要同步修改这些参数。

## 初始化

必须先完成 CubeMX 生成的外设初始化，再调用统一初始化函数：

```c
MX_GPIO_Init();
MX_TIM9_Init();
MX_TIM2_Init();
MX_TIM4_Init();
MX_USART1_UART_Init();

motor_init();
```

`motor_init()` 会执行以下操作：

1. 启动 TIM9 两路 PWM
2. 将两路 PWM 清零并把方向引脚拉低
3. 清零 TIM2、TIM4 编码器计数
4. 启动两路编码器接口

## 电机控制

```c
// 两路电机按各自配置的正方向运行，PWM 命令为 500/1000。
motor_set_speed(Motor_Config[0], 500.0f);
motor_set_speed(Motor_Config[1], 500.0f);

// 电机 2 反向运行，PWM 命令为 400/1000。
motor_set_speed(Motor_Config[1], -400.0f);

motor_stop(Motor_Config[0]);
motor_brake(Motor_Config[1]);
```

`motor_set_speed()` 是唯一的速度/PWM 设置接口：

- `1000`：按该电机配置的正方向输出 100% PWM
- `500`：按正方向输出 50% PWM
- `0`：PWM 清零，两个方向脚拉低
- `-500`：按反方向输出 50% PWM
- `-1000`：按反方向输出 100% PWM

函数会根据定时器 ARR 自动计算比较值，不再依赖固定的 `ARR=9999`。左右电机的正方向由 `Motor_Config[].positive_direction` 配置，修改接线后只需调整该字段。

## 编码器数据

```c
encoder_update_motion(0.01f);

const encoder_motion_data *motion = encoder_get_motion_data(0);
float rpm = motion->rotational_speed_rpm;
float speed = motion->linear_speed_m_s;
float distance = motion->distance_m;
```

`encoder_update_motion()` 的采样周期必须填写真实调用间隔。当前默认上报接口每 `100 ms` 更新并发送一次：

```c
for (;;) {
    encoder_motion_report_process();
    osDelay(10U);
}
```

串口上报默认使用 USART1，可通过 `ENCODER_REPORT_UART_HANDLE` 和相关周期宏调整。

## 注意事项

- 方向由 `motor_set_speed()` 根据速度正负号自动设置，不再单独调用方向函数。
- `motor_stop()` 是自然停止，`motor_brake()` 是主动刹车。
- 编码器相邻两次采样之间的计数增量绝对值不能超过 `32767`。
- `encoder_get_total_count()` 内部会读取并消耗一次增量，不要在同一周期重复调用增量接口。
- 若转速符号与期望方向相反，可交换编码器 A/B 相位，或在上层对对应电机的 RPM 取反。

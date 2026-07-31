# MG996R 双舵机驱动

## 硬件配置

| 用途 | 外设/引脚 |
| --- | --- |
| 舵机1，吊臂升降 | TIM1_CH3，PE13，`PWM1` |
| 舵机2，末端电磁铁 Yaw | TIM1_CH4，PE14，`PWM2` |
| PWM 频率 | 50 Hz |
| TIM1 输入时钟 | 168 MHz |
| PSC / ARR | 167 / 19999 |
| 计数频率 | 1 MHz，即 1 count/us |
| 目标脉宽 | 500~2500 us |
| 软件角度范围 | 0~180°，分辨率 0.1° |

TIM1 的 20 ms 周期对应 MG996R 的 50 Hz 控制周期。默认脉宽端点用于覆盖
0~180°目标范围，首次装机必须脱离负载并逐步校准，避免舵机或齿轮顶死。

## 初始化和调用

必须先完成 `MX_TIM1_Init()`，再初始化驱动：

```c
if (Servo_Init() != HAL_OK) {
    Error_Handler();
}
```

初始化后两个通道各自输出 `SERVO_LIFT_INIT_ANGLE_DEG` 和
`SERVO_END_YAW_INIT_ANGLE_DEG`：升降取行程上端 `0°`，此时电磁铁中心在纸面
以上 `40 mm`，角度增大对应下降；末端 Yaw 取中心角 `90°`。升降不用中位，是因为
连杆在 `90°` 时电磁铁已经低于纸面，上电瞬间就会顶在纸上。升降的这个值必须与
`Task/crane_control` 的 `lift_zero_angle_deg`、`max_z_mm` 换算出的上端角一致。

设置目标后，以固定 5~10 ms 周期调用 `Servo_Update()`，驱动会依次执行一阶低通、
卡尔曼滤波、0.1°量化和 CCR 更新。

```c
Servo_SetAngle(SERVO_LIFT, 120.0f);
Servo_SetAngle(SERVO_END_YAW, 70.0f);

for (;;) {
    Servo_Update();
    osDelay(10);
}
```

`Servo_Stop()` 只冻结当前软件角并继续输出 PWM 保持位置，不会切断舵机电源。

## 安装校准

方向和机械零位配置集中在 `Servo.h`：

```c
#define SERVO_LIFT_REVERSED          0U
#define SERVO_END_YAW_REVERSED       0U
#define SERVO_LIFT_ZERO_TRIM_DEG     (0.0f)
#define SERVO_END_YAW_ZERO_TRIM_DEG  (0.0f)
```

装配后只调整这些宏，不要在上层控制逻辑中重复反向或叠加零位补偿。

## 注意事项

- MG996R 使用独立、足够电流的电源，舵机电源地必须与 MCU 共地。
- 升降齿轮齿条不自锁，断电防坠必须由机械结构保证。
- 本驱动没有位置反馈，`current_angle_deg` 是 PWM 指令角，不是实测角。
- 多任务同时控制舵机时，由上层提供互斥保护。

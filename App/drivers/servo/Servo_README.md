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

初始化后两个通道立即输出 `SERVO_LIFT_INIT_ANGLE_DEG` 和
`SERVO_END_YAW_INIT_ANGLE_DEG`。所有角度命令都直接量化并写入 CCR，不经过低通或
卡尔曼滤波，舵机以自身速度移动。`Servo_Update()` 仅为兼容现有任务保留，不执行
任何处理。

```c
Servo_SetAngle(SERVO_LIFT, 120.0f);
Servo_SetAngle(SERVO_END_YAW, 70.0f);
```

`Servo_Stop()` 只冻结当前软件角并继续输出 PWM 保持位置，不会切断舵机电源。

## Z 轴高度调试

Z 轴约定 `0°` 为最高、`180°` 为最低。高度只由下面两个宏决定：

```c
#define SERVO_LIFT_INIT_ANGLE_DEG     40.0f
#define SERVO_LIFT_LOWERED_ANGLE_DEG  180.0f
```

`SERVO_LIFT_INIT_ANGLE_DEG` 同时控制上电位置和任务抬起位置；
`SERVO_LIFT_LOWERED_ANGLE_DEG` 控制吸取和放置时的下降位置。实机调试时逐步增加下降
角度，确认连杆未到机械限位后再继续增加。根据当前舵机安装方向，Z 轴的 `0°`、
`90°`、`180°` 分别直接输出 `2500 us`、`1500 us`、`500 us`。

## 注意事项

- MG996R 使用独立、足够电流的电源，舵机电源地必须与 MCU 共地。
- 升降齿轮齿条不自锁，断电防坠必须由机械结构保证。
- 本驱动没有位置反馈，`current_angle_deg` 是 PWM 指令角，不是实测角。
- 多任务同时控制舵机时，由上层提供互斥保护。

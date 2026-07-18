# IMU JustFloat 测试 Demo

该 Demo 仅用于验证 ICM42688P、Fusion 姿态解算和 USART1 数据链路。

## 串口配置

- USART1 TX: PA9
- 波特率: 115200
- 数据格式: 8N1
- 发送方式: 优先 DMA；DMA 启动失败或 10 ms 内未完成时自动降级为阻塞发送
- 发送频率: 100 Hz

USB 转串口模块需要将 RX 接 PA9，并与开发板共地。测试只发送数据，不需要连接 PA10。

## JustFloat 帧

每帧包含 10 个小端 IEEE-754 `float32`，随后是帧尾 `00 00 80 7F`。

| 通道 | 数据 | 单位 |
|---|---|---|
| 0 | Roll | degree |
| 1 | Pitch | degree |
| 2 | Yaw | degree |
| 3 | Accel X | g |
| 4 | Accel Y | g |
| 5 | Accel Z | g |
| 6 | Gyro X | dps |
| 7 | Gyro Y | dps |
| 8 | Gyro Z | dps |
| 9 | Temperature | degree Celsius |

程序入口位于 FreeRTOS 的 `defaultTask`，任务栈为 2 KB。IMU 初始化失败或尚未产生首帧时仍会发送诊断帧：通道 0 至 8 为 0，通道 9 为 `-(AppIMU_GetInitStatus() + 1)`。

诊断值对应关系：`-1=OK但尚无采样`、`-2=ERROR`、`-3=TIMEOUT`、`-4=BAD_PARAM`、`-5=NOT_INIT`、`-6=WHOAMI_ERROR`。

ICM42688P 没有磁力计，因此 Yaw 会随运行时间产生漂移；这不代表串口协议或 Fusion 算法调用失败。

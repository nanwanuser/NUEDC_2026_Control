# Gimbal_ctrl_Task 双轴云台控制

## 轴定义与坐标

- 电机1控制 Yaw，电机2控制 Pitch。
- 两轴软件角度范围均为 `-45.0°～45.0°`。
- 所有角度接口使用 `0.1°` 为一个单位：`10` 表示 `1.0°`，`450` 表示 `45.0°`。
- 电机物理正转对应软件角度减小，电机物理反转对应软件角度增加。
- 超出限幅的相对或绝对目标会钳位到最近的边界。

## 初始化

`gimbal_ctrl_initialize()` 按以下顺序执行：

1. 电机1不发生物理运动。
2. 电机2以正转方向进入力矩模式，目标电流 `200 mA`。
3. 保持 `2000 ms`，再把力矩目标清零。
4. 电机2以 `10 RPM`、加速度 `2` 相对反转 `100°`。
5. 等待运动完成后，向电机1和电机2分别发送 `0xF8` 当前位置清零命令。
6. 将 Yaw、Pitch 软件角度记录均设为 `0.0°`。

如果任一步发送、接收或驱动器应答失败，初始化立即返回对应错误，扫描和角度控制保持禁用。

## 角度控制接口

```c
gimbal_ctrl_yaw_relative(angle_tenths, speed_rpm, acceleration);
gimbal_ctrl_yaw_absolute(angle_tenths, speed_rpm, acceleration);
gimbal_ctrl_pitch_relative(angle_tenths, speed_rpm, acceleration);
gimbal_ctrl_pitch_absolute(angle_tenths, speed_rpm, acceleration);
```

- 相对接口使用 PD42S1 `0xF3` 相对位置命令。
- 绝对接口使用 PD42S1 `0xF2` 绝对位置命令。
- 连续 `0.1°` 运动使用累计目标位置换算，避免每次独立取整造成明显累计误差。

示例：

```c
/* Yaw 软件角度增加 1.0°，对应电机1物理反转。 */
gimbal_ctrl_yaw_relative(10, 10U, 2U);

/* Pitch 移动到软件绝对角度 -20.0°。 */
gimbal_ctrl_pitch_absolute(-200, 10U, 2U);
```

## 扫描模式

`gimbal_ctrl_scan()` 只能在初始化成功后调用：

1. 先移动到 `(Pitch, Yaw) = (-45°, -45°)`。
2. Yaw 以 `10 RPM`、加速度 `2`，按 `1°` 小步移动到另一端，每步检查一次最新视觉结果。
3. Yaw 到达边界后，Pitch 增加 `5°`，再反向扫描下一行。
4. 收到未丢失目标立即返回 `HAL_OK`。
5. 最后一行到达 `(45°, 45°)` 仍未找到目标时返回 `HAL_TIMEOUT`。

扫描步长、速度和加速度可通过 `Gimbal_ctrl_Task.h` 中的配置宏调整。

## 视觉串口协议与输入

视觉使用 `USART1`，串口参数为 `115200 baud, 8 data bits, no parity, 1 stop bit`。云台任务采用简单的 HAL 阻塞轮询接收，不使用 DMA、接收中断、消息队列或额外调度任务。

- MaixCAM `A19 (UART1_TX)` 接主控 `PB7 (USART1_RX)`。
- 需要双向通信时，MaixCAM `A18 (UART1_RX)` 接主控 `PB6 (USART1_TX)`；当前只接收视觉数据时可不接。
- 两块板必须共地并使用 `3.3 V TTL` 电平；PD42S1/MAX485 继续独立使用 `USART2`。

每个视觉数据包固定为8字节，多字节整数为小端：

| 字节 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | HEAD1 | `uint8_t` | 固定 `0xAA` |
| 1 | HEAD2 | `uint8_t` | 固定 `0x55` |
| 2 | STATUS | `uint8_t` | `0x00` 有效，`0xFF` 目标丢失 |
| 3-4 | DX | `int16_t` | `靶心x - 激光x`，向右为正 |
| 5-6 | DY | `int16_t` | `激光y - 靶心y`，向上为正 |
| 7 | CHECKSUM | `uint8_t` | 字节0到6累加和的低8位 |

接收实现位于 `Gimbal_ctrl_Vision.c`：先搜索帧头 `AA 55`，再收满8字节并检查状态和校验。`STATUS=0xFF` 时还会检查四个坐标字节均为 `0xFF`。不完整、校验错误或未知状态的数据包不会更新控制向量。

完整无效包：

```text
AA 55 FF FF FF FF FF FA
```

有效向量 `DX=+20`、`DY=-10`：

```text
AA 55 00 14 00 F6 FF 08
```

已解包数据也可直接通过下列接口提交：

```c
void gimbal_ctrl_vision_input(uint8_t status, int16_t x, int16_t y);
```

- `status=0x00` 时，`x > 0` 使激光向右，`y > 0` 使激光向上。
- `status=0xFF` 时判定目标丢失；其他状态被忽略。
- 是否丢失只由 `STATUS` 决定，因此有效向量 `(-1, -1)` 不会被误判。
- 扫描过程中每移动 `1°` 轮询一次 USART1；视觉修正阶段等待下一帧的默认超时为 `1000 ms`。

## 视觉修正

`gimbal_ctrl_correct()` 的行为：

1. 收到有效向量后计算 `abs(x) + abs(y)`；小于 `50` 时返回 `HAL_OK`。
2. 不使用圆周角或空间几何反解，只沿输入 `(x, y)` 方向做归一化直线小步进。
3. 向量绝对值最大的轴本次移动 `1.0°`，另一轴按比例缩放，任一轴单次不超过 `1.0°`。
4. 两轴速度按步进比例缩放，使完成时间尽量接近，从而近似沿向量直线运动。
5. 等待下一次有效视觉输入超时后返回 `HAL_TIMEOUT`。
6. 最多执行5次位置修正；达到5次后按需求返回 `HAL_OK`。

## FreeRTOS 任务入口

`Task/Gimbal_ctrl_Task/Gimbal_ctrl_Task.c` 提供强定义 `Gimbal_ctrl_App()`，依次执行：

```text
初始化 -> 扫描 -> 找到目标后执行视觉修正 -> 任务退出
```

`Core/Src/freertos.c` 中保留 CubeMX 生成的弱定义空循环，链接时由本模块的强定义覆盖。`test.txt` 中依赖圆周角的几何反解方法未被使用。
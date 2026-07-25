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
2. 电机2以正转方向进入力矩模式，目标电流 `400 mA`。
3. 保持 `2000 ms`，再把力矩目标清零。
4. 电机2以 `10 RPM`、加速度 `2` 相对反转 `100°`。
5. 等待运动完成后，向电机1和电机2分别发送 `0xF8` 当前位置清零命令。
6. 将 Yaw、Pitch 软件角度记录均设为 `0.0°`。

如果任一步发送、接收或驱动器应答失败，初始化立即返回对应错误，扫描和角度控制保持禁用。`Gimbal_ctrl_App()` 在初始化成功后还会额外等待 `2000 ms`，再进入第一次扫描；该等待与回零时的 `2000 ms` 力矩保持是两个独立阶段。

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

1. 保持调用时的当前 Pitch，不再预先移动到 `-45°`。
2. Yaw 先从当前位置按 `1°` 小步移动到最近的 `-45°` 或 `45°` 限位，再横扫到另一端，确保当前 Pitch 的整行都被覆盖。
3. 当前 Pitch 未找到目标时，Pitch 向上增加 `5°`，Yaw 反向扫描下一行。
4. 每个 Yaw 小步及 Pitch 换行后都会轮询一次视觉串口；收到新的有效目标帧立即返回 `HAL_OK`。
5. Pitch 到达 `45°` 且该行扫描结束仍无目标时，单次调用返回 `HAL_TIMEOUT`。

自动任务不会因 `HAL_TIMEOUT` 退出：它保持 `gimbal_ctrl_status = HAL_ERROR` 并再次从当前 Pitch 扫描。因此到达 Pitch `45°` 后仍未找到目标时，会重复扫描 `45°` 这一行，直到重新收到有效向量。

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

`gimbal_ctrl_correct()` 每次只消费一个尚未处理的新有效向量：

1. 同一帧由序列号去重，不会重复修正。
2. 收到有效向量后计算 `abs(x) + abs(y)`；小于 `50` 时认为当前帧已收敛，返回 `HAL_OK` 但不移动。
3. 未收敛时不使用圆周角或空间几何反解，只沿输入 `(x, y)` 方向做归一化直线小步进。
4. 向量绝对值最大的轴本次移动 `1.0°`，另一轴按比例缩放，任一轴单次不超过 `1.0°`。
5. 两轴速度按步进比例缩放，使完成时间尽量接近，从而近似沿向量直线运动。
6. 收到丢失帧或串口错误时返回 `HAL_ERROR`；默认 `1000 ms` 内没有新有效帧时返回 `HAL_TIMEOUT`。

修正不再限制为5次。自动任务会一直等待新的有效帧，并对每个新帧执行一次修正。

## FreeRTOS 自动工作流与共享状态

`Task/Gimbal_ctrl_Task/Gimbal_ctrl_Task.c` 提供强定义 `Gimbal_ctrl_App()`，运行流程为：

```text
上电初始化
  -> 额外等待 2000 ms
  -> 从当前 Pitch 扫描
  -> 收到有效向量：gimbal_ctrl_status = HAL_OK
  -> 每个新有效向量修正一次，并保持 HAL_OK
  -> 丢失帧 / 等待超时 / 通信或运动错误：gimbal_ctrl_status = HAL_ERROR
  -> 从丢失时的当前 Pitch 重新扫描，未找到则 Pitch 每次向上增加 5°
  -> 重新找到目标后恢复持续修正
```

共享变量定义在 `Core/Src/freertos.c`，声明在 `Gimbal_ctrl_Task.h`：

```c
extern volatile HAL_StatusTypeDef gimbal_ctrl_status;
```

- `HAL_OK`：最近收到的是有效目标向量，任务处于持续修正状态。
- `HAL_ERROR`：初始化失败、目标丢失、视觉等待超时，或任务正在重新扫描。
- 初始化失败时任务退出；初始化成功后任务常驻，不会因一次扫描结束或目标丢失而退出。

`Core/Src/freertos.c` 中的 CubeMX 弱定义空循环仍被本模块的强定义覆盖。`test.txt` 中依赖圆周角的几何反解方法未被使用。

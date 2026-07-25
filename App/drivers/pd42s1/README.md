# PD42S1 闭环步进电机驱动

本目录实现 PD42S1 在正点原子自定义协议下的力矩、绝对位置、相对位置和当前位置清零控制；底层 MAX485 变长帧处理位于 `App/lib/max485`。

## 硬件与串口

| 用途 | 外设/引脚 | 配置 |
| --- | --- | --- |
| PD42S1 总线 | USART2，PA2(TX)、PA3(RX) | 115200、8N1、无校验 |
| MAX485 方向 | PE8，用户标签 `EN485` | 默认高电平发送、低电平接收 |

如果使用的 485 模块方向电平相反，请在 `App/lib/max485/max485.h` 中交换 `MAX485_TX_ENABLE_LEVEL` 和 `MAX485_RX_ENABLE_LEVEL`。

驱动器侧必须将 `MODBUS` 设置为失能，并将两台电机地址分别设置为 1 和 2。多机总线只在物理总线两端保留 120Ω 终端电阻。

## 自定义协议

```text
C5 地址 功能码 [变长数据] CHECKSUM 5C
```

`CHECKSUM` 为从 `C5` 到最后一个数据字节的 8 位累加和。底层接口根据调用者传入的负载长度打包，不使用固定帧长；接收端以 2 ms 空闲间隔判断一帧结束，因此数据区允许出现 `0x5C`。

当前使用的命令长度如下：

| 功能 | 功能码 | 下行数据长度 | 成功应答数据长度 |
| --- | --- | ---: | ---: |
| 力矩模式 | `0xF0` | 3 字节 | 1 字节 |
| 绝对位置模式 | `0xF2` | 8 字节 | 1 字节 |
| 相对位置模式 | `0xF3` | 8 字节 | 1 字节 |
| 当前位置清零 | `0xF8` | 0 字节 | 1 字节 |

## 控制接口

```c
pd42s1_set_torque(motor_id, direction, current_ma);
pd42s1_move_absolute(motor_id, direction, acceleration,
                     speed_rpm, position_units);
pd42s1_move_relative(motor_id, direction, acceleration,
                     speed_rpm, position_units);
pd42s1_clear_position(motor_id);
```

参数范围：

- 电机地址：`PD42S1_MOTOR_1_ID` 或 `PD42S1_MOTOR_2_ID`。
- 方向：`PD42S1_DIRECTION_FORWARD` 或 `PD42S1_DIRECTION_REVERSE`。
- 加速度：0～200。
- 速度：0～6000 RPM。
- 力矩模式目标 Iq 电流：0～3000 mA。
- 位置单位：51200 等于一圈。

驱动器启用应答时，每次发送成功后必须立即调用 `pd42s1_receive_response()` 接收应答，再发送下一条命令，避免旧应答残留在 USART2 中。

双轴云台的初始化、扫描、视觉修正和 0.1° 角度接口说明见 `Task/Gimbal_ctrl_Task/README.md`。
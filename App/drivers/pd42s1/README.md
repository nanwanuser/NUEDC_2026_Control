# PD42S1 闭环步进电机驱动

本目录实现 PD42S1 在正点原子自定义协议下的力矩、绝对位置、相对位置、位置清零、
状态清除和限位回零控制（回零接口保留但本项目未使用，见下文）；底层 MAX485 变长帧
处理位于 `App/lib/max485`。

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
| 解除堵转保护 | `0xF9` | 0 字节 | 1 字节 |
| 清除状态（堵转、刹车、失能） | `0xFB` | 0 字节 | 1 字节 |
| 设置左限位原点位置 | `0x90` | 4 字节 | 5 字节 |
| 设置有无限位回零 | `0x91` | 8 字节 | 9 字节 |
| 触发回零 | `0x92` | 1 字节 | 2 字节 |
| 强制中断并退出回零 | `0x93` | 0 字节 | 1 字节 |
| 读取回零状态 | `0x96` | 0 字节 | 2 字节 |
| 读取电机实时位置 | `0x2A` | 0 字节 | 5 字节 |
| 读取电机位置误差 | `0x2B` | 0 字节 | 5 字节 |
| 读取电机到位标志 | `0x30` | 0 字节 | 2 字节 |

`0x90`～`0x93` 的应答会把参数回显在结果字节之后，长度与控制类命令不同，因此用
`pd42s1_receive_home_reply()` 接收，而不是 `pd42s1_receive_response()`。`0x96`
自带收发，无需另外接收应答。

## 控制接口

```c
pd42s1_set_torque(motor_id, direction, current_ma);
pd42s1_move_absolute(motor_id, direction, acceleration,
                     speed_rpm, position_units);
pd42s1_move_relative(motor_id, direction, acceleration,
                     speed_rpm, position_units);
pd42s1_clear_position(motor_id);
pd42s1_release_stall_protection(motor_id);
pd42s1_clear_state(motor_id);
pd42s1_set_left_limit_origin(motor_id, position_units);
pd42s1_set_home_parameters(motor_id, mode, direction,
                           speed_rpm, limit_current_ma);
pd42s1_trigger_home(motor_id, trigger);
pd42s1_abort_home(motor_id);
pd42s1_read_home_state(motor_id, &state, timeout_ms);
pd42s1_read_realtime_position(motor_id, &position_units, timeout_ms);
pd42s1_read_position_error(motor_id, &position_error_units, timeout_ms);
pd42s1_read_arrival_flag(motor_id, &arrival, timeout_ms);
```

`0xF2` 的单字节成功应答只表示驱动器接受了命令，不表示电机已经到位。任务执行到
轨迹终点后本项目调用 `pd42s1_read_position_error()`；其返回的大端有符号
`int32_t` 误差绝对值严格小于 `100 units` 时判定该轴到位。

## 堵转与状态清除

电机堵转后驱动器会锁存堵转保护，在解除之前拒绝执行任何运动命令，但仍然对命令
返回成功应答。也就是说，一台锁存住的驱动器和一台正常的驱动器在总线上看不出区别，
只能从机构不动看出来。

无限位回零（`PD42S1_HOME_MODE_LEFT_NO_LIMIT` / `..._RIGHT_NO_LIMIT`）就是靠电机
顶住硬限位堵转来判断到位的，因此这种回零一定会锁存堵转保护，回零完成后必须发送
`0xFB` 清除，否则该轴此后完全不动。`0xF9` 只解除堵转，`0xFB` 同时清除堵转、刹车
和失能三种状态。手册另外提醒：`0xFC` 立即停止（刹车）之后也要及时清除状态，否则
电机会严重发烫。

用力矩模式（`0xF0`）顶硬限位时，只要电流低于驱动器的堵转电流（`0x6B` 设置，出厂
`1500 mA`）就不会锁存堵转保护——本项目顶零用 `800 mA`。但力矩模式不会自己结束，
顶完必须发一条零电流的 `0xF0`，否则电机会一直顶着机架。

## 本项目不使用驱动器自带的回零

`0x90`～`0x96` 一组回零接口在驱动里保留可用，但 `Task/crane_control` 已经不再调用：
两轴改为力矩模式（`0xF0`）顶到机械硬限位定零位。原因见下两节，以及
`Task/crane_control/README.md` 的「为什么放弃驱动器自带的回零」。

## 回零零点位置（`0x90`）不是硬限位本身

驱动器把「找到限位」和「零点在哪」当成两件事：`0x91`+`0x92` 让电机顶到硬限位，
`0x90` 则规定找到之后要走到哪个位置去。出厂值是 `51200`，也就是**整整一圈**
（手册 4.5.1 与 5.5.5 的示例都是这个值），所以不设置 `0x90` 的轴会在收到底之后
反向退出一圈——伸缩轴上按 `94.2478 mm/rev` 就是退出约 94 mm。

因此若要用驱动器的回零、并以硬限位本身为零点，必须先把 `0x90` 设为 `0`。
`pd42s1_set_left_limit_origin()` 负责这条命令。该参数由驱动器掉电保存，所以
即使换回旧固件，写进去的值也还在。

驱动器自己还有一个回零超时（`0x95` 修改，默认 `10000 ms`，可用 `0x94` 读回）。
上位机的等待时间必须比它长，否则超时是上位机先到，中断掉一次本来还在正常
进行的回零。

参数范围：

- 电机地址：`PD42S1_MOTOR_1_ID` 或 `PD42S1_MOTOR_2_ID`。
- 方向：`PD42S1_DIRECTION_FORWARD` 或 `PD42S1_DIRECTION_REVERSE`。
- 加速度：0～200。
- 速度：0～6000 RPM。
- 力矩模式目标 Iq 电流：0～3000 mA。
- 位置单位：51200 等于一圈。

驱动器启用应答时，每次发送成功后必须立即调用 `pd42s1_receive_response()` 接收应答，再发送下一条命令，避免旧应答残留在 USART2 中。

本项目约定地址 1 控制塔身 Yaw，地址 2 控制吊臂齿条伸缩。两台新驱动器默认
地址相同时，应分别单独接线完成地址设置，再接入同一条 RS-485 总线。

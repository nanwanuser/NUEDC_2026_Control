# 底盘与巡线任务

该目录按照 `Task/README.md` 的约定，提供 CubeMX `Track_line_App` 弱任务的强定义，统一管理两路底盘电机，并使用八路数字灰度传感器完成巡线。

## 灰度传感器接线

| 模块引脚 | STM32 引脚 | 配置 |
| --- | --- | --- |
| `AD0` | `PD9` | 推挽输出 |
| `AD1` | `PD10` | 推挽输出 |
| `AD2` | `PD11` | 推挽输出 |
| `OUT` | `PD8` | 输入，无上下拉 |

默认 `OUT` 高电平表示检测到线，通道 0 位于车体左侧。如果模块逻辑相反，修改 `track_line.c` 中 `Gray_Sensor_Config[].active_level`；如果通道物理方向相反，将 `TRACK_LINE_CHANNEL_0_IS_LEFT` 改为 `0U`。

## 电机映射

- 电机 1，`Motor_Config[0]`：左轮
- 电机 2，`Motor_Config[1]`：右轮
- 正速度：小车前进
- 负速度：小车后退

如果实车左右轮对应关系不同，修改 `track_line.h` 中的 `CHASSIS_LEFT_MOTOR_INDEX` 和 `CHASSIS_RIGHT_MOTOR_INDEX`。如果单侧正方向相反，应修改 `motor.c` 中对应电机的 `positive_direction`。

## 使用方法

底盘命令范围与 motor 驱动一致，为 `-1000.0f` 到 `1000.0f`：

```c
chassis_forward(500.0f);
chassis_backward(500.0f);
chassis_turn_left(400.0f);
chassis_turn_right(400.0f);
chassis_set_wheel_speed(300.0f, 500.0f);
chassis_set_motion(500.0f, 100.0f);
chassis_stop();
chassis_brake();
```

`chassis_set_motion()` 中，`forward_speed` 正数表示前进，`turn_speed` 正数表示左转。混控结果超出范围时会等比例缩放。

所有底盘接口只更新目标命令，`Track_line_App` 每 `CHASSIS_TASK_PERIOD_MS` 先执行一次巡线计算，再调用 `chassis_process()` 把命令写入两路电机。

## 巡线算法

算法读取 `gray_sensor_read_byte()` 返回的 8 位数据，把通道位置映射为 `+3.5` 到 `-3.5`，对所有有效通道求平均位置，再按偏差范围选择三级固定轮速：

| 通道位置 | 左轮 | 右轮 |
| --- | ---: | ---: |
| 0，急左转 | 0 | 250 |
| 1、2，普通左转 | 68 | 234 |
| 3，轻微左转 | 148 | 228 |
| 3、4 居中 | 200 | 200 |
| 4，轻微右转 | 228 | 148 |
| 5、6，普通右转 | 234 | 68 |
| 7，急右转 | 250 | 0 |

丢线后，小车会按最后一次偏差方向使用单侧轮搜索；持续 `TRACK_LINE_LOST_STOP_MS` 仍未找到线时自然停止。上电时如果没有检测到线，小车保持停止。

主要调参项位于 `track_line.h`：

- `TRACK_LINE_BASE_SPEED`：直线基础速度，默认 `200`
- `TRACK_LINE_SLIGHT_INNER_SPEED`、`TRACK_LINE_SLIGHT_OUTER_SPEED`：轻微转向轮速
- `TRACK_LINE_NORMAL_INNER_SPEED`、`TRACK_LINE_NORMAL_OUTER_SPEED`：普通转向轮速
- `TRACK_LINE_SHARP_INNER_SPEED`、`TRACK_LINE_SHARP_OUTER_SPEED`：急转轮速
- `TRACK_LINE_LOST_SEARCH_SPEED`：丢线搜索速度，默认 `200`
- `TRACK_LINE_LOST_STOP_MS`：丢线停止时间

调用 `track_line_disable()` 可以停止自动巡线，之后可使用底盘接口手动控制；调用 `track_line_enable()` 恢复自动巡线。

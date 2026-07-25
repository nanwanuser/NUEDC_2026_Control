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

底盘接口设置的是车轮目标转速，单位为 `rpm`，默认范围为 `-600.0f` 到 `600.0f`：

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

所有底盘接口只更新目标转速。`Track_line_App` 每 `CHASSIS_TASK_PERIOD_MS`（默认 `5 ms`）更新巡线目标，`defaultTask` 每 `MOTOR_SPEED_CONTROL_PERIOD_MS`（固定 `1 ms`）读取两路编码器、运行两套独立 PID，并把 PID 输出写入电机 PWM。

## 巡线算法

算法读取 `gray_sensor_read_byte()` 返回的 8 位数据，把通道位置映射为 `+3.5` 到 `-3.5`，对所有有效通道求平均位置，再按偏差范围选择三级固定轮速：

| 通道位置 | 左轮 | 右轮 |
| --- | ---: | ---: |
| 0，急左转 | -80 | 200 |
| 1、2，普通左转 | 64 | 90 |
| 3，轻微左转 | 68 | 88 |
| 3、4 居中 | 80 | 80 |
| 4，轻微右转 | 88 | 68 |
| 5、6，普通右转 | 90 | 64 |
| 7，急右转 | 200 | -80 |

正常巡线后只要灰度模块没有检测到线，小车就立即以 `TRACK_LINE_SEARCH_TURN_SPEED` 原地左转找线，左轮目标为负、右轮目标为正。丢线时间超过 `TRACK_LINE_LOST_CONFIRM_MS` 后，通道 0 检测到线时立即急左转，通道 7 检测到线时立即急右转；其他通道需连续检测到轨迹达到 `TRACK_LINE_REACQUIRE_CONFIRM_MS` 后恢复正常巡线，确认期间继续原地左转。持续 `TRACK_LINE_LOST_STOP_MS` 仍未找到线时自然停止。上电时如果从未检测到线，小车保持停止。

主要调参项位于 `track_line.h`：

- `CHASSIS_TASK_PERIOD_MS`：灰度检测与底盘输出周期，默认 `5 ms`（约 `200 Hz`）
- `MOTOR_SPEED_CONTROL_PERIOD_MS`：电机速度闭环周期，固定 `1 ms`（`1000 Hz`）
- `TRACK_LINE_BASE_SPEED`：直线基础速度，默认 `80`
- `TRACK_LINE_SLIGHT_INNER_SPEED`、`TRACK_LINE_SLIGHT_OUTER_SPEED`：轻微转向轮速
- `TRACK_LINE_NORMAL_INNER_SPEED`、`TRACK_LINE_NORMAL_OUTER_SPEED`：普通转向轮速
- `TRACK_LINE_SHARP_INNER_SPEED`、`TRACK_LINE_SHARP_OUTER_SPEED`：急转轮速
- `TRACK_LINE_LOST_CONFIRM_MS`：开始执行重新捕获确认的丢线时间，默认 `15 ms`
- `TRACK_LINE_SEARCH_TURN_SPEED`：丢线后原地左转的轮速绝对值，默认 `60 rpm`
- `TRACK_LINE_REACQUIRE_CONFIRM_MS`：重新检测到轨迹的稳定确认时间，默认 `10 ms`
- `TRACK_LINE_LOST_STOP_MS`：丢线停止时间

调用 `track_line_disable()` 可以停止自动巡线，之后可使用底盘接口手动控制；调用 `track_line_enable()` 恢复自动巡线。

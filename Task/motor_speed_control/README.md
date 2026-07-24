# 电机速度闭环任务

该任务使用 `App/lib/simulink_pid` 生成的 PID 控制器，对电机 1 做 1 ms 速度闭环。

目标速度按三角波循环变化：

```text
0 -> +600 -> 0 -> -600 -> 0 rpm
```

默认斜率为 `100 rpm/s`，所以每个 `0 <-> 600 rpm` 区间持续约 6 秒。参数可在
`motor_speed_control.h` 中修改：

- `MOTOR_SPEED_CONTROL_INDEX`：闭环控制的电机编号，默认为 0
- `MOTOR_SPEED_TARGET_MAX_RPM`：最大正反转目标速度
- `MOTOR_SPEED_TARGET_RAMP_RPM_PER_S`：速度变化斜率
- `SIMULINK_PID_MOTOR_OUTPUT_SIGN`：Simulink 输出到电机命令的方向映射，当前模型使用 `1.0f`
- `MOTOR_SPEED_REPORT_PERIOD_MS`：串口波形数据周期

USART1 使用 FireWater 文本协议，每行依次为目标速度、当前速度和有符号累计编码器计数：

```text
123.45,120.38,81636\n
```

串口参数由 CubeMX 配置为 `115200, 8N1`，发送使用中断方式，不阻塞 1 ms 控制环。

当前 Simulink 生成代码内部仍使用固定 `700 rpm` 作为参考值。任务通过输入变换令
模型内部误差严格等于 `目标速度 - 当前速度`，因此无需手工修改生成文件。若重新整定
模型并修改了该固定参考值，需要同步修改 `SIMULINK_PID_MODEL_REFERENCE_RPM`。

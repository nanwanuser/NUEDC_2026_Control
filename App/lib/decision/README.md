# 拼图决策模块

模块接收视觉二进制协议解包后的 `DecisionVisionFrame`，输出每片碎片的
`pick -> transit -> place` 三个位姿。协议字段与结构体的对应关系为：

```text
seq                         -> DecisionVisionFrame.seq
piece_count                 -> DecisionVisionFrame.piece_count
piece[].id                  -> DecisionPiece.id
piece[].cx/cy               -> DecisionPiece.center
piece[].vertex_count        -> DecisionPiece.vertex_count
piece[].vertices[].x/y      -> DecisionPiece.vertices
```

视觉数据不需要提供 `yaw`。决策模块根据多边形顶点计算刚体旋转。
顶点允许顺时针或逆时针输入，但必须沿轮廓依次排列，不能交叉或乱序。

## 求解方式

题目的三项计分任务里，第 1(1)、1(2) 用选手自备碎片，第 2(1)、2(2) 用测评现场
提供的碎片——现场碎片的形状事先不可知，因此不存在可以预置的模板。原先的
固定 ID 模式已删除，只保留边匹配搜索。

`Decision_Solve()` 枚举长度接近的候选边，反向对齐后回溯拼接，并检查：

- 碎片不重叠；
- 面积接近外接矩形面积；
- 矩形边长符合题目范围；
- 每片至少有一条边位于矩形外边界。

求解成功后，模块把长边旋转到纸面 X 轴方向，并把矩形中心移动到
`DecisionConfig.target_center`。

## 末端角度约定

圆形吸盘或电磁铁在吸取时不要求对准碎片方向，因此：

```text
pick.yaw_deg  = 0
place.yaw_deg = 从当前碎片姿态到目标姿态所需的旋转量
```

放置点已经包含非中心吸取产生的 XY 旋转补偿。执行时不能再次绕碎片中心修正
放置坐标。

这里的 `yaw_deg` 只按拼图几何选取，不考虑机构末端行程。`Task/route_planning`
会给同一次搬运的所有路径点加上同一个偏置，把末端拉回中位，见
`Task/crane_control/README.md` 的「末端 Yaw 基准偏置」。由于只有抓放两点的
差值决定拼图姿态，该偏置不改变拼图结果。

步进电机轨迹只负责高位移动，分别停在吸取点和放置点正上方：

```text
approach.end = pick_above
transfer.start = pick_above
transfer.end = place_above
```

升降舵机不参与连续轨迹插值，由决策状态机在两个步进轴确认到位后单独动作。

## 调用示例

```c
DecisionTaskRequest request;

DecisionTask_GetDefaultRequest(&request);
request.vision = vision_frame;
request.config.target_center.x_mm = 105.0f;
request.config.target_center.y_mm = 220.0f;
request.execution.current_pose = robot_current_pose;

DecisionTask_Submit(&request);
```

实际使用中 `target_center` 和 Z 高度由 `Task/mission` 从吊臂配置推导，不要
在这里写死，见 `Task/crane_control/README.md`。

## 轨迹接入

求解成功后，`DecisionTask` 会自动把每个 `DecisionMove` 转换为
`TrajectoryRequest` 并依次提交给 `RoutePlanning`：

```text
current -> pick_above -> 确认双轴到位 -> 下降吸附 -> 上升
        -> place_above -> 确认双轴到位 -> 下降释放 -> 上升 -> 下一片
```

第一片的 `current` 使用 `request.execution.current_pose`，后续碎片自动使用上一片
的 `place_above`。速度、加速度和舵机行程等待时间位于 `request.execution`。执行状态通过
`DecisionTask_Output.execution_state` 查看；轨迹生成错误通过
`DecisionTask_Output.trajectory_result` 查看。

当前吸附和释放使用定时等待。接入负压或工具反馈后，可以在
`DECISION_EXECUTION_GRIP_DWELL` 和 `DECISION_EXECUTION_RELEASE_DWELL` 状态替换
为反馈确认。

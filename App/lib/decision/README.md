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

## 两种模式

`DECISION_MODE_FIXED_ID`：按照 `id` 查找 `DecisionFixedLayout` 中的目标顶点，
对当前顶点和目标顶点做刚体配准。目标顶点使用纸面毫米坐标，因此目标矩形的
位置也由模板直接确定。

`DECISION_MODE_GENERAL`：枚举长度接近的候选边，反向对齐后回溯拼接，并检查：

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

目前轨迹模块只有一个中间点。决策模块将它设置为吸取点正上方：

```text
transit.x/y = pick.x/y
transit.z   = config.transit_z_mm
```

这会优先完成抬升，再转移到放置点。若后续要求“放置点上方再垂直下降”，需要
把轨迹模块扩展为两个安全路径点。

## 调用示例

```c
DecisionTaskRequest request;

DecisionTask_GetDefaultRequest(&request);
request.mode = DECISION_MODE_GENERAL;
request.vision = vision_frame;
request.config.target_center.x_mm = 105.0f;
request.config.target_center.y_mm = 220.0f;
request.execution.current_pose = robot_current_pose;

DecisionTask_Submit(&request);
```

固定模式还需要填写 `request.fixed_layout`。模板的顶点起点和绕行方向可以与
视觉输出不同，模块会自动尝试循环移位和反向匹配。

## 轨迹接入

求解成功后，`DecisionTask` 会自动把每个 `DecisionMove` 转换为
`TrajectoryRequest` 并依次提交给 `RoutePlanning`：

```text
current -> pick -> 等待吸附 -> transit -> place -> 等待释放 -> 下一片
```

第一片的 `current` 使用 `request.execution.current_pose`，后续碎片自动使用上一片
的 `place`。速度、加速度和等待时间位于 `request.execution`。执行状态通过
`DecisionTask_Output.execution_state` 查看；轨迹生成错误通过
`DecisionTask_Output.trajectory_result` 查看。

当前吸附和释放使用定时等待。接入负压或工具反馈后，可以在
`DECISION_EXECUTION_GRIP_DWELL` 和 `DECISION_EXECUTION_RELEASE_DWELL` 状态替换
为反馈确认。

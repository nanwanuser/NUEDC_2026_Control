# 拼图决策模块

模块接收视觉 JSON 解析后的 `DecisionVisionFrame`，输出每片碎片的
`pick`、`pick_above`、`place_above` 和 `place` 四个位姿。JSON 字段与结构体的
对应关系为：

```text
seq                       -> DecisionVisionFrame.seq
pieces[].id               -> DecisionPiece.id
pieces[].cx_mm/cy_mm      -> DecisionPiece.center
pieces[].vertex_count     -> DecisionPiece.vertex_count
pieces[].vertices_mm      -> DecisionPiece.vertices
```

视觉数据不需要提供 `yaw`。决策模块根据多边形顶点计算刚体旋转。
顶点允许顺时针或逆时针输入，但必须沿轮廓依次排列，不能交叉或乱序。

## 两种模式

`DECISION_MODE_FIXED_TEMPLATE`：枚举观测碎片与模板碎片的一一对应关系，通过顶点数和
刚体配准误差选择全局最优匹配。实际匹配**不使用观测 `id`**。
`pieces[].id` 仅表示本帧观测编号，并原样写入
`DecisionMove.piece_id`。目标顶点使用 A4 纸面毫米坐标，因此目标矩形的位置由
模板直接确定。

旧名称 `DECISION_MODE_FIXED_ID` 作为兼容别名保留，新代码应使用
`DECISION_MODE_FIXED_TEMPLATE`。

`DECISION_MODE_GENERAL`：枚举候选边、端点和已放置边上的事件点，反向对齐后回溯
拼接。事件点既包括边的原始端点，也包括其他已放置碎片落在长边上的顶点，因此
支持“一条长边由多条短边连续覆盖”。求解过程检查：

- 碎片不重叠；
- 面积接近外接矩形面积；
- 矩形边长符合题目范围；
- 每片至少有一条边位于矩形外边界。
- 每条内部边都被其他碎片的反向共线边完整覆盖，不能留下缝隙。

求解成功后，模块把长边旋转到纸面 X 轴方向，并把矩形中心移动到
`DecisionConfig.target_center`。

题 1 和题 2 使用 Figure 2 的已知模板，可直接把四片碎片放到 A4 下半区的最终
矩形位置，一次动作计划同时完成“移入下半区”和“拼成矩形”。调用
`DecisionTemplate_GetFigure2Layout()` 可生成题面中的 `100 mm x 60 mm` 模板；
默认中心 `(105, 220)` 对应范围 `x=55..155 mm, y=190..250 mm`，完全位于 A4
下半区。题 3 使用通用模式，不依赖碎片模板或观测编号。

通用搜索使用模块内静态工作区以避免占用任务栈，因此
`Decision_SolveGeneral()` 不可并发或递归调用。每个候选锚片的搜索节点上限、接触
长度、共线误差、角度误差和位姿去重精度均位于 `DecisionConfig`。

## 末端角度约定

圆形吸盘或电磁铁在吸取时不要求对准碎片方向，因此：

```text
pick.yaw_deg  = 0
place.yaw_deg = 从当前碎片姿态到目标姿态所需的旋转量
```

放置点已经包含非中心吸取产生的 XY 旋转补偿。执行时不能再次绕碎片中心修正
放置坐标。

决策模块在吸取点和放置点正上方各生成一个安全点：

```text
pick_above.x/y  = pick.x/y
place_above.x/y = place.x/y
pick_above.z    = place_above.z = config.transit_z_mm
```

末端先垂直抬升，再在安全高度平移并完成相对旋转，最后从 `place_above` 垂直下降，
避免纸片贴近桌面横移或旋转。

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

固定模式还需要填写 `request.fixed_layout`。模板的排列顺序、模板内部编号、顶点
起点和绕行方向均可与视觉输出不同，模块会做全局一一匹配，并自动尝试顶点循环
移位和反向匹配。

## 轨迹接入

求解成功后，`DecisionTask` 会自动把每个 `DecisionMove` 转换为
`TrajectoryRequest` 并依次提交给 `RoutePlanning`：

```text
current -> current_above -> pick_above -> pick -> 等待吸附
pick -> pick_above -> place_above -> place -> 等待释放 -> 下一片
```

第一片的 `current` 使用 `request.execution.current_pose`，后续碎片自动使用上一片
的 `place`。速度、加速度和等待时间位于 `request.execution`。执行状态通过
`DecisionTask_Output.execution_state` 查看；轨迹生成错误通过
`DecisionTask_Output.trajectory_result` 查看。

当前执行机构按电磁铁吸取纸片设计，吸附和释放使用定时等待，不依赖夹爪动作或
夹爪反馈。轨迹的 `grip` 状态由 `CraneControl_SetMagnet()` 映射到电磁铁；该函数
当前是弱硬件钩子，分配 GPIO 后再提供强实现。

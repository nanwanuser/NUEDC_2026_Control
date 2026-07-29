# 拼图决策模块

模块接收视觉 JSON 解析后的 `DecisionVisionFrame`，输出每片碎片的
`pick -> transit -> place` 三个位姿。JSON 字段与结构体的对应关系为：

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

目前轨迹模块只有一个中间点。决策模块将它设置为吸取点正上方：

```text
transit.x/y = pick.x/y
transit.z   = config.transit_z_mm
```

这会优先完成抬升，再转移到放置点。若后续要求“放置点上方再垂直下降”，需要
把轨迹模块扩展为两个安全路径点。

## 调用示例

```c
DecisionTaskRequest request = {0};

request.mode = DECISION_MODE_GENERAL;
request.vision = vision_frame;
Decision_GetDefaultConfig(&request.config);
request.config.target_center.x_mm = 105.0f;
request.config.target_center.y_mm = 220.0f;

DecisionTask_Submit(&request);
```

固定模式还需要填写 `request.fixed_layout`。模板的顶点起点和绕行方向可以与
视觉输出不同，模块会自动尝试循环移位和反向匹配。

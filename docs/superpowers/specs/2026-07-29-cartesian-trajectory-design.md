# 四舵机机械臂末端笛卡尔轨迹设计

## 1. 目标与边界

轨迹模块接收当前位置、吸取位置、安全路径点、放置位置以及运动约束，输出任意时刻的末端参考位姿：

```text
Pose(x_mm, y_mm, z_mm, yaw_deg)
```

模块只负责末端笛卡尔轨迹生成，不负责：

- 逆运动学；
- 舵机角度与 PWM 输出；
- 电磁铁控制；
- FreeRTOS 调度周期；
- 碰撞检测与路径点搜索。

决策任务必须给出可执行的安全路径点。轨迹模块不改变路径点，只进行空间插值和时间参数化。

## 2. 文件与依赖

新增纯 C 算法库：

```text
App/lib/trajectory/trajectory.h
App/lib/trajectory/trajectory.c
App/lib/trajectory/README.md
```

模块不依赖 HAL、FreeRTOS、舵机驱动和动态内存。`CMakeLists.txt` 只增加源文件和头文件目录，使算法随 STM32 工程编译。

本次不修改 FreeRTOS、`.ioc`、逆运动学或舵机驱动。

## 3. 数据接口

位置单位统一为 `mm`，角度单位统一为 `deg`，时间单位统一为 `s`，全部使用 `float`。

```c
typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float yaw_deg;
} TrajectoryPose;

typedef struct {
    float max_linear_velocity_mm_s;
    float max_linear_acceleration_mm_s2;
    float max_yaw_velocity_deg_s;
    float max_yaw_acceleration_deg_s2;
} TrajectoryLimits;

typedef struct {
    TrajectoryPose current;
    TrajectoryPose pick;
    TrajectoryPose transit;
    TrajectoryPose place;
    TrajectoryLimits limits;
} TrajectoryRequest;
```

轨迹分为两个独立阶段：

```c
typedef enum {
    TRAJECTORY_PHASE_APPROACH = 0,
    TRAJECTORY_PHASE_TRANSFER = 1
} TrajectoryPhase;
```

- `APPROACH`：`current -> pick`，到达吸取点后停止；
- `TRANSFER`：`pick -> transit -> place`，通过安全点后在放置点停止。

公共接口：

```c
typedef enum {
    TRAJECTORY_RESULT_OK = 0,
    TRAJECTORY_RESULT_INVALID_ARGUMENT,
    TRAJECTORY_RESULT_INVALID_LIMIT,
    TRAJECTORY_RESULT_NUMERIC_ERROR
} TrajectoryResult;

typedef enum {
    TRAJECTORY_STATE_RUNNING = 0,
    TRAJECTORY_STATE_COMPLETE,
    TRAJECTORY_STATE_INVALID_ARGUMENT,
    TRAJECTORY_STATE_INVALID_PHASE
} TrajectoryState;

TrajectoryResult Trajectory_Generate(
    const TrajectoryRequest *request,
    TrajectoryPlan *plan);

TrajectoryState Trajectory_Evaluate(
    const TrajectoryPlan *plan,
    TrajectoryPhase phase,
    float time_s,
    TrajectoryPose *reference);

float Trajectory_GetDuration(
    const TrajectoryPlan *plan,
    TrajectoryPhase phase);
```

`TrajectoryPlan` 由调用方静态分配，内部保存两阶段的多项式系数和时长；调用方不直接修改其成员。

调用方完成 `APPROACH` 后控制电磁铁并等待吸附，确认成功后才开始计算 `TRANSFER` 阶段的相对时间。模块内部不包含等待时间。

## 4. 吸取阶段算法

`current -> pick` 采用笛卡尔直线和五次时间缩放：

```text
s(u) = 10u^3 - 15u^4 + 6u^5,  u in [0, 1]
```

位姿按 `P(u) = P0 + s(u)(P1 - P0)` 计算。起点和终点的速度、加速度均为零。

`yaw` 先展开为最短角距离。例如 `170 deg -> -170 deg` 按 `+20 deg` 运动。输出角度归一化到 `[-180 deg, 180 deg)`。

## 5. 搬运阶段算法

`pick -> transit -> place` 使用两段五次 Hermite 样条：

- 两段分别精确连接 `pick -> transit` 和 `transit -> place`；
- `pick` 和 `place` 的速度、加速度为零；
- `transit` 两侧使用相同的物理速度和加速度，保证 `C2` 连续；
- 安全点加速度取零，中间速度由相邻割线计算并限幅；
- 对发生方向反转的单个坐标分量，将该分量的安全点速度限制为零，避免过冲；
- 只要安全点处仍存在至少一个非零运动分量，末端便连续通过而不停车。

若输入几何退化导致所有分量在安全点的连续安全速度均为零，模块采用安全降级：精确到达安全点并停车。它不会通过制造空间过冲来强行维持非零速度。

`yaw` 按路径点顺序逐段展开最短角距离，并采用独立的角速度、角加速度约束；它与 XYZ 共享各段时长，因此同时到达路径点。

## 6. 自动时间参数化

模块先依据每段空间距离、角距离和四项运动限制计算基础时长，再生成五次多项式。

约束检查使用五次曲线的 Bezier 导数控制点凸包界：

- 一阶导数控制点给出该段最大线速度和角速度的保守上界；
- 二阶导数控制点给出该段最大线加速度和角加速度的保守上界；
- 若任一上界超限，则按速度比例或加速度比例平方根统一放大时长并重新生成系数。

这种方法不依赖离散采样点，能够保证生成轨迹不超过配置限制。只延长时间，不修改输入路径点。

## 7. 状态与错误处理

`Trajectory_Generate` 检查：

- 请求和输出指针非空；
- 所有位姿和运动限制均为有限数；
- 四项速度、加速度限制均大于零。

`Trajectory_Evaluate` 的行为：

- `time_s <= 0`：返回阶段起点和 `RUNNING`，零时长阶段除外；
- `0 < time_s < duration`：返回插值位姿和 `RUNNING`；
- `time_s >= duration`：保持阶段终点并返回 `COMPLETE`；
- 零距离阶段：直接返回终点和 `COMPLETE`；
- 无效阶段、空指针或非有限时间：返回错误状态。

模块不静默修正非法运动限制，也不产生 NaN 或无穷输出。

## 8. 验证标准

主机单元测试覆盖：

1. 吸取阶段起点、终点和中点；
2. 搬运阶段精确经过安全点；
3. 吸取点和放置点的零速度、零加速度；
4. 安全点两侧位置、速度、加速度连续；
5. 线速度、线加速度、角速度和角加速度不超过限制；
6. `170 deg -> -170 deg` 的最短角插值；
7. 零距离、重复路径点和退化安全点；
8. 空指针、NaN、无穷和非正限制。

集成验证要求：

- `Debug` 配置完整清理构建成功；
- 轨迹库不引用 HAL、FreeRTOS 或舵机符号；
- 工作区不产生未跟踪的测试二进制文件。

## 9. 后续集成

`Route_planning_App` 从决策任务获得 `TrajectoryRequest` 后调用本模块。跟踪控制任务只消费 `TrajectoryPose`。两个任务之间的队列、通知和调用周期由系统集成阶段定义，不属于本次实现。

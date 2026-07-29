# 决策与末端轨迹可视化仿真设计

## 1. 目标与范围

在工程根目录新增 `simulation`，使用 Python 和 Matplotlib 构建算法验证性仿真。仿真同时覆盖：

- `DECISION_MODE_FIXED_ID` 固定 ID 模板决策；
- `DECISION_MODE_GENERAL` 通用无序拼图决策；
- `current -> pick` approach 轨迹；
- `pick -> transit -> place` transfer 轨迹；
- 末端 `Pose(x, y, z, yaw)` 和期望夹取命令 `grip`；
- 多碎片顺序执行时的路径与约束风险。

本次不实现四连杆实体、逆运动学、关节角、舵机零位或限位仿真。输入仅使用两个确定性的内置场景，不实现 JSON 文件加载。

## 2. 核心原则

仿真必须调用工程中的实际 C 算法，不在 Python 中重写决策搜索或轨迹生成。这样仿真结果与 STM32 固件共享同一份算法实现，避免两套实现漂移。

Python 负责：

- 构造内置视觉帧、固定模板和配置；
- 调用 C API；
- 补齐决策层未提供的 `current` 和轨迹限制；
- 串行编排多片搬运；
- 采样轨迹、计算显示指标；
- 绘制交互动画和无界面快照。

## 3. 架构

数据流如下：

```text
内置场景
  -> ctypes 调用 Decision_Solve()
  -> DecisionPlan.moves[]
  -> Python 执行编排器补齐 current 和 limits
  -> ctypes 调用 Trajectory_Generate()/Trajectory_Evaluate()
  -> Pose + grip + phase 时间序列
  -> Matplotlib 动画、状态和约束曲线
```

计划文件边界：

- `simulation/build_native.py`：调用当前 MinGW，把 `decision.c` 和 `trajectory.c` 编译为仿真 DLL。
- `simulation/native_bridge.c`：仅导出关键结构的 `sizeof` 和字段 `offsetof`，用于验证 Python/C ABI；不包含或复制算法。
- `simulation/bindings.py`：定义与 C 头文件一一对应的 `ctypes.Structure`、枚举值和函数签名。
- `simulation/scenarios.py`：提供固定模板和通用拼图两个内置场景。
- `simulation/simulator.py`：调用 C 算法，串行编排每个 `DecisionMove`，生成统一时间轴。
- `simulation/visualization.py`：构建 Matplotlib 界面并更新动画。
- `simulation/main.py`：命令行和图形入口。
- `simulation/tests/`：算法绑定、决策、轨迹、编排和渲染测试。

构建产物、快照和 Python 缓存放在 `simulation/build` 或缓存目录，由 `simulation/.gitignore` 局部忽略。

## 4. 场景与执行语义

两个内置场景都包含四片多边形及确定性的初始散布姿态：

- 固定模式同时提供按碎片 ID 对应的目标多边形模板；
- 通用模式仅提供散布碎片和目标矩形中心，由现有回溯搜索求解布局。

执行器按 `DecisionPlan.moves[]` 顺序处理碎片：

1. 第一片的 `current` 使用内置 home 位姿。
2. 后续片的 `current` 使用上一片完成时的 `place` 位姿。
3. 调用 `Trajectory_Generate()` 生成 approach 和 transfer。
4. approach 完成后保持在 pick 一段固定显示时间，保持 `grip=1`。
5. 保持结束等价于调用 `RoutePlanning_ResumeTransfer()`，开始 transfer。
6. place 完成时 `grip=0`，碎片固定到最终姿态，然后进入下一片。

仿真不额外增加固件尚不存在的安全路径点。若上一片 place 到下一片 pick 发生低高度横移，界面标记风险，从而真实暴露当前编排缺口。

碎片动画使用刚体变换：吸附前保持初始多边形；吸附后以 pick 抓取点为基准，随末端 XY 和 yaw 平移旋转；释放后保持最终多边形。

## 5. 可视化与交互

主窗口为四区布局：

- `Decision Board`：初始碎片、目标轮廓、抓取点和搬运中的碎片；
- `Cartesian Path`：三维末端路径、关键点、已走路径和当前末端 yaw；
- `Pose Timeline`：`x/y/z/yaw` 时间曲线与当前时间游标；
- `Limits & Grip`：线速度、线加速度、角速度、角加速度的限制占用率，以及 `grip` 状态。

交互包括：

- `Fixed ID / General` 模式单选；
- 播放/暂停；
- 重置；
- 时间滑块；
- 当前碎片 ID、phase、状态、grip 和时间状态栏。

限制占用率超过 100% 或检测到低高度横移时使用红色警示。界面使用英文短标签，避免宿主缺少中文字体导致渲染告警。

命令行支持 `--mode fixed|general` 和 `--snapshot <path>`。指定快照时使用无界面后端，渲染完整总览后退出；默认打开交互窗口。

## 6. 错误处理

- 找不到 GCC 时，指出缺少的命令和检测路径，不自动安装。
- 缺少 NumPy 或 Matplotlib 时，报告具体模块，不自动修改 Python 环境。
- DLL 编译或加载失败时，保留编译器诊断并返回非零退出码。
- C 决策返回非 OK 结果时，显示模式和枚举值，不继续生成伪造轨迹。
- C 轨迹返回非 OK 结果时，显示碎片 ID、请求关键点和枚举值，停止该次仿真。
- 非法时间、空计划或非有限数值视为测试失败，不静默修正。

## 7. 测试与验收

### 7.1 原生绑定

- DLL 从当前工程 C 源和无算法逻辑的 ABI bridge 构建成功；
- Python 与 C 的关键结构大小和字段偏移一致；
- 两个公共算法 API 可调用并返回稳定枚举值。

### 7.2 决策

- 固定和通用模式均返回 `DECISION_RESULT_OK`；
- 两种模式均输出四个 move，piece ID 不重复；
- 固定模式的最终多边形匹配对应模板；
- 通用模式的最终多边形组成配置范围内的目标矩形，无明显面积重叠。

### 7.3 轨迹与编排

- 每片 approach 起点等于 current，终点等于 pick；
- transfer 精确经过 transit 并结束于 place；
- transit 两侧位置、速度和加速度在数值容差内连续；
- yaw 使用最短旋转方向；
- `grip` 时序为 approach 途中 0、pick 保持及 transfer 途中 1、place 完成 0；
- 四片顺序执行，前一片释放后才开始下一片。

### 7.4 约束与渲染

- 密集采样的合成线速度、线加速度、角速度和角加速度不超过请求限制及浮点容差；
- 固定与通用模式都能在无界面环境生成非空 PNG；
- 快照像素包含足够的非背景变化，排除空白渲染。

最终验证命令：

```powershell
python -m unittest discover simulation/tests -v
python simulation/main.py --mode fixed --snapshot simulation/build/fixed.png
python simulation/main.py --mode general --snapshot simulation/build/general.png
```

## 8. 非目标

- 不修改 `App/lib/decision`、`App/lib/trajectory` 或 FreeRTOS 任务逻辑；
- 不实现视觉 JSON 解析；
- 不实现真实时间调度、舵机动力学、电磁吸附反馈或碰撞物理；
- 不补充执行编排层到固件；
- 不自动安装依赖或修改系统配置。

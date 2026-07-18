# IMU 服务层

本目录是 IMU 的服务层，负责把底层驱动和上层姿态算法拼成一个可以直接给 demo 或业务层使用的模块。

## 文件说明

- app_imu.c/.h：IMU 模块对外接口，负责初始化、周期处理、数据就绪策略和姿态结果访问。

## 职责边界

服务层应该负责：

- 初始化 ICM42688P 驱动。
- 按目标采样节奏执行融合更新。
- 组合 DRDY 中断触发和轮询兜底策略。
- 对外提供 AppIMU_Init()、AppIMU_Process()、AppIMU_GetEulerDeg() 等接口。

服务层不应该负责：

- 底层寄存器细节。
- 四元数数学实现。
- OLED、VOFA+ 或其他表现层逻辑。

## 依赖方向

- 依赖驱动层：icm42688p。
- 依赖算法层：imu_fusion。
- 被：main.c 或后续应用模块调用。

## 集成方式

上层代码通常只需要调用：

- AppIMU_Init()
- AppIMU_Process()
- AppIMU_GetEulerDeg() 或 AppIMU_GetQuaternion()

这样可以把 SPI 细节和姿态解算过程隔离在 IMU 内部。

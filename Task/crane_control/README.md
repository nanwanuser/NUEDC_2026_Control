# 塔吊末端控制

本模块连接 `Task/route_planning` 与执行器驱动。规划器提供世界坐标系中的
`x_mm`、`y_mm`、`z_mm`、`yaw_deg` 和 `grip`，控制层只保留最新样本并以
20 ms 周期更新执行器，避免路径点排队造成滞后。

## 坐标定义

`CraneControlConfig.origin` 是塔吊局部坐标系在世界坐标系中的原点：

- `origin.x_mm`、`origin.y_mm`：塔吊回转中心的世界坐标。
- `origin.z_mm`：升降机构机械零位的世界高度。
- `origin.yaw_deg`：吊臂机械零方向在世界坐标系中的角度。

控制层执行以下变换：

```text
dx = x - origin.x
dy = y - origin.y
radius = sqrt(dx^2 + dy^2)
boom_yaw = atan2(dy, dx) - origin.yaw
z_local = z - origin.z
```

规划器的 `yaw_deg` 按世界坐标解释。末端 Yaw 舵机自动减去当前吊臂的世界
方向，因此吊臂回转时，末端仍尽量保持规划器指定的世界朝向。

## 初始坐标与标定

默认配置由 `CraneControl_LoadDefaultConfig()` 生成。可在任意项目源文件中
覆盖弱函数，在控制任务启动前修改原点及机械参数：

```c
void CraneControl_CustomizeConfig(CraneControlConfig *config)
{
    config->origin.x_mm = 120.0f;
    config->origin.y_mm = 80.0f;
    config->origin.z_mm = 35.0f;
    config->origin.yaw_deg = 90.0f;

    config->reach_zero_radius_mm = 45.0f;
    config->min_radius_mm = 45.0f;
    config->max_radius_mm = 145.0f;
}
```

默认按 M1、Z30 齿轮计算：每转行程为 `pi * 30 = 94.2478 mm`。伸缩轴使用
`reach_mm_per_motor_revolution`，升降轴使用
`lift_mm_per_degree = 94.2478 / 360`。实际机构若有减速级、不同有效分度圆或
安装偏差，必须按实测结果修改；四个 `*_direction_sign` 用于反转机械方向。

## 执行器与限制

- PD42S1 地址 1：塔身/吊臂 Yaw，使用绝对位置模式。
- PD42S1 地址 2：齿条伸缩，使用绝对位置模式。
- MG996R 舵机 1：升降；舵机 2：末端电磁铁 Yaw，均限制为 0–180°。
- `min/max_boom_yaw_deg`、`min/max_radius_mm` 和 `min/max_z_mm` 是软件工作空间。
- 超出工作空间的整组目标会被拒绝，状态为 `CRANE_CONTROL_OUT_OF_WORKSPACE`。

上电前应先把机构移动到定义的机械零位，并确保 PD42S1 的当前位置零点与该
配置一致。当前工程尚未分配电磁铁 GPIO，`CraneControl_SetMagnet()` 是弱函数，
需要在硬件确定后覆盖实现。舵机没有位置反馈，状态中的角度是软件输出角度，
不是实际测量值。

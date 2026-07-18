# TB8812 电机驱动说明

## 1. 文件位置

- 头文件：`App/drivers/tb6612/TB6612.h`
- 源文件：`App/drivers/tb6612/TB6612.c`

这两个文件用于控制基于 H 桥驱动芯片的直流电机，当前版本支持：

- 多电机统一初始化
- PWM 调速
- 正反转控制
- 停止
- 刹车

## 2. 当前代码结构

当前驱动内部使用全局数组 `Motor_Config[motor_count]` 保存所有电机的硬件配置。

也就是说：

- 电机数量由 `motor_count` 决定
- 每个电机绑定的定时器、通道、方向引脚，统一写在 `TB6612.c` 中
- `motor_init()` 会一次性初始化全部电机

当前工程尚未分配电机 PWM 和方向引脚，驱动提供安全的弱空配置。完成 CubeMX 配置后，可在板级配置源文件中提供同名强定义：

```c
motor_config Motor_Config[motor_count] = {
    {&htim1, TIM_CHANNEL_1, GPIOE, GPIO_PIN_7,  GPIOE, GPIO_PIN_8},
    {&htim1, TIM_CHANNEL_2, GPIOE, GPIO_PIN_12, GPIOE, GPIO_PIN_13},
};
```

## 3. 使用前准备

在调用驱动函数前，需要先完成以下初始化：

1. `HAL_Init()`
2. 时钟初始化
3. GPIO 初始化
4. PWM 对应定时器初始化

例如：

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_TIM1_Init();
motor_init();
```

## 4. 接口说明

### `void motor_init(void);`

初始化所有电机。

功能：

- 启动所有已配置电机对应的 PWM 通道
- 调用 `motor_stop()`，让所有电机进入默认停止状态

调用示例：

```c
motor_init();
```

### `void motor_set_speed(motor_config Motor_Config, float speed);`

设置单个电机速度。

参数说明：

- `Motor_Config`：目标电机配置，一般传 `Motor_Config[0]`、`Motor_Config[1]`
- `speed`：速度值，范围 `0.0 ~ 1000.0`

函数行为：

- 小于 `0.0` 时自动限制为 `0.0`
- 大于 `1000.0` 时自动限制为 `1000.0`
- 当前实现使用 `speed * 10.0f` 作为 PWM 比较值

调用示例：

```c
motor_set_speed(Motor_Config[0], 500.0f);
```

### `void set_direction(motor_config Motor_Config, Motor_Direction direction);`

设置单个电机转动方向。

方向枚举：

```c
typedef enum {
    CW = 0,
    CCW = 1,
} Motor_Direction;
```

参数说明：

- `Motor_Config`：目标电机配置
- `direction`：转动方向

调用示例：

```c
set_direction(Motor_Config[0], CW);
set_direction(Motor_Config[1], CCW);
```

说明：

- `CW` 和 `CCW` 的实际方向与电机接线有关
- 如果测试发现正反方向相反，交换 `CW` / `CCW` 对应的输出逻辑即可

### `void motor_stop(motor_config Motor_Config);`

停止单个电机。

函数行为：

- 先把速度设为 `0`
- 再将两个方向引脚都拉低

调用示例：

```c
motor_stop(Motor_Config[0]);
```

### `void motor_brake(motor_config Motor_Config);`

刹车单个电机。

函数行为：

- 先把速度设为 `0`
- 再将两个方向引脚都拉高

调用示例：

```c
motor_brake(Motor_Config[0]);
```

## 5. 典型调用流程

### 电机 1 正转

```c
motor_init();
set_direction(Motor_Config[0], CW);
motor_set_speed(Motor_Config[0], 700.0f);
```

### 电机 2 反转

```c
set_direction(Motor_Config[1], CCW);
motor_set_speed(Motor_Config[1], 400.0f);
```

### 停止电机

```c
motor_stop(Motor_Config[0]);
```

### 刹车

```c
motor_brake(Motor_Config[1]);
```

## 6. 注意事项

### 1. 必须先初始化定时器和 GPIO

如果没有先执行 `MX_GPIO_Init()` 和 `MX_TIM1_Init()`，驱动函数无法正常工作。

### 2. 先设方向，再设速度

建议调用顺序：

```c
set_direction(Motor_Config[x], CW);
motor_set_speed(Motor_Config[x], 600.0f);
```

这样语义更清晰，也更便于调试。

### 3. `motor_stop()` 和 `motor_brake()` 含义不同

- `motor_stop()`：停止输出，电机自然停下
- `motor_brake()`：主动刹车，停止更快

具体效果会和驱动芯片连接方式、电机惯量有关。

### 4. 当前版本仍然与本工程绑定

当前驱动内部直接使用：

- `motor_count`
- `Motor_Config[]`
- `htim1`
- 当前工程中的 GPIO 定义

***如果后续要移植到别的工程，通常需要修改 `Motor_Config[]` 中的内容。***

未提供板级强配置时，驱动会跳过未配置的电机，不会操作空句柄或空 GPIO。

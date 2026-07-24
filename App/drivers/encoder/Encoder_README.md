  # 编码器驱动说明

## 1. 文件位置

- 头文件：`App/drivers/encoder/Encoder.h`
- 源文件：`App/drivers/encoder/Encoder.c`

这两个文件用于读取 STM32 定时器编码器接口的计数值，当前版本支持：

- 多编码器统一初始化
- 读取当前计数器值
- 读取相邻两次采样之间的增量计数
- 读取累计总计数
- 编码器计数清零

## 2. 当前代码结构

当前驱动内部使用全局数组 `Encoder_Config[encoder_count]` 保存所有编码器的硬件配置和运行状态。

也就是说：

- 编码器数量由 `encoder_count` 决定
- 每个编码器绑定的定时器统一写在 `Encoder.c` 中
- `encoder_init()` 会一次性初始化全部编码器
- `total_count` 和 `last_count` 由驱动内部维护，用户一般不需要手动修改

当前工程没有分配编码器定时器，驱动提供不会操作硬件的弱默认配置：

```c
__weak encoder_config Encoder_Config[encoder_count] = {0};
```

## 3. 用户需要修改哪些参数

如果你要把这个驱动用到自己的工程里，通常只需要改下面几个地方。

### 1. 修改编码器数量

在 `Encoder.h` 中：

```c
#define encoder_count 1U
```

如果你有两个编码器，就改成：

```c
#define encoder_count 2U
```

### 2. 修改编码器绑定的定时器

在板级配置源文件中提供同名强定义，例如：

```c
encoder_config Encoder_Config[encoder_count] = {
    {&htim2, 0, 0},
};
```

这里的 `&htim2` 表示编码器使用 `TIM2`，并会覆盖驱动中的弱默认配置。

如果你的编码器接在 `TIM1`，就改成：

```c
{&htim1, 0, 0}
```

如果有多个编码器，就继续往数组里加：

```c
encoder_config Encoder_Config[encoder_count] = {
    {&htim1, 0, 0},
    {&htim3, 0, 0},
};
```

### 3. 修改 CubeMX / 定时器编码器模式配置

除了改驱动文件本身，还必须保证对应定时器已经在 CubeMX 中配置成编码器模式。

需要确认：

- 定时器已经配置为 `Encoder Mode`
- 使用的是 `CH1` 和 `CH2`
- 两个引脚已经正确复用到对应定时器
- 工程里已经生成对应的 `htimx`

如果这些没有配好，即使驱动代码没问题，也读不到正确计数。

## 4. 使用前准备

在调用驱动函数前，需要先完成以下初始化：

1. `HAL_Init()`
2. 时钟初始化
3. GPIO 初始化
4. 编码器对应定时器初始化
5. 调用 `encoder_init()`

例如：

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_TIM2_Init();
encoder_init();
```

## 5. 结构体成员说明

### `typedef struct { ... } encoder_config;`

```c
typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t total_count;
    uint16_t last_count;
} encoder_config;
```

各成员作用如下：

- `htim`
  绑定的定时器句柄，例如 `&htim1`、`&htim2`

- `total_count`
  从上次清零开始累计得到的总计数，正转增加，反转减少

- `last_count`
  上一次读取时保存下来的定时器当前值，用来和这一次的计数器值做差，计算增量

说明：

- `total_count` 和 `last_count` 都是驱动内部维护变量
- 正常使用时，用户只需要读取，不建议手动修改

## 6. 接口说明

### `void encoder_init(void);`

初始化所有编码器。

功能：

- 遍历 `Encoder_Config[]`
- 把每个编码器的定时器计数器清零
- 把 `total_count` 清零
- 把 `last_count` 清零
- 启动编码器接口 `TIM_CHANNEL_1 | TIM_CHANNEL_2`

调用示例：

```c
encoder_init();
```

注意：

- 这个函数通常只在上电初始化时调用一次

### `int16_t encoder_get_delta_count(encoder_config *Encoder_Config);`

读取距离上一次读取后的增量计数。

参数说明：

- `Encoder_Config`：目标编码器配置结构体指针，一般传 `&Encoder_Config[0]`

返回值说明：

- 正数：表示当前方向计数增加
- 负数：表示当前方向计数减少
- `0`：表示这次读取时计数没有变化，或者传入参数无效

函数行为：

- 读取当前定时器 `CNT`
- 用 `当前计数值 - 上一次计数值` 计算增量
- 自动更新 `last_count`
- 自动把增量累加到 `total_count`

调用示例：

```c
int16_t delta;
delta = encoder_get_delta_count(&Encoder_Config[0]);
```

适用场景：

- 计算短时间内位移变化
- 做速度估算
- 判断正反转方向

注意：

- 两次调用间隔不要太长
- 必须保证两次读取之间的真实增量绝对值不要超过 `32767`
- 否则 16 位差值可能产生歧义

### `int32_t encoder_get_total_count(encoder_config *Encoder_Config);`

读取从上次清零开始累计得到的总计数。

参数说明：

- `Encoder_Config`：目标编码器配置结构体指针，一般传 `&Encoder_Config[0]`

返回值说明：

- 返回累计总计数

函数行为：

- 内部先调用一次 `encoder_get_delta_count()`
- 再返回更新后的 `total_count`

调用示例：

```c
int32_t total;
total = encoder_get_total_count(&Encoder_Config[0]);
```

适用场景：

- 读取累计位移
- 做位置闭环控制

注意：

- 因为这个函数内部会更新一次计数状态，所以不要在同一时刻反复混用
  `encoder_get_total_count()` 和 `encoder_get_delta_count()` 去重复取同一份增量

### `uint16_t encoder_get_now_counter(encoder_config Encoder_Config);`

读取当前定时器计数器的原始值。

参数说明：

- `Encoder_Config`：目标编码器配置结构体，一般传 `Encoder_Config[0]`

返回值说明：

- 返回当前定时器 `CNT` 的原始 16 位数值

函数行为：

- 只读取当前定时器寄存器值
- 不会修改 `total_count`
- 不会修改 `last_count`

调用示例：

```c
uint16_t counter;
counter = encoder_get_now_counter(Encoder_Config[0]);
```

适用场景：

- 调试观察当前计数器值
- 排查编码器是否有输入

### `void encoder_clear_count(encoder_config *Encoder_Config);`

清零指定编码器的计数。

参数说明：

- `Encoder_Config`：目标编码器配置结构体指针，一般传 `&Encoder_Config[0]`

函数行为：

- 把定时器 `CNT` 清零
- 把 `total_count` 清零
- 把 `last_count` 清零

调用示例：

```c
encoder_clear_count(&Encoder_Config[0]);
```

适用场景：

- 上电后重新归零
- 每次测试前清零
- 位置控制前建立新的参考零点

## 7. 典型调用流程

### 1. 初始化编码器

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_TIM2_Init();
encoder_init();
```

### 2. 读取当前原始计数器值

```c
uint16_t counter;
counter = encoder_get_now_counter(Encoder_Config[0]);
```

### 3. 读取本次增量

```c
int16_t delta;
delta = encoder_get_delta_count(&Encoder_Config[0]);
```

### 4. 读取累计总计数

```c
int32_t total;
total = encoder_get_total_count(&Encoder_Config[0]);
```

### 5. 清零后重新开始计数

```c
encoder_clear_count(&Encoder_Config[0]);
```

## 8. 注意事项

### 1. 必须先初始化对应定时器

如果没有先执行对应的 `MX_TIMx_Init()`，编码器驱动无法正常工作。

### 2. 当前驱动默认使用 16 位计数差分

当前增量计算方式是：

```c
delta_count = (int16_t)(current_count - Encoder_Config->last_count);
```

这个写法可以正确处理 16 位定时器的回绕，例如：

- 正转从 `65535` 到 `0`
- 反转从 `0` 到 `65535`

但前提是：

- 两次读取之间的真实计数变化不能超过半个 16 位范围

也就是不能采样太慢。

### 3. `encoder_get_total_count()` 会更新内部状态

因为它内部调用了 `encoder_get_delta_count()`，所以：

- 调一次 `encoder_get_total_count()`，等于已经消耗了一次“增量读取”

如果你在同一个周期里又马上调一次 `encoder_get_delta_count()`，第二次得到的增量可能就是 `0` 或很小。

建议：

- 要么这个周期只用 `encoder_get_delta_count()`
- 要么这个周期只用 `encoder_get_total_count()`

### 4. 当前版本仍然与本工程绑定

当前驱动内部直接依赖：

- `encoder_count`
- `Encoder_Config[]`
- `htimx`
- 当前工程中的 CubeMX 定时器配置

如果后续移植到别的工程，通常需要修改：

- `encoder_count`
- `Encoder_Config[]`
- 对应的 `MX_TIMx_Init()`
- CubeMX 中的编码器引脚和定时器映射

## 9. 当前工程参数

MG310 + MGR 当前使用：
```c
#define ENCODER_PULSES_PER_MOTOR_REV 500.0f
#define ENCODER_GEAR_RATIO 20.409f
#define ENCODER_QUADRATURE_MULTIPLIER 4.0f
#define ENCODER_WHEEL_DIAMETER_M 0.038f
```
`encoder_update_motion()` 直接计算原始 RPM、线速度和距离；速度滤波已取消。模式 2 可手动转动一圈，以 USART1 输出的 `COUNT` 差值核对实际每圈计数。

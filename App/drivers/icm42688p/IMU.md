# IMU 集成说明

本文档记录 `match_2026_7` 工程中 ICM42688P IMU 的软件结构、CubeMX 配置、中断处理方式、算法调用流程和调参方法。

## 1. 当前集成目标

本工程只集成 IMU 软件层，不改变原有 OLED、舵机、超声波、串口等业务逻辑。

IMU 使用 ICM42688P，通过 SPI1 读取加速度计和陀螺仪数据，上层使用四元数姿态解算，并提供欧拉角接口给后续显示、控制或串口上位机使用。

当前设计原则：

- `PC4` 的 IMU 中断只作为 data-ready 标志，不在中断函数里读 SPI。
- 主循环调用 `AppIMU_Process()`，完成传感器读取和四元数更新。
- 如果 INT1 线路暂时未接好，软件保留 1ms 轮询兜底，便于先调通硬件。
- IMU 算法模块独立放在 `App/lib/imu_fusion`，方便后续迁移和调参。

## 2. 相关文件

IMU 相关源码位于：

```text
App/drivers/icm42688p/icm42688p.c
App/drivers/icm42688p/icm42688p.h
App/lib/imu_fusion/imu_fusion.c
App/lib/imu_fusion/imu_fusion.h
App/lib/imu_service/app_imu.c
App/lib/imu_service/app_imu.h
```

CubeMX/HAL 相关文件：

```text
Core/Src/main.c
Core/Src/gpio.c
Core/Src/spi.c
Core/Src/dma.c
Core/Src/stm32f4xx_it.c
Core/Inc/main.h
Core/Inc/spi.h
Core/Inc/dma.h
Core/Inc/stm32f4xx_it.h
Core/Inc/stm32f4xx_hal_conf.h
NUEDC_2026_Control.ioc
```

构建相关文件：

```text
CMakeLists.txt
cmake/stm32cubemx/CMakeLists.txt
```

## 3. 硬件连接

ICM42688P 推荐连接如下：

| ICM42688P | STM32F407 | 功能 |
|---|---|---|
| CS | PA4 | SPI 片选，低有效 |
| SCK | PA5 | SPI1_SCK |
| MISO | PA6 | SPI1_MISO |
| MOSI | PA7 | SPI1_MOSI |
| INT1 | PC4 | 数据就绪中断 |
| VCC | 3.3V | 电源 |
| GND | GND | 地 |

注意：ICM42688P 一般使用 3.3V 供电和 3.3V IO，不要直接接 5V IO。

## 4. CubeMX 配置

### 4.1 SPI1

在 CubeMX 中启用 `SPI1`：

| 参数 | 设置 |
|---|---|
| Mode | Full-Duplex Master |
| Data Size | 8 Bits |
| First Bit | MSB First |
| NSS | Software |
| Prescaler | 4 |
| CPOL | Low |
| CPHA | 1 Edge |

对应引脚：

| 引脚 | 功能 |
|---|---|
| PA5 | SPI1_SCK |
| PA6 | SPI1_MISO |
| PA7 | SPI1_MOSI |

当前 SPI 分频为 `/4`，在 84MHz APB2 下约为 21MHz。若硬件线较长或读数不稳定，可以先改成 `/8` 或 `/16` 验证通信稳定性。

### 4.2 DMA

SPI1 使用 DMA：

| DMA 请求 | Stream | Channel | Direction | Priority |
|---|---|---|---|---|
| SPI1_RX | DMA2_Stream0 | Channel 3 | Peripheral to Memory | Very High |
| SPI1_TX | DMA2_Stream3 | Channel 3 | Memory to Peripheral | Very High |

NVIC 中需要使能：

| 中断 | 优先级 |
|---|---|
| DMA2_Stream0_IRQn | 0 |
| DMA2_Stream3_IRQn | 0 |

### 4.3 GPIO

| 引脚 | 模式 | 说明 |
|---|---|---|
| PA4 | GPIO_Output | ICM42688P CS，默认输出高电平 |
| PC4 | GPIO_EXTI4 | ICM42688P INT1 |

`PC4` 当前配置为 `GPIO_MODE_IT_RISING_FALLING`，原因是前期调试时 INT1 极性可能不确定，双边沿更容易先确认中断有没有进。硬件稳定后，可以根据 ICM42688P INT1 实际配置改成单边沿。

### 4.4 NVIC

需要使能：

| 中断 | 用途 |
|---|---|
| EXTI4_IRQn | PC4 IMU INT1 数据就绪中断 |
| DMA2_Stream0_IRQn | SPI1 RX DMA 完成中断 |
| DMA2_Stream3_IRQn | SPI1 TX DMA 完成中断 |

当前工程将 EXTI4 和 SPI1 DMA 中断抢占优先级设为 `5`，与 FreeRTOS 的可调用中断优先级边界保持一致。

## 5. 软件调用流程

### 5.1 初始化流程

`main.c` 中初始化顺序应保持：

```c
MX_GPIO_Init();
MX_DMA_Init();
MX_SPI1_Init();
MX_USART1_UART_Init();
MX_TIM1_Init();

OLED_Init();
HCSR04_OLED_Init();
AppIMU_Init();
```

关键点：

- `MX_DMA_Init()` 必须在 `MX_SPI1_Init()` 之前调用。
- `MX_SPI1_Init()` 必须在 `AppIMU_Init()` 之前调用。
- `AppIMU_Init()` 会初始化 ICM42688P 和姿态解算状态。

### 5.2 主循环流程

`while(1)` 中调用：

```c
HCSR04_OLED_Process();
AppIMU_Process();
```

`AppIMU_Process()` 内部逻辑：

1. 判断 IMU 是否初始化成功。
2. 检查 ICM42688P data-ready 标志。
3. 如果中断没有触发，则按 1ms 周期轮询兜底。
4. 调用 `IMU_FusionUpdate()` 读取 IMU 原始数据。
5. 使用 Mahony 四元数算法更新姿态。
6. 更新欧拉角输出。

## 6. 中断处理逻辑

`PC4` 对应 `EXTI4_IRQHandler()`：

```c
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
```

HAL 回调中只置位 IMU 数据就绪标志：

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ICM42688P_INT1_Pin)
  {
    ICM42688_Int1IrqHandler();
  }
}
```

不要在 `HAL_GPIO_EXTI_Callback()` 里直接读 SPI，原因：

- SPI DMA 读写可能阻塞或等待中断完成。
- 在 EXTI 中断里做复杂操作会影响系统实时性。
- 中断嵌套和 DMA 回调容易引入难查的死锁或时序问题。

正确做法是：中断只通知主循环，主循环再读取 IMU。

## 7. IMU 上层接口

`../../services/IMU_service/app_imu.h` 对外提供：

```c
void AppIMU_Init(void);
void AppIMU_Process(void);
uint8_t AppIMU_IsReady(void);
icm42688_status_t AppIMU_GetInitStatus(void);
const imu_euler_t *AppIMU_GetEulerDeg(void);
const imu_quat_t *AppIMU_GetQuaternion(void);
```

使用示例：

```c
const imu_euler_t *euler = AppIMU_GetEulerDeg();

if (euler != NULL)
{
    float roll = euler->roll;
    float pitch = euler->pitch;
    float yaw = euler->yaw;
}
```

初始化状态可通过 `AppIMU_GetInitStatus()` 查看。

常见状态含义：

| 状态 | 含义 |
|---|---|
| 0 | OK |
| 1 | ERROR |
| 2 | TIMEOUT |
| 3 | BAD_PARAM |
| 4 | NOT_INIT |
| 5 | WHOAMI_ERROR |

如果返回 `WHOAMI_ERROR`，优先检查 SPI 引脚、CS 引脚、电源和 ICM42688P 地址读取逻辑。

## 8. 姿态解算参数

默认融合参数位于 `../../lib/IMU_RC/imu_fusion.c` 的 `IMU_FusionGetDefaultConfig()`。

当前参数：

```c
config->gyro_static_threshold_dps = 1.5f;
config->accel_static_threshold_g = 0.08f;
config->accel_lpf_alpha = 0.45f;
config->bias_alpha = 0.001f;
config->mahony_kp = 5.5f;
config->mahony_ki = 0.02f;
```

参数含义：

| 参数 | 作用 | 调大效果 | 调小效果 |
|---|---|---|---|
| gyro_static_threshold_dps | 静止判断陀螺仪阈值 | 更容易认为静止 | 更严格 |
| accel_static_threshold_g | 静止判断加速度阈值 | 更容易认为静止 | 更严格 |
| accel_lpf_alpha | 加速度低通滤波系数 | 动态响应更快，但噪声更大 | 更稳，但延迟更大 |
| bias_alpha | 零漂在线估计速度 | 零漂收敛更快，但运动时可能误估 | 更稳，但收敛慢 |
| mahony_kp | 加速度修正姿态强度 | 姿态回正快，动态更灵敏 | 更平滑，但响应慢 |
| mahony_ki | 积分修正强度 | 长期漂移更快被压住 | 更不容易积分过冲 |

## 9. 动态响应调参建议

如果感觉动态响应差，优先按下面顺序调：

1. 增大 `accel_lpf_alpha`，例如从 `0.45f` 改到 `0.55f` 或 `0.65f`。
2. 增大 `mahony_kp`，例如从 `5.5f` 改到 `6.5f` 或 `7.5f`。
3. 保持 `mahony_ki` 不要太大，避免积分导致姿态慢慢偏。
4. 确认 `APP_IMU_SAMPLE_DT_S` 和实际调用周期一致，当前为 `0.001f`，对应 1kHz。
5. 如果主循环被 OLED 或其他任务拖慢，需要考虑用定时器节拍或 RTOS 任务固定 IMU 更新周期。

如果感觉静止时抖动大，优先按下面顺序调：

1. 降低 `accel_lpf_alpha`，例如 `0.35f`。
2. 降低 `mahony_kp`，例如 `4.0f`。
3. 检查机械安装是否松动。
4. 降低 SPI 速率到 `/8` 验证通信噪声。
5. 增加上电静止校准时间。

如果感觉零漂明显，优先检查：

1. 上电时 IMU 是否保持静止。
2. `bias_alpha` 是否太小。
3. 静止判断阈值是否太严格，导致零漂估计没有启动。
4. 温升后是否需要更长时间在线估计。

## 10. 调试步骤

建议按以下顺序排查：

1. 确认板子能正常烧录，LED 或已有 OLED 逻辑正常。
2. 确认 `AppIMU_Init()` 返回状态为 OK。
3. 如果不是 OK，先读 `AppIMU_GetInitStatus()`。
4. 如果是 `WHOAMI_ERROR`，检查 SPI1、PA4 CS、电源和地。
5. 如果初始化 OK 但姿态不动，检查 `PC4` INT1 是否触发。
6. 如果 INT1 不触发，确认 `AppIMU_Process()` 的 1ms 轮询是否仍能更新数据。
7. 如果数据跳变很大，降低 SPI 速率并检查杜邦线长度。
8. 如果姿态方向反了，检查传感器安装方向和坐标轴映射。

## 11. 后续可加功能

当前工程已经完成 IMU 软件接入，但还没有强制把姿态数据显示出来。后续可选方向：

- OLED 显示 `Roll / Pitch / Yaw`。
- 串口发送 JustFloat 格式到上位机。
- 将 IMU 姿态用于舵机闭环控制。
- 用定时器中断或 RTOS 任务固定 IMU 更新周期。
- 根据车体实际安装方向增加坐标轴转换层。

## 12. 当前编译验证

当前工程已通过 CMake 编译验证：

```text
cmake --preset Debug
cmake --build C:\Users\ASUS\Desktop\match_2026_7\build\Debug
```

编译产物：

```text
C:\Users\ASUS\Desktop\match_2026_7\build\Debug\oled_f4.elf
```

如果之后用 CubeMX 重新生成代码，需要重新检查以下文件是否仍然保留 IMU 相关内容：

```text
Core/Src/main.c
Core/Src/gpio.c
Core/Src/spi.c
Core/Src/dma.c
Core/Src/stm32f4xx_it.c
Core/Inc/main.h
Core/Inc/stm32f4xx_it.h
CMakeLists.txt
cmake/stm32cubemx/CMakeLists.txt
```

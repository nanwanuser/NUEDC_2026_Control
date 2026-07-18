# HC-SR04 超声波模块驱动说明

本目录提供 HC-SR04 超声波测距模块驱动。当前设计分为两层：

- `HC-SR04.h / HC-SR04.c`：纯测距驱动，只负责 Trig/Echo 和距离计算
- `HC-SR04_OLED.h / HC-SR04_OLED.c`：可选 OLED 显示封装，负责把测距结果显示到 OLED

这样后续如果只想获取距离值，不需要依赖 OLED。

## 文件说明

- `HC-SR04.h`：测距驱动接口
- `HC-SR04.c`：测距驱动实现
- `HC-SR04_OLED.h`：OLED 显示封装接口
- `HC-SR04_OLED.c`：OLED 显示封装实现

## 当前硬件配置

默认引脚：

- Trig：`PA0`
- Echo：`PA1`

对应宏定义在 `HC-SR04.h` 中：

```c
#define HCSR04_TRIG_GPIO_Port      GPIOA
#define HCSR04_TRIG_Pin            GPIO_PIN_0
#define HCSR04_ECHO_GPIO_Port      GPIOA
#define HCSR04_ECHO_Pin            GPIO_PIN_1
```

如果更换引脚，修改这四个宏，并同步调整 CubeMX 配置。

## CubeMX 配置要求

你需要确认：

- `PA0` 配置为 GPIO Output
- `PA1` 配置为 GPIO Input
- GPIOA 时钟开启
- `PA0 / PA1` 没有被其他外设占用

当前驱动使用 Cortex-M4 的 DWT 周期计数器做微秒计时，不需要额外配置 TIM 定时器。

## 初始化

纯测距用法：

```c
#include "HC-SR04.h"

HCSR04_Init();
```

OLED 显示封装用法：

```c
#include "HC-SR04_OLED.h"

OLED_Init();
HCSR04_OLED_Init();
```

## 直接返回测量值

如果你只想直接获得距离值，使用下面两个函数：

```c
float distance_cm = HCSR04_GetDistanceCm();
float distance_mm = HCSR04_GetDistanceMm();
```

返回值说明：

- `>= 0.0f`：测量成功
- `-1.0f`：测量失败，宏名为 `HCSR04_INVALID_DISTANCE`

如果需要知道失败原因，可以读取最近一次状态：

```c
HCSR04_StatusTypeDef status = HCSR04_GetLastStatus();
```

状态值：

- `HCSR04_OK`：成功
- `HCSR04_TIMEOUT`：等待 Echo 超时
- `HCSR04_ERROR`：参数错误或其他错误

## 带状态码的测距接口

如果你希望明确判断是否成功，推荐使用：

```c
float distance_cm;

if (HCSR04_ReadDistanceCm(&distance_cm) == HCSR04_OK) {
    /* distance_cm 单位 cm */
}
```

其他接口：

```c
HCSR04_ReadPulseUs(&pulse_us);
HCSR04_ReadDistanceMm(&distance_mm);
HCSR04_ReadData(&data);
```

`HCSR04_ReadData()` 会同时返回高电平时间、cm 和 mm：

```c
HCSR04_DataTypeDef data;

if (HCSR04_ReadData(&data) == HCSR04_OK) {
    /* data.pulse_us */
    /* data.distance_cm */
    /* data.distance_mm */
}
```

## OLED 显示用法

如果你希望主函数里直接显示距离到 OLED：

```c
#include "HC-SR04_OLED.h"

OLED_Init();
HCSR04_OLED_Init();

while (1)
{
    HCSR04_OLED_Process();
}
```

`HCSR04_OLED_Process()` 内部会完成：

- 测距
- 判断状态
- OLED 清屏
- 显示 `Dist:xx.xcm`、`Timeout` 或 `Error`
- 延时 100 ms

## 典型纯测距示例

```c
#include "HC-SR04.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    HCSR04_Init();

    while (1)
    {
        float distance_cm = HCSR04_GetDistanceCm();

        if (distance_cm >= 0.0f) {
            /* 在这里使用 distance_cm */
        }

        HAL_Delay(100);
    }
}
```

## 注意事项

- HC-SR04 的 Echo 通常是 5 V 输出，STM32F407 输入建议做 5 V 到 3.3 V 分压或电平转换。
- 两次测距之间建议间隔至少 60 ms。
- 如果一直返回 `HCSR04_TIMEOUT`，优先检查接线、供电、Trig 脉冲和 Echo 电平。
- 当前测距实现是阻塞式，最长等待时间由 `HCSR04_TIMEOUT_US` 决定。

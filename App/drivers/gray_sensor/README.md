# 八路灰度传感器驱动说明

## 1. 模块说明

该模块内部带有数据选择器，单片机通过 `AD0`、`AD1`、`AD2` 写入三位通道地址，再从 `OUT` 读取对应通道的数字状态。

只需要四个 GPIO 就可以读取八路灰度传感器：

- `AD0`、`AD1`、`AD2`：通道地址输出
- `OUT`：当前通道状态输入

这里的 `AD0`、`AD1`、`AD2` 是地址选择引脚，不是 ADC 模拟输入引脚。

## 2. 文件位置

- 头文件：`App/drivers/gray_sensor/Gray_Sensor.h`
- 源文件：`App/drivers/gray_sensor/Gray_Sensor.c`

当前驱动支持：

- 选择指定通道
- 读取单路状态
- 连续读取八路状态
- 将八路状态打包为一个 `uint8_t`
- 配置 `OUT` 高电平有效或低电平有效

## 3. 接线说明

以下是源模块建议的引脚配置，当前工程尚未在 CubeMX 中启用：

| 模块引脚 | STM32 引脚 | GPIO 模式 | 说明 |
| --- | --- | --- | --- |
| `5V` | `5V` | 电源 | 模块供电 |
| `GND` | `GND` | 电源地 | 必须与 STM32 共地 |
| `AD0` | `PD9` | 推挽输出 | 地址最低位 |
| `AD1` | `PD10` | 推挽输出 | 地址中间位 |
| `AD2` | `PD11` | 推挽输出 | 地址最高位 |
| `OUT` | `PD8` | 输入 | 读取所选通道状态 |

模块使用 `5V` 供电不代表 `OUT` 一定是 `3.3V` 电平。首次接线前应测量 `OUT` 的高电平电压；如果输出为 `5V`，需要确认所用 STM32 引脚是否支持该电平，必要时增加电平转换或分压电路。

## 4. 通道地址

驱动按照三位二进制地址选择通道：

| 通道 | AD2 | AD1 | AD0 |
| --- | --- | --- | --- |
| 0 | 0 | 0 | 0 |
| 1 | 0 | 0 | 1 |
| 2 | 0 | 1 | 0 |
| 3 | 0 | 1 | 1 |
| 4 | 1 | 0 | 0 |
| 5 | 1 | 0 | 1 |
| 6 | 1 | 1 | 0 |
| 7 | 1 | 1 | 1 |

切换地址后，驱动内部会等待一小段时间，再读取 `OUT`，防止通道还没有稳定就采样。

## 5. GPIO 配置

在 CubeMX 中完成以下配置：

- `PD9`：`GPIO_Output`
- `PD10`：`GPIO_Output`
- `PD11`：`GPIO_Output`
- `PD8`：`GPIO_Input`
- 输出类型：`Push-Pull`
- 上下拉：先使用 `No pull-up and no pull-down`
- 输出速度：`Low`

当前工程尚未配置这四个引脚。使用前需要在 CubeMX 中启用 GPIOD 时钟并生成对应 GPIO 初始化代码。

## 6. 驱动配置

灰度模块数量在 `Gray_Sensor.h` 中设置：

```c
#define gray_sensor_count 1U
```

驱动内置弱空配置，默认不会操作任何 GPIO。完成板级引脚分配后，在板级配置源文件中提供同名强定义：

```c
gray_sensor_config Gray_Sensor_Config[gray_sensor_count] = {
    {GPIOD, GPIO_PIN_9,
     GPIOD, GPIO_PIN_10,
     GPIOD, GPIO_PIN_11,
     GPIOD, GPIO_PIN_8,
     GRAY_SENSOR_ACTIVE_LOW},
};
```

有效电平说明：

- `GRAY_SENSOR_ACTIVE_LOW`：`OUT` 为低电平时，驱动返回 `1`
- `GRAY_SENSOR_ACTIVE_HIGH`：`OUT` 为高电平时，驱动返回 `1`

示例使用低电平有效。返回值 `1` 具体代表黑线还是白色背景，取决于传感器阈值、电位器调整和安装方式，应以实测结果为准。

## 7. 使用方法

在 `main.c` 中包含头文件：

```c
#include "Gray_Sensor.h"
```

完成 HAL、时钟和 GPIO 初始化后，再初始化灰度模块：

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();

gray_sensor_init();
```

### 读取单路

```c
uint8_t channel_state;

channel_state = gray_sensor_read_channel(Gray_Sensor_Config[0], 3U);
```

上面代码选择通道 3，并返回该通道的逻辑状态 `0` 或 `1`。

### 读取全部通道

```c
uint8_t gray_data[8];

gray_sensor_read_all(Gray_Sensor_Config[0], gray_data);
```

读取完成后：

- `gray_data[0]` 对应通道 0
- `gray_data[1]` 对应通道 1
- 依次类推
- `gray_data[7]` 对应通道 7

### 打包读取八路

巡线时建议直接使用打包读取：

```c
uint8_t gray_state;

gray_state = gray_sensor_read_byte(Gray_Sensor_Config[0]);
```

返回值中 `bit0 ~ bit7` 分别对应通道 `0 ~ 7`。例如返回 `0x18`，表示通道 3 和通道 4 的逻辑状态为 `1`。

## 8. 接口说明

### `void gray_sensor_init(void);`

初始化全部灰度模块，把地址线设置为通道 0。

### `void gray_sensor_set_channel(gray_sensor_config config, uint8_t channel);`

选择通道，范围为 `0 ~ 7`。传入大于 7 的值时，驱动会自动限制为通道 7。

### `uint8_t gray_sensor_read_channel(gray_sensor_config config, uint8_t channel);`

选择并读取指定通道，返回经过有效电平转换后的 `0` 或 `1`。

### `uint8_t gray_sensor_read_now(gray_sensor_config config);`

不切换通道，直接读取当前 `OUT`，并按照有效电平配置转换为 `0` 或 `1`。

### `void gray_sensor_read_all(gray_sensor_config config, uint8_t *buffer);`

依次读取八个通道，将结果写入长度至少为 8 的数组。

### `uint8_t gray_sensor_read_byte(gray_sensor_config config);`

依次读取八个通道，并将结果打包到一个字节中。

## 9. 调试建议

第一次上电时，先固定读取一个通道并观察返回值，再依次检查八路：

```c
uint8_t test_state;

test_state = gray_sensor_read_channel(Gray_Sensor_Config[0], 0U);
```

如果返回逻辑相反，修改 `Gray_Sensor_Config` 中的有效电平，不需要改读取函数。

如果八路数据始终相同，优先检查：

- `AD0`、`AD1`、`AD2` 是否配置为 GPIO 输出
- 三根地址线是否按照通道号正常翻转
- `OUT` 是否配置为 GPIO 输入
- STM32 和模块是否共地
- 模块阈值是否调整合适
- 通道编号与传感器从左到右的物理顺序是否一致

传感器从左到右对应通道 0 还是通道 7，需要根据模块丝印或实测确认。驱动只保证通道号和地址值一一对应，不对物理安装方向做额外转换。

# OLED 驱动使用说明

本目录提供 0.96 寸 I2C OLED 屏幕驱动，适用于常见 SSD1306 兼容屏，分辨率按 128x64 设计。

## 文件说明

- `OLED.h`：对外显示、绘图和显存控制接口
- `OLED.c`：OLED 初始化、I2C 通信、显存刷新和绘图实现
- `OLED_Data.h`：字模和图片数据声明
- `OLED_Data.c`：ASCII 字模、中文字符字模和图片数据

## 当前硬件配置

当前默认使用软件 I2C：

```c
//#define OLED_USE_HW_I2C
#define OLED_USE_SW_I2C
```
默认引脚来自 main.h：
```c
SCL：PB6，宏名 I2C3_SCL_Pin
SDA：PB7，宏名 I2C3_SDA_Pin
```
OLED 地址：0x3C << 1，即 0x78
CubeMX 中 SCL 和 SDA 引脚 user label 需要设置为：
```c
I2C3_SCL
I2C3_SDA
```
初始化顺序
```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
OLED_Init();
```
OLED_Init() 会初始化软件 I2C 引脚、发送 OLED 初始化命令、清屏并刷新屏幕。
显存和刷新机制
大部分显示和绘图函数只修改显存，不会立刻显示到屏幕。
修改内容后需要调用：
```c
OLED_Update();
```
局部刷新可以调用：
```c
OLED_UpdateArea(x, y, width, height);
```
常用接口

清屏
```c
OLED_Clear();
OLED_Update();
```
显示字符串
```c
OLED_ShowString(0, 0, "Hello", OLED_8X16);
OLED_Update();
```
支持字号：
```c
OLED_8X16
OLED_6X8
```
显示数字
```c
OLED_ShowNum(0, 0, 1234, 4, OLED_8X16);
OLED_ShowSignedNum(0, 16, -56, 2, OLED_8X16);
OLED_ShowHexNum(0, 32, 0xABCD, 4, OLED_8X16);
OLED_Update();
```
格式化显示
```c
OLED_Printf(0, 0, OLED_8X16, "V=%d", 12);
OLED_Update();
```
绘图
```c
OLED_DrawPoint(10, 10);
OLED_DrawLine(0, 0, 127, 63);
OLED_DrawRectangle(0, 0, 128, 64, OLED_UNFILLED);
OLED_DrawCircle(64, 32, 20, OLED_UNFILLED);
OLED_Update();
```
填充图形：
```c
OLED_DrawRectangle(0, 0, 128, 64, OLED_FILLED);
OLED_Update();
```
显示中文
```c
OLED_ShowChinese(0, 0, "你好");
OLED_Update();
```
注意：中文必须已经添加到 OLED_Data.c 的 OLED_CF16x16 字模表中。
完整示例
```c
OLED_Init();
OLED_Clear();
OLED_ShowString(0, 0, "OLED OK", OLED_8X16);
OLED_DrawRectangle(0, 20, 80, 24, OLED_UNFILLED);
OLED_Update();
```
切换到硬件 I2C
在 OLED.c 中改为：
```c
#define OLED_USE_HW_I2C
//#define OLED_USE_SW_I2C
```
同时确认：
CubeMX 已启用对应 I2C 外设
OLED_I2C 指向正确的 I2C_HandleTypeDef
调用 OLED_Init() 前已完成 I2C 初始化

当前代码默认硬件 I2C 句柄为：
```c
#define OLED_I2C hi2c3
extern I2C_HandleTypeDef hi2c3;
```
##注意事项
    坐标原点在左上角，X 范围 0~127，Y 范围 0~63。
    写入字符、数字、图形后，要调用 OLED_Update() 才会显示。
    软件 I2C 建议外接上拉电阻。
    如果 OLED 地址不是 0x3C，需要修改地址。
    当前软件 I2C 未处理 ACK，接线或地址错误时不会主动报错。
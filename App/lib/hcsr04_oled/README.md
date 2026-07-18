# HC-SR04 OLED 显示组合库

该可选模块组合了 HC-SR04 驱动和 OLED 驱动。由于它同时依赖两个底层驱动，
本身不属于硬件驱动，因此放在 `App/lib` 中。

先调用 `OLED_Init()`，再调用 `HCSR04_OLED_Init()`；随后从应用代码或任务中
周期调用 `HCSR04_OLED_Process()`。

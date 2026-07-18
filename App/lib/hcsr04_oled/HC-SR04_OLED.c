#include "HC-SR04_OLED.h"
#include "HC-SR04.h"
#include "OLED.h"

void HCSR04_OLED_Init(void)
{
    HCSR04_Init();

    OLED_Clear();
    OLED_ShowString(0, 0, "HC-SR04 Ready", OLED_8X16);
    OLED_Update();
}

void HCSR04_OLED_Process(void)
{
    float distance_cm = 0.0f;
    HCSR04_StatusTypeDef status;

    status = HCSR04_ReadDistanceCm(&distance_cm);

    OLED_Clear();
    OLED_ShowString(0, 0, "HC-SR04", OLED_8X16);

    if (status == HCSR04_OK) {
        uint32_t distance_x10 = (uint32_t)(distance_cm * 10.0f + 0.5f);
        OLED_Printf(0, 24, OLED_8X16, "Dist:%lu.%lucm",
                    (unsigned long)(distance_x10 / 10U),
                    (unsigned long)(distance_x10 % 10U));
    } else if (status == HCSR04_TIMEOUT) {
        OLED_ShowString(0, 24, "Timeout", OLED_8X16);
    } else {
        OLED_ShowString(0, 24, "Error", OLED_8X16);
    }

    OLED_Update();
    HAL_Delay(100);
}

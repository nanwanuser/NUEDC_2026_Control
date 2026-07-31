#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

#define __weak __attribute__((weak))

typedef enum {
    HAL_OK = 0,
    HAL_ERROR
} HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

#endif

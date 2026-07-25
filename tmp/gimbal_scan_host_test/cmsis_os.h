#ifndef CMSIS_OS_H
#define CMSIS_OS_H

#include <stdint.h>

typedef enum {
    osOK = 0
} osStatus_t;

osStatus_t osDelay(uint32_t ticks);
void osThreadExit(void);

#endif

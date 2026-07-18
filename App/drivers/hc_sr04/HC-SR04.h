#ifndef OLED_F4_HC_SR04_H
#define OLED_F4_HC_SR04_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stddef.h>
#include <stdint.h>

/* Hardware mapping ----------------------------------------------------------*/
#define HCSR04_TRIG_GPIO_Port      GPIOA
#define HCSR04_TRIG_Pin            GPIO_PIN_0
#define HCSR04_ECHO_GPIO_Port      GPIOA
#define HCSR04_ECHO_Pin            GPIO_PIN_1

/* Measurement parameters ----------------------------------------------------*/
#define HCSR04_TRIGGER_US          10U
#define HCSR04_TIMEOUT_US          30000U
#define HCSR04_SOUND_SPEED_CM_US   0.0343f
#define HCSR04_INVALID_DISTANCE    (-1.0f)

typedef enum {
    HCSR04_OK = 0,
    HCSR04_ERROR,
    HCSR04_TIMEOUT
} HCSR04_StatusTypeDef;

typedef struct {
    uint32_t pulse_us;
    float distance_cm;
    float distance_mm;
} HCSR04_DataTypeDef;

void HCSR04_Init(void);
HCSR04_StatusTypeDef HCSR04_ReadPulseUs(uint32_t *pulse_us);
HCSR04_StatusTypeDef HCSR04_ReadDistanceCm(float *distance_cm);
HCSR04_StatusTypeDef HCSR04_ReadDistanceMm(float *distance_mm);
HCSR04_StatusTypeDef HCSR04_ReadData(HCSR04_DataTypeDef *data);
float HCSR04_GetDistanceCm(void);
float HCSR04_GetDistanceMm(void);
HCSR04_StatusTypeDef HCSR04_GetLastStatus(void);
float HCSR04_ReadDistanceCm_Blocking(void);
float HCSR04_ReadDistanceMm_Blocking(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_F4_HC_SR04_H */

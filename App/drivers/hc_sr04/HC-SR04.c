#include "HC-SR04.h"

static uint32_t hcsr04_cycles_per_us = 0U;
static HCSR04_StatusTypeDef hcsr04_last_status = HCSR04_ERROR;

static void HCSR04_DWT_Init(void)
{
    if (hcsr04_cycles_per_us == 0U) {
        hcsr04_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
        if (hcsr04_cycles_per_us == 0U) {
            hcsr04_cycles_per_us = 1U;
        }
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void HCSR04_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * hcsr04_cycles_per_us;

    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    }
}

static uint8_t HCSR04_WaitPinState(GPIO_PinState state, uint32_t timeout_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout_ticks = timeout_us * hcsr04_cycles_per_us;

    while (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) != state) {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_ticks) {
            return 0U;
        }
    }

    return 1U;
}

static void HCSR04_SendTrigger(void)
{
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    HCSR04_DelayUs(2U);

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    HCSR04_DelayUs(HCSR04_TRIGGER_US);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
}

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    HCSR04_DWT_Init();

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = HCSR04_TRIG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HCSR04_TRIG_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HCSR04_ECHO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HCSR04_ECHO_GPIO_Port, &GPIO_InitStruct);
}

HCSR04_StatusTypeDef HCSR04_ReadPulseUs(uint32_t *pulse_us)
{
    uint32_t echo_start;
    uint32_t echo_ticks;

    if (pulse_us == NULL) {
        hcsr04_last_status = HCSR04_ERROR;
        return HCSR04_ERROR;
    }

    HCSR04_DWT_Init();
    *pulse_us = 0U;

    if (!HCSR04_WaitPinState(GPIO_PIN_RESET, HCSR04_TIMEOUT_US)) {
        hcsr04_last_status = HCSR04_TIMEOUT;
        return HCSR04_TIMEOUT;
    }

    HCSR04_SendTrigger();

    if (!HCSR04_WaitPinState(GPIO_PIN_SET, HCSR04_TIMEOUT_US)) {
        hcsr04_last_status = HCSR04_TIMEOUT;
        return HCSR04_TIMEOUT;
    }

    echo_start = DWT->CYCCNT;

    if (!HCSR04_WaitPinState(GPIO_PIN_RESET, HCSR04_TIMEOUT_US)) {
        hcsr04_last_status = HCSR04_TIMEOUT;
        return HCSR04_TIMEOUT;
    }

    echo_ticks = (uint32_t)(DWT->CYCCNT - echo_start);
    *pulse_us = echo_ticks / hcsr04_cycles_per_us;

    hcsr04_last_status = HCSR04_OK;
    return HCSR04_OK;
}

HCSR04_StatusTypeDef HCSR04_ReadDistanceCm(float *distance_cm)
{
    uint32_t pulse_us;
    HCSR04_StatusTypeDef status;

    if (distance_cm == NULL) {
        hcsr04_last_status = HCSR04_ERROR;
        return HCSR04_ERROR;
    }

    *distance_cm = HCSR04_INVALID_DISTANCE;
    status = HCSR04_ReadPulseUs(&pulse_us);
    if (status != HCSR04_OK) {
        return status;
    }

    *distance_cm = ((float)pulse_us * HCSR04_SOUND_SPEED_CM_US) / 2.0f;
    return HCSR04_OK;
}

HCSR04_StatusTypeDef HCSR04_ReadDistanceMm(float *distance_mm)
{
    float distance_cm;
    HCSR04_StatusTypeDef status;

    if (distance_mm == NULL) {
        hcsr04_last_status = HCSR04_ERROR;
        return HCSR04_ERROR;
    }

    *distance_mm = HCSR04_INVALID_DISTANCE;
    status = HCSR04_ReadDistanceCm(&distance_cm);
    if (status != HCSR04_OK) {
        return status;
    }

    *distance_mm = distance_cm * 10.0f;
    return HCSR04_OK;
}

HCSR04_StatusTypeDef HCSR04_ReadData(HCSR04_DataTypeDef *data)
{
    HCSR04_StatusTypeDef status;

    if (data == NULL) {
        hcsr04_last_status = HCSR04_ERROR;
        return HCSR04_ERROR;
    }

    data->pulse_us = 0U;
    data->distance_cm = HCSR04_INVALID_DISTANCE;
    data->distance_mm = HCSR04_INVALID_DISTANCE;

    status = HCSR04_ReadPulseUs(&data->pulse_us);
    if (status != HCSR04_OK) {
        return status;
    }

    data->distance_cm = ((float)data->pulse_us * HCSR04_SOUND_SPEED_CM_US) / 2.0f;
    data->distance_mm = data->distance_cm * 10.0f;

    return HCSR04_OK;
}

float HCSR04_GetDistanceCm(void)
{
    float distance_cm;

    if (HCSR04_ReadDistanceCm(&distance_cm) != HCSR04_OK) {
        return HCSR04_INVALID_DISTANCE;
    }

    return distance_cm;
}

float HCSR04_GetDistanceMm(void)
{
    float distance_mm;

    if (HCSR04_ReadDistanceMm(&distance_mm) != HCSR04_OK) {
        return HCSR04_INVALID_DISTANCE;
    }

    return distance_mm;
}

HCSR04_StatusTypeDef HCSR04_GetLastStatus(void)
{
    return hcsr04_last_status;
}

float HCSR04_ReadDistanceCm_Blocking(void)
{
    return HCSR04_GetDistanceCm();
}

float HCSR04_ReadDistanceMm_Blocking(void)
{
    return HCSR04_GetDistanceMm();
}

#ifndef GIMBAL_CTRL_VISION_H
#define GIMBAL_CTRL_VISION_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Receive, validate, and submit one MaixCAM vision frame from USART1.
 * @param timeout_ms Overall blocking timeout in milliseconds.
 * @retval HAL_OK A valid target or target-lost frame was submitted.
 * @retval HAL_TIMEOUT No complete valid frame arrived before timeout.
 * @retval HAL_BUSY USART1 is already receiving data.
 */
HAL_StatusTypeDef gimbal_vision_receive(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif

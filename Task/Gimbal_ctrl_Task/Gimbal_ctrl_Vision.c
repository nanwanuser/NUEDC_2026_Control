#include "Gimbal_ctrl_Vision.h"

#include "Gimbal_ctrl_Task.h"
#include "usart.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>

#define GIMBAL_VISION_FRAME_SIZE 8U
#define GIMBAL_VISION_HEAD1 0xAAU
#define GIMBAL_VISION_HEAD2 0x55U
#define GIMBAL_VISION_STATUS_INDEX 2U
#define GIMBAL_VISION_DATA_INDEX 3U
#define GIMBAL_VISION_CHECKSUM_INDEX 7U

static int16_t gimbal_vision_decode_i16(uint8_t low, uint8_t high)
{
    uint16_t raw = (uint16_t)low | ((uint16_t)high << 8U);
    if (raw <= INT16_MAX) {
        return (int16_t)raw;
    }
    return (int16_t)((int32_t)raw - ((int32_t)UINT16_MAX + 1L));
}

static uint8_t gimbal_vision_checksum(const uint8_t *frame)
{
    uint8_t checksum = 0U;
    size_t index;
    for (index = 0U; index < GIMBAL_VISION_CHECKSUM_INDEX; index++) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum;
}

static bool gimbal_vision_frame_is_valid(const uint8_t *frame)
{
    size_t index;
    uint8_t status = frame[GIMBAL_VISION_STATUS_INDEX];
    if (gimbal_vision_checksum(frame) != frame[GIMBAL_VISION_CHECKSUM_INDEX]) {
        return false;
    }
    if (status == GIMBAL_VISION_STATUS_VALID) {
        return true;
    }
    if (status != GIMBAL_VISION_STATUS_LOST) {
        return false;
    }
    for (index = GIMBAL_VISION_DATA_INDEX;
         index < GIMBAL_VISION_CHECKSUM_INDEX; index++) {
        if (frame[index] != UINT8_MAX) {
            return false;
        }
    }
    return true;
}

static uint8_t gimbal_vision_append_byte(uint8_t *frame, uint8_t index,
                                         uint8_t byte)
{
    if (index == 0U) {
        frame[0] = byte;
        return byte == GIMBAL_VISION_HEAD1 ? 1U : 0U;
    }
    if (index == 1U && byte != GIMBAL_VISION_HEAD2) {
        frame[0] = byte;
        return byte == GIMBAL_VISION_HEAD1 ? 1U : 0U;
    }
    frame[index] = byte;
    return (uint8_t)(index + 1U);
}

static void gimbal_vision_submit(const uint8_t *frame)
{
    int16_t x = gimbal_vision_decode_i16(frame[3], frame[4]);
    int16_t y = gimbal_vision_decode_i16(frame[5], frame[6]);
    gimbal_ctrl_vision_input(frame[GIMBAL_VISION_STATUS_INDEX], x, y);
}

static uint32_t gimbal_vision_remaining(uint32_t start_tick,
                                        uint32_t timeout_ms)
{
    uint32_t elapsed = HAL_GetTick() - start_tick;
    return elapsed < timeout_ms ? timeout_ms - elapsed : 0U;
}

static void gimbal_vision_clear_overrun(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

HAL_StatusTypeDef gimbal_vision_receive(uint32_t timeout_ms)
{
    uint8_t frame[GIMBAL_VISION_FRAME_SIZE];
    uint8_t index = 0U;
    uint8_t byte;
    uint32_t start_tick = HAL_GetTick();
    uint32_t remaining;
    HAL_StatusTypeDef status;
    gimbal_vision_clear_overrun();
    while ((remaining = gimbal_vision_remaining(start_tick, timeout_ms)) > 0U) {
        status = HAL_UART_Receive(&huart1, &byte, 1U, remaining);
        if (status != HAL_OK) {
            return status;
        }
        index = gimbal_vision_append_byte(frame, index, byte);
        if (index == GIMBAL_VISION_FRAME_SIZE) {
            if (gimbal_vision_frame_is_valid(frame)) {
                gimbal_vision_submit(frame);
                return HAL_OK;
            }
            index = 0U;
        }
    }
    return HAL_TIMEOUT;
}

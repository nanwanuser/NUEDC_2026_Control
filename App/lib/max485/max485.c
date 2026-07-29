#include "max485.h"

#include <stdbool.h>
#include <string.h>

typedef struct {
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    uint32_t last_tx_tick;
    bool initialized;
    bool has_transmitted;
} max485_context_t;

static max485_context_t s_max485;

static void max485_set_direction(GPIO_PinState level)
{
    HAL_GPIO_WritePin(s_max485.enable_port, s_max485.enable_pin, level);
}

static max485_status_t max485_from_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_TIMEOUT) {
        return MAX485_STATUS_TIMEOUT;
    }
    if (status == HAL_BUSY) {
        return MAX485_STATUS_UART_BUSY;
    }
    return MAX485_STATUS_UART_ERROR;
}

static uint32_t max485_remaining_timeout(uint32_t start_tick,
                                         uint32_t timeout_ms)
{
    uint32_t elapsed_ms;

    if (timeout_ms == HAL_MAX_DELAY) {
        return HAL_MAX_DELAY;
    }
    elapsed_ms = HAL_GetTick() - start_tick;
    return elapsed_ms >= timeout_ms ? 0U : timeout_ms - elapsed_ms;
}

static void max485_wait_frame_interval(void)
{
    uint32_t elapsed_ms;

    if (!s_max485.has_transmitted) {
        return;
    }
    elapsed_ms = HAL_GetTick() - s_max485.last_tx_tick;
    if (elapsed_ms < MAX485_MIN_FRAME_INTERVAL_MS) {
        HAL_Delay(MAX485_MIN_FRAME_INTERVAL_MS - elapsed_ms);
    }
}

static max485_status_t max485_wait_for_header(uint8_t *header,
                                              uint32_t start_tick,
                                              uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    uint32_t remaining_ms;

    do {
        remaining_ms = max485_remaining_timeout(start_tick, timeout_ms);
        if (remaining_ms == 0U) {
            return MAX485_STATUS_TIMEOUT;
        }
        hal_status = HAL_UART_Receive(s_max485.uart, header, 1U, remaining_ms);
        if (hal_status != HAL_OK) {
            return max485_from_hal_status(hal_status);
        }
    } while (*header != MAX485_FRAME_HEADER);

    return MAX485_STATUS_OK;
}

static max485_status_t max485_collect_until_idle(uint8_t *raw_frame,
                                                 uint16_t *raw_length,
                                                 uint32_t start_tick,
                                                 uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    uint32_t remaining_ms;
    uint32_t byte_timeout_ms;

    while (*raw_length < MAX485_MAX_FRAME_LENGTH) {
        remaining_ms = max485_remaining_timeout(start_tick, timeout_ms);
        if (remaining_ms == 0U) {
            return MAX485_STATUS_TIMEOUT;
        }
        byte_timeout_ms = remaining_ms < MAX485_INTER_BYTE_TIMEOUT_MS
                              ? remaining_ms
                              : MAX485_INTER_BYTE_TIMEOUT_MS;
        hal_status = HAL_UART_Receive(s_max485.uart,
                                      &raw_frame[*raw_length],
                                      1U,
                                      byte_timeout_ms);
        if (hal_status == HAL_TIMEOUT) {
            return MAX485_STATUS_OK;
        }
        if (hal_status != HAL_OK) {
            return max485_from_hal_status(hal_status);
        }
        (*raw_length)++;
    }

    return raw_frame[*raw_length - 1U] == MAX485_FRAME_TAIL
               ? MAX485_STATUS_OK
               : MAX485_STATUS_FRAME_OVERFLOW;
}

void max485_init(UART_HandleTypeDef *uart,
                 GPIO_TypeDef *enable_port,
                 uint16_t enable_pin)
{
    s_max485 = (max485_context_t) {
        .uart = uart,
        .enable_port = enable_port,
        .enable_pin = enable_pin,
        .initialized = uart != NULL && enable_port != NULL,
    };
    if (s_max485.initialized) {
        max485_set_direction(MAX485_RX_ENABLE_LEVEL);
    }
}

uint8_t max485_calculate_checksum(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint8_t checksum = 0U;

    if (data == NULL) {
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        checksum = (uint8_t)(checksum + data[index]);
    }
    return checksum;
}

max485_status_t max485_pack_frame(uint8_t address,
                                 uint8_t function,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 uint8_t *output,
                                 uint16_t output_capacity,
                                 uint16_t *output_length)
{
    uint16_t frame_length = payload_length + MAX485_FRAME_OVERHEAD;

    if (output == NULL || output_length == NULL ||
        (payload == NULL && payload_length > 0U)) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    if (payload_length > MAX485_MAX_PAYLOAD_LENGTH) {
        return MAX485_STATUS_PAYLOAD_TOO_LONG;
    }
    if (output_capacity < frame_length) {
        return MAX485_STATUS_BUFFER_TOO_SMALL;
    }

    output[0] = MAX485_FRAME_HEADER;
    output[1] = address;
    output[2] = function;
    if (payload_length > 0U) {
        memcpy(&output[3], payload, payload_length);
    }
    output[3U + payload_length] =
        max485_calculate_checksum(output, 3U + payload_length);
    output[4U + payload_length] = MAX485_FRAME_TAIL;
    *output_length = frame_length;
    return MAX485_STATUS_OK;
}

max485_status_t max485_send_frame(uint8_t address,
                                 uint8_t function,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 uint32_t timeout_ms)
{
    uint8_t raw_frame[MAX485_MAX_FRAME_LENGTH];
    uint16_t raw_length;
    max485_status_t status;
    HAL_StatusTypeDef hal_status;

    if (!s_max485.initialized) {
        return MAX485_STATUS_NOT_INITIALIZED;
    }
    status = max485_pack_frame(address, function, payload, payload_length,
                               raw_frame, sizeof(raw_frame), &raw_length);
    if (status != MAX485_STATUS_OK) {
        return status;
    }

    max485_wait_frame_interval();
    max485_set_direction(MAX485_TX_ENABLE_LEVEL);
    hal_status = HAL_UART_Transmit(s_max485.uart, raw_frame, raw_length,
                                   timeout_ms);
    max485_set_direction(MAX485_RX_ENABLE_LEVEL);
    s_max485.last_tx_tick = HAL_GetTick();
    s_max485.has_transmitted = true;
    return hal_status == HAL_OK ? MAX485_STATUS_OK
                                : max485_from_hal_status(hal_status);
}

max485_status_t max485_unpack_frame(const uint8_t *raw_frame,
                                   uint16_t raw_length,
                                   max485_frame_t *frame)
{
    uint16_t payload_length;
    uint8_t expected_checksum;

    if (raw_frame == NULL || frame == NULL) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }
    if (raw_length < MAX485_FRAME_OVERHEAD ||
        raw_frame[0] != MAX485_FRAME_HEADER ||
        raw_frame[raw_length - 1U] != MAX485_FRAME_TAIL) {
        return MAX485_STATUS_FRAME_FORMAT_ERROR;
    }

    payload_length = raw_length - MAX485_FRAME_OVERHEAD;
    if (payload_length > MAX485_MAX_PAYLOAD_LENGTH) {
        return MAX485_STATUS_PAYLOAD_TOO_LONG;
    }
    expected_checksum =
        max485_calculate_checksum(raw_frame, raw_length - 2U);
    if (raw_frame[raw_length - 2U] != expected_checksum) {
        return MAX485_STATUS_CHECKSUM_ERROR;
    }

    frame->address = raw_frame[1];
    frame->function = raw_frame[2];
    frame->payload_length = payload_length;
    if (payload_length > 0U) {
        memcpy(frame->payload, &raw_frame[3], payload_length);
    }
    return MAX485_STATUS_OK;
}

max485_status_t max485_receive_frame(max485_frame_t *frame,
                                    uint32_t timeout_ms)
{
    uint8_t raw_frame[MAX485_MAX_FRAME_LENGTH];
    uint16_t raw_length = 1U;
    uint32_t start_tick;
    max485_status_t status;

    if (!s_max485.initialized) {
        return MAX485_STATUS_NOT_INITIALIZED;
    }
    if (frame == NULL || timeout_ms == 0U) {
        return MAX485_STATUS_INVALID_ARGUMENT;
    }

    max485_set_direction(MAX485_RX_ENABLE_LEVEL);
    start_tick = HAL_GetTick();
    status = max485_wait_for_header(&raw_frame[0], start_tick, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    status = max485_collect_until_idle(raw_frame, &raw_length,
                                       start_tick, timeout_ms);
    if (status != MAX485_STATUS_OK) {
        return status;
    }
    return max485_unpack_frame(raw_frame, raw_length, frame);
}

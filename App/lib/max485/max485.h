#ifndef MAX485_H
#define MAX485_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX485_FRAME_HEADER 0xC5U
#define MAX485_FRAME_TAIL 0x5CU
#define MAX485_FRAME_OVERHEAD 5U
#define MAX485_MAX_PAYLOAD_LENGTH 64U
#define MAX485_MAX_FRAME_LENGTH \
    (MAX485_MAX_PAYLOAD_LENGTH + MAX485_FRAME_OVERHEAD)

#ifndef MAX485_TX_ENABLE_LEVEL
#define MAX485_TX_ENABLE_LEVEL GPIO_PIN_SET
#endif

#ifndef MAX485_RX_ENABLE_LEVEL
#define MAX485_RX_ENABLE_LEVEL GPIO_PIN_RESET
#endif

#ifndef MAX485_MIN_FRAME_INTERVAL_MS
#define MAX485_MIN_FRAME_INTERVAL_MS 2U
#endif

#ifndef MAX485_INTER_BYTE_TIMEOUT_MS
#define MAX485_INTER_BYTE_TIMEOUT_MS 2U
#endif

typedef enum {
    MAX485_STATUS_OK = 0,
    MAX485_STATUS_INVALID_ARGUMENT,
    MAX485_STATUS_NOT_INITIALIZED,
    MAX485_STATUS_BUFFER_TOO_SMALL,
    MAX485_STATUS_PAYLOAD_TOO_LONG,
    MAX485_STATUS_UART_BUSY,
    MAX485_STATUS_UART_ERROR,
    MAX485_STATUS_TIMEOUT,
    MAX485_STATUS_FRAME_OVERFLOW,
    MAX485_STATUS_FRAME_FORMAT_ERROR,
    MAX485_STATUS_CHECKSUM_ERROR,
    MAX485_STATUS_UNEXPECTED_FRAME,
} max485_status_t;

typedef struct {
    uint8_t address;
    uint8_t function;
    uint8_t payload[MAX485_MAX_PAYLOAD_LENGTH];
    uint16_t payload_length;
} max485_frame_t;

/**
 * @brief Bind the MAX485 transport to one UART and one direction pin.
 * @note TX is active at MAX485_TX_ENABLE_LEVEL and RX at
 *       MAX485_RX_ENABLE_LEVEL.
 */
void max485_init(UART_HandleTypeDef *uart,
                 GPIO_TypeDef *enable_port,
                 uint16_t enable_pin);

/**
 * @brief Calculate the protocol 8-bit additive checksum.
 * @param data Bytes from the frame header through the final payload byte.
 * @param length Number of bytes included in the checksum.
 */
uint8_t max485_calculate_checksum(const uint8_t *data, uint16_t length);

/**
 * @brief Pack a variable-length custom-protocol frame.
 * @param payload May be NULL only when payload_length is zero.
 * @param output_length Receives payload_length + 5 on success.
 */
max485_status_t max485_pack_frame(uint8_t address,
                                 uint8_t function,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 uint8_t *output,
                                 uint16_t output_capacity,
                                 uint16_t *output_length);

/**
 * @brief Pack and send one complete frame in blocking mode.
 * @note The function restores the transceiver to RX mode before returning and
 *       enforces the manual's minimum 2 ms interval between transmitted frames.
 */
max485_status_t max485_send_frame(uint8_t address,
                                 uint8_t function,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 uint32_t timeout_ms);

/**
 * @brief Validate and unpack one complete variable-length frame.
 */
max485_status_t max485_unpack_frame(const uint8_t *raw_frame,
                                   uint16_t raw_length,
                                   max485_frame_t *frame);

/**
 * @brief Receive and unpack one frame with blocking single-byte UART reads.
 * @note An idle gap marks the end of the variable-length frame. Do not call
 *       concurrently from multiple tasks without external serialization.
 */
max485_status_t max485_receive_frame(max485_frame_t *frame,
                                    uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif

#ifndef VISION_UART_H
#define VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "decision.h"

typedef enum {
    VISION_UART_STATE_IDLE = 0,
    VISION_UART_STATE_RECEIVING,
    VISION_UART_STATE_STABLE,
    VISION_UART_STATE_SUBMITTED,
    VISION_UART_STATE_ERROR
} VisionUartState;

typedef struct {
    VisionUartState state;
    uint16_t last_seq;
    uint8_t stable_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t dropped_byte_count;
} VisionUartOutput;

void VisionUart_Init(void);
uint8_t VisionUart_SetFixedLayout(const DecisionFixedLayout *layout);
void VisionUart_GetOutput(VisionUartOutput *output);
uint8_t VisionUart_ReceiveAndSubmit(void);

#ifdef __cplusplus
}
#endif

#endif

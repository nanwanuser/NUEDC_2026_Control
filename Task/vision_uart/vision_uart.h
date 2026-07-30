#ifndef VISION_UART_H
#define VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "decision.h"
#include "decision_task.h"

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
    /* Mirrors the armed request so the caller can tell which run this is. */
    uint32_t arm_id;
} VisionUartOutput;

void VisionUart_Init(void);
uint8_t VisionUart_SetFixedLayout(const DecisionFixedLayout *layout);

/* Arms one acquisition. The task stays idle with the receiver off until this
   is called, so a key press is what starts the contest run. The submitted
   DecisionTaskRequest is base_request with mode, vision and fixed_layout
   replaced by what the camera actually sent. */
uint8_t VisionUart_Arm(const DecisionTaskRequest *base_request, uint32_t arm_id);
void VisionUart_Abort(void);

void VisionUart_GetOutput(VisionUartOutput *output);
void VisionUart_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

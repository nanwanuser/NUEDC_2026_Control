#ifndef VISION_MODE_RETRY_H
#define VISION_MODE_RETRY_H

#include "vision_protocol.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_MODE_RETRY_MAX_ATTEMPTS 5U

typedef struct {
    uint32_t last_send_tick;
    uint8_t attempt_count;
    uint8_t response_seen;
} VisionModeRetry;

void VisionModeRetry_Arm(VisionModeRetry *retry, uint32_t now_tick);
uint8_t VisionModeRetry_TakeDue(
    VisionModeRetry *retry,
    uint32_t now_tick,
    uint32_t retry_ticks);
void VisionModeRetry_Stop(VisionModeRetry *retry);
uint8_t VisionModeRetry_ResultMatches(
    DecisionStrategy strategy,
    VisionProtocolResult result);

#ifdef __cplusplus
}
#endif

#endif

#include "vision_mode_retry.h"

#include <stddef.h>

void VisionModeRetry_Arm(VisionModeRetry *retry, uint32_t now_tick)
{
    if (retry == NULL) {
        return;
    }
    retry->last_send_tick = now_tick;
    retry->attempt_count = 1U;
    retry->response_seen = 0U;
}

uint8_t VisionModeRetry_TakeDue(VisionModeRetry *retry,
                                uint32_t now_tick,
                                uint32_t retry_ticks)
{
    if (retry == NULL || retry->response_seen != 0U ||
        retry->attempt_count >= VISION_MODE_RETRY_MAX_ATTEMPTS ||
        (now_tick - retry->last_send_tick) < retry_ticks) {
        return 0U;
    }

    retry->last_send_tick = now_tick;
    ++retry->attempt_count;
    return 1U;
}

void VisionModeRetry_Stop(VisionModeRetry *retry)
{
    if (retry != NULL) {
        retry->response_seen = 1U;
    }
}

uint8_t VisionModeRetry_ResultMatches(DecisionStrategy strategy,
                                      VisionProtocolResult result)
{
    if (strategy == DECISION_STRATEGY_GEOMETRIC) {
        return result == VISION_PROTOCOL_RESULT_FRAME;
    }
    if (strategy == DECISION_STRATEGY_CARD_PATTERN) {
        return result == VISION_PROTOCOL_RESULT_CARD_CHUNK;
    }
    return 0U;
}

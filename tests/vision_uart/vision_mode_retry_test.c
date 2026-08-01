#include "vision_mode_retry.h"

#include <stdio.h>

#define ASSERT_TRUE(condition)                                                \
    do {                                                                      \
        if (!(condition)) {                                                   \
            fprintf(stderr, "assertion failed at line %d: %s\n",            \
                    __LINE__, #condition);                                    \
            return 1;                                                         \
        }                                                                     \
    } while (0)

static int test_retry_runs_at_interval_up_to_five_total_attempts(void)
{
    VisionModeRetry retry;
    uint32_t tick = 1000U;

    VisionModeRetry_Arm(&retry, tick);
    ASSERT_TRUE(retry.attempt_count == 1U);
    ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, tick + 99U, 100U) == 0U);

    for (uint8_t expected = 2U; expected <= 5U; ++expected) {
        tick += 100U;
        ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, tick, 100U) == 1U);
        ASSERT_TRUE(retry.attempt_count == expected);
    }
    ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, tick + 100U, 100U) == 0U);
    ASSERT_TRUE(retry.attempt_count == 5U);
    return 0;
}

static int test_only_matching_result_stops_retries(void)
{
    VisionModeRetry retry;

    VisionModeRetry_Arm(&retry, 0U);
    ASSERT_TRUE(VisionModeRetry_ResultMatches(
                    DECISION_STRATEGY_GEOMETRIC,
                    VISION_PROTOCOL_RESULT_CARD_CHUNK) == 0U);
    ASSERT_TRUE(VisionModeRetry_ResultMatches(
                    DECISION_STRATEGY_CARD_PATTERN,
                    VISION_PROTOCOL_RESULT_FRAME) == 0U);
    ASSERT_TRUE(VisionModeRetry_ResultMatches(
                    DECISION_STRATEGY_GEOMETRIC,
                    VISION_PROTOCOL_RESULT_FRAME) == 1U);
    ASSERT_TRUE(VisionModeRetry_ResultMatches(
                    DECISION_STRATEGY_CARD_PATTERN,
                    VISION_PROTOCOL_RESULT_CARD_CHUNK) == 1U);

    ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, 100U, 100U) == 1U);
    VisionModeRetry_Stop(&retry);
    ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, 200U, 100U) == 0U);
    ASSERT_TRUE(retry.attempt_count == 2U);
    return 0;
}

static int test_tick_wraparound_keeps_retry_interval(void)
{
    VisionModeRetry retry;

    VisionModeRetry_Arm(&retry, UINT32_MAX - 49U);
    ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, 25U, 100U) == 0U);
    ASSERT_TRUE(VisionModeRetry_TakeDue(&retry, 50U, 100U) == 1U);
    return 0;
}

int main(void)
{
    if (test_retry_runs_at_interval_up_to_five_total_attempts() != 0) return 1;
    if (test_only_matching_result_stops_retries() != 0) return 1;
    if (test_tick_wraparound_keeps_retry_interval() != 0) return 1;
    puts("vision mode retry tests passed");
    return 0;
}

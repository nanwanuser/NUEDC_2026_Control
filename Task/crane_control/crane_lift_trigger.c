#include "crane_lift_trigger.h"

#include <stddef.h>

#define CRANE_LIFT_Z_EPSILON_MM          0.001f
#define CRANE_LIFT_STABLE_SAMPLE_COUNT   2U

static CraneLiftDirection detect_direction(float delta_z_mm)
{
    if (delta_z_mm > CRANE_LIFT_Z_EPSILON_MM) {
        return CRANE_LIFT_DIRECTION_UP;
    }
    if (delta_z_mm < -CRANE_LIFT_Z_EPSILON_MM) {
        return CRANE_LIFT_DIRECTION_DOWN;
    }
    return CRANE_LIFT_DIRECTION_NONE;
}

static void update_direction(CraneLiftTriggerState *state,
                             CraneLiftDirection direction,
                             float min_target_z_mm,
                             float max_target_z_mm)
{
    if (direction == CRANE_LIFT_DIRECTION_NONE) {
        if (state->stable_sample_count < CRANE_LIFT_STABLE_SAMPLE_COUNT) {
            state->stable_sample_count++;
        }
        if (state->stable_sample_count >= CRANE_LIFT_STABLE_SAMPLE_COUNT) {
            state->observed_direction = CRANE_LIFT_DIRECTION_NONE;
        }
        return;
    }
    state->stable_sample_count = 0U;
    if (direction != state->observed_direction) {
        state->observed_direction = direction;
        state->target_z_mm = direction == CRANE_LIFT_DIRECTION_UP
                                 ? max_target_z_mm
                                 : min_target_z_mm;
        state->command_pending = 1U;
    }
}

void CraneLiftTrigger_Init(CraneLiftTriggerState *state,
                           float initial_target_z_mm)
{
    if (state == NULL) {
        return;
    }
    *state = (CraneLiftTriggerState) {
        .target_z_mm = initial_target_z_mm
    };
}

uint8_t CraneLiftTrigger_Update(CraneLiftTriggerState *state,
                                float reference_z_mm,
                                float min_target_z_mm,
                                float max_target_z_mm,
                                float *target_z_mm)
{
    CraneLiftDirection direction;
    float delta_z_mm;

    if (state == NULL || target_z_mm == NULL) {
        return 0U;
    }
    if (state->initialized == 0U) {
        state->previous_reference_z_mm = reference_z_mm;
        state->initialized = 1U;
        *target_z_mm = state->target_z_mm;
        return state->command_pending;
    }
    delta_z_mm = reference_z_mm - state->previous_reference_z_mm;
    state->previous_reference_z_mm = reference_z_mm;
    direction = detect_direction(delta_z_mm);
    update_direction(state, direction, min_target_z_mm, max_target_z_mm);
    *target_z_mm = state->target_z_mm;
    return state->command_pending;
}

void CraneLiftTrigger_Acknowledge(CraneLiftTriggerState *state)
{
    if (state != NULL) {
        state->command_pending = 0U;
    }
}

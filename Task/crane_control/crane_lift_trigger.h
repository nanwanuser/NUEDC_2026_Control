#ifndef CRANE_LIFT_TRIGGER_H
#define CRANE_LIFT_TRIGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    CRANE_LIFT_DIRECTION_NONE = 0,
    CRANE_LIFT_DIRECTION_UP = 1,
    CRANE_LIFT_DIRECTION_DOWN = -1
} CraneLiftDirection;

typedef struct {
    float previous_reference_z_mm;
    float target_z_mm;
    CraneLiftDirection observed_direction;
    uint8_t stable_sample_count;
    uint8_t command_pending;
    uint8_t initialized;
} CraneLiftTriggerState;

/**
 * @brief Reset the Z-direction trigger and its retained target.
 * @param state Trigger state.
 * @param initial_target_z_mm Initial local Z target.
 */
void CraneLiftTrigger_Init(CraneLiftTriggerState *state,
                           float initial_target_z_mm);

/**
 * @brief Observe one planner Z sample and resolve the retained extreme target.
 * @param state Trigger state.
 * @param reference_z_mm Planner world Z sample.
 * @param min_target_z_mm Local Z target for downward motion.
 * @param max_target_z_mm Local Z target for upward motion.
 * @param target_z_mm Receives the retained local Z target.
 * @return Nonzero while the new extreme command is waiting to be accepted.
 */
uint8_t CraneLiftTrigger_Update(CraneLiftTriggerState *state,
                                float reference_z_mm,
                                float min_target_z_mm,
                                float max_target_z_mm,
                                float *target_z_mm);

/**
 * @brief Mark the pending extreme command as accepted by the servo driver.
 * @param state Trigger state.
 */
void CraneLiftTrigger_Acknowledge(CraneLiftTriggerState *state);

#ifdef __cplusplus
}
#endif

#endif

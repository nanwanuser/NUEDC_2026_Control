#ifndef MISSION_H
#define MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "crane_control.h"
#include "decision.h"
#include "decision_task.h"

typedef enum {
    MISSION_NONE = 0,
    /* key1: purely geometric assembly. Requirement 1 (the team's own pieces
       moved from the upper half of the A4 sheet to the lower half) and
       requirement 2(1) (white pieces supplied at the venue) are the same
       edge-matching solve, so they share one mission. */
    MISSION_GEOMETRIC,
    /* key2: requirement 2(2), the same geometry plus matching playing-card
       patterns across adjacent pieces. The pattern term is not implemented
       yet, so this currently behaves like MISSION_GEOMETRIC. */
    MISSION_CARD_PATTERN
} MissionId;

typedef enum {
    MISSION_STATE_IDLE = 0,
    /* Armed and waiting for three consistent vision frames. */
    MISSION_STATE_ACQUIRING,
    MISSION_STATE_RUNNING,
    MISSION_STATE_COMPLETE,
    MISSION_STATE_FAILED,
    MISSION_STATE_TIMEOUT
} MissionState;

typedef struct {
    MissionId mission;
    MissionState state;
    uint32_t run_id;
    /* Milliseconds since the key press; frozen once the run ends. */
    uint32_t elapsed_ms;
    uint8_t placed_count;
    uint8_t piece_count;
    DecisionResult decision_result;
    TrajectoryResult trajectory_result;
    /* Why the crane stopped, which is the usual reason a run fails during
       debugging: a target outside the boom or reach travel. */
    CraneControlStatus crane_status;
} MissionOutput;

void Mission_Init(void);

/* Optional known-geometry template, matched by piece ID. Registering one
   switches both missions to fixed-ID registration, which is faster and more
   accurate but only works for the pieces in the template. Leave it unset for
   venue-supplied pieces. */
uint8_t Mission_SetFixedLayout(const DecisionFixedLayout *layout);
void Mission_ClearFixedLayout(void);

/* Same effect as pressing the matching key, for host or debug triggering. */
uint8_t Mission_Start(MissionId mission);
void Mission_Abort(void);

void Mission_GetOutput(MissionOutput *output);
void Mission_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

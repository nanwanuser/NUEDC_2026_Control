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

/* Why an acquisition failed, beeped out after the failure tone. The numeric
   value is the number of short beeps, so keep the codes small and ordered from
   "nothing arrived" to "arrived but unusable". */
typedef enum {
    MISSION_DIAG_NONE = 0,
    /* No bytes at all on USART1: wiring, the camera's /dev/ttyS0, or a baud
       mismatch. */
    MISSION_DIAG_NO_DATA = 1,
    /* Bytes arrived but no frame ever decoded: framing, CRC, or byte order. */
    MISSION_DIAG_FRAME_REJECTED = 2,
    /* Valid frames arrived but never agreed three times running, so the pieces
       were still moving or the measurement is jittering. */
    MISSION_DIAG_NOT_STABLE = 3,
    /* The ring buffer overran, so the task is not draining USART1 fast
       enough. */
    MISSION_DIAG_RX_OVERFLOW = 4,
    /* A stable frame was handed over but the decision refused it. */
    MISSION_DIAG_SUBMIT_REFUSED = 5
} MissionDiagnosis;

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
    /* Set when a run ends in FAILED or TIMEOUT during acquisition. */
    MissionDiagnosis diagnosis;
    /* Copied from the vision task at the moment the run ended, so a debugger or
       host can read the counters without racing the acquisition. */
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t dropped_byte_count;
    uint8_t stable_count;
} MissionOutput;

void Mission_Init(void);

/* Same effect as pressing the matching key, for host or debug triggering. */
uint8_t Mission_Start(MissionId mission);
void Mission_Abort(void);

void Mission_GetOutput(MissionOutput *output);
void Mission_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

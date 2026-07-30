#include "mission.h"

#include "buzzer.h"
#include "crane_control.h"
#include "key.h"
#include "route_planning.h"
#include "vision_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#define MISSION_TASK_PERIOD_MS      5U
#define MISSION_KEY_COUNT           2U
#define MISSION_DEBOUNCE_MS         25U
/* The contest allows two minutes from the key press to the finished puzzle. */
#define MISSION_TIME_LIMIT_MS       120000U
#define MISSION_ACQUIRE_TIMEOUT_MS  20000U
#define MISSION_START_BEEP_MS       120U
#define MISSION_DONE_BEEP_MS        600U
#define MISSION_FAIL_BEEP_MS        150U
#define MISSION_FAIL_BEEP_GAP_MS    150U
#define MISSION_FAIL_BEEP_COUNT     3U

/* The crane's frame is the authority on where things are: its origin sits at the
   column, and the boom's zero heading points away from it. Rather than restate
   the sheet's placement here, the target is derived from the reach the crane
   actually has, so it is inside the workspace by construction.

   The assembled rectangle needs its whole footprint reachable, so it goes at the
   middle of the reach band, which is the only placement that leaves clearance at
   both travel ends. With a 70-230 mm band that is r = 150 mm, and the largest
   allowed rectangle has a 75 mm half-diagonal, so the fit is tight but real. */

typedef struct {
    MissionId mission;
    GPIO_TypeDef *port;
    uint16_t pin;
} MissionKeyBinding;

static const MissionKeyBinding Mission_KeyBindings[MISSION_KEY_COUNT] = {
    { MISSION_GEOMETRIC,    Key1_GPIO_Port, Key1_Pin },
    { MISSION_CARD_PATTERN, Key2_GPIO_Port, Key2_Pin },
};

static key_handle_t Mission_Keys[MISSION_KEY_COUNT];
static buzzer_handle_t Mission_Buzzer;
static DecisionFixedLayout Mission_FixedLayout;
static volatile MissionOutput Mission_Output;
static volatile MissionId Mission_StartRequest;
static volatile uint8_t Mission_AbortRequest;
static uint32_t Mission_NextRunId = 1U;

static void publish_output(const MissionOutput *output)
{
    taskENTER_CRITICAL();
    Mission_Output = *output;
    taskEXIT_CRITICAL();
}

static uint32_t next_run_id(void)
{
    uint32_t run_id = Mission_NextRunId;

    ++Mission_NextRunId;
    if (Mission_NextRunId == 0U) {
        Mission_NextRunId = 1U;
    }
    return run_id;
}

static uint32_t elapsed_ms(uint32_t start_tick, uint32_t now_tick)
{
    const uint32_t tick_frequency = osKernelGetTickFreq();

    if (tick_frequency == 0U) {
        return 0U;
    }
    return (uint32_t)(((uint64_t)(now_tick - start_tick) * 1000U) /
                      tick_frequency);
}

void Mission_Init(void)
{
    MissionOutput output;
    buzzer_config_t buzzer_config;
    uint32_t index;

    (void)memset(&Mission_FixedLayout, 0, sizeof(Mission_FixedLayout));
    Mission_StartRequest = MISSION_NONE;
    Mission_AbortRequest = 0U;
    Mission_NextRunId = 1U;

    /* The keys are active low, so the pins need a pull-up to read a defined
       level when released and a falling edge to mark the press. MX_GPIO_Init
       leaves them floating on a rising edge, so override that here rather than
       in the generated file. */
    for (index = 0U; index < MISSION_KEY_COUNT; ++index) {
        GPIO_InitTypeDef gpio_config;

        (void)memset(&gpio_config, 0, sizeof(gpio_config));
        gpio_config.Pin = Mission_KeyBindings[index].pin;
        gpio_config.Mode = GPIO_MODE_IT_FALLING;
        gpio_config.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(Mission_KeyBindings[index].port, &gpio_config);
    }

    for (index = 0U; index < MISSION_KEY_COUNT; ++index) {
        key_config_t key_config;

        (void)memset(&key_config, 0, sizeof(key_config));
        key_config.gpio_port = Mission_KeyBindings[index].port;
        key_config.gpio_pin = Mission_KeyBindings[index].pin;
        /* The keys pull the pin down when pressed. */
        key_config.pressed_level = GPIO_PIN_RESET;
        key_config.debounce_time_ms = MISSION_DEBOUNCE_MS;
        (void)key_init(&Mission_Keys[index], &key_config);
    }

    (void)memset(&buzzer_config, 0, sizeof(buzzer_config));
    buzzer_config.gpio_port = Buzz_GPIO_Port;
    buzzer_config.gpio_pin = Buzz_Pin;
    buzzer_config.active_level = GPIO_PIN_SET;
    (void)buzzer_init(&Mission_Buzzer, &buzzer_config);

    (void)memset(&output, 0, sizeof(output));
    output.state = MISSION_STATE_IDLE;
    output.decision_result = DECISION_RESULT_OK;
    output.trajectory_result = TRAJECTORY_RESULT_OK;
    output.crane_status = CRANE_CONTROL_OK;
    Mission_Output = output;
}

uint8_t Mission_SetFixedLayout(const DecisionFixedLayout *layout)
{
    if (layout == NULL || layout->piece_count == 0U ||
        layout->piece_count > DECISION_MAX_PIECES) {
        return 0U;
    }

    Mission_FixedLayout = *layout;
    return 1U;
}

void Mission_ClearFixedLayout(void)
{
    (void)memset(&Mission_FixedLayout, 0, sizeof(Mission_FixedLayout));
}

uint8_t Mission_Start(MissionId mission)
{
    if (mission != MISSION_GEOMETRIC && mission != MISSION_CARD_PATTERN) {
        return 0U;
    }

    taskENTER_CRITICAL();
    Mission_StartRequest = mission;
    Mission_AbortRequest = 0U;
    taskEXIT_CRITICAL();
    return 1U;
}

void Mission_Abort(void)
{
    taskENTER_CRITICAL();
    Mission_StartRequest = MISSION_NONE;
    Mission_AbortRequest = 1U;
    taskEXIT_CRITICAL();
}

void Mission_GetOutput(MissionOutput *output)
{
    if (output == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *output = Mission_Output;
    taskEXIT_CRITICAL();
}

/* Fills the DecisionTaskRequest that the vision task will complete with the
   measured piece polygons. */
static void build_request(MissionId mission, DecisionTaskRequest *request)
{
    CraneControlConfig crane_config;
    TrajectoryPose target_pose;

    (void)mission;
    DecisionTask_GetDefaultRequest(request);
    CraneControl_GetConfig(&crane_config);

    /* Place the target on the boom's zero heading at the middle of the reach, so
       the rectangle is centred in the sweep and every piece is a boom turn away
       from it. Expressing it as a pose keeps the frame conversion in the crane
       module, which owns it. */
    CraneControl_GetPoseAt(0.0f,
                           0.5f * (crane_config.min_radius_mm +
                                   crane_config.max_radius_mm),
                           crane_config.min_z_mm,
                           &target_pose);
    request->config.target_center.x_mm = target_pose.x_mm;
    request->config.target_center.y_mm = target_pose.y_mm;

    /* The crane's lift is a two-position servo, so the planner's transit height
       has to be the crane's own travel height rather than a nominal 40 mm. */
    request->config.pick_z_mm = crane_config.min_z_mm;
    request->config.place_z_mm = crane_config.min_z_mm;
    request->config.transit_z_mm = crane_config.max_z_mm;
    /* The first waypoint has to be where the crane actually is. */
    CraneControl_GetCurrentPose(&request->execution.current_pose);

    /* Both missions solve the same geometry; a registered template just makes
       the solve a per-ID registration instead of an edge-matching search. */
    request->mode = Mission_FixedLayout.piece_count > 0U
        ? DECISION_MODE_FIXED_ID
        : DECISION_MODE_GENERAL;
}

static uint8_t arm_mission(MissionId mission, uint32_t run_id)
{
    DecisionTaskRequest request;
    CraneControlState crane_state;

    /* The crane task parks the boom before it accepts references. Starting ahead
       of that would submit a plan nothing executes, and the run would burn the
       whole time limit before anyone noticed. */
    CraneControl_GetState(&crane_state);
    if (crane_state.initialized == 0U) {
        return 0U;
    }

    build_request(mission, &request);
    if (request.mode == DECISION_MODE_FIXED_ID &&
        VisionUart_SetFixedLayout(&Mission_FixedLayout) == 0U) {
        return 0U;
    }
    return VisionUart_Arm(&request, run_id);
}

static MissionId take_key_request(void)
{
    uint32_t index;

    key_process_all(Mission_Keys, MISSION_KEY_COUNT);
    for (index = 0U; index < MISSION_KEY_COUNT; ++index) {
        if ((key_get_events(&Mission_Keys[index]) & KEY_EVENT_PRESSED) != 0U) {
            return Mission_KeyBindings[index].mission;
        }
    }
    return MISSION_NONE;
}

static MissionId take_start_request(void)
{
    MissionId mission;

    taskENTER_CRITICAL();
    mission = Mission_StartRequest;
    Mission_StartRequest = MISSION_NONE;
    taskEXIT_CRITICAL();
    return mission;
}

static uint8_t take_abort_request(void)
{
    uint8_t requested;

    taskENTER_CRITICAL();
    requested = Mission_AbortRequest;
    Mission_AbortRequest = 0U;
    taskEXIT_CRITICAL();
    return requested;
}

/* Short-short-short marks a rejected or failed run; one long tone marks the
   finished puzzle, which is the audible indication the rules ask for. */
static void signal_failure(void)
{
    uint32_t index;

    for (index = 0U; index < MISSION_FAIL_BEEP_COUNT; ++index) {
        (void)buzzer_beep(&Mission_Buzzer, MISSION_FAIL_BEEP_MS);
        osDelay(MISSION_FAIL_BEEP_MS);
        buzzer_off(&Mission_Buzzer);
        osDelay(MISSION_FAIL_BEEP_GAP_MS);
    }
}

void Mission_App(void *argument)
{
    MissionOutput output;
    DecisionTaskOutput decision_output;
    VisionUartOutput vision_output;
    uint32_t run_start_tick = 0U;

    (void)argument;
    Mission_GetOutput(&output);

    for (;;) {
        const uint32_t now_tick = osKernelGetTickCount();
        MissionId requested = take_key_request();
        const uint8_t running = (output.state == MISSION_STATE_ACQUIRING) ||
                                (output.state == MISSION_STATE_RUNNING);

        if (requested == MISSION_NONE) {
            requested = take_start_request();
        }

        if (take_abort_request() != 0U) {
            VisionUart_Abort();
            RoutePlanning_Cancel();
            output.state = MISSION_STATE_IDLE;
            output.elapsed_ms = elapsed_ms(run_start_tick, now_tick);
            publish_output(&output);
            requested = MISSION_NONE;
        }

        /* A key press during a run is ignored: the rules require the device to
           finish on its own without intervention. */
        if (requested != MISSION_NONE && running == 0U) {
            (void)memset(&output, 0, sizeof(output));
            output.mission = requested;
            output.run_id = next_run_id();
            output.decision_result = DECISION_RESULT_OK;
            output.trajectory_result = TRAJECTORY_RESULT_OK;
            output.crane_status = CRANE_CONTROL_OK;
            run_start_tick = now_tick;

            RoutePlanning_Cancel();
            if (arm_mission(requested, output.run_id) != 0U) {
                output.state = MISSION_STATE_ACQUIRING;
                publish_output(&output);
                (void)buzzer_beep(&Mission_Buzzer, MISSION_START_BEEP_MS);
            } else {
                output.state = MISSION_STATE_FAILED;
                publish_output(&output);
                signal_failure();
            }
        } else if (output.state == MISSION_STATE_ACQUIRING) {
            output.elapsed_ms = elapsed_ms(run_start_tick, now_tick);
            VisionUart_GetOutput(&vision_output);

            if (vision_output.arm_id == output.run_id &&
                vision_output.state == VISION_UART_STATE_SUBMITTED) {
                output.state = MISSION_STATE_RUNNING;
            } else if (vision_output.arm_id == output.run_id &&
                       vision_output.state == VISION_UART_STATE_ERROR) {
                output.state = MISSION_STATE_FAILED;
            } else if (output.elapsed_ms >= MISSION_ACQUIRE_TIMEOUT_MS) {
                VisionUart_Abort();
                output.state = MISSION_STATE_TIMEOUT;
            }
            publish_output(&output);
            if (output.state == MISSION_STATE_FAILED ||
                output.state == MISSION_STATE_TIMEOUT) {
                signal_failure();
            }
        } else if (output.state == MISSION_STATE_RUNNING) {
            CraneControlState crane_state;

            output.elapsed_ms = elapsed_ms(run_start_tick, now_tick);
            taskENTER_CRITICAL();
            decision_output = DecisionTask_Output;
            taskEXIT_CRITICAL();
            CraneControl_GetState(&crane_state);

            output.decision_result = decision_output.result;
            output.trajectory_result = decision_output.trajectory_result;
            output.piece_count = decision_output.plan.move_count;
            output.placed_count = decision_output.active_move_index;
            output.crane_status = crane_state.status;

            if (decision_output.execution_state == DECISION_EXECUTION_COMPLETE) {
                output.placed_count = decision_output.plan.move_count;
                output.state = MISSION_STATE_COMPLETE;
            } else if (decision_output.execution_state ==
                       DECISION_EXECUTION_ERROR) {
                output.state = MISSION_STATE_FAILED;
            } else if (crane_state.status != CRANE_CONTROL_OK) {
                /* The crane refused a reference, so the arm is no longer
                   following the plan. Stop rather than run out the clock. */
                RoutePlanning_Cancel();
                output.state = MISSION_STATE_FAILED;
            } else if (output.elapsed_ms >= MISSION_TIME_LIMIT_MS) {
                RoutePlanning_Cancel();
                output.state = MISSION_STATE_TIMEOUT;
            }
            publish_output(&output);

            if (output.state == MISSION_STATE_COMPLETE) {
                (void)buzzer_beep(&Mission_Buzzer, MISSION_DONE_BEEP_MS);
            } else if (output.state == MISSION_STATE_FAILED ||
                       output.state == MISSION_STATE_TIMEOUT) {
                signal_failure();
            }
        }

        buzzer_process(&Mission_Buzzer);
        osDelay(MISSION_TASK_PERIOD_MS);
    }
}

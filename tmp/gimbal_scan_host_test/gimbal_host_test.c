#include "Gimbal_ctrl_Task.h"
#include "Gimbal_ctrl_Motion.h"
#include "Gimbal_ctrl_Vision.h"
#include "cmsis_os.h"

#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MOVE_LOG_CAPACITY 4096
#define EVENT_CAPACITY 16
#define APP_STATUS_LOG_CAPACITY 32

typedef struct {
    gimbal_motion_axis_t axis;
    int16_t target;
    int16_t yaw_before;
    int16_t pitch_before;
    uint16_t speed_rpm;
    uint8_t acceleration;
    bool asynchronous;
} move_log_t;

typedef struct {
    int receive_call;
    uint8_t status;
    int16_t x;
    int16_t y;
} vision_event_t;

volatile HAL_StatusTypeDef gimbal_ctrl_status = HAL_ERROR;

static uint32_t mock_tick;
static int16_t mock_yaw;
static int16_t mock_pitch;
static HAL_StatusTypeDef mock_home_status = HAL_OK;
static move_log_t move_log[MOVE_LOG_CAPACITY];
static int move_count;
static vision_event_t events[EVENT_CAPACITY];
static int event_count;
static int event_index;
static int receive_calls;
static uint32_t first_delay;
static int thread_exit_calls;
static int stop_calls;
static HAL_StatusTypeDef mock_stop_status = HAL_OK;
static uint32_t mock_motion_time_ms;

static bool app_mode;
static bool app_reacquired;
static bool app_jump_ready;
static HAL_StatusTypeDef app_status_by_receive[APP_STATUS_LOG_CAPACITY];
static HAL_StatusTypeDef app_status_after_reacquire;
static int app_scan_moves_before_reacquire;
static int16_t app_rescan_pitch;
static uint32_t app_reacquire_tick;
static uint32_t app_correction_tick;
static int app_corrections_after_reacquire;
static jmp_buf app_jump;

#define CHECK(condition, message)                                           \
    do {                                                                    \
        if (!(condition)) {                                                 \
            fprintf(stderr, "FAIL: %s\n", message);                       \
            return false;                                                   \
        }                                                                   \
    } while (0)

static void reset_logs(void)
{
    move_count = 0;
    event_count = 0;
    event_index = 0;
    receive_calls = 0;
}

static void reset_mocks(void)
{
    int index;
    mock_tick = 0U;
    mock_yaw = 0;
    mock_pitch = 0;
    mock_home_status = HAL_OK;
    first_delay = UINT32_MAX;
    thread_exit_calls = 0;
    stop_calls = 0;
    mock_stop_status = HAL_OK;
    mock_motion_time_ms = 0U;
    app_mode = false;
    app_reacquired = false;
    app_jump_ready = false;
    for (index = 0; index < APP_STATUS_LOG_CAPACITY; index++) {
        app_status_by_receive[index] = HAL_BUSY;
    }
    app_status_after_reacquire = HAL_BUSY;
    app_scan_moves_before_reacquire = 0;
    app_rescan_pitch = INT16_MIN;
    app_reacquire_tick = 0U;
    app_correction_tick = 0U;
    app_corrections_after_reacquire = 0;
    reset_logs();
}

static void add_event(int call, uint8_t status, int16_t x, int16_t y)
{
    if (event_count >= EVENT_CAPACITY) {
        abort();
    }
    events[event_count++] = (vision_event_t){call, status, x, y};
}

static void log_move(gimbal_motion_axis_t axis, int16_t target,
                     uint16_t speed_rpm, uint8_t acceleration,
                     bool asynchronous)
{
    if (move_count >= MOVE_LOG_CAPACITY) {
        abort();
    }
    move_log[move_count++] = (move_log_t){
        axis, target, mock_yaw, mock_pitch, speed_rpm, acceleration,
        asynchronous};
}

static void set_axis(gimbal_motion_axis_t axis, int16_t target)
{
    if (axis == GIMBAL_MOTION_AXIS_YAW) {
        mock_yaw = target;
    } else if (axis == GIMBAL_MOTION_AXIS_PITCH) {
        mock_pitch = target;
    }
}

uint32_t HAL_GetTick(void)
{
    return mock_tick;
}

osStatus_t osDelay(uint32_t ticks)
{
    if (first_delay == UINT32_MAX) {
        first_delay = ticks;
    }
    mock_tick += ticks;
    return osOK;
}

void osThreadExit(void)
{
    thread_exit_calls++;
}

void gimbal_motion_reset(void)
{
    mock_yaw = 0;
    mock_pitch = 0;
}

HAL_StatusTypeDef gimbal_motion_home_pitch(void)
{
    return mock_home_status;
}

HAL_StatusTypeDef gimbal_motion_stop_all(void)
{
    stop_calls++;
    return mock_stop_status;
}

void gimbal_motion_request_yaw_compensation(int32_t delta_angle_tenths)
{
    (void)delta_angle_tenths;
}

HAL_StatusTypeDef gimbal_motion_process_yaw_compensation(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef gimbal_motion_start_absolute(
    gimbal_motion_axis_t axis, int16_t target_angle_tenths,
    uint16_t speed_rpm, uint8_t acceleration, uint32_t *motion_time_ms)
{
    int16_t before = axis == GIMBAL_MOTION_AXIS_YAW ? mock_yaw : mock_pitch;
    (void)speed_rpm;
    (void)acceleration;
    log_move(axis, target_angle_tenths, speed_rpm, acceleration, true);
    set_axis(axis, target_angle_tenths);
    *motion_time_ms = mock_motion_time_ms;
    if (app_mode && app_reacquired && target_angle_tenths != before) {
        if (app_corrections_after_reacquire == 0) {
            app_correction_tick = mock_tick;
        }
        app_corrections_after_reacquire++;
    }
    return HAL_OK;
}

HAL_StatusTypeDef gimbal_motion_move_relative(
    gimbal_motion_axis_t axis, int16_t delta_angle_tenths,
    uint16_t speed_rpm, uint8_t acceleration)
{
    int16_t current = axis == GIMBAL_MOTION_AXIS_YAW ? mock_yaw : mock_pitch;
    log_move(axis, (int16_t)(current + delta_angle_tenths),
             speed_rpm, acceleration, false);
    set_axis(axis, (int16_t)(current + delta_angle_tenths));
    return HAL_OK;
}

HAL_StatusTypeDef gimbal_motion_move_absolute(
    gimbal_motion_axis_t axis, int16_t target_angle_tenths,
    uint16_t speed_rpm, uint8_t acceleration)
{
    if (app_mode && receive_calls > 1 && !app_reacquired) {
        app_scan_moves_before_reacquire++;
    }
    log_move(axis, target_angle_tenths, speed_rpm, acceleration, false);
    set_axis(axis, target_angle_tenths);
    return HAL_OK;
}

int16_t gimbal_motion_get_angle(gimbal_motion_axis_t axis)
{
    return axis == GIMBAL_MOTION_AXIS_YAW ? mock_yaw : mock_pitch;
}

HAL_StatusTypeDef gimbal_vision_receive(uint32_t timeout_ms)
{
    vision_event_t *event = NULL;
    receive_calls++;
    mock_tick += timeout_ms;
    if (app_mode && receive_calls < APP_STATUS_LOG_CAPACITY) {
        app_status_by_receive[receive_calls] = gimbal_ctrl_status;
    }
    if (app_mode && app_reacquired && app_jump_ready && receive_calls == 18) {
        app_status_after_reacquire = gimbal_ctrl_status;
        longjmp(app_jump, 1);
    }
    if (event_index < event_count &&
        events[event_index].receive_call == receive_calls) {
        event = &events[event_index++];
        gimbal_ctrl_vision_input(event->status, event->x, event->y);
        if (app_mode && receive_calls == 13) {
            app_reacquired = true;
            app_reacquire_tick = mock_tick;
            app_rescan_pitch = mock_pitch;
        }
        return HAL_OK;
    }
    return app_mode ? HAL_ERROR : HAL_TIMEOUT;
}

static bool initialize_for_test(void)
{
    return gimbal_ctrl_initialize() == HAL_OK;
}

static bool test_configuration_constants(void)
{
    CHECK(GIMBAL_HOMING_TORQUE_DURATION_MS == 4000U,
          "homing torque must last 4000 ms");
    CHECK(GIMBAL_HOMING_SPEED_RPM == 10U &&
              GIMBAL_HOMING_ACCELERATION == 0U,
          "homing reverse must use 10 RPM and direct start");
    CHECK(GIMBAL_SCAN_SPEED_RPM == 10U && GIMBAL_SCAN_ACCELERATION == 0U,
          "scan must use 10 RPM and direct start");
    CHECK(GIMBAL_SCAN_YAW_STEP_TENTHS == 180 &&
              GIMBAL_SCAN_FINE_YAW_STEP_TENTHS == 30 &&
              GIMBAL_SCAN_FINE_ATTEMPTS == 6U &&
              GIMBAL_SCAN_PITCH_STEP_TENTHS == 90 &&
              GIMBAL_VISION_POLL_TIMEOUT_MS == 50U,
          "scan must use 18-degree coarse and six 3-degree fine Yaw steps");
    CHECK(GIMBAL_VISION_STEP_MAX_TENTHS == 15 &&
              GIMBAL_VISION_SPEED_MIN_RPM == 30U &&
              GIMBAL_VISION_SPEED_MAX_RPM == 60U,
          "vision correction limits must be 1.5 degrees and 30-60 RPM");
    CHECK(GIMBAL_YAW_ANGLE_MIN_TENTHS == -1800 &&
              GIMBAL_YAW_ANGLE_MAX_TENTHS == 1800 &&
              GIMBAL_PITCH_ANGLE_MIN_TENTHS == 0 &&
              GIMBAL_PITCH_ANGLE_MAX_TENTHS == 450,
          "axis software limits must match the requested ranges");
    return true;
}

static bool test_scan_starts_at_current_pitch(void)
{
    int first_pitch_index = -1;
    int index;
    bool reached_top = false;
    bool descended = false;
    reset_mocks();
    CHECK(initialize_for_test(), "initialization should succeed");
    mock_yaw = 100;
    mock_pitch = 120;
    reset_logs();
    CHECK(gimbal_ctrl_scan() == HAL_TIMEOUT,
          "scan without a target should end with HAL_TIMEOUT");
    for (index = 0; index < move_count; index++) {
        CHECK(move_log[index].speed_rpm == GIMBAL_SCAN_SPEED_RPM &&
                  move_log[index].acceleration == GIMBAL_SCAN_ACCELERATION,
              "every scan move must use 10 RPM and acceleration zero");
        if (move_log[index].axis != GIMBAL_MOTION_AXIS_PITCH) {
            continue;
        }
        if (first_pitch_index < 0) {
            first_pitch_index = index;
        }
        CHECK(move_log[index].target >= GIMBAL_PITCH_ANGLE_MIN_TENTHS &&
                  move_log[index].target <= GIMBAL_PITCH_ANGLE_MAX_TENTHS,
              "Pitch scan target must remain inside 0 to 45 degrees");
        reached_top = reached_top || move_log[index].target == 450;
        descended = descended || (reached_top && move_log[index].target < 450);
    }
    CHECK(first_pitch_index >= 0 && move_log[first_pitch_index].target == 210,
          "first Pitch row increment should be +9.0 degrees");
    CHECK(move_log[first_pitch_index].yaw_before == -1800,
          "current Pitch row should be fully scanned before Pitch advances");
    CHECK(reached_top && descended,
          "scan must reach 45 degrees and then reverse downward");
    CHECK(mock_pitch == 0, "downward pass must finish at zero degrees");
    CHECK(stop_calls == 0, "scan without a target must not issue a stop");
    return true;
}

static bool test_scan_finds_target_on_current_pitch(void)
{
    int index;
    reset_mocks();
    CHECK(initialize_for_test(), "initialization should succeed");
    mock_yaw = 100;
    mock_pitch = 120;
    reset_logs();
    add_event(3, GIMBAL_VISION_STATUS_VALID, 100, 0);
    CHECK(gimbal_ctrl_scan() == HAL_OK,
          "fresh target on current Pitch should finish the active step then stop scanning");
    CHECK(mock_pitch == 120, "target found on current row must not change Pitch");
    CHECK(mock_yaw == 280, "Yaw should finish the 18-degree step where target is found");
    for (index = 0; index < move_count; index++) {
        CHECK(move_log[index].axis != GIMBAL_MOTION_AXIS_PITCH,
              "current-row target must be found before a Pitch move");
        CHECK(move_log[index].speed_rpm == GIMBAL_SCAN_SPEED_RPM &&
                  move_log[index].acceleration == 0U,
              "target acquisition scan must use the requested parameters");
    }
    CHECK(stop_calls == 0, "valid scan data must not interrupt the active step");
    return true;
}

static bool test_scan_polls_during_motion(void)
{
    reset_mocks();
    CHECK(initialize_for_test(), "initialization should succeed");
    mock_yaw = 100;
    mock_pitch = 120;
    mock_motion_time_ms = 200U;
    reset_logs();
    add_event(3, GIMBAL_VISION_STATUS_VALID, 100, 0);
    CHECK(gimbal_ctrl_scan() == HAL_OK,
          "target received during a move should finish the active step before scanning exits");
    CHECK(move_count == 1 && receive_calls == 6,
          "scan should keep polling until the commanded move completes");
    CHECK(stop_calls == 0 && mock_tick >= mock_motion_time_ms,
          "valid data must be latched without interrupting the active step");
    return true;
}

static bool test_lost_target_runs_reverse_fine_scan(void)
{
    static const int16_t expected_targets[] = {
        280, 250, 220, 190, 160, 130, 100, 280
    };
    size_t index;
    reset_mocks();
    CHECK(initialize_for_test(), "initialization should succeed");
    mock_yaw = 100;
    mock_pitch = 120;
    reset_logs();
    add_event(2, GIMBAL_VISION_STATUS_VALID, 100, 0);
    add_event(3, GIMBAL_VISION_STATUS_LOST, -1, -1);
    add_event(16, GIMBAL_VISION_STATUS_VALID, 100, 0);
    CHECK(gimbal_ctrl_scan() == HAL_OK,
          "coarse scan should resume after six failed fine steps");
    CHECK(move_count == 8,
          "one coarse, six fine, and one resumed coarse step are expected");
    for (index = 0U; index < sizeof(expected_targets) / sizeof(expected_targets[0]); index++) {
        CHECK(move_log[index].axis == GIMBAL_MOTION_AXIS_YAW &&
                  move_log[index].target == expected_targets[index],
              "fine scan must reverse in 3-degree steps before coarse resumes");
    }
    CHECK(stop_calls == 0,
          "coarse and fine scanning must not interrupt an active step");
    return true;
}

static bool test_direct_vision_input_is_fresh(void)
{
    reset_mocks();
    CHECK(initialize_for_test(), "initialization should succeed");
    mock_yaw = -100;
    mock_pitch = 200;
    reset_logs();
    gimbal_ctrl_vision_input(GIMBAL_VISION_STATUS_VALID, 20, -10);
    CHECK(gimbal_ctrl_scan() == HAL_OK,
          "directly submitted fresh vector should finish scan without motion");
    CHECK(move_count == 0, "fresh direct input should be accepted before motion");
    CHECK(stop_calls == 0, "fresh direct input must not issue the stop sequence");
    return true;
}

static bool test_one_correction_per_fresh_vector(void)
{
    int first_move_count;
    int first_receive_count;
    int index;
    uint32_t lost_tick;
    reset_mocks();
    CHECK(initialize_for_test(), "initialization should succeed");
    reset_logs();
    gimbal_ctrl_vision_input(GIMBAL_VISION_STATUS_VALID, 100, 50);
    CHECK(gimbal_ctrl_correct() == HAL_OK,
          "first valid vector should produce one correction");
    CHECK(move_count == 2 && mock_yaw == 15 && mock_pitch == 8,
          "vector (100,50) should normalize to (1.5,0.8) degrees");
    CHECK(move_log[0].speed_rpm == 60U && move_log[1].speed_rpm == 32U,
          "correction speed should scale between 30 and 60 RPM");
    first_move_count = move_count;
    CHECK(gimbal_ctrl_correct() == HAL_TIMEOUT,
          "same vector must not be corrected twice");
    CHECK(move_count == first_move_count,
          "no new frame must not issue another motor command");
    gimbal_ctrl_vision_input(GIMBAL_VISION_STATUS_VALID, -100, 100);
    CHECK(gimbal_ctrl_correct() == HAL_OK,
          "second fresh vector should produce one new correction");
    CHECK(mock_yaw == 0 && mock_pitch == 23,
          "second correction should move at most 1.5 degrees per axis");
    gimbal_ctrl_vision_input(GIMBAL_VISION_STATUS_LOST, -1, -1);
    lost_tick = mock_tick;
    first_receive_count = receive_calls;
    CHECK(gimbal_ctrl_correct() == HAL_ERROR,
          "already received lost frame should fail immediately");
    CHECK(mock_tick == lost_tick && receive_calls == first_receive_count,
          "lost frame should not wait for the vision timeout");
    gimbal_ctrl_vision_input(GIMBAL_VISION_STATUS_VALID, 20, 20);
    CHECK(gimbal_ctrl_correct() == HAL_OK,
          "small fresh vector should still produce one correction");
    CHECK(mock_yaw == 15 && mock_pitch == 38,
          "small vector should normalize to a full correction step");
    mock_yaw = 1790;
    mock_pitch = 440;
    gimbal_ctrl_vision_input(GIMBAL_VISION_STATUS_VALID, 100, 100);
    CHECK(gimbal_ctrl_correct() == HAL_OK &&
              mock_yaw == 1800 && mock_pitch == 450,
          "vision correction must obey independent axis limits");
    for (index = 0; index < move_count; index++) {
        CHECK(move_log[index].asynchronous &&
                  move_log[index].speed_rpm >= 30U &&
                  move_log[index].speed_rpm <= 60U &&
                  move_log[index].acceleration == GIMBAL_VISION_ACCELERATION,
              "all correction moves must use 30-60 RPM and acceleration 2");
    }
    return true;
}

static bool test_freertos_continuous_workflow(void)
{
    int call;
    reset_mocks();
    add_event(1, GIMBAL_VISION_STATUS_VALID, 0, 100);
    add_event(4, GIMBAL_VISION_STATUS_VALID, 100, 0);
    add_event(5, GIMBAL_VISION_STATUS_LOST, -1, -1);
    add_event(13, GIMBAL_VISION_STATUS_VALID, 100, 0);
    add_event(14, GIMBAL_VISION_STATUS_VALID, 100, 0);
    add_event(15, GIMBAL_VISION_STATUS_VALID, 100, 0);
    add_event(16, GIMBAL_VISION_STATUS_VALID, 100, 0);
    add_event(17, GIMBAL_VISION_STATUS_VALID, 100, 0);
    app_mode = true;
    app_jump_ready = true;
    if (setjmp(app_jump) == 0) {
        Gimbal_ctrl_App(NULL);
        CHECK(false, "continuous task should not return after initialization");
    }
    app_jump_ready = false;
    CHECK(first_delay == GIMBAL_SCAN_START_DELAY_MS,
          "first task delay should be the extra 2000 ms scan delay");
    CHECK(receive_calls == 18,
          "test should observe status after five reacquired corrections");
    for (call = 1; call <= 17; call++) {
        CHECK(app_status_by_receive[call] == HAL_ERROR,
              "status must stay HAL_ERROR before five consecutive corrections");
    }
    CHECK(app_status_by_receive[18] == HAL_OK,
          "the fifth consecutive correction must enable the chassis");
    CHECK(app_scan_moves_before_reacquire == 0,
          "fewer than eight consecutive failures must not start scanning");
    CHECK(app_rescan_pitch == 15,
          "rescan should retain the Pitch reached by the last correction");
    CHECK(app_corrections_after_reacquire == 5 &&
              app_status_after_reacquire == HAL_OK,
          "reacquisition must require five successful corrections");
    CHECK(app_correction_tick - app_reacquire_tick ==
              GIMBAL_TARGET_CORRECTION_DELAY_MS,
          "reacquired target must wait 1000 ms before correction");
    CHECK(stop_calls == 0,
          "target acquisition must not interrupt the active scan step");
    CHECK(thread_exit_calls == 0,
          "successful initialization should keep the task alive");
    return true;
}

int main(void)
{
    int passed = 0;
    passed += test_configuration_constants();
    passed += test_scan_starts_at_current_pitch();
    passed += test_scan_finds_target_on_current_pitch();
    passed += test_scan_polls_during_motion();
    passed += test_lost_target_runs_reverse_fine_scan();
    passed += test_direct_vision_input_is_fresh();
    passed += test_one_correction_per_fresh_vector();
    passed += test_freertos_continuous_workflow();
    if (passed != 8) {
        return EXIT_FAILURE;
    }
    printf("PASS: 8 gimbal workflow tests\n");
    printf("PASS: current-Pitch scan, up to 45 then down to 0 degrees\n");
    printf("PASS: finish active step, 1000 ms delay, 1.5 degree correction\n");
    printf("PASS: eight consecutive failures before loss recovery scan\n");
    return EXIT_SUCCESS;
}

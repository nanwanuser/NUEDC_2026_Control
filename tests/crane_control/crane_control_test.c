#include "crane_control.h"

#include "Servo.h"
#include "pd42s1.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t fake_tick;
static uint8_t capture_yaw_move;
static pd42s1_command_t captured_yaw_command;
static pd42s1_direction_t captured_yaw_direction;
static uint16_t captured_yaw_speed_rpm;
static uint32_t captured_yaw_position_units;
static uint32_t yaw_absolute_command_count;
static uint32_t reach_absolute_command_count;
static uint32_t captured_lift_command_count;
static uint32_t captured_lift_immediate_count;
static float captured_lift_angle_deg;
static pd42s1_arrival_t yaw_arrival = PD42S1_ARRIVAL_REACHED;
static pd42s1_arrival_t reach_arrival = PD42S1_ARRIVAL_REACHED;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

void HAL_Delay(uint32_t delay_ms)
{
    fake_tick += delay_ms;
}

HAL_StatusTypeDef Servo_Init(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngle(Servo_Id_t servo_id, float angle_deg)
{
    if (servo_id == SERVO_LIFT) {
        captured_lift_command_count++;
        captured_lift_angle_deg = angle_deg;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngleImmediate(Servo_Id_t servo_id, float angle_deg)
{
    if (servo_id == SERVO_LIFT) {
        captured_lift_command_count++;
        captured_lift_immediate_count++;
        captured_lift_angle_deg = angle_deg;
    }
    return HAL_OK;
}

void Servo_Update(void)
{
}

uint8_t Servo_IsAtTarget(Servo_Id_t servo_id)
{
    (void)servo_id;
    return 1U;
}

void pd42s1_init(void)
{
}

max485_status_t pd42s1_set_torque(uint8_t motor_id,
                                 pd42s1_direction_t direction,
                                 uint16_t current_ma)
{
    (void)motor_id;
    (void)direction;
    (void)current_ma;
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_move_absolute(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units)
{
    (void)acceleration;
    (void)speed_rpm;
    if (motor_id == PD42S1_MOTOR_1_ID) {
        ++yaw_absolute_command_count;
    } else if (motor_id == PD42S1_MOTOR_2_ID) {
        ++reach_absolute_command_count;
    }
    if (capture_yaw_move != 0U && motor_id == PD42S1_MOTOR_1_ID) {
        captured_yaw_command = PD42S1_COMMAND_ABSOLUTE_POSITION;
        captured_yaw_direction = direction;
        captured_yaw_speed_rpm = speed_rpm;
        captured_yaw_position_units = position_units;
        capture_yaw_move = 0U;
    }
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_move_relative(uint8_t motor_id,
                                    pd42s1_direction_t direction,
                                    uint8_t acceleration,
                                    uint16_t speed_rpm,
                                    uint32_t position_units)
{
    (void)acceleration;
    (void)speed_rpm;
    if (capture_yaw_move != 0U && motor_id == PD42S1_MOTOR_1_ID) {
        captured_yaw_command = PD42S1_COMMAND_RELATIVE_POSITION;
        captured_yaw_direction = direction;
        captured_yaw_position_units = position_units;
        capture_yaw_move = 0U;
    }
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_clear_position(uint8_t motor_id)
{
    (void)motor_id;
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_release_stall_protection(uint8_t motor_id)
{
    (void)motor_id;
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_clear_state(uint8_t motor_id)
{
    (void)motor_id;
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_receive_response(uint8_t motor_id,
                                       pd42s1_command_t command,
                                       pd42s1_result_t *result,
                                       uint32_t timeout_ms)
{
    (void)motor_id;
    (void)command;
    (void)timeout_ms;
    *result = PD42S1_RESULT_SUCCESS;
    return MAX485_STATUS_OK;
}

max485_status_t pd42s1_read_arrival_flag(uint8_t motor_id,
                                        pd42s1_arrival_t *arrival,
                                        uint32_t timeout_ms)
{
    (void)timeout_ms;
    *arrival = motor_id == PD42S1_MOTOR_1_ID ? yaw_arrival : reach_arrival;
    return MAX485_STATUS_OK;
}

void CraneControl_LoadDefaultConfig(CraneControlConfig *config)
{
    (void)memset(config, 0, sizeof(*config));
}

void CraneControl_CustomizeConfig(CraneControlConfig *config)
{
    (void)config;
}

static CraneControlConfig test_config(void)
{
    CraneControlConfig config;

    (void)memset(&config, 0, sizeof(config));
    config.origin.yaw_deg = 90.0f;
    config.startup_boom_yaw_deg = 90.0f;
    config.yaw_motor_revolutions_per_crane_revolution = 1.0f;
    config.reach_zero_radius_mm = 70.0f;
    config.reach_mm_per_motor_revolution = 94.2478f;
    config.lift_zero_angle_deg = 0.0f;
    config.lift_mm_per_degree = 94.2478f / 360.0f;
    config.end_yaw_center_angle_deg = 90.0f;
    config.min_boom_yaw_deg = -90.0f;
    config.max_boom_yaw_deg = 90.0f;
    config.min_radius_mm = 70.0f;
    config.max_radius_mm = 230.0f;
    config.min_z_mm = -39.0f;
    config.max_z_mm = 0.0f;
    config.yaw_direction_sign = -1;
    config.reach_direction_sign = 1;
    config.lift_direction_sign = -1;
    config.end_yaw_direction_sign = -1;
    config.yaw_speed_rpm = 40U;
    config.reach_speed_rpm = 20U;
    config.min_stepper_change_units = 1U;
    config.yaw_acceleration = 10U;
    config.reach_acceleration = 5U;
    config.expect_stepper_response = 1U;
    config.home_on_startup = 1U;
    config.home_torque_current_ma = 400U;
    config.home_push_ms = 1U;
    config.arrival_timeout_ms = 3000U;
    return config;
}

static int test_reverse_home_reports_maximum_yaw_stop(void)
{
    CraneControlConfig config = test_config();
    TrajectoryPose pose;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }

    CraneControl_GetCurrentPose(&pose);
    if (fabsf(pose.x_mm + 70.0f) > 0.01f || fabsf(pose.y_mm) > 0.01f) {
        fprintf(stderr,
                "reverse home datum is wrong: pose=(%.2f, %.2f), expected=(-70.00, 0.00)\n",
                pose.x_mm, pose.y_mm);
        return 1;
    }
    return 0;
}

/* The yaw axis is commanded as an absolute drive position measured from the
   datum homing established, not as a per-tick increment. That matters at 50 Hz:
   an increment scheme would accumulate every rounding of a fractional step, so
   the boom's idea of where it is would drift away from the plan's over a run,
   whereas an absolute target is self-correcting - a command the drive misses is
   simply restated by the next tick.
 *
 * Both legs below are checked against the datum rather than against each other,
 * which is what tells the two schemes apart: with the datum at +90 deg of boom
 * yaw and yaw_direction_sign = -1, a world heading of +90 deg is a quarter turn
 * from the datum (12800 units) and a heading of 0 deg is a half turn (25600). An
 * increment scheme would ask for 12800 twice. */
static int test_yaw_commands_absolute_positions_from_the_homed_datum(void)
{
    CraneControlConfig config = test_config();
    RoutePlanningOutput output;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }

    (void)memset(&output, 0, sizeof(output));
    output.plan_id = 1U;
    output.phase = TRAJECTORY_PHASE_APPROACH;
    output.state = TRAJECTORY_STATE_RUNNING;
    output.result = TRAJECTORY_RESULT_OK;
    output.reference.pose.x_mm = 0.0f;
    output.reference.pose.y_mm = 100.0f;
    output.reference.pose.z_mm = 0.0f;
    output.reference.pose.yaw_deg = 90.0f;

    captured_yaw_command = PD42S1_COMMAND_RELATIVE_POSITION;
    captured_yaw_direction = PD42S1_DIRECTION_REVERSE;
    captured_yaw_position_units = 0U;
    capture_yaw_move = 1U;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("planner output was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    if (captured_yaw_command != PD42S1_COMMAND_ABSOLUTE_POSITION ||
        captured_yaw_direction != PD42S1_DIRECTION_FORWARD ||
        captured_yaw_position_units != 12800U) {
        fprintf(stderr,
                "first yaw command=(0x%02X, %u, %lu), expected=(0x%02X, %u, 12800)\n",
                (unsigned)captured_yaw_command,
                (unsigned)captured_yaw_direction,
                (unsigned long)captured_yaw_position_units,
                (unsigned)PD42S1_COMMAND_ABSOLUTE_POSITION,
                (unsigned)PD42S1_DIRECTION_FORWARD);
        return 1;
    }

    output.reference.pose.x_mm = 100.0f;
    output.reference.pose.y_mm = 0.0f;
    output.reference.pose.yaw_deg = 0.0f;
    captured_yaw_position_units = 0U;
    capture_yaw_move = 1U;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("second planner output was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    if (captured_yaw_command != PD42S1_COMMAND_ABSOLUTE_POSITION ||
        captured_yaw_direction != PD42S1_DIRECTION_FORWARD ||
        captured_yaw_position_units != 25600U) {
        fprintf(stderr,
                "second yaw command=(0x%02X, %u, %lu), expected=(0x%02X, %u, 25600)\n",
                (unsigned)captured_yaw_command,
                (unsigned)captured_yaw_direction,
                (unsigned long)captured_yaw_position_units,
                (unsigned)PD42S1_COMMAND_ABSOLUTE_POSITION,
                (unsigned)PD42S1_DIRECTION_FORWARD);
        return 1;
    }
    return 0;
}

static int test_planner_z_does_not_control_lift(void)
{
    CraneControlConfig config = test_config();
    RoutePlanningOutput output;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }

    (void)memset(&output, 0, sizeof(output));
    output.plan_id = 1U;
    output.phase = TRAJECTORY_PHASE_APPROACH;
    output.state = TRAJECTORY_STATE_RUNNING;
    output.result = TRAJECTORY_RESULT_OK;
    output.reference.pose.x_mm = 0.0f;
    output.reference.pose.y_mm = 100.0f;
    output.reference.pose.z_mm = 0.0f;
    output.reference.pose.yaw_deg = 90.0f;

    captured_lift_command_count = 0U;
    captured_lift_immediate_count = 0U;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("initial lift reference was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();

    output.reference.pose.z_mm = -1.0f;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("descending lift reference was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    if (captured_lift_command_count != 0U) {
        fputs("planner descent moved the explicitly controlled lift\n", stderr);
        return 1;
    }

    output.reference.pose.z_mm = 0.0f;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("rising lift reference was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    if (captured_lift_command_count != 0U) {
        fputs("planner ascent moved the explicitly controlled lift\n", stderr);
        return 1;
    }
    return 0;
}

static int test_complete_reference_uses_full_speed_to_settle(void)
{
    CraneControlConfig config = test_config();
    RoutePlanningOutput output;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }

    (void)memset(&output, 0, sizeof(output));
    output.plan_id = 1U;
    output.phase = TRAJECTORY_PHASE_APPROACH;
    output.state = TRAJECTORY_STATE_RUNNING;
    output.result = TRAJECTORY_RESULT_OK;
    output.reference.pose.x_mm = 0.0f;
    output.reference.pose.y_mm = 100.0f;
    output.reference.pose.z_mm = 0.0f;
    output.reference.pose.yaw_deg = 90.0f;

    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("running endpoint reference was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();

    output.state = TRAJECTORY_STATE_COMPLETE;
    captured_yaw_speed_rpm = 0U;
    capture_yaw_move = 1U;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("complete endpoint reference was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    if (capture_yaw_move != 0U || captured_yaw_speed_rpm != config.yaw_speed_rpm) {
        fprintf(stderr,
                "complete yaw speed %u RPM, expected %u RPM\n",
                (unsigned)captured_yaw_speed_rpm,
                (unsigned)config.yaw_speed_rpm);
        return 1;
    }
    return 0;
}

static int test_complete_waits_for_both_axes_to_arrive(void)
{
    CraneControlConfig config = test_config();
    RoutePlanningOutput output;
    CraneControlState state;
    uint32_t yaw_commands_after_first_complete;
    uint32_t reach_commands_after_first_complete;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }

    (void)memset(&output, 0, sizeof(output));
    output.plan_id = 7U;
    output.phase = TRAJECTORY_PHASE_APPROACH;
    output.state = TRAJECTORY_STATE_COMPLETE;
    output.result = TRAJECTORY_RESULT_OK;
    output.reference.pose.x_mm = 0.0f;
    output.reference.pose.y_mm = 100.0f;
    output.reference.pose.z_mm = 0.0f;
    output.reference.pose.yaw_deg = 90.0f;

    yaw_arrival = PD42S1_ARRIVAL_REACHED;
    reach_arrival = PD42S1_ARRIVAL_NOT_REACHED;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("complete output was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    CraneControl_GetState(&state);
    yaw_commands_after_first_complete = yaw_absolute_command_count;
    reach_commands_after_first_complete = reach_absolute_command_count;
    if (state.yaw_at_target == 0U || state.reach_at_target != 0U ||
        state.axes_at_target != 0U) {
        fputs("controller reported arrival before both axes arrived\n", stderr);
        return 1;
    }

    reach_arrival = PD42S1_ARRIVAL_REACHED;
    if (CraneControl_SubmitPlannerOutput(&output) != CRANE_CONTROL_OK) {
        fputs("repeated complete output was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    CraneControl_GetState(&state);
    if (yaw_absolute_command_count != yaw_commands_after_first_complete ||
        reach_absolute_command_count != reach_commands_after_first_complete) {
        fputs("repeated complete sample restarted a stepper command\n", stderr);
        return 1;
    }
    if (state.axes_at_target == 0U) {
        fputs("controller did not report arrival after both axes arrived\n",
              stderr);
        return 1;
    }
    return 0;
}

static int test_lift_is_controlled_explicitly(void)
{
    CraneControlConfig config = test_config();
    CraneControlState state;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }
    captured_lift_immediate_count = 0U;

    if (CraneControl_CommandLift(CRANE_LIFT_LOWERED) != CRANE_CONTROL_OK) {
        fputs("lower lift command was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    CraneControl_GetState(&state);
    if (captured_lift_immediate_count != 1U ||
        fabsf(captured_lift_angle_deg - SERVO_MAX_ANGLE_DEG) > 0.01f ||
        state.lift_position != CRANE_LIFT_LOWERED) {
        fputs("explicit lower command did not drive the lift to 180 degrees\n",
              stderr);
        return 1;
    }

    if (CraneControl_CommandLift(CRANE_LIFT_RAISED) != CRANE_CONTROL_OK) {
        fputs("raise lift command was rejected\n", stderr);
        return 1;
    }
    CraneControl_Update();
    CraneControl_GetState(&state);
    if (captured_lift_immediate_count != 2U ||
        fabsf(captured_lift_angle_deg - SERVO_CENTER_ANGLE_DEG) > 0.01f ||
        state.lift_position != CRANE_LIFT_RAISED) {
        fputs("explicit raise command did not drive the lift to 90 degrees\n",
              stderr);
        return 1;
    }
    return 0;
}

static int test_axis_arrival_has_a_timeout(void)
{
    CraneControlConfig config = test_config();
    RoutePlanningOutput output;
    CraneControlState state;

    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }
    (void)memset(&output, 0, sizeof(output));
    output.plan_id = 9U;
    output.phase = TRAJECTORY_PHASE_TRANSFER;
    output.state = TRAJECTORY_STATE_COMPLETE;
    output.result = TRAJECTORY_RESULT_OK;
    output.reference.pose.x_mm = 0.0f;
    output.reference.pose.y_mm = 100.0f;
    output.reference.pose.z_mm = 0.0f;
    output.reference.pose.yaw_deg = 90.0f;
    yaw_arrival = PD42S1_ARRIVAL_NOT_REACHED;
    reach_arrival = PD42S1_ARRIVAL_NOT_REACHED;

    (void)CraneControl_SubmitPlannerOutput(&output);
    CraneControl_Update();
    fake_tick += config.arrival_timeout_ms;
    (void)CraneControl_SubmitPlannerOutput(&output);
    CraneControl_Update();
    CraneControl_GetState(&state);
    yaw_arrival = PD42S1_ARRIVAL_REACHED;
    reach_arrival = PD42S1_ARRIVAL_REACHED;

    if (state.status != CRANE_CONTROL_ARRIVAL_TIMEOUT) {
        fprintf(stderr, "arrival timeout status %u, expected %u\n",
                (unsigned)state.status,
                (unsigned)CRANE_CONTROL_ARRIVAL_TIMEOUT);
        return 1;
    }
    return 0;
}

static int test_measured_transfer_yaw_fits_the_wrist(void)
{
    CraneControlConfig config = test_config();
    TrajectoryPose poses[4];
    float bias_deg = 0.0f;
    uint8_t index;

    config.origin.x_mm = 148.5f;
    config.origin.y_mm = -50.0f;
    if (CraneControl_Init(&config) != CRANE_CONTROL_OK) {
        fputs("CraneControl_Init failed\n", stderr);
        return 1;
    }

    (void)memset(poses, 0, sizeof(poses));
    poses[0].x_mm = 49.8666687f;
    poses[0].y_mm = 37.3500023f;
    poses[0].z_mm = -39.0f;
    poses[0].yaw_deg = 0.0f;
    poses[1] = poses[0];
    poses[1].z_mm = 0.0f;
    poses[2].x_mm = 213.357346f;
    poses[2].y_mm = 101.494759f;
    poses[2].z_mm = 0.0f;
    poses[2].yaw_deg = -131.536621f;
    poses[3] = poses[2];
    poses[3].z_mm = -39.0f;

    if (CraneControl_ChooseYawBias(poses, 4U, &bias_deg) !=
            CRANE_CONTROL_OK ||
        fabsf(bias_deg - 168.4159f) > 0.05f) {
        fprintf(stderr, "measured transfer bias %.3f, expected 168.416\n",
                bias_deg);
        return 1;
    }
    for (index = 0U; index < 4U; ++index) {
        poses[index].yaw_deg += bias_deg;
        if (CraneControl_CheckPose(&poses[index]) != CRANE_CONTROL_OK) {
            fprintf(stderr, "biased measured pose %u is outside the workspace\n",
                    (unsigned)index);
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    if (test_reverse_home_reports_maximum_yaw_stop() != 0) {
        return 1;
    }
    if (test_yaw_commands_absolute_positions_from_the_homed_datum() != 0) {
        return 1;
    }
    if (test_planner_z_does_not_control_lift() != 0) {
        return 1;
    }
    if (test_complete_reference_uses_full_speed_to_settle() != 0) {
        return 1;
    }
    if (test_complete_waits_for_both_axes_to_arrive() != 0) {
        return 1;
    }
    if (test_lift_is_controlled_explicitly() != 0) {
        return 1;
    }
    if (test_axis_arrival_has_a_timeout() != 0) {
        return 1;
    }
    if (test_measured_transfer_yaw_fits_the_wrist() != 0) {
        return 1;
    }
    puts("crane control tests passed");
    return 0;
}

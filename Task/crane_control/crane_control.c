#include "crane_control.h"

#include "Servo.h"
#include "pd42s1.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#define CRANE_PI_F                         3.141592654f
#define CRANE_RAD_TO_DEG                  (180.0f / CRANE_PI_F)
#define CRANE_RADIUS_EPSILON_MM           0.001f
/* How far outside the minimum reach the innermost candidate arc rides, so the
   sub-chords either side of it still clear that bound. */
#define CRANE_TRANSIT_CLEARANCE_MM        6.0f
/* Candidate arc radii tried per leg, from the innermost outwards. */
#define CRANE_TRANSIT_RADIUS_STEPS        16U
/* Past two points the arc gains almost nothing, and the waypoint budget of a
   phase cannot spare more anyway. */
#define CRANE_MAX_TRANSIT_POSES           2U
/* Slack at the travel ends, absorbing float rounding without letting a target
   past anything the mechanism would notice. */
#define CRANE_LIMIT_TOLERANCE_MM          0.05f
#define CRANE_LIMIT_TOLERANCE_DEG         0.05f
/* An axis is accepted when the drive's signed 0x2B position error is strictly
   inside this band. */
#define CRANE_POSITION_ERROR_THRESHOLD_UNITS 200U
/* Consecutive bus faults tolerated before the run is abandoned.
 *
 * A single lost reply used to end the run: mission.c stops the moment
 * crane_status leaves OK, and one late byte on a blocking RS485 link that has to
 * serve two drives inside a 20 ms tick is enough to produce one. That is the
 * mid-run failure that beeped out as CRANE_REFUSED. Now that the targets are
 * absolute, a lost frame costs nothing but the tick - the next one restates the
 * same target - so a fault only means something if it persists. Eight ticks is
 * 160 ms, long enough to rule out a stray timeout and short enough that a
 * genuinely dead bus still stops the arm well inside the time limit. */
#define CRANE_MAX_CONSECUTIVE_BUS_FAULTS  8U
/* Startup has no next control tick to restate these frame-setting commands. */
#define CRANE_STARTUP_COMM_ATTEMPTS       3U
#define CRANE_STARTUP_COMM_RETRY_DELAY_MS 10U
#define CRANE_STARTUP_RESPONSE_TIMEOUT_MS 160U
#define CRANE_POSITION_MODE_POLL_ATTEMPTS 10U
#define CRANE_POSITION_MODE_POLL_DELAY_MS 20U
/* How long startup holds both axes against their stops. Nothing reports arrival in
   torque mode, so this is not a timeout to be beaten but the whole duration of the
   datum move, and it has to cover the worst case: an axis starting from the far end
   of its travel. Arriving early costs only the remainder spent resting against the
   stop, which at CRANE_HOME_CURRENT_MA is a light load rather than a fight. */
#define CRANE_HOME_PUSH_MS                3000U
typedef struct {
    int64_t position_units;
    uint8_t valid;
} CraneMotorPosition;

typedef struct {
    pd42s1_direction_t direction;
    uint32_t position_units;
} CraneMotorCommand;
static CraneControlConfig s_config;
static CraneControlState s_state;
static RoutePlanningOutput s_pending_output;
static RoutePlanningOutput s_active_output;
static CraneMotorPosition s_yaw_position;
static CraneMotorPosition s_reach_position;
static uint8_t s_consecutive_bus_faults;
static uint8_t s_axis_target_active;
static uint32_t s_arrival_wait_start_tick;
/* Where each drive's zero sits in crane coordinates. Without homing the only
   available datum is "wherever the mechanism was standing at reset", so this holds
   the park pose and the mechanism has to actually be parked. Homing replaces it
   with the hard stop each axis was driven into, which is the travel minimum, and
   from then on the datum is a property of the frame rather than of the operator. */
static float s_yaw_datum_deg;
static float s_reach_datum_mm;
static volatile uint8_t s_output_pending;
static volatile uint8_t s_grip_command_pending;
static volatile uint8_t s_pending_grip;
static volatile TrajectoryPose s_last_pose;
static uint8_t s_config_loaded;
static float normalize_angle(float angle_deg)
{
    float normalized = fmodf(angle_deg + 180.0f, 360.0f);

    if (normalized < 0.0f) {
        normalized += 360.0f;
    }
    return normalized - 180.0f;
}

static float unwrap_near(float angle_deg, float reference_deg)
{
    return reference_deg + normalize_angle(angle_deg - reference_deg);
}
static uint8_t sign_is_valid(int8_t sign)
{
    return (uint8_t)(sign == 1 || sign == -1);
}
static uint8_t config_values_are_finite(const CraneControlConfig *config)
{
    return (uint8_t)(isfinite(config->origin.x_mm) &&
                     isfinite(config->origin.y_mm) &&
                     isfinite(config->origin.z_mm) &&
                     isfinite(config->origin.yaw_deg) &&
                     isfinite(config->startup_boom_yaw_deg) &&
                     isfinite(config->yaw_motor_revolutions_per_crane_revolution) &&
                     isfinite(config->reach_zero_radius_mm) &&
                     isfinite(config->reach_mm_per_motor_revolution) &&
                     isfinite(config->lift_zero_angle_deg) &&
                     isfinite(config->lift_mm_per_degree) &&
                     isfinite(config->end_yaw_center_angle_deg) &&
                     isfinite(config->end_yaw_zero_offset_deg) &&
                     isfinite(config->min_boom_yaw_deg) &&
                     isfinite(config->max_boom_yaw_deg) &&
                     isfinite(config->min_radius_mm) &&
                     isfinite(config->max_radius_mm) &&
                     isfinite(config->min_z_mm) &&
                     isfinite(config->max_z_mm));
}
static uint8_t config_ranges_are_valid(const CraneControlConfig *config)
{
    return (uint8_t)(config->min_boom_yaw_deg <= config->max_boom_yaw_deg &&
                     config->startup_boom_yaw_deg >=
                         config->min_boom_yaw_deg &&
                     config->startup_boom_yaw_deg <=
                         config->max_boom_yaw_deg &&
                     config->min_radius_mm <= config->max_radius_mm &&
                     config->min_z_mm <= config->max_z_mm &&
                     config->yaw_motor_revolutions_per_crane_revolution > 0.0f &&
                     config->reach_mm_per_motor_revolution > 0.0f &&
                     config->lift_mm_per_degree > 0.0f &&
                     config->reach_zero_radius_mm >= config->min_radius_mm &&
                     config->reach_zero_radius_mm <= config->max_radius_mm &&
                     config->lift_zero_angle_deg >= SERVO_MIN_ANGLE_DEG &&
                     config->lift_zero_angle_deg <= SERVO_MAX_ANGLE_DEG &&
                     config->end_yaw_center_angle_deg >= SERVO_MIN_ANGLE_DEG &&
                     config->end_yaw_center_angle_deg <= SERVO_MAX_ANGLE_DEG &&
                     config->yaw_speed_rpm > 0U &&
                     config->yaw_speed_rpm <= PD42S1_MAX_SPEED_RPM &&
                     config->reach_speed_rpm > 0U &&
                     config->reach_speed_rpm <= PD42S1_MAX_SPEED_RPM &&
                     config->yaw_acceleration <= PD42S1_MAX_ACCELERATION &&
                     config->reach_acceleration <= PD42S1_MAX_ACCELERATION &&
                     config->expect_stepper_response <= 1U &&
                     config->home_on_startup <= 1U &&
                     config->home_torque_current_ma > 0U &&
                     config->home_torque_current_ma <=
                         PD42S1_MAX_TORQUE_CURRENT_MA &&
                     config->home_push_ms > 0U &&
                     config->arrival_timeout_ms > 0U);
}
static uint8_t config_is_valid(const CraneControlConfig *config)
{
    return (uint8_t)(config != NULL && config_values_are_finite(config) &&
                     config_ranges_are_valid(config) &&
                     sign_is_valid(config->yaw_direction_sign) &&
                     sign_is_valid(config->reach_direction_sign) &&
                     sign_is_valid(config->lift_direction_sign) &&
                     sign_is_valid(config->end_yaw_direction_sign));
}

static uint8_t pose_is_finite(const TrajectoryPose *pose)
{
    return (uint8_t)(isfinite(pose->x_mm) && isfinite(pose->y_mm) &&
                     isfinite(pose->z_mm) && isfinite(pose->yaw_deg));
}

static uint8_t planner_output_is_valid(const RoutePlanningOutput *output)
{
    return (uint8_t)(output != NULL &&
                     output->result == TRAJECTORY_RESULT_OK &&
                      (output->state == TRAJECTORY_STATE_RUNNING ||
                       output->state == TRAJECTORY_STATE_COMPLETE) &&
                      (output->state == TRAJECTORY_STATE_COMPLETE ||
                       (output->waypoint_count >= 2U &&
                        output->waypoint_index > 0U &&
                        output->waypoint_index < output->waypoint_count)) &&
                      pose_is_finite(&output->reference.pose));
}

/* The accessors at the end of this file are usable before startup, so callers
   can lay out their targets in this controller's frame while the crane is still
   parked. */
static void ensure_config(void)
{
    if (s_config_loaded == 0U) {
        CraneControl_LoadDefaultConfig(&s_config);
        CraneControl_CustomizeConfig(&s_config);
        s_config_loaded = 1U;
    }
}

/* Cartesian pose of a boom/reach/wrist posture, i.e. the inverse of
   transform_pose(), used to report where the crane currently is. */
static void pose_from_axes(float boom_yaw_deg,
                           float radius_mm,
                           float z_mm,
                           TrajectoryPose *pose)
{
    const float heading_deg = s_config.origin.yaw_deg + boom_yaw_deg;
    const float heading_rad = heading_deg / CRANE_RAD_TO_DEG;

    pose->x_mm = s_config.origin.x_mm + radius_mm * cosf(heading_rad);
    pose->y_mm = s_config.origin.y_mm + radius_mm * sinf(heading_rad);
    pose->z_mm = z_mm;
    /* A centred wrist holds this world yaw. */
    pose->yaw_deg = normalize_angle(heading_deg +
                                    s_config.end_yaw_zero_offset_deg);
}

static void publish_pose(const TrajectoryPose *pose)
{
    taskENTER_CRITICAL();
    s_last_pose = *pose;
    taskEXIT_CRITICAL();
}

static void publish_state(CraneControlStatus status,
                          const RoutePlanningOutput *output,
                          const CraneActuatorTarget *target)
{
    taskENTER_CRITICAL();
    s_state.status = status;
    if (output != NULL) {
        s_state.plan_id = output->plan_id;
        s_state.phase = output->phase;
        s_state.planner_state = output->state;
        s_state.waypoint_index = output->waypoint_index;
    }
    if (target != NULL) {
        s_state.target = *target;
    }
    taskEXIT_CRITICAL();
}

static void publish_arrival_flags(uint8_t yaw_at_target,
                                  uint8_t reach_at_target)
{
    taskENTER_CRITICAL();
    s_state.yaw_at_target = yaw_at_target;
    s_state.reach_at_target = reach_at_target;
    s_state.axes_at_target =
        (uint8_t)(yaw_at_target != 0U && reach_at_target != 0U);
    taskEXIT_CRITICAL();
}

static uint8_t position_units_are_within_threshold(int32_t position_units)
{
    const uint32_t magnitude = position_units < 0
                                   ? (uint32_t)(-(int64_t)position_units)
                                   : (uint32_t)position_units;

    return (uint8_t)(magnitude < CRANE_POSITION_ERROR_THRESHOLD_UNITS);
}

static void publish_position_errors(int32_t yaw_error_units,
                                    int32_t reach_error_units)
{
    const uint8_t yaw_at_target =
        position_units_are_within_threshold(yaw_error_units);
    const uint8_t reach_at_target =
        position_units_are_within_threshold(reach_error_units);

    taskENTER_CRITICAL();
    s_state.yaw_position_error_units = yaw_error_units;
    s_state.reach_position_error_units = reach_error_units;
    s_state.yaw_at_target = yaw_at_target;
    s_state.reach_at_target = reach_at_target;
    s_state.axes_at_target =
        (uint8_t)(yaw_at_target != 0U && reach_at_target != 0U);
    taskEXIT_CRITICAL();
}

/* The lift uses two fixed positions. Planner Z still selects whether the retained
   target is raised or lowered, but it is not converted into an angle. */
static float lift_angle_for_z(float z_mm)
{
    return z_mm < s_config.max_z_mm
               ? SERVO_LIFT_LOWERED_ANGLE_DEG
               : SERVO_LIFT_RAISED_ANGLE_DEG;
}

/* Maps one planner pose onto the boom, reach, and wrist axes. Lift and grip are
   explicit decision-state commands and are deliberately left unchanged. */
static void transform_pose(const TrajectoryPose *pose,
                           float reference_boom_yaw_deg,
                           CraneActuatorTarget *target)
{
    float dx = pose->x_mm - s_config.origin.x_mm;
    float dy = pose->y_mm - s_config.origin.y_mm;
    float world_heading_deg;
    float relative_end_yaw_deg;

    target->radius_mm = sqrtf(dx * dx + dy * dy);
    world_heading_deg = target->radius_mm > CRANE_RADIUS_EPSILON_MM
                            ? atan2f(dy, dx) * CRANE_RAD_TO_DEG
                            : s_config.origin.yaw_deg + reference_boom_yaw_deg;
    target->boom_yaw_deg = unwrap_near(
        normalize_angle(world_heading_deg - s_config.origin.yaw_deg),
        reference_boom_yaw_deg);
    relative_end_yaw_deg = normalize_angle(pose->yaw_deg -
        world_heading_deg - s_config.end_yaw_zero_offset_deg);
    target->end_yaw_servo_angle_deg = s_config.end_yaw_center_angle_deg +
        (float)s_config.end_yaw_direction_sign * relative_end_yaw_deg;
}

static void transform_reference(const TrajectoryReference *reference,
                                CraneActuatorTarget *target)
{
    transform_pose(&reference->pose, s_state.target.boom_yaw_deg, target);
}

/* Boom, reach, and wrist travel, which depend only on the planner pose. The
   travel ends are held to CRANE_LIMIT_TOLERANCE_MM rather than exactly, because
   the park pose sits on the minimum reach: a reference meant to reach it lands a
   float rounding either side, and the strict comparison would reject half of
   them. The tolerance is well under one step of either drive. */
static uint8_t pose_axes_are_in_workspace(const CraneActuatorTarget *target)
{
    return (uint8_t)(target->boom_yaw_deg >=
                         s_config.min_boom_yaw_deg - CRANE_LIMIT_TOLERANCE_DEG &&
                     target->boom_yaw_deg <=
                         s_config.max_boom_yaw_deg + CRANE_LIMIT_TOLERANCE_DEG &&
                     target->radius_mm >=
                         s_config.min_radius_mm - CRANE_LIMIT_TOLERANCE_MM &&
                     target->radius_mm <=
                         s_config.max_radius_mm + CRANE_LIMIT_TOLERANCE_MM &&
                     target->end_yaw_servo_angle_deg >= SERVO_MIN_ANGLE_DEG &&
                     target->end_yaw_servo_angle_deg <= SERVO_MAX_ANGLE_DEG);
}

static uint8_t target_is_in_workspace(const CraneActuatorTarget *target)
{
    return (uint8_t)(pose_axes_are_in_workspace(target) &&
                     target->z_mm >= s_config.min_z_mm &&
                     target->z_mm <= s_config.max_z_mm &&
                     target->lift_servo_angle_deg >= SERVO_MIN_ANGLE_DEG &&
                     target->lift_servo_angle_deg <= SERVO_MAX_ANGLE_DEG);
}

/* Planner targets are absolute in the crane frame and are sent to the drive as
   absolute positions too, so the two never have to agree about history. The
   earlier scheme sent signed increments from a software-held position, which made
   every lost, clipped, or refused frame a permanent offset: the drive had moved
   less than the software believed and nothing ever corrected it, so the boom
   walked away from the plan over a run. An absolute command is idempotent - the
   manual notes that repeating one produces no motion - so a dropped frame costs
   at most the 20 ms until the next tick restates the same target.
 *
 * The drive's own zero is established by clearing its position counter (0xF8),
 * which is what makes its absolute frame and this one the same frame. Done at
 * startup alone that still assumes the mechanism was left where it was, which is
 * why the real datum comes from homing into the hard stops and clearing there;
 * see CraneControl_Home(). */
static uint8_t revolutions_to_position(float revolutions,
                                       CraneMotorPosition *position)
{
    double magnitude_units = fabs((double)revolutions) *
                             PD42S1_POSITION_UNITS_PER_REVOLUTION;
    int64_t rounded_units;

    if (!isfinite(revolutions) || magnitude_units > (double)UINT32_MAX) {
        return 0U;
    }
    rounded_units = (int64_t)(magnitude_units + 0.5);
    position->position_units = revolutions < 0.0f ? -rounded_units
                                                  : rounded_units;
    position->valid = 1U;
    return 1U;
}

static uint8_t position_has_changed(const CraneMotorPosition *next,
                                    const CraneMotorPosition *previous,
                                    uint8_t force)
{
    uint64_t difference;

    if (force != 0U || previous->valid == 0U) {
        return 1U;
    }
    difference = next->position_units >= previous->position_units
                     ? (uint64_t)(next->position_units -
                                  previous->position_units)
                     : (uint64_t)(previous->position_units -
                                  next->position_units);
    return (uint8_t)(difference >= s_config.min_stepper_change_units);
}

/* The drive takes an absolute target as a direction byte plus an unsigned
   magnitude, so a signed position splits into the two. */
static uint8_t absolute_command_from_position(const CraneMotorPosition *next,
                                              CraneMotorCommand *command)
{
    const uint64_t magnitude_units = next->position_units < 0
                                         ? (uint64_t)(-next->position_units)
                                         : (uint64_t)next->position_units;

    if (magnitude_units > UINT32_MAX) {
        return 0U;
    }
    command->direction = next->position_units < 0 ? PD42S1_DIRECTION_REVERSE
                                                  : PD42S1_DIRECTION_FORWARD;
    command->position_units = (uint32_t)magnitude_units;
    return 1U;
}

static CraneControlStatus send_motor_command(uint8_t motor_id,
                                             uint16_t speed_rpm,
                                             uint8_t acceleration,
                                             const CraneMotorCommand *command)
{
    pd42s1_result_t result;
    max485_status_t status = pd42s1_move_absolute(
        motor_id, command->direction, acceleration, speed_rpm,
        command->position_units);

    if (status != MAX485_STATUS_OK) {
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    if (s_config.expect_stepper_response == 0U) {
        return CRANE_CONTROL_OK;
    }
    status = pd42s1_receive_response(motor_id,
                                     PD42S1_COMMAND_ABSOLUTE_POSITION,
                                     &result, PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    return result == PD42S1_RESULT_SUCCESS ? CRANE_CONTROL_OK
                                           : CRANE_CONTROL_DRIVE_REJECTED;
}

static CraneControlStatus command_axis(uint8_t motor_id,
                                       uint16_t speed_rpm,
                                       uint8_t acceleration,
                                       float target_revolutions,
                                       uint8_t force,
                                       CraneMotorPosition *previous)
{
    CraneMotorPosition next = {0};
    CraneMotorCommand absolute = {0};
    CraneControlStatus status;

    if (revolutions_to_position(target_revolutions, &next) == 0U ||
        absolute_command_from_position(&next, &absolute) == 0U) {
        return CRANE_CONTROL_OUT_OF_WORKSPACE;
    }
    /* Only a bus-traffic saving now: an absolute target the drive already holds
       produces no motion, so skipping it changes nothing but the load on a link
       that has to serve two motors inside one 20 ms tick. */
    if (position_has_changed(&next, previous, force) == 0U) {
        return CRANE_CONTROL_OK;
    }
    status = send_motor_command(motor_id,
                                speed_rpm,
                                acceleration, &absolute);
    if (status == CRANE_CONTROL_OK) {
        *previous = next;
    }
    return status;
}

/* Sends one of the drive's no-payload commands and waits for it to be accepted.
 *
 * They are all idempotent, so a lost reply is worth retrying rather than
 * reporting: startup has no next tick to restate anything on, and the calls that
 * use this are the ones that decide what frame the run is measured in. */
static CraneControlStatus send_simple_command(uint8_t motor_id,
                                              pd42s1_command_t command,
                                              uint32_t retry_delay_ms,
                                              uint32_t response_timeout_ms)
{
    CraneControlStatus worst = CRANE_CONTROL_OK;
    uint8_t attempt;

    for (attempt = 0U; attempt < CRANE_STARTUP_COMM_ATTEMPTS; ++attempt) {
        pd42s1_result_t result;
        max485_status_t status;

        switch (command) {
        case PD42S1_COMMAND_CLEAR_POSITION:
            status = pd42s1_clear_position(motor_id);
            break;
        case PD42S1_COMMAND_RELEASE_STALL_PROTECTION:
            status = pd42s1_release_stall_protection(motor_id);
            break;
        case PD42S1_COMMAND_CLEAR_STATE:
            status = pd42s1_clear_state(motor_id);
            break;
        default:
            return CRANE_CONTROL_INVALID_CONFIG;
        }
        if (status != MAX485_STATUS_OK) {
            worst = CRANE_CONTROL_TRANSPORT_ERROR;
        } else if (s_config.expect_stepper_response == 0U) {
            return CRANE_CONTROL_OK;
        } else {
            status = pd42s1_receive_response(motor_id, command, &result,
                                            response_timeout_ms);
            if (status != MAX485_STATUS_OK) {
                worst = CRANE_CONTROL_TRANSPORT_ERROR;
            } else if (result != PD42S1_RESULT_SUCCESS) {
                worst = CRANE_CONTROL_DRIVE_REJECTED;
            } else {
                return CRANE_CONTROL_OK;
            }
        }
        if ((uint8_t)(attempt + 1U) < CRANE_STARTUP_COMM_ATTEMPTS &&
            retry_delay_ms != 0U) {
            HAL_Delay(retry_delay_ms);
        }
    }
    return worst;
}

static CraneControlStatus read_zero_position(uint8_t motor_id)
{
    CraneControlStatus status = CRANE_CONTROL_OK;
    uint8_t attempt;

    for (attempt = 0U; attempt < CRANE_STARTUP_COMM_ATTEMPTS; ++attempt) {
        int32_t position_units;
        const max485_status_t bus_status = pd42s1_read_realtime_position(
            motor_id, &position_units, CRANE_STARTUP_RESPONSE_TIMEOUT_MS);

        if (bus_status != MAX485_STATUS_OK) {
            status = CRANE_CONTROL_TRANSPORT_ERROR;
        } else if (position_units_are_within_threshold(position_units) == 0U) {
            status = CRANE_CONTROL_DRIVE_REJECTED;
        } else {
            return CRANE_CONTROL_OK;
        }
        if ((uint8_t)(attempt + 1U) < CRANE_STARTUP_COMM_ATTEMPTS) {
            HAL_Delay(CRANE_STARTUP_COMM_RETRY_DELAY_MS);
        }
    }
    return status;
}

static CraneControlStatus wait_for_position_mode(uint8_t motor_id)
{
    CraneControlStatus status = CRANE_CONTROL_DRIVE_REJECTED;
    uint8_t attempt;

    for (attempt = 0U; attempt < CRANE_POSITION_MODE_POLL_ATTEMPTS; ++attempt) {
        pd42s1_work_mode_t mode;
        const max485_status_t bus_status = pd42s1_read_work_mode(
            motor_id, &mode, CRANE_STARTUP_RESPONSE_TIMEOUT_MS);

        if (bus_status != MAX485_STATUS_OK) {
            status = CRANE_CONTROL_TRANSPORT_ERROR;
        } else if (mode == PD42S1_WORK_MODE_COMMUNICATION_POSITION) {
            return CRANE_CONTROL_OK;
        } else {
            status = CRANE_CONTROL_DRIVE_REJECTED;
        }
        if ((uint8_t)(attempt + 1U) < CRANE_POSITION_MODE_POLL_ATTEMPTS) {
            HAL_Delay(CRANE_POSITION_MODE_POLL_DELAY_MS);
        }
    }
    return status;
}

static CraneControlStatus wait_for_axes_position_mode(void)
{
    CraneControlStatus status = wait_for_position_mode(PD42S1_MOTOR_1_ID);

    if (status != CRANE_CONTROL_OK) {
        return status;
    }
    return wait_for_position_mode(PD42S1_MOTOR_2_ID);
}

/* Makes both drives read zero where the mechanism is standing right now, and
   adopts that as the software position too. Whoever calls this owns the question
   of what pose "here" actually is. */
static CraneControlStatus clear_axis_positions(void)
{
    static const uint8_t motor_ids[2] = {
        PD42S1_MOTOR_1_ID, PD42S1_MOTOR_2_ID
    };
    CraneControlStatus worst = CRANE_CONTROL_OK;
    uint8_t index;

    for (index = 0U; index < 2U; ++index) {
        CraneControlStatus status =
            send_simple_command(motor_ids[index],
                                PD42S1_COMMAND_CLEAR_POSITION,
                                CRANE_STARTUP_COMM_RETRY_DELAY_MS,
                                CRANE_STARTUP_RESPONSE_TIMEOUT_MS);

        if (status != CRANE_CONTROL_OK) {
            worst = status;
        }
    }
    if (worst != CRANE_CONTROL_OK) {
        return worst;
    }
    for (index = 0U; index < 2U; ++index) {
        const CraneControlStatus status = read_zero_position(motor_ids[index]);

        if (status != CRANE_CONTROL_OK) {
            return status;
        }
    }
    s_yaw_position.position_units = 0;
    s_yaw_position.valid = 1U;
    s_reach_position.position_units = 0;
    s_reach_position.valid = 1U;
    return worst;
}

/* Selects a position mode without moving either axis.
 *
 * Relative zero is used before 0xF8 because this drive rejects clearing while
 * communication torque mode is active. Absolute zero is used after 0xF8 to enter
 * the mode used by task targets. Neither command produces physical motion. */
static CraneControlStatus set_axis_position_mode_zero(
    pd42s1_command_t command)
{
    static const uint8_t motor_ids[2] = {
        PD42S1_MOTOR_1_ID, PD42S1_MOTOR_2_ID
    };
    CraneControlStatus worst = CRANE_CONTROL_OK;
    uint8_t index;

    for (index = 0U; index < 2U; ++index) {
        const uint8_t motor_id = motor_ids[index];
        const uint16_t speed_rpm = motor_id == PD42S1_MOTOR_1_ID
                                       ? s_config.yaw_speed_rpm
                                       : s_config.reach_speed_rpm;
        const uint8_t acceleration = motor_id == PD42S1_MOTOR_1_ID
                                         ? s_config.yaw_acceleration
                                         : s_config.reach_acceleration;
        CraneControlStatus status = CRANE_CONTROL_OK;
        uint8_t attempt;

        for (attempt = 0U; attempt < CRANE_STARTUP_COMM_ATTEMPTS; ++attempt) {
            max485_status_t bus_status;

            if (command == PD42S1_COMMAND_RELATIVE_POSITION) {
                bus_status = pd42s1_move_relative(
                    motor_id, PD42S1_DIRECTION_FORWARD,
                    acceleration, speed_rpm, 0U);
            } else if (command == PD42S1_COMMAND_ABSOLUTE_POSITION) {
                bus_status = pd42s1_move_absolute(
                    motor_id, PD42S1_DIRECTION_FORWARD,
                    acceleration, speed_rpm, 0U);
            } else {
                return CRANE_CONTROL_INVALID_CONFIG;
            }

            if (bus_status != MAX485_STATUS_OK) {
                status = CRANE_CONTROL_TRANSPORT_ERROR;
            } else if (s_config.expect_stepper_response != 0U) {
                pd42s1_result_t result;

                bus_status = pd42s1_receive_response(
                    motor_id, command, &result,
                    CRANE_STARTUP_RESPONSE_TIMEOUT_MS);
                if (bus_status != MAX485_STATUS_OK) {
                    status = CRANE_CONTROL_TRANSPORT_ERROR;
                } else if (result != PD42S1_RESULT_SUCCESS) {
                    status = CRANE_CONTROL_DRIVE_REJECTED;
                } else {
                    status = CRANE_CONTROL_OK;
                }
            } else {
                status = CRANE_CONTROL_OK;
            }
            if (status == CRANE_CONTROL_OK) {
                break;
            }
            if ((uint8_t)(attempt + 1U) < CRANE_STARTUP_COMM_ATTEMPTS) {
                HAL_Delay(CRANE_STARTUP_COMM_RETRY_DELAY_MS);
            }
        }
        if (status != CRANE_CONTROL_OK) {
            worst = status;
        }
    }
    return worst;
}

/* Defines the drive's zero as wherever the mechanism is standing at reset, which
   is what makes the drives' absolute frame and this controller's the same frame.
   It replaces a startup move that swung the boom to startup_boom_yaw_deg from an
   assumed zero: nothing had ever datumed the drive, so that assumption was only
   true if the boom happened to be sitting at yaw zero, and if it was not, the
   whole run was offset by however far out it was.
 *
 * So the boom has to be parked at startup_boom_yaw_deg with the reach at its zero
 * radius before reset, which is the requirement CraneControl_Home() removes. This
 * is the fallback for when homing is switched off or did not finish. */
static CraneControlStatus establish_axis_zero(void)
{
    CraneControlStatus status = set_axis_position_mode_zero(
        PD42S1_COMMAND_RELATIVE_POSITION);

    if (status == CRANE_CONTROL_OK) {
        status = clear_axis_positions();
    }
    if (status == CRANE_CONTROL_OK) {
        status = set_axis_position_mode_zero(
            PD42S1_COMMAND_ABSOLUTE_POSITION);
    }

    /* Both drives now read zero, and zero is the pose the mechanism is standing
       in, which without switches has to be taken to be the park pose. */
    s_yaw_datum_deg = s_config.startup_boom_yaw_deg;
    s_reach_datum_mm = s_config.reach_zero_radius_mm;
    /* Reported so a caller can beep about a quiet drive, but not a veto: see
       CraneControl_Init(). */
    return status;
}

static CraneControlStatus update_arrival_state(void)
{
    int32_t yaw_error_units;
    int32_t reach_error_units;
    max485_status_t status;

    if (s_axis_target_active == 0U) {
        return CRANE_CONTROL_OK;
    }
    if (s_config.expect_stepper_response == 0U) {
        publish_arrival_flags(1U, 1U);
        s_axis_target_active = 0U;
        return CRANE_CONTROL_OK;
    }

    status = pd42s1_read_position_error(PD42S1_MOTOR_1_ID,
                                        &yaw_error_units,
                                        PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        publish_arrival_flags(0U, 0U);
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    status = pd42s1_read_position_error(PD42S1_MOTOR_2_ID,
                                        &reach_error_units,
                                        PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        publish_arrival_flags(position_units_are_within_threshold(yaw_error_units),
                              0U);
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    publish_position_errors(yaw_error_units, reach_error_units);
    if (position_units_are_within_threshold(yaw_error_units) != 0U &&
        position_units_are_within_threshold(reach_error_units) != 0U) {
        s_axis_target_active = 0U;
        return CRANE_CONTROL_OK;
    }
    if ((uint32_t)(HAL_GetTick() - s_arrival_wait_start_tick) >=
        s_config.arrival_timeout_ms) {
        s_axis_target_active = 0U;
        return CRANE_CONTROL_ARRIVAL_TIMEOUT;
    }
    return CRANE_CONTROL_OK;
}

static CraneControlStatus apply_output(const RoutePlanningOutput *output)
{
    CraneActuatorTarget target = s_state.target;
    CraneControlStatus status;
    float yaw_revolutions;
    float reach_revolutions;

    transform_reference(&output->reference, &target);
    if (target_is_in_workspace(&target) == 0U) {
        publish_state(CRANE_CONTROL_OUT_OF_WORKSPACE, output, &target);
        return CRANE_CONTROL_OUT_OF_WORKSPACE;
    }
    /* Both axes are measured from wherever the drive's zero was established -
       the park pose at reset, or the hard stop after homing. */
    yaw_revolutions = (float)s_config.yaw_direction_sign *
        (target.boom_yaw_deg - s_yaw_datum_deg) *
        s_config.yaw_motor_revolutions_per_crane_revolution / 360.0f;
    reach_revolutions = (float)s_config.reach_direction_sign *
        (target.radius_mm - s_reach_datum_mm) /
        s_config.reach_mm_per_motor_revolution;
    if (output->state == TRAJECTORY_STATE_RUNNING) {
        publish_arrival_flags(0U, 0U);
        status = command_axis(PD42S1_MOTOR_1_ID, s_config.yaw_speed_rpm,
                              s_config.yaw_acceleration, yaw_revolutions,
                              1U, &s_yaw_position);
        if (status == CRANE_CONTROL_OK) {
            status = command_axis(PD42S1_MOTOR_2_ID,
                                  s_config.reach_speed_rpm,
                                  s_config.reach_acceleration,
                                  reach_revolutions, 1U, &s_reach_position);
        }
        if (status == CRANE_CONTROL_OK) {
            s_active_output = *output;
            s_arrival_wait_start_tick = HAL_GetTick();
            s_axis_target_active = 1U;
        }
    } else {
        status = CRANE_CONTROL_OK;
        s_axis_target_active = 0U;
    }
    if (status == CRANE_CONTROL_OK &&
        Servo_SetAngle(SERVO_END_YAW,
                       target.end_yaw_servo_angle_deg) != HAL_OK) {
        status = CRANE_CONTROL_SERVO_ERROR;
    }
    if (status == CRANE_CONTROL_OK) {
        publish_pose(&output->reference.pose);
    }
    publish_state(status, output, &target);
    return status;
}
static uint8_t take_pending_output(RoutePlanningOutput *output)
{
    uint8_t pending;

    taskENTER_CRITICAL();
    pending = s_output_pending;
    if (pending != 0U) {
        *output = s_pending_output;
        s_output_pending = 0U;
    }
    taskEXIT_CRITICAL();
    return pending;
}

static CraneControlStatus apply_pending_grip_command(void)
{
    uint8_t grip_pending;
    uint8_t grip;

    taskENTER_CRITICAL();
    grip_pending = s_grip_command_pending;
    grip = s_pending_grip;
    s_grip_command_pending = 0U;
    taskEXIT_CRITICAL();

    if (grip_pending != 0U) {
        CraneControl_SetMagnet(grip);
        taskENTER_CRITICAL();
        s_state.target.grip = grip;
        taskEXIT_CRITICAL();
    }
    return CRANE_CONTROL_OK;
}
/* Confirms the same limited-height raised angle used by Servo_Init() before either
   stepper starts homing. The immediate API writes the PWM target here, so no
   software filter settling loop is needed. */
static CraneControlStatus park_lift(void)
{
    if (Servo_SetAngleImmediate(
            SERVO_LIFT, lift_angle_for_z(s_config.max_z_mm)) != HAL_OK) {
        return CRANE_CONTROL_SERVO_ERROR;
    }
    return CRANE_CONTROL_OK;
}

CraneControlStatus CraneControl_Init(const CraneControlConfig *config)
{
    CraneControlConfig default_config;
    TrajectoryPose park_pose;

    if (config == NULL) {
        CraneControl_LoadDefaultConfig(&default_config);
        config = &default_config;
    }
    if (config_is_valid(config) == 0U) {
        return CRANE_CONTROL_INVALID_CONFIG;
    }
    s_config = *config;
    s_config_loaded = 1U;
    (void)memset(&s_state, 0, sizeof(s_state));
    (void)memset(&s_yaw_position, 0, sizeof(s_yaw_position));
    (void)memset(&s_reach_position, 0, sizeof(s_reach_position));
    s_grip_command_pending = 0U;
    s_pending_grip = 0U;
    s_output_pending = 0U;
    s_consecutive_bus_faults = 0U;
    s_axis_target_active = 0U;
    s_arrival_wait_start_tick = 0U;
    s_state.status = CRANE_CONTROL_OK;
    s_state.target.boom_yaw_deg = s_config.startup_boom_yaw_deg;
    s_state.target.z_mm = s_config.max_z_mm;
    s_state.target.lift_servo_angle_deg = lift_angle_for_z(s_config.max_z_mm);
    s_state.target.end_yaw_servo_angle_deg =
        s_config.end_yaw_center_angle_deg;
    s_state.lift_position = CRANE_LIFT_RAISED;
    /* Startup parks the boom at startup_boom_yaw_deg with the reach at its zero
       radius and the lift at the top of its stroke, so that is the pose a first
       plan has to start from. */
    s_state.target.radius_mm = s_config.reach_zero_radius_mm;
    pose_from_axes(s_config.startup_boom_yaw_deg,
                   s_config.reach_zero_radius_mm,
                   s_state.target.z_mm,
                   &park_pose);
    publish_pose(&park_pose);
    /* Release the magnet before anything moves, so a reset mid-carry does not
       leave a piece stuck to the tool. */
    CraneControl_SetMagnet(0U);
    pd42s1_init();
    if (Servo_Init() != HAL_OK) {
        s_state.status = CRANE_CONTROL_SERVO_ERROR;
        return s_state.status;
    }
    /* Restate the top of the stroke as an angle, so the lift is at the park height
       before any drive moves - see park_lift(). */
    if (park_lift() != CRANE_CONTROL_OK) {
        s_state.status = CRANE_CONTROL_SERVO_ERROR;
        return s_state.status;
    }
    /* Never enter the ready state unless both drive counters were cleared and
       read back near zero. Otherwise an absolute task target would be interpreted
       in the drive's stale coordinate frame and could send an axis across its
       whole travel. */
    s_state.status = s_config.home_on_startup != 0U
                         ? CraneControl_Home(CRANE_HOME_PUSH_MS)
                         : establish_axis_zero();
    if (s_state.status != CRANE_CONTROL_OK) {
        return s_state.status;
    }
    /* The push left both axes resting where they stopped and clear_axis_positions()
       named that zero, so the position both drives hold is already the one this
       controller wants them at. Restating it as an absolute zero is what makes the
       first tick of the control loop a no-op instead of a move: s_yaw_position and
       s_reach_position both read zero, so the first planner sample that lands on the
       datum pose changes nothing, and one that does not is a move from a position
       the drive really is at. */
    s_state.initialized = 1U;
    return CRANE_CONTROL_OK;
}

/* Holds a steady current towards one axis's stop, without asking for a position.
 *
 * Torque mode is the whole point: 0xF0 names a direction and a current and nothing
 * else, so the axis pulls until something stops it and then simply sits there
 * loaded. There is no target to reach, so there is nothing for the drive to
 * overshoot, retreat from, or report as failed - which is what the drive's own
 * homing kept doing.
 *
 * The current has to be low enough to be a push rather than a fight, because the
 * axis spends the rest of the push against the frame. It also has to stay under the
 * drive's stall current (1500 mA out of the box) so this does not latch the stall
 * protection it used to have to clear. */
static CraneControlStatus send_torque(uint8_t motor_id,
                                      pd42s1_direction_t direction,
                                      uint16_t current_ma)
{
    pd42s1_result_t result;
    max485_status_t status = pd42s1_set_torque(motor_id, direction, current_ma);

    if (status != MAX485_STATUS_OK) {
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    if (s_config.expect_stepper_response == 0U) {
        return CRANE_CONTROL_OK;
    }
    status = pd42s1_receive_response(motor_id, PD42S1_COMMAND_TORQUE, &result,
                                     PD42S1_UART_TIMEOUT_MS);
    if (status != MAX485_STATUS_OK) {
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    return result == PD42S1_RESULT_SUCCESS ? CRANE_CONTROL_OK
                                          : CRANE_CONTROL_DRIVE_REJECTED;
}

static CraneControlStatus push_axis(uint8_t motor_id,
                                    pd42s1_direction_t direction)
{
    /* Anything the previous run latched - a stall, a brake, a disable - would keep
       the push from moving the axis at all, and unlike a homing seek there is no
       state to read back that would say so. Its own outcome is not decisive: on a
       drive with nothing latched it is a no-op. */
    (void)send_simple_command(motor_id, PD42S1_COMMAND_CLEAR_STATE, 0U,
                              PD42S1_UART_TIMEOUT_MS);
    return send_torque(motor_id, direction, s_config.home_torque_current_ma);
}

/* Ends the push and leaves the axis holding where it stopped.
 *
 * Torque mode does not end by itself: a drive left in it keeps pulling. Zero
 * current stops the pull, and clearing the state afterwards covers anything the
 * push latched. Neither is allowed to be skipped on the way to reporting an error,
 * so this takes the status so far and returns the worse of the two. */
static CraneControlStatus release_axis(uint8_t motor_id,
                                       CraneControlStatus so_far)
{
    /* Direction is irrelevant at zero current; forward is simply the valid one. */
    CraneControlStatus status = send_torque(motor_id,
                                            PD42S1_DIRECTION_FORWARD, 0U);

    if (so_far != CRANE_CONTROL_OK) {
        status = so_far;
    }
    {
        const CraneControlStatus cleared =
            send_simple_command(motor_id, PD42S1_COMMAND_CLEAR_STATE, 0U,
                                PD42S1_UART_TIMEOUT_MS);

        if (status == CRANE_CONTROL_OK) {
            status = cleared;
        }
    }
    return status;
}

CraneControlStatus CraneControl_Home(uint32_t push_ms)
{
    CraneControlStatus status;
    CraneControlStatus reach_status;

    if (s_config_loaded == 0U) {
        return CRANE_CONTROL_NOT_INITIALIZED;
    }
    if (s_config.home_on_startup == 0U) {
        return CRANE_CONTROL_OK;
    }
    if (push_ms == 0U) {
        push_ms = s_config.home_push_ms;
    }
    /* Both axes are pushed at the same time, and the push is all there is: no seek,
       no target, no state to poll. The drive's own switchless homing is not used at
       all any more, because two of its properties could not be worked around from
       here. It travels to its configured origin position after finding the stop,
       which is what made the reach spring back out; and its outcome only arrives as
       a state code, which stayed silent for the boom no matter what this code did
       about the origin position or the timeout. Torque mode has neither problem -
       it is a direction and a current, so the axis pulls until the mechanism stops
       it, and then rests there.
     *
     * Pushing both at once is safe here in a way that a simultaneous *seek* would
     * not have been. The reach only ever moves inwards, so the arm's radius only
     * shrinks for the whole push, and the boom's stop is at a fixed angle it can
     * reach from anywhere in its arc. Nothing about the boom's rotation depends on
     * how far the reach has got, so the ordering that mattered for homing - reach
     * first, so the boom never sweeps an extended arm - is satisfied by the reach
     * being inbound the entire time rather than by waiting for it to arrive.
     *
     * REVERSE draws the reach in and drives the boom into its measured +90 deg
     * stop. The yaw datum below must name that same physical end; assigning the
     * opposite end makes the first task command continue towards the stop instead
     * of leaving it. */
    status = push_axis(PD42S1_MOTOR_2_ID, PD42S1_DIRECTION_REVERSE);
    reach_status = status;
    {
        const CraneControlStatus yaw_status =
            push_axis(PD42S1_MOTOR_1_ID, PD42S1_DIRECTION_REVERSE);

        if (status == CRANE_CONTROL_OK) {
            status = yaw_status;
        }
    }
    /* Held for the whole budget rather than until something says it has arrived,
       because in torque mode nothing does. The wait therefore has to cover the worst
       case, which is an axis starting from the far end of its travel; arriving early
       just means resting against the stop for the remainder. */
    HAL_Delay(push_ms);
    /* Both drives have to come out of torque mode whatever happened, or they keep
       pulling into their stops for the rest of the run. So this is not conditional on
       the pushes having worked, and the reach is released first: the arm is drawn in,
       and that is the axis that must not be left loaded against the frame. */
    reach_status = release_axis(PD42S1_MOTOR_2_ID, reach_status);
    status = release_axis(PD42S1_MOTOR_1_ID, status);
    if (reach_status != CRANE_CONTROL_OK) {
        status = reach_status;
    }
    if (status != CRANE_CONTROL_OK) {
        return status;
    }
    /* A zero-step relative command exits communication torque mode without
       moving the mechanism. This mode transition is required before 0xF8. */
    status = set_axis_position_mode_zero(PD42S1_COMMAND_RELATIVE_POSITION);
    if (status != CRANE_CONTROL_OK) {
        return status;
    }
    status = wait_for_axes_position_mode();
    if (status != CRANE_CONTROL_OK) {
        return status;
    }
    /* The push moved the mechanism, so the drives' counters no longer read what they
       did when startup cleared them. Clearing them here is what puts the stop at
       position zero in the drives' own frame, and it has to happen after the push
       rather than before it: software calls the stop zero, so if the drives were
       only cleared at the power-up position the two frames would differ by the whole
       push, and every absolute target afterwards would name somewhere the mechanism
       had already passed. An absolute target the drive believes it is already at
       produces no motion, which is exactly how a reach that never comes back in
       looks from outside. */
    status = clear_axis_positions();
    if (status != CRANE_CONTROL_OK) {
        return status;
    }
    /* Puts both drives into the absolute mode the run uses, without moving them.
       The counters were just cleared, so zero is the position each drive is already
       at. Doing it here rather than letting the first task target switch modes keeps
       every task command in one absolute coordinate frame. */
    status = set_axis_position_mode_zero(PD42S1_COMMAND_ABSOLUTE_POSITION);
    if (status != CRANE_CONTROL_OK) {
        return status;
    }
    /* Both drives now read zero at their stop, so the datum is what pose the stops
       are: the reach fully drawn in, and the boom at the maximum-yaw end reached by
       the REVERSE torque command. With yaw_direction_sign = -1, targets below this
       angle become positive/FORWARD drive positions and therefore leave the
       REVERSE stop. */
    taskENTER_CRITICAL();
    s_yaw_datum_deg = s_config.max_boom_yaw_deg;
    s_reach_datum_mm = s_config.min_radius_mm;
    /* The mechanism is at its end stops, so that is the pose to report and the one
       a first plan has to start from. */
    s_state.target.boom_yaw_deg = s_yaw_datum_deg;
    s_state.target.radius_mm = s_reach_datum_mm;
    taskEXIT_CRITICAL();
    {
        TrajectoryPose home_pose;

        pose_from_axes(s_yaw_datum_deg, s_reach_datum_mm,
                       s_state.target.z_mm, &home_pose);
        publish_pose(&home_pose);
    }
    return CRANE_CONTROL_OK;
}

CraneControlStatus CraneControl_SubmitPlannerOutput(
    const RoutePlanningOutput *output)
{
    if (planner_output_is_valid(output) == 0U) {
        return CRANE_CONTROL_INVALID_ARGUMENT;
    }
    if (s_state.initialized == 0U) {
        return CRANE_CONTROL_NOT_INITIALIZED;
    }
    taskENTER_CRITICAL();
    s_pending_output = *output;
    s_output_pending = 1U;
    taskEXIT_CRITICAL();
    return CRANE_CONTROL_OK;
}

/* Whether a failed reference is worth abandoning the run for.
 *
 * A bus fault says the reply did not arrive, not that the drive refused: the
 * absolute target either landed or it did not, and either way the next tick sends
 * the same one again. A workspace or servo rejection is different - it is the
 * same verdict on every retry, so reporting it at once is what makes the beep
 * mean something. */
static uint8_t status_is_transient(CraneControlStatus status)
{
    return (uint8_t)(status == CRANE_CONTROL_TRANSPORT_ERROR ||
                     status == CRANE_CONTROL_DRIVE_REJECTED);
}

void CraneControl_Update(void)
{
    RoutePlanningOutput output;
    CraneControlStatus actuator_status;
    uint8_t applied_output = 0U;

    if (take_pending_output(&output) != 0U) {
        const CraneControlStatus status = apply_output(&output);

        applied_output = 1U;
        if (status == CRANE_CONTROL_OK) {
            s_consecutive_bus_faults = 0U;
        } else if (status_is_transient(status) != 0U &&
                   s_consecutive_bus_faults < CRANE_MAX_CONSECUTIVE_BUS_FAULTS) {
            ++s_consecutive_bus_faults;
            taskENTER_CRITICAL();
            s_pending_output = output;
            s_output_pending = 1U;
            taskEXIT_CRITICAL();
            publish_state(CRANE_CONTROL_OK, NULL, NULL);
        }
    }
    if (applied_output == 0U && s_axis_target_active != 0U) {
        const CraneControlStatus status = update_arrival_state();

        if (status == CRANE_CONTROL_OK) {
            s_consecutive_bus_faults = 0U;
            publish_state(CRANE_CONTROL_OK, &s_active_output, NULL);
        } else if (status_is_transient(status) != 0U &&
                   s_consecutive_bus_faults < CRANE_MAX_CONSECUTIVE_BUS_FAULTS) {
            ++s_consecutive_bus_faults;
            publish_state(CRANE_CONTROL_OK, NULL, NULL);
        } else {
            publish_state(status, NULL, NULL);
        }
    }
    actuator_status = apply_pending_grip_command();
    if (actuator_status != CRANE_CONTROL_OK) {
        publish_state(actuator_status, NULL, NULL);
    }
    Servo_Update();
}

void CraneControl_GetState(CraneControlState *state)
{
    if (state == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *state = s_state;
    taskEXIT_CRITICAL();
}

CraneControlStatus CraneControl_CommandLift(CraneLiftPosition position)
{
    float z_mm;
    float angle_deg;

    if (position != CRANE_LIFT_RAISED && position != CRANE_LIFT_LOWERED) {
        return CRANE_CONTROL_INVALID_ARGUMENT;
    }
    if (s_state.initialized == 0U) {
        return CRANE_CONTROL_NOT_INITIALIZED;
    }

    z_mm = position == CRANE_LIFT_LOWERED
               ? s_config.min_z_mm
               : s_config.max_z_mm;
    angle_deg = lift_angle_for_z(z_mm);

    /* The lift is an endpoint actuator with no feedback. Write its PWM in the
       caller's tick so a completed stepper move cannot be followed by a lost or
       delayed mailbox command. Resetting the servo state also prevents the
       periodic filter update from restoring the previous angle. */
    taskENTER_CRITICAL();
    if (Servo_SetAngleImmediate(SERVO_LIFT, angle_deg) != HAL_OK) {
        taskEXIT_CRITICAL();
        return CRANE_CONTROL_SERVO_ERROR;
    }
    s_state.target.z_mm = z_mm;
    s_state.target.lift_servo_angle_deg = angle_deg;
    s_state.lift_position = position;
    s_last_pose.z_mm = z_mm;
    taskEXIT_CRITICAL();
    return CRANE_CONTROL_OK;
}

CraneControlStatus CraneControl_CommandGrip(uint8_t enabled)
{
    if (s_state.initialized == 0U) {
        return CRANE_CONTROL_NOT_INITIALIZED;
    }
    taskENTER_CRITICAL();
    s_pending_grip = enabled != 0U ? 1U : 0U;
    s_grip_command_pending = 1U;
    taskEXIT_CRITICAL();
    return CRANE_CONTROL_OK;
}

void CraneControl_GetConfig(CraneControlConfig *config)
{
    if (config == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    ensure_config();
    *config = s_config;
    taskEXIT_CRITICAL();
}

void CraneControl_GetPoseAt(float boom_yaw_deg,
                            float radius_mm,
                            float z_mm,
                            TrajectoryPose *pose)
{
    if (pose == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    ensure_config();
    pose_from_axes(boom_yaw_deg, radius_mm, z_mm, pose);
    taskEXIT_CRITICAL();
}

void CraneControl_GetCurrentPose(TrajectoryPose *pose)
{
    if (pose == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    if (s_state.initialized == 0U) {
        /* Not started yet, so report the pose startup will park at, lift included:
           a first plan that began at z = 0 would order a stroke the mechanism is
           not at, and the trigger would read it as a descent. */
        ensure_config();
        pose_from_axes(s_config.startup_boom_yaw_deg,
                       s_config.reach_zero_radius_mm, s_config.max_z_mm, pose);
    } else {
        *pose = s_last_pose;
    }
    taskEXIT_CRITICAL();
}

/* World yaw the wrist has to hold at one waypoint, before any bias. */
static float required_world_yaw(const TrajectoryPose *pose)
{
    const float dx = pose->x_mm - s_config.origin.x_mm;
    const float dy = pose->y_mm - s_config.origin.y_mm;
    const float radius_mm = sqrtf(dx * dx + dy * dy);
    const float heading_deg = radius_mm > CRANE_RADIUS_EPSILON_MM
                                  ? atan2f(dy, dx) * CRANE_RAD_TO_DEG
                                  : s_config.origin.yaw_deg;

    /* Wrist angle for zero bias, expressed as an offset from servo centre. */
    return normalize_angle(pose->yaw_deg - heading_deg -
                           s_config.end_yaw_zero_offset_deg);
}

CraneControlStatus CraneControl_ChooseYawBias(const TrajectoryPose *poses,
                                             uint8_t count,
                                             float *bias_deg)
{
    float lowest_deg = 0.0f;
    float highest_deg = 0.0f;
    float travel_deg;
    float centre_deg;
    uint8_t index;

    if (poses == NULL || count == 0U || bias_deg == NULL) {
        return CRANE_CONTROL_INVALID_ARGUMENT;
    }
    taskENTER_CRITICAL();
    ensure_config();
    for (index = 0U; index < count; ++index) {
        float required_deg;

        if (pose_is_finite(&poses[index]) == 0U) {
            taskEXIT_CRITICAL();
            return CRANE_CONTROL_INVALID_ARGUMENT;
        }
        required_deg = (float)s_config.end_yaw_direction_sign *
                       required_world_yaw(&poses[index]);
        /* Keep the run of angles contiguous: a set that straddles +-180 has to
           be unwrapped or its span would look like a full turn. */
        if (index == 0U) {
            lowest_deg = required_deg;
            highest_deg = required_deg;
        } else {
            const float unwrapped = unwrap_near(
                required_deg, 0.5f * (lowest_deg + highest_deg));

            if (unwrapped < lowest_deg) {
                lowest_deg = unwrapped;
            }
            if (unwrapped > highest_deg) {
                highest_deg = unwrapped;
            }
        }
    }
    /* Centre the required span on the servo's own centre. */
    centre_deg = 0.5f * (lowest_deg + highest_deg);
    *bias_deg = -(float)s_config.end_yaw_direction_sign * centre_deg;
    travel_deg = highest_deg - lowest_deg;
    taskEXIT_CRITICAL();

    /* Even centred, a span wider than the servo travel cannot fit. */
    return travel_deg <= (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG)
               ? CRANE_CONTROL_OK
               : CRANE_CONTROL_OUT_OF_WORKSPACE;
}

/* Smallest and largest radius a straight Cartesian leg passes through. Both are
   exact: along a line the radius falls to a single minimum at the foot of the
   perpendicular from the column and rises towards both ends. */
static void leg_radius_bounds(const TrajectoryPose *from,
                              const TrajectoryPose *to,
                              float *min_mm,
                              float *max_mm)
{
    const float ax = from->x_mm - s_config.origin.x_mm;
    const float ay = from->y_mm - s_config.origin.y_mm;
    const float dx = (to->x_mm - s_config.origin.x_mm) - ax;
    const float dy = (to->y_mm - s_config.origin.y_mm) - ay;
    const float length_squared = dx * dx + dy * dy;
    const float radius_a = sqrtf(ax * ax + ay * ay);
    const float radius_b = sqrtf((ax + dx) * (ax + dx) + (ay + dy) * (ay + dy));

    *max_mm = radius_a > radius_b ? radius_a : radius_b;
    *min_mm = radius_a < radius_b ? radius_a : radius_b;
    if (length_squared > CRANE_RADIUS_EPSILON_MM) {
        const float fraction = -(ax * dx + ay * dy) / length_squared;

        if (fraction > 0.0f && fraction < 1.0f) {
            const float fx = ax + fraction * dx;
            const float fy = ay + fraction * dy;

            *min_mm = sqrtf(fx * fx + fy * fy);
        }
    }
}

static uint8_t leg_is_in_reach(const TrajectoryPose *from,
                               const TrajectoryPose *to)
{
    float min_mm;
    float max_mm;

    leg_radius_bounds(from, to, &min_mm, &max_mm);
    return (uint8_t)(min_mm >= s_config.min_radius_mm - CRANE_LIMIT_TOLERANCE_MM &&
                     max_mm <= s_config.max_radius_mm + CRANE_LIMIT_TOLERANCE_MM);
}

/* Lays `wanted` poses along an arc of the given radius, and reports whether
   every resulting sub-leg stays inside the reach band. */
static uint8_t fill_transit_poses(const TrajectoryPose *from,
                                  const TrajectoryPose *to,
                                  uint8_t wanted,
                                  float radius_mm,
                                  TrajectoryPose *poses)
{
    const float ax = from->x_mm - s_config.origin.x_mm;
    const float ay = from->y_mm - s_config.origin.y_mm;
    const float bx = to->x_mm - s_config.origin.x_mm;
    const float by = to->y_mm - s_config.origin.y_mm;
    const float radius_a = sqrtf(ax * ax + ay * ay);
    const float radius_b = sqrtf(bx * bx + by * by);
    const TrajectoryPose *previous = from;
    float heading_a;
    float heading_b;
    float yaw_b;
    uint8_t index;

    if (radius_a <= CRANE_RADIUS_EPSILON_MM ||
        radius_b <= CRANE_RADIUS_EPSILON_MM) {
        /* A leg ending over the column has no heading to interpolate, and the
           end itself is already outside the reach band. */
        return 0U;
    }
    heading_a = atan2f(ay, ax) * CRANE_RAD_TO_DEG;
    heading_b = unwrap_near(atan2f(by, bx) * CRANE_RAD_TO_DEG, heading_a);
    yaw_b = unwrap_near(to->yaw_deg, from->yaw_deg);

    for (index = 0U; index < wanted; ++index) {
        const float fraction = (float)(index + 1U) / (float)(wanted + 1U);
        const float heading_deg = heading_a + fraction * (heading_b - heading_a);
        const float heading_rad = heading_deg / CRANE_RAD_TO_DEG;
        CraneActuatorTarget target = {0};
        TrajectoryPose *pose = &poses[index];

        pose->x_mm = s_config.origin.x_mm + radius_mm * cosf(heading_rad);
        pose->y_mm = s_config.origin.y_mm + radius_mm * sinf(heading_rad);
        /* Hold the height the leg starts at: the lift is triggered by the sign
           of a z change, so a bump here would cost two extra strokes. */
        pose->z_mm = from->z_mm;
        pose->yaw_deg = normalize_angle(from->yaw_deg +
                                        fraction * (yaw_b - from->yaw_deg));
        transform_pose(pose, s_config.startup_boom_yaw_deg, &target);
        if (pose_axes_are_in_workspace(&target) == 0U ||
            leg_is_in_reach(previous, pose) == 0U) {
            return 0U;
        }
        previous = pose;
    }
    return leg_is_in_reach(previous, to);
}

CraneControlStatus CraneControl_PlanTransitPoses(const TrajectoryPose *from,
                                                const TrajectoryPose *to,
                                                TrajectoryPose *poses,
                                                uint8_t capacity,
                                                uint8_t *count)
{
    CraneControlStatus status = CRANE_CONTROL_OUT_OF_WORKSPACE;

    if (from == NULL || to == NULL || count == NULL ||
        (poses == NULL && capacity > 0U) ||
        pose_is_finite(from) == 0U || pose_is_finite(to) == 0U) {
        return CRANE_CONTROL_INVALID_ARGUMENT;
    }
    *count = 0U;

    taskENTER_CRITICAL();
    ensure_config();
    if (leg_is_in_reach(from, to) != 0U) {
        status = CRANE_CONTROL_OK;
    } else {
        uint8_t wanted;
        const uint8_t limit = capacity < CRANE_MAX_TRANSIT_POSES
                                  ? capacity
                                  : CRANE_MAX_TRANSIT_POSES;
        const float inner_mm = s_config.min_radius_mm +
                               CRANE_TRANSIT_CLEARANCE_MM;

        /* Fewest poses first, so a leg that only needs one does not spend a
           waypoint the rest of the phase may want. */
        for (wanted = 1U; wanted <= limit && status != CRANE_CONTROL_OK;
             ++wanted) {
            uint8_t step;

            /* Nearest arc first. The spline overshoots the radius of the poses
               it passes through, so an arc further out than the leg needs
               trades a minimum-reach violation for a maximum-reach one. */
            for (step = 0U; step < CRANE_TRANSIT_RADIUS_STEPS; ++step) {
                const float radius_mm = inner_mm +
                    (s_config.max_radius_mm - inner_mm) * (float)step /
                    (float)(CRANE_TRANSIT_RADIUS_STEPS - 1U);

                if (fill_transit_poses(from, to, wanted, radius_mm, poses) !=
                    0U) {
                    *count = wanted;
                    status = CRANE_CONTROL_OK;
                    break;
                }
            }
        }
    }
    taskEXIT_CRITICAL();
    return status;
}

CraneControlStatus CraneControl_CheckPose(const TrajectoryPose *pose)
{
    CraneActuatorTarget target = {0};
    uint8_t in_workspace;

    if (pose == NULL || pose_is_finite(pose) == 0U) {
        return CRANE_CONTROL_INVALID_ARGUMENT;
    }
    taskENTER_CRITICAL();
    ensure_config();
    /* Check against the boom's own park angle so a pose right over the column,
       where the heading is undefined, does not depend on the live state. */
    transform_pose(pose, s_config.startup_boom_yaw_deg, &target);
    in_workspace = pose_axes_are_in_workspace(&target);
    taskEXIT_CRITICAL();
    return in_workspace != 0U ? CRANE_CONTROL_OK
                              : CRANE_CONTROL_OUT_OF_WORKSPACE;
}

__weak void CraneControl_SetMagnet(uint8_t enabled)
{
    (void)enabled;
}

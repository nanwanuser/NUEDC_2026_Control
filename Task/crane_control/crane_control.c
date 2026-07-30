#include "crane_control.h"
#include "crane_lift_trigger.h"

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
static CraneMotorPosition s_yaw_position;
static CraneMotorPosition s_reach_position;
static CraneLiftTriggerState s_lift_trigger;
static uint8_t s_lift_command_pending;
static volatile uint8_t s_output_pending;
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
                     config->expect_stepper_response <= 1U);
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
    }
    if (target != NULL) {
        s_state.target = *target;
    }
    taskEXIT_CRITICAL();
}

/* Maps one planner pose onto the boom, reach, and wrist axes. The lift is left
   alone here because it is driven by the sign of the z change, not by z. */
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
    s_lift_command_pending = CraneLiftTrigger_Update(
        &s_lift_trigger, reference->pose.z_mm, s_config.min_z_mm,
        s_config.max_z_mm, &target->z_mm);
    target->lift_servo_angle_deg = s_config.lift_zero_angle_deg +
        (float)s_config.lift_direction_sign * target->z_mm /
        s_config.lift_mm_per_degree;
    target->grip = reference->grip != 0U ? 1U : 0U;
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

/* Planner targets remain absolute in the crane frame. Keep the last accepted
   motor target in software and send only the signed increment to the drive. */
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

static uint8_t relative_command_from_positions(
    const CraneMotorPosition *next,
    const CraneMotorPosition *previous,
    CraneMotorCommand *command)
{
    const int64_t previous_units = previous->valid != 0U
                                       ? previous->position_units
                                       : 0;
    const int64_t delta_units = next->position_units - previous_units;
    const uint64_t magnitude_units = delta_units < 0
                                         ? (uint64_t)(-delta_units)
                                         : (uint64_t)delta_units;

    if (magnitude_units > UINT32_MAX) {
        return 0U;
    }
    command->direction = delta_units < 0 ? PD42S1_DIRECTION_REVERSE
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
    max485_status_t status = pd42s1_move_relative(
        motor_id, command->direction, acceleration, speed_rpm,
        command->position_units);

    if (status != MAX485_STATUS_OK) {
        return CRANE_CONTROL_TRANSPORT_ERROR;
    }
    if (s_config.expect_stepper_response == 0U) {
        return CRANE_CONTROL_OK;
    }
    status = pd42s1_receive_response(motor_id,
                                     PD42S1_COMMAND_RELATIVE_POSITION,
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
    CraneMotorCommand relative = {0};
    CraneControlStatus status;

    if (revolutions_to_position(target_revolutions, &next) == 0U ||
        relative_command_from_positions(&next, previous, &relative) == 0U) {
        return CRANE_CONTROL_OUT_OF_WORKSPACE;
    }
    if (position_has_changed(&next, previous, force) == 0U) {
        return CRANE_CONTROL_OK;
    }
    if (relative.position_units == 0U) {
        *previous = next;
        return CRANE_CONTROL_OK;
    }
    status = send_motor_command(motor_id, speed_rpm, acceleration, &relative);
    if (status == CRANE_CONTROL_OK) {
        *previous = next;
    }
    return status;
}

static CraneControlStatus initialize_yaw_axis(void)
{
    float revolutions = (float)s_config.yaw_direction_sign *
        s_config.startup_boom_yaw_deg *
        s_config.yaw_motor_revolutions_per_crane_revolution / 360.0f;

    return command_axis(PD42S1_MOTOR_1_ID, s_config.yaw_speed_rpm,
                        s_config.yaw_acceleration, revolutions, 1U,
                        &s_yaw_position);
}

static CraneControlStatus apply_output(const RoutePlanningOutput *output)
{
    CraneActuatorTarget target;
    CraneControlStatus status;
    float yaw_revolutions;
    float reach_revolutions;
    uint8_t force = output->state == TRAJECTORY_STATE_COMPLETE ? 1U : 0U;

    transform_reference(&output->reference, &target);
    if (target_is_in_workspace(&target) == 0U) {
        publish_state(CRANE_CONTROL_OUT_OF_WORKSPACE, output, &target);
        return CRANE_CONTROL_OUT_OF_WORKSPACE;
    }
    yaw_revolutions = (float)s_config.yaw_direction_sign *
        target.boom_yaw_deg *
        s_config.yaw_motor_revolutions_per_crane_revolution / 360.0f;
    reach_revolutions = (float)s_config.reach_direction_sign *
        (target.radius_mm - s_config.reach_zero_radius_mm) /
        s_config.reach_mm_per_motor_revolution;
    status = command_axis(PD42S1_MOTOR_1_ID, s_config.yaw_speed_rpm,
                          s_config.yaw_acceleration, yaw_revolutions,
                          force, &s_yaw_position);
    if (status == CRANE_CONTROL_OK) {
        status = command_axis(PD42S1_MOTOR_2_ID, s_config.reach_speed_rpm,
                              s_config.reach_acceleration, reach_revolutions,
                              force, &s_reach_position);
    }
    if (status == CRANE_CONTROL_OK && s_lift_command_pending != 0U) {
        if (Servo_SetAngle(SERVO_LIFT,
                           target.lift_servo_angle_deg) != HAL_OK) {
            status = CRANE_CONTROL_SERVO_ERROR;
        } else {
            CraneLiftTrigger_Acknowledge(&s_lift_trigger);
            s_lift_command_pending = 0U;
        }
    }
    if (status == CRANE_CONTROL_OK &&
        Servo_SetAngle(SERVO_END_YAW,
                       target.end_yaw_servo_angle_deg) != HAL_OK) {
        status = CRANE_CONTROL_SERVO_ERROR;
    }
    if (status == CRANE_CONTROL_OK) {
        CraneControl_SetMagnet(target.grip);
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
CraneControlStatus CraneControl_Init(const CraneControlConfig *config)
{
    CraneControlConfig default_config;
    CraneControlStatus status;
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
    CraneLiftTrigger_Init(&s_lift_trigger, 0.0f);
    s_lift_command_pending = 0U;
    s_output_pending = 0U;
    s_state.status = CRANE_CONTROL_OK;
    s_state.target.boom_yaw_deg = s_config.startup_boom_yaw_deg;
    s_state.target.lift_servo_angle_deg = s_config.lift_zero_angle_deg;
    s_state.target.end_yaw_servo_angle_deg =
        s_config.end_yaw_center_angle_deg;
    /* Startup parks the boom at startup_boom_yaw_deg with the reach at its zero
       radius, so that is the pose a first plan has to start from. */
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
    status = initialize_yaw_axis();
    if (status != CRANE_CONTROL_OK) {
        s_state.status = status;
        return status;
    }
    s_state.initialized = 1U;
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

void CraneControl_Update(void)
{
    RoutePlanningOutput output;

    if (take_pending_output(&output) != 0U) {
        (void)apply_output(&output);
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
        /* Not started yet, so report the pose startup will park at. */
        ensure_config();
        pose_from_axes(s_config.startup_boom_yaw_deg,
                       s_config.reach_zero_radius_mm, 0.0f, pose);
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

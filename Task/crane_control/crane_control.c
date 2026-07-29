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
#define CRANE_GEAR_TRAVEL_MM_PER_REV      94.2477796f
#define CRANE_LIFT_TRAVEL_MM_PER_DEG      (CRANE_GEAR_TRAVEL_MM_PER_REV / 360.0f)
#define CRANE_DEFAULT_Z_LIMIT_MM          (CRANE_GEAR_TRAVEL_MM_PER_REV / 4.0f)
#define CRANE_DEFAULT_STEPPER_SPEED_RPM   60U
#define CRANE_DEFAULT_ACCELERATION        50U
#define CRANE_DEFAULT_MIN_CHANGE_UNITS    16U
#define CRANE_RADIUS_EPSILON_MM           0.001f

typedef struct {
    pd42s1_direction_t direction;
    uint32_t position_units;
    uint8_t valid;
} CraneMotorCommand;
static CraneControlConfig s_config;
static CraneControlState s_state;
static RoutePlanningOutput s_pending_output;
static CraneMotorCommand s_yaw_command;
static CraneMotorCommand s_reach_command;
static volatile uint8_t s_output_pending;
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
                     config->stepper_acceleration <= PD42S1_MAX_ACCELERATION &&
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

static void transform_reference(const TrajectoryReference *reference,
                                CraneActuatorTarget *target)
{
    float dx = reference->pose.x_mm - s_config.origin.x_mm;
    float dy = reference->pose.y_mm - s_config.origin.y_mm;
    float world_heading_deg;
    float relative_end_yaw_deg;

    target->radius_mm = sqrtf(dx * dx + dy * dy);
    target->z_mm = reference->pose.z_mm - s_config.origin.z_mm;
    world_heading_deg = target->radius_mm > CRANE_RADIUS_EPSILON_MM
                            ? atan2f(dy, dx) * CRANE_RAD_TO_DEG
                            : s_config.origin.yaw_deg + s_state.target.boom_yaw_deg;
    target->boom_yaw_deg = unwrap_near(
        normalize_angle(world_heading_deg - s_config.origin.yaw_deg),
        s_state.target.boom_yaw_deg);
    target->lift_servo_angle_deg = s_config.lift_zero_angle_deg +
        (float)s_config.lift_direction_sign * target->z_mm /
        s_config.lift_mm_per_degree;
    relative_end_yaw_deg = normalize_angle(reference->pose.yaw_deg -
        world_heading_deg - s_config.end_yaw_zero_offset_deg);
    target->end_yaw_servo_angle_deg = s_config.end_yaw_center_angle_deg +
        (float)s_config.end_yaw_direction_sign * relative_end_yaw_deg;
    target->grip = reference->grip != 0U ? 1U : 0U;
}

static uint8_t target_is_in_workspace(const CraneActuatorTarget *target)
{
    return (uint8_t)(target->boom_yaw_deg >= s_config.min_boom_yaw_deg &&
                     target->boom_yaw_deg <= s_config.max_boom_yaw_deg &&
                     target->radius_mm >= s_config.min_radius_mm &&
                     target->radius_mm <= s_config.max_radius_mm &&
                     target->z_mm >= s_config.min_z_mm &&
                     target->z_mm <= s_config.max_z_mm &&
                     target->lift_servo_angle_deg >= SERVO_MIN_ANGLE_DEG &&
                     target->lift_servo_angle_deg <= SERVO_MAX_ANGLE_DEG &&
                     target->end_yaw_servo_angle_deg >= SERVO_MIN_ANGLE_DEG &&
                     target->end_yaw_servo_angle_deg <= SERVO_MAX_ANGLE_DEG);
}

static uint8_t revolutions_to_command(float revolutions,
                                      CraneMotorCommand *command)
{
    double position_units = fabs((double)revolutions) *
                            PD42S1_POSITION_UNITS_PER_REVOLUTION;

    if (!isfinite(revolutions) || position_units > (double)UINT32_MAX) {
        return 0U;
    }
    command->direction = revolutions < 0.0f ? PD42S1_DIRECTION_REVERSE
                                            : PD42S1_DIRECTION_FORWARD;
    command->position_units = (uint32_t)(position_units + 0.5);
    command->valid = 1U;
    return 1U;
}

static uint8_t command_has_changed(const CraneMotorCommand *next,
                                   const CraneMotorCommand *previous,
                                   uint8_t force)
{
    uint32_t difference;

    if (force != 0U || previous->valid == 0U ||
        next->direction != previous->direction) {
        return 1U;
    }
    difference = next->position_units > previous->position_units
                     ? next->position_units - previous->position_units
                     : previous->position_units - next->position_units;
    return (uint8_t)(difference >= s_config.min_stepper_change_units);
}

static CraneControlStatus send_motor_command(uint8_t motor_id,
                                             uint16_t speed_rpm,
                                             const CraneMotorCommand *command)
{
    pd42s1_result_t result;
    max485_status_t status = pd42s1_move_absolute(
        motor_id, command->direction, s_config.stepper_acceleration,
        speed_rpm, command->position_units);

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
                                       float revolutions,
                                       uint8_t force,
                                       CraneMotorCommand *previous)
{
    CraneMotorCommand next = {0};
    CraneControlStatus status;

    if (revolutions_to_command(revolutions, &next) == 0U) {
        return CRANE_CONTROL_OUT_OF_WORKSPACE;
    }
    if (command_has_changed(&next, previous, force) == 0U) {
        return CRANE_CONTROL_OK;
    }
    status = send_motor_command(motor_id, speed_rpm, &next);
    if (status == CRANE_CONTROL_OK) {
        *previous = next;
    }
    return status;
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
                          yaw_revolutions, force, &s_yaw_command);
    if (status == CRANE_CONTROL_OK) {
        status = command_axis(PD42S1_MOTOR_2_ID, s_config.reach_speed_rpm,
                              reach_revolutions, force, &s_reach_command);
    }
    if (status == CRANE_CONTROL_OK &&
        Servo_SetAngles(target.lift_servo_angle_deg,
                        target.end_yaw_servo_angle_deg) != HAL_OK) {
        status = CRANE_CONTROL_SERVO_ERROR;
    }
    if (status == CRANE_CONTROL_OK) {
        CraneControl_SetMagnet(target.grip);
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

void CraneControl_LoadDefaultConfig(CraneControlConfig *config)
{
    if (config == NULL) {
        return;
    }
    (void)memset(config, 0, sizeof(*config));
    config->yaw_motor_revolutions_per_crane_revolution = 1.0f;
    config->reach_mm_per_motor_revolution = CRANE_GEAR_TRAVEL_MM_PER_REV;
    config->lift_zero_angle_deg = SERVO_CENTER_ANGLE_DEG;
    config->lift_mm_per_degree = CRANE_LIFT_TRAVEL_MM_PER_DEG;
    config->end_yaw_center_angle_deg = SERVO_CENTER_ANGLE_DEG;
    config->min_boom_yaw_deg = -180.0f;
    config->max_boom_yaw_deg = 180.0f;
    config->max_radius_mm = 250.0f;
    config->min_z_mm = -CRANE_DEFAULT_Z_LIMIT_MM;
    config->max_z_mm = CRANE_DEFAULT_Z_LIMIT_MM;
    config->yaw_direction_sign = 1;
    config->reach_direction_sign = 1;
    config->lift_direction_sign = 1;
    config->end_yaw_direction_sign = 1;
    config->yaw_speed_rpm = CRANE_DEFAULT_STEPPER_SPEED_RPM;
    config->reach_speed_rpm = CRANE_DEFAULT_STEPPER_SPEED_RPM;
    config->min_stepper_change_units = CRANE_DEFAULT_MIN_CHANGE_UNITS;
    config->stepper_acceleration = CRANE_DEFAULT_ACCELERATION;
    config->expect_stepper_response = 1U;
}

CraneControlStatus CraneControl_Init(const CraneControlConfig *config)
{
    CraneControlConfig default_config;

    if (config == NULL) {
        CraneControl_LoadDefaultConfig(&default_config);
        config = &default_config;
    }
    if (config_is_valid(config) == 0U) {
        return CRANE_CONTROL_INVALID_CONFIG;
    }
    s_config = *config;
    (void)memset(&s_state, 0, sizeof(s_state));
    (void)memset(&s_yaw_command, 0, sizeof(s_yaw_command));
    (void)memset(&s_reach_command, 0, sizeof(s_reach_command));
    s_output_pending = 0U;
    s_state.status = CRANE_CONTROL_OK;
    s_state.target.lift_servo_angle_deg = s_config.lift_zero_angle_deg;
    s_state.target.end_yaw_servo_angle_deg =
        s_config.end_yaw_center_angle_deg;
    pd42s1_init();
    if (Servo_Init() != HAL_OK) {
        s_state.status = CRANE_CONTROL_SERVO_ERROR;
        return s_state.status;
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

__weak void CraneControl_SetMagnet(uint8_t enabled)
{
    (void)enabled;
}

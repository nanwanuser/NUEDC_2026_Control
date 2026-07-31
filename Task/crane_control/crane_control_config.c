#include "crane_control.h"

#include "Servo.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#define CRANE_DEFAULT_GEAR_TRAVEL_MM_REV  94.2477796f
#define CRANE_DEFAULT_LIFT_MM_PER_DEG     (CRANE_DEFAULT_GEAR_TRAVEL_MM_REV / 360.0f)
#define CRANE_DEFAULT_Z_LIMIT_MM          (CRANE_DEFAULT_GEAR_TRAVEL_MM_REV / 4.0f)
#define CRANE_DEFAULT_STEPPER_SPEED_RPM   60U
#define CRANE_DEFAULT_ACCELERATION        10U
#define CRANE_DEFAULT_MIN_CHANGE_UNITS    16U
#define CRANE_DEFAULT_ARRIVAL_TIMEOUT_MS  12000U
/* World frame, and the one every layer shares: it is the vision end's A4 frame.
 * The sheet lies landscape, its top-left corner is the origin, +X runs right
 * along the long edge (0..297) and +Y runs down the short edge (0..210). Y
 * pointing down makes this left-handed when seen from above, so a rising
 * atan2(dy, dx) sweeps clockwise on the sheet - which is the direction startup
 * homing turns the boom, and why no sign flip is needed between the two.
 *
 * The column stands off the y = 0 long edge, centred on it: half the long edge
 * along x, 50 mm outside the sheet along y. */
#define CRANE_A4_LONG_EDGE_MM             297.0f
#define CRANE_A4_SHORT_EDGE_MM            210.0f
#define CRANE_ORIGIN_X_MM                 (CRANE_A4_LONG_EDGE_MM / 2.0f)
#define CRANE_ORIGIN_Y_MM                 (-50.0f)
/* World height assigned to the logical raised state. The lift controller maps
   that state directly to SERVO_LIFT_INIT_ANGLE_DEG. */
#define CRANE_ORIGIN_Z_MM                 40.0f
/* World heading the boom holds at local yaw zero, i.e. +Y: from the column that
   points across the sheet's y = 0 edge towards the far edge. */
#define CRANE_ZERO_WORLD_YAW_DEG          90.0f
/* Homing turns the boom clockwise onto its stop, and the stop is the +90 deg end
   of the local arc. Local +90 on top of the +Y zero heading is a world heading of
   180 deg, which points along -X: the parked boom lies across the sheet towards
   x = 0, exactly as the mechanism does. Startup therefore parks here and
   CraneControl_Home() adopts max_boom_yaw_deg as the datum. */
#define CRANE_STARTUP_BOOM_YAW_DEG        90.0f
#define CRANE_REACH_ZERO_RADIUS_MM        70.0f
#define CRANE_REACH_TRAVEL_MM             200.0f
#define CRANE_REACH_MM_PER_MOTOR_REV      92.4555f
#define CRANE_GEAR_TRAVEL_MM_PER_REV      94.2478f
#define CRANE_MIN_BOOM_YAW_DEG            (-90.0f)
#define CRANE_MAX_BOOM_YAW_DEG            90.0f
/* The lift is a two-position servo. These Z values describe the planner's raised
   and lowered states only; the control layer commands the physical endpoints
   configured by the servo driver. */
#define CRANE_MIN_LOCAL_Z_MM              (-45.0f)
/* Logical travel height; the lift controller maps it to the fixed raised angle. */
#define CRANE_MAX_LOCAL_Z_MM              0.0f
/* Retained for configuration compatibility; fixed-position lift control does not
   use this value to calculate either commanded angle. */
#define CRANE_LIFT_ZERO_ANGLE_DEG         SERVO_LIFT_INIT_ANGLE_DEG
#define CRANE_END_YAW_CENTER_DEG           90.0f
#define CRANE_YAW_SPEED_RPM               10U
#define CRANE_REACH_SPEED_RPM             50U
/* Each waypoint is one fixed absolute-position command, so the drive owns the
   acceleration and braking profile. Lower values produce gentler ramps. */
#define CRANE_YAW_ACCELERATION            100U
#define CRANE_REACH_ACCELERATION          100U
/* Startup datum. Neither axis has a limit switch, and the drive's own switchless
   homing is not used: it retreats to a configured origin position after finding the
   stop, and its outcome only arrives as a state code. Instead both axes are simply
   held in torque mode towards their stop - the reach drawing in, the boom turning
   towards the positive end of its arc - and whatever they come to rest against is
   taken as zero. See CraneControl_Home().
 *
 * The current has to move the axis against friction and then be gentle enough to
 * sit against the frame for the rest of the push, since nothing reports arrival and
 * the push runs for its full duration. It also wants to stay under the drive's own
 * stall current (1500 mA by default) so the push does not latch stall protection.
 * Too low instead and an axis stops short on friction alone, putting the datum
 * wherever that happened.
 *
 * 3000 ms covers the worst case at this current, which is an axis starting from the
 * far end of its travel: the reach's full 160 mm stroke, or the boom's whole
 * 180 deg arc. Arriving early costs only the remainder spent resting on the stop. */
#define CRANE_HOME_ON_STARTUP             1U
#define CRANE_HOME_CURRENT_MA             800U
#define CRANE_HOME_PUSH_MS                3000U
#define CRANE_DEFAULT_HOME_CURRENT_MA     400U
#define CRANE_DEFAULT_HOME_PUSH_MS        3000U

/* PE12 drives the relay coil that switches the electromagnet. Reassign both
   defines together if the magnet moves to another pin. */
#define CRANE_MAGNET_GPIO_PORT            Magnet_GPIO_Port
#define CRANE_MAGNET_GPIO_PIN             Magnet_Pin
/* A high level closes the relay and energises the magnet. */
#define CRANE_MAGNET_ACTIVE_LEVEL         GPIO_PIN_SET

void CraneControl_LoadDefaultConfig(CraneControlConfig *config)
{
    if (config == NULL) {
        return;
    }
    (void)memset(config, 0, sizeof(*config));
    config->startup_boom_yaw_deg = 0.0f;
    config->yaw_motor_revolutions_per_crane_revolution = 1.0f;
    config->reach_mm_per_motor_revolution = CRANE_DEFAULT_GEAR_TRAVEL_MM_REV;
    config->lift_zero_angle_deg = SERVO_CENTER_ANGLE_DEG;
    config->lift_mm_per_degree = CRANE_DEFAULT_LIFT_MM_PER_DEG;
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
    config->yaw_acceleration = CRANE_DEFAULT_ACCELERATION;
    config->reach_acceleration = CRANE_DEFAULT_ACCELERATION;
    config->expect_stepper_response = 1U;
    config->home_on_startup = 0U;
    config->home_torque_current_ma = CRANE_DEFAULT_HOME_CURRENT_MA;
    config->home_push_ms = CRANE_DEFAULT_HOME_PUSH_MS;
    config->arrival_timeout_ms = CRANE_DEFAULT_ARRIVAL_TIMEOUT_MS;
}

/**
 * @brief Apply the measured A4-board crane geometry and motion settings.
 * @param config Default configuration to override.
 */
void CraneControl_CustomizeConfig(CraneControlConfig *config)
{
    if (config == NULL) {
        return;
    }
    config->origin.x_mm = CRANE_ORIGIN_X_MM;
    config->origin.y_mm = CRANE_ORIGIN_Y_MM;
    config->origin.z_mm = CRANE_ORIGIN_Z_MM;
    config->origin.yaw_deg = CRANE_ZERO_WORLD_YAW_DEG;
    config->startup_boom_yaw_deg = CRANE_STARTUP_BOOM_YAW_DEG;
    config->yaw_motor_revolutions_per_crane_revolution = 1.0f;
    config->reach_zero_radius_mm = CRANE_REACH_ZERO_RADIUS_MM;
    config->reach_mm_per_motor_revolution = CRANE_REACH_MM_PER_MOTOR_REV;
    config->lift_mm_per_degree = CRANE_GEAR_TRAVEL_MM_PER_REV / 360.0f;
    config->lift_zero_angle_deg = CRANE_LIFT_ZERO_ANGLE_DEG;
    config->min_boom_yaw_deg = CRANE_MIN_BOOM_YAW_DEG;
    config->max_boom_yaw_deg = CRANE_MAX_BOOM_YAW_DEG;
    config->min_radius_mm = CRANE_REACH_ZERO_RADIUS_MM;
    config->max_radius_mm = CRANE_REACH_ZERO_RADIUS_MM + CRANE_REACH_TRAVEL_MM;
    config->min_z_mm = CRANE_MIN_LOCAL_Z_MM;
    config->max_z_mm = CRANE_MAX_LOCAL_Z_MM;
    config->end_yaw_center_angle_deg = CRANE_END_YAW_CENTER_DEG;
    config->end_yaw_zero_offset_deg = 0.0f;
    config->yaw_direction_sign = -1;
    config->reach_direction_sign = 1;
    /* Retained for configuration compatibility; endpoint lift control does not
       use the scale or direction to calculate an intermediate servo angle. */
    config->lift_direction_sign = -1;
    config->end_yaw_direction_sign = 1;
    config->yaw_speed_rpm = CRANE_YAW_SPEED_RPM;
    config->reach_speed_rpm = CRANE_REACH_SPEED_RPM;
    config->yaw_acceleration = CRANE_YAW_ACCELERATION;
    config->reach_acceleration = CRANE_REACH_ACCELERATION;
    config->expect_stepper_response = 1U;
    /* Zero this to fall back on being left at the park pose before reset, which is
       the only datum available without the startup push; see CraneControl_Home(). */
    config->home_on_startup = CRANE_HOME_ON_STARTUP;
    config->home_torque_current_ma = CRANE_HOME_CURRENT_MA;
    config->home_push_ms = CRANE_HOME_PUSH_MS;
    config->arrival_timeout_ms = CRANE_DEFAULT_ARRIVAL_TIMEOUT_MS;
}

/**
 * @brief Drive the electromagnet from the planner's grip flag.
 * @param enabled Non-zero energises the magnet.
 * @note MX_GPIO_Init() already owns the pin as a push-pull output driven low,
 *       so the relay starts released. Overrides the weak default.
 */
void CraneControl_SetMagnet(uint8_t enabled)
{
    HAL_GPIO_WritePin(CRANE_MAGNET_GPIO_PORT, CRANE_MAGNET_GPIO_PIN,
                      enabled != 0U
                          ? CRANE_MAGNET_ACTIVE_LEVEL
                          : (CRANE_MAGNET_ACTIVE_LEVEL == GPIO_PIN_SET
                                 ? GPIO_PIN_RESET
                                 : GPIO_PIN_SET));
}

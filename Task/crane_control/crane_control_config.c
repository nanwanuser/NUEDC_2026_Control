#include "crane_control.h"

#include "Servo.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#define CRANE_DEFAULT_GEAR_TRAVEL_MM_REV  94.2477796f
#define CRANE_DEFAULT_LIFT_MM_PER_DEG     (CRANE_DEFAULT_GEAR_TRAVEL_MM_REV / 360.0f)
#define CRANE_DEFAULT_Z_LIMIT_MM          (CRANE_DEFAULT_GEAR_TRAVEL_MM_REV / 4.0f)
#define CRANE_DEFAULT_STEPPER_SPEED_RPM   60U
#define CRANE_DEFAULT_ACCELERATION        50U
#define CRANE_DEFAULT_MIN_CHANGE_UNITS    16U
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
/* World height of local z = 0, which is the top of the lift stroke: measured
   magnet-centre height with the lift servo at its 0 deg end. */
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
#define CRANE_GEAR_TRAVEL_MM_PER_REV      94.2478f
#define CRANE_MIN_BOOM_YAW_DEG            (-90.0f)
#define CRANE_MAX_BOOM_YAW_DEG            90.0f
/* The lift is a two-position servo: CraneLiftTrigger snaps every reference to
   one of these two heights, so the low one *is* the pick depth and nothing
   between them is ever commanded.
 *
 * The datum is now measured rather than derived: at the servo's 0 deg end the
 * magnet centre sits CRANE_ORIGIN_Z_MM = 40 mm above the sheet, and that end is
 * the top of the stroke - the linkage descends as the angle *rises*, which is what
 * lift_direction_sign = -1 says. So local z = 0 is the top, every commanded height
 * is negative, and picking at 1 mm of world height is 1 - 40 = -39 mm of local z.
 *
 * At lift_mm_per_degree = 94.2478/360 = 0.2618 mm/deg those 39 mm cost 149.0 deg,
 * putting the low stop at 0 + 149.0 = 149.0 deg. That is inside the servo's 180 deg
 * but no longer with much to spare: only 31 deg, about 8 mm, is left beyond the
 * pick. If the magnet turns out to need more depth than that, the linkage has to be
 * lowered mechanically rather than asked for here - the stroke is already using
 * five sixths of the servo's travel. */
#define CRANE_MIN_LOCAL_Z_MM              (-39.0f)
/* Travel height, and the top of the stroke. Only has to clear the pieces and the
   assembled rectangle on the way past, which is a couple of millimetres of card,
   so the measured 40 mm of world height here is far more than needed - it is set by
   where the linkage rests at the servo's 0 deg end, not chosen. */
#define CRANE_MAX_LOCAL_Z_MM              0.0f
/* Which servo angle holds z = 0. Now the stroke's upper end, so it is also the
   angle Servo_Init() parks the lift at (SERVO_LIFT_INIT_ANGLE_DEG) - the two have
   to agree or startup jumps the lift once before park_lift() even runs.
 *
 * No longer a free knob for how deep the magnet goes: it cannot be lowered, being
 * already at the servo's 0 deg limit, and raising it would drop the top of the
 * stroke as well as the bottom. Adjust CRANE_MIN_LOCAL_Z_MM for pick depth, within
 * the 31 deg the stroke leaves unused. */
#define CRANE_LIFT_ZERO_ANGLE_DEG         SERVO_LIFT_INIT_ANGLE_DEG
#define CRANE_END_YAW_CENTER_DEG           90.0f
#define CRANE_YAW_SPEED_RPM               40U
#define CRANE_REACH_SPEED_RPM             20U
#define CRANE_YAW_ACCELERATION            10U
#define CRANE_REACH_ACCELERATION          5U
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
#define CRANE_HOME_CURRENT_MA             400U
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
    config->reach_mm_per_motor_revolution = CRANE_GEAR_TRAVEL_MM_PER_REV;
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
    /* The linkage descends as the servo angle rises, so height and angle run
       opposite ways; lift_angle_for_z() needs the sign to say so. */
    config->lift_direction_sign = -1;
    config->end_yaw_direction_sign = -1;
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

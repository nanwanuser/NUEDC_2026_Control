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
#define CRANE_A4_LONG_EDGE_MM             297.0f
#define CRANE_ORIGIN_X_MM                 (CRANE_A4_LONG_EDGE_MM / 2.0f)
#define CRANE_ORIGIN_Y_MM                 (-50.0f)
#define CRANE_ORIGIN_Z_MM                 20.0f
#define CRANE_ZERO_WORLD_YAW_DEG          90.0f
#define CRANE_STARTUP_BOOM_YAW_DEG        90.0f
#define CRANE_REACH_ZERO_RADIUS_MM        70.0f
#define CRANE_REACH_TRAVEL_MM             160.0f
#define CRANE_GEAR_TRAVEL_MM_PER_REV      94.2478f
#define CRANE_MIN_BOOM_YAW_DEG            (-90.0f)
#define CRANE_MAX_BOOM_YAW_DEG            90.0f
/* The lift is a two-position servo: CraneLiftTrigger snaps every reference to
   one of these two heights, so the low one *is* the pick depth and nothing
   between them is ever commanded. -15 mm left the magnet short of the pieces.
 *
 * The servo reaches 0..180 deg and lift_mm_per_degree is 94.2478/360 =
 * 0.2618 mm/deg, so the stroke spans 180 * 0.2618 = 47.1 mm in total and the two
 * stops have to share it. -22 mm of pick depth costs 84.0 deg on its own, which
 * is why the extra 3 mm the magnet needed could not come from here: at a 90 deg
 * zero the low stop was already down to 6 deg. The 3 mm comes out of
 * CRANE_LIFT_ZERO_ANGLE_DEG instead, and the transit height pays for it - see
 * below. */
#define CRANE_MIN_LOCAL_Z_MM              (-22.0f)
/* Travel height. Only has to clear the pieces and the assembled rectangle on the
   way past, which is a couple of millimetres of card, so trading 3 mm of it for
   3 mm of extra reach downwards costs nothing that matters. */
#define CRANE_MAX_LOCAL_Z_MM              17.0f
/* Which servo angle holds z = 0, and the knob that sets how much of the stroke
   lies below the paper. Raising it moves the whole stroke down at 0.2618 mm per
   degree without costing any travel, so this is what to change if the magnet
   still does not reach: +12 deg over the 90 deg centre is the 3 mm that was
   measured short. Both stops have to stay inside the servo's 0..180 deg, which
   at 102 deg puts the low one at 102 - 84.0 = 18.0 deg and the high one at
   102 + 64.9 = 166.9 deg, each with room to spare for servo tolerance. */
#define CRANE_LIFT_ZERO_ANGLE_DEG         102.0f
#define CRANE_END_YAW_CENTER_DEG           90.0f
#define CRANE_YAW_SPEED_RPM               40U
#define CRANE_REACH_SPEED_RPM             20U
#define CRANE_YAW_ACCELERATION            10U
#define CRANE_REACH_ACCELERATION          5U
/* Homing. On now that the boom's rotation has its limit switch: the boom homes
   against the switch, the reach against the hard end it hits when fully drawn in.
   Seek slowly, because both axes are deliberately being driven into their stops.
   The limit current is what the drive uses to recognise the reach's stop, so it
   has to be enough to move the axis and little enough not to fight the frame. */
#define CRANE_HOME_ON_STARTUP             1U
#define CRANE_HOME_SPEED_RPM              15U
#define CRANE_HOME_CURRENT_MA             800U
#define CRANE_DEFAULT_HOME_SPEED_RPM      15U
#define CRANE_DEFAULT_HOME_CURRENT_MA     800U

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
    config->home_speed_rpm = CRANE_DEFAULT_HOME_SPEED_RPM;
    config->home_limit_current_ma = CRANE_DEFAULT_HOME_CURRENT_MA;
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
    config->lift_direction_sign = 1;
    config->end_yaw_direction_sign = -1;
    config->yaw_speed_rpm = CRANE_YAW_SPEED_RPM;
    config->reach_speed_rpm = CRANE_REACH_SPEED_RPM;
    config->yaw_acceleration = CRANE_YAW_ACCELERATION;
    config->reach_acceleration = CRANE_REACH_ACCELERATION;
    config->expect_stepper_response = 1U;
    /* Zero this to fall back on being left at the park pose before reset, which is
       the only datum available without homing; see CraneControl_Home(). */
    config->home_on_startup = CRANE_HOME_ON_STARTUP;
    config->home_speed_rpm = CRANE_HOME_SPEED_RPM;
    config->home_limit_current_ma = CRANE_HOME_CURRENT_MA;
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

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
#define CRANE_MIN_LOCAL_Z_MM              (-15.0f)
#define CRANE_MAX_LOCAL_Z_MM              20.0f
#define CRANE_END_YAW_CENTER_DEG           90.0f
#define CRANE_YAW_SPEED_RPM               40U
#define CRANE_REACH_SPEED_RPM             20U
#define CRANE_YAW_ACCELERATION            10U
#define CRANE_REACH_ACCELERATION          5U

/* PE2 is wired to the third key, which no mission uses, so it drives the
   electromagnet's switch here. Reassign both defines together if the magnet
   moves to its own pin. */
#define CRANE_MAGNET_GPIO_PORT            Key3_GPIO_Port
#define CRANE_MAGNET_GPIO_PIN             Key3_Pin
/* A high level energises the magnet through the switching transistor. */
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
}

/**
 * @brief Drive the electromagnet from the planner's grip flag.
 * @param enabled Non-zero energises the magnet.
 * @note MX_GPIO_Init() leaves this pin as an EXTI input, so the first call
 *       reconfigures it as an output. Overrides the weak default.
 */
void CraneControl_SetMagnet(uint8_t enabled)
{
    static uint8_t configured;

    if (configured == 0U) {
        GPIO_InitTypeDef gpio_config;

        (void)memset(&gpio_config, 0, sizeof(gpio_config));
        gpio_config.Pin = CRANE_MAGNET_GPIO_PIN;
        gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_config.Pull = GPIO_NOPULL;
        gpio_config.Speed = GPIO_SPEED_FREQ_LOW;
        /* Release before switching the pin to an output, so enabling the driver
           cannot briefly energise the magnet. */
        HAL_GPIO_WritePin(CRANE_MAGNET_GPIO_PORT, CRANE_MAGNET_GPIO_PIN,
                          CRANE_MAGNET_ACTIVE_LEVEL == GPIO_PIN_SET
                              ? GPIO_PIN_RESET
                              : GPIO_PIN_SET);
        HAL_GPIO_Init(CRANE_MAGNET_GPIO_PORT, &gpio_config);
        configured = 1U;
    }
    HAL_GPIO_WritePin(CRANE_MAGNET_GPIO_PORT, CRANE_MAGNET_GPIO_PIN,
                      enabled != 0U
                          ? CRANE_MAGNET_ACTIVE_LEVEL
                          : (CRANE_MAGNET_ACTIVE_LEVEL == GPIO_PIN_SET
                                 ? GPIO_PIN_RESET
                                 : GPIO_PIN_SET));
}

#ifndef GIMBAL_CTRL_MOTION_H
#define GIMBAL_CTRL_MOTION_H

#include "main.h"

#include <stdint.h>

typedef enum {
    GIMBAL_MOTION_AXIS_YAW = 0,
    GIMBAL_MOTION_AXIS_PITCH,
} gimbal_motion_axis_t;

/** @brief Reset both internal motor-position records to software zero. */
void gimbal_motion_reset(void);

/** @brief Execute Pitch homing and clear both motors' hardware positions. */
HAL_StatusTypeDef gimbal_motion_home_pitch(void);

/** @brief Start an absolute software-angle move without waiting for completion. */
HAL_StatusTypeDef gimbal_motion_start_absolute(gimbal_motion_axis_t axis,
                                               int16_t target_angle_tenths,
                                               uint16_t speed_rpm,
                                               uint8_t acceleration,
                                               uint32_t *motion_time_ms);

/** @brief Move one axis by a relative software angle and wait for completion. */
HAL_StatusTypeDef gimbal_motion_move_relative(gimbal_motion_axis_t axis,
                                              int16_t delta_angle_tenths,
                                              uint16_t speed_rpm,
                                              uint8_t acceleration);

/** @brief Move one axis to an absolute software angle and wait for completion. */
HAL_StatusTypeDef gimbal_motion_move_absolute(gimbal_motion_axis_t axis,
                                              int16_t target_angle_tenths,
                                              uint16_t speed_rpm,
                                              uint8_t acceleration);

/** @brief Return one axis's current software angle in 0.1-degree units. */
int16_t gimbal_motion_get_angle(gimbal_motion_axis_t axis);

#endif
#ifndef GIMBAL_CTRL_TASK_H
#define GIMBAL_CTRL_TASK_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_ANGLE_MIN_TENTHS (-450)
#define GIMBAL_ANGLE_MAX_TENTHS 450
#define GIMBAL_ANGLE_TENTHS_PER_DEGREE 10
#define GIMBAL_VISION_STATUS_VALID 0x00U
#define GIMBAL_VISION_STATUS_LOST 0xFFU

#ifndef GIMBAL_MOTION_SPEED_RPM
#define GIMBAL_MOTION_SPEED_RPM 10U
#endif

#ifndef GIMBAL_MOTION_ACCELERATION
#define GIMBAL_MOTION_ACCELERATION 2U
#endif

#ifndef GIMBAL_HOMING_TORQUE_MA
#define GIMBAL_HOMING_TORQUE_MA 400U
#endif

#ifndef GIMBAL_HOMING_TORQUE_DURATION_MS
#define GIMBAL_HOMING_TORQUE_DURATION_MS 2000U
#endif

#ifndef GIMBAL_HOMING_REVERSE_ANGLE_TENTHS
#define GIMBAL_HOMING_REVERSE_ANGLE_TENTHS 1000
#endif

#ifndef GIMBAL_SCAN_START_DELAY_MS
#define GIMBAL_SCAN_START_DELAY_MS 2000U
#endif

#ifndef GIMBAL_SCAN_YAW_STEP_TENTHS
#define GIMBAL_SCAN_YAW_STEP_TENTHS 10
#endif

#ifndef GIMBAL_SCAN_PITCH_STEP_TENTHS
#define GIMBAL_SCAN_PITCH_STEP_TENTHS 50
#endif

#ifndef GIMBAL_VISION_STEP_MAX_TENTHS
#define GIMBAL_VISION_STEP_MAX_TENTHS 10
#endif

#ifndef GIMBAL_VISION_CONVERGENCE_PIXELS
#define GIMBAL_VISION_CONVERGENCE_PIXELS 50U
#endif

#ifndef GIMBAL_VISION_WAIT_TIMEOUT_MS
#define GIMBAL_VISION_WAIT_TIMEOUT_MS 1000U
#endif

#ifndef GIMBAL_VISION_POLL_TIMEOUT_MS
#define GIMBAL_VISION_POLL_TIMEOUT_MS 10U
#endif

#ifndef GIMBAL_MOTION_SETTLE_MS
#define GIMBAL_MOTION_SETTLE_MS 20U
#endif

typedef struct {
    int16_t yaw_angle_tenths;
    int16_t pitch_angle_tenths;
    bool initialized;
} gimbal_ctrl_state_t;

/**
 * @brief Initialize the gimbal and establish the software zero position.
 * @note Yaw remains still. Pitch applies 400 mA forward torque for 2000 ms,
 *       then moves 100 degrees in reverse and clears both hardware positions.
 */
HAL_StatusTypeDef gimbal_ctrl_initialize(void);

/**
 * @brief Scan from the current Pitch, then continue upward in 5-degree rows.
 * @retval HAL_OK A fresh, non-lost vision target was received.
 * @retval HAL_TIMEOUT Pitch reached 45 degrees without finding a target.
 */
HAL_StatusTypeDef gimbal_ctrl_scan(void);

/**
 * @brief Consume one fresh vision vector and perform at most one correction.
 * @note Each call moves either axis by no more than 1.0 degree.
 */
HAL_StatusTypeDef gimbal_ctrl_correct(void);

/**
 * @brief Submit one decoded MaixCAM vision result.
 * @param status 0x00 for a valid target or 0xFF for target loss.
 * @param x Signed horizontal pixel vector; positive moves the laser right.
 * @param y Signed vertical pixel vector; positive moves the laser upward.
 * @note Values with any other status are ignored.
 */
void gimbal_ctrl_vision_input(uint8_t status, int16_t x, int16_t y);

/** @brief Move Yaw by a relative angle in 0.1-degree units. */
HAL_StatusTypeDef gimbal_ctrl_yaw_relative(int16_t angle_tenths,
                                           uint16_t speed_rpm,
                                           uint8_t acceleration);

/** @brief Move Yaw to an absolute software angle in 0.1-degree units. */
HAL_StatusTypeDef gimbal_ctrl_yaw_absolute(int16_t angle_tenths,
                                           uint16_t speed_rpm,
                                           uint8_t acceleration);

/** @brief Move Pitch by a relative angle in 0.1-degree units. */
HAL_StatusTypeDef gimbal_ctrl_pitch_relative(int16_t angle_tenths,
                                             uint16_t speed_rpm,
                                             uint8_t acceleration);

/** @brief Move Pitch to an absolute software angle in 0.1-degree units. */
HAL_StatusTypeDef gimbal_ctrl_pitch_absolute(int16_t angle_tenths,
                                             uint16_t speed_rpm,
                                             uint8_t acceleration);

/** @brief Copy the current software angles and initialization state. */
void gimbal_ctrl_get_state(gimbal_ctrl_state_t *state);

/**
 * @brief Latest continuous-tracking state shared between FreeRTOS tasks.
 * @note HAL_OK means the latest vector was valid; HAL_ERROR means tracking is
 *       unavailable and the gimbal is scanning or initialization failed.
 */
extern volatile HAL_StatusTypeDef gimbal_ctrl_status;

/** @brief FreeRTOS gimbal task entry. */
void Gimbal_ctrl_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif

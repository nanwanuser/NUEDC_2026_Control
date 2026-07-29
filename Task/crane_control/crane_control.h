#ifndef CRANE_CONTROL_H
#define CRANE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "route_planning.h"

#include <stdint.h>

typedef enum {
    CRANE_CONTROL_OK = 0,
    CRANE_CONTROL_INVALID_ARGUMENT,
    CRANE_CONTROL_INVALID_CONFIG,
    CRANE_CONTROL_NOT_INITIALIZED,
    CRANE_CONTROL_OUT_OF_WORKSPACE,
    CRANE_CONTROL_SERVO_ERROR,
    CRANE_CONTROL_TRANSPORT_ERROR,
    CRANE_CONTROL_DRIVE_REJECTED,
} CraneControlStatus;

typedef struct {
    TrajectoryPose origin;
    float yaw_motor_revolutions_per_crane_revolution;
    float reach_zero_radius_mm;
    float reach_mm_per_motor_revolution;
    float lift_zero_angle_deg;
    float lift_mm_per_degree;
    float end_yaw_center_angle_deg;
    float end_yaw_zero_offset_deg;
    float min_boom_yaw_deg;
    float max_boom_yaw_deg;
    float min_radius_mm;
    float max_radius_mm;
    float min_z_mm;
    float max_z_mm;
    int8_t yaw_direction_sign;
    int8_t reach_direction_sign;
    int8_t lift_direction_sign;
    int8_t end_yaw_direction_sign;
    uint16_t yaw_speed_rpm;
    uint16_t reach_speed_rpm;
    uint32_t min_stepper_change_units;
    uint8_t stepper_acceleration;
    uint8_t expect_stepper_response;
} CraneControlConfig;

typedef struct {
    float boom_yaw_deg;
    float radius_mm;
    float z_mm;
    float lift_servo_angle_deg;
    float end_yaw_servo_angle_deg;
    uint8_t grip;
} CraneActuatorTarget;

typedef struct {
    CraneControlStatus status;
    CraneActuatorTarget target;
    uint32_t plan_id;
    TrajectoryPhase phase;
    TrajectoryState planner_state;
    uint8_t initialized;
} CraneControlState;

/**
 * @brief Fill a configuration with this crane's conservative default values.
 * @param config Destination configuration.
 */
void CraneControl_LoadDefaultConfig(CraneControlConfig *config);

/**
 * @brief Project hook for overriding origin, calibration, and motion limits.
 * @param config Default configuration to modify before controller startup.
 * @note Override this weak function in a project configuration source file.
 */
void CraneControl_CustomizeConfig(CraneControlConfig *config);

/**
 * @brief Initialize the crane controller and bind both actuator drivers.
 * @param config Runtime coordinate and transmission configuration; NULL uses
 *               CraneControl_LoadDefaultConfig().
 * @return Initialization result.
 */
CraneControlStatus CraneControl_Init(const CraneControlConfig *config);

/**
 * @brief Non-blocking task interface for one route-planner output sample.
 * @param output Planner output containing the Cartesian reference.
 * @return CRANE_CONTROL_OK when the sample was accepted for processing.
 */
CraneControlStatus CraneControl_SubmitPlannerOutput(
    const RoutePlanningOutput *output);

/**
 * @brief Process the newest accepted reference and update both servos.
 * @note Call from the crane control task at a fixed period.
 */
void CraneControl_Update(void);

/**
 * @brief Copy the latest controller state for diagnostics or decision logic.
 * @param state Destination state.
 */
void CraneControl_GetState(CraneControlState *state);

/**
 * @brief Hardware hook for the planner's grip flag.
 * @note Override this weak function after assigning the electromagnet GPIO.
 */
void CraneControl_SetMagnet(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif

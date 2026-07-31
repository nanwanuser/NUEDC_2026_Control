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
    CRANE_CONTROL_ARRIVAL_TIMEOUT,
} CraneControlStatus;

typedef enum {
    CRANE_LIFT_RAISED = 0,
    CRANE_LIFT_LOWERED = 1,
} CraneLiftPosition;

typedef struct {
    TrajectoryPose origin;
    float startup_boom_yaw_deg;
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
    uint8_t yaw_acceleration;
    uint8_t reach_acceleration;
    uint8_t expect_stepper_response;
    /* Non-zero lets startup datum both axes against their mechanical ends instead
       of trusting that the mechanism was parked before reset. Zero it if the
       mechanism must not be driven into its stops at all, in which case the
       operator owns the datum again. */
    uint8_t home_on_startup;
    /* Torque and duration of the startup push that finds both hard stops. Neither
       axis has a switch and neither is homed by the drive any more: startup simply
       holds this current towards each stop for this long and takes whatever the
       mechanism is resting against as zero. The current has to overcome friction
       and stay gentle enough to sit against the frame; the duration has to cover
       the worst case, which is a full stroke from the far end. */
    uint16_t home_torque_current_ma;
    uint16_t home_push_ms;
    /* Maximum time allowed after a trajectory endpoint for both drives to set
       their physical in-position flags. */
    uint32_t arrival_timeout_ms;
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
    uint8_t waypoint_index;
    CraneLiftPosition lift_position;
    uint8_t yaw_at_target;
    uint8_t reach_at_target;
    uint8_t axes_at_target;
    int32_t yaw_position_error_units;
    int32_t reach_position_error_units;
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

/** Immediately write a full-speed lift move to the fixed raised or lowered
 * endpoint configured by the servo driver. */
CraneControlStatus CraneControl_CommandLift(CraneLiftPosition position);

/** Queue an electromagnet command in the crane task. */
CraneControlStatus CraneControl_CommandGrip(uint8_t enabled);

/**
 * @brief Copy the active configuration, so callers can place their targets in
 *        this controller's coordinate frame instead of assuming one.
 * @param config Destination configuration.
 * @note Usable before CraneControl_Init(); it then reports the configuration
 *       that startup will apply.
 */
void CraneControl_GetConfig(CraneControlConfig *config);

/**
 * @brief Report the last commanded pose in the planner's Cartesian frame.
 * @param pose Destination pose.
 * @note Usable before CraneControl_Init(); it then reports the park pose the
 *       crane reaches at startup. Callers use this as the first waypoint of a
 *       plan so the run does not begin with a jump.
 */
void CraneControl_GetCurrentPose(TrajectoryPose *pose);

/**
 * @brief Convert a boom/reach/lift posture into a planner Cartesian pose.
 * @param boom_yaw_deg Boom angle relative to its own zero heading.
 * @param radius_mm Reach measured from the column.
 * @param z_mm Height in the planner frame.
 * @param pose Receives the pose, with the yaw a centred wrist holds there.
 * @note Lets callers name targets by reach and boom angle instead of hard-coding
 *       this controller's frame. Usable before CraneControl_Init().
 */
void CraneControl_GetPoseAt(float boom_yaw_deg,
                            float radius_mm,
                            float z_mm,
                            TrajectoryPose *pose);

/**
 * @brief Pick a yaw datum that keeps a whole waypoint set inside wrist travel.
 *
 * The wrist sits on the boom, so the world yaw it can hold is the boom heading
 * plus at most half the servo travel. A plan that fixes the tool's world yaw in
 * advance therefore leaves the wrist out of range for many pick/place pairs.
 * Only the yaw *difference* between pick and place turns the piece, so adding
 * one constant to every waypoint of a move places the piece just the same while
 * re-centring the wrist. This returns that constant.
 *
 * @param poses Waypoints of one move, all of which get the same offset.
 * @param count Number of waypoints, at least one.
 * @param bias_deg Receives the offset to add to every waypoint's yaw_deg.
 * @return CRANE_CONTROL_OK when the offset brings every waypoint inside wrist
 *         travel, CRANE_CONTROL_OUT_OF_WORKSPACE when no offset can (the
 *         best-effort offset is still written).
 * @note Usable before CraneControl_Init().
 */
CraneControlStatus CraneControl_ChooseYawBias(const TrajectoryPose *poses,
                                             uint8_t count,
                                             float *bias_deg);

/**
 * @brief Bridge two poses whose straight path would leave the reach band.
 *
 * The planner interpolates in Cartesian space, so a straight leg between two
 * reachable points can still cut inside the minimum reach: the chord of a wide
 * boom sweep passes nearer the column than either end. The crane then refuses a
 * reference part-way through a move that passed every waypoint check.
 *
 * This returns intermediate poses that bulge the leg outwards, holding a radius
 * just outside the wider end, so the whole path stays in the band.
 *
 * @param from Start pose of the leg.
 * @param to End pose of the leg.
 * @param poses Receives the intermediate poses, in order.
 * @param capacity Number of poses that fit; zero is allowed.
 * @param count Receives how many were written, zero when the straight leg is
 *              already safe or capacity ran out.
 * @return CRANE_CONTROL_OK when the resulting path is inside the workspace,
 *         CRANE_CONTROL_OUT_OF_WORKSPACE when it is not even with the poses
 *         added, which includes the case of too little capacity.
 * @note Usable before CraneControl_Init().
 */
CraneControlStatus CraneControl_PlanTransitPoses(const TrajectoryPose *from,
                                                const TrajectoryPose *to,
                                                TrajectoryPose *poses,
                                                uint8_t capacity,
                                                uint8_t *count);

/**
 * @brief Check one planner pose against the workspace without moving anything.
 * @param pose Cartesian pose in the planner frame.
 * @return CRANE_CONTROL_OK when the pose maps inside boom, reach, and wrist
 *         travel; CRANE_CONTROL_OUT_OF_WORKSPACE otherwise.
 * @note Lift motion is commanded separately with CraneControl_CommandLift().
 *       Usable before CraneControl_Init().
 */
CraneControlStatus CraneControl_CheckPose(const TrajectoryPose *pose);

/* How often CraneControl_Update() polls the two physical arrival flags. */
#define CRANE_TICK_PERIOD_MS 20U

/**
 * @brief Datum both stepper axes by pushing them against their mechanical ends.
 * @param push_ms How long to hold torque towards each stop; zero uses
 *                config.home_push_ms.
 * @return CRANE_CONTROL_OK once both axes have been pushed home and their end
 *         stops established as each axis's zero.
 * @note Does not use the drive's homing mode. Neither axis has a limit switch, and
 *       the drive's switchless homing was retreating to its own origin position and
 *       reporting states this code could not act on. Instead both axes are simply
 *       held in torque mode (config.home_torque_current_ma) towards their stop for
 *       config.home_push_ms, after which whatever they are resting against is
 *       taken as zero. Both axes are pushed at once, which is safe because the
 *       reach only ever draws inwards and the boom's stop is at a fixed angle.
 *       Blocks for the whole push, so it belongs in startup rather than the control
 *       loop. Does nothing and reports OK when config.home_on_startup is zero.
 */
CraneControlStatus CraneControl_Home(uint32_t push_ms);

/**
 * @brief Hardware hook for the planner's grip flag.
 * @note Override this weak function after assigning the electromagnet GPIO.
 */
void CraneControl_SetMagnet(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif

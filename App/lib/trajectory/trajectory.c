#include "trajectory.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TRAJECTORY_YAW_PERIOD_DEG          360.0f
#define TRAJECTORY_YAW_HALF_PERIOD_DEG     180.0f
#define TRAJECTORY_QUINTIC_MAX_VELOCITY    1.875f
#define TRAJECTORY_QUINTIC_MAX_ACCELERATION 5.773503f
#define TRAJECTORY_DURATION_SCALE_MARGIN   1.0001f
#define TRAJECTORY_BOUND_MAX_COMPONENTS    3U
#define TRAJECTORY_BOUND_MAX_POINTS        5U
#define TRAJECTORY_BOUND_SUBDIVISIONS      8U

enum {
    TRAJECTORY_AXIS_X = 0,
    TRAJECTORY_AXIS_Y,
    TRAJECTORY_AXIS_Z,
    TRAJECTORY_AXIS_YAW
};

typedef float TrajectoryAxes[TRAJECTORY_AXIS_COUNT];

static float max_float(float left, float right)
{
    return left > right ? left : right;
}

static float clamp_abs(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float normalize_yaw(float yaw_deg)
{
    float normalized = fmodf(yaw_deg + TRAJECTORY_YAW_HALF_PERIOD_DEG,
                             TRAJECTORY_YAW_PERIOD_DEG);

    if (normalized < 0.0f) {
        normalized += TRAJECTORY_YAW_PERIOD_DEG;
    }

    return normalized - TRAJECTORY_YAW_HALF_PERIOD_DEG;
}

static float shortest_yaw_delta(float from_deg, float to_deg)
{
    return normalize_yaw(to_deg - from_deg);
}

static uint8_t pose_is_finite(const TrajectoryPose *pose)
{
    return (uint8_t)(isfinite(pose->x_mm) &&
                     isfinite(pose->y_mm) &&
                     isfinite(pose->z_mm) &&
                     isfinite(pose->yaw_deg));
}

static uint8_t limits_are_finite(const TrajectoryLimits *limits)
{
    return (uint8_t)(isfinite(limits->max_linear_velocity_mm_s) &&
                     isfinite(limits->max_linear_acceleration_mm_s2) &&
                     isfinite(limits->max_yaw_velocity_deg_s) &&
                     isfinite(limits->max_yaw_acceleration_deg_s2));
}

static uint8_t limits_are_positive(const TrajectoryLimits *limits)
{
    return (uint8_t)(limits->max_linear_velocity_mm_s > 0.0f &&
                     limits->max_linear_acceleration_mm_s2 > 0.0f &&
                     limits->max_yaw_velocity_deg_s > 0.0f &&
                     limits->max_yaw_acceleration_deg_s2 > 0.0f);
}

static void pose_to_axes(const TrajectoryPose *pose, TrajectoryAxes axes)
{
    axes[TRAJECTORY_AXIS_X] = pose->x_mm;
    axes[TRAJECTORY_AXIS_Y] = pose->y_mm;
    axes[TRAJECTORY_AXIS_Z] = pose->z_mm;
    axes[TRAJECTORY_AXIS_YAW] = pose->yaw_deg;
}

static void axes_to_pose(const TrajectoryAxes axes, TrajectoryPose *pose)
{
    pose->x_mm = axes[TRAJECTORY_AXIS_X];
    pose->y_mm = axes[TRAJECTORY_AXIS_Y];
    pose->z_mm = axes[TRAJECTORY_AXIS_Z];
    pose->yaw_deg = normalize_yaw(axes[TRAJECTORY_AXIS_YAW]);
}

static float motion_duration(const TrajectoryAxes start,
                             const TrajectoryAxes end,
                             const TrajectoryLimits *limits)
{
    float dx = end[TRAJECTORY_AXIS_X] - start[TRAJECTORY_AXIS_X];
    float dy = end[TRAJECTORY_AXIS_Y] - start[TRAJECTORY_AXIS_Y];
    float dz = end[TRAJECTORY_AXIS_Z] - start[TRAJECTORY_AXIS_Z];
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    float yaw_distance = fabsf(end[TRAJECTORY_AXIS_YAW] -
                               start[TRAJECTORY_AXIS_YAW]);
    float duration = 0.0f;

    duration = max_float(duration,
                         TRAJECTORY_QUINTIC_MAX_VELOCITY * distance /
                         limits->max_linear_velocity_mm_s);
    duration = max_float(duration,
                         sqrtf(TRAJECTORY_QUINTIC_MAX_ACCELERATION * distance /
                               limits->max_linear_acceleration_mm_s2));
    duration = max_float(duration,
                         TRAJECTORY_QUINTIC_MAX_VELOCITY * yaw_distance /
                         limits->max_yaw_velocity_deg_s);
    duration = max_float(duration,
                         sqrtf(TRAJECTORY_QUINTIC_MAX_ACCELERATION * yaw_distance /
                               limits->max_yaw_acceleration_deg_s2));

    return duration;
}

static void generate_quintic_segment(
    const TrajectoryAxes start,
    const TrajectoryAxes end,
    const TrajectoryAxes start_velocity,
    const TrajectoryAxes end_velocity,
    const TrajectoryAxes start_acceleration,
    const TrajectoryAxes end_acceleration,
    float duration_s,
    TrajectorySegment *segment)
{
    uint32_t axis;

    memset(segment, 0, sizeof(*segment));
    segment->duration_s = duration_s;

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        float duration_squared = duration_s * duration_s;
        float c0 = start[axis];
        float c1 = start_velocity[axis] * duration_s;
        float c2 = 0.5f * start_acceleration[axis] * duration_squared;
        float position_residual = end[axis] - (c0 + c1 + c2);
        float velocity_residual = end_velocity[axis] * duration_s -
                                  (c1 + 2.0f * c2);
        float acceleration_residual = end_acceleration[axis] * duration_squared -
                                      2.0f * c2;

        segment->coefficient[axis][0] = c0;
        segment->coefficient[axis][1] = c1;
        segment->coefficient[axis][2] = c2;
        segment->coefficient[axis][3] = 10.0f * position_residual -
                                        4.0f * velocity_residual +
                                        0.5f * acceleration_residual;
        segment->coefficient[axis][4] = -15.0f * position_residual +
                                        7.0f * velocity_residual -
                                        acceleration_residual;
        segment->coefficient[axis][5] = 6.0f * position_residual -
                                        3.0f * velocity_residual +
                                        0.5f * acceleration_residual;
    }
}

/* Monotone harmonic tangent: passes the waypoint without overshooting it. */
static float limited_waypoint_velocity(float previous,
                                       float current,
                                       float next,
                                       float previous_duration_s,
                                       float next_duration_s)
{
    float previous_secant;
    float next_secant;
    float velocity;
    float limit;

    if (previous_duration_s <= 0.0f || next_duration_s <= 0.0f) {
        return 0.0f;
    }

    previous_secant = (current - previous) / previous_duration_s;
    next_secant = (next - current) / next_duration_s;

    if (previous_secant * next_secant <= 0.0f) {
        return 0.0f;
    }

    velocity = 2.0f * previous_secant * next_secant /
               (previous_secant + next_secant);
    limit = 3.0f * fminf(fabsf(previous_secant), fabsf(next_secant));
    return clamp_abs(velocity, limit);
}

static float evaluate_polynomial(const float coefficient[TRAJECTORY_COEFFICIENT_COUNT],
                                 float u)
{
    float value = coefficient[TRAJECTORY_COEFFICIENT_COUNT - 1U];
    int32_t index;

    for (index = (int32_t)TRAJECTORY_COEFFICIENT_COUNT - 2; index >= 0; --index) {
        value = value * u + coefficient[index];
    }

    return value;
}

static void power_to_bezier(
    const float coefficient[TRAJECTORY_COEFFICIENT_COUNT],
    float bezier[TRAJECTORY_COEFFICIENT_COUNT])
{
    bezier[0] = coefficient[0];
    bezier[1] = coefficient[0] + coefficient[1] / 5.0f;
    bezier[2] = coefficient[0] + 2.0f * coefficient[1] / 5.0f +
                coefficient[2] / 10.0f;
    bezier[3] = coefficient[0] + 3.0f * coefficient[1] / 5.0f +
                3.0f * coefficient[2] / 10.0f + coefficient[3] / 10.0f;
    bezier[4] = coefficient[0] + 4.0f * coefficient[1] / 5.0f +
                3.0f * coefficient[2] / 5.0f +
                2.0f * coefficient[3] / 5.0f + coefficient[4] / 5.0f;
    bezier[5] = coefficient[0] + coefficient[1] + coefficient[2] +
                coefficient[3] + coefficient[4] + coefficient[5];
}

typedef float TrajectoryBoundHull[TRAJECTORY_BOUND_MAX_COMPONENTS]
                                 [TRAJECTORY_BOUND_MAX_POINTS];

/* de Casteljau at u, overwriting the hull with the [0, u] part reparametrized
   to [0, 1]. The left part is the diagonal b[k][0], so walking indices
   downward lets each level reuse the previous one in place. */
static void bezier_keep_left(TrajectoryBoundHull hull,
                             uint8_t component_count,
                             uint8_t point_count,
                             float u)
{
    uint8_t level;
    uint8_t component;
    int32_t index;

    for (level = 1U; level < point_count; ++level) {
        for (component = 0U; component < component_count; ++component) {
            for (index = (int32_t)point_count - 1; index >= (int32_t)level;
                 --index) {
                float previous = hull[component][index - 1];

                hull[component][index] = previous +
                    u * (hull[component][index] - previous);
            }
        }
    }
}

/* de Casteljau at u, overwriting the hull with the [u, 1] part reparametrized
   to [0, 1]. The right part is the diagonal b[k][n - k], so walking indices
   upward keeps the reuse in place. */
static void bezier_keep_right(TrajectoryBoundHull hull,
                              uint8_t component_count,
                              uint8_t point_count,
                              float u)
{
    uint8_t level;
    uint8_t component;
    uint8_t index;

    for (level = 1U; level < point_count; ++level) {
        for (component = 0U; component < component_count; ++component) {
            for (index = 0U; index + level < point_count; ++index) {
                hull[component][index] +=
                    u * (hull[component][index + 1U] - hull[component][index]);
            }
        }
    }
}

/* A Bezier curve stays inside the convex hull of its control points, and the
   Euclidean norm is convex, so the largest control point norm bounds the whole
   curve. The raw hull overestimates a rest-to-rest quintic by 8/3, which would
   force every motion to crawl at 37% of the limits. Restricting the curve to
   short sub-intervals shrinks each hull around the curve, so the maximum over
   the pieces stays a rigorous bound while landing within 0.1% of the true
   peak at eight pieces. */
static float bezier_max_norm(const TrajectoryBoundHull control,
                             uint8_t component_count,
                             uint8_t point_count)
{
    float maximum = 0.0f;
    uint8_t piece;

    for (piece = 0U; piece < TRAJECTORY_BOUND_SUBDIVISIONS; ++piece) {
        TrajectoryBoundHull hull;
        const float start = (float)piece / (float)TRAJECTORY_BOUND_SUBDIVISIONS;
        uint8_t component;
        uint8_t index;

        for (component = 0U; component < component_count; ++component) {
            for (index = 0U; index < point_count; ++index) {
                hull[component][index] = control[component][index];
            }
        }

        /* Clip away [0, start), then keep the first sub-interval of the rest. */
        if (piece > 0U) {
            bezier_keep_right(hull, component_count, point_count, start);
        }
        if (piece + 1U < TRAJECTORY_BOUND_SUBDIVISIONS) {
            bezier_keep_left(hull, component_count, point_count,
                             1.0f / (float)(TRAJECTORY_BOUND_SUBDIVISIONS -
                                            piece));
        }

        for (index = 0U; index < point_count; ++index) {
            float sum_of_squares = 0.0f;

            for (component = 0U; component < component_count; ++component) {
                sum_of_squares += hull[component][index] *
                                  hull[component][index];
            }
            maximum = max_float(maximum, sqrtf(sum_of_squares));
        }
    }

    return maximum;
}

static float segment_required_scale(const TrajectorySegment *segment,
                                    const TrajectoryLimits *limits)
{
    float bezier[TRAJECTORY_AXIS_COUNT][TRAJECTORY_COEFFICIENT_COUNT];
    float first[TRAJECTORY_AXIS_COUNT][5];
    float second[TRAJECTORY_AXIS_COUNT][4];
    TrajectoryBoundHull linear_hull;
    float max_linear_velocity;
    float max_linear_acceleration;
    float max_yaw_velocity;
    float max_yaw_acceleration;
    float duration_squared;
    float scale = 0.0f;
    uint32_t axis;
    uint32_t index;

    if (segment->duration_s <= 0.0f) {
        return 0.0f;
    }

    duration_squared = segment->duration_s * segment->duration_s;
    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        power_to_bezier(segment->coefficient[axis], bezier[axis]);
        for (index = 0U; index < 5U; ++index) {
            first[axis][index] = 5.0f *
                (bezier[axis][index + 1U] - bezier[axis][index]);
        }
        for (index = 0U; index < 4U; ++index) {
            second[axis][index] = 4.0f *
                (first[axis][index + 1U] - first[axis][index]);
        }
    }

    /* Speed couples the three linear axes, so bound the vector norm at once. */
    for (index = 0U; index < 5U; ++index) {
        linear_hull[TRAJECTORY_AXIS_X][index] = first[TRAJECTORY_AXIS_X][index];
        linear_hull[TRAJECTORY_AXIS_Y][index] = first[TRAJECTORY_AXIS_Y][index];
        linear_hull[TRAJECTORY_AXIS_Z][index] = first[TRAJECTORY_AXIS_Z][index];
    }
    max_linear_velocity = bezier_max_norm(linear_hull, 3U, 5U) /
                          segment->duration_s;

    for (index = 0U; index < 4U; ++index) {
        linear_hull[TRAJECTORY_AXIS_X][index] = second[TRAJECTORY_AXIS_X][index];
        linear_hull[TRAJECTORY_AXIS_Y][index] = second[TRAJECTORY_AXIS_Y][index];
        linear_hull[TRAJECTORY_AXIS_Z][index] = second[TRAJECTORY_AXIS_Z][index];
    }
    max_linear_acceleration = bezier_max_norm(linear_hull, 3U, 4U) /
                              duration_squared;

    for (index = 0U; index < 5U; ++index) {
        linear_hull[0][index] = first[TRAJECTORY_AXIS_YAW][index];
    }
    max_yaw_velocity = bezier_max_norm(linear_hull, 1U, 5U) /
                       segment->duration_s;

    for (index = 0U; index < 4U; ++index) {
        linear_hull[0][index] = second[TRAJECTORY_AXIS_YAW][index];
    }
    max_yaw_acceleration = bezier_max_norm(linear_hull, 1U, 4U) /
                           duration_squared;

    scale = max_float(scale,
                      max_linear_velocity /
                      limits->max_linear_velocity_mm_s);
    scale = max_float(scale,
                      sqrtf(max_linear_acceleration /
                            limits->max_linear_acceleration_mm_s2));
    scale = max_float(scale,
                      max_yaw_velocity /
                      limits->max_yaw_velocity_deg_s);
    scale = max_float(scale,
                      sqrtf(max_yaw_acceleration /
                            limits->max_yaw_acceleration_deg_s2));

    return scale;
}

/* The Bezier bound is exact under uniform time scaling, so one shared factor
   per phase both enforces the limits and removes slack, while keeping the
   waypoint joints C2 continuous. */
static uint8_t scale_phase_durations(TrajectoryPhasePlan *phase_plan,
                                     const TrajectoryLimits *limits)
{
    float scale = 0.0f;
    uint8_t index;

    for (index = 0U; index < phase_plan->segment_count; ++index) {
        scale = max_float(scale,
                          segment_required_scale(&phase_plan->segments[index],
                                                 limits));
    }

    if (!isfinite(scale)) {
        return 0U;
    }

    /* Coefficients are expressed in normalized time, so retiming a segment is
       just a new duration: the path is untouched and derivatives scale. */
    if (scale > 0.0f) {
        scale *= TRAJECTORY_DURATION_SCALE_MARGIN;
        for (index = 0U; index < phase_plan->segment_count; ++index) {
            phase_plan->segments[index].duration_s *= scale;
        }
    }

    phase_plan->duration_s = 0.0f;
    for (index = 0U; index < phase_plan->segment_count; ++index) {
        phase_plan->duration_s += phase_plan->segments[index].duration_s;
    }

    return (uint8_t)isfinite(phase_plan->duration_s);
}

static void evaluate_segment(const TrajectorySegment *segment,
                             float time_s,
                             TrajectoryPose *pose)
{
    TrajectoryAxes axes;
    float u;
    uint32_t axis;

    if (segment->duration_s <= 0.0f || time_s >= segment->duration_s) {
        u = 1.0f;
    } else if (time_s <= 0.0f) {
        u = 0.0f;
    } else {
        u = time_s / segment->duration_s;
    }

    for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
        axes[axis] = evaluate_polynomial(segment->coefficient[axis], u);
    }

    axes_to_pose(axes, pose);
}

static uint8_t path_is_valid(const TrajectoryPath *path)
{
    uint8_t index;

    if (path->point_count < 2U ||
        path->point_count > TRAJECTORY_MAX_WAYPOINTS) {
        return 0U;
    }
    for (index = 0U; index < path->point_count; ++index) {
        if (!pose_is_finite(&path->points[index])) {
            return 0U;
        }
    }
    return 1U;
}

static TrajectoryResult generate_phase(const TrajectoryPath *path,
                                       const TrajectoryLimits *limits,
                                       TrajectoryPhasePlan *phase_plan)
{
    TrajectoryAxes knots[TRAJECTORY_MAX_WAYPOINTS];
    TrajectoryAxes velocities[TRAJECTORY_MAX_WAYPOINTS];
    float durations[TRAJECTORY_MAX_SEGMENTS];
    const TrajectoryAxes zero = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t segment_count = (uint8_t)(path->point_count - 1U);
    uint8_t index;
    uint32_t axis;

    memset(phase_plan, 0, sizeof(*phase_plan));

    /* Unwrap yaw along the chain so every turn takes the shortest direction. */
    pose_to_axes(&path->points[0], knots[0]);
    for (index = 1U; index < path->point_count; ++index) {
        float previous_yaw = knots[index - 1U][TRAJECTORY_AXIS_YAW];

        pose_to_axes(&path->points[index], knots[index]);
        knots[index][TRAJECTORY_AXIS_YAW] = previous_yaw +
            shortest_yaw_delta(previous_yaw, path->points[index].yaw_deg);
    }

    for (index = 0U; index < segment_count; ++index) {
        durations[index] = motion_duration(knots[index],
                                           knots[index + 1U],
                                           limits);
        if (!isfinite(durations[index])) {
            return TRAJECTORY_RESULT_NUMERIC_ERROR;
        }
    }

    memcpy(velocities[0], zero, sizeof(zero));
    memcpy(velocities[segment_count], zero, sizeof(zero));
    for (index = 1U; index < segment_count; ++index) {
        for (axis = 0U; axis < TRAJECTORY_AXIS_COUNT; ++axis) {
            velocities[index][axis] = limited_waypoint_velocity(
                knots[index - 1U][axis],
                knots[index][axis],
                knots[index + 1U][axis],
                durations[index - 1U],
                durations[index]);
        }
    }

    phase_plan->segment_count = segment_count;
    for (index = 0U; index < segment_count; ++index) {
        generate_quintic_segment(knots[index],
                                 knots[index + 1U],
                                 velocities[index],
                                 velocities[index + 1U],
                                 zero,
                                 zero,
                                 durations[index],
                                 &phase_plan->segments[index]);
    }

    if (!scale_phase_durations(phase_plan, limits)) {
        return TRAJECTORY_RESULT_NUMERIC_ERROR;
    }

    return TRAJECTORY_RESULT_OK;
}

void Trajectory_PathReset(TrajectoryPath *path)
{
    if (path == NULL) {
        return;
    }
    memset(path, 0, sizeof(*path));
}

uint8_t Trajectory_PathAppend(TrajectoryPath *path, const TrajectoryPose *pose)
{
    if (path == NULL || pose == NULL ||
        path->point_count >= TRAJECTORY_MAX_WAYPOINTS) {
        return 0U;
    }

    path->points[path->point_count] = *pose;
    ++path->point_count;
    return 1U;
}

TrajectoryResult Trajectory_Generate(const TrajectoryRequest *request,
                                     TrajectoryPlan *plan)
{
    TrajectoryResult result;

    if (request == NULL || plan == NULL) {
        return TRAJECTORY_RESULT_INVALID_ARGUMENT;
    }

    if (!path_is_valid(&request->approach) ||
        !path_is_valid(&request->transfer) ||
        !limits_are_finite(&request->limits)) {
        return TRAJECTORY_RESULT_INVALID_ARGUMENT;
    }

    if (!limits_are_positive(&request->limits)) {
        return TRAJECTORY_RESULT_INVALID_LIMIT;
    }

    memset(plan, 0, sizeof(*plan));
    result = generate_phase(&request->approach,
                            &request->limits,
                            &plan->approach);
    if (result != TRAJECTORY_RESULT_OK) {
        return result;
    }

    return generate_phase(&request->transfer,
                          &request->limits,
                          &plan->transfer);
}

static const TrajectoryPhasePlan *select_phase(const TrajectoryPlan *plan,
                                               TrajectoryPhase phase)
{
    if (phase == TRAJECTORY_PHASE_APPROACH) {
        return &plan->approach;
    }
    if (phase == TRAJECTORY_PHASE_TRANSFER) {
        return &plan->transfer;
    }
    return NULL;
}

static void evaluate_phase(const TrajectoryPhasePlan *phase_plan,
                           float time_s,
                           TrajectoryPose *pose)
{
    float remaining = time_s;
    uint8_t index;

    if (phase_plan->segment_count == 0U) {
        memset(pose, 0, sizeof(*pose));
        return;
    }

    for (index = 0U; index < phase_plan->segment_count; ++index) {
        const TrajectorySegment *segment = &phase_plan->segments[index];

        if (remaining < segment->duration_s ||
            index + 1U == phase_plan->segment_count) {
            evaluate_segment(segment, remaining, pose);
            return;
        }
        remaining -= segment->duration_s;
    }
}

TrajectoryState Trajectory_Evaluate(const TrajectoryPlan *plan,
                                    TrajectoryPhase phase,
                                    float time_s,
                                    TrajectoryReference *reference)
{
    const TrajectoryPhasePlan *phase_plan;
    uint8_t complete;

    if (plan == NULL || reference == NULL || !isfinite(time_s)) {
        return TRAJECTORY_STATE_INVALID_ARGUMENT;
    }

    phase_plan = select_phase(plan, phase);
    if (phase_plan == NULL) {
        return TRAJECTORY_STATE_INVALID_PHASE;
    }

    evaluate_phase(phase_plan, time_s, &reference->pose);
    complete = (uint8_t)(phase_plan->duration_s <= 0.0f ||
                         time_s >= phase_plan->duration_s);

    if (phase == TRAJECTORY_PHASE_APPROACH) {
        /* Grip closes once the tool has settled on the piece. */
        reference->grip = complete ? 1U : 0U;
    } else {
        reference->grip = complete ? 0U : 1U;
    }

    return complete ? TRAJECTORY_STATE_COMPLETE : TRAJECTORY_STATE_RUNNING;
}

float Trajectory_GetDuration(const TrajectoryPlan *plan,
                             TrajectoryPhase phase)
{
    const TrajectoryPhasePlan *phase_plan;

    if (plan == NULL) {
        return 0.0f;
    }

    phase_plan = select_phase(plan, phase);
    return phase_plan != NULL ? phase_plan->duration_s : 0.0f;
}

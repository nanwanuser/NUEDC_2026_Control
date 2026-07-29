#include "decision.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DECISION_PI                    3.14159265358979323846f
#define DECISION_GEOMETRY_EPSILON_MM   0.05f
#define DECISION_MIN_POLYGON_AREA_MM2  1.0f
#define DECISION_GRASP_GRID_DIVISIONS  12U
#define DECISION_WAYPOINT_EPSILON_MM   0.1f
#define DECISION_WAYPOINT_EPSILON_DEG  0.1f

typedef struct {
    float cosine;
    float sine;
    float tx;
    float ty;
} RigidTransform;

typedef struct {
    DecisionPiece pieces[DECISION_MAX_PIECES];
    const DecisionConfig *config;
    uint8_t piece_count;
    uint32_t nodes;
    uint8_t limit_reached;
    uint8_t solution_found;
    float total_area;
    float best_score;
    float best_rect_angle_rad;
    float best_min_x;
    float best_max_x;
    float best_min_y;
    float best_max_y;
    RigidTransform best_transforms[DECISION_MAX_PIECES];
} GeneralSearch;

static float square(float value)
{
    return value * value;
}

static DecisionPoint point_subtract(DecisionPoint left, DecisionPoint right)
{
    DecisionPoint result = {left.x_mm - right.x_mm,
                            left.y_mm - right.y_mm};
    return result;
}

static float point_cross(DecisionPoint left, DecisionPoint right)
{
    return left.x_mm * right.y_mm - left.y_mm * right.x_mm;
}

static float point_length(DecisionPoint point)
{
    return sqrtf(square(point.x_mm) + square(point.y_mm));
}

static DecisionPoint transform_point(const RigidTransform *transform,
                                     DecisionPoint point)
{
    DecisionPoint result;

    result.x_mm = transform->cosine * point.x_mm -
                  transform->sine * point.y_mm + transform->tx;
    result.y_mm = transform->sine * point.x_mm +
                  transform->cosine * point.y_mm + transform->ty;
    return result;
}

static float normalize_degrees(float angle_deg)
{
    float result = fmodf(angle_deg + 180.0f, 360.0f);

    if (result < 0.0f) {
        result += 360.0f;
    }
    return result - 180.0f;
}

static float polygon_signed_area(const DecisionPoint *vertices,
                                 uint8_t vertex_count)
{
    float twice_area = 0.0f;
    uint8_t index;

    for (index = 0U; index < vertex_count; ++index) {
        uint8_t next = (uint8_t)((index + 1U) % vertex_count);
        twice_area += point_cross(vertices[index], vertices[next]);
    }
    return 0.5f * twice_area;
}

static void reverse_vertices(DecisionPoint *vertices, uint8_t vertex_count)
{
    uint8_t left = 0U;
    uint8_t right = (uint8_t)(vertex_count - 1U);

    while (left < right) {
        DecisionPoint temporary = vertices[left];
        vertices[left] = vertices[right];
        vertices[right] = temporary;
        ++left;
        --right;
    }
}

static uint8_t point_is_finite(DecisionPoint point)
{
    return (uint8_t)(isfinite(point.x_mm) && isfinite(point.y_mm));
}

static uint8_t config_is_valid(const DecisionConfig *config)
{
    return (uint8_t)(config != NULL &&
                     point_is_finite(config->target_center) &&
                     isfinite(config->pick_z_mm) &&
                     isfinite(config->transit_z_mm) &&
                     isfinite(config->place_z_mm) &&
                     config->transit_z_mm > config->pick_z_mm &&
                     config->transit_z_mm > config->place_z_mm &&
                     config->edge_length_tolerance_mm >= 0.0f &&
                     config->boundary_tolerance_mm >= 0.0f &&
                     config->max_fill_error_ratio >= 0.0f &&
                     config->min_short_side_mm > 0.0f &&
                     config->max_short_side_mm >= config->min_short_side_mm &&
                     config->min_long_side_mm >= config->min_short_side_mm &&
                     config->max_long_side_mm >= config->min_long_side_mm &&
                     config->max_search_nodes > 0U);
}

static uint8_t normalize_frame(const DecisionVisionFrame *frame,
                               DecisionPiece pieces[DECISION_MAX_PIECES])
{
    uint8_t piece_index;

    if (frame == NULL || frame->piece_count == 0U ||
        frame->piece_count > DECISION_MAX_PIECES) {
        return 0U;
    }

    for (piece_index = 0U; piece_index < frame->piece_count; ++piece_index) {
        const DecisionPiece *source = &frame->pieces[piece_index];
        DecisionPiece *destination = &pieces[piece_index];
        float area;
        uint8_t vertex_index;
        uint8_t previous_index;

        if (source->vertex_count < 3U ||
            source->vertex_count > DECISION_MAX_VERTICES ||
            !point_is_finite(source->center)) {
            return 0U;
        }

        for (previous_index = 0U; previous_index < piece_index; ++previous_index) {
            if (pieces[previous_index].id == source->id) {
                return 0U;
            }
        }

        *destination = *source;
        for (vertex_index = 0U;
             vertex_index < destination->vertex_count;
             ++vertex_index) {
            if (!point_is_finite(destination->vertices[vertex_index])) {
                return 0U;
            }
        }

        area = polygon_signed_area(destination->vertices,
                                   destination->vertex_count);
        if (!isfinite(area) || fabsf(area) < DECISION_MIN_POLYGON_AREA_MM2) {
            return 0U;
        }
        if (area < 0.0f) {
            reverse_vertices(destination->vertices,
                             destination->vertex_count);
        }
    }
    return 1U;
}

static float distance_to_segment(DecisionPoint point,
                                 DecisionPoint start,
                                 DecisionPoint end)
{
    DecisionPoint edge = point_subtract(end, start);
    DecisionPoint offset = point_subtract(point, start);
    float denominator = square(edge.x_mm) + square(edge.y_mm);
    float parameter;
    DecisionPoint closest;

    if (denominator <= square(DECISION_GEOMETRY_EPSILON_MM)) {
        return point_length(offset);
    }

    parameter = (offset.x_mm * edge.x_mm + offset.y_mm * edge.y_mm) /
                denominator;
    if (parameter < 0.0f) parameter = 0.0f;
    if (parameter > 1.0f) parameter = 1.0f;

    closest.x_mm = start.x_mm + parameter * edge.x_mm;
    closest.y_mm = start.y_mm + parameter * edge.y_mm;
    return point_length(point_subtract(point, closest));
}

/* Returns -1 outside, 0 on the boundary and 1 inside. */
static int32_t point_polygon_location(DecisionPoint point,
                                      const DecisionPoint *vertices,
                                      uint8_t vertex_count)
{
    uint8_t inside = 0U;
    uint8_t index;

    for (index = 0U; index < vertex_count; ++index) {
        uint8_t next = (uint8_t)((index + 1U) % vertex_count);
        DecisionPoint start = vertices[index];
        DecisionPoint end = vertices[next];

        if (distance_to_segment(point, start, end) <=
            DECISION_GEOMETRY_EPSILON_MM) {
            return 0;
        }

        if ((start.y_mm > point.y_mm) != (end.y_mm > point.y_mm)) {
            float crossing_x = start.x_mm +
                (point.y_mm - start.y_mm) *
                (end.x_mm - start.x_mm) /
                (end.y_mm - start.y_mm);
            if (crossing_x > point.x_mm) {
                inside = (uint8_t)!inside;
            }
        }
    }
    return inside != 0U ? 1 : -1;
}

static float minimum_edge_distance(DecisionPoint point,
                                   const DecisionPoint *vertices,
                                   uint8_t vertex_count)
{
    float minimum = FLT_MAX;
    uint8_t index;

    for (index = 0U; index < vertex_count; ++index) {
        uint8_t next = (uint8_t)((index + 1U) % vertex_count);
        float distance = distance_to_segment(point,
                                             vertices[index],
                                             vertices[next]);
        if (distance < minimum) {
            minimum = distance;
        }
    }
    return minimum;
}

static DecisionPoint find_grasp_point(const DecisionPiece *piece)
{
    DecisionPoint best = piece->vertices[0];
    float best_clearance = -1.0f;
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    uint32_t x_index;
    uint32_t y_index;
    uint8_t vertex_index;

    if (point_polygon_location(piece->center,
                               piece->vertices,
                               piece->vertex_count) >= 0) {
        best = piece->center;
        best_clearance = minimum_edge_distance(best,
                                               piece->vertices,
                                               piece->vertex_count);
    }

    for (vertex_index = 0U; vertex_index < piece->vertex_count; ++vertex_index) {
        DecisionPoint vertex = piece->vertices[vertex_index];
        if (vertex.x_mm < min_x) min_x = vertex.x_mm;
        if (vertex.x_mm > max_x) max_x = vertex.x_mm;
        if (vertex.y_mm < min_y) min_y = vertex.y_mm;
        if (vertex.y_mm > max_y) max_y = vertex.y_mm;
    }

    for (x_index = 1U; x_index < DECISION_GRASP_GRID_DIVISIONS; ++x_index) {
        for (y_index = 1U; y_index < DECISION_GRASP_GRID_DIVISIONS; ++y_index) {
            DecisionPoint candidate;
            float clearance;

            candidate.x_mm = min_x + (max_x - min_x) *
                (float)x_index / (float)DECISION_GRASP_GRID_DIVISIONS;
            candidate.y_mm = min_y + (max_y - min_y) *
                (float)y_index / (float)DECISION_GRASP_GRID_DIVISIONS;

            if (point_polygon_location(candidate,
                                       piece->vertices,
                                       piece->vertex_count) < 0) {
                continue;
            }

            clearance = minimum_edge_distance(candidate,
                                               piece->vertices,
                                               piece->vertex_count);
            if (clearance > best_clearance) {
                best = candidate;
                best_clearance = clearance;
            }
        }
    }
    return best;
}

static uint8_t fit_rigid_transform(const DecisionPoint *source,
                                   const DecisionPoint *target,
                                   uint8_t count,
                                   RigidTransform *transform,
                                   float *root_mean_square_error)
{
    DecisionPoint source_center = {0.0f, 0.0f};
    DecisionPoint target_center = {0.0f, 0.0f};
    float cosine_sum = 0.0f;
    float sine_sum = 0.0f;
    float norm;
    float squared_error = 0.0f;
    uint8_t index;

    for (index = 0U; index < count; ++index) {
        source_center.x_mm += source[index].x_mm;
        source_center.y_mm += source[index].y_mm;
        target_center.x_mm += target[index].x_mm;
        target_center.y_mm += target[index].y_mm;
    }
    source_center.x_mm /= (float)count;
    source_center.y_mm /= (float)count;
    target_center.x_mm /= (float)count;
    target_center.y_mm /= (float)count;

    for (index = 0U; index < count; ++index) {
        DecisionPoint source_offset = point_subtract(source[index], source_center);
        DecisionPoint target_offset = point_subtract(target[index], target_center);
        cosine_sum += source_offset.x_mm * target_offset.x_mm +
                      source_offset.y_mm * target_offset.y_mm;
        sine_sum += source_offset.x_mm * target_offset.y_mm -
                    source_offset.y_mm * target_offset.x_mm;
    }

    norm = sqrtf(square(cosine_sum) + square(sine_sum));
    if (norm <= DECISION_GEOMETRY_EPSILON_MM) {
        return 0U;
    }

    transform->cosine = cosine_sum / norm;
    transform->sine = sine_sum / norm;
    transform->tx = target_center.x_mm -
        (transform->cosine * source_center.x_mm -
         transform->sine * source_center.y_mm);
    transform->ty = target_center.y_mm -
        (transform->sine * source_center.x_mm +
         transform->cosine * source_center.y_mm);

    for (index = 0U; index < count; ++index) {
        DecisionPoint mapped = transform_point(transform, source[index]);
        squared_error += square(mapped.x_mm - target[index].x_mm) +
                         square(mapped.y_mm - target[index].y_mm);
    }

    *root_mean_square_error = sqrtf(squared_error / (float)count);
    return (uint8_t)isfinite(*root_mean_square_error);
}

static uint8_t find_best_fixed_transform(const DecisionPiece *piece,
                                         const DecisionFixedPiece *target,
                                         RigidTransform *best_transform,
                                         float *best_error)
{
    DecisionPoint correspondence[DECISION_MAX_VERTICES];
    uint8_t direction_index;
    uint8_t shift;

    *best_error = FLT_MAX;
    for (direction_index = 0U; direction_index < 2U; ++direction_index) {
        int32_t direction = direction_index == 0U ? 1 : -1;
        for (shift = 0U; shift < piece->vertex_count; ++shift) {
            RigidTransform candidate;
            float error;
            uint8_t vertex_index;

            for (vertex_index = 0U;
                 vertex_index < piece->vertex_count;
                 ++vertex_index) {
                int32_t target_index = (int32_t)shift +
                                       direction * (int32_t)vertex_index;
                while (target_index < 0) {
                    target_index += piece->vertex_count;
                }
                target_index %= piece->vertex_count;
                correspondence[vertex_index] =
                    target->target_vertices[target_index];
            }

            if (fit_rigid_transform(piece->vertices,
                                    correspondence,
                                    piece->vertex_count,
                                    &candidate,
                                    &error) != 0U &&
                error < *best_error) {
                *best_error = error;
                *best_transform = candidate;
            }
        }
    }
    return (uint8_t)(*best_error < FLT_MAX);
}

static const DecisionFixedPiece *find_fixed_piece(
    const DecisionFixedLayout *layout,
    uint8_t id)
{
    uint8_t index;

    for (index = 0U; index < layout->piece_count; ++index) {
        if (layout->pieces[index].id == id) {
            return &layout->pieces[index];
        }
    }
    return NULL;
}

static void fill_move(const DecisionPiece *piece,
                      DecisionPoint grasp,
                      DecisionPoint place,
                      float rotation_deg,
                      const DecisionConfig *config,
                      DecisionMove *move)
{
    const float target_yaw_deg = normalize_degrees(rotation_deg);

    move->piece_id = piece->id;
    move->pick.x_mm = grasp.x_mm;
    move->pick.y_mm = grasp.y_mm;
    move->pick.z_mm = config->pick_z_mm;
    move->pick.yaw_deg = 0.0f;

    /* Lift straight up before travelling, and rotate the piece while high. */
    move->pick_above.x_mm = grasp.x_mm;
    move->pick_above.y_mm = grasp.y_mm;
    move->pick_above.z_mm = config->transit_z_mm;
    move->pick_above.yaw_deg = 0.0f;

    move->place_above.x_mm = place.x_mm;
    move->place_above.y_mm = place.y_mm;
    move->place_above.z_mm = config->transit_z_mm;
    move->place_above.yaw_deg = target_yaw_deg;

    move->place.x_mm = place.x_mm;
    move->place.y_mm = place.y_mm;
    move->place.z_mm = config->place_z_mm;
    move->place.yaw_deg = target_yaw_deg;
}

static float orientation(DecisionPoint start,
                         DecisionPoint end,
                         DecisionPoint point)
{
    return point_cross(point_subtract(end, start),
                       point_subtract(point, start));
}

static uint8_t segments_properly_intersect(DecisionPoint a0,
                                           DecisionPoint a1,
                                           DecisionPoint b0,
                                           DecisionPoint b1)
{
    float ab0 = orientation(a0, a1, b0);
    float ab1 = orientation(a0, a1, b1);
    float ba0 = orientation(b0, b1, a0);
    float ba1 = orientation(b0, b1, a1);
    float epsilon = DECISION_GEOMETRY_EPSILON_MM;

    return (uint8_t)(((ab0 > epsilon && ab1 < -epsilon) ||
                      (ab0 < -epsilon && ab1 > epsilon)) &&
                     ((ba0 > epsilon && ba1 < -epsilon) ||
                      (ba0 < -epsilon && ba1 > epsilon)));
}

static void transformed_vertices(const DecisionPiece *piece,
                                 const RigidTransform *transform,
                                 DecisionPoint *vertices)
{
    uint8_t index;

    for (index = 0U; index < piece->vertex_count; ++index) {
        vertices[index] = transform_point(transform, piece->vertices[index]);
    }
}

static DecisionPoint average_vertices(const DecisionPoint *vertices,
                                      uint8_t count)
{
    DecisionPoint average = {0.0f, 0.0f};
    uint8_t index;

    for (index = 0U; index < count; ++index) {
        average.x_mm += vertices[index].x_mm;
        average.y_mm += vertices[index].y_mm;
    }
    average.x_mm /= (float)count;
    average.y_mm /= (float)count;
    return average;
}

static uint8_t polygons_overlap(const DecisionPiece *left_piece,
                                const RigidTransform *left_transform,
                                const DecisionPiece *right_piece,
                                const RigidTransform *right_transform)
{
    DecisionPoint left[DECISION_MAX_VERTICES];
    DecisionPoint right[DECISION_MAX_VERTICES];
    uint8_t left_index;
    uint8_t right_index;

    transformed_vertices(left_piece, left_transform, left);
    transformed_vertices(right_piece, right_transform, right);

    for (left_index = 0U; left_index < left_piece->vertex_count; ++left_index) {
        uint8_t left_next =
            (uint8_t)((left_index + 1U) % left_piece->vertex_count);
        for (right_index = 0U;
             right_index < right_piece->vertex_count;
             ++right_index) {
            uint8_t right_next =
                (uint8_t)((right_index + 1U) % right_piece->vertex_count);
            if (segments_properly_intersect(left[left_index],
                                            left[left_next],
                                            right[right_index],
                                            right[right_next]) != 0U) {
                return 1U;
            }
        }
    }

    for (left_index = 0U; left_index < left_piece->vertex_count; ++left_index) {
        uint8_t left_next =
            (uint8_t)((left_index + 1U) % left_piece->vertex_count);
        DecisionPoint midpoint = {
            0.5f * (left[left_index].x_mm + left[left_next].x_mm),
            0.5f * (left[left_index].y_mm + left[left_next].y_mm)
        };

        if (point_polygon_location(left[left_index],
                                   right,
                                   right_piece->vertex_count) > 0 ||
            point_polygon_location(midpoint,
                                   right,
                                   right_piece->vertex_count) > 0) {
            return 1U;
        }
    }
    for (right_index = 0U; right_index < right_piece->vertex_count; ++right_index) {
        uint8_t right_next =
            (uint8_t)((right_index + 1U) % right_piece->vertex_count);
        DecisionPoint midpoint = {
            0.5f * (right[right_index].x_mm + right[right_next].x_mm),
            0.5f * (right[right_index].y_mm + right[right_next].y_mm)
        };

        if (point_polygon_location(right[right_index],
                                   left,
                                   left_piece->vertex_count) > 0 ||
            point_polygon_location(midpoint,
                                   left,
                                   left_piece->vertex_count) > 0) {
            return 1U;
        }
    }

    if (point_polygon_location(average_vertices(left,
                                                left_piece->vertex_count),
                               right,
                               right_piece->vertex_count) > 0 ||
        point_polygon_location(average_vertices(right,
                                                right_piece->vertex_count),
                               left,
                               left_piece->vertex_count) > 0) {
        return 1U;
    }
    return 0U;
}

static uint8_t align_piece_edges(const DecisionPiece *placed_piece,
                                 const RigidTransform *placed_transform,
                                 uint8_t placed_edge,
                                 const DecisionPiece *new_piece,
                                 uint8_t new_edge,
                                 float length_tolerance_mm,
                                 RigidTransform *new_transform,
                                 float *length_error)
{
    uint8_t placed_next =
        (uint8_t)((placed_edge + 1U) % placed_piece->vertex_count);
    uint8_t new_next =
        (uint8_t)((new_edge + 1U) % new_piece->vertex_count);
    DecisionPoint placed_start = transform_point(
        placed_transform, placed_piece->vertices[placed_edge]);
    DecisionPoint placed_end = transform_point(
        placed_transform, placed_piece->vertices[placed_next]);
    DecisionPoint new_start = new_piece->vertices[new_edge];
    DecisionPoint new_end = new_piece->vertices[new_next];
    DecisionPoint source_vector = point_subtract(new_end, new_start);
    DecisionPoint target_vector = point_subtract(placed_start, placed_end);
    DecisionPoint source_midpoint;
    DecisionPoint target_midpoint;
    float source_length = point_length(source_vector);
    float target_length = point_length(target_vector);
    float denominator;

    *length_error = fabsf(source_length - target_length);
    if (source_length <= DECISION_GEOMETRY_EPSILON_MM ||
        target_length <= DECISION_GEOMETRY_EPSILON_MM ||
        *length_error > length_tolerance_mm) {
        return 0U;
    }

    denominator = source_length * target_length;
    new_transform->cosine =
        (source_vector.x_mm * target_vector.x_mm +
         source_vector.y_mm * target_vector.y_mm) / denominator;
    new_transform->sine = point_cross(source_vector, target_vector) /
                          denominator;

    source_midpoint.x_mm = 0.5f * (new_start.x_mm + new_end.x_mm);
    source_midpoint.y_mm = 0.5f * (new_start.y_mm + new_end.y_mm);
    target_midpoint.x_mm = 0.5f * (placed_start.x_mm + placed_end.x_mm);
    target_midpoint.y_mm = 0.5f * (placed_start.y_mm + placed_end.y_mm);
    new_transform->tx = target_midpoint.x_mm -
        (new_transform->cosine * source_midpoint.x_mm -
         new_transform->sine * source_midpoint.y_mm);
    new_transform->ty = target_midpoint.y_mm -
        (new_transform->sine * source_midpoint.x_mm +
         new_transform->cosine * source_midpoint.y_mm);
    return 1U;
}

static void project_layout(const GeneralSearch *search,
                           const RigidTransform *transforms,
                           float angle_rad,
                           float *min_x,
                           float *max_x,
                           float *min_y,
                           float *max_y)
{
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    uint8_t piece_index;

    *min_x = FLT_MAX;
    *max_x = -FLT_MAX;
    *min_y = FLT_MAX;
    *max_y = -FLT_MAX;

    for (piece_index = 0U; piece_index < search->piece_count; ++piece_index) {
        const DecisionPiece *piece = &search->pieces[piece_index];
        uint8_t vertex_index;

        for (vertex_index = 0U;
             vertex_index < piece->vertex_count;
             ++vertex_index) {
            DecisionPoint point = transform_point(&transforms[piece_index],
                                                  piece->vertices[vertex_index]);
            float x = cosine * point.x_mm + sine * point.y_mm;
            float y = -sine * point.x_mm + cosine * point.y_mm;
            if (x < *min_x) *min_x = x;
            if (x > *max_x) *max_x = x;
            if (y < *min_y) *min_y = y;
            if (y > *max_y) *max_y = y;
        }
    }
}

static uint8_t each_piece_has_outer_edge(const GeneralSearch *search,
                                         const RigidTransform *transforms,
                                         float angle_rad,
                                         float min_x,
                                         float max_x,
                                         float min_y,
                                         float max_y)
{
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    float tolerance = search->config->boundary_tolerance_mm;
    uint8_t piece_index;

    for (piece_index = 0U; piece_index < search->piece_count; ++piece_index) {
        const DecisionPiece *piece = &search->pieces[piece_index];
        uint8_t outer_edge_found = 0U;
        uint8_t edge_index;

        for (edge_index = 0U; edge_index < piece->vertex_count; ++edge_index) {
            uint8_t next = (uint8_t)((edge_index + 1U) % piece->vertex_count);
            DecisionPoint p0 = transform_point(&transforms[piece_index],
                                               piece->vertices[edge_index]);
            DecisionPoint p1 = transform_point(&transforms[piece_index],
                                               piece->vertices[next]);
            float x0 = cosine * p0.x_mm + sine * p0.y_mm;
            float y0 = -sine * p0.x_mm + cosine * p0.y_mm;
            float x1 = cosine * p1.x_mm + sine * p1.y_mm;
            float y1 = -sine * p1.x_mm + cosine * p1.y_mm;

            if ((fabsf(x0 - min_x) <= tolerance &&
                 fabsf(x1 - min_x) <= tolerance) ||
                (fabsf(x0 - max_x) <= tolerance &&
                 fabsf(x1 - max_x) <= tolerance) ||
                (fabsf(y0 - min_y) <= tolerance &&
                 fabsf(y1 - min_y) <= tolerance) ||
                (fabsf(y0 - max_y) <= tolerance &&
                 fabsf(y1 - max_y) <= tolerance)) {
                outer_edge_found = 1U;
                break;
            }
        }

        if (outer_edge_found == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static void evaluate_complete_layout(GeneralSearch *search,
                                     const RigidTransform *transforms,
                                     float attachment_error)
{
    uint8_t piece_index;

    for (piece_index = 0U; piece_index < search->piece_count; ++piece_index) {
        const DecisionPiece *piece = &search->pieces[piece_index];
        uint8_t edge_index;

        for (edge_index = 0U; edge_index < piece->vertex_count; ++edge_index) {
            uint8_t next = (uint8_t)((edge_index + 1U) % piece->vertex_count);
            DecisionPoint start = transform_point(&transforms[piece_index],
                                                  piece->vertices[edge_index]);
            DecisionPoint end = transform_point(&transforms[piece_index],
                                                piece->vertices[next]);
            DecisionPoint edge = point_subtract(end, start);
            float angle = atan2f(edge.y_mm, edge.x_mm);
            float min_x;
            float max_x;
            float min_y;
            float max_y;
            float width;
            float height;
            float long_side;
            float short_side;
            float bounding_area;
            float fill_error_ratio;
            float score;

            project_layout(search, transforms, angle,
                           &min_x, &max_x, &min_y, &max_y);
            width = max_x - min_x;
            height = max_y - min_y;

            if (width < height) {
                angle += 0.5f * DECISION_PI;
                project_layout(search, transforms, angle,
                               &min_x, &max_x, &min_y, &max_y);
                width = max_x - min_x;
                height = max_y - min_y;
            }

            long_side = width;
            short_side = height;
            if (long_side < search->config->min_long_side_mm ||
                long_side > search->config->max_long_side_mm ||
                short_side < search->config->min_short_side_mm ||
                short_side > search->config->max_short_side_mm) {
                continue;
            }

            bounding_area = long_side * short_side;
            if (bounding_area <= DECISION_MIN_POLYGON_AREA_MM2) {
                continue;
            }
            fill_error_ratio = fabsf(bounding_area - search->total_area) /
                               bounding_area;
            if (fill_error_ratio > search->config->max_fill_error_ratio) {
                continue;
            }
            if (each_piece_has_outer_edge(search,
                                          transforms,
                                          angle,
                                          min_x,
                                          max_x,
                                          min_y,
                                          max_y) == 0U) {
                continue;
            }

            score = fill_error_ratio +
                    attachment_error /
                    (long_side + short_side + DECISION_GEOMETRY_EPSILON_MM);
            if (score < search->best_score) {
                search->best_score = score;
                search->best_rect_angle_rad = angle;
                search->best_min_x = min_x;
                search->best_max_x = max_x;
                search->best_min_y = min_y;
                search->best_max_y = max_y;
                (void)memcpy(search->best_transforms,
                             transforms,
                             sizeof(search->best_transforms));
                search->solution_found = 1U;
            }
        }
    }
}

static void search_layout(GeneralSearch *search,
                          uint8_t placed_mask,
                          RigidTransform transforms[DECISION_MAX_PIECES],
                          uint8_t used_edges[DECISION_MAX_PIECES],
                          float attachment_error)
{
    uint8_t placed_index;

    ++search->nodes;
    if (search->nodes > search->config->max_search_nodes) {
        search->limit_reached = 1U;
        return;
    }

    if (placed_mask == (uint8_t)((1U << search->piece_count) - 1U)) {
        evaluate_complete_layout(search, transforms, attachment_error);
        return;
    }

    for (placed_index = 0U;
         placed_index < search->piece_count;
         ++placed_index) {
        const DecisionPiece *placed_piece;
        uint8_t placed_edge;

        if ((placed_mask & (uint8_t)(1U << placed_index)) == 0U) {
            continue;
        }
        placed_piece = &search->pieces[placed_index];

        for (placed_edge = 0U;
             placed_edge < placed_piece->vertex_count;
             ++placed_edge) {
            uint8_t new_index;

            if ((used_edges[placed_index] &
                 (uint8_t)(1U << placed_edge)) != 0U) {
                continue;
            }

            for (new_index = 0U;
                 new_index < search->piece_count;
                 ++new_index) {
                const DecisionPiece *new_piece;
                uint8_t new_edge;

                if ((placed_mask & (uint8_t)(1U << new_index)) != 0U) {
                    continue;
                }
                new_piece = &search->pieces[new_index];

                for (new_edge = 0U;
                     new_edge < new_piece->vertex_count;
                     ++new_edge) {
                    RigidTransform candidate_transform;
                    float length_error;
                    uint8_t overlap = 0U;
                    uint8_t compare_index;
                    uint8_t previous_placed_edges;

                    if (align_piece_edges(placed_piece,
                                          &transforms[placed_index],
                                          placed_edge,
                                          new_piece,
                                          new_edge,
                                          search->config->edge_length_tolerance_mm,
                                          &candidate_transform,
                                          &length_error) == 0U) {
                        continue;
                    }

                    for (compare_index = 0U;
                         compare_index < search->piece_count;
                         ++compare_index) {
                        if ((placed_mask &
                             (uint8_t)(1U << compare_index)) != 0U &&
                            polygons_overlap(&search->pieces[compare_index],
                                             &transforms[compare_index],
                                             new_piece,
                                             &candidate_transform) != 0U) {
                            overlap = 1U;
                            break;
                        }
                    }
                    if (overlap != 0U) {
                        continue;
                    }

                    transforms[new_index] = candidate_transform;
                    previous_placed_edges = used_edges[placed_index];
                    used_edges[placed_index] |=
                        (uint8_t)(1U << placed_edge);
                    used_edges[new_index] = (uint8_t)(1U << new_edge);

                    search_layout(search,
                                  (uint8_t)(placed_mask |
                                            (uint8_t)(1U << new_index)),
                                  transforms,
                                  used_edges,
                                  attachment_error + length_error);

                    used_edges[placed_index] = previous_placed_edges;
                    used_edges[new_index] = 0U;
                    if (search->limit_reached != 0U) {
                        return;
                    }
                }
            }
        }
    }
}

void Decision_GetDefaultConfig(DecisionConfig *config)
{
    if (config == NULL) {
        return;
    }

    config->target_center.x_mm = 105.0f;
    config->target_center.y_mm = 220.0f;
    config->pick_z_mm = 0.0f;
    config->transit_z_mm = 40.0f;
    config->place_z_mm = 0.0f;
    config->edge_length_tolerance_mm = 3.0f;
    config->boundary_tolerance_mm = 3.0f;
    config->max_fill_error_ratio = 0.08f;
    config->min_short_side_mm = 50.0f;
    config->max_short_side_mm = 90.0f;
    config->min_long_side_mm = 90.0f;
    config->max_long_side_mm = 120.0f;
    config->max_search_nodes = DECISION_DEFAULT_MAX_NODES;
}

DecisionResult Decision_SolveFixed(const DecisionVisionFrame *frame,
                                   const DecisionFixedLayout *layout,
                                   const DecisionConfig *config,
                                   DecisionPlan *plan)
{
    DecisionPiece pieces[DECISION_MAX_PIECES];
    uint8_t piece_index;

    if (frame == NULL || layout == NULL || plan == NULL ||
        config_is_valid(config) == 0U ||
        layout->piece_count == 0U ||
        layout->piece_count > DECISION_MAX_PIECES) {
        return DECISION_RESULT_INVALID_ARGUMENT;
    }
    if (normalize_frame(frame, pieces) == 0U) {
        return DECISION_RESULT_INVALID_FRAME;
    }

    (void)memset(plan, 0, sizeof(*plan));
    plan->seq = frame->seq;
    for (piece_index = 0U; piece_index < frame->piece_count; ++piece_index) {
        const DecisionFixedPiece *target =
            find_fixed_piece(layout, pieces[piece_index].id);
        RigidTransform transform;
        DecisionPoint grasp;
        DecisionPoint place;
        float error;
        float rotation_deg;
        uint8_t target_vertex_index;

        if (target == NULL) {
            return DECISION_RESULT_TEMPLATE_NOT_FOUND;
        }
        if (target->vertex_count != pieces[piece_index].vertex_count) {
            return DECISION_RESULT_TEMPLATE_MISMATCH;
        }
        for (target_vertex_index = 0U;
             target_vertex_index < target->vertex_count;
             ++target_vertex_index) {
            if (!point_is_finite(target->target_vertices[target_vertex_index])) {
                return DECISION_RESULT_TEMPLATE_MISMATCH;
            }
        }

        if (find_best_fixed_transform(&pieces[piece_index],
                                      target,
                                      &transform,
                                      &error) == 0U ||
            error > config->edge_length_tolerance_mm) {
            return DECISION_RESULT_TEMPLATE_MISMATCH;
        }

        grasp = find_grasp_point(&pieces[piece_index]);
        place = transform_point(&transform, grasp);
        rotation_deg = atan2f(transform.sine, transform.cosine) *
                       180.0f / DECISION_PI;
        fill_move(&pieces[piece_index],
                  grasp,
                  place,
                  rotation_deg,
                  config,
                  &plan->moves[plan->move_count]);
        ++plan->move_count;
    }
    return DECISION_RESULT_OK;
}

DecisionResult Decision_SolveGeneral(const DecisionVisionFrame *frame,
                                     const DecisionConfig *config,
                                     DecisionPlan *plan)
{
    GeneralSearch search;
    RigidTransform transforms[DECISION_MAX_PIECES];
    uint8_t used_edges[DECISION_MAX_PIECES] = {0U};
    float rectangle_center_x;
    float rectangle_center_y;
    float rectangle_cosine;
    float rectangle_sine;
    uint8_t piece_index;

    if (frame == NULL || plan == NULL || config_is_valid(config) == 0U) {
        return DECISION_RESULT_INVALID_ARGUMENT;
    }

    (void)memset(&search, 0, sizeof(search));
    if (normalize_frame(frame, search.pieces) == 0U) {
        return DECISION_RESULT_INVALID_FRAME;
    }
    search.config = config;
    search.piece_count = frame->piece_count;
    search.best_score = FLT_MAX;

    for (piece_index = 0U; piece_index < search.piece_count; ++piece_index) {
        search.total_area += polygon_signed_area(
            search.pieces[piece_index].vertices,
            search.pieces[piece_index].vertex_count);
        transforms[piece_index].cosine = 1.0f;
        transforms[piece_index].sine = 0.0f;
        transforms[piece_index].tx = 0.0f;
        transforms[piece_index].ty = 0.0f;
    }

    search_layout(&search, 1U, transforms, used_edges, 0.0f);
    if (search.solution_found == 0U) {
        return search.limit_reached != 0U ?
            DECISION_RESULT_SEARCH_LIMIT : DECISION_RESULT_NO_SOLUTION;
    }

    rectangle_center_x = 0.5f * (search.best_min_x + search.best_max_x);
    rectangle_center_y = 0.5f * (search.best_min_y + search.best_max_y);
    rectangle_cosine = cosf(search.best_rect_angle_rad);
    rectangle_sine = sinf(search.best_rect_angle_rad);

    (void)memset(plan, 0, sizeof(*plan));
    plan->seq = frame->seq;
    for (piece_index = 0U; piece_index < search.piece_count; ++piece_index) {
        DecisionPoint grasp = find_grasp_point(&search.pieces[piece_index]);
        DecisionPoint layout_grasp = transform_point(
            &search.best_transforms[piece_index], grasp);
        DecisionPoint place;
        float rotation_rad = atan2f(search.best_transforms[piece_index].sine,
                                    search.best_transforms[piece_index].cosine) -
                             search.best_rect_angle_rad;

        place.x_mm = config->target_center.x_mm +
            rectangle_cosine * layout_grasp.x_mm +
            rectangle_sine * layout_grasp.y_mm - rectangle_center_x;
        place.y_mm = config->target_center.y_mm -
            rectangle_sine * layout_grasp.x_mm +
            rectangle_cosine * layout_grasp.y_mm - rectangle_center_y;

        fill_move(&search.pieces[piece_index],
                  grasp,
                  place,
                  rotation_rad * 180.0f / DECISION_PI,
                  config,
                  &plan->moves[plan->move_count]);
        ++plan->move_count;
    }
    return DECISION_RESULT_OK;
}

DecisionResult Decision_Solve(DecisionMode mode,
                              const DecisionVisionFrame *frame,
                              const DecisionFixedLayout *fixed_layout,
                              const DecisionConfig *config,
                              DecisionPlan *plan)
{
    if (mode == DECISION_MODE_FIXED_ID) {
        return Decision_SolveFixed(frame, fixed_layout, config, plan);
    }
    if (mode == DECISION_MODE_GENERAL) {
        return Decision_SolveGeneral(frame, config, plan);
    }
    return DECISION_RESULT_INVALID_ARGUMENT;
}

/* Coincident waypoints would insert a needless full stop, so drop them. */
static uint8_t append_distinct_waypoint(TrajectoryPath *path,
                                        const TrajectoryPose *pose)
{
    if (path->point_count > 0U) {
        const TrajectoryPose *last = &path->points[path->point_count - 1U];

        if (fabsf(pose->x_mm - last->x_mm) <= DECISION_WAYPOINT_EPSILON_MM &&
            fabsf(pose->y_mm - last->y_mm) <= DECISION_WAYPOINT_EPSILON_MM &&
            fabsf(pose->z_mm - last->z_mm) <= DECISION_WAYPOINT_EPSILON_MM &&
            fabsf(normalize_degrees(pose->yaw_deg - last->yaw_deg)) <=
                DECISION_WAYPOINT_EPSILON_DEG) {
            path->points[path->point_count - 1U] = *pose;
            return 1U;
        }
    }
    return Trajectory_PathAppend(path, pose);
}

uint8_t Decision_BuildTrajectoryRequest(const DecisionMove *move,
                                        const TrajectoryPose *current,
                                        const TrajectoryLimits *limits,
                                        TrajectoryRequest *request)
{
    TrajectoryPose current_above;

    if (move == NULL || current == NULL || limits == NULL || request == NULL) {
        return 0U;
    }
    if (!isfinite(current->x_mm) || !isfinite(current->y_mm) ||
        !isfinite(current->z_mm) || !isfinite(current->yaw_deg)) {
        return 0U;
    }

    /* Rise straight up first, keeping yaw, so the tool cannot sweep the board.
       The wrist then turns back to the pick angle during the high cruise. */
    current_above = *current;
    if (current_above.z_mm < move->pick_above.z_mm) {
        current_above.z_mm = move->pick_above.z_mm;
    }

    Trajectory_PathReset(&request->approach);
    if (append_distinct_waypoint(&request->approach, current) == 0U ||
        append_distinct_waypoint(&request->approach, &current_above) == 0U ||
        append_distinct_waypoint(&request->approach, &move->pick_above) == 0U ||
        append_distinct_waypoint(&request->approach, &move->pick) == 0U ||
        request->approach.point_count < 2U) {
        return 0U;
    }

    Trajectory_PathReset(&request->transfer);
    if (append_distinct_waypoint(&request->transfer, &move->pick) == 0U ||
        append_distinct_waypoint(&request->transfer, &move->pick_above) == 0U ||
        append_distinct_waypoint(&request->transfer, &move->place_above) == 0U ||
        append_distinct_waypoint(&request->transfer, &move->place) == 0U ||
        request->transfer.point_count < 2U) {
        return 0U;
    }

    request->limits = *limits;
    return 1U;
}

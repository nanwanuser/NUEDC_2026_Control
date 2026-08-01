#include "decision.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DECISION_PI                    3.14159265358979323846f
#define DECISION_GEOMETRY_EPSILON_MM   0.05f
#define DECISION_MIN_POLYGON_AREA_MM2  1.0f
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
    /* Both bound any rectangle the finished layout could still be accepted in;
       see layout_can_still_fit(). */
    float max_diagonal_mm;
    float max_bounding_area_mm2;
    /* Set once a layout is found that the task's scoring cannot tell from a
       perfect one, which ends the search; see DECISION_GOOD_ENOUGH_SCORE. */
    uint8_t good_enough;
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
                     isfinite(config->paper_divider_x_mm) &&
                     config->paper_divider_x_mm >= 0.0f &&
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

static void apply_assembly_clearance(DecisionPoint target_center,
                                     DecisionPoint *place)
{
    const float dx = place->x_mm - target_center.x_mm;
    const float dy = place->y_mm - target_center.y_mm;
    const float distance_mm = sqrtf(dx * dx + dy * dy);

    if (distance_mm > DECISION_GEOMETRY_EPSILON_MM) {
        const float scale = DECISION_ASSEMBLY_CLEARANCE_MM / distance_mm;

        place->x_mm += scale * dx;
        place->y_mm += scale * dy;
    }
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

/* Depth a point has to reach inside the other polygon before the pair counts as
   overlapping. Two pieces joined along edges of unequal length necessarily have
   the longer one poking into its neighbour by up to half the length difference,
   so a zero-tolerance test rejects exactly the layouts the loose edge tolerance
   was widened to admit. Judging by depth instead separates that unavoidable
   nibble from one piece genuinely sitting on top of another. */
static uint8_t point_is_inside_by(DecisionPoint point,
                                  const DecisionPoint *vertices,
                                  uint8_t vertex_count,
                                  float depth_mm)
{
    if (point_polygon_location(point, vertices, vertex_count) <= 0) {
        return 0U;
    }
    return (uint8_t)(minimum_edge_distance(point, vertices, vertex_count) >
                     depth_mm);
}

static uint8_t polygons_overlap(const DecisionPiece *left_piece,
                                const RigidTransform *left_transform,
                                const DecisionPiece *right_piece,
                                const RigidTransform *right_transform,
                                float tolerance_mm)
{
    DecisionPoint left[DECISION_MAX_VERTICES];
    DecisionPoint right[DECISION_MAX_VERTICES];
    uint8_t left_index;
    uint8_t right_index;

    transformed_vertices(left_piece, left_transform, left);
    transformed_vertices(right_piece, right_transform, right);

    /* No edge-crossing test: two pieces joined along edges of unequal length
       cross at the seam by construction, so a crossing on its own says nothing
       about whether they really overlap. The depth tests below decide instead,
       and a genuine stack always drives some vertex, midpoint or centroid well
       inside the other piece. */

    for (left_index = 0U; left_index < left_piece->vertex_count; ++left_index) {
        uint8_t left_next =
            (uint8_t)((left_index + 1U) % left_piece->vertex_count);
        DecisionPoint midpoint = {
            0.5f * (left[left_index].x_mm + left[left_next].x_mm),
            0.5f * (left[left_index].y_mm + left[left_next].y_mm)
        };

        if (point_is_inside_by(left[left_index],
                               right,
                               right_piece->vertex_count,
                               tolerance_mm) != 0U ||
            point_is_inside_by(midpoint,
                               right,
                               right_piece->vertex_count,
                               tolerance_mm) != 0U) {
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

        if (point_is_inside_by(right[right_index],
                               left,
                               left_piece->vertex_count,
                               tolerance_mm) != 0U ||
            point_is_inside_by(midpoint,
                               left,
                               left_piece->vertex_count,
                               tolerance_mm) != 0U) {
            return 1U;
        }
    }

    /* A centroid inside the other piece is a real stack rather than a nibble at
       the seam, so it stays a zero-tolerance rejection. */
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

/* One way of laying a new piece's edge against an already-placed edge. The two
   edges are made anti-parallel so the pieces end up on opposite sides of the
   seam; what is left is where along the seam the new edge sits, which is what
   `anchor` selects. See align_piece_edges(). */
typedef enum {
    EDGE_ANCHOR_CENTERED = 0,
    EDGE_ANCHOR_START,
    EDGE_ANCHOR_END,
    EDGE_ANCHOR_COUNT
} EdgeAnchor;

/* Places `new_edge` of `new_piece` against `placed_edge` of an already-placed
   piece and returns the rigid transform that does it, or zero if that pairing is
   not usable at all.
 *
 * Two edges of a dissection meet in one of two ways. Either they are the two
 * sides of one whole cut, in which case they have the same length up to cutting
 * error, or one of them is only part of the other: a cut that stops partway
 * across the rectangle leaves a long edge on one side facing two shorter edges
 * on the other. That second case is a T-junction, and it is not an exotic
 * arrangement - two independent straight cuts across a rectangle produce one
 * whenever their ends do not happen to coincide, which for hand-cut pieces is
 * essentially always. The task's own figure 2 is cut that way.
 *
 * So the length difference cannot be treated as an error to be bounded. Instead
 * the shorter edge is allowed to be a sub-segment of the longer, and `anchor`
 * picks which sub-segment: flush with one end of the seam, flush with the other,
 * or centred. Centred is the least-squares placement for a pair that really is
 * one whole cut, and the three coincide when the lengths agree, so nothing about
 * the equal-length case changes.
 *
 * `corner_error` is what the caller accumulates as layout quality: the offset
 * between the two edges' ends that were meant to coincide. It is the length
 * disagreement for a centred join and zero for an anchored one, since an
 * anchored join makes one pair of corners meet exactly - which is also the
 * quantity the task scores, adjacent pieces' corresponding vertices within
 * 2 cm. */
static uint8_t align_piece_edges(const DecisionPiece *placed_piece,
                                 const RigidTransform *placed_transform,
                                 uint8_t placed_edge,
                                 const DecisionPiece *new_piece,
                                 uint8_t new_edge,
                                 EdgeAnchor anchor,
                                 float length_tolerance_mm,
                                 RigidTransform *new_transform,
                                 float *corner_error,
                                 uint8_t *spent_placed_edge,
                                 uint8_t *spent_new_edge)
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
    /* Reversed, so the new piece lands on the far side of the seam. */
    DecisionPoint target_vector = point_subtract(placed_start, placed_end);
    DecisionPoint source_anchor;
    DecisionPoint target_anchor;
    float source_length = point_length(source_vector);
    float target_length = point_length(target_vector);
    float length_difference = fabsf(source_length - target_length);
    float shared_length = source_length < target_length
                              ? source_length
                              : target_length;
    float denominator;

    if (source_length <= DECISION_GEOMETRY_EPSILON_MM ||
        target_length <= DECISION_GEOMETRY_EPSILON_MM) {
        return 0U;
    }
    /* However the two edges are laid together, they have to share a real length
       of seam. Without this a piece could be hung off a neighbour by a corner,
       which is not a join and multiplies the branching for nothing. The task
       guarantees every edge is at least 2 cm, so this rejects nothing valid. */
    if (shared_length < DECISION_MIN_EDGE_OVERLAP_MM) {
        return 0U;
    }
    /* A centred join asserts the two edges are the same whole cut, so it still
       has to pass the length tolerance; the anchored ones do not, because for
       them the difference is the rest of the seam rather than an error. */
    if (anchor == EDGE_ANCHOR_CENTERED) {
        if (length_difference > length_tolerance_mm) {
            return 0U;
        }
        *corner_error = length_difference;
        /* One whole cut: neither edge has seam left over. */
        *spent_placed_edge = 1U;
        *spent_new_edge = 1U;
    } else {
        /* Nothing to choose between the two ends when the edges already agree,
           so let the centred candidate stand for all three and skip these. */
        if (length_difference <= length_tolerance_mm) {
            return 0U;
        }
        *corner_error = 0.0f;
        /* The shorter edge lies wholly inside the longer, so only it is spent;
           the longer keeps the rest of the seam for another piece. */
        *spent_placed_edge = (uint8_t)(target_length <= source_length);
        *spent_new_edge = (uint8_t)(source_length <= target_length);
    }

    denominator = source_length * target_length;
    new_transform->cosine =
        (source_vector.x_mm * target_vector.x_mm +
         source_vector.y_mm * target_vector.y_mm) / denominator;
    new_transform->sine = point_cross(source_vector, target_vector) /
                          denominator;

    switch (anchor) {
    case EDGE_ANCHOR_START:
        /* The new edge's first corner meets the seam's first corner, which on
           the reversed placed edge is placed_end. */
        source_anchor = new_start;
        target_anchor = placed_end;
        break;
    case EDGE_ANCHOR_END:
        source_anchor = new_end;
        target_anchor = placed_start;
        break;
    case EDGE_ANCHOR_CENTERED:
    default:
        source_anchor.x_mm = 0.5f * (new_start.x_mm + new_end.x_mm);
        source_anchor.y_mm = 0.5f * (new_start.y_mm + new_end.y_mm);
        target_anchor.x_mm = 0.5f * (placed_start.x_mm + placed_end.x_mm);
        target_anchor.y_mm = 0.5f * (placed_start.y_mm + placed_end.y_mm);
        break;
    }

    new_transform->tx = target_anchor.x_mm -
        (new_transform->cosine * source_anchor.x_mm -
         new_transform->sine * source_anchor.y_mm);
    new_transform->ty = target_anchor.y_mm -
        (new_transform->sine * source_anchor.x_mm +
         new_transform->cosine * source_anchor.y_mm);
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

/* Clipping a triangle by a triangle yields at most six vertices. */
#define DECISION_CLIP_MAX_VERTICES 8U

/* Area shared by two convex polygons, by clipping `subject` against each edge of
   `clip` (Sutherland-Hodgman). Both must wind counter-clockwise. */
static float convex_clip_area(const DecisionPoint *subject,
                              uint8_t subject_count,
                              const DecisionPoint *clip,
                              uint8_t clip_count)
{
    DecisionPoint current[DECISION_CLIP_MAX_VERTICES];
    DecisionPoint clipped[DECISION_CLIP_MAX_VERTICES];
    uint8_t current_count = subject_count;
    uint8_t clip_index;
    uint8_t index;

    for (index = 0U; index < subject_count; ++index) {
        current[index] = subject[index];
    }

    for (clip_index = 0U; clip_index < clip_count; ++clip_index) {
        DecisionPoint origin = clip[clip_index];
        DecisionPoint edge =
            point_subtract(clip[(uint8_t)((clip_index + 1U) % clip_count)],
                           origin);
        uint8_t clipped_count = 0U;

        if (current_count < 3U) {
            return 0.0f;
        }

        for (index = 0U; index < current_count; ++index) {
            DecisionPoint start = current[index];
            DecisionPoint end =
                current[(uint8_t)((index + 1U) % current_count)];
            /* Positive means left of the clip edge, i.e. inside. */
            float start_side = point_cross(edge, point_subtract(start, origin));
            float end_side = point_cross(edge, point_subtract(end, origin));

            if (start_side >= 0.0f &&
                clipped_count < DECISION_CLIP_MAX_VERTICES) {
                clipped[clipped_count] = start;
                ++clipped_count;
            }
            if (((start_side > 0.0f) && (end_side < 0.0f)) ||
                ((start_side < 0.0f) && (end_side > 0.0f))) {
                float fraction = start_side / (start_side - end_side);
                DecisionPoint crossing;

                crossing.x_mm = start.x_mm +
                                fraction * (end.x_mm - start.x_mm);
                crossing.y_mm = start.y_mm +
                                fraction * (end.y_mm - start.y_mm);
                if (clipped_count < DECISION_CLIP_MAX_VERTICES) {
                    clipped[clipped_count] = crossing;
                    ++clipped_count;
                }
            }
        }

        for (index = 0U; index < clipped_count; ++index) {
            current[index] = clipped[index];
        }
        current_count = clipped_count;
    }

    if (current_count < 3U) {
        return 0.0f;
    }
    return fabsf(polygon_signed_area(current, current_count));
}

/* Area covered by both polygons at once. Each polygon is split into the fan of
   triangles from its first vertex; taken with their signed orientations those
   triangles sum to the polygon's own indicator function, which holds for concave
   polygons too, so summing the signed pairwise triangle intersections gives the
   exact shared area rather than an approximation. */
static float polygon_intersection_area(const DecisionPoint *left,
                                       uint8_t left_count,
                                       const DecisionPoint *right,
                                       uint8_t right_count)
{
    float total = 0.0f;
    uint8_t left_index;
    uint8_t right_index;

    for (left_index = 1U; (uint8_t)(left_index + 1U) < left_count;
         ++left_index) {
        DecisionPoint left_triangle[3];
        float left_area;

        left_triangle[0] = left[0];
        left_triangle[1] = left[left_index];
        left_triangle[2] = left[left_index + 1U];
        left_area = polygon_signed_area(left_triangle, 3U);
        if (fabsf(left_area) < DECISION_MIN_POLYGON_AREA_MM2) {
            continue;
        }
        if (left_area < 0.0f) {
            DecisionPoint swap = left_triangle[1];
            left_triangle[1] = left_triangle[2];
            left_triangle[2] = swap;
        }

        for (right_index = 1U; (uint8_t)(right_index + 1U) < right_count;
             ++right_index) {
            DecisionPoint right_triangle[3];
            float right_area;
            float shared;

            right_triangle[0] = right[0];
            right_triangle[1] = right[right_index];
            right_triangle[2] = right[right_index + 1U];
            right_area = polygon_signed_area(right_triangle, 3U);
            if (fabsf(right_area) < DECISION_MIN_POLYGON_AREA_MM2) {
                continue;
            }
            if (right_area < 0.0f) {
                DecisionPoint swap = right_triangle[1];
                right_triangle[1] = right_triangle[2];
                right_triangle[2] = swap;
            }

            shared = convex_clip_area(left_triangle, 3U, right_triangle, 3U);
            if ((left_area < 0.0f) != (right_area < 0.0f)) {
                total -= shared;
            } else {
                total += shared;
            }
        }
    }
    return total;
}

/* Total doubly-covered area in the layout. Summing over pairs counts an area
   shared by three pieces more than once, which only ever overstates the error of
   a layout already far past being a tiling. */
static float layout_overlap_area(const GeneralSearch *search,
                                 const RigidTransform *transforms)
{
    float total = 0.0f;
    uint8_t left_index;

    for (left_index = 0U;
         (uint8_t)(left_index + 1U) < search->piece_count;
         ++left_index) {
        DecisionPoint left[DECISION_MAX_VERTICES];
        uint8_t right_index;

        transformed_vertices(&search->pieces[left_index],
                             &transforms[left_index],
                             left);
        for (right_index = (uint8_t)(left_index + 1U);
             right_index < search->piece_count;
             ++right_index) {
            DecisionPoint right[DECISION_MAX_VERTICES];
            float shared;

            transformed_vertices(&search->pieces[right_index],
                                 &transforms[right_index],
                                 right);
            shared = polygon_intersection_area(
                left, search->pieces[left_index].vertex_count,
                right, search->pieces[right_index].vertex_count);
            if (shared > 0.0f) {
                total += shared;
            }
        }
    }
    return total;
}

static void evaluate_complete_layout(GeneralSearch *search,
                                     const RigidTransform *transforms,
                                     float attachment_error)
{
    uint8_t piece_index;
    /* Neither depends on which rectangle orientation is tried below, so both are
       measured once rather than inside the loop. */
    float overlap_area = layout_overlap_area(search, transforms);
    float covered_area = search->total_area - overlap_area;

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
            /* Charge for gap and for overlap, not for their difference. The
               pieces' areas always sum to the true rectangle, so comparing that
               sum against the bounding box says nothing about whether they
               actually tile it: stacking two pieces shrinks the box by about as
               much as the hole it leaves, and the two errors cancel. Measuring
               the uncovered area and the doubly-covered area separately is what
               distinguishes a tiling from a pile. */
            fill_error_ratio = ((bounding_area - covered_area) + overlap_area) /
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
                search->good_enough =
                    (uint8_t)(score <= DECISION_GOOD_ENOUGH_SCORE);
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

/* Can the layout placed so far still end up inside the largest allowed target
 * rectangle?
 *
 * Attaching another piece never moves the pieces already placed, so a partial
 * layout that is too big to fit is a dead end no matter what follows, and the
 * whole subtree can be dropped. Allowing edges of unequal length to meet
 * multiplies the branching, and without a prune the node budget is spent long
 * before the search reaches the true tiling - which is what made the solver
 * return whichever arrangement it happened to stumble on first.
 *
 * Two necessary conditions are tested, both of which are exact for the point set
 * rather than approximations, so this never discards a layout that could have
 * been accepted:
 *
 * - Every distance inside a w x h rectangle is at most its diagonal.
 * - Some orientation must enclose the set in a rectangle that is within the
 *   allowed side lengths and small enough in area for the pieces to fill it. The
 *   smallest enclosing rectangle of a convex hull always has a side flush with
 *   one of its edges, so walking the hull edges tries every orientation that
 *   could win. The area test is what the fill gate will demand at the end: the
 *   pieces have a fixed total area and a rectangle bigger than
 *   total_area / (1 - max_fill_error_ratio) cannot be filled by them however the
 *   rest are placed, since adding pieces only ever grows the enclosing
 *   rectangle.
 *
 * Vertices are used rather than the hull for the diameter, since the diameter of
 * a point set is attained at hull vertices anyway and the sets here are tiny. */
static uint8_t layout_can_still_fit(const GeneralSearch *search,
                                    uint8_t placed_mask,
                                    const RigidTransform *transforms)
{
    DecisionPoint points[DECISION_MAX_PIECES * DECISION_MAX_VERTICES];
    uint8_t hull[2U * DECISION_MAX_PIECES * DECISION_MAX_VERTICES];
    uint8_t order[DECISION_MAX_PIECES * DECISION_MAX_VERTICES];
    uint8_t count = 0U;
    uint8_t hull_count = 0U;
    uint8_t piece_index;
    uint8_t i;
    uint8_t j;

    for (piece_index = 0U; piece_index < search->piece_count; ++piece_index) {
        const DecisionPiece *piece = &search->pieces[piece_index];
        uint8_t vertex_index;

        if ((placed_mask & (uint8_t)(1U << piece_index)) == 0U) {
            continue;
        }
        for (vertex_index = 0U;
             vertex_index < piece->vertex_count;
             ++vertex_index) {
            points[count] = transform_point(&transforms[piece_index],
                                            piece->vertices[vertex_index]);
            ++count;
        }
    }

    for (i = 0U; i < count; ++i) {
        for (j = (uint8_t)(i + 1U); j < count; ++j) {
            if (point_length(point_subtract(points[i], points[j])) >
                search->max_diagonal_mm) {
                return 0U;
            }
        }
    }

    /* Andrew's monotone chain over indices sorted by x then y. */
    for (i = 0U; i < count; ++i) {
        order[i] = i;
    }
    for (i = 1U; i < count; ++i) {
        uint8_t key = order[i];
        j = i;
        while (j > 0U &&
               (points[order[j - 1U]].x_mm > points[key].x_mm ||
                (points[order[j - 1U]].x_mm == points[key].x_mm &&
                 points[order[j - 1U]].y_mm > points[key].y_mm))) {
            order[j] = order[j - 1U];
            --j;
        }
        order[j] = key;
    }
    for (i = 0U; i < count; ++i) {
        while (hull_count >= 2U &&
               point_cross(point_subtract(points[hull[hull_count - 1U]],
                                          points[hull[hull_count - 2U]]),
                           point_subtract(points[order[i]],
                                          points[hull[hull_count - 2U]])) <=
                   0.0f) {
            --hull_count;
        }
        hull[hull_count] = order[i];
        ++hull_count;
    }
    {
        const uint8_t lower_count = (uint8_t)(hull_count + 1U);

        for (i = (uint8_t)(count - 1U); i > 0U; --i) {
            while (hull_count >= lower_count &&
                   point_cross(point_subtract(points[hull[hull_count - 1U]],
                                              points[hull[hull_count - 2U]]),
                               point_subtract(points[order[i - 1U]],
                                              points[hull[hull_count - 2U]])) <=
                       0.0f) {
                --hull_count;
            }
            hull[hull_count] = order[i - 1U];
            ++hull_count;
        }
    }
    /* The walk closes on its start, which is not a distinct hull vertex. */
    if (hull_count > 1U) {
        --hull_count;
    }
    if (hull_count < 2U) {
        return 1U;
    }

    /* Does some orientation contain the set within the largest allowed target
       rectangle? The smallest enclosing rectangle of a convex hull always has a
       side flush with one of its edges, so trying each hull edge's direction
       covers every orientation that could win. */
    for (i = 0U; i < hull_count; ++i) {
        const uint8_t next = (uint8_t)((i + 1U) % hull_count);
        DecisionPoint edge = point_subtract(points[hull[next]],
                                           points[hull[i]]);
        const float length = point_length(edge);
        float along_min = FLT_MAX;
        float along_max = -FLT_MAX;
        float across_min = FLT_MAX;
        float across_max = -FLT_MAX;
        float width;
        float height;
        float long_side;
        float short_side;

        if (length <= DECISION_GEOMETRY_EPSILON_MM) {
            continue;
        }
        for (j = 0U; j < count; ++j) {
            const float along = (points[j].x_mm * edge.x_mm +
                                 points[j].y_mm * edge.y_mm) / length;
            const float across = point_cross(edge, points[j]) / length;

            if (along < along_min) along_min = along;
            if (along > along_max) along_max = along;
            if (across < across_min) across_min = across;
            if (across > across_max) across_max = across;
        }
        width = along_max - along_min;
        height = across_max - across_min;
        long_side = width > height ? width : height;
        short_side = width > height ? height : width;
        if (long_side <= search->config->max_long_side_mm &&
            short_side <= search->config->max_short_side_mm &&
            long_side * short_side <= search->max_bounding_area_mm2) {
            return 1U;
        }
    }

    return 0U;
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

    if (search->good_enough != 0U) {
        return;
    }

    if (layout_can_still_fit(search, placed_mask, transforms) == 0U) {
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

            /* Only fully consumed edges are closed to further joins. An edge
               that took an anchored partner still has seam left over for the
               next piece, which is the whole point of a T-junction. */
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
                    uint8_t anchor;

                    for (anchor = 0U; anchor < (uint8_t)EDGE_ANCHOR_COUNT;
                         ++anchor) {
                        RigidTransform candidate_transform;
                        float corner_error;
                        uint8_t spent_placed_edge = 0U;
                        uint8_t spent_new_edge = 0U;
                        uint8_t overlap = 0U;
                        uint8_t compare_index;
                        uint8_t previous_placed_edges;
                        uint8_t previous_new_edges;

                        if (align_piece_edges(
                                placed_piece,
                                &transforms[placed_index],
                                placed_edge,
                                new_piece,
                                new_edge,
                                (EdgeAnchor)anchor,
                                search->config->edge_length_tolerance_mm,
                                &candidate_transform,
                                &corner_error,
                                &spent_placed_edge,
                                &spent_new_edge) == 0U) {
                            continue;
                        }

                        for (compare_index = 0U;
                             compare_index < search->piece_count;
                             ++compare_index) {
                            if ((placed_mask &
                                 (uint8_t)(1U << compare_index)) != 0U &&
                                polygons_overlap(
                                    &search->pieces[compare_index],
                                    &transforms[compare_index],
                                    new_piece,
                                    &candidate_transform,
                                    DECISION_OVERLAP_TOLERANCE_MM) != 0U) {
                                overlap = 1U;
                                break;
                            }
                        }
                        if (overlap != 0U) {
                            continue;
                        }

                        transforms[new_index] = candidate_transform;
                        previous_placed_edges = used_edges[placed_index];
                        previous_new_edges = used_edges[new_index];
                        /* Mark whatever this join used up. A centred join spends
                           both edges. An anchored one lays the shorter edge
                           inside the longer, so the shorter is spent and the
                           longer keeps the leftover seam for the next piece -
                           which is exactly the T-junction that closing both
                           edges used to make unreachable.
                         *
                           Accumulated rather than assigned: a piece placed
                           earlier in this branch may already have edges spoken
                           for, and overwriting them reopened seams that were
                           already taken. */
                        if (spent_placed_edge != 0U) {
                            used_edges[placed_index] |=
                                (uint8_t)(1U << placed_edge);
                        }
                        if (spent_new_edge != 0U) {
                            used_edges[new_index] |=
                                (uint8_t)(1U << new_edge);
                        }

                        search_layout(search,
                                      (uint8_t)(placed_mask |
                                                (uint8_t)(1U << new_index)),
                                      transforms,
                                      used_edges,
                                      attachment_error + corner_error);

                        used_edges[placed_index] = previous_placed_edges;
                        used_edges[new_index] = previous_new_edges;
                        if (search->limit_reached != 0U) {
                            return;
                        }
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

    /* Centre of the place half of the sheet: the pieces come from x < 148.5 and
       are assembled on the other side of that line. The task layer overrides this
       with a point chosen against the crane's reach band, but the default has to
       be in the right half on its own, or a caller that takes it as-is assembles
       on top of the pieces. */
    config->target_center.x_mm = 222.75f;
    config->target_center.y_mm = 105.0f;
    config->paper_divider_x_mm = DECISION_PAPER_DIVIDER_X_MM;
    config->pick_z_mm = 0.0f;
    config->transit_z_mm = 40.0f;
    config->place_z_mm = 0.0f;
    config->edge_length_tolerance_mm = DECISION_EDGE_TOLERANCE_MM;
    config->boundary_tolerance_mm = DECISION_BOUNDARY_TOLERANCE_MM;
    config->max_fill_error_ratio = DECISION_MAX_FILL_ERROR_RATIO;
    config->min_short_side_mm = DECISION_MIN_SHORT_SIDE_MM;
    config->max_short_side_mm = DECISION_MAX_SHORT_SIDE_MM;
    config->min_long_side_mm = DECISION_MIN_LONG_SIDE_MM;
    config->max_long_side_mm = DECISION_MAX_LONG_SIDE_MM;
    config->max_search_nodes = DECISION_DEFAULT_MAX_NODES;
}

DecisionResult Decision_SolveGeneral(const DecisionVisionFrame *frame,
                                     const DecisionConfig *config,
                                     DecisionPlan *plan)
{
    GeneralSearch search;
    RigidTransform transforms[DECISION_MAX_PIECES];
    uint8_t used_edges[DECISION_MAX_PIECES] = {0U};
    DecisionPoint target_center;
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
    /* Checked before the layout search rather than after it, because a frame from
       the wrong half is not a geometry problem and solving it first would only
       spend the node budget to reach the same answer. */
    if (config->paper_divider_x_mm > 0.0f) {
        for (piece_index = 0U; piece_index < frame->piece_count;
             ++piece_index) {
            if (frame->pieces[piece_index].center.x_mm >=
                config->paper_divider_x_mm) {
                return DECISION_RESULT_WRONG_HALF;
            }
        }
    }
    search.config = config;
    search.piece_count = frame->piece_count;
    search.best_score = FLT_MAX;
    search.max_diagonal_mm = sqrtf(square(config->max_long_side_mm) +
                                   square(config->max_short_side_mm));

    for (piece_index = 0U; piece_index < search.piece_count; ++piece_index) {
        search.total_area += polygon_signed_area(
            search.pieces[piece_index].vertices,
            search.pieces[piece_index].vertex_count);
        transforms[piece_index].cosine = 1.0f;
        transforms[piece_index].sine = 0.0f;
        transforms[piece_index].tx = 0.0f;
        transforms[piece_index].ty = 0.0f;
    }

    /* Largest rectangle the pieces could still fill to the accuracy the fill gate
       demands. Kept just above the gate's own bound so the prune never rejects a
       layout evaluate_complete_layout() would have accepted. */
    search.max_bounding_area_mm2 = search.total_area /
        (1.0f - config->max_fill_error_ratio) + 1.0f;

    search_layout(&search, 1U, transforms, used_edges, 0.0f);
    if (search.solution_found == 0U) {
        return search.limit_reached != 0U ?
            DECISION_RESULT_SEARCH_LIMIT : DECISION_RESULT_NO_SOLUTION;
    }

    rectangle_center_x = 0.5f * (search.best_min_x + search.best_max_x);
    rectangle_center_y = 0.5f * (search.best_min_y + search.best_max_y);
    rectangle_cosine = cosf(search.best_rect_angle_rad);
    rectangle_sine = sinf(search.best_rect_angle_rad);
    /* Which half is which is fixed by the shared A4 frame, not read off the
       pieces: picking happens in the left half and assembly in the right one, so
       the stated target is used exactly as given. */
    target_center = config->target_center;

    (void)memset(plan, 0, sizeof(*plan));
    plan->seq = frame->seq;
    for (piece_index = 0U; piece_index < search.piece_count; ++piece_index) {
        /* The camera supplies the calibrated magnet target. Preserve it exactly
           instead of replacing it with a grid-searched polygon interior point. */
        DecisionPoint grasp = search.pieces[piece_index].center;
        DecisionPoint layout_grasp = transform_point(
            &search.best_transforms[piece_index], grasp);
        DecisionPoint place;
        float rotation_rad = atan2f(search.best_transforms[piece_index].sine,
                                    search.best_transforms[piece_index].cosine) -
                             search.best_rect_angle_rad;

        place.x_mm = target_center.x_mm +
            rectangle_cosine * layout_grasp.x_mm +
            rectangle_sine * layout_grasp.y_mm - rectangle_center_x;
        place.y_mm = target_center.y_mm -
            rectangle_sine * layout_grasp.x_mm +
            rectangle_cosine * layout_grasp.y_mm - rectangle_center_y;
        apply_assembly_clearance(target_center, &place);

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

DecisionResult Decision_Solve(const DecisionVisionFrame *frame,
                              const DecisionConfig *config,
                              DecisionPlan *plan)
{
    return Decision_SolveGeneral(frame, config, plan);
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
        request->approach.point_count < 2U) {
        return 0U;
    }

    Trajectory_PathReset(&request->transfer);
    if (append_distinct_waypoint(&request->transfer, &move->pick_above) == 0U ||
        append_distinct_waypoint(&request->transfer, &move->place_above) == 0U ||
        request->transfer.point_count < 2U) {
        return 0U;
    }

    request->limits = *limits;
    return 1U;
}

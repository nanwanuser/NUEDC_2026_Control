/* Host tests for the assembly solver's tolerance to imperfect pieces.
 *
 * The device is scored on one geometric criterion: adjacent pieces' matching
 * vertices within 2 cm. Every threshold in DecisionConfig is looser than the
 * pieces are cut on purpose, because a threshold that rejects the true layout
 * yields NO_SOLUTION rather than a lower score. These tests pin that down: a
 * rectangle is cut into pieces, the cut is then spoiled by a stated amount, and
 * the solver still has to find an arrangement. */

#include "decision.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

static void check(int condition, const char *what)
{
    if (condition == 0) {
        printf("FAIL: %s\n", what);
        ++failures;
    }
}

static void set_piece(DecisionPiece *piece,
                      uint8_t id,
                      uint8_t vertex_count,
                      const float *xy)
{
    uint8_t index;

    (void)memset(piece, 0, sizeof(*piece));
    piece->id = id;
    piece->vertex_count = vertex_count;
    for (index = 0U; index < vertex_count; ++index) {
        piece->vertices[index].x_mm = xy[2U * index];
        piece->vertices[index].y_mm = xy[2U * index + 1U];
    }
    /* The centre is the pick point supplied by vision. */
    for (index = 0U; index < vertex_count; ++index) {
        piece->center.x_mm += piece->vertices[index].x_mm;
        piece->center.y_mm += piece->vertices[index].y_mm;
    }
    piece->center.x_mm /= (float)vertex_count;
    piece->center.y_mm /= (float)vertex_count;
}

/* Rotates and translates a piece so the solver is not handed pieces already in
   their solved pose, which would let a broken search still pass. */
static void scatter(DecisionPiece *piece, float degrees, float dx, float dy)
{
    const float radians = degrees * 3.14159265358979323846f / 180.0f;
    const float c = cosf(radians);
    const float s = sinf(radians);
    uint8_t index;

    for (index = 0U; index < piece->vertex_count; ++index) {
        const float x = piece->vertices[index].x_mm;
        const float y = piece->vertices[index].y_mm;

        piece->vertices[index].x_mm = c * x - s * y + dx;
        piece->vertices[index].y_mm = s * x + c * y + dy;
    }
    {
        const float x = piece->center.x_mm;
        const float y = piece->center.y_mm;

        piece->center.x_mm = c * x - s * y + dx;
        piece->center.y_mm = s * x + c * y + dy;
    }
}

/* A 100x70 mm rectangle split into four quadrant pieces, which is inside the
   task's 9x5..12x9 cm envelope.
 *
 * `error_mm` is applied to each piece independently, so the interior cut is no
 * longer shared: every piece's idea of where the middle is differs by up to
 * this much. That is the defect a hand-cut set has and the reason the mating
 * edges disagree in length. Moving one shared corner instead would keep the four
 * quadrants a perfect tiling and prove nothing. */
static void build_frame(DecisionVisionFrame *frame, float error_mm)
{
    const float w = 100.0f;
    const float h = 70.0f;
    const float cx = 0.5f * w;
    const float cy = 0.5f * h;
    /* Fixed signs rather than random, so a failure is reproducible. */
    const float e = error_mm;

    /* Each piece keeps the outer rectangle corner it owns, since the task
       guarantees every piece has an edge on the target rectangle's boundary.
       Only the interior vertices carry the error. */
    const float p0[8] = {0.0f, 0.0f, cx + e, 0.0f, cx + e, cy - e, 0.0f, cy - e};
    const float p1[8] = {cx - e, 0.0f, w, 0.0f, w, cy + e, cx - e, cy + e};
    const float p2[8] = {cx + e, cy - e, w, cy - e, w, h, cx + e, h};
    const float p3[8] = {0.0f, cy + e, cx - e, cy + e, cx - e, h, 0.0f, h};

    (void)memset(frame, 0, sizeof(*frame));
    frame->seq = 1U;
    frame->piece_count = 4U;
    set_piece(&frame->pieces[0], 1U, 4U, p0);
    set_piece(&frame->pieces[1], 2U, 4U, p1);
    set_piece(&frame->pieces[2], 3U, 4U, p2);
    set_piece(&frame->pieces[3], 4U, 4U, p3);

    /* Scattered across the pick half of the sheet, which is where the vision end
       measures them: landscape A4, top-left origin, +X right, +Y down, pieces
       left of x = 148.5 and the assembly built right of it. The solver rejects a
       frame whose pieces straddle that line, so these translations are chosen to
       put every piece centre well inside the left half rather than anywhere
       convenient. */
    scatter(&frame->pieces[0], 17.0f, 21.0f, 31.0f);
    scatter(&frame->pieces[1], -35.0f, 38.5f, 83.5f);
    scatter(&frame->pieces[2], 78.0f, 76.0f, 66.0f);
    scatter(&frame->pieces[3], -61.0f, 52.0f, 146.0f);
}

static const char *result_name(DecisionResult result)
{
    switch (result) {
    case DECISION_RESULT_OK:               return "OK";
    case DECISION_RESULT_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
    case DECISION_RESULT_INVALID_FRAME:    return "INVALID_FRAME";
    case DECISION_RESULT_NO_SOLUTION:      return "NO_SOLUTION";
    case DECISION_RESULT_SEARCH_LIMIT:     return "SEARCH_LIMIT";
    case DECISION_RESULT_NUMERIC_ERROR:    return "NUMERIC_ERROR";
    case DECISION_RESULT_WRONG_HALF:       return "WRONG_HALF";
    default:                               return "?";
    }
}

static void solve_with_bias(float bias_mm, const char *label)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionResult result;
    char message[128];

    build_frame(&frame, bias_mm);
    Decision_GetDefaultConfig(&config);
    result = Decision_Solve(&frame, &config, &plan);

    (void)snprintf(message, sizeof(message),
                   "%s: expected OK, got %s", label, result_name(result));
    check(result == DECISION_RESULT_OK, message);
    if (result != DECISION_RESULT_OK) {
        return;
    }

    (void)snprintf(message, sizeof(message),
                   "%s: expected 4 moves, got %u",
                   label, (unsigned)plan.move_count);
    check(plan.move_count == 4U, message);
}

/* A perfect cut has to solve, or nothing below means anything. */
static void test_exact_pieces(void)
{
    solve_with_bias(0.0f, "exact pieces");
}

/* The point of the whole exercise: pieces whose mating edges disagree by
   millimetres, which is what scissors produce. A 3 mm bias makes the shared
   cut edges differ by 3 mm and 6 mm across the four pieces. */
static void test_hand_cut_pieces(void)
{
    solve_with_bias(1.5f, "1.5 mm cutting error");
    solve_with_bias(3.0f, "3 mm cutting error");
}

/* Pieces that cannot tile any rectangle must still be refused, otherwise the
   loosened tolerances have simply stopped checking anything. */
static void test_rejects_impossible_layout(void)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionResult result;

    /* Four 20x75 strips. Their areas sum to 6000 mm², exactly the area of a
       valid 100x60 target, so the set can only be refused on geometry and not
       for being obviously too small. Four such strips tile just three
       rectangles - 80x75, 40x150 and 20x300 - and every one of them is outside
       the task's 9x5 to 12x9 cm range, while every partial arrangement leaves
       far too much of its bounding rectangle empty. Every edge is at least the
       2 cm the task requires, so the set is legal input. */
    {
        const float strip[8] = {0.0f, 0.0f, 20.0f, 0.0f, 20.0f, 75.0f,
                                0.0f, 75.0f};
        uint8_t i;

        (void)memset(&frame, 0, sizeof(frame));
        frame.seq = 1U;
        frame.piece_count = 4U;
        for (i = 0U; i < 4U; ++i) {
            set_piece(&frame.pieces[i], (uint8_t)(i + 1U), 4U, strip);
        }
        scatter(&frame.pieces[0], 11.0f, 30.0f, 20.0f);
        scatter(&frame.pieces[1], -47.0f, 95.0f, 60.0f);
        scatter(&frame.pieces[2], 68.0f, 40.0f, 120.0f);
        scatter(&frame.pieces[3], -23.0f, 110.0f, 165.0f);
    }
    Decision_GetDefaultConfig(&config);
    result = Decision_Solve(&frame, &config, &plan);
    check(result != DECISION_RESULT_OK,
          "mismatched pieces: expected refusal, got OK");
}

/* The overlap depth tolerance must not have gone so far that the solver can
   "solve" a layout by stacking pieces. Checked on the plan it produces rather
   than by feeding it an unsolvable set: the place points it chose must be far
   enough apart that no two pieces occupy the same spot. Two 50x35 quadrants
   sitting on top of each other would put their place points within a few
   millimetres, whereas a real tiling separates them by tens. */
static void test_plan_places_pieces_apart(float bias_mm, const char *label)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    uint8_t i;
    uint8_t j;

    build_frame(&frame, bias_mm);
    Decision_GetDefaultConfig(&config);
    if (Decision_Solve(&frame, &config, &plan) != DECISION_RESULT_OK) {
        return;
    }

    for (i = 0U; i < plan.move_count; ++i) {
        for (j = (uint8_t)(i + 1U); j < plan.move_count; ++j) {
            const float dx = plan.moves[i].place.x_mm -
                             plan.moves[j].place.x_mm;
            const float dy = plan.moves[i].place.y_mm -
                             plan.moves[j].place.y_mm;
            const float distance = sqrtf(dx * dx + dy * dy);
            char message[128];

            (void)snprintf(message, sizeof(message),
                           "%s: place points %u and %u only %.1f mm apart",
                           label, (unsigned)i, (unsigned)j, (double)distance);
            check(distance > 10.0f, message);
        }
    }
}

/* A frame the vision side should never send, but which must be reported as bad
   input rather than as "no solution", because the two beep out differently. */
static void test_rejects_degenerate_frame(void)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;

    build_frame(&frame, 0.0f);
    frame.pieces[1].vertex_count = 2U;
    Decision_GetDefaultConfig(&config);
    check(Decision_Solve(&frame, &config, &plan) ==
              DECISION_RESULT_INVALID_FRAME,
          "two-vertex piece: expected INVALID_FRAME");

    build_frame(&frame, 0.0f);
    frame.pieces[2].id = frame.pieces[0].id;
    check(Decision_Solve(&frame, &config, &plan) ==
              DECISION_RESULT_INVALID_FRAME,
          "duplicate id: expected INVALID_FRAME");
}

/* The task allows 9x5 cm at the small end, so the solver must not have been
   tuned to only the one rectangle above. */
static void test_small_rectangle(void)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    const float left[8] = {0.0f, 0.0f, 45.0f, 0.0f, 45.0f, 50.0f, 0.0f, 50.0f};
    const float right[8] = {0.0f, 0.0f, 45.0f, 0.0f, 45.0f, 50.0f, 0.0f, 50.0f};

    (void)memset(&frame, 0, sizeof(frame));
    frame.seq = 2U;
    frame.piece_count = 2U;
    set_piece(&frame.pieces[0], 1U, 4U, left);
    set_piece(&frame.pieces[1], 2U, 4U, right);
    scatter(&frame.pieces[0], 23.0f, 25.0f, 30.0f);
    scatter(&frame.pieces[1], -47.0f, 40.0f, 130.0f);

    Decision_GetDefaultConfig(&config);
    check(Decision_Solve(&frame, &config, &plan) == DECISION_RESULT_OK,
          "90x50 mm rectangle: expected OK");
}

/* The four axis-aligned quadrants above are an easy case: every piece has a
   right angle at each corner and edges of matching nominal length, so the search
   has many ways to fit them. The task's own pieces (figure 2) are cut by slanted
   lines into triangles and quadrilaterals, where a given edge matches only one
   other edge and the angles have to agree too. That is the case worth pinning
   down, since it is far more sensitive to cutting error.
 *
 * A 100x60 mm rectangle crossed by two slanted cuts, one from (20,0) to (80,60)
 * and one from (0,20) to (100,40), which meet at (50,30). The four regions really
 * do tile the rectangle, and each keeps at least one edge on the outer boundary as
 * the task guarantees.
 *
 * These pieces also meet at T-junctions, which is the harder half of the case.
 * Neither cut ends where the other does, so along each cut a long edge of one
 * piece faces two shorter edges of two others - and the vision end reports that
 * long edge as one edge, because its two halves are collinear and look like a
 * single side of the contour. A matcher that only joins edges of equal length
 * cannot represent this layout at all and refuses a set that plainly tiles. */
static void build_slanted_frame(DecisionVisionFrame *frame, float error_mm)
{
    const float e = error_mm;
    /* Where the two cuts cross. */
    const float ix = 50.0f;
    const float iy = 30.0f;
    const float a[8] = {0.0f, 0.0f, 20.0f + e, 0.0f, ix, iy, 0.0f, 20.0f - e};
    const float b[8] = {20.0f, 0.0f, 100.0f, 0.0f, 100.0f, 40.0f + e, ix + e, iy};
    const float c[8] = {0.0f, 20.0f, ix, iy - e, 80.0f - e, 60.0f, 0.0f, 60.0f};
    const float d[8] = {ix, iy, 100.0f, 40.0f, 100.0f, 60.0f, 80.0f + e, 60.0f};

    (void)memset(frame, 0, sizeof(*frame));
    frame->seq = 3U;
    frame->piece_count = 4U;
    set_piece(&frame->pieces[0], 1U, 4U, a);
    set_piece(&frame->pieces[1], 2U, 4U, b);
    set_piece(&frame->pieces[2], 3U, 4U, c);
    set_piece(&frame->pieces[3], 4U, 4U, d);

    scatter(&frame->pieces[0], 13.0f, 22.0f, 28.0f);
    scatter(&frame->pieces[1], -52.0f, 40.0f, 88.0f);
    scatter(&frame->pieces[2], 67.0f, 78.0f, 62.0f);
    scatter(&frame->pieces[3], -29.0f, 50.0f, 150.0f);
}

/* The task's own pieces must be solved, not refused, at the cutting accuracy a
   hand-cut set actually has. A refusal here is what the device reports as a
   diagnostic beep instead of assembling anything. */
static void test_slanted_pieces_are_handled(void)
{
    const float errors[3] = {0.0f, 1.5f, 3.0f};
    uint8_t index;

    for (index = 0U; index < 3U; ++index) {
        DecisionVisionFrame frame;
        DecisionConfig config;
        DecisionPlan plan;
        DecisionResult result;
        char message[128];

        build_slanted_frame(&frame, errors[index]);
        Decision_GetDefaultConfig(&config);
        result = Decision_Solve(&frame, &config, &plan);

        (void)snprintf(message, sizeof(message),
                       "slanted pieces at %.1f mm: expected OK, got %s",
                       (double)errors[index], result_name(result));
        check(result == DECISION_RESULT_OK, message);
    }
}

/* Shifts every piece along x and returns what the solver made of the frame,
   along with the centroid of the place points it chose. */
static DecisionResult solve_shifted(float piece_offset_x_mm,
                                    DecisionPoint *centroid)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionResult result;
    uint8_t index;

    build_frame(&frame, 0.0f);
    for (index = 0U; index < frame.piece_count; ++index) {
        uint8_t vertex;

        frame.pieces[index].center.x_mm += piece_offset_x_mm;
        for (vertex = 0U; vertex < frame.pieces[index].vertex_count; ++vertex) {
            frame.pieces[index].vertices[vertex].x_mm += piece_offset_x_mm;
        }
    }

    Decision_GetDefaultConfig(&config);
    result = Decision_Solve(&frame, &config, &plan);

    centroid->x_mm = 0.0f;
    centroid->y_mm = 0.0f;
    if (result != DECISION_RESULT_OK || plan.move_count == 0U) {
        return result;
    }
    for (index = 0U; index < plan.move_count; ++index) {
        centroid->x_mm += plan.moves[index].place.x_mm;
        centroid->y_mm += plan.moves[index].place.y_mm;
    }
    centroid->x_mm /= (float)plan.move_count;
    centroid->y_mm /= (float)plan.move_count;
    return result;
}

/* Which half is which is not inferred any more: both ends of the link use the
   vision end's A4 frame - landscape sheet, top-left origin, +X right, +Y down -
   so the pick half is x < 148.5 and the place half is x > 148.5, full stop. Two
   things follow, and this pins down both: the stated target is used exactly as
   given rather than mirrored to wherever the pieces are not, and a frame whose
   pieces are on the assembly side is refused outright instead of being solved
   into a plan that assembles on top of them. */
static void test_halves_are_fixed_by_the_shared_frame(void)
{
    const float divider = DECISION_PAPER_DIVIDER_X_MM;
    DecisionConfig config;
    DecisionPoint centroid;
    DecisionResult result;
    char message[128];

    Decision_GetDefaultConfig(&config);
    check(config.target_center.x_mm > divider,
          "default target: not in the place half");

    /* build_frame lays the pieces out in the pick half, so this is the nominal
       case: it solves, and the assembly stays where the config asked for it. */
    result = solve_shifted(0.0f, &centroid);
    (void)snprintf(message, sizeof(message),
                   "pieces in the pick half: expected OK, got %s",
                   result_name(result));
    check(result == DECISION_RESULT_OK, message);
    check(centroid.x_mm > divider,
          "pieces in the pick half: assembly did not stay in the place half");

    /* Same pieces pushed across the midline. Nothing on the device can fix that,
       so it has to come back as WRONG_HALF rather than as a plan or as one of
       the geometry failures. */
    result = solve_shifted(140.0f, &centroid);
    (void)snprintf(message, sizeof(message),
                   "pieces in the place half: expected WRONG_HALF, got %s",
                   result_name(result));
    check(result == DECISION_RESULT_WRONG_HALF, message);
}

static void test_trajectory_stops_above_pick_and_place(void)
{
    DecisionMove move;
    TrajectoryPose current;
    TrajectoryLimits limits;
    TrajectoryRequest request;
    const TrajectoryPose *approach_end;
    const TrajectoryPose *transfer_start;
    const TrajectoryPose *transfer_end;

    (void)memset(&move, 0, sizeof(move));
    (void)memset(&current, 0, sizeof(current));
    (void)memset(&limits, 0, sizeof(limits));
    move.pick.x_mm = 60.0f;
    move.pick.y_mm = 40.0f;
    move.pick.z_mm = -45.0f;
    move.pick_above = move.pick;
    move.pick_above.z_mm = 0.0f;
    move.place.x_mm = 210.0f;
    move.place.y_mm = 100.0f;
    move.place.z_mm = -45.0f;
    move.place.yaw_deg = 30.0f;
    move.place_above = move.place;
    move.place_above.z_mm = 0.0f;
    current.x_mm = 78.0f;
    current.y_mm = -50.0f;
    current.z_mm = 0.0f;
    limits.max_linear_velocity_mm_s = 120.0f;
    limits.max_linear_acceleration_mm_s2 = 300.0f;
    limits.max_yaw_velocity_deg_s = 90.0f;
    limits.max_yaw_acceleration_deg_s2 = 180.0f;

    check(Decision_BuildTrajectoryRequest(&move, &current, &limits,
                                          &request) != 0U,
          "above-only trajectory request was rejected");
    approach_end = &request.approach.points[request.approach.point_count - 1U];
    transfer_start = &request.transfer.points[0];
    transfer_end = &request.transfer.points[request.transfer.point_count - 1U];
    check(fabsf(approach_end->z_mm - move.pick_above.z_mm) < 0.001f,
          "approach descended before the stepper axes were confirmed");
    check(fabsf(transfer_start->z_mm - move.pick_above.z_mm) < 0.001f,
          "transfer started from the lowered pick pose");
    check(fabsf(transfer_end->z_mm - move.place_above.z_mm) < 0.001f,
          "transfer descended before the place axes were confirmed");
}

static void test_pick_uses_vision_center(void)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionPoint expected_center;
    uint8_t piece_id;
    uint8_t move_index;
    uint8_t found = 0U;

    build_frame(&frame, 0.0f);
    frame.pieces[0].center.x_mm += 3.0f;
    frame.pieces[0].center.y_mm += 2.0f;
    expected_center = frame.pieces[0].center;
    piece_id = frame.pieces[0].id;
    Decision_GetDefaultConfig(&config);

    check(Decision_Solve(&frame, &config, &plan) == DECISION_RESULT_OK,
          "vision-center frame did not solve");
    for (move_index = 0U; move_index < plan.move_count; ++move_index) {
        if (plan.moves[move_index].piece_id == piece_id) {
            found = 1U;
            check(fabsf(plan.moves[move_index].pick.x_mm -
                        expected_center.x_mm) < 0.001f &&
                      fabsf(plan.moves[move_index].pick.y_mm -
                            expected_center.y_mm) < 0.001f,
                  "pick did not use the center supplied by vision");
            break;
        }
    }
    check(found != 0U, "vision-center piece was missing from the plan");
}

static void test_place_centers_include_safety_clearance(void)
{
    const float quadrant_center_radius_mm =
        sqrtf(25.0f * 25.0f + 17.5f * 17.5f);
    const float expected_radius_mm =
        quadrant_center_radius_mm + DECISION_ASSEMBLY_CLEARANCE_MM;
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    uint8_t move_index;

    build_frame(&frame, 0.0f);
    Decision_GetDefaultConfig(&config);
    check(Decision_Solve(&frame, &config, &plan) == DECISION_RESULT_OK,
          "clearance frame did not solve");

    for (move_index = 0U; move_index < plan.move_count; ++move_index) {
        const float dx = plan.moves[move_index].place.x_mm -
                         config.target_center.x_mm;
        const float dy = plan.moves[move_index].place.y_mm -
                         config.target_center.y_mm;
        const float radius_mm = sqrtf(dx * dx + dy * dy);
        char message[128];

        (void)snprintf(message, sizeof(message),
                       "move %u clearance radius %.3f, expected %.3f",
                       (unsigned)move_index,
                       (double)radius_mm,
                       (double)expected_radius_mm);
        check(fabsf(radius_mm - expected_radius_mm) < 0.01f, message);
    }
}

static void add_card_edge_event(DecisionCardPieceFeatures *features,
                                uint8_t edge_index,
                                uint8_t position_q8,
                                DecisionCardColor color)
{
    DecisionCardEdgeEvent *event =
        &features->edge_events[features->edge_event_count++];

    (void)memset(event, 0, sizeof(*event));
    event->edge_index = edge_index;
    event->position_q8 = position_q8;
    event->color = color;
    event->width_q4_mm = 8U;
    event->confidence = 255U;
}

static const DecisionMove *find_move(const DecisionPlan *plan,
                                     uint8_t piece_id)
{
    uint8_t index;

    for (index = 0U; index < plan->move_count; ++index) {
        if (plan->moves[index].piece_id == piece_id) {
            return &plan->moves[index];
        }
    }
    return NULL;
}

static float place_distance(const DecisionPlan *plan,
                            uint8_t left_id,
                            uint8_t right_id)
{
    const DecisionMove *left = find_move(plan, left_id);
    const DecisionMove *right = find_move(plan, right_id);
    float dx;
    float dy;

    if (left == NULL || right == NULL) {
        return 0.0f;
    }
    dx = left->place.x_mm - right->place.x_mm;
    dy = left->place.y_mm - right->place.y_mm;
    return sqrtf(dx * dx + dy * dy);
}

static void reverse_card_piece_input(DecisionPiece *piece,
                                     DecisionCardPieceFeatures *features)
{
    uint8_t left = 0U;
    uint8_t right = (uint8_t)(piece->vertex_count - 1U);
    uint8_t event_index;

    while (left < right) {
        DecisionPoint temporary = piece->vertices[left];
        piece->vertices[left] = piece->vertices[right];
        piece->vertices[right] = temporary;
        ++left;
        --right;
    }
    for (event_index = 0U; event_index < features->edge_event_count;
         ++event_index) {
        DecisionCardEdgeEvent *event = &features->edge_events[event_index];
        event->edge_index = (uint8_t)(
            (2U * piece->vertex_count - 2U - event->edge_index) %
            piece->vertex_count);
        event->position_q8 = (uint8_t)(255U - event->position_q8);
    }
}

/* Four equal rectangles have many geometry-perfect arrangements. These cut
   events describe the four real seams, so the opposite pieces (1,3) and (2,4)
   must remain diagonally separated in the selected card layout. */
static void test_card_features_break_geometric_ties(void)
{
    DecisionCardFrame card;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionResult result;
    float diagonal_13;
    float diagonal_24;
    uint8_t piece_index;

    (void)memset(&card, 0, sizeof(card));
    build_frame(&card.vision, 0.0f);
    card.layout_id = 0x12345678U;
    card.piece_count = card.vision.piece_count;
    card.pieces[0].piece_id = 1U;
    card.pieces[1].piece_id = 2U;
    card.pieces[2].piece_id = 3U;
    card.pieces[3].piece_id = 4U;

    add_card_edge_event(&card.pieces[0], 1U, 51U, DECISION_CARD_COLOR_RED);
    add_card_edge_event(&card.pieces[1], 3U, 204U, DECISION_CARD_COLOR_RED);
    add_card_edge_event(&card.pieces[0], 2U, 89U, DECISION_CARD_COLOR_BLACK);
    add_card_edge_event(&card.pieces[3], 0U, 166U, DECISION_CARD_COLOR_BLACK);
    add_card_edge_event(&card.pieces[1], 2U, 115U, DECISION_CARD_COLOR_RED);
    add_card_edge_event(&card.pieces[2], 0U, 140U, DECISION_CARD_COLOR_RED);
    add_card_edge_event(&card.pieces[3], 1U, 153U, DECISION_CARD_COLOR_BLACK);
    add_card_edge_event(&card.pieces[2], 3U, 102U, DECISION_CARD_COLOR_BLACK);

    Decision_GetDefaultConfig(&config);
    result = Decision_SolveCard(&card, &config, &plan);
    check(result == DECISION_RESULT_OK,
          "card tie-break: expected a card solution");
    if (result != DECISION_RESULT_OK) {
        return;
    }
    {
        char message[96];
        (void)snprintf(message, sizeof(message),
                       "card tie-break visited %lu search nodes",
                       (unsigned long)plan.search_nodes);
        check(plan.search_nodes < 100U, message);
    }

    diagonal_13 = place_distance(&plan, 1U, 3U);
    diagonal_24 = place_distance(&plan, 2U, 4U);
    check(diagonal_13 > place_distance(&plan, 1U, 2U) + 5.0f &&
              diagonal_13 > place_distance(&plan, 1U, 4U) + 5.0f,
          "card tie-break: pieces 1 and 3 were not diagonal");
    check(diagonal_24 > place_distance(&plan, 2U, 1U) + 5.0f &&
              diagonal_24 > place_distance(&plan, 2U, 3U) + 5.0f,
          "card tie-break: pieces 2 and 4 were not diagonal");

    for (piece_index = 0U; piece_index < card.piece_count; ++piece_index) {
        reverse_card_piece_input(&card.vision.pieces[piece_index],
                                 &card.pieces[piece_index]);
    }
    result = Decision_SolveCard(&card, &config, &plan);
    check(result == DECISION_RESULT_OK,
          "card tie-break: reversed vertex traversal did not solve");
    if (result == DECISION_RESULT_OK) {
        check(place_distance(&plan, 1U, 3U) >
                  place_distance(&plan, 1U, 2U) + 5.0f,
              "card tie-break: reversed traversal changed the layout");
    }
}

static void test_card_without_pattern_is_ambiguous(void)
{
    DecisionCardFrame card;
    DecisionConfig config;
    DecisionPlan plan;

    (void)memset(&card, 0, sizeof(card));
    build_frame(&card.vision, 0.0f);
    card.piece_count = card.vision.piece_count;
    card.pieces[0].piece_id = 1U;
    card.pieces[1].piece_id = 2U;
    card.pieces[2].piece_id = 3U;
    card.pieces[3].piece_id = 4U;
    Decision_GetDefaultConfig(&config);

    check(Decision_SolveCard(&card, &config, &plan) ==
              DECISION_RESULT_CARD_AMBIGUOUS,
          "card strategy accepted a layout without pattern evidence");
}

static void test_strategy_dispatch_requires_card_data(void)
{
    DecisionVisionFrame vision;
    DecisionConfig config;
    DecisionPlan plan;

    build_frame(&vision, 0.0f);
    Decision_GetDefaultConfig(&config);
    check(Decision_SolveStrategy(DECISION_STRATEGY_GEOMETRIC,
                                 &vision, NULL, &config, &plan) ==
              DECISION_RESULT_OK,
          "geometric strategy unexpectedly required card data");
    check(Decision_SolveStrategy(DECISION_STRATEGY_CARD_PATTERN,
                                 &vision, NULL, &config, &plan) ==
              DECISION_RESULT_INVALID_ARGUMENT,
          "card strategy accepted a request without card data");
}

int main(void)
{
    test_exact_pieces();
    test_hand_cut_pieces();
    test_small_rectangle();
    test_rejects_impossible_layout();
    test_plan_places_pieces_apart(0.0f, "exact pieces");
    test_plan_places_pieces_apart(3.0f, "3 mm cutting error");
    test_rejects_degenerate_frame();
    test_slanted_pieces_are_handled();
    test_halves_are_fixed_by_the_shared_frame();
    test_trajectory_stops_above_pick_and_place();
    test_pick_uses_vision_center();
    test_place_centers_include_safety_clearance();
    test_card_features_break_geometric_ties();
    test_card_without_pattern_is_ambiguous();
    test_strategy_dispatch_requires_card_data();

    if (failures != 0) {
        printf("%d decision test(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("decision tests passed\n");
    return EXIT_SUCCESS;
}

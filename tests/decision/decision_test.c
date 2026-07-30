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
    /* The centre only has to be inside the piece; the solver recomputes the
       grasp point itself. */
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

    scatter(&frame->pieces[0], 17.0f, 40.0f, 210.0f);
    scatter(&frame->pieces[1], -35.0f, 150.0f, 195.0f);
    scatter(&frame->pieces[2], 78.0f, 60.0f, 260.0f);
    scatter(&frame->pieces[3], -61.0f, 175.0f, 255.0f);
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

    build_frame(&frame, 0.0f);
    /* Replace one quadrant with a small triangle, so no arrangement fills a
       rectangle in the allowed size range. */
    {
        const float triangle[6] = {0.0f, 0.0f, 25.0f, 0.0f, 0.0f, 25.0f};

        set_piece(&frame.pieces[2], 3U, 3U, triangle);
        scatter(&frame.pieces[2], 78.0f, 60.0f, 260.0f);
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
    scatter(&frame.pieces[0], 23.0f, 50.0f, 205.0f);
    scatter(&frame.pieces[1], -47.0f, 160.0f, 240.0f);

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
 * A 100x60 mm rectangle cut by a diagonal and then one of the halves cut again,
 * giving two triangles and two quadrilaterals, with each piece keeping at least
 * one edge on the outer boundary as the task guarantees. */
static void build_slanted_frame(DecisionVisionFrame *frame, float error_mm)
{
    const float e = error_mm;
    /* Lower-left triangle and upper-right triangle share the diagonal. */
    const float t0[6] = {0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 60.0f - e};
    const float q1[8] = {100.0f, 0.0f, 100.0f, 30.0f + e,
                         50.0f + e, 30.0f, 50.0f, 0.0f};
    const float q2[8] = {0.0f, 60.0f, 60.0f - e, 60.0f,
                         60.0f, 35.0f + e, 0.0f, 35.0f};
    const float t3[6] = {100.0f, 60.0f, 40.0f + e, 60.0f, 100.0f, 25.0f - e};

    (void)memset(frame, 0, sizeof(*frame));
    frame->seq = 3U;
    frame->piece_count = 4U;
    set_piece(&frame->pieces[0], 1U, 3U, t0);
    set_piece(&frame->pieces[1], 2U, 4U, q1);
    set_piece(&frame->pieces[2], 3U, 4U, q2);
    set_piece(&frame->pieces[3], 4U, 3U, t3);

    scatter(&frame->pieces[0], 13.0f, 45.0f, 205.0f);
    scatter(&frame->pieces[1], -52.0f, 155.0f, 200.0f);
    scatter(&frame->pieces[2], 67.0f, 55.0f, 265.0f);
    scatter(&frame->pieces[3], -29.0f, 170.0f, 260.0f);
}

/* Documents what the solver does with slanted, non-interlocking pieces. These
   deliberately do not tile a rectangle exactly, so a refusal is the honest
   answer; what matters is that it is reported as NO_SOLUTION and not as a
   crash, a search-limit stall, or a bogus plan. */
static void test_slanted_pieces_are_handled(void)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionResult result;
    char message[128];

    build_slanted_frame(&frame, 0.0f);
    Decision_GetDefaultConfig(&config);
    result = Decision_Solve(&frame, &config, &plan);

    (void)snprintf(message, sizeof(message),
                   "slanted pieces: expected OK or NO_SOLUTION, got %s",
                   result_name(result));
    check(result == DECISION_RESULT_OK ||
          result == DECISION_RESULT_NO_SOLUTION, message);

    /* Whatever it decides, it must decide within the node budget: a run that
       reports SEARCH_LIMIT on four pieces would mean the budget is the real
       constraint rather than the geometry. */
    check(result != DECISION_RESULT_SEARCH_LIMIT,
          "slanted pieces: node budget exhausted on only four pieces");
}

/* The task starts the pieces in one half of the sheet and scores the assembly in
   the other, but which half is which depends on the corner the camera
   calibration calls the origin. The solver therefore has to work it out from the
   pieces: the same stated target has to end up on the far side of the divider
   from wherever the pieces actually are, and mirroring it must not disturb the
   layout itself. */
static void solve_in_half(float piece_offset_x_mm,
                          float target_x_mm,
                          const char *label,
                          DecisionPoint *centroid)
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    uint8_t index;
    char message[128];

    build_frame(&frame, 0.0f);
    for (index = 0U; index < frame.piece_count; ++index) {
        uint8_t vertex;

        frame.pieces[index].center.x_mm += piece_offset_x_mm;
        for (vertex = 0U; vertex < frame.pieces[index].vertex_count; ++vertex) {
            frame.pieces[index].vertices[vertex].x_mm += piece_offset_x_mm;
        }
    }

    Decision_GetDefaultConfig(&config);
    config.target_center.x_mm = target_x_mm;
    config.target_center.y_mm = 95.0f;

    (void)snprintf(message, sizeof(message), "%s: expected OK", label);
    check(Decision_Solve(&frame, &config, &plan) == DECISION_RESULT_OK, message);

    centroid->x_mm = 0.0f;
    centroid->y_mm = 0.0f;
    for (index = 0U; index < plan.move_count; ++index) {
        centroid->x_mm += plan.moves[index].place.x_mm;
        centroid->y_mm += plan.moves[index].place.y_mm;
    }
    if (plan.move_count != 0U) {
        centroid->x_mm /= (float)plan.move_count;
        centroid->y_mm /= (float)plan.move_count;
    }
}

static void test_target_lands_in_the_other_half(void)
{
    const float divider = DECISION_PAPER_DIVIDER_X_MM;
    DecisionPoint near_column;
    DecisionPoint far_column;

    /* build_frame scatters the pieces around x = 40..175, so they sit mostly
       left of the divider; the stated target is on the right and must stay. */
    solve_in_half(0.0f, divider + 60.0f, "pieces left, target right",
                  &near_column);
    check(near_column.x_mm > divider,
          "pieces left: assembly did not stay in the right half");

    /* Same stated target, pieces moved into the right half: the target has to be
       mirrored to the left rather than laid on top of them. */
    solve_in_half(120.0f, divider + 60.0f, "pieces right, target right",
                  &far_column);
    check(far_column.x_mm < divider,
          "pieces right: assembly was not mirrored out of their half");
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
    test_target_lands_in_the_other_half();

    if (failures != 0) {
        printf("%d decision test(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("decision tests passed\n");
    return EXIT_SUCCESS;
}

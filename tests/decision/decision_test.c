#include "decision.h"
#include "decision_template.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f

#define ASSERT_TRUE(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            fprintf(stderr, "%s:%d assertion failed: %s\n",                    \
                    __FILE__, __LINE__, #condition);                              \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

#define ASSERT_EQ_INT(expected, actual)                                          \
    do {                                                                         \
        int expected_value = (int)(expected);                                     \
        int actual_value = (int)(actual);                                         \
        if (expected_value != actual_value) {                                     \
            fprintf(stderr, "%s:%d expected %d, got %d\n",                     \
                    __FILE__, __LINE__, expected_value, actual_value);             \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

#define ASSERT_NEAR(expected, actual, tolerance)                                 \
    do {                                                                         \
        float expected_value = (float)(expected);                                 \
        float actual_value = (float)(actual);                                     \
        float tolerance_value = (float)(tolerance);                               \
        if (fabsf(actual_value - expected_value) > tolerance_value) {             \
            fprintf(stderr, "%s:%d expected %.7g, got %.7g\n",                 \
                    __FILE__, __LINE__, expected_value, actual_value);             \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

static const float ObservationAnglesDeg[DECISION_MAX_PIECES] = {
    -18.0f, 71.0f, 143.0f, -96.0f
};

static const DecisionPoint ObservationTranslations[DECISION_MAX_PIECES] = {
    {30.0f, 20.0f},
    {155.0f, 30.0f},
    {45.0f, 105.0f},
    {165.0f, 115.0f}
};

static const uint8_t ObservationIds[DECISION_MAX_PIECES] = {41U, 7U, 99U, 12U};

static float normalize_degrees(float angle_deg)
{
    float result = fmodf(angle_deg + 180.0f, 360.0f);
    if (result < 0.0f) {
        result += 360.0f;
    }
    return result - 180.0f;
}

static float point_distance(DecisionPoint left, DecisionPoint right)
{
    return hypotf(left.x_mm - right.x_mm, left.y_mm - right.y_mm);
}

static DecisionPoint rotate_translate(DecisionPoint point,
                                      float angle_deg,
                                      DecisionPoint translation)
{
    float angle_rad = angle_deg * TEST_PI / 180.0f;
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    DecisionPoint result;

    result.x_mm = cosine * point.x_mm - sine * point.y_mm + translation.x_mm;
    result.y_mm = sine * point.x_mm + cosine * point.y_mm + translation.y_mm;
    return result;
}

static void build_frame(const uint8_t template_order[DECISION_MAX_PIECES],
                        DecisionVisionFrame *frame)
{
    DecisionFixedLayout canonical;
    DecisionPoint canonical_center = {50.0f, 30.0f};
    uint8_t observation_index;

    DecisionTemplate_GetFigure2Layout(canonical_center, &canonical);
    (void)memset(frame, 0, sizeof(*frame));
    frame->seq = 20260729U;
    frame->piece_count = DECISION_MAX_PIECES;

    for (observation_index = 0U; observation_index < DECISION_MAX_PIECES;
         ++observation_index) {
        uint8_t template_index = template_order[observation_index];
        const DecisionFixedPiece *source = &canonical.pieces[template_index];
        DecisionPiece *piece = &frame->pieces[observation_index];
        uint8_t vertex_index;

        piece->id = ObservationIds[observation_index];
        piece->vertex_count = source->vertex_count;
        for (vertex_index = 0U; vertex_index < piece->vertex_count;
             ++vertex_index) {
            piece->vertices[vertex_index] = rotate_translate(
                source->target_vertices[vertex_index],
                ObservationAnglesDeg[observation_index],
                ObservationTranslations[observation_index]);
            piece->center.x_mm += piece->vertices[vertex_index].x_mm;
            piece->center.y_mm += piece->vertices[vertex_index].y_mm;
        }
        piece->center.x_mm /= (float)piece->vertex_count;
        piece->center.y_mm /= (float)piece->vertex_count;
    }
}

static DecisionPoint apply_move(DecisionPoint point, const DecisionMove *move)
{
    DecisionPoint relative = {point.x_mm - move->pick.x_mm,
                              point.y_mm - move->pick.y_mm};
    DecisionPoint result;
    float angle_rad = move->place.yaw_deg * TEST_PI / 180.0f;
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);

    result.x_mm = cosine * relative.x_mm - sine * relative.y_mm +
                  move->place.x_mm;
    result.y_mm = sine * relative.x_mm + cosine * relative.y_mm +
                  move->place.y_mm;
    return result;
}

static void build_final_vertices(
    const DecisionVisionFrame *frame,
    const DecisionPlan *plan,
    DecisionPoint final_vertices[DECISION_MAX_PIECES][DECISION_MAX_VERTICES])
{
    uint8_t piece_index;

    for (piece_index = 0U; piece_index < frame->piece_count; ++piece_index) {
        uint8_t vertex_index;
        ASSERT_EQ_INT(frame->pieces[piece_index].id,
                      plan->moves[piece_index].piece_id);
        for (vertex_index = 0U;
             vertex_index < frame->pieces[piece_index].vertex_count;
             ++vertex_index) {
            final_vertices[piece_index][vertex_index] = apply_move(
                frame->pieces[piece_index].vertices[vertex_index],
                &plan->moves[piece_index]);
        }
    }
}

static void test_fixed_geometry_matching(void)
{
    static const uint8_t Order[DECISION_MAX_PIECES] = {1U, 3U, 0U, 2U};
    DecisionVisionFrame frame;
    DecisionFixedLayout layout;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionPoint target_center = {105.0f, 220.0f};
    DecisionPoint final_vertices[DECISION_MAX_PIECES][DECISION_MAX_VERTICES];
    uint8_t piece_index;

    build_frame(Order, &frame);
    DecisionTemplate_GetFigure2Layout(target_center, &layout);
    Decision_GetDefaultConfig(&config);
    ASSERT_EQ_INT(DECISION_RESULT_OK,
                  Decision_SolveFixed(&frame, &layout, &config, &plan));
    ASSERT_EQ_INT(DECISION_MAX_PIECES, plan.move_count);
    build_final_vertices(&frame, &plan, final_vertices);

    for (piece_index = 0U; piece_index < DECISION_MAX_PIECES; ++piece_index) {
        const DecisionFixedPiece *target = &layout.pieces[Order[piece_index]];
        uint8_t vertex_index;

        ASSERT_EQ_INT(ObservationIds[piece_index], plan.moves[piece_index].piece_id);
        ASSERT_NEAR(normalize_degrees(-ObservationAnglesDeg[piece_index]),
                    plan.moves[piece_index].place.yaw_deg,
                    0.02f);
        for (vertex_index = 0U; vertex_index < target->vertex_count;
             ++vertex_index) {
            uint8_t candidate_index;
            uint8_t found = 0U;

            ASSERT_TRUE(target->target_vertices[vertex_index].y_mm >= 148.5f);
            for (candidate_index = 0U; candidate_index < target->vertex_count;
                 ++candidate_index) {
                if (point_distance(target->target_vertices[vertex_index],
                                   final_vertices[piece_index][candidate_index]) <
                    0.05f) {
                    found = 1U;
                    break;
                }
            }
            ASSERT_TRUE(found != 0U);
        }
    }
}

static float distance_to_line(DecisionPoint point,
                              DecisionPoint start,
                              DecisionPoint end)
{
    float dx = end.x_mm - start.x_mm;
    float dy = end.y_mm - start.y_mm;
    return fabsf(dx * (point.y_mm - start.y_mm) -
                 dy * (point.x_mm - start.x_mm)) / hypotf(dx, dy);
}

static void sort_three(float values[3])
{
    uint8_t index;
    for (index = 1U; index < 3U; ++index) {
        float value = values[index];
        uint8_t insertion = index;
        while (insertion > 0U && values[insertion - 1U] > value) {
            values[insertion] = values[insertion - 1U];
            --insertion;
        }
        values[insertion] = value;
    }
}

static void verify_general_solution(
    const uint8_t order[DECISION_MAX_PIECES])
{
    DecisionVisionFrame frame;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionPoint final_vertices[DECISION_MAX_PIECES][DECISION_MAX_VERTICES];
    float min_x = 1.0e9f;
    float max_x = -1.0e9f;
    float min_y = 1.0e9f;
    float max_y = -1.0e9f;
    uint8_t large_index = 0U;
    uint8_t piece_index;

    build_frame(order, &frame);
    Decision_GetDefaultConfig(&config);
    ASSERT_EQ_INT(DECISION_RESULT_OK,
                  Decision_SolveGeneral(&frame, &config, &plan));
    ASSERT_EQ_INT(DECISION_MAX_PIECES, plan.move_count);
    build_final_vertices(&frame, &plan, final_vertices);

    for (piece_index = 0U; piece_index < DECISION_MAX_PIECES; ++piece_index) {
        uint8_t vertex_index;
        if (order[piece_index] == DECISION_FIGURE2_PIECE_LARGE_TRIANGLE) {
            large_index = piece_index;
        }
        for (vertex_index = 0U;
             vertex_index < frame.pieces[piece_index].vertex_count;
             ++vertex_index) {
            DecisionPoint point = final_vertices[piece_index][vertex_index];
            if (point.x_mm < min_x) min_x = point.x_mm;
            if (point.x_mm > max_x) max_x = point.x_mm;
            if (point.y_mm < min_y) min_y = point.y_mm;
            if (point.y_mm > max_y) max_y = point.y_mm;
        }
    }
    ASSERT_NEAR(55.0f, min_x, 0.05f);
    ASSERT_NEAR(155.0f, max_x, 0.05f);
    ASSERT_NEAR(190.0f, min_y, 0.05f);
    ASSERT_NEAR(250.0f, max_y, 0.05f);

    {
        DecisionPoint long_start = final_vertices[large_index][0];
        DecisionPoint long_end = final_vertices[large_index][1];
        float long_length = point_distance(long_start, long_end);
        float covering_lengths[3];
        uint8_t covering_count = 0U;

        for (piece_index = 0U; piece_index < frame.pieces[large_index].vertex_count;
             ++piece_index) {
            uint8_t next = (uint8_t)((piece_index + 1U) %
                frame.pieces[large_index].vertex_count);
            float length = point_distance(final_vertices[large_index][piece_index],
                                          final_vertices[large_index][next]);
            if (length > long_length) {
                long_length = length;
                long_start = final_vertices[large_index][piece_index];
                long_end = final_vertices[large_index][next];
            }
        }
        ASSERT_NEAR(100.0f, long_length, 0.05f);

        for (piece_index = 0U; piece_index < DECISION_MAX_PIECES; ++piece_index) {
            uint8_t edge_index;
            if (piece_index == large_index) continue;
            for (edge_index = 0U;
                 edge_index < frame.pieces[piece_index].vertex_count;
                 ++edge_index) {
                uint8_t next = (uint8_t)((edge_index + 1U) %
                    frame.pieces[piece_index].vertex_count);
                DecisionPoint start = final_vertices[piece_index][edge_index];
                DecisionPoint end = final_vertices[piece_index][next];

                if (distance_to_line(start, long_start, long_end) < 0.05f &&
                    distance_to_line(end, long_start, long_end) < 0.05f &&
                    covering_count < 3U) {
                    covering_lengths[covering_count] = point_distance(start, end);
                    ++covering_count;
                }
            }
        }
        ASSERT_EQ_INT(3, covering_count);
        sort_three(covering_lengths);
        ASSERT_NEAR(20.0f, covering_lengths[0], 0.05f);
        ASSERT_NEAR(30.0f, covering_lengths[1], 0.05f);
        ASSERT_NEAR(50.0f, covering_lengths[2], 0.05f);
    }
}

static void test_general_permutations(void)
{
    static const uint8_t FirstOrder[DECISION_MAX_PIECES] = {1U, 3U, 0U, 2U};
    static const uint8_t SecondOrder[DECISION_MAX_PIECES] = {2U, 0U, 3U, 1U};
    verify_general_solution(FirstOrder);
    verify_general_solution(SecondOrder);
}

static void test_failure_results(void)
{
    static const uint8_t Order[DECISION_MAX_PIECES] = {1U, 3U, 0U, 2U};
    DecisionVisionFrame frame;
    DecisionFixedLayout layout;
    DecisionConfig config;
    DecisionPlan plan;
    DecisionPoint target_center = {105.0f, 220.0f};

    build_frame(Order, &frame);
    DecisionTemplate_GetFigure2Layout(target_center, &layout);
    Decision_GetDefaultConfig(&config);

    frame.piece_count = 3U;
    ASSERT_EQ_INT(DECISION_RESULT_TEMPLATE_MISMATCH,
                  Decision_SolveFixed(&frame, &layout, &config, &plan));

    frame.piece_count = DECISION_MAX_PIECES;
    frame.pieces[1].id = frame.pieces[0].id;
    ASSERT_EQ_INT(DECISION_RESULT_INVALID_FRAME,
                  Decision_SolveGeneral(&frame, &config, &plan));

    build_frame(Order, &frame);
    frame.piece_count = 1U;
    ASSERT_EQ_INT(DECISION_RESULT_NO_SOLUTION,
                  Decision_SolveGeneral(&frame, &config, &plan));
}

int main(void)
{
    test_fixed_geometry_matching();
    test_general_permutations();
    test_failure_results();
    puts("decision tests passed");
    return EXIT_SUCCESS;
}

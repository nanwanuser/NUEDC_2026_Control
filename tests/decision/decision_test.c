#include "decision.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f

#define ASSERT_TRUE(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            fprintf(stderr, "%s:%d assertion failed: %s\n",                  \
                    __FILE__, __LINE__, #condition);                              \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

#define ASSERT_EQ_INT(expected, actual)                                          \
    do {                                                                         \
        int expected_value = (int)(expected);                                     \
        int actual_value = (int)(actual);                                         \
        if (expected_value != actual_value) {                                     \
            fprintf(stderr, "%s:%d expected %d, got %d\n",                   \
                    __FILE__, __LINE__, expected_value, actual_value);             \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

#define ASSERT_NEAR(expected, actual, tolerance)                                 \
    do {                                                                         \
        float expected_value = (float)(expected);                                 \
        float actual_value = (float)(actual);                                     \
        float tolerance_value = (float)(tolerance);                               \
        if (fabsf(expected_value - actual_value) > tolerance_value) {             \
            fprintf(stderr, "%s:%d expected %.7g, got %.7g (tol %.7g)\n",     \
                    __FILE__, __LINE__, expected_value, actual_value,              \
                    tolerance_value);                                              \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

static DecisionPoint rotate_translate(DecisionPoint point,
                                      float angle_deg,
                                      float tx,
                                      float ty)
{
    float angle_rad = angle_deg * TEST_PI / 180.0f;
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    DecisionPoint result = {
        cosine * point.x_mm - sine * point.y_mm + tx,
        sine * point.x_mm + cosine * point.y_mm + ty
    };
    return result;
}

static void set_rectangle(DecisionPiece *piece,
                          uint8_t id,
                          float min_x,
                          float min_y,
                          float max_x,
                          float max_y)
{
    piece->id = id;
    piece->vertex_count = 4U;
    piece->vertices[0] = (DecisionPoint){min_x, min_y};
    piece->vertices[1] = (DecisionPoint){max_x, min_y};
    piece->vertices[2] = (DecisionPoint){max_x, max_y};
    piece->vertices[3] = (DecisionPoint){min_x, max_y};
    piece->center = (DecisionPoint){0.5f * (min_x + max_x),
                                    0.5f * (min_y + max_y)};
}

static void transform_piece(DecisionPiece *piece,
                            float angle_deg,
                            float tx,
                            float ty)
{
    uint8_t index;

    for (index = 0U; index < piece->vertex_count; ++index) {
        piece->vertices[index] = rotate_translate(piece->vertices[index],
                                                  angle_deg,
                                                  tx,
                                                  ty);
    }
    piece->center = rotate_translate(piece->center, angle_deg, tx, ty);
}

static DecisionPoint apply_move(const DecisionMove *move, DecisionPoint point)
{
    float angle_rad = move->place.yaw_deg * TEST_PI / 180.0f;
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    float relative_x = point.x_mm - move->pick.x_mm;
    float relative_y = point.y_mm - move->pick.y_mm;
    DecisionPoint result = {
        move->place.x_mm + cosine * relative_x - sine * relative_y,
        move->place.y_mm + sine * relative_x + cosine * relative_y
    };
    return result;
}

static void test_fixed_mode_recovers_rotation(void)
{
    DecisionConfig config;
    DecisionVisionFrame frame;
    DecisionFixedLayout layout;
    DecisionPlan plan;
    DecisionResult result;
    DecisionPoint target[4] = {
        {80.0f, 190.0f},
        {120.0f, 190.0f},
        {120.0f, 220.0f},
        {80.0f, 220.0f}
    };
    uint8_t index;

    (void)memset(&frame, 0, sizeof(frame));
    (void)memset(&layout, 0, sizeof(layout));
    Decision_GetDefaultConfig(&config);

    frame.seq = 12U;
    frame.piece_count = 1U;
    frame.pieces[0].id = 7U;
    frame.pieces[0].vertex_count = 4U;
    for (index = 0U; index < 4U; ++index) {
        frame.pieces[0].vertices[index] =
            rotate_translate(target[index], 37.0f, 28.0f, -95.0f);
    }
    frame.pieces[0].center = rotate_translate(
        (DecisionPoint){100.0f, 205.0f}, 37.0f, 28.0f, -95.0f);

    layout.piece_count = 1U;
    layout.pieces[0].id = 7U;
    layout.pieces[0].vertex_count = 4U;
    for (index = 0U; index < 4U; ++index) {
        layout.pieces[0].target_vertices[index] = target[index];
    }

    result = Decision_SolveFixed(&frame, &layout, &config, &plan);
    ASSERT_EQ_INT(DECISION_RESULT_OK, result);
    ASSERT_EQ_INT(1, plan.move_count);
    ASSERT_EQ_INT(7, plan.moves[0].piece_id);
    ASSERT_NEAR(-37.0f, plan.moves[0].place.yaw_deg, 0.01f);
    ASSERT_NEAR(plan.moves[0].pick.x_mm,
                plan.moves[0].transit.x_mm,
                0.001f);
    ASSERT_NEAR(config.transit_z_mm,
                plan.moves[0].transit.z_mm,
                0.001f);

    for (index = 0U; index < 4U; ++index) {
        DecisionPoint placed = apply_move(&plan.moves[0],
                                          frame.pieces[0].vertices[index]);
        ASSERT_NEAR(target[index].x_mm, placed.x_mm, 0.01f);
        ASSERT_NEAR(target[index].y_mm, placed.y_mm, 0.01f);
    }
}

static void test_general_mode_builds_horizontal_rectangle(void)
{
    DecisionConfig config;
    DecisionVisionFrame frame;
    DecisionPlan plan;
    DecisionResult result;
    const float angles[4] = {23.0f, -51.0f, 82.0f, -117.0f};
    const float translations[4][2] = {
        {35.0f, 20.0f},
        {145.0f, 35.0f},
        {55.0f, 100.0f},
        {160.0f, 110.0f}
    };
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    float total_area = 4.0f * 50.0f * 30.0f;
    uint8_t piece_index;

    (void)memset(&frame, 0, sizeof(frame));
    Decision_GetDefaultConfig(&config);
    frame.seq = 23U;
    frame.piece_count = 4U;

    set_rectangle(&frame.pieces[0], 0U, 0.0f, 0.0f, 50.0f, 30.0f);
    set_rectangle(&frame.pieces[1], 1U, 50.0f, 0.0f, 100.0f, 30.0f);
    set_rectangle(&frame.pieces[2], 2U, 0.0f, 30.0f, 50.0f, 60.0f);
    set_rectangle(&frame.pieces[3], 3U, 50.0f, 30.0f, 100.0f, 60.0f);

    for (piece_index = 0U; piece_index < frame.piece_count; ++piece_index) {
        transform_piece(&frame.pieces[piece_index],
                        angles[piece_index],
                        translations[piece_index][0],
                        translations[piece_index][1]);
    }

    result = Decision_SolveGeneral(&frame, &config, &plan);
    ASSERT_EQ_INT(DECISION_RESULT_OK, result);
    ASSERT_EQ_INT(4, plan.move_count);

    for (piece_index = 0U; piece_index < frame.piece_count; ++piece_index) {
        uint8_t vertex_index;
        for (vertex_index = 0U;
             vertex_index < frame.pieces[piece_index].vertex_count;
             ++vertex_index) {
            DecisionPoint placed = apply_move(
                &plan.moves[piece_index],
                frame.pieces[piece_index].vertices[vertex_index]);
            if (placed.x_mm < min_x) min_x = placed.x_mm;
            if (placed.x_mm > max_x) max_x = placed.x_mm;
            if (placed.y_mm < min_y) min_y = placed.y_mm;
            if (placed.y_mm > max_y) max_y = placed.y_mm;
        }
    }

    ASSERT_NEAR(config.target_center.x_mm, 0.5f * (min_x + max_x), 0.05f);
    ASSERT_NEAR(config.target_center.y_mm, 0.5f * (min_y + max_y), 0.05f);
    ASSERT_TRUE((max_x - min_x) >= (max_y - min_y));
    ASSERT_TRUE((max_x - min_x) >= config.min_long_side_mm);
    ASSERT_TRUE((max_x - min_x) <= config.max_long_side_mm);
    ASSERT_TRUE((max_y - min_y) >= config.min_short_side_mm);
    ASSERT_TRUE((max_y - min_y) <= config.max_short_side_mm);
    ASSERT_NEAR(total_area,
                (max_x - min_x) * (max_y - min_y),
                0.5f);
}

static void test_general_mode_rejects_non_rectangle(void)
{
    DecisionConfig config;
    DecisionVisionFrame frame;
    DecisionPlan plan;
    DecisionResult result;

    (void)memset(&frame, 0, sizeof(frame));
    Decision_GetDefaultConfig(&config);
    frame.piece_count = 1U;
    frame.pieces[0].id = 1U;
    frame.pieces[0].vertex_count = 3U;
    frame.pieces[0].vertices[0] = (DecisionPoint){0.0f, 0.0f};
    frame.pieces[0].vertices[1] = (DecisionPoint){40.0f, 0.0f};
    frame.pieces[0].vertices[2] = (DecisionPoint){0.0f, 30.0f};
    frame.pieces[0].center = (DecisionPoint){13.3f, 10.0f};

    result = Decision_SolveGeneral(&frame, &config, &plan);
    ASSERT_EQ_INT(DECISION_RESULT_NO_SOLUTION, result);
}

static void test_general_mode_supports_two_and_three_pieces(void)
{
    const uint8_t piece_counts[2] = {2U, 3U};
    const float split_x[2][4] = {
        {0.0f, 50.0f, 100.0f, 0.0f},
        {0.0f, 30.0f, 60.0f, 100.0f}
    };
    const float angles[3] = {31.0f, -64.0f, 118.0f};
    const float translations[3][2] = {
        {35.0f, 25.0f}, {145.0f, 35.0f}, {90.0f, 110.0f}
    };
    uint8_t case_index;

    for (case_index = 0U; case_index < 2U; ++case_index) {
        DecisionConfig config;
        DecisionVisionFrame frame;
        DecisionPlan plan;
        DecisionResult result;
        float min_x = FLT_MAX;
        float max_x = -FLT_MAX;
        float min_y = FLT_MAX;
        float max_y = -FLT_MAX;
        uint8_t piece_index;

        (void)memset(&frame, 0, sizeof(frame));
        Decision_GetDefaultConfig(&config);
        frame.piece_count = piece_counts[case_index];

        for (piece_index = 0U; piece_index < frame.piece_count; ++piece_index) {
            set_rectangle(&frame.pieces[piece_index],
                          piece_index,
                          split_x[case_index][piece_index],
                          0.0f,
                          split_x[case_index][piece_index + 1U],
                          60.0f);
            transform_piece(&frame.pieces[piece_index],
                            angles[piece_index],
                            translations[piece_index][0],
                            translations[piece_index][1]);
        }

        result = Decision_SolveGeneral(&frame, &config, &plan);
        ASSERT_EQ_INT(DECISION_RESULT_OK, result);
        ASSERT_EQ_INT(frame.piece_count, plan.move_count);

        for (piece_index = 0U; piece_index < frame.piece_count; ++piece_index) {
            uint8_t vertex_index;
            for (vertex_index = 0U; vertex_index < 4U; ++vertex_index) {
                DecisionPoint placed = apply_move(
                    &plan.moves[piece_index],
                    frame.pieces[piece_index].vertices[vertex_index]);
                if (placed.x_mm < min_x) min_x = placed.x_mm;
                if (placed.x_mm > max_x) max_x = placed.x_mm;
                if (placed.y_mm < min_y) min_y = placed.y_mm;
                if (placed.y_mm > max_y) max_y = placed.y_mm;
            }
        }

        ASSERT_NEAR(100.0f, max_x - min_x, 0.1f);
        ASSERT_NEAR(60.0f, max_y - min_y, 0.1f);
        ASSERT_NEAR(config.target_center.x_mm,
                    0.5f * (min_x + max_x),
                    0.05f);
        ASSERT_NEAR(config.target_center.y_mm,
                    0.5f * (min_y + max_y),
                    0.05f);
    }
}

static void test_general_mode_handles_irregular_pieces(void)
{
    DecisionConfig config;
    DecisionVisionFrame frame;
    DecisionPlan plan;
    DecisionResult result;
    const DecisionPoint target[4][3] = {
        {{0.0f, 0.0f}, {100.0f, 0.0f}, {50.0f, 30.0f}},
        {{100.0f, 0.0f}, {100.0f, 60.0f}, {50.0f, 30.0f}},
        {{100.0f, 60.0f}, {0.0f, 60.0f}, {50.0f, 30.0f}},
        {{0.0f, 60.0f}, {0.0f, 0.0f}, {50.0f, 30.0f}}
    };
    const float angles[4] = {-18.0f, 71.0f, 143.0f, -96.0f};
    const float translations[4][2] = {
        {30.0f, 20.0f}, {155.0f, 30.0f},
        {45.0f, 105.0f}, {165.0f, 115.0f}
    };
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    uint8_t piece_index;

    (void)memset(&frame, 0, sizeof(frame));
    Decision_GetDefaultConfig(&config);
    frame.piece_count = 4U;

    for (piece_index = 0U; piece_index < frame.piece_count; ++piece_index) {
        uint8_t vertex_index;
        frame.pieces[piece_index].id = (uint8_t)(10U + piece_index);
        frame.pieces[piece_index].vertex_count = 3U;
        frame.pieces[piece_index].center = (DecisionPoint){0.0f, 0.0f};
        for (vertex_index = 0U; vertex_index < 3U; ++vertex_index) {
            frame.pieces[piece_index].vertices[vertex_index] =
                target[piece_index][vertex_index];
            frame.pieces[piece_index].center.x_mm +=
                target[piece_index][vertex_index].x_mm / 3.0f;
            frame.pieces[piece_index].center.y_mm +=
                target[piece_index][vertex_index].y_mm / 3.0f;
        }
        transform_piece(&frame.pieces[piece_index],
                        angles[piece_index],
                        translations[piece_index][0],
                        translations[piece_index][1]);
    }

    result = Decision_SolveGeneral(&frame, &config, &plan);
    ASSERT_EQ_INT(DECISION_RESULT_OK, result);

    for (piece_index = 0U; piece_index < frame.piece_count; ++piece_index) {
        uint8_t vertex_index;
        for (vertex_index = 0U; vertex_index < 3U; ++vertex_index) {
            DecisionPoint placed = apply_move(
                &plan.moves[piece_index],
                frame.pieces[piece_index].vertices[vertex_index]);
            if (placed.x_mm < min_x) min_x = placed.x_mm;
            if (placed.x_mm > max_x) max_x = placed.x_mm;
            if (placed.y_mm < min_y) min_y = placed.y_mm;
            if (placed.y_mm > max_y) max_y = placed.y_mm;
        }
    }

    ASSERT_NEAR(100.0f, max_x - min_x, 0.1f);
    ASSERT_NEAR(60.0f, max_y - min_y, 0.1f);
    ASSERT_NEAR(6000.0f, (max_x - min_x) * (max_y - min_y), 1.0f);
}

static void test_decision_move_builds_trajectory(void)
{
    const DecisionMove move = {
        .piece_id = 2U,
        .pick = {40.0f, 60.0f, 0.0f, 0.0f},
        .transit = {40.0f, 60.0f, 40.0f, 35.0f},
        .place = {105.0f, 220.0f, 0.0f, 35.0f}
    };
    const TrajectoryPose current = {0.0f, 0.0f, 40.0f, 0.0f};
    const TrajectoryLimits limits = {120.0f, 300.0f, 90.0f, 180.0f};
    TrajectoryRequest request;
    TrajectoryPlan trajectory;

    ASSERT_TRUE(Decision_BuildTrajectoryRequest(&move,
                                                &current,
                                                &limits,
                                                &request) != 0U);
    ASSERT_NEAR(current.x_mm, request.current.x_mm, 0.001f);
    ASSERT_NEAR(move.pick.x_mm, request.pick.x_mm, 0.001f);
    ASSERT_NEAR(move.transit.z_mm, request.transit.z_mm, 0.001f);
    ASSERT_NEAR(move.place.y_mm, request.place.y_mm, 0.001f);
    ASSERT_NEAR(limits.max_linear_velocity_mm_s,
                request.limits.max_linear_velocity_mm_s,
                0.001f);
    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &trajectory));
    ASSERT_TRUE(Trajectory_GetDuration(&trajectory,
                                       TRAJECTORY_PHASE_APPROACH) > 0.0f);
    ASSERT_TRUE(Trajectory_GetDuration(&trajectory,
                                       TRAJECTORY_PHASE_TRANSFER) > 0.0f);
}

int main(void)
{
    test_fixed_mode_recovers_rotation();
    test_general_mode_supports_two_and_three_pieces();
    test_general_mode_builds_horizontal_rectangle();
    test_general_mode_handles_irregular_pieces();
    test_general_mode_rejects_non_rectangle();
    test_decision_move_builds_trajectory();
    puts("decision tests passed");
    return EXIT_SUCCESS;
}

#include "vision_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
        if ((actual_value < expected_value - tolerance_value) ||                  \
            (actual_value > expected_value + tolerance_value)) {                  \
            fprintf(stderr, "%s:%d expected %.7g, got %.7g\n",               \
                    __FILE__, __LINE__, expected_value, actual_value);              \
            exit(EXIT_FAILURE);                                                   \
        }                                                                         \
    } while (0)

static const char ValidJson[] =
    "{"
    "\"metadata\":{\"source\":\"maixcam\",\"values\":[1,2,3]},"
    "\"pieces\":["
      "{"
        "\"vertices_mm\":[[72.1,58.0],[95.6,58.2],[101.3,74.1],[80.4,80.6]],"
        "\"vertex_count\":4,\"cy_mm\":67.2,\"cx_mm\":83.4,\"id\":0"
      "},"
      "{"
        "\"id\":3,\"cx_mm\":-12.5,\"cy_mm\":2.5e2,\"vertex_count\":3,"
        "\"vertices_mm\":[[-20,240],[-5.0,240],[-12.5,270]]"
      "}"
    "],"
    "\"seq\":12,\"type\":\"VISION_RESULT\""
    "}";

static void test_valid_message(void)
{
    DecisionVisionFrame frame;
    VisionJsonResult result = VisionJson_ParseString(ValidJson, &frame);

    ASSERT_EQ_INT(VISION_JSON_RESULT_OK, result);
    ASSERT_EQ_INT(12, frame.seq);
    ASSERT_EQ_INT(2, frame.piece_count);
    ASSERT_EQ_INT(0, frame.pieces[0].id);
    ASSERT_EQ_INT(4, frame.pieces[0].vertex_count);
    ASSERT_NEAR(83.4f, frame.pieces[0].center.x_mm, 0.001f);
    ASSERT_NEAR(80.4f, frame.pieces[0].vertices[3].x_mm, 0.001f);
    ASSERT_EQ_INT(3, frame.pieces[1].id);
    ASSERT_NEAR(250.0f, frame.pieces[1].center.y_mm, 0.001f);
    ASSERT_NEAR(-20.0f, frame.pieces[1].vertices[0].x_mm, 0.001f);
}

static void test_explicit_length(void)
{
    DecisionVisionFrame frame;
    const char suffix[] = " ignored";
    char buffer[sizeof(ValidJson) + sizeof(suffix)];

    (void)memcpy(buffer, ValidJson, sizeof(ValidJson) - 1U);
    (void)memcpy(&buffer[sizeof(ValidJson) - 1U], suffix, sizeof(suffix));

    ASSERT_EQ_INT(VISION_JSON_RESULT_OK,
                  VisionJson_Parse(buffer, sizeof(ValidJson) - 1U, &frame));
    ASSERT_EQ_INT(2, frame.piece_count);
}

static void test_protocol_errors(void)
{
    static const char MissingField[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1}";
    static const char WrongType[] =
        "{\"type\":\"STATE\",\"seq\":1,\"pieces\":[]}";
    static const char DuplicateField[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"seq\":2,\"pieces\":[]}";
    static const char Incomplete[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"pieces\":[";
    static const char DuplicateId[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"pieces\":["
        "{\"id\":2,\"cx_mm\":1,\"cy_mm\":1,\"vertex_count\":3,"
        "\"vertices_mm\":[[0,0],[2,0],[1,2]]},"
        "{\"id\":2,\"cx_mm\":4,\"cy_mm\":1,\"vertex_count\":3,"
        "\"vertices_mm\":[[3,0],[5,0],[4,2]]}]}";
    static const char CountMismatch[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"pieces\":["
        "{\"id\":1,\"cx_mm\":1,\"cy_mm\":1,\"vertex_count\":4,"
        "\"vertices_mm\":[[0,0],[2,0],[1,2]]}]}";
    DecisionVisionFrame frame;

    ASSERT_EQ_INT(VISION_JSON_RESULT_MISSING_FIELD,
                  VisionJson_ParseString(MissingField, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_WRONG_MESSAGE_TYPE,
                  VisionJson_ParseString(WrongType, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_DUPLICATE_FIELD,
                  VisionJson_ParseString(DuplicateField, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_INCOMPLETE,
                  VisionJson_ParseString(Incomplete, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_DUPLICATE_PIECE_ID,
                  VisionJson_ParseString(DuplicateId, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_VERTEX_COUNT_MISMATCH,
                  VisionJson_ParseString(CountMismatch, &frame));
}

static void test_range_errors(void)
{
    static const char TooManyPieces[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"pieces\":[{},{},{},{},{}]}";
    static const char TooManyVertices[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"pieces\":["
        "{\"id\":1,\"cx_mm\":1,\"cy_mm\":1,\"vertex_count\":6,"
        "\"vertices_mm\":[[0,0],[1,0],[2,1],[2,2],[1,3],[0,2]]}]}";
    static const char InvalidCoordinate[] =
        "{\"type\":\"VISION_RESULT\",\"seq\":1,\"pieces\":["
        "{\"id\":1,\"cx_mm\":1e999,\"cy_mm\":1,\"vertex_count\":3,"
        "\"vertices_mm\":[[0,0],[2,0],[1,2]]}]}";
    DecisionVisionFrame frame;

    ASSERT_EQ_INT(VISION_JSON_RESULT_TOO_MANY_PIECES,
                  VisionJson_ParseString(TooManyPieces, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_TOO_MANY_VERTICES,
                  VisionJson_ParseString(TooManyVertices, &frame));
    ASSERT_EQ_INT(VISION_JSON_RESULT_INVALID_VALUE,
                  VisionJson_ParseString(InvalidCoordinate, &frame));
}

static void test_failure_does_not_modify_output(void)
{
    DecisionVisionFrame frame;
    DecisionVisionFrame before;

    (void)memset(&frame, 0x5A, sizeof(frame));
    before = frame;
    ASSERT_EQ_INT(VISION_JSON_RESULT_INCOMPLETE,
                  VisionJson_ParseString("{\"type\":", &frame));
    ASSERT_TRUE(memcmp(&before, &frame, sizeof(frame)) == 0);
    ASSERT_TRUE(strcmp("incomplete",
                       VisionJson_ResultString(VISION_JSON_RESULT_INCOMPLETE)) == 0);
}

int main(void)
{
    test_valid_message();
    test_explicit_length();
    test_protocol_errors();
    test_range_errors();
    test_failure_does_not_modify_output();
    puts("vision JSON tests passed");
    return EXIT_SUCCESS;
}

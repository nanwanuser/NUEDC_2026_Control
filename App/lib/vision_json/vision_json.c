#define JSMN_STATIC
#define JSMN_STRICT
#include "jsmn.h"

#include "vision_json.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VISION_JSON_MAX_TOKENS 160U
#define VISION_JSON_NUMBER_BUFFER_SIZE 48U

enum {
    ROOT_FIELD_TYPE = 1U << 0,
    ROOT_FIELD_SEQ = 1U << 1,
    ROOT_FIELD_PIECES = 1U << 2,
    ROOT_REQUIRED_FIELDS = ROOT_FIELD_TYPE | ROOT_FIELD_SEQ | ROOT_FIELD_PIECES
};

enum {
    PIECE_FIELD_ID = 1U << 0,
    PIECE_FIELD_CX = 1U << 1,
    PIECE_FIELD_CY = 1U << 2,
    PIECE_FIELD_VERTEX_COUNT = 1U << 3,
    PIECE_FIELD_VERTICES = 1U << 4,
    PIECE_REQUIRED_FIELDS = PIECE_FIELD_ID | PIECE_FIELD_CX | PIECE_FIELD_CY |
                            PIECE_FIELD_VERTEX_COUNT | PIECE_FIELD_VERTICES
};

static jsmntok_t VisionJson_Tokens[VISION_JSON_MAX_TOKENS];

static uint8_t token_equals(const char *json,
                            const jsmntok_t *token,
                            const char *expected)
{
    size_t expected_length;
    size_t token_length;

    if (token->type != JSMN_STRING || token->start < 0 || token->end < token->start) {
        return 0U;
    }

    expected_length = strlen(expected);
    token_length = (size_t)(token->end - token->start);
    return (uint8_t)(token_length == expected_length &&
                     memcmp(&json[token->start], expected, expected_length) == 0);
}

static int32_t skip_token(const jsmntok_t *tokens,
                          int32_t token_count,
                          int32_t token_index)
{
    const jsmntok_t *token;
    int32_t next;
    int32_t child;

    if (token_index < 0 || token_index >= token_count) {
        return -1;
    }

    token = &tokens[token_index];
    next = token_index + 1;

    if (token->type == JSMN_ARRAY) {
        for (child = 0; child < token->size; ++child) {
            next = skip_token(tokens, token_count, next);
            if (next < 0) {
                return -1;
            }
        }
    } else if (token->type == JSMN_OBJECT) {
        for (child = 0; child < token->size; ++child) {
            if (next >= token_count || tokens[next].type != JSMN_STRING) {
                return -1;
            }
            ++next;
            next = skip_token(tokens, token_count, next);
            if (next < 0) {
                return -1;
            }
        }
    }
    return next;
}

static uint8_t parse_uint32_token(const char *json,
                                  const jsmntok_t *token,
                                  uint32_t *value)
{
    uint32_t result = 0U;
    int32_t index;

    if (token->type != JSMN_PRIMITIVE || token->start < 0 ||
        token->end <= token->start) {
        return 0U;
    }
    if ((token->end - token->start) > 1 && json[token->start] == '0') {
        return 0U;
    }

    for (index = token->start; index < token->end; ++index) {
        uint32_t digit;

        if (json[index] < '0' || json[index] > '9') {
            return 0U;
        }
        digit = (uint32_t)(json[index] - '0');
        if (result > (UINT32_MAX - digit) / 10U) {
            return 0U;
        }
        result = result * 10U + digit;
    }

    *value = result;
    return 1U;
}

static uint8_t parse_coordinate_token(const char *json,
                                      const jsmntok_t *token,
                                      float *value)
{
    char number[VISION_JSON_NUMBER_BUFFER_SIZE];
    char *end_pointer;
    size_t length;
    float parsed;

    if (token->type != JSMN_PRIMITIVE || token->start < 0 ||
        token->end <= token->start) {
        return 0U;
    }

    length = (size_t)(token->end - token->start);
    if (length >= sizeof(number)) {
        return 0U;
    }

    (void)memcpy(number, &json[token->start], length);
    number[length] = '\0';
    errno = 0;
    end_pointer = NULL;
    parsed = strtof(number, &end_pointer);

    if (errno == ERANGE || end_pointer != &number[length] ||
        !isfinite(parsed) ||
        fabsf(parsed) > VISION_JSON_MAX_ABS_COORDINATE_MM) {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

static VisionJsonResult parse_vertices(const char *json,
                                       const jsmntok_t *tokens,
                                       int32_t token_count,
                                       int32_t array_index,
                                       DecisionPiece *piece,
                                       uint8_t *parsed_vertex_count)
{
    const jsmntok_t *array_token = &tokens[array_index];
    int32_t next = array_index + 1;
    int32_t vertex_index;

    if (array_token->type != JSMN_ARRAY) {
        return VISION_JSON_RESULT_INVALID_VALUE;
    }
    if (array_token->size > (int32_t)DECISION_MAX_VERTICES) {
        return VISION_JSON_RESULT_TOO_MANY_VERTICES;
    }
    if (array_token->size < 3) {
        return VISION_JSON_RESULT_INVALID_VALUE;
    }

    for (vertex_index = 0; vertex_index < array_token->size; ++vertex_index) {
        const jsmntok_t *point_token;

        if (next < 0 || next >= token_count) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
        point_token = &tokens[next];
        if (point_token->type != JSMN_ARRAY || point_token->size != 2 ||
            next + 2 >= token_count) {
            return VISION_JSON_RESULT_INVALID_VALUE;
        }
        if (parse_coordinate_token(json,
                                   &tokens[next + 1],
                                   &piece->vertices[vertex_index].x_mm) == 0U ||
            parse_coordinate_token(json,
                                   &tokens[next + 2],
                                   &piece->vertices[vertex_index].y_mm) == 0U) {
            return VISION_JSON_RESULT_INVALID_VALUE;
        }

        next = skip_token(tokens, token_count, next);
        if (next < 0) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
    }

    *parsed_vertex_count = (uint8_t)array_token->size;
    return VISION_JSON_RESULT_OK;
}

static VisionJsonResult parse_piece(const char *json,
                                    const jsmntok_t *tokens,
                                    int32_t token_count,
                                    int32_t object_index,
                                    DecisionPiece *piece)
{
    const jsmntok_t *object_token = &tokens[object_index];
    uint32_t fields = 0U;
    uint8_t parsed_vertex_count = 0U;
    int32_t next = object_index + 1;
    int32_t field_index;

    if (object_token->type != JSMN_OBJECT) {
        return VISION_JSON_RESULT_INVALID_VALUE;
    }

    for (field_index = 0; field_index < object_token->size; ++field_index) {
        const jsmntok_t *key;
        const jsmntok_t *value;
        uint32_t integer_value;
        uint32_t field_bit = 0U;
        VisionJsonResult result = VISION_JSON_RESULT_OK;

        if (next < 0 || next + 1 >= token_count) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
        key = &tokens[next];
        value = &tokens[next + 1];

        if (token_equals(json, key, "id") != 0U) {
            field_bit = PIECE_FIELD_ID;
            if (parse_uint32_token(json, value, &integer_value) == 0U ||
                integer_value > UINT8_MAX) {
                return VISION_JSON_RESULT_INVALID_VALUE;
            }
            piece->id = (uint8_t)integer_value;
        } else if (token_equals(json, key, "cx_mm") != 0U) {
            field_bit = PIECE_FIELD_CX;
            if (parse_coordinate_token(json, value, &piece->center.x_mm) == 0U) {
                return VISION_JSON_RESULT_INVALID_VALUE;
            }
        } else if (token_equals(json, key, "cy_mm") != 0U) {
            field_bit = PIECE_FIELD_CY;
            if (parse_coordinate_token(json, value, &piece->center.y_mm) == 0U) {
                return VISION_JSON_RESULT_INVALID_VALUE;
            }
        } else if (token_equals(json, key, "vertex_count") != 0U) {
            field_bit = PIECE_FIELD_VERTEX_COUNT;
            if (parse_uint32_token(json, value, &integer_value) == 0U) {
                return VISION_JSON_RESULT_INVALID_VALUE;
            }
            if (integer_value > DECISION_MAX_VERTICES) {
                return VISION_JSON_RESULT_TOO_MANY_VERTICES;
            }
            piece->vertex_count = (uint8_t)integer_value;
        } else if (token_equals(json, key, "vertices_mm") != 0U) {
            field_bit = PIECE_FIELD_VERTICES;
            result = parse_vertices(json,
                                    tokens,
                                    token_count,
                                    next + 1,
                                    piece,
                                    &parsed_vertex_count);
            if (result != VISION_JSON_RESULT_OK) {
                return result;
            }
        }

        if (field_bit != 0U) {
            if ((fields & field_bit) != 0U) {
                return VISION_JSON_RESULT_DUPLICATE_FIELD;
            }
            fields |= field_bit;
        }

        next = skip_token(tokens, token_count, next + 1);
        if (next < 0) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
    }

    if ((fields & PIECE_REQUIRED_FIELDS) != PIECE_REQUIRED_FIELDS) {
        return VISION_JSON_RESULT_MISSING_FIELD;
    }
    if (piece->vertex_count < 3U) {
        return VISION_JSON_RESULT_INVALID_VALUE;
    }
    if (piece->vertex_count != parsed_vertex_count) {
        return VISION_JSON_RESULT_VERTEX_COUNT_MISMATCH;
    }
    return VISION_JSON_RESULT_OK;
}

static VisionJsonResult parse_pieces(const char *json,
                                     const jsmntok_t *tokens,
                                     int32_t token_count,
                                     int32_t array_index,
                                     DecisionVisionFrame *frame)
{
    const jsmntok_t *array_token = &tokens[array_index];
    int32_t next = array_index + 1;
    int32_t piece_index;

    if (array_token->type != JSMN_ARRAY || array_token->size <= 0) {
        return VISION_JSON_RESULT_INVALID_VALUE;
    }
    if (array_token->size > (int32_t)DECISION_MAX_PIECES) {
        return VISION_JSON_RESULT_TOO_MANY_PIECES;
    }

    frame->piece_count = (uint8_t)array_token->size;
    for (piece_index = 0; piece_index < array_token->size; ++piece_index) {
        VisionJsonResult result;
        int32_t previous_index;

        if (next < 0 || next >= token_count) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
        result = parse_piece(json,
                             tokens,
                             token_count,
                             next,
                             &frame->pieces[piece_index]);
        if (result != VISION_JSON_RESULT_OK) {
            return result;
        }

        for (previous_index = 0; previous_index < piece_index; ++previous_index) {
            if (frame->pieces[previous_index].id == frame->pieces[piece_index].id) {
                return VISION_JSON_RESULT_DUPLICATE_PIECE_ID;
            }
        }

        next = skip_token(tokens, token_count, next);
        if (next < 0) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
    }
    return VISION_JSON_RESULT_OK;
}

static VisionJsonResult map_parser_error(int32_t parser_result)
{
    if (parser_result == JSMN_ERROR_PART) {
        return VISION_JSON_RESULT_INCOMPLETE;
    }
    if (parser_result == JSMN_ERROR_NOMEM) {
        return VISION_JSON_RESULT_TOO_COMPLEX;
    }
    return VISION_JSON_RESULT_INVALID_JSON;
}

VisionJsonResult VisionJson_Parse(const char *json,
                                  size_t length,
                                  DecisionVisionFrame *frame)
{
    DecisionVisionFrame parsed_frame;
    jsmn_parser parser;
    int32_t token_count;
    int32_t next = 1;
    int32_t field_index;
    uint32_t fields = 0U;

    if (json == NULL || frame == NULL || length == 0U) {
        return VISION_JSON_RESULT_INVALID_ARGUMENT;
    }
    if (length > VISION_JSON_MAX_LENGTH || length > UINT_MAX) {
        return VISION_JSON_RESULT_TOO_LONG;
    }

    (void)memset(&parsed_frame, 0, sizeof(parsed_frame));
    jsmn_init(&parser);
    token_count = jsmn_parse(&parser,
                             json,
                             length,
                             VisionJson_Tokens,
                             VISION_JSON_MAX_TOKENS);
    if (token_count < 0) {
        return map_parser_error(token_count);
    }
    if (token_count == 0 || VisionJson_Tokens[0].type != JSMN_OBJECT ||
        skip_token(VisionJson_Tokens, token_count, 0) != token_count) {
        return VISION_JSON_RESULT_INVALID_JSON;
    }

    for (field_index = 0;
         field_index < VisionJson_Tokens[0].size;
         ++field_index) {
        const jsmntok_t *key;
        const jsmntok_t *value;
        uint32_t field_bit = 0U;
        VisionJsonResult result = VISION_JSON_RESULT_OK;

        if (next < 0 || next + 1 >= token_count) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
        key = &VisionJson_Tokens[next];
        value = &VisionJson_Tokens[next + 1];

        if (token_equals(json, key, "type") != 0U) {
            field_bit = ROOT_FIELD_TYPE;
            if (token_equals(json, value, "VISION_RESULT") == 0U) {
                return VISION_JSON_RESULT_WRONG_MESSAGE_TYPE;
            }
        } else if (token_equals(json, key, "seq") != 0U) {
            field_bit = ROOT_FIELD_SEQ;
            if (parse_uint32_token(json, value, &parsed_frame.seq) == 0U) {
                return VISION_JSON_RESULT_INVALID_VALUE;
            }
        } else if (token_equals(json, key, "pieces") != 0U) {
            field_bit = ROOT_FIELD_PIECES;
            result = parse_pieces(json,
                                  VisionJson_Tokens,
                                  token_count,
                                  next + 1,
                                  &parsed_frame);
            if (result != VISION_JSON_RESULT_OK) {
                return result;
            }
        }

        if (field_bit != 0U) {
            if ((fields & field_bit) != 0U) {
                return VISION_JSON_RESULT_DUPLICATE_FIELD;
            }
            fields |= field_bit;
        }

        next = skip_token(VisionJson_Tokens, token_count, next + 1);
        if (next < 0) {
            return VISION_JSON_RESULT_INVALID_JSON;
        }
    }

    if ((fields & ROOT_REQUIRED_FIELDS) != ROOT_REQUIRED_FIELDS) {
        return VISION_JSON_RESULT_MISSING_FIELD;
    }

    *frame = parsed_frame;
    return VISION_JSON_RESULT_OK;
}

VisionJsonResult VisionJson_ParseString(const char *json,
                                        DecisionVisionFrame *frame)
{
    if (json == NULL) {
        return VISION_JSON_RESULT_INVALID_ARGUMENT;
    }
    return VisionJson_Parse(json, strlen(json), frame);
}

const char *VisionJson_ResultString(VisionJsonResult result)
{
    static const char *const names[] = {
        "ok",
        "invalid_argument",
        "too_long",
        "incomplete",
        "invalid_json",
        "too_complex",
        "wrong_message_type",
        "missing_field",
        "duplicate_field",
        "invalid_value",
        "too_many_pieces",
        "too_many_vertices",
        "vertex_count_mismatch",
        "duplicate_piece_id"
    };

    if ((uint32_t)result >= (uint32_t)(sizeof(names) / sizeof(names[0]))) {
        return "unknown";
    }
    return names[result];
}

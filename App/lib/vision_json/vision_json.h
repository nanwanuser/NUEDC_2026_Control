#ifndef VISION_JSON_H
#define VISION_JSON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "decision.h"

#define VISION_JSON_MAX_LENGTH 2048U
#define VISION_JSON_MAX_ABS_COORDINATE_MM 1000.0f

typedef enum {
    VISION_JSON_RESULT_OK = 0,
    VISION_JSON_RESULT_INVALID_ARGUMENT,
    VISION_JSON_RESULT_TOO_LONG,
    VISION_JSON_RESULT_INCOMPLETE,
    VISION_JSON_RESULT_INVALID_JSON,
    VISION_JSON_RESULT_TOO_COMPLEX,
    VISION_JSON_RESULT_WRONG_MESSAGE_TYPE,
    VISION_JSON_RESULT_MISSING_FIELD,
    VISION_JSON_RESULT_DUPLICATE_FIELD,
    VISION_JSON_RESULT_INVALID_VALUE,
    VISION_JSON_RESULT_TOO_MANY_PIECES,
    VISION_JSON_RESULT_TOO_MANY_VERTICES,
    VISION_JSON_RESULT_VERTEX_COUNT_MISMATCH,
    VISION_JSON_RESULT_DUPLICATE_PIECE_ID
} VisionJsonResult;

/* length excludes the optional trailing NUL byte. This function is not reentrant. */
VisionJsonResult VisionJson_Parse(const char *json,
                                  size_t length,
                                  DecisionVisionFrame *frame);

VisionJsonResult VisionJson_ParseString(const char *json,
                                        DecisionVisionFrame *frame);

const char *VisionJson_ResultString(VisionJsonResult result);

#ifdef __cplusplus
}
#endif

#endif

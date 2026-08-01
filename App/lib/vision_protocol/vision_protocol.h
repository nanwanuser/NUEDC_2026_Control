#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "decision.h"

#define VISION_PROTOCOL_VERSION               1U
#define VISION_PROTOCOL_TYPE_FRAME            0x01U
#define VISION_PROTOCOL_TYPE_CARD_CHUNK       0x02U
#define VISION_PROTOCOL_HEADER_FIRST          0xAAU
#define VISION_PROTOCOL_HEADER_SECOND         0x55U
#define VISION_PROTOCOL_END_FIRST             0x0DU
#define VISION_PROTOCOL_END_SECOND            0x0AU
#define VISION_PROTOCOL_MAX_PAYLOAD_LENGTH     106U
#define VISION_PROTOCOL_MAX_FRAME_LENGTH       118U
#define VISION_PROTOCOL_STABLE_FRAME_COUNT     3U
#define VISION_PROTOCOL_COORD_TOLERANCE_MM     0.5f
#define VISION_PROTOCOL_CARD_FORMAT_VERSION    1U
#define VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH 1536U
#define VISION_PROTOCOL_CARD_CHUNK_HEADER_LENGTH 11U
#define VISION_PROTOCOL_CARD_MAX_CHUNK_DATA \
    (VISION_PROTOCOL_MAX_PAYLOAD_LENGTH - \
     VISION_PROTOCOL_CARD_CHUNK_HEADER_LENGTH)
#define VISION_PROTOCOL_CARD_RECEIVED_BYTES \
    ((VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH + 7U) / 8U)

typedef enum {
    VISION_PROTOCOL_RESULT_NONE = 0,
    VISION_PROTOCOL_RESULT_FRAME,
    VISION_PROTOCOL_RESULT_CARD_CHUNK,
    VISION_PROTOCOL_RESULT_INVALID_LENGTH,
    VISION_PROTOCOL_RESULT_INVALID_VERSION,
    VISION_PROTOCOL_RESULT_INVALID_TYPE,
    VISION_PROTOCOL_RESULT_INVALID_END,
    VISION_PROTOCOL_RESULT_CRC_ERROR,
    VISION_PROTOCOL_RESULT_INVALID_DATA
} VisionProtocolResult;

typedef struct {
    uint16_t seq;
    uint32_t layout_id;
    uint16_t total_length;
    uint16_t offset;
    uint16_t full_crc;
    uint8_t data_length;
    uint8_t data[VISION_PROTOCOL_CARD_MAX_CHUNK_DATA];
} VisionProtocolCardChunk;

typedef struct {
    uint16_t seq;
    uint8_t type;
    /* The frame's former mode byte, kept so the wire layout is unchanged. */
    uint8_t reserved;
    DecisionVisionFrame frame;
    VisionProtocolCardChunk card_chunk;
} VisionProtocolPacket;

typedef struct {
    uint8_t data[VISION_PROTOCOL_MAX_FRAME_LENGTH];
    uint16_t length;
    uint16_t expected_length;
} VisionProtocolParser;

typedef struct {
    VisionProtocolPacket candidate;
    uint8_t stable_count;
    uint8_t has_candidate;
} VisionProtocolStabilizer;

typedef struct {
    uint32_t layout_id;
    uint16_t total_length;
    uint16_t full_crc;
    uint16_t received_count;
    uint8_t active;
    uint8_t data[VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH];
    uint8_t received[VISION_PROTOCOL_CARD_RECEIVED_BYTES];
} VisionProtocolCardAssembler;

void VisionProtocolParser_Init(VisionProtocolParser *parser);
void VisionProtocolParser_Reset(VisionProtocolParser *parser);
uint8_t VisionProtocolParser_HasPartialFrame(const VisionProtocolParser *parser);
VisionProtocolResult VisionProtocolParser_PushByte(
    VisionProtocolParser *parser,
    uint8_t byte,
    VisionProtocolPacket *packet);

void VisionProtocolStabilizer_Init(VisionProtocolStabilizer *stabilizer);
void VisionProtocolStabilizer_Reset(VisionProtocolStabilizer *stabilizer);
uint8_t VisionProtocolStabilizer_Add(
    VisionProtocolStabilizer *stabilizer,
    const VisionProtocolPacket *packet,
    VisionProtocolPacket *stable_packet);

void VisionProtocolCardAssembler_Init(VisionProtocolCardAssembler *assembler);
void VisionProtocolCardAssembler_Reset(VisionProtocolCardAssembler *assembler);
uint8_t VisionProtocolCardAssembler_Add(
    VisionProtocolCardAssembler *assembler,
    const VisionProtocolCardChunk *chunk,
    DecisionCardFrame *card_frame);

uint16_t VisionProtocol_Crc16(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif

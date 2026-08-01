#include "vision_protocol.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ASSERT_TRUE(condition)                                                \
    do {                                                                      \
        if (!(condition)) {                                                   \
            printf("assertion failed at line %d: %s\n",                     \
                   __LINE__, #condition);                                     \
            return 1;                                                         \
        }                                                                     \
    } while (0)

typedef struct {
    uint8_t id;
    uint8_t vertex_count;
    int16_t cx;
    int16_t cy;
    int16_t vertices[DECISION_MAX_VERTICES][2];
} RawPiece;

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    write_u16(data, (uint16_t)value);
    write_u16(&data[2], (uint16_t)(value >> 16U));
}

static size_t encode_frame(uint16_t seq,
                           uint8_t reserved,
                           const RawPiece *pieces,
                           uint8_t piece_count,
                           uint8_t *buffer)
{
    uint16_t payload_length = 2U;
    uint16_t position = 8U;
    uint16_t crc;
    uint8_t piece_index;

    for (piece_index = 0U; piece_index < piece_count; ++piece_index) {
        payload_length = (uint16_t)(payload_length + 6U +
                                    4U * pieces[piece_index].vertex_count);
    }

    buffer[0] = VISION_PROTOCOL_HEADER_FIRST;
    buffer[1] = VISION_PROTOCOL_HEADER_SECOND;
    buffer[2] = VISION_PROTOCOL_VERSION;
    buffer[3] = VISION_PROTOCOL_TYPE_FRAME;
    write_u16(&buffer[4], seq);
    write_u16(&buffer[6], payload_length);
    buffer[position++] = reserved;
    buffer[position++] = piece_count;

    for (piece_index = 0U; piece_index < piece_count; ++piece_index) {
        uint8_t vertex_index;

        buffer[position++] = pieces[piece_index].id;
        buffer[position++] = pieces[piece_index].vertex_count;
        write_u16(&buffer[position], (uint16_t)pieces[piece_index].cx);
        position += 2U;
        write_u16(&buffer[position], (uint16_t)pieces[piece_index].cy);
        position += 2U;
        for (vertex_index = 0U;
             vertex_index < pieces[piece_index].vertex_count;
             ++vertex_index) {
            write_u16(&buffer[position],
                      (uint16_t)pieces[piece_index].vertices[vertex_index][0]);
            position += 2U;
            write_u16(&buffer[position],
                      (uint16_t)pieces[piece_index].vertices[vertex_index][1]);
            position += 2U;
        }
    }

    crc = VisionProtocol_Crc16(&buffer[2], 6U + payload_length);
    write_u16(&buffer[position], crc);
    position += 2U;
    buffer[position++] = VISION_PROTOCOL_END_FIRST;
    buffer[position++] = VISION_PROTOCOL_END_SECOND;
    return position;
}

static VisionProtocolResult feed(VisionProtocolParser *parser,
                                 const uint8_t *frame,
                                 size_t length,
                                 VisionProtocolPacket *packet)
{
    VisionProtocolResult result = VISION_PROTOCOL_RESULT_NONE;
    size_t index;

    for (index = 0U; index < length; ++index) {
        VisionProtocolResult current =
            VisionProtocolParser_PushByte(parser, frame[index], packet);
        if (current != VISION_PROTOCOL_RESULT_NONE) {
            result = current;
        }
    }
    return result;
}

static size_t build_card_blob(uint8_t *blob)
{
    uint16_t position = 0U;

    blob[position++] = VISION_PROTOCOL_CARD_FORMAT_VERSION;
    blob[position++] = 1U;
    write_u16(&blob[position], 77U);
    position += 2U;

    blob[position++] = 7U;
    blob[position++] = 3U;
    write_u16(&blob[position], 100U);
    position += 2U;
    write_u16(&blob[position], 200U);
    position += 2U;
    write_u16(&blob[position], 0U);
    position += 2U;
    write_u16(&blob[position], 0U);
    position += 2U;
    write_u16(&blob[position], 200U);
    position += 2U;
    write_u16(&blob[position], 0U);
    position += 2U;
    write_u16(&blob[position], 0U);
    position += 2U;
    write_u16(&blob[position], 200U);
    position += 2U;

    blob[position++] = 2U;
    blob[position++] = 0U;
    blob[position++] = 51U;
    blob[position++] = DECISION_CARD_COLOR_RED;
    blob[position++] = 15U;
    blob[position++] = 8U;
    blob[position++] = 230U;
    blob[position++] = 1U;
    blob[position++] = 204U;
    blob[position++] = DECISION_CARD_COLOR_BLACK;
    blob[position++] = (uint8_t)-20;
    blob[position++] = 12U;
    blob[position++] = 240U;

    blob[position++] = 1U;
    write_u16(&blob[position], 120U);
    position += 2U;
    write_u16(&blob[position], 130U);
    position += 2U;
    write_u16(&blob[position], 45U);
    position += 2U;
    blob[position++] = DECISION_CARD_COLOR_RED;
    blob[position++] = DECISION_CARD_PRIMITIVE_DOT;
    blob[position++] = 5U;
    blob[position++] = 220U;
    return position;
}

static size_t encode_card_chunk(uint16_t seq,
                                uint32_t layout_id,
                                uint16_t total_length,
                                uint16_t offset,
                                uint16_t full_crc,
                                const uint8_t *chunk,
                                uint8_t chunk_length,
                                uint8_t *buffer)
{
    uint16_t payload_length = (uint16_t)(11U + chunk_length);
    uint16_t position = 8U;
    uint16_t frame_crc;

    buffer[0] = VISION_PROTOCOL_HEADER_FIRST;
    buffer[1] = VISION_PROTOCOL_HEADER_SECOND;
    buffer[2] = VISION_PROTOCOL_VERSION;
    buffer[3] = VISION_PROTOCOL_TYPE_CARD_CHUNK;
    write_u16(&buffer[4], seq);
    write_u16(&buffer[6], payload_length);
    write_u32(&buffer[position], layout_id);
    position += 4U;
    write_u16(&buffer[position], total_length);
    position += 2U;
    write_u16(&buffer[position], offset);
    position += 2U;
    write_u16(&buffer[position], full_crc);
    position += 2U;
    buffer[position++] = chunk_length;
    (void)memcpy(&buffer[position], chunk, chunk_length);
    position = (uint16_t)(position + chunk_length);
    frame_crc = VisionProtocol_Crc16(&buffer[2], 6U + payload_length);
    write_u16(&buffer[position], frame_crc);
    position += 2U;
    buffer[position++] = VISION_PROTOCOL_END_FIRST;
    buffer[position++] = VISION_PROTOCOL_END_SECOND;
    return position;
}

static int test_decode_four_pieces(void)
{
    RawPiece pieces[4] = {
        {0U, 3U, 100, 200, {{0, 0}, {100, 0}, {0, 100}}},
        {1U, 4U, 300, 400, {{200, 300}, {400, 300}, {400, 500}, {200, 500}}},
        {2U, 5U, -100, 50, {{-200, 0}, {-100, -50}, {0, 0}, {0, 100}, {-200, 100}}},
        {3U, 3U, 600, 700, {{500, 600}, {700, 600}, {600, 800}}}
    };
    uint8_t frame[VISION_PROTOCOL_MAX_FRAME_LENGTH];
    VisionProtocolParser parser;
    VisionProtocolPacket packet;
    /* Non-zero reserved byte: the decoder must accept and carry it. */
    size_t length = encode_frame(42U, 0x5AU, pieces, 4U, frame);

    VisionProtocolParser_Init(&parser);
    ASSERT_TRUE(length <= VISION_PROTOCOL_MAX_FRAME_LENGTH);
    ASSERT_TRUE(feed(&parser, frame, length, &packet) ==
                VISION_PROTOCOL_RESULT_FRAME);
    ASSERT_TRUE(packet.seq == 42U);
    ASSERT_TRUE(packet.reserved == 0x5AU);
    ASSERT_TRUE(packet.frame.piece_count == 4U);
    ASSERT_TRUE(packet.frame.pieces[2].vertex_count == 5U);
    ASSERT_TRUE(fabsf(packet.frame.pieces[2].center.x_mm + 10.0f) < 0.001f);
    return 0;
}

static int test_crc_and_end_rejected(void)
{
    RawPiece piece = {0U, 3U, 100, 200, {{0, 0}, {100, 0}, {0, 100}}};
    uint8_t frame[VISION_PROTOCOL_MAX_FRAME_LENGTH];
    VisionProtocolParser parser;
    VisionProtocolPacket packet;
    size_t length = encode_frame(1U, 0U, &piece, 1U, frame);

    frame[10] ^= 1U;
    VisionProtocolParser_Init(&parser);
    ASSERT_TRUE(feed(&parser, frame, length, &packet) ==
                VISION_PROTOCOL_RESULT_CRC_ERROR);

    (void)encode_frame(1U, 0U, &piece, 1U, frame);
    frame[length - 1U] = 0U;
    VisionProtocolParser_Init(&parser);
    ASSERT_TRUE(feed(&parser, frame, length, &packet) ==
                VISION_PROTOCOL_RESULT_INVALID_END);
    return 0;
}

static int test_three_stable_frames(void)
{
    RawPiece samples[3] = {
        {7U, 4U, 100, 100, {{0, 0}, {200, 0}, {200, 200}, {0, 200}}},
        {7U, 4U, 102, 99, {{202, 201}, {1, 199}, {2, 1}, {201, -1}}},
        {7U, 4U, 98, 101, {{-1, 202}, {199, 201}, {198, 1}, {-2, 0}}}
    };
    uint8_t frame[VISION_PROTOCOL_MAX_FRAME_LENGTH];
    VisionProtocolParser parser;
    VisionProtocolStabilizer stabilizer;
    VisionProtocolPacket packet;
    VisionProtocolPacket stable;
    uint8_t index;

    VisionProtocolParser_Init(&parser);
    VisionProtocolStabilizer_Init(&stabilizer);
    for (index = 0U; index < 3U; ++index) {
        /* Reserved differs per frame: the stabilizer must not compare it. */
        size_t length = encode_frame((uint16_t)(10U + index),
                                     index,
                                     &samples[index], 1U, frame);
        ASSERT_TRUE(feed(&parser, frame, length, &packet) ==
                    VISION_PROTOCOL_RESULT_FRAME);
        ASSERT_TRUE(VisionProtocolStabilizer_Add(&stabilizer,
                                                 &packet,
                                                 &stable) ==
                    (uint8_t)(index == 2U));
    }
    ASSERT_TRUE(stable.seq == 12U);
    ASSERT_TRUE(fabsf(stable.frame.pieces[0].center.x_mm - 10.0f) < 0.01f);
    ASSERT_TRUE(fabsf(stable.frame.pieces[0].vertices[0].x_mm) < 0.11f);
    return 0;
}

static int test_card_chunks_reassemble_out_of_order(void)
{
    uint8_t blob[VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH];
    uint8_t frame[VISION_PROTOCOL_MAX_FRAME_LENGTH];
    size_t blob_length = build_card_blob(blob);
    uint16_t full_crc = VisionProtocol_Crc16(blob, blob_length);
    uint16_t split = (uint16_t)(blob_length / 2U);
    VisionProtocolParser parser;
    VisionProtocolCardAssembler assembler;
    VisionProtocolPacket packet;
    DecisionCardFrame card;
    size_t frame_length;

    VisionProtocolParser_Init(&parser);
    VisionProtocolCardAssembler_Init(&assembler);

    frame_length = encode_card_chunk(
        10U, 0x12345678U, (uint16_t)blob_length, split, full_crc,
        &blob[split], (uint8_t)(blob_length - split), frame);
    ASSERT_TRUE(feed(&parser, frame, frame_length, &packet) ==
                VISION_PROTOCOL_RESULT_CARD_CHUNK);
    ASSERT_TRUE(packet.type == VISION_PROTOCOL_TYPE_CARD_CHUNK);
    ASSERT_TRUE(VisionProtocolCardAssembler_Add(
                    &assembler, &packet.card_chunk, &card) == 0U);

    frame_length = encode_card_chunk(
        11U, 0x12345678U, (uint16_t)blob_length, 0U, full_crc,
        blob, (uint8_t)split, frame);
    ASSERT_TRUE(feed(&parser, frame, frame_length, &packet) ==
                VISION_PROTOCOL_RESULT_CARD_CHUNK);
    ASSERT_TRUE(VisionProtocolCardAssembler_Add(
                    &assembler, &packet.card_chunk, &card) != 0U);
    ASSERT_TRUE(card.layout_id == 0x12345678U);
    ASSERT_TRUE(card.piece_count == 1U);
    ASSERT_TRUE(card.vision.seq == 77U);
    ASSERT_TRUE(card.pieces[0].edge_event_count == 2U);
    ASSERT_TRUE(card.pieces[0].edge_events[1].color ==
                DECISION_CARD_COLOR_BLACK);
    ASSERT_TRUE(card.pieces[0].primitive_count == 1U);
    ASSERT_TRUE(fabsf(card.pieces[0].primitives[0].center.x_mm - 12.0f) <
                0.001f);
    return 0;
}

static int test_card_aggregate_crc_failure_resets_assembler(void)
{
    uint8_t blob[VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH];
    uint8_t frame[VISION_PROTOCOL_MAX_FRAME_LENGTH];
    size_t blob_length = build_card_blob(blob);
    uint16_t full_crc = VisionProtocol_Crc16(blob, blob_length);
    uint16_t split = (uint16_t)(blob_length / 2U);
    VisionProtocolParser parser;
    VisionProtocolCardAssembler assembler;
    VisionProtocolPacket packet;
    DecisionCardFrame card;
    size_t frame_length;
    uint8_t cycle;

    VisionProtocolParser_Init(&parser);
    VisionProtocolCardAssembler_Init(&assembler);
    for (cycle = 0U; cycle < 2U; ++cycle) {
        uint16_t advertised_crc = cycle == 0U
            ? (uint16_t)(full_crc ^ 1U)
            : full_crc;

        frame_length = encode_card_chunk(
            20U, 0x87654321U, (uint16_t)blob_length, 0U, advertised_crc,
            blob, (uint8_t)split, frame);
        ASSERT_TRUE(feed(&parser, frame, frame_length, &packet) ==
                    VISION_PROTOCOL_RESULT_CARD_CHUNK);
        ASSERT_TRUE(VisionProtocolCardAssembler_Add(
                        &assembler, &packet.card_chunk, &card) == 0U);

        frame_length = encode_card_chunk(
            21U, 0x87654321U, (uint16_t)blob_length, split, advertised_crc,
            &blob[split], (uint8_t)(blob_length - split), frame);
        ASSERT_TRUE(feed(&parser, frame, frame_length, &packet) ==
                    VISION_PROTOCOL_RESULT_CARD_CHUNK);
        ASSERT_TRUE(VisionProtocolCardAssembler_Add(
                        &assembler, &packet.card_chunk, &card) ==
                    (uint8_t)(cycle == 1U));
    }
    ASSERT_TRUE(card.layout_id == 0x87654321U);
    return 0;
}

static int test_encode_mode_command(void)
{
    uint8_t frame[VISION_PROTOCOL_MODE_COMMAND_FRAME_LENGTH];
    uint16_t encoded_crc;
    size_t length = VisionProtocol_EncodeModeCommand(
        DECISION_STRATEGY_CARD_PATTERN, 0x1234U, frame, sizeof(frame));

    ASSERT_TRUE(length == VISION_PROTOCOL_MODE_COMMAND_FRAME_LENGTH);
    ASSERT_TRUE(frame[0] == VISION_PROTOCOL_HEADER_FIRST);
    ASSERT_TRUE(frame[1] == VISION_PROTOCOL_HEADER_SECOND);
    ASSERT_TRUE(frame[2] == VISION_PROTOCOL_VERSION);
    ASSERT_TRUE(frame[3] == VISION_PROTOCOL_TYPE_MODE_COMMAND);
    ASSERT_TRUE(frame[4] == 0x34U && frame[5] == 0x12U);
    ASSERT_TRUE(frame[6] == 1U && frame[7] == 0U);
    ASSERT_TRUE(frame[8] == VISION_PROTOCOL_MODE_CARD_PATTERN);
    encoded_crc = (uint16_t)(frame[9] | ((uint16_t)frame[10] << 8U));
    ASSERT_TRUE(encoded_crc == VisionProtocol_Crc16(&frame[2], 7U));
    ASSERT_TRUE(frame[11] == VISION_PROTOCOL_END_FIRST);
    ASSERT_TRUE(frame[12] == VISION_PROTOCOL_END_SECOND);

    ASSERT_TRUE(VisionProtocol_EncodeModeCommand(
                    DECISION_STRATEGY_GEOMETRIC, 7U,
                    frame, sizeof(frame)) == sizeof(frame));
    ASSERT_TRUE(frame[8] == VISION_PROTOCOL_MODE_GEOMETRIC);
    ASSERT_TRUE(VisionProtocol_EncodeModeCommand(
                    (DecisionStrategy)99, 0U,
                    frame, sizeof(frame)) == 0U);
    ASSERT_TRUE(VisionProtocol_EncodeModeCommand(
                    DECISION_STRATEGY_GEOMETRIC, 0U,
                    frame, sizeof(frame) - 1U) == 0U);
    ASSERT_TRUE(VisionProtocol_EncodeModeCommand(
                    DECISION_STRATEGY_GEOMETRIC, 0U,
                    NULL, sizeof(frame)) == 0U);
    return 0;
}

int main(void)
{
    if (test_decode_four_pieces() != 0) return 1;
    if (test_crc_and_end_rejected() != 0) return 1;
    if (test_three_stable_frames() != 0) return 1;
    if (test_card_chunks_reassemble_out_of_order() != 0) return 1;
    if (test_card_aggregate_crc_failure_resets_assembler() != 0) return 1;
    if (test_encode_mode_command() != 0) return 1;
    puts("vision protocol tests passed");
    return 0;
}

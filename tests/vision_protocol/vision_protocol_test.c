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

static size_t encode_frame(uint16_t seq,
                           DecisionMode mode,
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
    buffer[position++] = (uint8_t)mode;
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
    size_t length = encode_frame(42U, DECISION_MODE_GENERAL,
                                 pieces, 4U, frame);

    VisionProtocolParser_Init(&parser);
    ASSERT_TRUE(length <= VISION_PROTOCOL_MAX_FRAME_LENGTH);
    ASSERT_TRUE(feed(&parser, frame, length, &packet) ==
                VISION_PROTOCOL_RESULT_FRAME);
    ASSERT_TRUE(packet.seq == 42U);
    ASSERT_TRUE(packet.mode == DECISION_MODE_GENERAL);
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
    size_t length = encode_frame(1U, DECISION_MODE_GENERAL, &piece, 1U, frame);

    frame[10] ^= 1U;
    VisionProtocolParser_Init(&parser);
    ASSERT_TRUE(feed(&parser, frame, length, &packet) ==
                VISION_PROTOCOL_RESULT_CRC_ERROR);

    (void)encode_frame(1U, DECISION_MODE_GENERAL, &piece, 1U, frame);
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
        size_t length = encode_frame((uint16_t)(10U + index),
                                     DECISION_MODE_GENERAL,
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

static int test_ack(void)
{
    uint8_t ack[VISION_PROTOCOL_ACK_FRAME_LENGTH];
    uint16_t crc;

    ASSERT_TRUE(VisionProtocol_EncodeAck(0x1234U,
                                        VISION_PROTOCOL_ACK_ACCEPTED,
                                        ack,
                                        sizeof(ack)) == sizeof(ack));
    ASSERT_TRUE(ack[4] == 0x34U && ack[5] == 0x12U);
    ASSERT_TRUE(ack[11] == 0x0DU && ack[12] == 0x0AU);
    crc = VisionProtocol_Crc16(&ack[2], 7U);
    ASSERT_TRUE(ack[9] == (uint8_t)crc && ack[10] == (uint8_t)(crc >> 8U));
    return 0;
}

int main(void)
{
    if (test_decode_four_pieces() != 0) return 1;
    if (test_crc_and_end_rejected() != 0) return 1;
    if (test_three_stable_frames() != 0) return 1;
    if (test_ack() != 0) return 1;
    puts("vision protocol tests passed");
    return 0;
}

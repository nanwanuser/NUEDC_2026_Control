#include "vision_protocol.h"

#include <math.h>
#include <string.h>

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)read_u16_le(data) |
           ((uint32_t)read_u16_le(&data[2]) << 16U);
}

uint16_t VisionProtocol_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

    if (data == NULL) {
        return 0U;
    }

    for (index = 0U; index < length; ++index) {
        uint8_t bit;

        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static VisionProtocolResult decode_frame(const uint8_t *data,
                                         uint16_t frame_length,
                                         VisionProtocolPacket *packet)
{
    uint16_t payload_length;
    uint16_t payload_end;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint16_t position;
    uint8_t piece_index;

    if (data == NULL || packet == NULL || frame_length < 14U) {
        return VISION_PROTOCOL_RESULT_INVALID_LENGTH;
    }

    payload_length = read_u16_le(&data[6]);
    if (payload_length < 2U ||
        payload_length > VISION_PROTOCOL_MAX_PAYLOAD_LENGTH ||
        frame_length != (uint16_t)(12U + payload_length)) {
        return VISION_PROTOCOL_RESULT_INVALID_LENGTH;
    }

    payload_end = (uint16_t)(8U + payload_length);
    if (data[payload_end + 2U] != VISION_PROTOCOL_END_FIRST ||
        data[payload_end + 3U] != VISION_PROTOCOL_END_SECOND) {
        return VISION_PROTOCOL_RESULT_INVALID_END;
    }
    if (data[2] != VISION_PROTOCOL_VERSION) {
        return VISION_PROTOCOL_RESULT_INVALID_VERSION;
    }
    if (data[3] != VISION_PROTOCOL_TYPE_FRAME &&
        data[3] != VISION_PROTOCOL_TYPE_CARD_CHUNK) {
        return VISION_PROTOCOL_RESULT_INVALID_TYPE;
    }

    received_crc = read_u16_le(&data[payload_end]);
    calculated_crc = VisionProtocol_Crc16(&data[2],
                                           (size_t)(6U + payload_length));
    if (received_crc != calculated_crc) {
        return VISION_PROTOCOL_RESULT_CRC_ERROR;
    }

    (void)memset(packet, 0, sizeof(*packet));
    packet->seq = read_u16_le(&data[4]);
    packet->type = data[3];
    if (packet->type == VISION_PROTOCOL_TYPE_CARD_CHUNK) {
        VisionProtocolCardChunk *chunk = &packet->card_chunk;
        uint8_t data_length;

        if (payload_length < VISION_PROTOCOL_CARD_CHUNK_HEADER_LENGTH) {
            return VISION_PROTOCOL_RESULT_INVALID_LENGTH;
        }
        chunk->seq = packet->seq;
        chunk->layout_id = read_u32_le(&data[8]);
        chunk->total_length = read_u16_le(&data[12]);
        chunk->offset = read_u16_le(&data[14]);
        chunk->full_crc = read_u16_le(&data[16]);
        data_length = data[18];
        if (data_length > VISION_PROTOCOL_CARD_MAX_CHUNK_DATA ||
            payload_length != (uint16_t)(
                VISION_PROTOCOL_CARD_CHUNK_HEADER_LENGTH + data_length) ||
            chunk->total_length == 0U ||
            chunk->total_length >
                VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH ||
            chunk->offset > chunk->total_length ||
            data_length > (uint16_t)(chunk->total_length - chunk->offset)) {
            return VISION_PROTOCOL_RESULT_INVALID_DATA;
        }
        chunk->data_length = data_length;
        (void)memcpy(chunk->data, &data[19], data_length);
        return VISION_PROTOCOL_RESULT_CARD_CHUNK;
    }

    /* data[8] used to select the solve mode. Every task now uses the same
       edge-matching solve, so it is accepted and ignored rather than removed:
       dropping the byte would change the frame layout for the vision host. */
    packet->reserved = data[8];
    packet->frame.seq = packet->seq;
    packet->frame.piece_count = data[9];
    if (packet->frame.piece_count == 0U ||
        packet->frame.piece_count > DECISION_MAX_PIECES) {
        return VISION_PROTOCOL_RESULT_INVALID_DATA;
    }

    position = 10U;
    for (piece_index = 0U;
         piece_index < packet->frame.piece_count;
         ++piece_index) {
        DecisionPiece *piece = &packet->frame.pieces[piece_index];
        uint8_t vertex_index;
        uint8_t previous_index;

        if ((uint16_t)(payload_end - position) < 6U) {
            return VISION_PROTOCOL_RESULT_INVALID_LENGTH;
        }

        piece->id = data[position++];
        piece->vertex_count = data[position++];
        if (piece->vertex_count < 3U ||
            piece->vertex_count > DECISION_MAX_VERTICES ||
            (uint16_t)(payload_end - position) <
                (uint16_t)(4U + 4U * piece->vertex_count)) {
            return VISION_PROTOCOL_RESULT_INVALID_DATA;
        }

        piece->center.x_mm = (float)read_i16_le(&data[position]) * 0.1f;
        position += 2U;
        piece->center.y_mm = (float)read_i16_le(&data[position]) * 0.1f;
        position += 2U;

        for (vertex_index = 0U;
             vertex_index < piece->vertex_count;
             ++vertex_index) {
            piece->vertices[vertex_index].x_mm =
                (float)read_i16_le(&data[position]) * 0.1f;
            position += 2U;
            piece->vertices[vertex_index].y_mm =
                (float)read_i16_le(&data[position]) * 0.1f;
            position += 2U;
        }

        for (previous_index = 0U;
             previous_index < piece_index;
             ++previous_index) {
            if (packet->frame.pieces[previous_index].id == piece->id) {
                return VISION_PROTOCOL_RESULT_INVALID_DATA;
            }
        }
    }

    return position == payload_end
        ? VISION_PROTOCOL_RESULT_FRAME
        : VISION_PROTOCOL_RESULT_INVALID_LENGTH;
}

void VisionProtocolParser_Init(VisionProtocolParser *parser)
{
    VisionProtocolParser_Reset(parser);
}

void VisionProtocolParser_Reset(VisionProtocolParser *parser)
{
    if (parser == NULL) {
        return;
    }
    parser->length = 0U;
    parser->expected_length = 0U;
}

uint8_t VisionProtocolParser_HasPartialFrame(const VisionProtocolParser *parser)
{
    return (uint8_t)(parser != NULL && parser->length != 0U);
}

VisionProtocolResult VisionProtocolParser_PushByte(
    VisionProtocolParser *parser,
    uint8_t byte,
    VisionProtocolPacket *packet)
{
    VisionProtocolResult result;

    if (parser == NULL || packet == NULL) {
        return VISION_PROTOCOL_RESULT_INVALID_DATA;
    }

    if (parser->length == 0U) {
        if (byte == VISION_PROTOCOL_HEADER_FIRST) {
            parser->data[0] = byte;
            parser->length = 1U;
        }
        return VISION_PROTOCOL_RESULT_NONE;
    }

    if (parser->length == 1U) {
        if (byte == VISION_PROTOCOL_HEADER_SECOND) {
            parser->data[1] = byte;
            parser->length = 2U;
        } else if (byte != VISION_PROTOCOL_HEADER_FIRST) {
            parser->length = 0U;
        }
        return VISION_PROTOCOL_RESULT_NONE;
    }

    if (parser->length >= VISION_PROTOCOL_MAX_FRAME_LENGTH) {
        VisionProtocolParser_Reset(parser);
        return VISION_PROTOCOL_RESULT_INVALID_LENGTH;
    }
    parser->data[parser->length++] = byte;

    if (parser->length == 8U) {
        uint16_t payload_length = read_u16_le(&parser->data[6]);

        if (payload_length < 2U ||
            payload_length > VISION_PROTOCOL_MAX_PAYLOAD_LENGTH) {
            VisionProtocolParser_Reset(parser);
            return VISION_PROTOCOL_RESULT_INVALID_LENGTH;
        }
        parser->expected_length = (uint16_t)(12U + payload_length);
    }

    if (parser->expected_length != 0U &&
        parser->length == parser->expected_length) {
        result = decode_frame(parser->data, parser->length, packet);
        VisionProtocolParser_Reset(parser);
        return result;
    }

    return VISION_PROTOCOL_RESULT_NONE;
}

static void sort_pieces_by_id(DecisionVisionFrame *frame)
{
    uint8_t index;

    for (index = 1U; index < frame->piece_count; ++index) {
        DecisionPiece value = frame->pieces[index];
        uint8_t position = index;

        while (position > 0U &&
               frame->pieces[position - 1U].id > value.id) {
            frame->pieces[position] = frame->pieces[position - 1U];
            --position;
        }
        frame->pieces[position] = value;
    }
}

static uint8_t align_piece(const DecisionPiece *reference,
                           const DecisionPiece *incoming,
                           DecisionPiece *aligned)
{
    float best_error = INFINITY;
    uint8_t best_offset = 0U;
    int8_t best_direction = 1;
    uint8_t offset;
    int8_t direction;

    if (reference->id != incoming->id ||
        reference->vertex_count != incoming->vertex_count ||
        fabsf(reference->center.x_mm - incoming->center.x_mm) >
            VISION_PROTOCOL_COORD_TOLERANCE_MM ||
        fabsf(reference->center.y_mm - incoming->center.y_mm) >
            VISION_PROTOCOL_COORD_TOLERANCE_MM) {
        return 0U;
    }

    for (direction = -1; direction <= 1; direction += 2) {
        for (offset = 0U; offset < incoming->vertex_count; ++offset) {
            float max_error = 0.0f;
            uint8_t vertex_index;

            for (vertex_index = 0U;
                 vertex_index < incoming->vertex_count;
                 ++vertex_index) {
                int16_t mapped = (int16_t)offset +
                                 (int16_t)direction * vertex_index;
                uint8_t incoming_index;
                float dx;
                float dy;

                while (mapped < 0) {
                    mapped += incoming->vertex_count;
                }
                incoming_index = (uint8_t)(mapped % incoming->vertex_count);
                dx = fabsf(reference->vertices[vertex_index].x_mm -
                           incoming->vertices[incoming_index].x_mm);
                dy = fabsf(reference->vertices[vertex_index].y_mm -
                           incoming->vertices[incoming_index].y_mm);
                if (dx > max_error) max_error = dx;
                if (dy > max_error) max_error = dy;
            }

            if (max_error < best_error) {
                best_error = max_error;
                best_offset = offset;
                best_direction = direction;
            }
        }
    }

    if (best_error > VISION_PROTOCOL_COORD_TOLERANCE_MM) {
        return 0U;
    }

    *aligned = *incoming;
    for (offset = 0U; offset < incoming->vertex_count; ++offset) {
        int16_t mapped = (int16_t)best_offset +
                         (int16_t)best_direction * offset;
        while (mapped < 0) {
            mapped += incoming->vertex_count;
        }
        aligned->vertices[offset] =
            incoming->vertices[(uint8_t)(mapped % incoming->vertex_count)];
    }
    return 1U;
}

static uint8_t align_packet(const VisionProtocolPacket *reference,
                            const VisionProtocolPacket *incoming,
                            VisionProtocolPacket *aligned)
{
    uint8_t piece_index;

    if (reference->frame.piece_count != incoming->frame.piece_count) {
        return 0U;
    }

    *aligned = *incoming;
    sort_pieces_by_id(&aligned->frame);
    for (piece_index = 0U;
         piece_index < reference->frame.piece_count;
         ++piece_index) {
        DecisionPiece piece;

        if (align_piece(&reference->frame.pieces[piece_index],
                        &aligned->frame.pieces[piece_index],
                        &piece) == 0U) {
            return 0U;
        }
        aligned->frame.pieces[piece_index] = piece;
    }
    return 1U;
}

static void average_packet(VisionProtocolPacket *average,
                           const VisionProtocolPacket *sample,
                           uint8_t previous_count)
{
    float divisor = (float)previous_count + 1.0f;
    uint8_t piece_index;

    average->seq = sample->seq;
    average->frame.seq = sample->seq;
    for (piece_index = 0U;
         piece_index < average->frame.piece_count;
         ++piece_index) {
        DecisionPiece *target = &average->frame.pieces[piece_index];
        const DecisionPiece *source = &sample->frame.pieces[piece_index];
        uint8_t vertex_index;

        target->center.x_mm =
            (target->center.x_mm * previous_count + source->center.x_mm) /
            divisor;
        target->center.y_mm =
            (target->center.y_mm * previous_count + source->center.y_mm) /
            divisor;
        for (vertex_index = 0U;
             vertex_index < target->vertex_count;
             ++vertex_index) {
            target->vertices[vertex_index].x_mm =
                (target->vertices[vertex_index].x_mm * previous_count +
                 source->vertices[vertex_index].x_mm) / divisor;
            target->vertices[vertex_index].y_mm =
                (target->vertices[vertex_index].y_mm * previous_count +
                 source->vertices[vertex_index].y_mm) / divisor;
        }
    }
}

void VisionProtocolStabilizer_Init(VisionProtocolStabilizer *stabilizer)
{
    VisionProtocolStabilizer_Reset(stabilizer);
}

void VisionProtocolStabilizer_Reset(VisionProtocolStabilizer *stabilizer)
{
    if (stabilizer == NULL) {
        return;
    }
    (void)memset(stabilizer, 0, sizeof(*stabilizer));
}

uint8_t VisionProtocolStabilizer_Add(
    VisionProtocolStabilizer *stabilizer,
    const VisionProtocolPacket *packet,
    VisionProtocolPacket *stable_packet)
{
    VisionProtocolPacket aligned;

    if (stabilizer == NULL || packet == NULL || stable_packet == NULL) {
        return 0U;
    }

    if (stabilizer->has_candidate == 0U) {
        stabilizer->candidate = *packet;
        sort_pieces_by_id(&stabilizer->candidate.frame);
        stabilizer->stable_count = 1U;
        stabilizer->has_candidate = 1U;
        return 0U;
    }

    if (align_packet(&stabilizer->candidate, packet, &aligned) == 0U) {
        stabilizer->candidate = *packet;
        sort_pieces_by_id(&stabilizer->candidate.frame);
        stabilizer->stable_count = 1U;
        return 0U;
    }

    average_packet(&stabilizer->candidate,
                   &aligned,
                   stabilizer->stable_count);
    ++stabilizer->stable_count;
    if (stabilizer->stable_count < VISION_PROTOCOL_STABLE_FRAME_COUNT) {
        return 0U;
    }

    *stable_packet = stabilizer->candidate;
    return 1U;
}

size_t VisionProtocol_EncodeModeCommand(DecisionStrategy strategy,
                                        uint16_t seq,
                                        uint8_t *buffer,
                                        size_t capacity)
{
    uint8_t mode;
    uint16_t crc;

    if (buffer == NULL ||
        capacity < VISION_PROTOCOL_MODE_COMMAND_FRAME_LENGTH) {
        return 0U;
    }
    if (strategy == DECISION_STRATEGY_GEOMETRIC) {
        mode = VISION_PROTOCOL_MODE_GEOMETRIC;
    } else if (strategy == DECISION_STRATEGY_CARD_PATTERN) {
        mode = VISION_PROTOCOL_MODE_CARD_PATTERN;
    } else {
        return 0U;
    }

    buffer[0] = VISION_PROTOCOL_HEADER_FIRST;
    buffer[1] = VISION_PROTOCOL_HEADER_SECOND;
    buffer[2] = VISION_PROTOCOL_VERSION;
    buffer[3] = VISION_PROTOCOL_TYPE_MODE_COMMAND;
    write_u16_le(&buffer[4], seq);
    write_u16_le(&buffer[6], 1U);
    buffer[8] = mode;
    crc = VisionProtocol_Crc16(&buffer[2], 7U);
    write_u16_le(&buffer[9], crc);
    buffer[11] = VISION_PROTOCOL_END_FIRST;
    buffer[12] = VISION_PROTOCOL_END_SECOND;
    return VISION_PROTOCOL_MODE_COMMAND_FRAME_LENGTH;
}

static uint8_t card_take(const uint8_t *data,
                         uint16_t length,
                         uint16_t *position,
                         uint16_t count,
                         const uint8_t **value)
{
    if (data == NULL || position == NULL || value == NULL ||
        *position > length || count > (uint16_t)(length - *position)) {
        return 0U;
    }
    *value = &data[*position];
    *position = (uint16_t)(*position + count);
    return 1U;
}

static uint8_t decode_card_blob(const uint8_t *data,
                                uint16_t length,
                                uint32_t layout_id,
                                DecisionCardFrame *card_frame)
{
    DecisionCardFrame decoded;
    const uint8_t *value;
    uint16_t position = 0U;
    uint8_t piece_index;

    if (card_frame == NULL ||
        card_take(data, length, &position, 4U, &value) == 0U ||
        value[0] != VISION_PROTOCOL_CARD_FORMAT_VERSION ||
        value[1] == 0U || value[1] > DECISION_MAX_PIECES) {
        return 0U;
    }

    (void)memset(&decoded, 0, sizeof(decoded));
    decoded.layout_id = layout_id;
    decoded.piece_count = value[1];
    decoded.vision.piece_count = value[1];
    decoded.vision.seq = read_u16_le(&value[2]);

    for (piece_index = 0U; piece_index < decoded.piece_count;
         ++piece_index) {
        DecisionPiece *piece = &decoded.vision.pieces[piece_index];
        DecisionCardPieceFeatures *features = &decoded.pieces[piece_index];
        uint8_t vertex_index;
        uint8_t event_index;
        uint8_t primitive_index;
        uint8_t previous_index;

        if (card_take(data, length, &position, 6U, &value) == 0U) {
            return 0U;
        }
        piece->id = value[0];
        piece->vertex_count = value[1];
        features->piece_id = piece->id;
        if (piece->vertex_count < 3U ||
            piece->vertex_count > DECISION_MAX_VERTICES) {
            return 0U;
        }
        for (previous_index = 0U; previous_index < piece_index;
             ++previous_index) {
            if (decoded.vision.pieces[previous_index].id == piece->id) {
                return 0U;
            }
        }
        piece->center.x_mm = (float)read_i16_le(&value[2]) * 0.1f;
        piece->center.y_mm = (float)read_i16_le(&value[4]) * 0.1f;

        for (vertex_index = 0U; vertex_index < piece->vertex_count;
             ++vertex_index) {
            if (card_take(data, length, &position, 4U, &value) == 0U) {
                return 0U;
            }
            piece->vertices[vertex_index].x_mm =
                (float)read_i16_le(value) * 0.1f;
            piece->vertices[vertex_index].y_mm =
                (float)read_i16_le(&value[2]) * 0.1f;
        }

        if (card_take(data, length, &position, 1U, &value) == 0U ||
            value[0] > DECISION_CARD_MAX_EDGE_EVENTS_PER_PIECE) {
            return 0U;
        }
        features->edge_event_count = value[0];
        for (event_index = 0U;
             event_index < features->edge_event_count;
             ++event_index) {
            DecisionCardEdgeEvent *event =
                &features->edge_events[event_index];

            if (card_take(data, length, &position, 6U, &value) == 0U) {
                return 0U;
            }
            event->edge_index = value[0];
            event->position_q8 = value[1];
            event->color = (DecisionCardColor)value[2];
            event->tangent_deg = (int8_t)value[3];
            event->width_q4_mm = value[4];
            event->confidence = value[5];
            if (event->edge_index >= piece->vertex_count ||
                (event->color != DECISION_CARD_COLOR_RED &&
                 event->color != DECISION_CARD_COLOR_BLACK)) {
                return 0U;
            }
        }

        if (card_take(data, length, &position, 1U, &value) == 0U ||
            value[0] > DECISION_CARD_MAX_PRIMITIVES_PER_PIECE) {
            return 0U;
        }
        features->primitive_count = value[0];
        for (primitive_index = 0U;
             primitive_index < features->primitive_count;
             ++primitive_index) {
            DecisionCardPrimitive *primitive =
                &features->primitives[primitive_index];

            if (card_take(data, length, &position, 10U, &value) == 0U) {
                return 0U;
            }
            primitive->center.x_mm = (float)read_i16_le(value) * 0.1f;
            primitive->center.y_mm =
                (float)read_i16_le(&value[2]) * 0.1f;
            primitive->area_mm2 = (float)read_u16_le(&value[4]) * 0.1f;
            primitive->color = (DecisionCardColor)value[6];
            primitive->kind = (DecisionCardPrimitiveKind)value[7];
            primitive->angle_deg = (int8_t)value[8];
            primitive->confidence = value[9];
            if (primitive->color != DECISION_CARD_COLOR_RED &&
                primitive->color != DECISION_CARD_COLOR_BLACK) {
                return 0U;
            }
        }
    }

    if (position != length) {
        return 0U;
    }
    *card_frame = decoded;
    return 1U;
}

void VisionProtocolCardAssembler_Init(VisionProtocolCardAssembler *assembler)
{
    VisionProtocolCardAssembler_Reset(assembler);
}

void VisionProtocolCardAssembler_Reset(VisionProtocolCardAssembler *assembler)
{
    if (assembler != NULL) {
        (void)memset(assembler, 0, sizeof(*assembler));
    }
}

static uint8_t card_byte_received(const VisionProtocolCardAssembler *assembler,
                                  uint16_t position)
{
    return (uint8_t)(assembler->received[position >> 3U] &
                     (uint8_t)(1U << (position & 7U)));
}

static void mark_card_byte_received(VisionProtocolCardAssembler *assembler,
                                    uint16_t position)
{
    assembler->received[position >> 3U] |=
        (uint8_t)(1U << (position & 7U));
}

uint8_t VisionProtocolCardAssembler_Add(
    VisionProtocolCardAssembler *assembler,
    const VisionProtocolCardChunk *chunk,
    DecisionCardFrame *card_frame)
{
    uint16_t index;

    if (assembler == NULL || chunk == NULL || card_frame == NULL ||
        chunk->total_length == 0U ||
        chunk->total_length > VISION_PROTOCOL_CARD_MAX_SERIALIZED_LENGTH ||
        chunk->data_length > VISION_PROTOCOL_CARD_MAX_CHUNK_DATA ||
        chunk->offset > chunk->total_length ||
        chunk->data_length >
            (uint16_t)(chunk->total_length - chunk->offset)) {
        return 0U;
    }

    if (assembler->active == 0U ||
        assembler->layout_id != chunk->layout_id ||
        assembler->total_length != chunk->total_length ||
        assembler->full_crc != chunk->full_crc) {
        VisionProtocolCardAssembler_Reset(assembler);
        assembler->active = 1U;
        assembler->layout_id = chunk->layout_id;
        assembler->total_length = chunk->total_length;
        assembler->full_crc = chunk->full_crc;
    }

    for (index = 0U; index < chunk->data_length; ++index) {
        uint16_t destination = (uint16_t)(chunk->offset + index);

        if (card_byte_received(assembler, destination) != 0U) {
            if (assembler->data[destination] != chunk->data[index]) {
                VisionProtocolCardAssembler_Reset(assembler);
                return 0U;
            }
            continue;
        }
        assembler->data[destination] = chunk->data[index];
        mark_card_byte_received(assembler, destination);
        ++assembler->received_count;
    }

    if (assembler->received_count != assembler->total_length) {
        return 0U;
    }
    if (VisionProtocol_Crc16(assembler->data, assembler->total_length) !=
        assembler->full_crc ||
        decode_card_blob(assembler->data,
                         assembler->total_length,
                         assembler->layout_id,
                         card_frame) == 0U) {
        VisionProtocolCardAssembler_Reset(assembler);
        return 0U;
    }

    VisionProtocolCardAssembler_Reset(assembler);
    return 1U;
}

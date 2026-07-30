#include "vision_uart.h"

#include "decision_task.h"
#include "vision_protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"

#include <string.h>

#define VISION_UART_TASK_PERIOD_MS       1U
#define VISION_UART_FRAME_TIMEOUT_MS     50U
#define VISION_UART_TX_TIMEOUT_MS        20U
#define VISION_UART_RX_CHUNK_SIZE        VISION_PROTOCOL_MAX_FRAME_LENGTH
#define VISION_UART_RX_RING_SIZE         512U

static uint8_t VisionUart_RxChunk[VISION_UART_RX_CHUNK_SIZE];
static uint8_t VisionUart_RxRing[VISION_UART_RX_RING_SIZE];
static volatile uint16_t VisionUart_RxHead;
static volatile uint16_t VisionUart_RxTail;
static volatile uint8_t VisionUart_Receiving;
static volatile uint8_t VisionUart_RxOverflow;
static volatile uint32_t VisionUart_DroppedBytes;
static volatile VisionUartOutput VisionUart_Output;
static DecisionFixedLayout VisionUart_FixedLayout;
static volatile DecisionTaskRequest VisionUart_ArmedRequest;
static volatile uint32_t VisionUart_ArmedId;
static volatile uint8_t VisionUart_ArmPending;
static volatile uint8_t VisionUart_AbortRequested;

static void publish_output(const VisionUartOutput *output)
{
    taskENTER_CRITICAL();
    VisionUart_Output = *output;
    taskEXIT_CRITICAL();
}

static uint8_t ring_pop(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL) {
        return 0U;
    }

    tail = VisionUart_RxTail;
    if (tail == VisionUart_RxHead) {
        return 0U;
    }
    *byte = VisionUart_RxRing[tail];
    VisionUart_RxTail = (uint16_t)((tail + 1U) % VISION_UART_RX_RING_SIZE);
    return 1U;
}

static void start_receive(void)
{
    /* A previous run de-initialised USART1 after submitting, so bring the
       peripheral back up before every acquisition. */
    if (huart1.gState == HAL_UART_STATE_RESET) {
        MX_USART1_UART_Init();
    }

    taskENTER_CRITICAL();
    VisionUart_RxHead = 0U;
    VisionUart_RxTail = 0U;
    VisionUart_RxOverflow = 0U;
    taskEXIT_CRITICAL();

    VisionUart_Receiving = 1U;
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                   VisionUart_RxChunk,
                                   VISION_UART_RX_CHUNK_SIZE) != HAL_OK) {
        VisionUart_Receiving = 0U;
    }
}

static void stop_receive(void)
{
    VisionUart_Receiving = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
}

static void send_ack(uint16_t seq, VisionProtocolAckStatus status)
{
    uint8_t frame[VISION_PROTOCOL_ACK_FRAME_LENGTH];
    size_t length = VisionProtocol_EncodeAck(seq,
                                             status,
                                             frame,
                                             sizeof(frame));

    if (length != 0U) {
        (void)HAL_UART_Transmit(&huart1,
                               frame,
                               (uint16_t)length,
                               VISION_UART_TX_TIMEOUT_MS);
    }
}

void VisionUart_Init(void)
{
    VisionUartOutput output;

    VisionUart_RxHead = 0U;
    VisionUart_RxTail = 0U;
    VisionUart_Receiving = 0U;
    VisionUart_RxOverflow = 0U;
    VisionUart_DroppedBytes = 0U;
    VisionUart_ArmPending = 0U;
    VisionUart_AbortRequested = 0U;
    VisionUart_ArmedId = 0U;
    (void)memset(&VisionUart_FixedLayout, 0, sizeof(VisionUart_FixedLayout));
    (void)memset((void *)&VisionUart_ArmedRequest, 0,
                 sizeof(VisionUart_ArmedRequest));
    (void)memset(&output, 0, sizeof(output));
    output.state = VISION_UART_STATE_IDLE;
    VisionUart_Output = output;
}

uint8_t VisionUart_Arm(const DecisionTaskRequest *base_request, uint32_t arm_id)
{
    if (base_request == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    VisionUart_ArmedRequest = *base_request;
    VisionUart_ArmedId = arm_id;
    VisionUart_ArmPending = 1U;
    VisionUart_AbortRequested = 0U;
    taskEXIT_CRITICAL();

    return 1U;
}

void VisionUart_Abort(void)
{
    taskENTER_CRITICAL();
    VisionUart_ArmPending = 0U;
    VisionUart_AbortRequested = 1U;
    taskEXIT_CRITICAL();
}

uint8_t VisionUart_SetFixedLayout(const DecisionFixedLayout *layout)
{
    uint8_t index;

    if (layout == NULL || layout->piece_count == 0U ||
        layout->piece_count > DECISION_MAX_PIECES ||
        VisionUart_Receiving != 0U) {
        return 0U;
    }
    for (index = 0U; index < layout->piece_count; ++index) {
        uint8_t previous_index;

        if (layout->pieces[index].vertex_count < 3U ||
            layout->pieces[index].vertex_count > DECISION_MAX_VERTICES) {
            return 0U;
        }
        for (previous_index = 0U;
             previous_index < index;
             ++previous_index) {
            if (layout->pieces[previous_index].id == layout->pieces[index].id) {
                return 0U;
            }
        }
    }

    VisionUart_FixedLayout = *layout;
    return 1U;
}

void VisionUart_GetOutput(VisionUartOutput *output)
{
    if (output == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *output = VisionUart_Output;
    taskEXIT_CRITICAL();
}

void VisionUart_App(void *argument)
{
    VisionProtocolParser parser;
    VisionProtocolStabilizer stabilizer;
    VisionProtocolPacket packet;
    VisionProtocolPacket stable_packet;
    DecisionTaskRequest base_request;
    VisionUartOutput output;
    uint32_t last_byte_tick;
    uint8_t submitted = 1U;

    (void)argument;
    VisionProtocolParser_Init(&parser);
    VisionProtocolStabilizer_Init(&stabilizer);
    (void)memset(&base_request, 0, sizeof(base_request));
    VisionUart_GetOutput(&output);
    last_byte_tick = osKernelGetTickCount();

    for (;;) {
        uint8_t byte;

        /* Idle with the receiver off until a mission arms the acquisition, so
           frames arriving before the start key are ignored on purpose. */
        if (VisionUart_ArmPending != 0U) {
            taskENTER_CRITICAL();
            base_request = VisionUart_ArmedRequest;
            output.arm_id = VisionUart_ArmedId;
            VisionUart_ArmPending = 0U;
            VisionUart_AbortRequested = 0U;
            taskEXIT_CRITICAL();

            VisionProtocolParser_Reset(&parser);
            VisionProtocolStabilizer_Reset(&stabilizer);
            output.stable_count = 0U;
            submitted = 0U;
            last_byte_tick = osKernelGetTickCount();
            start_receive();
            output.state = VisionUart_Receiving != 0U
                ? VISION_UART_STATE_RECEIVING
                : VISION_UART_STATE_ERROR;
            publish_output(&output);
        }

        if (VisionUart_AbortRequested != 0U) {
            taskENTER_CRITICAL();
            VisionUart_AbortRequested = 0U;
            taskEXIT_CRITICAL();
            if (submitted == 0U) {
                stop_receive();
                submitted = 1U;
                output.state = VISION_UART_STATE_IDLE;
                output.stable_count = 0U;
                publish_output(&output);
            }
        }

        if (submitted != 0U) {
            osDelay(VISION_UART_TASK_PERIOD_MS);
            continue;
        }

        if (VisionUart_RxOverflow != 0U) {
            taskENTER_CRITICAL();
            VisionUart_RxOverflow = 0U;
            VisionUart_RxTail = VisionUart_RxHead;
            taskEXIT_CRITICAL();
            VisionProtocolParser_Reset(&parser);
            VisionProtocolStabilizer_Reset(&stabilizer);
            output.stable_count = 0U;
            ++output.invalid_frame_count;
            output.dropped_byte_count = VisionUart_DroppedBytes;
            publish_output(&output);
        }

        while (submitted == 0U && ring_pop(&byte) != 0U) {
            VisionProtocolResult result;

            last_byte_tick = osKernelGetTickCount();
            result = VisionProtocolParser_PushByte(&parser, byte, &packet);
            if (result == VISION_PROTOCOL_RESULT_FRAME) {
                ++output.valid_frame_count;
                output.last_seq = packet.seq;
                if (VisionProtocolStabilizer_Add(&stabilizer,
                                                 &packet,
                                                 &stable_packet) != 0U) {
                    DecisionTaskRequest request;

                    /* The mission that armed this run owns the mode, so a
                       fixed-ID run without a template is rejected outright. */
                    if (base_request.mode == DECISION_MODE_FIXED_ID &&
                        VisionUart_FixedLayout.piece_count == 0U) {
                        send_ack(packet.seq, VISION_PROTOCOL_ACK_INVALID);
                        VisionProtocolStabilizer_Reset(&stabilizer);
                        output.stable_count = 0U;
                        ++output.invalid_frame_count;
                        publish_output(&output);
                        continue;
                    }

                    output.stable_count = stabilizer.stable_count;
                    output.state = VISION_UART_STATE_STABLE;
                    publish_output(&output);

                    stop_receive();
                    send_ack(packet.seq, VISION_PROTOCOL_ACK_ACCEPTED);
                    (void)HAL_UART_DeInit(&huart1);

                    request = base_request;
                    request.vision = stable_packet.frame;
                    request.fixed_layout = VisionUart_FixedLayout;
                    if (DecisionTask_Submit(&request) != 0U) {
                        output.state = VISION_UART_STATE_SUBMITTED;
                    } else {
                        output.state = VISION_UART_STATE_ERROR;
                    }
                    submitted = 1U;
                } else {
                    output.stable_count = stabilizer.stable_count;
                    send_ack(packet.seq, VISION_PROTOCOL_ACK_OK);
                }
                publish_output(&output);
            } else if (result != VISION_PROTOCOL_RESULT_NONE) {
                VisionProtocolStabilizer_Reset(&stabilizer);
                output.stable_count = 0U;
                ++output.invalid_frame_count;
                publish_output(&output);
            }
        }

        if (submitted == 0U &&
            VisionProtocolParser_HasPartialFrame(&parser) != 0U &&
            (osKernelGetTickCount() - last_byte_tick) >=
                pdMS_TO_TICKS(VISION_UART_FRAME_TIMEOUT_MS)) {
            VisionProtocolParser_Reset(&parser);
            VisionProtocolStabilizer_Reset(&stabilizer);
            output.stable_count = 0U;
            ++output.invalid_frame_count;
            publish_output(&output);
        }

        osDelay(VISION_UART_TASK_PERIOD_MS);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t index;

    if (huart != &huart1 || VisionUart_Receiving == 0U) {
        return;
    }

    for (index = 0U; index < size; ++index) {
        uint16_t next = (uint16_t)((VisionUart_RxHead + 1U) %
                                   VISION_UART_RX_RING_SIZE);

        if (next == VisionUart_RxTail) {
            VisionUart_RxOverflow = 1U;
            VisionUart_DroppedBytes += (uint32_t)(size - index);
            break;
        }
        VisionUart_RxRing[VisionUart_RxHead] = VisionUart_RxChunk[index];
        VisionUart_RxHead = next;
    }

    if (VisionUart_Receiving != 0U) {
        if (HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                       VisionUart_RxChunk,
                                       VISION_UART_RX_CHUNK_SIZE) != HAL_OK) {
            VisionUart_Receiving = 0U;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 && VisionUart_Receiving != 0U) {
        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                         VisionUart_RxChunk,
                                         VISION_UART_RX_CHUNK_SIZE);
    }
}

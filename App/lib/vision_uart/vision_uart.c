#include "vision_uart.h"

#include "decision_task.h"
#include "vision_mode_retry.h"
#include "vision_protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"

#include <string.h>

#define VISION_UART_TASK_PERIOD_MS       1U
#define VISION_UART_FRAME_TIMEOUT_MS     50U
#define VISION_UART_TX_TIMEOUT_MS        20U
#define VISION_UART_MODE_RETRY_MS        100U
#define VISION_UART_RX_CHUNK_SIZE        VISION_PROTOCOL_MAX_FRAME_LENGTH
#define VISION_UART_RX_RING_SIZE         512U

static uint8_t VisionUart_RxChunk[VISION_UART_RX_CHUNK_SIZE];
static uint8_t VisionUart_RxRing[VISION_UART_RX_RING_SIZE];
static volatile uint16_t VisionUart_RxHead;
static volatile uint16_t VisionUart_RxTail;
static volatile uint8_t VisionUart_Receiving;
static volatile uint8_t VisionUart_RxOverflow;
static volatile uint32_t VisionUart_DroppedBytes;
static volatile uint32_t VisionUart_LineErrors;
static volatile VisionUartOutput VisionUart_Output;
static volatile DecisionTaskRequest VisionUart_ArmedRequest;
static volatile uint32_t VisionUart_ArmedId;
static volatile uint8_t VisionUart_ArmPending;
static volatile uint8_t VisionUart_AbortRequested;
/* Kept off the 3 KB vision task stack: a complete card payload and its
   reassembly buffer are both kilobyte-scale. */
static VisionProtocolCardAssembler VisionUart_CardAssembler;
static DecisionCardFrame VisionUart_CardDecoded;
static DecisionCardFrame VisionUart_CardCandidate;
static DecisionTaskRequest VisionUart_BaseRequest;

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

static uint8_t start_receive(void)
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
    VisionUart_DroppedBytes = 0U;
    VisionUart_LineErrors = 0U;
    taskEXIT_CRITICAL();

    VisionUart_Receiving = 1U;
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                   VisionUart_RxChunk,
                                   VISION_UART_RX_CHUNK_SIZE) != HAL_OK) {
        VisionUart_Receiving = 0U;
        return 0U;
    }
    return 1U;
}

static void close_receive(void)
{
    VisionUart_Receiving = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UART_DeInit(&huart1);

    taskENTER_CRITICAL();
    VisionUart_RxHead = 0U;
    VisionUart_RxTail = 0U;
    taskEXIT_CRITICAL();
}

static uint8_t send_mode_command(DecisionStrategy strategy, uint16_t seq)
{
    uint8_t frame[VISION_PROTOCOL_MODE_COMMAND_FRAME_LENGTH];
    size_t length = VisionProtocol_EncodeModeCommand(
        strategy, seq, frame, sizeof(frame));

    if (length == 0U) {
        return 0U;
    }
    return HAL_UART_Transmit(&huart1,
                             frame,
                             (uint16_t)length,
                             VISION_UART_TX_TIMEOUT_MS) == HAL_OK;
}

void VisionUart_Init(void)
{
    VisionUartOutput output;

    VisionUart_RxHead = 0U;
    VisionUart_RxTail = 0U;
    VisionUart_Receiving = 0U;
    VisionUart_RxOverflow = 0U;
    VisionUart_DroppedBytes = 0U;
    VisionUart_LineErrors = 0U;
    VisionUart_ArmPending = 0U;
    VisionUart_AbortRequested = 0U;
    VisionUart_ArmedId = 0U;
    (void)memset((void *)&VisionUart_ArmedRequest, 0,
                 sizeof(VisionUart_ArmedRequest));
    VisionProtocolCardAssembler_Init(&VisionUart_CardAssembler);
    (void)memset(&VisionUart_CardDecoded, 0, sizeof(VisionUart_CardDecoded));
    (void)memset(&VisionUart_CardCandidate, 0,
                 sizeof(VisionUart_CardCandidate));
    (void)memset(&VisionUart_BaseRequest, 0,
                 sizeof(VisionUart_BaseRequest));
    (void)memset(&output, 0, sizeof(output));
    output.state = VISION_UART_STATE_IDLE;
    VisionUart_Output = output;

    /* The start key owns the receive window. USART1 stays physically closed
       before the first acquisition just as it does between later runs. */
    (void)HAL_UART_DeInit(&huart1);
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
    VisionUartOutput output;
    uint32_t last_byte_tick;
    uint32_t card_layout_id = 0U;
    uint16_t card_full_crc = 0U;
    uint8_t card_stable_count = 0U;
    VisionModeRetry mode_retry;
    uint8_t submitted = 1U;

    (void)argument;
    VisionProtocolParser_Init(&parser);
    VisionProtocolStabilizer_Init(&stabilizer);
    VisionUart_GetOutput(&output);
    last_byte_tick = osKernelGetTickCount();

    for (;;) {
        uint8_t byte;

        /* Idle with the receiver off until a mission arms the acquisition, so
           frames arriving before the start key are ignored on purpose. */
        if (VisionUart_ArmPending != 0U) {
            taskENTER_CRITICAL();
            VisionUart_BaseRequest = VisionUart_ArmedRequest;
            output.arm_id = VisionUart_ArmedId;
            VisionUart_ArmPending = 0U;
            VisionUart_AbortRequested = 0U;
            taskEXIT_CRITICAL();

            VisionProtocolParser_Reset(&parser);
            VisionProtocolStabilizer_Reset(&stabilizer);
            VisionProtocolCardAssembler_Reset(&VisionUart_CardAssembler);
            card_layout_id = 0U;
            card_full_crc = 0U;
            card_stable_count = 0U;
            output.stable_count = 0U;
            output.mode_command_tx_count = 0U;
            output.mode_command_error_count = 0U;
            submitted = 0U;
            last_byte_tick = osKernelGetTickCount();
            if (start_receive() != 0U) {
                output.state = VISION_UART_STATE_RECEIVING;
                VisionModeRetry_Arm(&mode_retry, osKernelGetTickCount());
                if (send_mode_command(
                        VisionUart_BaseRequest.strategy,
                        (uint16_t)output.arm_id) != 0U) {
                    ++output.mode_command_tx_count;
                } else {
                    ++output.mode_command_error_count;
                }
            } else {
                close_receive();
                output.state = VISION_UART_STATE_ERROR;
                submitted = 1U;
            }
            publish_output(&output);
        }

        if (VisionUart_AbortRequested != 0U) {
            taskENTER_CRITICAL();
            VisionUart_AbortRequested = 0U;
            taskEXIT_CRITICAL();
            if (submitted == 0U) {
                close_receive();
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

        if (VisionModeRetry_TakeDue(
                &mode_retry,
                osKernelGetTickCount(),
                pdMS_TO_TICKS(VISION_UART_MODE_RETRY_MS)) != 0U) {
            if (send_mode_command(
                    VisionUart_BaseRequest.strategy,
                    (uint16_t)output.arm_id) != 0U) {
                ++output.mode_command_tx_count;
            } else {
                ++output.mode_command_error_count;
            }
            publish_output(&output);
        }

        if (VisionUart_RxOverflow != 0U) {
            taskENTER_CRITICAL();
            VisionUart_RxOverflow = 0U;
            VisionUart_RxTail = VisionUart_RxHead;
            taskEXIT_CRITICAL();
            VisionProtocolParser_Reset(&parser);
            VisionProtocolStabilizer_Reset(&stabilizer);
            VisionProtocolCardAssembler_Reset(&VisionUart_CardAssembler);
            card_stable_count = 0U;
            output.stable_count = 0U;
            ++output.invalid_frame_count;
            output.dropped_byte_count = VisionUart_DroppedBytes;
            publish_output(&output);
        }

        /* Published as it changes rather than only on failure. A link that is
           wired but mismatched produces nothing else to publish, so without
           this the count would still be sitting in the local copy when the
           mission times out and reads the output. */
        if (output.line_error_count != VisionUart_LineErrors) {
            output.line_error_count = VisionUart_LineErrors;
            publish_output(&output);
        }

        while (submitted == 0U && ring_pop(&byte) != 0U) {
            VisionProtocolResult result;

            last_byte_tick = osKernelGetTickCount();
            result = VisionProtocolParser_PushByte(&parser, byte, &packet);
            if (VisionModeRetry_ResultMatches(
                    VisionUart_BaseRequest.strategy, result) != 0U) {
                VisionModeRetry_Stop(&mode_retry);
            }
            if (result == VISION_PROTOCOL_RESULT_FRAME &&
                VisionUart_BaseRequest.strategy ==
                    DECISION_STRATEGY_GEOMETRIC) {
                ++output.valid_frame_count;
                output.last_seq = packet.seq;
                if (VisionProtocolStabilizer_Add(&stabilizer,
                                                 &packet,
                                                 &stable_packet) != 0U) {
                    output.stable_count = stabilizer.stable_count;
                    output.state = VISION_UART_STATE_STABLE;
                    publish_output(&output);

                    VisionUart_BaseRequest.vision = stable_packet.frame;
                    /* Close the receive window before handing the stable data
                       over. Any later bytes are ignored until the next arm. */
                    close_receive();
                    if (DecisionTask_Submit(&VisionUart_BaseRequest) != 0U) {
                        output.state = VISION_UART_STATE_SUBMITTED;
                    } else {
                        output.state = VISION_UART_STATE_ERROR;
                    }
                    submitted = 1U;
                } else {
                    output.stable_count = stabilizer.stable_count;
                }
                publish_output(&output);
            } else if (result == VISION_PROTOCOL_RESULT_CARD_CHUNK &&
                       VisionUart_BaseRequest.strategy ==
                           DECISION_STRATEGY_CARD_PATTERN) {
                ++output.valid_frame_count;
                output.last_seq = packet.seq;
                if (VisionProtocolCardAssembler_Add(
                        &VisionUart_CardAssembler,
                        &packet.card_chunk,
                        &VisionUart_CardDecoded) != 0U) {
                    if (card_layout_id == VisionUart_CardDecoded.layout_id &&
                        card_full_crc == packet.card_chunk.full_crc) {
                        if (card_stable_count < UINT8_MAX) {
                            ++card_stable_count;
                        }
                    } else {
                        VisionUart_CardCandidate = VisionUart_CardDecoded;
                        card_layout_id = VisionUart_CardDecoded.layout_id;
                        card_full_crc = packet.card_chunk.full_crc;
                        card_stable_count = 1U;
                    }
                    output.stable_count = card_stable_count;
                    if (card_stable_count >=
                        VISION_PROTOCOL_STABLE_FRAME_COUNT) {
                        output.state = VISION_UART_STATE_STABLE;
                        publish_output(&output);
                        VisionUart_BaseRequest.card =
                            VisionUart_CardCandidate;
                        VisionUart_BaseRequest.vision =
                            VisionUart_CardCandidate.vision;
                        close_receive();
                        if (DecisionTask_Submit(&VisionUart_BaseRequest) != 0U) {
                            output.state = VISION_UART_STATE_SUBMITTED;
                        } else {
                            output.state = VISION_UART_STATE_ERROR;
                        }
                        submitted = 1U;
                    }
                    publish_output(&output);
                }
            } else if (result != VISION_PROTOCOL_RESULT_NONE) {
                /* The camera may interleave geometric and card results. A
                   well-formed frame for the other strategy is ignored rather
                   than counted as a link error. */
                if (result != VISION_PROTOCOL_RESULT_FRAME &&
                    result != VISION_PROTOCOL_RESULT_CARD_CHUNK) {
                    VisionProtocolStabilizer_Reset(&stabilizer);
                    VisionProtocolCardAssembler_Reset(
                        &VisionUart_CardAssembler);
                    card_stable_count = 0U;
                    output.stable_count = 0U;
                    ++output.invalid_frame_count;
                    publish_output(&output);
                }
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

        /* HAL_UARTEx_ReceiveToIdle_IT can fail to re-arm inside the callback,
           which would silently stall the acquisition, so surface it. Only while
           the acquisition is still open: a finished one stops the receiver on
           purpose, and the `submitted` guard is what keeps that from being read
           as a stall and overwriting the state published a few lines above. */
        if (submitted == 0U && VisionUart_Receiving == 0U) {
            close_receive();
            output.state = VISION_UART_STATE_ERROR;
            submitted = 1U;
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
        /* Count before re-arming. Without this, a wire carrying garbage looks
           exactly like no wire at all: both leave the ring buffer empty. */
        ++VisionUart_LineErrors;
        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                         VisionUart_RxChunk,
                                         VISION_UART_RX_CHUNK_SIZE);
    }
}

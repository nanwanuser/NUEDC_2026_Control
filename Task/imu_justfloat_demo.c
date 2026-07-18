/**
 * @file imu_justfloat_demo.c
 * @brief ICM42688P 与 Fusion 姿态算法的 USART1 JustFloat 测试 Demo。
 */

#include "imu_justfloat_demo.h"

#include "app_imu.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>

#define IMU_DEMO_CHANNEL_COUNT (10U)
#define IMU_DEMO_FRAME_TAIL_SIZE (4U)
#define IMU_DEMO_TX_PERIOD_MS (10U)
#define IMU_DEMO_DMA_TIMEOUT_MS (10U)
#define IMU_DEMO_TX_BUFFER_SIZE ((IMU_DEMO_CHANNEL_COUNT * sizeof(float)) + IMU_DEMO_FRAME_TAIL_SIZE)

static const uint8_t s_justfloat_tail[IMU_DEMO_FRAME_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};
static uint8_t s_tx_buffer[IMU_DEMO_TX_BUFFER_SIZE];
static float s_channels[IMU_DEMO_CHANNEL_COUNT];
static app_imu_telemetry_t s_telemetry;
static uint32_t s_last_tx_ms;
static uint32_t s_dma_start_ms;
static uint8_t s_dma_pending;
static uint8_t s_use_dma;

_Static_assert(sizeof(float) == 4U, "JustFloat requires 32-bit float");

void IMUJustFloatDemo_Init(void)
{
    AppIMU_Init();
    s_last_tx_ms = HAL_GetTick();
    s_dma_start_ms = s_last_tx_ms;
    s_dma_pending = 0U;
    s_use_dma = 1U;
}

void IMUJustFloatDemo_Process(void)
{
    HAL_StatusTypeDef uart_status;
    uint32_t now_ms;

    AppIMU_Process();

    now_ms = HAL_GetTick();
    if ((s_use_dma != 0U) && (s_dma_pending != 0U))
    {
        if (huart1.gState == HAL_UART_STATE_READY)
        {
            s_dma_pending = 0U;
        }
        else if ((now_ms - s_dma_start_ms) >= IMU_DEMO_DMA_TIMEOUT_MS)
        {
            /* DMA/TC 中断链异常时解除 HAL BUSY，并永久降级为阻塞发送。 */
            (void)HAL_UART_AbortTransmit(&huart1);
            s_dma_pending = 0U;
            s_use_dma = 0U;
        }
        else
        {
            return;
        }
    }

    if ((now_ms - s_last_tx_ms) < IMU_DEMO_TX_PERIOD_MS)
    {
        return;
    }

    if (AppIMU_GetTelemetry(&s_telemetry) != 0U)
    {
        s_channels[0] = s_telemetry.euler_deg.roll_deg;
        s_channels[1] = s_telemetry.euler_deg.pitch_deg;
        s_channels[2] = s_telemetry.euler_deg.yaw_deg;
        s_channels[3] = s_telemetry.accel_g.x;
        s_channels[4] = s_telemetry.accel_g.y;
        s_channels[5] = s_telemetry.accel_g.z;
        s_channels[6] = s_telemetry.gyro_dps.x;
        s_channels[7] = s_telemetry.gyro_dps.y;
        s_channels[8] = s_telemetry.gyro_dps.z;
        s_channels[9] = s_telemetry.temperature_c;
    }
    else
    {
        memset(s_channels, 0, sizeof(s_channels));
        /* 负值 -(status + 1) 用于区分正常温度与初始化错误。 */
        s_channels[9] = -((float)AppIMU_GetInitStatus() + 1.0f);
    }

    /* STM32F407 为小端序，直接复制 IEEE-754 float 后追加 JustFloat 帧尾。 */
    memcpy(s_tx_buffer, s_channels, sizeof(s_channels));
    memcpy(&s_tx_buffer[sizeof(s_channels)], s_justfloat_tail, sizeof(s_justfloat_tail));

    if (s_use_dma != 0U)
    {
        uart_status = HAL_UART_Transmit_DMA(&huart1, s_tx_buffer, (uint16_t)sizeof(s_tx_buffer));
        if (uart_status == HAL_OK)
        {
            s_last_tx_ms = now_ms;
            s_dma_start_ms = now_ms;
            s_dma_pending = 1U;
            return;
        }

        s_use_dma = 0U;
    }

    uart_status = HAL_UART_Transmit(&huart1,
                                    s_tx_buffer,
                                    (uint16_t)sizeof(s_tx_buffer),
                                    IMU_DEMO_DMA_TIMEOUT_MS);
    if (uart_status == HAL_OK)
    {
        s_last_tx_ms = now_ms;
    }
}

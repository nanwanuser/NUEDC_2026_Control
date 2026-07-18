#include "icm42688p.h"

#include <stddef.h>

#define ICM42688_GYRO_CONFIG1_DEFAULT        ((uint8_t)0x68U)
#define ICM42688_GYRO_ACCEL_CONFIG0_DEFAULT  ((uint8_t)0x44U)
#define ICM42688_ACCEL_CONFIG1_DEFAULT       ((uint8_t)0x10U)
#define ICM42688_TMST_CONFIG_DEFAULT         ((uint8_t)0x15U)

#define ICM42688_GYRO_AAF_STATIC2_DEFAULT    ((uint8_t)0x00U)
#define ICM42688_GYRO_AAF_STATIC3_DEFAULT    ((uint8_t)0x03U)
#define ICM42688_GYRO_AAF_STATIC4_DEFAULT    ((uint8_t)0x09U)
#define ICM42688_GYRO_AAF_STATIC5_DEFAULT    ((uint8_t)0xC0U)

#define ICM42688_ACCEL_AAF_STATIC2_DEFAULT   ((uint8_t)0x06U)
#define ICM42688_ACCEL_AAF_STATIC3_DEFAULT   ((uint8_t)0x09U)
#define ICM42688_ACCEL_AAF_STATIC4_DEFAULT   ((uint8_t)0xC0U)

#define ICM42688_SPI_DMA_BUFFER_SIZE         ((uint16_t)256U)

typedef enum
{
    ICM42688_SPI_DMA_IDLE = 0,
    ICM42688_SPI_DMA_BUSY,
    ICM42688_SPI_DMA_DONE,
    ICM42688_SPI_DMA_ERROR
} icm42688_spi_dma_state_t;

static icm42688_port_t s_icm42688_port = {0};
static icm42688_config_t s_icm42688_config = {0};
static float s_icm42688_accel_sensitivity = 0.0f;
static float s_icm42688_gyro_sensitivity = 0.0f;
static uint8_t s_icm42688_ready = 0U;
static uint8_t s_icm42688_data_ready = 0U;
static volatile icm42688_spi_dma_state_t s_icm42688_spi_dma_state = ICM42688_SPI_DMA_IDLE;
static volatile uint32_t s_icm42688_spi_dma_error = HAL_SPI_ERROR_NONE;
static uint8_t s_icm42688_spi_tx_buffer[ICM42688_SPI_DMA_BUFFER_SIZE + 1U] = {0};
static uint8_t s_icm42688_spi_rx_buffer[ICM42688_SPI_DMA_BUFFER_SIZE + 1U] = {0};

/**
 * @brief 将 HAL 返回值转换为驱动状态码。
 * @param hal_status HAL 接口返回值。
 * @return 驱动状态码。
 */
static icm42688_status_t ICM42688_HalToStatus(HAL_StatusTypeDef hal_status)
{
    if (hal_status == HAL_OK)
    {
        return ICM42688_STATUS_OK;
    }

    if (hal_status == HAL_TIMEOUT)
    {
        return ICM42688_STATUS_TIMEOUT;
    }

    return ICM42688_STATUS_ERROR;
}

/**
 * @brief 毫秒级延时。
 * @param delay_ms 延时时间，单位 ms。
 */
static void ICM42688_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/**
 * @brief 拉低片选，开始一次 SPI 事务。
 */
static void ICM42688_CsSelect(void)
{
    HAL_GPIO_WritePin(s_icm42688_port.cs_port, s_icm42688_port.cs_pin, GPIO_PIN_RESET);

    for (volatile uint32_t i = 0U; i < 64U; i++)
    {
        __NOP();
    }
}

/**
 * @brief 拉高片选，结束一次 SPI 事务。
 */
static void ICM42688_CsDeselect(void)
{
    for (volatile uint32_t i = 0U; i < 64U; i++)
    {
        __NOP();
    }

    HAL_GPIO_WritePin(s_icm42688_port.cs_port, s_icm42688_port.cs_pin, GPIO_PIN_SET);
}

/**
 * @brief 等待一次 SPI DMA 事务完成。
 * @return 事务结果状态。
 */
static icm42688_status_t ICM42688_WaitForDmaTransfer(void)
{
    uint32_t start_tick = HAL_GetTick();

    while (s_icm42688_spi_dma_state == ICM42688_SPI_DMA_BUSY)
    {
        if ((HAL_GetTick() - start_tick) >= s_icm42688_port.timeout_ms)
        {
            (void)HAL_SPI_Abort(s_icm42688_port.hspi);
            s_icm42688_spi_dma_state = ICM42688_SPI_DMA_IDLE;
            s_icm42688_spi_dma_error = HAL_SPI_ERROR_NONE;
            return ICM42688_STATUS_TIMEOUT;
        }
    }

    if (s_icm42688_spi_dma_state == ICM42688_SPI_DMA_DONE)
    {
        s_icm42688_spi_dma_state = ICM42688_SPI_DMA_IDLE;
        s_icm42688_spi_dma_error = HAL_SPI_ERROR_NONE;
        return ICM42688_STATUS_OK;
    }

    s_icm42688_spi_dma_state = ICM42688_SPI_DMA_IDLE;
    s_icm42688_spi_dma_error = HAL_SPI_ERROR_NONE;
    return ICM42688_STATUS_ERROR;
}

/**
 * @brief 通过 DMA 执行一次完整 SPI 全双工事务。
 * @param tx_buffer DMA 发送缓冲区。
 * @param rx_buffer DMA 接收缓冲区。
 * @param size 事务总字节数。
 * @return 事务结果状态。
 */
static icm42688_status_t ICM42688_SpiTransferDma(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size)
{
    HAL_StatusTypeDef hal_status;

    if ((tx_buffer == NULL) || (rx_buffer == NULL) || (size == 0U))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    s_icm42688_spi_dma_error = HAL_SPI_ERROR_NONE;
    s_icm42688_spi_dma_state = ICM42688_SPI_DMA_BUSY;

    hal_status = HAL_SPI_TransmitReceive_DMA(s_icm42688_port.hspi, tx_buffer, rx_buffer, size);
    if (hal_status != HAL_OK)
    {
        s_icm42688_spi_dma_state = ICM42688_SPI_DMA_IDLE;
        return ICM42688_HalToStatus(hal_status);
    }

    return ICM42688_WaitForDmaTransfer();
}

/**
 * @brief 读取一个或多个寄存器。
 * @param reg 起始寄存器地址。
 * @param buffer 数据接收缓冲区。
 * @param length 读取字节数。
 * @return 读取结果状态。
 */
static icm42688_status_t ICM42688_BusRead(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    uint8_t reg_addr;
    uint16_t index;
    icm42688_status_t status;

    if ((buffer == NULL) || (length == 0U) || (length > ICM42688_SPI_DMA_BUFFER_SIZE))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    reg_addr = reg | ICM42688_SPI_READ_MASK;
    s_icm42688_spi_tx_buffer[0] = reg_addr;
    for (index = 0U; index < length; index++)
    {
        s_icm42688_spi_tx_buffer[index + 1U] = 0xFFU;
    }

    ICM42688_CsSelect();

    status = ICM42688_SpiTransferDma(s_icm42688_spi_tx_buffer,
                                     s_icm42688_spi_rx_buffer,
                                     (uint16_t)(length + 1U));
    if (status == ICM42688_STATUS_OK)
    {
        for (index = 0U; index < length; index++)
        {
            buffer[index] = s_icm42688_spi_rx_buffer[index + 1U];
        }
    }

    ICM42688_CsDeselect();

    return status;
}

/**
 * @brief 写入一个或多个寄存器。
 * @param reg 起始寄存器地址。
 * @param buffer 待写入数据缓冲区。
 * @param length 写入字节数。
 * @return 写入结果状态。
 */
static icm42688_status_t ICM42688_BusWrite(uint8_t reg, const uint8_t *buffer, uint16_t length)
{
    uint8_t reg_addr;
    icm42688_status_t status;

    if ((buffer == NULL) || (length == 0U) || (length > ICM42688_SPI_DMA_BUFFER_SIZE))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    reg_addr = reg & (uint8_t)(~ICM42688_SPI_READ_MASK);
    s_icm42688_spi_tx_buffer[0] = reg_addr;
    for (uint16_t i = 0U; i < length; i++)
    {
        s_icm42688_spi_tx_buffer[i + 1U] = buffer[i];
    }

    ICM42688_CsSelect();

    status = ICM42688_SpiTransferDma(s_icm42688_spi_tx_buffer,
                                     s_icm42688_spi_rx_buffer,
                                     (uint16_t)(length + 1U));
    ICM42688_CsDeselect();

    return status;
}

/**
 * @brief 检查硬件端口配置是否有效。
 * @param port 端口配置指针。
 * @return 检查结果状态。
 */
static icm42688_status_t ICM42688_CheckPort(const icm42688_port_t *port)
{
    if ((port == NULL) || (port->hspi == NULL) || (port->cs_port == NULL) || (port->timeout_ms == 0U))
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    return ICM42688_STATUS_OK;
}

/**
 * @brief 检查传感器参数配置是否有效。
 * @param config 配置结构体指针。
 * @return 检查结果状态。
 */
static icm42688_status_t ICM42688_CheckConfig(const icm42688_config_t *config)
{
    if (config == NULL)
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    switch (config->accel_fs)
    {
        case ICM42688_ACCEL_FS_16G:
        case ICM42688_ACCEL_FS_8G:
        case ICM42688_ACCEL_FS_4G:
        case ICM42688_ACCEL_FS_2G:
            break;

        default:
            return ICM42688_STATUS_BAD_PARAM;
    }

    switch (config->gyro_fs)
    {
        case ICM42688_GYRO_FS_2000DPS:
        case ICM42688_GYRO_FS_1000DPS:
        case ICM42688_GYRO_FS_500DPS:
        case ICM42688_GYRO_FS_250DPS:
        case ICM42688_GYRO_FS_125DPS:
        case ICM42688_GYRO_FS_62DPS:
        case ICM42688_GYRO_FS_31DPS:
        case ICM42688_GYRO_FS_15DPS:
            break;

        default:
            return ICM42688_STATUS_BAD_PARAM;
    }

    switch (config->accel_odr)
    {
        case ICM42688_ODR_32KHZ:
        case ICM42688_ODR_16KHZ:
        case ICM42688_ODR_8KHZ:
        case ICM42688_ODR_4KHZ:
        case ICM42688_ODR_2KHZ:
        case ICM42688_ODR_1KHZ:
        case ICM42688_ODR_200HZ:
        case ICM42688_ODR_100HZ:
        case ICM42688_ODR_50HZ:
        case ICM42688_ODR_25HZ:
        case ICM42688_ODR_12_5HZ:
        case ICM42688_ODR_500HZ:
            break;

        default:
            return ICM42688_STATUS_BAD_PARAM;
    }

    switch (config->gyro_odr)
    {
        case ICM42688_ODR_32KHZ:
        case ICM42688_ODR_16KHZ:
        case ICM42688_ODR_8KHZ:
        case ICM42688_ODR_4KHZ:
        case ICM42688_ODR_2KHZ:
        case ICM42688_ODR_1KHZ:
        case ICM42688_ODR_200HZ:
        case ICM42688_ODR_100HZ:
        case ICM42688_ODR_50HZ:
        case ICM42688_ODR_25HZ:
        case ICM42688_ODR_12_5HZ:
        case ICM42688_ODR_500HZ:
            break;

        default:
            return ICM42688_STATUS_BAD_PARAM;
    }

    return ICM42688_STATUS_OK;
}

/**
 * @brief 将高低字节拼接为有符号 16 位数据。
 * @param msb 高字节。
 * @param lsb 低字节。
 * @return 拼接后的 16 位数据。
 */
static int16_t ICM42688_CombineInt16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

/**
 * @brief 根据当前量程更新灵敏度参数。
 */
static void ICM42688_UpdateSensitivity(void)
{
    switch (s_icm42688_config.accel_fs)
    {
        case ICM42688_ACCEL_FS_2G:
            s_icm42688_accel_sensitivity = 16384.0f;
            break;

        case ICM42688_ACCEL_FS_4G:
            s_icm42688_accel_sensitivity = 8192.0f;
            break;

        case ICM42688_ACCEL_FS_8G:
            s_icm42688_accel_sensitivity = 4096.0f;
            break;

        case ICM42688_ACCEL_FS_16G:
        default:
            s_icm42688_accel_sensitivity = 2048.0f;
            break;
    }

    switch (s_icm42688_config.gyro_fs)
    {
        case ICM42688_GYRO_FS_15DPS:
            s_icm42688_gyro_sensitivity = 2165.2f;
            break;

        case ICM42688_GYRO_FS_31DPS:
            s_icm42688_gyro_sensitivity = 1065.3f;
            break;

        case ICM42688_GYRO_FS_62DPS:
            s_icm42688_gyro_sensitivity = 532.9f;
            break;

        case ICM42688_GYRO_FS_125DPS:
            s_icm42688_gyro_sensitivity = 265.0f;
            break;

        case ICM42688_GYRO_FS_250DPS:
            s_icm42688_gyro_sensitivity = 131.0f;
            break;

        case ICM42688_GYRO_FS_500DPS:
            s_icm42688_gyro_sensitivity = 65.5f;
            break;

        case ICM42688_GYRO_FS_1000DPS:
            s_icm42688_gyro_sensitivity = 32.8f;
            break;

        case ICM42688_GYRO_FS_2000DPS:
        default:
            s_icm42688_gyro_sensitivity = 16.4f;
            break;
    }
}

/**
 * @brief 切换寄存器 Bank。
 * @param bank 目标 Bank 号。
 * @return 切换结果状态。
 */
static icm42688_status_t ICM42688_SelectBank(uint8_t bank)
{
    return ICM42688_BusWrite(ICM42688_REG_REG_BANK_SEL, &bank, 1U);
}

/**
 * @brief 写入滤波器和接口相关配置。
 * @return 配置结果状态。
 */
static icm42688_status_t ICM42688_ApplyFilterConfig(void)
{
    icm42688_status_t status;
    uint8_t value;

    value = 0x05U;
    status = ICM42688_BusWrite(ICM42688_REG_DRIVE_CONFIG, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_GYRO_CONFIG1_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_CONFIG1, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_GYRO_ACCEL_CONFIG0_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_ACCEL_CONFIG0, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_ACCEL_CONFIG1_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_ACCEL_CONFIG1, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_TMST_CONFIG_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_TMST_CONFIG, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_SelectBank(1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = 0x82U;
    status = ICM42688_BusWrite(ICM42688_REG_INTF_CONFIG4, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = 0x13U;
    status = ICM42688_BusWrite(ICM42688_REG_INTF_CONFIG6, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_GYRO_AAF_STATIC2_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_CONFIG_STATIC2, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_GYRO_AAF_STATIC3_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_CONFIG_STATIC3, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_GYRO_AAF_STATIC4_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_CONFIG_STATIC4, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_GYRO_AAF_STATIC5_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_CONFIG_STATIC5, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_SelectBank(2U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_ACCEL_AAF_STATIC2_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_ACCEL_CONFIG_STATIC2, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_ACCEL_AAF_STATIC3_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_ACCEL_CONFIG_STATIC3, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_ACCEL_AAF_STATIC4_DEFAULT;
    status = ICM42688_BusWrite(ICM42688_REG_ACCEL_CONFIG_STATIC4, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    return ICM42688_SelectBank(0U);
}

void ICM42688_GetDefaultPort(icm42688_port_t *port)
{
    if (port == NULL)
    {
        return;
    }

    port->hspi = ICM42688_DEFAULT_SPI_HANDLE;
    port->cs_port = ICM42688_DEFAULT_CS_PORT;
    port->cs_pin = ICM42688_DEFAULT_CS_PIN;
    port->timeout_ms = ICM42688_DEFAULT_TIMEOUT_MS;
}

void ICM42688_GetDefaultConfig(icm42688_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->gyro_fs = ICM42688_GYRO_FS_2000DPS;
    config->accel_fs = ICM42688_ACCEL_FS_16G;
    config->gyro_odr = ICM42688_ODR_1KHZ;
    config->accel_odr = ICM42688_ODR_1KHZ;
}

icm42688_status_t ICM42688_Init(const icm42688_port_t *port, const icm42688_config_t *config)
{
    icm42688_port_t default_port;
    icm42688_config_t default_config;
    const icm42688_port_t *active_port = port;
    const icm42688_config_t *active_config = config;
    icm42688_status_t status;
    uint8_t device_id = 0U;
    uint8_t value;

    if (active_port == NULL)
    {
        ICM42688_GetDefaultPort(&default_port);
        active_port = &default_port;
    }

    if (active_config == NULL)
    {
        ICM42688_GetDefaultConfig(&default_config);
        active_config = &default_config;
    }

    status = ICM42688_CheckPort(active_port);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_CheckConfig(active_config);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    s_icm42688_port = *active_port;
    s_icm42688_config = *active_config;
    s_icm42688_ready = 0U;
    s_icm42688_data_ready = 0U;
    s_icm42688_spi_dma_state = ICM42688_SPI_DMA_IDLE;
    s_icm42688_spi_dma_error = HAL_SPI_ERROR_NONE;

    HAL_GPIO_WritePin(s_icm42688_port.cs_port, s_icm42688_port.cs_pin, GPIO_PIN_SET);

    ICM42688_DelayMs(10U);

    status = ICM42688_SelectBank(0U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_GetDeviceId(&device_id);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    if (device_id != ICM42688_WHO_AM_I_VALUE)
    {
        return ICM42688_STATUS_WHOAMI_ERROR;
    }

    status = ICM42688_Reset();
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_GetDeviceId(&device_id);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    if (device_id != ICM42688_WHO_AM_I_VALUE)
    {
        return ICM42688_STATUS_WHOAMI_ERROR;
    }

    value = ICM42688_PWR_MGMT0_GYRO_MODE_LN | ICM42688_PWR_MGMT0_ACCEL_MODE_LN;
    status = ICM42688_BusWrite(ICM42688_REG_PWR_MGMT0, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    ICM42688_DelayMs(10U);

    value = 0x03U;
    status = ICM42688_BusWrite(ICM42688_REG_INT_CONFIG, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = 0x91U;
    status = ICM42688_BusWrite(ICM42688_REG_INTF_CONFIG1, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = 0x00U;
    status = ICM42688_BusWrite(ICM42688_REG_INT_CONFIG1, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = ICM42688_INT_SOURCE0_UI_DRDY_INT1_EN;
    status = ICM42688_BusWrite(ICM42688_REG_INT_SOURCE0, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = (uint8_t)s_icm42688_config.gyro_fs | (uint8_t)s_icm42688_config.gyro_odr;
    status = ICM42688_BusWrite(ICM42688_REG_GYRO_CONFIG0, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    value = (uint8_t)s_icm42688_config.accel_fs | (uint8_t)s_icm42688_config.accel_odr;
    status = ICM42688_BusWrite(ICM42688_REG_ACCEL_CONFIG0, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_ApplyFilterConfig();
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    ICM42688_UpdateSensitivity();
    s_icm42688_ready = 1U;

    return ICM42688_STATUS_OK;
}

uint8_t ICM42688_IsReady(void)
{
    return s_icm42688_ready;
}

icm42688_status_t ICM42688_GetDeviceId(uint8_t *device_id)
{
    if (device_id == NULL)
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    if (ICM42688_CheckPort(&s_icm42688_port) != ICM42688_STATUS_OK)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    return ICM42688_BusRead(ICM42688_REG_WHO_AM_I, device_id, 1U);
}

icm42688_status_t ICM42688_ReadRegister(uint8_t reg, uint8_t *data)
{
    if (ICM42688_CheckPort(&s_icm42688_port) != ICM42688_STATUS_OK)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    return ICM42688_BusRead(reg, data, 1U);
}

icm42688_status_t ICM42688_WriteRegister(uint8_t reg, uint8_t data)
{
    if (ICM42688_CheckPort(&s_icm42688_port) != ICM42688_STATUS_OK)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    return ICM42688_BusWrite(reg, &data, 1U);
}

icm42688_status_t ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    if (ICM42688_CheckPort(&s_icm42688_port) != ICM42688_STATUS_OK)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    return ICM42688_BusRead(reg, buffer, length);
}

icm42688_status_t ICM42688_WriteRegisters(uint8_t reg, const uint8_t *buffer, uint16_t length)
{
    if (ICM42688_CheckPort(&s_icm42688_port) != ICM42688_STATUS_OK)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    return ICM42688_BusWrite(reg, buffer, length);
}

icm42688_status_t ICM42688_ReadRaw(icm42688_raw_t *raw)
{
    uint8_t buffer[14];
    icm42688_status_t status;

    if (raw == NULL)
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    if (!s_icm42688_ready)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    status = ICM42688_BusRead(ICM42688_REG_TEMP_DATA1, buffer, sizeof(buffer));
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    raw->temperature = ICM42688_CombineInt16(buffer[0], buffer[1]);
    raw->accel_x = ICM42688_CombineInt16(buffer[2], buffer[3]);
    raw->accel_y = ICM42688_CombineInt16(buffer[4], buffer[5]);
    raw->accel_z = ICM42688_CombineInt16(buffer[6], buffer[7]);
    raw->gyro_x = ICM42688_CombineInt16(buffer[8], buffer[9]);
    raw->gyro_y = ICM42688_CombineInt16(buffer[10], buffer[11]);
    raw->gyro_z = ICM42688_CombineInt16(buffer[12], buffer[13]);

    return ICM42688_STATUS_OK;
}

icm42688_status_t ICM42688_ReadScaled(icm42688_scaled_t *data)
{
    icm42688_raw_t raw;
    icm42688_status_t status;

    if (data == NULL)
    {
        return ICM42688_STATUS_BAD_PARAM;
    }

    status = ICM42688_ReadRaw(&raw);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    data->temperature_c = ((float)raw.temperature / 132.48f) + 25.0f;
    data->accel_x_g = (float)raw.accel_x / s_icm42688_accel_sensitivity;
    data->accel_y_g = (float)raw.accel_y / s_icm42688_accel_sensitivity;
    data->accel_z_g = (float)raw.accel_z / s_icm42688_accel_sensitivity;
    data->gyro_x_dps = (float)raw.gyro_x / s_icm42688_gyro_sensitivity;
    data->gyro_y_dps = (float)raw.gyro_y / s_icm42688_gyro_sensitivity;
    data->gyro_z_dps = (float)raw.gyro_z / s_icm42688_gyro_sensitivity;

    return ICM42688_STATUS_OK;
}

icm42688_status_t ICM42688_Reset(void)
{
    icm42688_status_t status;
    uint8_t value = ICM42688_DEVICE_SOFT_RESET;

    if (ICM42688_CheckPort(&s_icm42688_port) != ICM42688_STATUS_OK)
    {
        return ICM42688_STATUS_NOT_INIT;
    }

    status = ICM42688_SelectBank(0U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    status = ICM42688_BusWrite(ICM42688_REG_DEVICE_CONFIG, &value, 1U);
    if (status != ICM42688_STATUS_OK)
    {
        return status;
    }

    ICM42688_DelayMs(2U);

    return ICM42688_SelectBank(0U);
}

float ICM42688_GetAccelSensitivity(void)
{
    return s_icm42688_accel_sensitivity;
}

float ICM42688_GetGyroSensitivity(void)
{
    return s_icm42688_gyro_sensitivity;
}

void ICM42688_Int1IrqHandler(void)
{
    s_icm42688_data_ready = 1U;
}

uint8_t ICM42688_GetDataReadyFlag(void)
{
    return s_icm42688_data_ready;
}

void ICM42688_ClearDataReadyFlag(void)
{
    s_icm42688_data_ready = 0U;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != NULL) && (hspi == s_icm42688_port.hspi))
    {
        s_icm42688_spi_dma_state = ICM42688_SPI_DMA_DONE;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != NULL) && (hspi == s_icm42688_port.hspi))
    {
        s_icm42688_spi_dma_error = hspi->ErrorCode;
        s_icm42688_spi_dma_state = ICM42688_SPI_DMA_ERROR;
    }
}

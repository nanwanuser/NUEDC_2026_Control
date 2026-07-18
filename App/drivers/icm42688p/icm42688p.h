#ifndef ICM42688P_H
#define ICM42688P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "imu_board.h"

#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

#define ICM42688_DEFAULT_SPI_HANDLE   (&hspi1)
#define ICM42688_DEFAULT_CS_PORT      ICM42688P_CS_GPIO_Port
#define ICM42688_DEFAULT_CS_PIN       ICM42688P_CS_Pin
#define ICM42688_DEFAULT_TIMEOUT_MS   ((uint32_t)100U)

/* Bank 0 */
#define ICM42688_REG_DEVICE_CONFIG          ((uint8_t)0x11U)
#define ICM42688_REG_DRIVE_CONFIG           ((uint8_t)0x13U)
#define ICM42688_REG_INT_CONFIG             ((uint8_t)0x14U)
#define ICM42688_REG_TEMP_DATA1             ((uint8_t)0x1DU)
#define ICM42688_REG_TEMP_DATA0             ((uint8_t)0x1EU)
#define ICM42688_REG_ACCEL_DATA_X1          ((uint8_t)0x1FU)
#define ICM42688_REG_ACCEL_DATA_X0          ((uint8_t)0x20U)
#define ICM42688_REG_ACCEL_DATA_Y1          ((uint8_t)0x21U)
#define ICM42688_REG_ACCEL_DATA_Y0          ((uint8_t)0x22U)
#define ICM42688_REG_ACCEL_DATA_Z1          ((uint8_t)0x23U)
#define ICM42688_REG_ACCEL_DATA_Z0          ((uint8_t)0x24U)
#define ICM42688_REG_GYRO_DATA_X1           ((uint8_t)0x25U)
#define ICM42688_REG_GYRO_DATA_X0           ((uint8_t)0x26U)
#define ICM42688_REG_GYRO_DATA_Y1           ((uint8_t)0x27U)
#define ICM42688_REG_GYRO_DATA_Y0           ((uint8_t)0x28U)
#define ICM42688_REG_GYRO_DATA_Z1           ((uint8_t)0x29U)
#define ICM42688_REG_GYRO_DATA_Z0           ((uint8_t)0x2AU)
#define ICM42688_REG_SIGNAL_PATH_RESET      ((uint8_t)0x4BU)
#define ICM42688_REG_INTF_CONFIG0           ((uint8_t)0x4CU)
#define ICM42688_REG_INTF_CONFIG1           ((uint8_t)0x4DU)
#define ICM42688_REG_PWR_MGMT0              ((uint8_t)0x4EU)
#define ICM42688_REG_GYRO_CONFIG0           ((uint8_t)0x4FU)
#define ICM42688_REG_ACCEL_CONFIG0          ((uint8_t)0x50U)
#define ICM42688_REG_GYRO_CONFIG1           ((uint8_t)0x51U)
#define ICM42688_REG_GYRO_ACCEL_CONFIG0     ((uint8_t)0x52U)
#define ICM42688_REG_ACCEL_CONFIG1          ((uint8_t)0x53U)
#define ICM42688_REG_TMST_CONFIG            ((uint8_t)0x54U)
#define ICM42688_REG_WHO_AM_I               ((uint8_t)0x75U)
#define ICM42688_REG_REG_BANK_SEL           ((uint8_t)0x76U)
#define ICM42688_REG_INT_CONFIG0            ((uint8_t)0x63U)
#define ICM42688_REG_INT_CONFIG1            ((uint8_t)0x64U)
#define ICM42688_REG_INT_SOURCE0            ((uint8_t)0x65U)

/* Bank 1 */
#define ICM42688_REG_GYRO_CONFIG_STATIC2    ((uint8_t)0x0BU)
#define ICM42688_REG_GYRO_CONFIG_STATIC3    ((uint8_t)0x0CU)
#define ICM42688_REG_GYRO_CONFIG_STATIC4    ((uint8_t)0x0DU)
#define ICM42688_REG_GYRO_CONFIG_STATIC5    ((uint8_t)0x0EU)
#define ICM42688_REG_INTF_CONFIG4           ((uint8_t)0x7AU)
#define ICM42688_REG_INTF_CONFIG5           ((uint8_t)0x7BU)
#define ICM42688_REG_INTF_CONFIG6           ((uint8_t)0x7CU)

/* Bank 2 */
#define ICM42688_REG_ACCEL_CONFIG_STATIC2   ((uint8_t)0x03U)
#define ICM42688_REG_ACCEL_CONFIG_STATIC3   ((uint8_t)0x04U)
#define ICM42688_REG_ACCEL_CONFIG_STATIC4   ((uint8_t)0x05U)

#define ICM42688_WHO_AM_I_VALUE             ((uint8_t)0x47U)
#define ICM42688_SPI_READ_MASK              ((uint8_t)0x80U)
#define ICM42688_DEVICE_SOFT_RESET          ((uint8_t)0x01U)
#define ICM42688_SIGNAL_PATH_ABORT_AND_FLUSH ((uint8_t)0x0AU)
#define ICM42688_PWR_MGMT0_ACCEL_MODE_LN    ((uint8_t)0x03U)
#define ICM42688_PWR_MGMT0_GYRO_MODE_LN     ((uint8_t)0x0CU)
#define ICM42688_INT_SOURCE0_UI_DRDY_INT1_EN ((uint8_t)0x08U)

typedef enum
{
    ICM42688_STATUS_OK = 0,
    ICM42688_STATUS_ERROR,
    ICM42688_STATUS_TIMEOUT,
    ICM42688_STATUS_BAD_PARAM,
    ICM42688_STATUS_NOT_INIT,
    ICM42688_STATUS_WHOAMI_ERROR
} icm42688_status_t;

typedef enum
{
    ICM42688_GYRO_FS_2000DPS = 0x00U,
    ICM42688_GYRO_FS_1000DPS = 0x20U,
    ICM42688_GYRO_FS_500DPS  = 0x40U,
    ICM42688_GYRO_FS_250DPS  = 0x60U,
    ICM42688_GYRO_FS_125DPS  = 0x80U,
    ICM42688_GYRO_FS_62DPS   = 0xA0U,
    ICM42688_GYRO_FS_31DPS   = 0xC0U,
    ICM42688_GYRO_FS_15DPS   = 0xE0U
} icm42688_gyro_fs_t;

typedef enum
{
    ICM42688_ACCEL_FS_16G = 0x00U,
    ICM42688_ACCEL_FS_8G  = 0x20U,
    ICM42688_ACCEL_FS_4G  = 0x40U,
    ICM42688_ACCEL_FS_2G  = 0x60U
} icm42688_accel_fs_t;

typedef enum
{
    ICM42688_ODR_32KHZ  = 0x01U,
    ICM42688_ODR_16KHZ  = 0x02U,
    ICM42688_ODR_8KHZ   = 0x03U,
    ICM42688_ODR_4KHZ   = 0x04U,
    ICM42688_ODR_2KHZ   = 0x05U,
    ICM42688_ODR_1KHZ   = 0x06U,
    ICM42688_ODR_200HZ  = 0x07U,
    ICM42688_ODR_100HZ  = 0x08U,
    ICM42688_ODR_50HZ   = 0x09U,
    ICM42688_ODR_25HZ   = 0x0AU,
    ICM42688_ODR_12_5HZ = 0x0BU,
    ICM42688_ODR_500HZ  = 0x0FU
} icm42688_odr_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t timeout_ms;
} icm42688_port_t;

typedef struct
{
    icm42688_gyro_fs_t gyro_fs;
    icm42688_accel_fs_t accel_fs;
    icm42688_odr_t gyro_odr;
    icm42688_odr_t accel_odr;
} icm42688_config_t;

typedef struct
{
    int16_t temperature;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm42688_raw_t;

typedef struct
{
    float temperature_c;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} icm42688_scaled_t;

void ICM42688_GetDefaultPort(icm42688_port_t *port);
void ICM42688_GetDefaultConfig(icm42688_config_t *config);
icm42688_status_t ICM42688_Init(const icm42688_port_t *port, const icm42688_config_t *config);
uint8_t ICM42688_IsReady(void);
icm42688_status_t ICM42688_GetDeviceId(uint8_t *device_id);
icm42688_status_t ICM42688_ReadRegister(uint8_t reg, uint8_t *data);
icm42688_status_t ICM42688_WriteRegister(uint8_t reg, uint8_t data);
icm42688_status_t ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length);
icm42688_status_t ICM42688_WriteRegisters(uint8_t reg, const uint8_t *buffer, uint16_t length);
icm42688_status_t ICM42688_ReadRaw(icm42688_raw_t *raw);
icm42688_status_t ICM42688_ReadScaled(icm42688_scaled_t *data);
icm42688_status_t ICM42688_Reset(void);
float ICM42688_GetAccelSensitivity(void);
float ICM42688_GetGyroSensitivity(void);
void ICM42688_Int1IrqHandler(void);
uint8_t ICM42688_GetDataReadyFlag(void);
void ICM42688_ClearDataReadyFlag(void);

#ifdef __cplusplus
}
#endif

#endif

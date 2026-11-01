#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum
{
    BMP280_READ_CHIP_ID,
    BMP280_RESET_SENSOR,
    BMP280_READ_STARTUP_STATUS,
    BMP280_READ_CALIBRATION,
    BMP280_VERIFY_CALIBRATION,
    BMP280_WRITE_CONFIGURATION,
    BMP280_START_MEASUREMENT,
    BMP280_READ_MEASUREMENT_STATUS,
    BMP280_READ_MEASUREMENT,
    BMP280_VERIFY_MEASUREMENT
} BMP280_State;

typedef enum
{
    BMP280_ERROR_NONE,
    BMP280_ERROR_CONFIG,
    BMP280_ERROR_NACK,
    BMP280_ERROR_I2C,
    BMP280_ERROR_TIMEOUT,
    BMP280_ERROR_BUS_STUCK,
    BMP280_ERROR_CHIP_ID,
    BMP280_ERROR_SENSOR_BUSY,
    BMP280_ERROR_CALIBRATION,
    BMP280_ERROR_CONFIG_MISMATCH,
    BMP280_ERROR_DATA_MISMATCH,
    BMP280_ERROR_INVALID_SAMPLE,
    BMP280_ERROR_RANGE
} BMP280_Error;

typedef struct
{
    float temperature_c;
    float pressure_pa;
    uint32_t sample_ms;
    BMP280_Error error;
    BMP280_State state;
    uint8_t i2c_address;
    uint8_t last_chip_id;
    bool valid;
    bool enabled;
} BMP280_Data;

extern BMP280_Data bmp280;

bool BMP280_Init(I2C_HandleTypeDef *hi2c);
void BMP280_Task(void);
void BMP280_SetEnabled(bool enabled);

#endif

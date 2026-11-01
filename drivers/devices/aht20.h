#ifndef AHT20_H
#define AHT20_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum
{
    AHT20_RESET_SENSOR,
    AHT20_READ_STARTUP_STATUS,
    AHT20_INITIALIZE_SENSOR,
    AHT20_READ_INIT_STATUS,
    AHT20_START_MEASUREMENT,
    AHT20_READ_MEASUREMENT_STATUS,
    AHT20_READ_MEASUREMENT
} AHT20_State;
typedef enum
{
    AHT20_ERROR_NONE,
    AHT20_ERROR_CONFIG,
    AHT20_ERROR_NACK,
    AHT20_ERROR_I2C,
    AHT20_ERROR_TIMEOUT,
    AHT20_ERROR_BUS_STUCK,
    AHT20_ERROR_SENSOR_BUSY,
    AHT20_ERROR_CALIBRATION,
    AHT20_ERROR_CRC,
    AHT20_ERROR_RANGE
} AHT20_Error;

typedef struct
{
    float temperature_c;
    float humidity_percent;
    uint32_t sample_ms;
    AHT20_Error error;
    AHT20_State state;
    bool valid;
    bool enabled;
} AHT20_Data;

extern AHT20_Data aht20;

bool AHT20_Init(I2C_HandleTypeDef *hi2c);

void AHT20_Task(void);

void AHT20_SetEnabled(bool enabled);

#endif

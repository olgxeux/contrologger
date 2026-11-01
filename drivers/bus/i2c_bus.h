#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#if USE_HAL_I2C_REGISTER_CALLBACKS != 0
#error "This manager uses the standard weak HAL callbacks; select REGISTER_CALLBACKS=0"
#endif
#define I2C_BUS_SCL_PORT GPIOB
#define I2C_BUS_SCL_PIN GPIO_PIN_8
#define I2C_BUS_SDA_PORT GPIOB
#define I2C_BUS_SDA_PIN GPIO_PIN_9

typedef enum
{
    I2C_BUS_IDLE,
    I2C_BUS_PENDING,
    I2C_BUS_OK,
    I2C_BUS_NACK,
    I2C_BUS_ERROR,
    I2C_BUS_TIMEOUT,
    I2C_BUS_STUCK,
    I2C_BUS_CONFIG
} I2C_BusResult;

typedef struct
{
    uint8_t address;
    const uint8_t *tx;
    uint16_t tx_size;
    uint8_t *rx;
    uint16_t rx_size;
    uint32_t timeout_ms;
    I2C_BusResult result;
} I2C_BusRequest;

bool I2C_BusInit(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef I2C_BusStart(I2C_BusRequest *request);

void I2C_BusTask(void);

I2C_BusResult I2C_BusGetStatus(void);

uint32_t I2C_BusGetRecoveryCount(void);

#endif

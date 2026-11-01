#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define POT_RAW_MIN 0U
#define POT_RAW_MAX 4095U
#define POT_SAMPLE_INTERVAL_MS 20U
#define POT_STALE_MS 100U

typedef enum
{
    POT_ERROR_NONE,
    POT_ERROR_CONFIG,
    POT_ERROR_ADC,
    POT_ERROR_TIMEOUT
} Pot_Error;

typedef enum
{
    POT_WAIT_SAMPLE,
    POT_WAIT_CONVERSION,
    POT_RETRY_ADC
} Pot_State;

typedef struct
{
    uint16_t raw;
    uint16_t filtered_raw;
    uint16_t position_permille;
    uint32_t sample_ms;
    Pot_Error error;
    Pot_State state;
    bool enabled;
    bool valid;
} Pot_Data;

extern Pot_Data potentiometer;

bool Pot_Init(ADC_HandleTypeDef *adc);
void Pot_Task(void);
void Pot_SetEnabled(bool enabled);

#endif

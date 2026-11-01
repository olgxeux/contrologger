#include "potentiometer.h"

#define CONVERSION_TIMEOUT_MS 5U
#define ADC_RETRY_MS 1000U

_Static_assert(POT_RAW_MIN < POT_RAW_MAX && POT_RAW_MAX <= 4095U, "Invalid potentiometer calibration");

Pot_Data potentiometer;
static ADC_HandleTypeDef *pot_adc;
static uint32_t state_since_ms;
static uint32_t wait_ms;
static bool filter_ready;

static void adc_failed(Pot_Error error, uint32_t now)
{
    potentiometer.valid = false;
    filter_ready = false;
    potentiometer.error = error;
    if (HAL_ADC_Stop(pot_adc) != HAL_OK)
    {
        potentiometer.error = POT_ERROR_ADC;
    }
    potentiometer.state = POT_RETRY_ADC;
    state_since_ms = now;
    wait_ms = ADC_RETRY_MS;
}

static bool prepare_adc(uint32_t now)
{
    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = ADC_CHANNEL_0;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_Stop(pot_adc) != HAL_OK || HAL_ADC_DeInit(pot_adc) != HAL_OK || HAL_ADC_Init(pot_adc) != HAL_OK ||
        HAL_ADC_ConfigChannel(pot_adc, &channel) != HAL_OK)
    {
        adc_failed(POT_ERROR_ADC, now);
        return false;
    }
    __HAL_ADC_ENABLE(pot_adc);
    potentiometer.state = POT_WAIT_SAMPLE;
    state_since_ms = now;
    wait_ms = 2U;
    return true;
}

bool Pot_Init(ADC_HandleTypeDef *adc)
{
    if (pot_adc != NULL)
    {
        return false;
    }
    potentiometer.error = POT_ERROR_CONFIG;
    if (adc == NULL || adc->Instance != ADC1 || adc->Init.ClockPrescaler != ADC_CLOCK_SYNC_PCLK_DIV4 ||
        adc->Init.Resolution != ADC_RESOLUTION_12B || adc->Init.DataAlign != ADC_DATAALIGN_RIGHT ||
        adc->Init.ScanConvMode != DISABLE || adc->Init.ContinuousConvMode != DISABLE ||
        adc->Init.DiscontinuousConvMode != DISABLE || adc->Init.NbrOfConversion != 1U ||
        adc->Init.ExternalTrigConv != ADC_SOFTWARE_START ||
        adc->Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_NONE || adc->Init.DMAContinuousRequests != DISABLE ||
        adc->Init.EOCSelection != ADC_EOC_SINGLE_CONV || HAL_RCC_GetPCLK2Freq() / 4U > 36000000U)
    {
        return false;
    }
    pot_adc = adc;
    potentiometer.enabled = true;
    potentiometer.error = POT_ERROR_NONE;
    (void)prepare_adc(HAL_GetTick());
    return true;
}

void Pot_SetEnabled(bool enabled)
{
    if (pot_adc == NULL || potentiometer.enabled == enabled)
    {
        return;
    }
    potentiometer.enabled = enabled;
    potentiometer.valid = false;
    potentiometer.error = POT_ERROR_NONE;
    filter_ready = false;
    if (!enabled)
    {
        if (HAL_ADC_Stop(pot_adc) != HAL_OK)
        {
            potentiometer.error = POT_ERROR_ADC;
        }
    }
    else
    {
        (void)prepare_adc(HAL_GetTick());
    }
}

static void accept_sample(uint16_t raw, uint32_t now)
{
    potentiometer.raw = raw;
    if (!filter_ready)
    {
        potentiometer.filtered_raw = raw;
        filter_ready = true;
    }
    else
    {
        potentiometer.filtered_raw = (uint16_t)((3U * potentiometer.filtered_raw + raw + 2U) / 4U);
    }
    uint32_t value = potentiometer.filtered_raw;
    if (value <= POT_RAW_MIN)
    {
        potentiometer.position_permille = 0U;
    }
    else if (value >= POT_RAW_MAX)
    {
        potentiometer.position_permille = 1000U;
    }
    else
    {
        potentiometer.position_permille =
            (uint16_t)(((value - POT_RAW_MIN) * 1000U + (POT_RAW_MAX - POT_RAW_MIN) / 2U) / (POT_RAW_MAX - POT_RAW_MIN)
            );
    }
    potentiometer.sample_ms = now;
    potentiometer.valid = true;
    potentiometer.error = POT_ERROR_NONE;
}

void Pot_Task(void)
{
    if (pot_adc == NULL || !potentiometer.enabled)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    if (potentiometer.state != POT_RETRY_ADC && HAL_ADC_GetError(pot_adc) != HAL_ADC_ERROR_NONE)
    {
        adc_failed(POT_ERROR_ADC, now);
        return;
    }
    if (potentiometer.valid && (uint32_t)(now - potentiometer.sample_ms) > POT_STALE_MS)
    {
        adc_failed(POT_ERROR_TIMEOUT, now);
        return;
    }

    switch (potentiometer.state)
    {
        case POT_RETRY_ADC:
            if ((uint32_t)(now - state_since_ms) >= wait_ms)
            {
                (void)prepare_adc(now);
            }
            return;

        case POT_WAIT_SAMPLE:
            if ((uint32_t)(now - state_since_ms) < wait_ms)
            {
                return;
            }

            if ((pot_adc->Instance->CR2 & ADC_CR2_ADON) == 0U || HAL_ADC_Start(pot_adc) != HAL_OK ||
                HAL_ADC_GetError(pot_adc) != HAL_ADC_ERROR_NONE)
            {
                adc_failed(POT_ERROR_ADC, now);
                return;
            }
            potentiometer.state = POT_WAIT_CONVERSION;
            state_since_ms = now;
            return;

        case POT_WAIT_CONVERSION:
            if (__HAL_ADC_GET_FLAG(pot_adc, ADC_FLAG_OVR) != RESET)
            {
                adc_failed(POT_ERROR_ADC, now);
                return;
            }
            if ((uint32_t)(now - state_since_ms) >= CONVERSION_TIMEOUT_MS)
            {
                adc_failed(POT_ERROR_TIMEOUT, now);
                return;
            }
            if (__HAL_ADC_GET_FLAG(pot_adc, ADC_FLAG_EOC) == RESET)
            {
                return;
            }
            if (HAL_ADC_PollForConversion(pot_adc, 0U) != HAL_OK)
            {
                adc_failed(POT_ERROR_ADC, now);
                return;
            }
            uint32_t raw = HAL_ADC_GetValue(pot_adc);
            if (raw > 4095U)
            {
                adc_failed(POT_ERROR_ADC, now);
                return;
            }
            accept_sample((uint16_t)raw, now);
            potentiometer.state = POT_WAIT_SAMPLE;
            state_since_ms = now;
            wait_ms = POT_SAMPLE_INTERVAL_MS;
            return;
    }
}

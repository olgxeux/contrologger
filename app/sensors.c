#include "sensors.h"
#include "app_state.h"
#include "aht20.h"
#include "bmp280.h"
#include "i2c.h"
static uint32_t aht_since, bmp_since;
void sensors_init(void)
{
    aht_since = bmp_since = HAL_GetTick();
    (void)AHT20_Init(&hi2c1);
    (void)BMP280_Init(&hi2c1);
}
bool aht20_fresh(void)
{
    return aht20.enabled && aht20.valid && (uint32_t)(HAL_GetTick() - aht20.sample_ms) < 2500U;
}
bool bmp280_fresh(void)
{
    return bmp280.enabled && bmp280.valid && (uint32_t)(HAL_GetTick() - bmp280.sample_ms) < 2500U;
}
void sensors_process(void)
{
    if (app_settings.aht20_on && !aht20.enabled)
        aht_since = HAL_GetTick();
    if (app_settings.bmp280_on && !bmp280.enabled)
        bmp_since = HAL_GetTick();
    AHT20_SetEnabled(app_settings.aht20_on);
    BMP280_SetEnabled(app_settings.bmp280_on);
    AHT20_Task();
    BMP280_Task();
}
bool sensors_fault(void)
{
    bool aht_expired = (uint32_t)(HAL_GetTick() - aht_since) > 5000U;
    bool bmp_expired = (uint32_t)(HAL_GetTick() - bmp_since) > 5000U;
    return (app_settings.aht20_on && (aht20.error != AHT20_ERROR_NONE || (aht_expired && !aht20_fresh()))) ||
           (app_settings.bmp280_on && (bmp280.error != BMP280_ERROR_NONE || (bmp_expired && !bmp280_fresh())));
}

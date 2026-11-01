#include "servo.h"
#include "app_state.h"
#include "potentiometer.h"
#include "sg90.h"
#include "adc.h"
#include "tim.h"
static bool parking = true, zero_reached, was_on;
static uint32_t zero_since, sample_ms;
static bool have_sample;
void servo_init(void)
{
    (void)Pot_Init(&hadc1);
    (void)SG90_Init(&htim3);
}
bool servo_parking(void)
{
    return parking;
}
bool servo_fault(void)
{
    return sg90.error != SG90_ERROR_NONE || (app_settings.servo_on && potentiometer.error != POT_ERROR_NONE);
}
void servo_process(void)
{
    uint32_t now = HAL_GetTick();
    if (app_settings.servo_on)
    {
        parking = false;
        zero_reached = false;
        was_on = true;
        Pot_SetEnabled(true);
        Pot_Task();
        (void)SG90_SetLimits(app_settings.servo_min, app_settings.servo_max);
        SG90_SetEnabled(true);
        if (!potentiometer.valid || (uint32_t)(now - potentiometer.sample_ms) > POT_STALE_MS)
        {
            SG90_ClearTarget();
            have_sample = false;
        }
        else if (!have_sample || sample_ms != potentiometer.sample_ms)
        {
            uint32_t span = (uint32_t)app_settings.servo_max - app_settings.servo_min;
            (void)SG90_SetTargetAngle((uint16_t)(app_settings.servo_min +
                                                 (span * potentiometer.position_permille + 500U) / 1000U));
            sample_ms = potentiometer.sample_ms;
            have_sample = true;
        }
    }
    else
    {
        Pot_SetEnabled(false);
        have_sample = false;
        if (was_on)
        {
            parking = true;
            zero_reached = false;
            was_on = false;
        }
        if (parking)
        {
            (void)SG90_SetLimits(0U, 180U);
            SG90_SetEnabled(true);
            (void)SG90_SetTargetAngle(0U);
        }
        else
            SG90_SetEnabled(false);
    }
    SG90_Task();
    if (parking)
    {
        if (sg90.state == SG90_PWM && sg90.command_angle_deg == 0U)
        {
            if (!zero_reached)
            {
                zero_reached = true;
                zero_since = now;
            }
            if ((uint32_t)(now - zero_since) >= 500U)
            {
                SG90_SetEnabled(false);
                parking = false;
            }
        }
        else
            zero_reached = false;
    }
}

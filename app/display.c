#include "display.h"
#include "app_state.h"
#include "sensors.h"
#include "servo.h"
#include "ssd1306.h"
#include "aht20.h"
#include "bmp280.h"
#include "sg90.h"
#include "potentiometer.h"
#include "format.h"
#include "i2c.h"
#include <stdio.h>
static uint32_t last_draw, started;
static uint8_t previous_mode;
static bool previous_on;
void display_init(void)
{
    started = HAL_GetTick();
    (void)SSD1306_Init(&hi2c1);
}
bool display_fault(void)
{
    return app_settings.display_on &&
           (ssd1306.error != SSD1306_ERROR_NONE || (!ssd1306.ready && (uint32_t)(HAL_GetTick() - started) > 5000U));
}
static void
sensor_line(uint8_t row, const char *name, bool enabled, bool valid, float temperature, float second, const char *unit)
{
    char line[48], t[16], v[16];
    if (!enabled || !valid)
    {
        (void)snprintf(line, sizeof(line), "%s %s", name, enabled ? "NO DATA" : "OFF");
    }
    else
    {
        format_number(t, sizeof(t), temperature);
        format_number(v, sizeof(v), second);
        (void)snprintf(line, sizeof(line), "%s %sC %s%s", name, t, v, unit);
    }
    (void)SSD1306_WriteLine(row, line);
}
static void servo_line(uint8_t row)
{
    char line[32];
    if (servo_fault())
        (void)snprintf(line, sizeof(line), "SERVO ERROR");
    else if (servo_parking())
        (void)snprintf(line, sizeof(line), "PARK cmd=%u", (unsigned)sg90.command_angle_deg);
    else if (!app_settings.servo_on)
        (void)snprintf(line, sizeof(line), "SERVO OFF");
    else if (sg90.state != SG90_PWM)
        (void)snprintf(line, sizeof(line), "SERVO WAIT INPUT");
    else
        (void)snprintf(line, sizeof(line), "SERVO cmd=%u", (unsigned)sg90.command_angle_deg);
    (void)SSD1306_WriteLine(row, line);
}
void display_process(void)
{
    if (app_settings.display_on && !ssd1306.enabled)
        started = HAL_GetTick();
    SSD1306_SetEnabled(app_settings.display_on);
    uint32_t now = HAL_GetTick();
    if (app_settings.display_on &&
        (!previous_on || previous_mode != app_settings.display_mode || (uint32_t)(now - last_draw) >= 250U))
    {
        last_draw = now;
        previous_mode = app_settings.display_mode;
        SSD1306_Clear();
        uint8_t mode = app_settings.display_mode;
        if (mode == 1U)
        {
            (void)SSD1306_WriteLine(0U, app_flash_error ? "FLASH ERROR" : "CONTROLOGGER");
            sensor_line(
                1U,
                "A",
                app_settings.aht20_on,
                aht20_fresh(),
                aht20.temperature_c,
                aht20.humidity_percent,
                "%"
            );
            sensor_line(
                2U,
                "B",
                app_settings.bmp280_on,
                bmp280_fresh(),
                bmp280.temperature_c,
                bmp280.pressure_pa / 100.0f,
                "hPa"
            );
            servo_line(3U);
#if SSD1306_HEIGHT == 64U
            char text[32];
            (void)snprintf(text, sizeof(text), "POT raw=%u", (unsigned)potentiometer.raw);
            SSD1306_WriteLine(4U, text);
            (void)snprintf(
                text,
                sizeof(text),
                "LIMIT %u..%u",
                (unsigned)app_settings.servo_min,
                (unsigned)app_settings.servo_max
            );
            SSD1306_WriteLine(5U, text);
            (void)snprintf(text, sizeof(text), "PULSE %u us", (unsigned)sg90.pulse_us);
            SSD1306_WriteLine(6U, text);
            SSD1306_WriteLine(7U, "CMD NOT FEEDBACK");
#endif
        }
        else if (mode == 2U || mode == 3U)
        {
            bool a = mode == 2U;
            SSD1306_WriteLine(0U, a ? "AHT20" : "BMP280");
            bool enabled = a ? app_settings.aht20_on : app_settings.bmp280_on;
            bool valid = a ? aht20_fresh() : bmp280_fresh();
            if (!enabled || !valid)
                SSD1306_WriteLine(1U, enabled ? "NO DATA" : "OFF");
            else
            {
                char number[16], text[32];
                format_number(number, sizeof(number), a ? aht20.temperature_c : bmp280.temperature_c);
                (void)snprintf(text, sizeof(text), "TEMP %s C", number);
                SSD1306_WriteLine(1U, text);
                format_number(number, sizeof(number), a ? aht20.humidity_percent : bmp280.pressure_pa / 100.0f);
                (void)snprintf(text, sizeof(text), "%s %s %s", a ? "RH" : "P", number, a ? "%" : "hPa");
                SSD1306_WriteLine(2U, text);
            }
        }
        else
        {
            char text[32];
            servo_line(0U);
            (void)snprintf(text, sizeof(text), "TARGET %u deg", (unsigned)sg90.target_angle_deg);
            SSD1306_WriteLine(1U, text);
            (void)snprintf(
                text,
                sizeof(text),
                "LIMIT %u..%u",
                (unsigned)app_settings.servo_min,
                (unsigned)app_settings.servo_max
            );
            SSD1306_WriteLine(2U, text);
            (void)snprintf(text, sizeof(text), "PULSE %u us", (unsigned)sg90.pulse_us);
            SSD1306_WriteLine(3U, text);
        }
        SSD1306_Refresh();
    }
    previous_on = app_settings.display_on;
    SSD1306_Task();
}

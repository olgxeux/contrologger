#include "log.h"
#include "app_state.h"
#include "sensors.h"
#include "servo.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "potentiometer.h"
#include "sg90.h"
#include "i2c_bus.h"
#include "usart.h"
#include "format.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#define QUEUE_SIZE 2048U
static char queue[QUEUE_SIZE];
static uint32_t head, tail, dropped, last_log, last_tick;
static uint64_t uptime;
static bool ready, first = true;
void log_init(void)
{
    NVIC_DisableIRQ(USART1_IRQn);
    huart1.Instance->CR1 &=
        ~(uint32_t)(USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE);
    huart1.Instance->CR3 &= ~(uint32_t)(USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    ready = true;
    last_tick = HAL_GetTick();
    uptime = last_tick;
}
void log_write(const char *format, ...)
{
    char line[192];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(line, sizeof(line) - 2U, format, args);
    va_end(args);
    if (n < 0 || (size_t)n >= sizeof(line) - 2U)
    {
        ++dropped;
        return;
    }
    size_t size = (size_t)n;
    line[size++] = '\r';
    line[size++] = '\n';
    uint32_t free_bytes = (tail + QUEUE_SIZE - head - 1U) % QUEUE_SIZE;
    if (size > free_bytes)
    {
        ++dropped;
        return;
    }
    for (size_t i = 0; i < size; ++i)
    {
        queue[head] = line[i];
        head = (head + 1U) % QUEUE_SIZE;
    }
}
bool log_idle(void)
{
    return head == tail && (!ready || (huart1.Instance->SR & USART_SR_TC) != 0U);
}
static const char *aht20_error_name(AHT20_Error error)
{
    switch (error)
    {
        case AHT20_ERROR_NONE:
            return "NONE";
        case AHT20_ERROR_CONFIG:
            return "CONFIG";
        case AHT20_ERROR_NACK:
            return "NACK";
        case AHT20_ERROR_I2C:
            return "I2C";
        case AHT20_ERROR_TIMEOUT:
            return "TIMEOUT";
        case AHT20_ERROR_BUS_STUCK:
            return "BUS_STUCK";
        case AHT20_ERROR_SENSOR_BUSY:
            return "SENSOR_BUSY";
        case AHT20_ERROR_CALIBRATION:
            return "CALIBRATION";
        case AHT20_ERROR_CRC:
            return "CRC";
        case AHT20_ERROR_RANGE:
            return "RANGE";
        default:
            return "UNKNOWN";
    }
}

static const char *bmp280_error_name(BMP280_Error error)
{
    switch (error)
    {
        case BMP280_ERROR_NONE:
            return "NONE";
        case BMP280_ERROR_CONFIG:
            return "CONFIG";
        case BMP280_ERROR_NACK:
            return "NACK";
        case BMP280_ERROR_I2C:
            return "I2C";
        case BMP280_ERROR_TIMEOUT:
            return "TIMEOUT";
        case BMP280_ERROR_BUS_STUCK:
            return "BUS_STUCK";
        case BMP280_ERROR_CHIP_ID:
            return "CHIP_ID";
        case BMP280_ERROR_SENSOR_BUSY:
            return "SENSOR_BUSY";
        case BMP280_ERROR_CALIBRATION:
            return "CALIBRATION";
        case BMP280_ERROR_CONFIG_MISMATCH:
            return "CONFIG_MISMATCH";
        case BMP280_ERROR_DATA_MISMATCH:
            return "DATA_MISMATCH";
        case BMP280_ERROR_INVALID_SAMPLE:
            return "INVALID_SAMPLE";
        case BMP280_ERROR_RANGE:
            return "RANGE";
        default:
            return "UNKNOWN";
    }
}

static const char *ssd1306_error_name(SSD1306_Error error)
{
    switch (error)
    {
        case SSD1306_ERROR_NONE:
            return "NONE";
        case SSD1306_ERROR_CONFIG:
            return "CONFIG";
        case SSD1306_ERROR_NACK:
            return "NACK";
        case SSD1306_ERROR_I2C:
            return "I2C";
        case SSD1306_ERROR_TIMEOUT:
            return "TIMEOUT";
        case SSD1306_ERROR_BUS_STUCK:
            return "BUS_STUCK";
        default:
            return "UNKNOWN";
    }
}

static const char *pot_error_name(Pot_Error error)
{
    switch (error)
    {
        case POT_ERROR_NONE:
            return "NONE";
        case POT_ERROR_CONFIG:
            return "CONFIG";
        case POT_ERROR_ADC:
            return "ADC";
        case POT_ERROR_TIMEOUT:
            return "TIMEOUT";
        default:
            return "UNKNOWN";
    }
}

static const char *bus_state_name(void)
{
    switch (I2C_BusGetStatus())
    {
        case I2C_BUS_IDLE:
            return "IDLE";
        case I2C_BUS_PENDING:
            return "TRANSFER";
        case I2C_BUS_TIMEOUT:
            return "TIMEOUT";
        case I2C_BUS_STUCK:
            return "STUCK";
        case I2C_BUS_CONFIG:
            return "CONFIG";
        default:
            return "ERROR";
    }
}
static const char *servo_error_name(void)
{
    switch (sg90.error)
    {
        case SG90_ERROR_NONE:
            return "NONE";
        case SG90_ERROR_CONFIG:
            return "CONFIG";
        default:
            return "TIMER";
    }
}
static void report_changes(void)
{
    static uint32_t last[5] = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    uint32_t state[5] = {
        (uint32_t)aht20.error * 8U + (app_settings.aht20_on ? 2U : 0U) + (aht20_fresh() ? 1U : 0U),
        (uint32_t)bmp280.error * 8U + (app_settings.bmp280_on ? 2U : 0U) + (bmp280_fresh() ? 1U : 0U),
        (uint32_t)ssd1306.error * 8U + (app_settings.display_on ? 2U : 0U) + (ssd1306.ready ? 1U : 0U),
        (uint32_t)potentiometer.error * 8U + (potentiometer.enabled ? 2U : 0U) + (potentiometer.valid ? 1U : 0U),
        (uint32_t)sg90.error * 16U + (uint32_t)sg90.state + (servo_parking() ? 8U : 0U)
    };
    if (state[0] != last[0])
        log_write(
            "AHT20 %s error=%s",
            !app_settings.aht20_on ? "OFF"
            : aht20_fresh()        ? "OK"
                                   : "NO DATA",
            aht20_error_name(aht20.error)
        );
    if (state[1] != last[1])
        log_write(
            "BMP280 %s error=%s",
            !app_settings.bmp280_on ? "OFF"
            : bmp280_fresh()        ? "OK"
                                    : "NO DATA",
            bmp280_error_name(bmp280.error)
        );
    if (state[2] != last[2])
        log_write(
            "DISPLAY %s error=%s",
            !app_settings.display_on ? "OFF"
            : ssd1306.ready          ? "READY"
                                     : "WAIT",
            ssd1306_error_name(ssd1306.error)
        );
    if (state[3] != last[3])
        log_write(
            "POT %s error=%s",
            !potentiometer.enabled ? "OFF"
            : potentiometer.valid  ? "VALID"
                                   : "WAIT",
            pot_error_name(potentiometer.error)
        );
    if (state[4] != last[4])
        log_write(
            "SERVO %s error=%s",
            servo_parking()          ? "PARKING"
            : !app_settings.servo_on ? "OFF"
            : sg90.state == SG90_PWM ? "PWM"
                                     : "WAIT",
            servo_error_name()
        );
    memcpy(last, state, sizeof(last));
}
void log_process(void)
{
    if (ready && tail != head && (huart1.Instance->SR & USART_SR_TXE) != 0U)
    {
        huart1.Instance->DR = (uint8_t)queue[tail];
        tail = (tail + 1U) % QUEUE_SIZE;
    }
    uint32_t now = HAL_GetTick();
    uptime += (uint32_t)(now - last_tick);
    last_tick = now;
    if (app_save_pending)
        return;
    report_changes();
    if (!first && (uint32_t)(now - last_log) < 1000U)
        return;
    first = false;
    last_log = now;
    char at[16] = "--", rh[16] = "--", bt[16] = "--", pressure[16] = "--";
    if (aht20_fresh())
    {
        format_number(at, sizeof(at), aht20.temperature_c);
        format_number(rh, sizeof(rh), aht20.humidity_percent);
    }
    if (bmp280_fresh())
    {
        format_number(bt, sizeof(bt), bmp280.temperature_c);
        format_number(pressure, sizeof(pressure), bmp280.pressure_pa / 100.0f);
    }
    uint64_t seconds = uptime / 1000U;
    char hours[21];
    format_u64(hours, seconds / 3600U);
    log_write(
        "[%s:%02u:%02u] AHT20 %s C %s %% | BMP280 %s C %s hPa | I2C=%s",
        hours,
        (unsigned)((seconds / 60U) % 60U),
        (unsigned)(seconds % 60U),
        at,
        rh,
        bt,
        pressure,
        bus_state_name()
    );
    log_write(
        "SERVO %s cmd=%u target=%u pulse=%u us limits=%u..%u | POT raw=%u %s | DISPLAY mode=%u",
        servo_parking()         ? "PARKING"
        : app_settings.servo_on ? "ON"
                                : "OFF",
        (unsigned)sg90.command_angle_deg,
        (unsigned)sg90.target_angle_deg,
        (unsigned)sg90.pulse_us,
        (unsigned)app_settings.servo_min,
        (unsigned)app_settings.servo_max,
        (unsigned)potentiometer.raw,
        potentiometer.valid ? "VALID" : "NO DATA",
        (unsigned)app_settings.display_mode
    );
    static uint32_t reported;
    if (dropped != reported)
    {
        uint32_t count = dropped;
        log_write("LOG dropped=%lu", (unsigned long)count);
        reported = count;
    }
}

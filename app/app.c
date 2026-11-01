#include "app.h"
#include "app_state.h"
#include "sensors.h"
#include "display.h"
#include "servo.h"
#include "led.h"
#include "log.h"
#include "cmd.h"
#include "settings.h"
#include "stm32f4xx_hal.h"
#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "i2c_bus.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "potentiometer.h"
#include "sg90.h"
Settings app_settings;
bool app_save_pending, app_flash_error;
static Settings requested;
static bool preparing;
static uint32_t prepare_since;

void app_init(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    log_init();
    cmd_init();
    led_init();
    bool loaded = settings_load(&app_settings);
    app_flash_error = settings_damaged && !loaded;
    log_write("BOOT settings=%s%s", loaded ? "FLASH" : "DEFAULTS", settings_damaged ? "; invalid record skipped" : "");
    log_write("FLASH saves pause processing. Wait for OK saved before next command.");
    (void)I2C_BusInit(&hi2c1);
    sensors_init();
    display_init();
    servo_init();
}
bool app_request_settings(const Settings *value)
{
    if (app_save_pending || !settings_valid(value))
        return false;
    if (settings_equal(value, &app_settings) && !app_flash_error)
    {
        log_write("OK unchanged");
        return true;
    }
    requested = *value;
    app_save_pending = true;
    preparing = false;
    log_write("BUSY saving; wait for OK saved");
    return true;
}
static void save_process(void)
{
    if (!preparing)
    {
        preparing = true;
        prepare_since = HAL_GetTick();
        AHT20_SetEnabled(false);
        BMP280_SetEnabled(false);
        SSD1306_SetEnabled(false);
        Pot_SetEnabled(false);
        SG90_ClearTarget();
    }

    uint32_t age = (uint32_t)(HAL_GetTick() - prepare_since);
    if (I2C_BusGetStatus() == I2C_BUS_PENDING || age < 25U || (!log_idle() && age < 250U))
        return;
    bool saved = settings_save(&requested);
    cmd_discard_input();
    if (saved)
    {
        app_settings = requested;
        app_flash_error = false;
        log_write("OK saved");
    }
    else
    {
        app_flash_error = true;
        log_write("ERR save; previous settings remain active");
    }
    app_save_pending = false;
    preparing = false;
}
void app_process(void)
{
    log_process();
    cmd_process();
    I2C_BusTask();
    if (app_save_pending)
    {
        save_process();
        led_process();
        return;
    }
    sensors_process();
    servo_process();
    display_process();
    led_process();
}
void Error_Handler(void)
{
    SG90_SetEnabled(false);
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_13;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pin);
    uint32_t last = HAL_GetTick();
    for (;;)
    {
        if ((uint32_t)(HAL_GetTick() - last) >= 200U)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            last = HAL_GetTick();
        }
    }
}

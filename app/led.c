#include "led.h"
#include "app_state.h"
#include "sensors.h"
#include "display.h"
#include "servo.h"
#include "i2c_bus.h"
#include "stm32f4xx_hal.h"
static uint32_t last_toggle;
void led_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_13;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pin);
}
void led_process(void)
{
    I2C_BusResult bus = I2C_BusGetStatus();
    bool fault = app_flash_error || (!app_save_pending && (sensors_fault() || display_fault() || servo_fault())) ||
                 (bus != I2C_BUS_IDLE && bus != I2C_BUS_PENDING);
    if (!fault)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        return;
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_toggle) >= 200U)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_toggle = now;
    }
}

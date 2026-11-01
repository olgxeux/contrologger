#include "sg90.h"

#define FRAME_US 20000U
#define STEP_MS 20U
#define RETRY_MS 1000U

_Static_assert(
    SG90_PULSE_AT_0_DEG_US >= 500U && SG90_PULSE_AT_0_DEG_US < SG90_PULSE_AT_180_DEG_US &&
        SG90_PULSE_AT_180_DEG_US <= 2500U,
    "Invalid servo pulse calibration"
);

SG90_Data sg90;
static TIM_HandleTypeDef *servo_timer;
static bool timer_started;
static bool target_valid;
static uint32_t target_since_ms;
static uint32_t step_since_ms;
static uint32_t retry_since_ms;

static uint16_t limit_angle(uint16_t angle)
{
    if (angle < sg90.min_angle_deg)
    {
        return sg90.min_angle_deg;
    }
    if (angle > sg90.max_angle_deg)
    {
        return sg90.max_angle_deg;
    }
    return angle;
}

static void write_pulse(uint16_t pulse_us)
{
    sg90.pulse_us = pulse_us;
    __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_1, pulse_us);
}

static void write_angle(void)
{
    uint32_t pulse =
        SG90_PULSE_AT_0_DEG_US +
        ((uint32_t)sg90.command_angle_deg * (SG90_PULSE_AT_180_DEG_US - SG90_PULSE_AT_0_DEG_US) + 90U) / 180U;
    write_pulse((uint16_t)pulse);
}

static bool start_timer(uint32_t now)
{
    if (HAL_TIM_PWM_Stop(servo_timer, TIM_CHANNEL_1) == HAL_OK)
    {
        write_pulse(0U);
        __HAL_TIM_SET_COUNTER(servo_timer, 0U);
        if (HAL_TIM_GenerateEvent(servo_timer, TIM_EVENTSOURCE_UPDATE) == HAL_OK &&
            HAL_TIM_PWM_Start(servo_timer, TIM_CHANNEL_1) == HAL_OK)
        {
            timer_started = true;
            sg90.error = SG90_ERROR_NONE;
            return true;
        }
    }
    timer_started = false;
    sg90.pulse_us = 0U;
    sg90.error = SG90_ERROR_TIMER;
    sg90.state = SG90_RETRY_TIMER;
    retry_since_ms = now;
    return false;
}

bool SG90_Init(TIM_HandleTypeDef *timer)
{
    if (servo_timer != NULL)
    {
        return false;
    }
    sg90.error = SG90_ERROR_CONFIG;
    if (timer == NULL || timer->Instance != TIM3)
    {
        return false;
    }

    RCC_ClkInitTypeDef clocks = {0};
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timer_hz *= 2U;
    }

    if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0U || timer_hz == 0U || timer_hz % 1000000U != 0U ||
        timer->Init.Prescaler != timer_hz / 1000000U - 1U || timer->Init.Period != FRAME_US - 1U ||
        timer->Init.CounterMode != TIM_COUNTERMODE_UP || timer->Init.ClockDivision != TIM_CLOCKDIVISION_DIV1 ||
        (timer->Instance->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE)) !=
            (TIM_OCMODE_PWM1 | TIM_CCMR1_OC1PE) ||
        (timer->Instance->CCER & TIM_CCER_CC1P) != 0U || (timer->Instance->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U)
    {
        return false;
    }
    servo_timer = timer;
    sg90.min_angle_deg = 0U;
    sg90.max_angle_deg = 180U;
    sg90.command_angle_deg = 0U;
    sg90.target_angle_deg = 0U;
    sg90.enabled = false;
    (void)start_timer(HAL_GetTick());
    return true;
}

void SG90_SetEnabled(bool enabled)
{
    if (servo_timer == NULL)
    {
        return;
    }
    sg90.enabled = enabled;
    if (!enabled)
    {
        target_valid = false;
        write_pulse(0U);
        if (timer_started)
        {
            sg90.state = SG90_OFF;
        }
    }
}

bool SG90_SetLimits(uint16_t minimum, uint16_t maximum)
{
    if (servo_timer == NULL || minimum > maximum || maximum > 180U)
    {
        return false;
    }
    sg90.min_angle_deg = minimum;
    sg90.max_angle_deg = maximum;
    sg90.target_angle_deg = limit_angle(sg90.target_angle_deg);
    sg90.command_angle_deg = limit_angle(sg90.command_angle_deg);
    if (sg90.pulse_us != 0U && timer_started)
    {
        write_angle();
    }
    return true;
}

bool SG90_SetTargetAngle(uint16_t angle)
{
    if (servo_timer == NULL || angle > 180U)
    {
        SG90_ClearTarget();
        return false;
    }
    sg90.target_angle_deg = limit_angle(angle);
    target_since_ms = HAL_GetTick();
    target_valid = true;
    return true;
}

void SG90_ClearTarget(void)
{
    target_valid = false;
    if (servo_timer != NULL)
    {
        write_pulse(0U);
        if (timer_started)
        {
            sg90.state = sg90.enabled ? SG90_WAIT_INPUT : SG90_OFF;
        }
    }
}

void SG90_Task(void)
{
    if (servo_timer == NULL)
    {
        return;
    }
    uint32_t now = HAL_GetTick();

    if (timer_started &&
        ((servo_timer->Instance->CR1 & TIM_CR1_CEN) == 0U || (servo_timer->Instance->CCER & TIM_CCER_CC1E) == 0U))
    {
        write_pulse(0U);
        timer_started = false;
        sg90.error = SG90_ERROR_TIMER;
        sg90.state = SG90_RETRY_TIMER;
        retry_since_ms = now;
    }
    if (!timer_started)
    {
        if ((uint32_t)(now - retry_since_ms) < RETRY_MS || !start_timer(now))
        {
            return;
        }
    }
    if (!sg90.enabled)
    {
        write_pulse(0U);
        sg90.state = SG90_OFF;
        return;
    }
    if (!target_valid || (uint32_t)(now - target_since_ms) > SG90_INPUT_TIMEOUT_MS)
    {
        SG90_ClearTarget();
        return;
    }
    if (sg90.state != SG90_PWM)
    {
        sg90.state = SG90_PWM;
        step_since_ms = now;
        write_angle();
        return;
    }
    if ((uint32_t)(now - step_since_ms) < STEP_MS)
    {
        return;
    }
    step_since_ms = now;
    if (sg90.command_angle_deg < sg90.target_angle_deg)
    {
        ++sg90.command_angle_deg;
    }
    if (sg90.command_angle_deg > sg90.target_angle_deg)
    {
        --sg90.command_angle_deg;
    }
    write_angle();
}

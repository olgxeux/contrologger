#include "i2c_bus.h"

#define CLOCK_STEP_MS 2U
#define LINE_TIMEOUT_MS 25U
#define BUS_SETTLE_MS 25U
#define RECOVERY_RETRY_MS 1000U

typedef enum
{
    BUS_IDLE,
    BUS_WAIT_FREE,
    BUS_TRANSMIT,
    BUS_RECEIVE,
    BUS_WAIT_STOP,
    BUS_RECOVER
} BusState;

typedef enum
{
    REC_WAIT_SCL_HIGH,
    REC_HOLD_SCL_HIGH,
    REC_HOLD_SCL_LOW,
    REC_STOP_SCL_LOW,
    REC_STOP_WAIT_SCL_HIGH,
    REC_STOP_HOLD_SCL_HIGH,
    REC_WAIT_BUS_FREE,
    REC_WAIT_RETRY
} RecoveryState;

static I2C_HandleTypeDef *bus;
static I2C_BusRequest *active;
static BusState bus_state;
static RecoveryState recovery;
static I2C_BusResult bus_fault;
static I2C_BusResult finish_result;
static uint32_t transfer_started_ms;
static uint32_t recovery_started_ms;
static uint32_t idle_started_ms;
static uint32_t recovery_count;
static uint32_t analog_filter;
static uint32_t digital_filter;
static uint32_t transfer_error;
static uint8_t clock_pulses;
static bool transfer_done;

static bool lines_high(void)
{
    return HAL_GPIO_ReadPin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(I2C_BUS_SDA_PORT, I2C_BUS_SDA_PIN) == GPIO_PIN_SET;
}

static bool stop_pending(void)
{
    return (READ_BIT(bus->Instance->CR1, I2C_CR1_STOP) != 0U);
}

static void disable_and_clear_bus_irqs(void)
{
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
}

bool I2C_BusInit(I2C_HandleTypeDef *hi2c)
{
    if (bus != NULL)
    {
        return hi2c == bus;
    }
    if (hi2c == NULL || hi2c->Instance != I2C1 || HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY ||
        hi2c->Init.ClockSpeed == 0U || hi2c->Init.ClockSpeed > 400000U ||
        hi2c->Init.AddressingMode != I2C_ADDRESSINGMODE_7BIT || hi2c->Init.DualAddressMode != I2C_DUALADDRESS_DISABLE ||
        hi2c->Init.GeneralCallMode != I2C_GENERALCALL_DISABLE || hi2c->Init.NoStretchMode != I2C_NOSTRETCH_DISABLE)
    {
        return false;
    }
    bus = hi2c;

    analog_filter = READ_BIT(bus->Instance->FLTR, I2C_FLTR_ANOFF);
    digital_filter = READ_BIT(bus->Instance->FLTR, I2C_FLTR_DNF);
    disable_and_clear_bus_irqs();
    bus_state = BUS_IDLE;
    idle_started_ms = HAL_GetTick();
    return true;
}

HAL_StatusTypeDef I2C_BusStart(I2C_BusRequest *request)
{
    if (request == NULL)
    {
        return HAL_ERROR;
    }
    if (bus != NULL && bus_state != BUS_IDLE)
    {
        return HAL_BUSY;
    }
    if (bus == NULL || request->address < 8U || request->address > 0x77U ||
        (request->tx_size == 0U && request->rx_size == 0U) || (request->tx_size != 0U && request->tx == NULL) ||
        (request->rx_size != 0U && request->rx == NULL) || request->timeout_ms == 0U ||
        request->timeout_ms > 0x7FFFFFFFU)
    {
        request->result = I2C_BUS_CONFIG;
        return HAL_ERROR;
    }
    active = request;
    active->result = I2C_BUS_PENDING;
    transfer_started_ms = HAL_GetTick();
    transfer_done = false;
    transfer_error = HAL_I2C_ERROR_NONE;
    finish_result = I2C_BUS_OK;
    bus_state = BUS_WAIT_FREE;
    return HAL_OK;
}

I2C_BusResult I2C_BusGetStatus(void)
{
    if (bus == NULL)
    {
        return I2C_BUS_CONFIG;
    }
    if (bus_state == BUS_RECOVER)
    {
        return bus_fault;
    }
    return bus_state == BUS_IDLE ? I2C_BUS_IDLE : I2C_BUS_PENDING;
}

uint32_t I2C_BusGetRecoveryCount(void)
{
    return recovery_count;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == bus)
    {
        transfer_done = true;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == bus)
    {
        transfer_done = true;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == bus)
    {
        transfer_error = HAL_I2C_GetError(hi2c);
    }
}

static void begin_recovery(I2C_BusResult error, uint32_t now)
{
    disable_and_clear_bus_irqs();
    if (HAL_I2C_DeInit(bus) != HAL_OK)
    {
        error = I2C_BUS_CONFIG;
    }
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    HAL_GPIO_WritePin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_BUS_SDA_PORT, I2C_BUS_SDA_PIN, GPIO_PIN_SET);
    GPIO_InitTypeDef pins = {0};
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    pins.Pin = I2C_BUS_SCL_PIN;
    HAL_GPIO_Init(I2C_BUS_SCL_PORT, &pins);
    pins.Pin = I2C_BUS_SDA_PIN;
    HAL_GPIO_Init(I2C_BUS_SDA_PORT, &pins);

    if (bus_state != BUS_RECOVER)
    {
        ++recovery_count;
    }
    bus_state = BUS_RECOVER;
    bus_fault = error;
    clock_pulses = 0U;
    recovery = REC_WAIT_SCL_HIGH;
    recovery_started_ms = now;
    if (active != NULL)
    {
        active->result = error;
        active = NULL;
    }
}

static void retry_recovery(I2C_BusResult error, uint32_t now)
{
    begin_recovery(error, now);
    recovery = REC_WAIT_RETRY;
}

static void recover_bus(uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - recovery_started_ms);
    switch (recovery)
    {
        case REC_WAIT_RETRY:
            if (elapsed >= RECOVERY_RETRY_MS)
            {
                recovery = REC_WAIT_SCL_HIGH;
                recovery_started_ms = now;
            }
            break;
        case REC_WAIT_SCL_HIGH:
        case REC_STOP_WAIT_SCL_HIGH:
            if (HAL_GPIO_ReadPin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN) == GPIO_PIN_SET)
            {
                recovery = recovery == REC_WAIT_SCL_HIGH ? REC_HOLD_SCL_HIGH : REC_STOP_HOLD_SCL_HIGH;
                recovery_started_ms = now;
            }
            else if (elapsed >= LINE_TIMEOUT_MS)
            {
                retry_recovery(I2C_BUS_STUCK, now);
            }
            break;
        case REC_HOLD_SCL_HIGH:
            if (elapsed < CLOCK_STEP_MS)
            {
                break;
            }
            if (HAL_GPIO_ReadPin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN) == GPIO_PIN_RESET)
            {
                retry_recovery(I2C_BUS_STUCK, now);
            }
            else if (HAL_GPIO_ReadPin(I2C_BUS_SDA_PORT, I2C_BUS_SDA_PIN) == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(I2C_BUS_SDA_PORT, I2C_BUS_SDA_PIN, GPIO_PIN_RESET);
                recovery = REC_STOP_SCL_LOW;
                recovery_started_ms = now;
            }
            else if (clock_pulses < 9U)
            {
                HAL_GPIO_WritePin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN, GPIO_PIN_RESET);
                recovery = REC_HOLD_SCL_LOW;
                recovery_started_ms = now;
            }
            else
            {
                retry_recovery(I2C_BUS_STUCK, now);
            }
            break;
        case REC_HOLD_SCL_LOW:
        case REC_STOP_SCL_LOW:
            if (elapsed >= CLOCK_STEP_MS)
            {
                HAL_GPIO_WritePin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN, GPIO_PIN_SET);
                if (recovery == REC_HOLD_SCL_LOW)
                {
                    ++clock_pulses;
                }
                recovery = recovery == REC_HOLD_SCL_LOW ? REC_WAIT_SCL_HIGH : REC_STOP_WAIT_SCL_HIGH;
                recovery_started_ms = now;
            }
            break;
        case REC_STOP_HOLD_SCL_HIGH:
            if (elapsed < CLOCK_STEP_MS)
            {
                break;
            }
            if (HAL_GPIO_ReadPin(I2C_BUS_SCL_PORT, I2C_BUS_SCL_PIN) == GPIO_PIN_RESET)
            {
                retry_recovery(I2C_BUS_STUCK, now);
            }
            else
            {
                HAL_GPIO_WritePin(I2C_BUS_SDA_PORT, I2C_BUS_SDA_PIN, GPIO_PIN_SET);
                recovery = REC_WAIT_BUS_FREE;
                recovery_started_ms = now;
            }
            break;
        case REC_WAIT_BUS_FREE:
            if (!lines_high())
            {
                retry_recovery(I2C_BUS_STUCK, now);
            }
            else if (elapsed >= BUS_SETTLE_MS)
            {
                if (HAL_I2C_Init(bus) != HAL_OK || HAL_I2CEx_ConfigAnalogFilter(bus, analog_filter) != HAL_OK ||
                    HAL_I2CEx_ConfigDigitalFilter(bus, digital_filter) != HAL_OK)
                {
                    retry_recovery(I2C_BUS_CONFIG, now);
                    break;
                }
                disable_and_clear_bus_irqs();
                if (!lines_high() || __HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) != RESET)
                {
                    retry_recovery(I2C_BUS_STUCK, now);
                    break;
                }
                bus_state = BUS_IDLE;
                idle_started_ms = now;
            }
            break;
    }
}

static void start_receive(void)
{
    transfer_done = false;
    bus_state = BUS_RECEIVE;

    if (HAL_I2C_Master_Seq_Receive_IT(
            bus,
            (uint16_t)(active->address << 1),
            active->rx,
            active->rx_size,
            I2C_OTHER_AND_LAST_FRAME
        ) != HAL_OK)
    {
        begin_recovery(I2C_BUS_ERROR, HAL_GetTick());
    }
}

void I2C_BusTask(void)
{
    if (bus == NULL)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    if (bus_state == BUS_RECOVER)
    {
        recover_bus(now);
        return;
    }
    if (bus_state == BUS_IDLE)
    {
        if (lines_high() && !stop_pending() && __HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) == RESET)
        {
            idle_started_ms = now;
        }
        else if ((uint32_t)(now - idle_started_ms) >= LINE_TIMEOUT_MS)
        {
            begin_recovery(I2C_BUS_STUCK, now);
        }
        return;
    }
    if ((uint32_t)(now - transfer_started_ms) >= active->timeout_ms)
    {
        begin_recovery(lines_high() ? I2C_BUS_TIMEOUT : I2C_BUS_STUCK, now);
        return;
    }
    if (bus_state == BUS_WAIT_FREE)
    {
        if (!lines_high() || stop_pending() || __HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) != RESET)
        {
            return;
        }
        disable_and_clear_bus_irqs();
        if (active->tx_size == 0U)
        {
            start_receive();
        }
        else
        {
            bus_state = BUS_TRANSMIT;
            uint32_t option = active->rx_size == 0U ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME;
            if (HAL_I2C_Master_Seq_Transmit_IT(
                    bus,
                    (uint16_t)(active->address << 1),
                    (uint8_t *)active->tx,
                    active->tx_size,
                    option
                ) != HAL_OK)
            {
                begin_recovery(I2C_BUS_ERROR, now);
            }
        }
        return;
    }
    if (bus_state == BUS_WAIT_STOP)
    {
        if (!stop_pending() && lines_high() && __HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) == RESET)
        {
            active->result = finish_result;
            active = NULL;
            bus_state = BUS_IDLE;
            idle_started_ms = now;
        }
        return;
    }

    if (HAL_NVIC_GetPendingIRQ(I2C1_ER_IRQn) != 0U)
    {
        HAL_I2C_ER_IRQHandler(bus);
        HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    }
    if (transfer_error != HAL_I2C_ERROR_NONE)
    {
        if (transfer_error == HAL_I2C_ERROR_AF)
        {
            finish_result = I2C_BUS_NACK;
            bus_state = BUS_WAIT_STOP;
            begin_recovery(I2C_BUS_ERROR, now);
        }
        return;
    }
    if (stop_pending())
    {
        return;
    }
    HAL_I2C_EV_IRQHandler(bus);
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    if (transfer_done)
    {
        if (bus_state == BUS_TRANSMIT && active->rx_size != 0U)
        {
            start_receive();
        }
        else
        {
            bus_state = BUS_WAIT_STOP;
        }
    }
}

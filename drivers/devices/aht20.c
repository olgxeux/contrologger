#include "aht20.h"
#include "i2c_bus.h"

#define ACCESS_TIMEOUT_MS 250U
#define TRANSFER_MS 100U
#define POWER_MS 41U
#define RESET_MS 21U
#define INIT_MS 11U
#define MEASURE_MS 81U
#define POLL_MS 11U
#define INIT_LIMIT_MS 100U
#define MEASURE_LIMIT_MS 200U
#define SAMPLE_MS 1000U
#define RETRY_MS 1000U
#define AHT20_ADDRESS 0x38U
#define STATUS_BUSY 0x80U
#define STATUS_CALIBRATED 0x08U

AHT20_Data aht20;
static bool sensor_initialized;
static bool restart_pending;
static uint32_t last_bus_recovery;
static uint32_t state_since;
static uint32_t wait_ms;
static uint32_t operation_since;
static uint8_t command[3];
static uint8_t data[7];
static I2C_BusRequest sensor_request;

static void schedule(AHT20_State next, uint32_t delay, uint32_t now)
{
    aht20.state = next;
    state_since = now;
    wait_ms = delay;
}

static void sensor_fail(AHT20_Error error, uint32_t now)
{
    aht20.valid = false;
    aht20.error = error;
    schedule(AHT20_RESET_SENSOR, RETRY_MS, now);
}

static AHT20_Error sensor_bus_error(I2C_BusResult result)
{
    switch (result)
    {
        case I2C_BUS_NACK:
            return AHT20_ERROR_NACK;
        case I2C_BUS_TIMEOUT:
            return AHT20_ERROR_TIMEOUT;
        case I2C_BUS_STUCK:
            return AHT20_ERROR_BUS_STUCK;
        case I2C_BUS_CONFIG:
            return AHT20_ERROR_CONFIG;
        default:
            return AHT20_ERROR_I2C;
    }
}

static uint8_t crc8(const uint8_t *bytes)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < 6U; ++i)
    {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31U);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

bool AHT20_Init(I2C_HandleTypeDef *hi2c)
{
    if (sensor_initialized)
    {
        return false;
    }
    aht20 = (AHT20_Data){0};
    if (!I2C_BusInit(hi2c))
    {
        aht20.error = AHT20_ERROR_CONFIG;
        return false;
    }
    sensor_request = (I2C_BusRequest){0};
    sensor_request.address = AHT20_ADDRESS;
    sensor_request.tx = command;
    sensor_request.rx = data;
    sensor_request.timeout_ms = TRANSFER_MS;
    last_bus_recovery = I2C_BusGetRecoveryCount();
    sensor_initialized = true;
    AHT20_SetEnabled(true);
    return true;
}

void AHT20_SetEnabled(bool enabled)
{
    if (!sensor_initialized || aht20.enabled == enabled)
    {
        return;
    }
    aht20.enabled = enabled;
    aht20.valid = false;
    aht20.error = AHT20_ERROR_NONE;
    restart_pending = true;
}

static void sensor_complete(uint32_t now)
{
    bool busy = (data[0] & STATUS_BUSY) != 0U;
    bool calibrated = (data[0] & STATUS_CALIBRATED) != 0U;

    switch (aht20.state)
    {
        case AHT20_RESET_SENSOR:
            schedule(AHT20_READ_STARTUP_STATUS, RESET_MS, now);
            break;
        case AHT20_READ_STARTUP_STATUS:
            if (busy)
            {
                sensor_fail(AHT20_ERROR_SENSOR_BUSY, now);
            }
            else if (calibrated)
            {
                schedule(AHT20_START_MEASUREMENT, 0U, now);
            }
            else
            {
                schedule(AHT20_INITIALIZE_SENSOR, 0U, now);
            }
            break;
        case AHT20_INITIALIZE_SENSOR:
            operation_since = now;
            schedule(AHT20_READ_INIT_STATUS, INIT_MS, now);
            break;
        case AHT20_READ_INIT_STATUS:
            if (!busy && calibrated)
            {
                schedule(AHT20_START_MEASUREMENT, 0U, now);
            }
            else if ((uint32_t)(now - operation_since) >= INIT_LIMIT_MS)
            {
                sensor_fail(busy ? AHT20_ERROR_SENSOR_BUSY : AHT20_ERROR_CALIBRATION, now);
            }
            else
            {
                schedule(AHT20_READ_INIT_STATUS, POLL_MS, now);
            }
            break;
        case AHT20_START_MEASUREMENT:
            operation_since = now;
            schedule(AHT20_READ_MEASUREMENT_STATUS, MEASURE_MS, now);
            break;
        case AHT20_READ_MEASUREMENT_STATUS:
            if (busy)
            {
                if ((uint32_t)(now - operation_since) >= MEASURE_LIMIT_MS)
                {
                    sensor_fail(AHT20_ERROR_SENSOR_BUSY, now);
                }
                else
                {
                    schedule(AHT20_READ_MEASUREMENT_STATUS, POLL_MS, now);
                }
            }
            else if (!calibrated)
            {
                sensor_fail(AHT20_ERROR_CALIBRATION, now);
            }
            else
            {
                schedule(AHT20_READ_MEASUREMENT, 0U, now);
            }
            break;
        case AHT20_READ_MEASUREMENT:
        {
            if (crc8(data) != data[6])
            {
                sensor_fail(AHT20_ERROR_CRC, now);
                break;
            }
            if (busy)
            {
                sensor_fail(AHT20_ERROR_SENSOR_BUSY, now);
                break;
            }
            if (!calibrated)
            {
                sensor_fail(AHT20_ERROR_CALIBRATION, now);
                break;
            }
            uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
            uint32_t raw_temperature = ((uint32_t)(data[3] & 15U) << 16) | ((uint32_t)data[4] << 8) | data[5];
            float temperature = (float)raw_temperature * (200.0f / 1048576.0f) - 50.0f;
            if (temperature < -40.0f || temperature > 85.0f)
            {
                sensor_fail(AHT20_ERROR_RANGE, now);
                break;
            }
            aht20.temperature_c = temperature;
            aht20.humidity_percent = (float)raw_humidity * (100.0f / 1048576.0f);
            aht20.sample_ms = now;
            aht20.valid = true;
            aht20.error = AHT20_ERROR_NONE;
            schedule(AHT20_START_MEASUREMENT, SAMPLE_MS, now);
            break;
        }
    }
}

void AHT20_Task(void)
{
    if (!sensor_initialized)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    uint32_t recovery_now = I2C_BusGetRecoveryCount();
    if (recovery_now != last_bus_recovery)
    {
        last_bus_recovery = recovery_now;
        aht20.valid = false;
        if (aht20.enabled)
        {
            aht20.error = sensor_bus_error(I2C_BusGetStatus());
        }
        restart_pending = true;
    }

    if (sensor_request.result == I2C_BUS_PENDING)
    {
        return;
    }
    if (restart_pending)
    {
        sensor_request.result = I2C_BUS_IDLE;
        restart_pending = false;
        schedule(AHT20_RESET_SENSOR, POWER_MS, now);
    }
    if (!aht20.enabled)
    {
        return;
    }
    if (sensor_request.result != I2C_BUS_IDLE)
    {
        I2C_BusResult result = sensor_request.result;
        sensor_request.result = I2C_BUS_IDLE;
        if (result == I2C_BUS_OK)
        {
            sensor_complete(now);
        }
        else
        {
            sensor_fail(sensor_bus_error(result), now);
        }
        return;
    }
    uint32_t elapsed = (uint32_t)(now - state_since);
    if (elapsed < wait_ms)
    {
        return;
    }

    if (elapsed - wait_ms >= ACCESS_TIMEOUT_MS)
    {
        I2C_BusResult status = I2C_BusGetStatus();
        AHT20_Error error =
            (status == I2C_BUS_IDLE || status == I2C_BUS_PENDING) ? AHT20_ERROR_TIMEOUT : sensor_bus_error(status);
        sensor_fail(error, now);
        return;
    }
    sensor_request.tx_size = 0U;
    sensor_request.rx_size = 0U;
    switch (aht20.state)
    {
        case AHT20_RESET_SENSOR:
            command[0] = 0xBAU;
            sensor_request.tx_size = 1U;
            break;
        case AHT20_INITIALIZE_SENSOR:
            command[0] = 0xBEU;
            command[1] = 0x08U;
            command[2] = 0x00U;
            sensor_request.tx_size = 3U;
            break;
        case AHT20_START_MEASUREMENT:
            command[0] = 0xACU;
            command[1] = 0x33U;
            command[2] = 0x00U;
            sensor_request.tx_size = 3U;
            break;
        case AHT20_READ_STARTUP_STATUS:
        case AHT20_READ_INIT_STATUS:
        case AHT20_READ_MEASUREMENT_STATUS:
            sensor_request.rx_size = 1U;
            break;
        case AHT20_READ_MEASUREMENT:
            sensor_request.rx_size = 7U;
            break;
    }

    if (I2C_BusStart(&sensor_request) == HAL_ERROR)
    {
        sensor_request.result = I2C_BUS_IDLE;
        sensor_fail(AHT20_ERROR_CONFIG, now);
    }
}

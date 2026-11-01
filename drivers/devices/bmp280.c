#include "bmp280.h"
#include "i2c_bus.h"
#include <string.h>

#define REG_CHIP_ID 0xD0U
#define REG_RESET 0xE0U
#define REG_STATUS 0xF3U
#define REG_CTRL_MEAS 0xF4U
#define REG_CONFIG 0xF5U
#define REG_MEASUREMENT 0xF7U
#define REG_CALIBRATION 0x88U
#define EXPECTED_CHIP_ID 0x58U
#define SOFT_RESET_COMMAND 0xB6U
#define STATUS_MEASURING 0x08U
#define STATUS_NVM_COPY 0x01U
#define CTRL_SLEEP_X1 0x24U
#define CTRL_FORCED_X1 0x25U
#define CONFIG_VALUE 0x00U
#define CONFIG_VALID_BITS 0xFDU
#define STARTUP_DELAY_MS 3U
#define MEASUREMENT_DELAY_MS 8U
#define STATUS_POLL_MS 3U
#define SENSOR_OPERATION_MS 100U
#define I2C_TRANSFER_MS 100U
#define BUS_ACCESS_MS 250U
#define SAMPLE_INTERVAL_MS 1000U
#define RECOVERY_RETRY_MS 1000U

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} Calibration;

BMP280_Data bmp280;
static Calibration calibration;
static I2C_BusRequest request;
static uint8_t command_bytes[2];
static uint8_t received_bytes[24];
static uint8_t calibration_bytes[24];
static uint8_t measurement_bytes[6];
static bool initialized;
static bool restart_pending;
static uint32_t last_bus_recovery;
static bool wrong_chip_id_seen;
static uint32_t step_started_ms;
static uint32_t step_delay_ms;
static uint32_t operation_started_ms;

static void schedule_step(BMP280_State next, uint32_t delay_ms, uint32_t now)
{
    bmp280.state = next;
    step_started_ms = now;
    step_delay_ms = delay_ms;
}

static void restart_detection(uint32_t delay_ms, uint32_t now)
{
    request.address = 0x76U;
    bmp280.i2c_address = request.address;
    wrong_chip_id_seen = false;
    schedule_step(BMP280_READ_CHIP_ID, delay_ms, now);
}

static void report_error(BMP280_Error error, uint32_t now)
{
    bmp280.valid = false;
    bmp280.error = error;
    restart_detection(RECOVERY_RETRY_MS, now);
}

static BMP280_Error convert_bus_error(I2C_BusResult result)
{
    switch (result)
    {
        case I2C_BUS_NACK:
            return BMP280_ERROR_NACK;
        case I2C_BUS_TIMEOUT:
            return BMP280_ERROR_TIMEOUT;
        case I2C_BUS_STUCK:
            return BMP280_ERROR_BUS_STUCK;
        case I2C_BUS_CONFIG:
            return BMP280_ERROR_CONFIG;
        default:
            return BMP280_ERROR_I2C;
    }
}

static void try_next_address(uint32_t now)
{
    if (request.address == 0x76U)
    {
        request.address = 0x77U;
        bmp280.i2c_address = request.address;
        schedule_step(BMP280_READ_CHIP_ID, 0U, now);
    }
    else
    {
        report_error(wrong_chip_id_seen ? BMP280_ERROR_CHIP_ID : BMP280_ERROR_NACK, now);
    }
}

static uint16_t read_unsigned_le16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static int16_t read_signed_le16(const uint8_t *bytes)
{
    int32_t value = read_unsigned_le16(bytes);
    if (value >= 32768)
    {
        value -= 65536;
    }
    return (int16_t)value;
}

static bool decode_calibration(void)
{
    bool all_ff = true;
    for (uint32_t i = 0U; i < sizeof(calibration_bytes); ++i)
    {
        if (calibration_bytes[i] != 0xFFU)
        {
            all_ff = false;
        }
    }
    calibration.dig_T1 = read_unsigned_le16(&calibration_bytes[0]);
    calibration.dig_T2 = read_signed_le16(&calibration_bytes[2]);
    calibration.dig_T3 = read_signed_le16(&calibration_bytes[4]);
    calibration.dig_P1 = read_unsigned_le16(&calibration_bytes[6]);
    calibration.dig_P2 = read_signed_le16(&calibration_bytes[8]);
    calibration.dig_P3 = read_signed_le16(&calibration_bytes[10]);
    calibration.dig_P4 = read_signed_le16(&calibration_bytes[12]);
    calibration.dig_P5 = read_signed_le16(&calibration_bytes[14]);
    calibration.dig_P6 = read_signed_le16(&calibration_bytes[16]);
    calibration.dig_P7 = read_signed_le16(&calibration_bytes[18]);
    calibration.dig_P8 = read_signed_le16(&calibration_bytes[20]);
    calibration.dig_P9 = read_signed_le16(&calibration_bytes[22]);

    return !all_ff && calibration.dig_P1 != 0U;
}

static BMP280_Error compensate_measurement(float *temperature_c, float *pressure_pa)
{
    uint32_t raw_pressure =
        ((uint32_t)measurement_bytes[0] << 12) | ((uint32_t)measurement_bytes[1] << 4) | (measurement_bytes[2] >> 4);
    uint32_t raw_temperature =
        ((uint32_t)measurement_bytes[3] << 12) | ((uint32_t)measurement_bytes[4] << 4) | (measurement_bytes[5] >> 4);
    if (raw_pressure == 0x80000U || raw_temperature == 0x80000U)
    {
        return BMP280_ERROR_INVALID_SAMPLE;
    }

    double temperature_linear =
        ((double)raw_temperature / 16384.0 - (double)calibration.dig_T1 / 1024.0) * calibration.dig_T2;
    double temperature_delta = (double)raw_temperature / 131072.0 - (double)calibration.dig_T1 / 8192.0;
    double temperature_curve = temperature_delta * temperature_delta * calibration.dig_T3;
    double temperature = (temperature_linear + temperature_curve) / 5120.0;
    if (!(temperature >= -40.0 && temperature <= 85.0))
    {
        return BMP280_ERROR_RANGE;
    }
    int32_t temperature_fine = (int32_t)(temperature_linear + temperature_curve);
    double temperature_offset = (double)temperature_fine / 2.0 - 64000.0;
    double pressure_offset = temperature_offset * temperature_offset * calibration.dig_P6 / 32768.0;
    pressure_offset += temperature_offset * calibration.dig_P5 * 2.0;
    pressure_offset = pressure_offset / 4.0 + (double)calibration.dig_P4 * 65536.0;
    double pressure_gain = (calibration.dig_P3 * temperature_offset * temperature_offset / 524288.0 +
                            calibration.dig_P2 * temperature_offset) /
                           524288.0;
    pressure_gain = (1.0 + pressure_gain / 32768.0) * calibration.dig_P1;
    if (pressure_gain == 0.0)
    {
        return BMP280_ERROR_CALIBRATION;
    }
    double pressure = (1048576.0 - raw_pressure - pressure_offset / 4096.0) * 6250.0 / pressure_gain;
    double pressure_quadratic = calibration.dig_P9 * pressure * pressure / 2147483648.0;
    double pressure_linear = pressure * calibration.dig_P8 / 32768.0;
    pressure += (pressure_quadratic + pressure_linear + calibration.dig_P7) / 16.0;
    if (!(pressure >= 30000.0 && pressure <= 110000.0))
    {
        return BMP280_ERROR_RANGE;
    }
    *temperature_c = (float)temperature;
    *pressure_pa = (float)pressure;
    return BMP280_ERROR_NONE;
}

bool BMP280_Init(I2C_HandleTypeDef *hi2c)
{
    if (initialized)
    {
        return false;
    }
    bmp280 = (BMP280_Data){0};
    if (!I2C_BusInit(hi2c))
    {
        bmp280.error = BMP280_ERROR_CONFIG;
        return false;
    }
    request = (I2C_BusRequest){0};
    request.tx = command_bytes;
    request.rx = received_bytes;
    request.timeout_ms = I2C_TRANSFER_MS;
    last_bus_recovery = I2C_BusGetRecoveryCount();
    initialized = true;
    BMP280_SetEnabled(true);
    return true;
}

void BMP280_SetEnabled(bool enabled)
{
    if (!initialized || bmp280.enabled == enabled)
    {
        return;
    }
    bmp280.enabled = enabled;
    bmp280.valid = false;
    bmp280.error = BMP280_ERROR_NONE;
    restart_pending = true;
}

static void handle_completed_request(uint32_t now)
{
    switch (bmp280.state)
    {
        case BMP280_READ_CHIP_ID:
            bmp280.last_chip_id = received_bytes[0];
            if (bmp280.last_chip_id != EXPECTED_CHIP_ID)
            {
                wrong_chip_id_seen = true;
                try_next_address(now);
            }
            else
            {
                schedule_step(BMP280_RESET_SENSOR, 0U, now);
            }
            break;
        case BMP280_RESET_SENSOR:
            operation_started_ms = now;
            schedule_step(BMP280_READ_STARTUP_STATUS, STARTUP_DELAY_MS, now);
            break;
        case BMP280_READ_STARTUP_STATUS:
            if ((received_bytes[0] & (STATUS_MEASURING | STATUS_NVM_COPY)) == 0U)
            {
                schedule_step(BMP280_READ_CALIBRATION, 0U, now);
            }
            else if ((uint32_t)(now - operation_started_ms) >= SENSOR_OPERATION_MS)
            {
                report_error(BMP280_ERROR_SENSOR_BUSY, now);
            }
            else
            {
                schedule_step(BMP280_READ_STARTUP_STATUS, STATUS_POLL_MS, now);
            }
            break;
        case BMP280_READ_CALIBRATION:
            memcpy(calibration_bytes, received_bytes, sizeof(calibration_bytes));
            schedule_step(BMP280_VERIFY_CALIBRATION, 0U, now);
            break;
        case BMP280_VERIFY_CALIBRATION:
            if (memcmp(calibration_bytes, received_bytes, sizeof(calibration_bytes)) != 0)
            {
                report_error(BMP280_ERROR_DATA_MISMATCH, now);
            }
            else if (!decode_calibration())
            {
                report_error(BMP280_ERROR_CALIBRATION, now);
            }
            else
            {
                schedule_step(BMP280_WRITE_CONFIGURATION, 0U, now);
            }
            break;
        case BMP280_WRITE_CONFIGURATION:
            schedule_step(BMP280_START_MEASUREMENT, 0U, now);
            break;
        case BMP280_START_MEASUREMENT:
            operation_started_ms = now;
            schedule_step(BMP280_READ_MEASUREMENT_STATUS, MEASUREMENT_DELAY_MS, now);
            break;
        case BMP280_READ_MEASUREMENT_STATUS:

            if ((received_bytes[1] & 0xFCU) != CTRL_SLEEP_X1 ||
                (received_bytes[2] & CONFIG_VALID_BITS) != CONFIG_VALUE || (received_bytes[1] & 3U) == 3U)
            {
                report_error(BMP280_ERROR_CONFIG_MISMATCH, now);
            }
            else if ((received_bytes[0] & (STATUS_MEASURING | STATUS_NVM_COPY)) != 0U || (received_bytes[1] & 3U) != 0U)
            {
                if ((uint32_t)(now - operation_started_ms) >= SENSOR_OPERATION_MS)
                {
                    report_error(BMP280_ERROR_SENSOR_BUSY, now);
                }
                else
                {
                    schedule_step(BMP280_READ_MEASUREMENT_STATUS, STATUS_POLL_MS, now);
                }
            }
            else
            {
                schedule_step(BMP280_READ_MEASUREMENT, 0U, now);
            }
            break;
        case BMP280_READ_MEASUREMENT:
            memcpy(measurement_bytes, received_bytes, sizeof(measurement_bytes));
            schedule_step(BMP280_VERIFY_MEASUREMENT, 0U, now);
            break;
        case BMP280_VERIFY_MEASUREMENT:
        {
            if (memcmp(measurement_bytes, received_bytes, sizeof(measurement_bytes)) != 0)
            {
                report_error(BMP280_ERROR_DATA_MISMATCH, now);
                break;
            }
            float temperature_c;
            float pressure_pa;
            BMP280_Error error = compensate_measurement(&temperature_c, &pressure_pa);
            if (error != BMP280_ERROR_NONE)
            {
                report_error(error, now);
                break;
            }
            bmp280.temperature_c = temperature_c;
            bmp280.pressure_pa = pressure_pa;
            bmp280.sample_ms = now;
            bmp280.error = BMP280_ERROR_NONE;
            bmp280.valid = true;
            schedule_step(BMP280_START_MEASUREMENT, SAMPLE_INTERVAL_MS, now);
            break;
        }
    }
}

void BMP280_Task(void)
{
    if (!initialized)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    uint32_t recovery_now = I2C_BusGetRecoveryCount();
    if (recovery_now != last_bus_recovery)
    {
        last_bus_recovery = recovery_now;
        bmp280.valid = false;
        if (bmp280.enabled)
        {
            bmp280.error = convert_bus_error(I2C_BusGetStatus());
        }
        restart_pending = true;
    }
    if (request.result == I2C_BUS_PENDING)
    {
        return;
    }

    if (restart_pending)
    {
        request.result = I2C_BUS_IDLE;
        restart_pending = false;
        restart_detection(STARTUP_DELAY_MS, now);
    }
    if (!bmp280.enabled)
    {
        return;
    }
    if (request.result != I2C_BUS_IDLE)
    {
        I2C_BusResult result = request.result;
        request.result = I2C_BUS_IDLE;
        if (result == I2C_BUS_OK)
        {
            handle_completed_request(now);
        }
        else if (bmp280.state == BMP280_READ_CHIP_ID && result == I2C_BUS_NACK)
        {
            try_next_address(now);
        }
        else
        {
            report_error(convert_bus_error(result), now);
        }
        return;
    }
    uint32_t elapsed = (uint32_t)(now - step_started_ms);
    if (elapsed < step_delay_ms)
    {
        return;
    }
    if (elapsed - step_delay_ms >= BUS_ACCESS_MS)
    {
        I2C_BusResult status = I2C_BusGetStatus();
        BMP280_Error error =
            (status == I2C_BUS_IDLE || status == I2C_BUS_PENDING) ? BMP280_ERROR_TIMEOUT : convert_bus_error(status);
        report_error(error, now);
        return;
    }

    request.tx_size = 1U;
    request.rx_size = 1U;
    switch (bmp280.state)
    {
        case BMP280_READ_CHIP_ID:
            command_bytes[0] = REG_CHIP_ID;
            break;
        case BMP280_RESET_SENSOR:
            command_bytes[0] = REG_RESET;
            command_bytes[1] = SOFT_RESET_COMMAND;
            request.tx_size = 2U;
            request.rx_size = 0U;
            break;
        case BMP280_READ_STARTUP_STATUS:
            command_bytes[0] = REG_STATUS;
            break;
        case BMP280_READ_CALIBRATION:
        case BMP280_VERIFY_CALIBRATION:
            command_bytes[0] = REG_CALIBRATION;
            request.rx_size = sizeof(calibration_bytes);
            break;
        case BMP280_WRITE_CONFIGURATION:
            command_bytes[0] = REG_CONFIG;
            command_bytes[1] = CONFIG_VALUE;
            request.tx_size = 2U;
            request.rx_size = 0U;
            break;
        case BMP280_START_MEASUREMENT:
            command_bytes[0] = REG_CTRL_MEAS;
            command_bytes[1] = CTRL_FORCED_X1;
            request.tx_size = 2U;
            request.rx_size = 0U;
            break;
        case BMP280_READ_MEASUREMENT_STATUS:
            command_bytes[0] = REG_STATUS;
            request.rx_size = 3U;
            break;
        case BMP280_READ_MEASUREMENT:
        case BMP280_VERIFY_MEASUREMENT:
            command_bytes[0] = REG_MEASUREMENT;
            request.rx_size = sizeof(measurement_bytes);
            break;
    }
    if (I2C_BusStart(&request) == HAL_ERROR)
    {
        request.result = I2C_BUS_IDLE;
        report_error(BMP280_ERROR_CONFIG, now);
    }
}

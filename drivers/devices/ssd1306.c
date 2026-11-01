#include "ssd1306.h"
#include "i2c_bus.h"
#include <string.h>

#define FRAME_BYTES (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)
#define CHUNK_BYTES 32U
#define I2C_TRANSFER_MS 100U
#define BUS_ACCESS_MS 250U
#define BUS_YIELD_MS 2U
#define STARTUP_DELAY_MS 101U
#define RESET_DELAY_MS 2U
#define DISPLAY_ON_DELAY_MS 101U
#define REFRESH_INTERVAL_MS 1000U
#define RETRY_INTERVAL_MS 1000U

static const uint8_t font[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* 32   */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* 33 ! */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* 34 " */
    {0x12, 0x3F, 0x12, 0x3F, 0x12}, /* 35 # */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* 36 $ */
    {0x23, 0x13, 0x6C, 0x62, 0x01}, /* 37 % */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* 38 & */
    {0x00, 0x04, 0x03, 0x00, 0x00}, /* 39 ' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* 40 ( */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* 41 ) */
    {0x2A, 0x1C, 0x3E, 0x1C, 0x2A}, /* 42 * */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* 43 + */
    {0x00, 0x40, 0x30, 0x00, 0x00}, /* 44 , */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* 45 - */
    {0x00, 0x00, 0x60, 0x00, 0x00}, /* 46 . */
    {0x40, 0x30, 0x08, 0x06, 0x01}, /* 47 / */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 48 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 49 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 50 2 */
    {0x41, 0x49, 0x49, 0x49, 0x36}, /* 51 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 52 4 */
    {0x4F, 0x49, 0x49, 0x49, 0x31}, /* 53 5 */
    {0x3E, 0x49, 0x49, 0x49, 0x30}, /* 54 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 55 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 56 8 */
    {0x06, 0x49, 0x49, 0x49, 0x3E}, /* 57 9 */
    {0x00, 0x00, 0x36, 0x00, 0x00}, /* 58 : */
    {0x00, 0x40, 0x36, 0x00, 0x00}, /* 59 ; */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* 60 < */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* 61 = */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* 62 > */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* 63 ? */
    {0x3E, 0x41, 0x5D, 0x55, 0x1E}, /* 64 @ */
    {0x7E, 0x09, 0x09, 0x09, 0x7E}, /* 65 A */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* 66 B */
    {0x3E, 0x41, 0x41, 0x41, 0x41}, /* 67 C */
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, /* 68 D */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* 69 E */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* 70 F */
    {0x3E, 0x41, 0x49, 0x49, 0x3A}, /* 71 G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 72 H */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* 73 I */
    {0x30, 0x40, 0x41, 0x3F, 0x01}, /* 74 J */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* 75 K */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* 76 L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 77 M */
    {0x7F, 0x02, 0x04, 0x08, 0x7F}, /* 78 N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 79 O */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* 80 P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 81 Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* 82 R */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* 83 S */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* 84 T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 85 U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 86 V */
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, /* 87 W */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* 88 X */
    {0x03, 0x04, 0x78, 0x04, 0x03}, /* 89 Y */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* 90 Z */
    {0x00, 0x7F, 0x41, 0x41, 0x00}, /* 91 [ */
    {0x01, 0x06, 0x08, 0x30, 0x40}, /* 92 \ */
    {0x00, 0x41, 0x41, 0x7F, 0x00}, /* 93 ] */
    {0x04, 0x02, 0x01, 0x02, 0x04}, /* 94 ^ */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* 95 _ */
    {0x00, 0x01, 0x02, 0x04, 0x00}, /* 96 ` */
    {0x20, 0x54, 0x54, 0x54, 0x78}, /* 97 a */
    {0x7F, 0x48, 0x44, 0x44, 0x38}, /* 98 b */
    {0x38, 0x44, 0x44, 0x44, 0x44}, /* 99 c */
    {0x38, 0x44, 0x44, 0x48, 0x7F}, /* 100 d */
    {0x38, 0x54, 0x54, 0x54, 0x58}, /* 101 e */
    {0x08, 0x7E, 0x09, 0x01, 0x02}, /* 102 f */
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, /* 103 g */
    {0x7F, 0x08, 0x04, 0x04, 0x78}, /* 104 h */
    {0x00, 0x44, 0x7D, 0x40, 0x00}, /* 105 i */
    {0x20, 0x40, 0x44, 0x3D, 0x00}, /* 106 j */
    {0x7F, 0x10, 0x28, 0x44, 0x00}, /* 107 k */
    {0x00, 0x41, 0x7F, 0x40, 0x00}, /* 108 l */
    {0x7C, 0x04, 0x78, 0x04, 0x78}, /* 109 m */
    {0x7C, 0x08, 0x04, 0x04, 0x78}, /* 110 n */
    {0x38, 0x44, 0x44, 0x44, 0x38}, /* 111 o */
    {0x7E, 0x12, 0x12, 0x12, 0x0C}, /* 112 p */
    {0x0C, 0x12, 0x12, 0x12, 0x7E}, /* 113 q */
    {0x7C, 0x08, 0x04, 0x04, 0x08}, /* 114 r */
    {0x48, 0x54, 0x54, 0x54, 0x24}, /* 115 s */
    {0x04, 0x3F, 0x44, 0x40, 0x20}, /* 116 t */
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, /* 117 u */
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, /* 118 v */
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, /* 119 w */
    {0x44, 0x28, 0x10, 0x28, 0x44}, /* 120 x */
    {0x0E, 0x50, 0x50, 0x50, 0x3E}, /* 121 y */
    {0x44, 0x64, 0x54, 0x4C, 0x44}, /* 122 z */
    {0x00, 0x08, 0x36, 0x41, 0x00}, /* 123 { */
    {0x00, 0x00, 0x7F, 0x00, 0x00}, /* 124 | */
    {0x00, 0x41, 0x36, 0x08, 0x00}, /* 125 } */
    {0x08, 0x04, 0x08, 0x08, 0x04}, /* 126 ~ */
};

SSD1306_Status ssd1306;
static bool initialized;
static bool restart_pending;
static uint32_t last_bus_recovery;
static bool refresh_requested;
static uint32_t step_started_ms;
static uint32_t step_delay_ms;
static uint16_t frame_offset;
static uint8_t drawing_frame[FRAME_BYTES];
static uint8_t sending_frame[FRAME_BYTES];
static uint8_t transfer_bytes[7U + CHUNK_BYTES];
static I2C_BusRequest request;

static GPIO_TypeDef *const reset_port = SSD1306_RESET_PORT;

static void schedule_step(SSD1306_State next, uint32_t delay_ms, uint32_t now)
{
    ssd1306.state = next;
    step_started_ms = now;
    step_delay_ms = delay_ms;
}

static void start_frame(uint32_t delay_ms, uint32_t now)
{
    memcpy(sending_frame, drawing_frame, sizeof(sending_frame));
    refresh_requested = false;
    frame_offset = 0U;
    schedule_step(SSD1306_CONFIGURE, delay_ms, now);
}

static SSD1306_Error convert_bus_error(I2C_BusResult result)
{
    switch (result)
    {
        case I2C_BUS_NACK:
            return SSD1306_ERROR_NACK;
        case I2C_BUS_TIMEOUT:
            return SSD1306_ERROR_TIMEOUT;
        case I2C_BUS_STUCK:
            return SSD1306_ERROR_BUS_STUCK;
        case I2C_BUS_CONFIG:
            return SSD1306_ERROR_CONFIG;
        default:
            return SSD1306_ERROR_I2C;
    }
}

static void report_error(SSD1306_Error error, uint32_t now)
{
    ssd1306.ready = false;
    ssd1306.error = error;
    schedule_step(SSD1306_ASSERT_RESET, RETRY_INTERVAL_MS, now);
}

bool SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
    if (initialized)
    {
        return false;
    }
    ssd1306 = (SSD1306_Status){.state = SSD1306_ASSERT_RESET};
    uint32_t reset_pin = SSD1306_RESET_PIN;
    if ((reset_port == NULL && reset_pin != 0U) ||
        (reset_port != NULL && (reset_pin == 0U || reset_pin > 0x8000U || (reset_pin & (reset_pin - 1U)) != 0U)) ||
        !I2C_BusInit(hi2c))
    {
        ssd1306.error = SSD1306_ERROR_CONFIG;
        return false;
    }
    request = (I2C_BusRequest){0};
    request.address = SSD1306_ADDRESS;
    request.tx = transfer_bytes;
    request.timeout_ms = I2C_TRANSFER_MS;
    SSD1306_Clear();
    last_bus_recovery = I2C_BusGetRecoveryCount();
    initialized = true;
    SSD1306_SetEnabled(true);
    return true;
}

void SSD1306_SetEnabled(bool enabled)
{
    if (!initialized || ssd1306.enabled == enabled)
    {
        return;
    }
    ssd1306.enabled = enabled;
    ssd1306.ready = false;
    ssd1306.error = SSD1306_ERROR_NONE;
    restart_pending = true;
}

void SSD1306_Clear(void)
{
    memset(drawing_frame, 0, sizeof(drawing_frame));
}

bool SSD1306_WriteLine(uint8_t row, const char *text)
{
    if (row >= SSD1306_TEXT_ROWS || text == NULL)
    {
        return false;
    }
    uint8_t *pixels = &drawing_frame[(uint32_t)row * SSD1306_WIDTH];
    memset(pixels, 0, SSD1306_WIDTH);
    bool exact = true;
    uint32_t column = 0U;
    while (column < SSD1306_TEXT_COLUMNS && text[column] != '\0')
    {
        uint8_t character = (uint8_t)text[column];
        if (character < 32U || character > 126U)
        {
            character = '?';
            exact = false;
        }
        memcpy(&pixels[column * 6U], font[character - 32U], 5U);
        ++column;
    }
    return exact && text[column] == '\0';
}

void SSD1306_Refresh(void)
{
    refresh_requested = true;
}

static void handle_completed_request(uint32_t now)
{
    switch (ssd1306.state)
    {
        case SSD1306_CONFIGURE:
            schedule_step(SSD1306_WRITE_FRAME, BUS_YIELD_MS, now);
            break;
        case SSD1306_WRITE_FRAME:
            frame_offset = (uint16_t)(frame_offset + CHUNK_BYTES);
            schedule_step(frame_offset < FRAME_BYTES ? SSD1306_WRITE_FRAME : SSD1306_TURN_ON, BUS_YIELD_MS, now);
            break;
        case SSD1306_TURN_ON:
            schedule_step(SSD1306_WAIT_VISIBLE, DISPLAY_ON_DELAY_MS, now);
            break;
        case SSD1306_TURN_OFF:
            ssd1306.error = SSD1306_ERROR_NONE;
            schedule_step(SSD1306_IDLE, 0U, now);
            break;
        default:
            report_error(SSD1306_ERROR_CONFIG, now);
            break;
    }
}

static void prepare_request(void)
{
    switch (ssd1306.state)
    {
        case SSD1306_CONFIGURE:
        {
            static const uint8_t settings[] = {
                0xD5, 0x80,                /* Clock divider / oscillator. */
                0xA8, SSD1306_HEIGHT - 1U, /* Multiplex ratio. */
                0xD3, 0x00,
                0x40,       /* Offset=0, start line=0. */
                0xA1, 0xC8, /* Звична орієнтація модуля. */
                0xDA, SSD1306_HEIGHT == 64U ? 0x12 : 0x02,
                0x81, 0x7F, /* Contrast. */
                0xD9, 0xF1,
                0xDB, 0x20, /* Precharge / VCOMH. */
                0xA4, 0xA6,
                0x2E, /* RAM, normal, scrolling OFF. */
                0x23, 0x00,
                0xD6, 0x00, /* Fade/blink/zoom OFF. */
                0x20, 0x02, /* Page addressing. */
                0x8D, 0x14  /* Internal charge pump ON. */
            };
            uint16_t length = 1U;
            transfer_bytes[0] = 0x00;
            if (!ssd1306.ready)
            {
                transfer_bytes[length++] = 0xAE;
            }
            memcpy(&transfer_bytes[length], settings, sizeof(settings));
            request.tx_size = (uint16_t)(length + sizeof(settings));
            break;
        }
        case SSD1306_WRITE_FRAME:
        {
            uint8_t page = (uint8_t)(frame_offset / SSD1306_WIDTH);
            uint8_t column = (uint8_t)(frame_offset % SSD1306_WIDTH);

            transfer_bytes[0] = 0x80;
            transfer_bytes[1] = (uint8_t)(0xB0U | page);
            transfer_bytes[2] = 0x80;
            transfer_bytes[3] = (uint8_t)(column & 0x0FU);
            transfer_bytes[4] = 0x80;
            transfer_bytes[5] = (uint8_t)(0x10U | (column >> 4));
            transfer_bytes[6] = 0x40;
            memcpy(&transfer_bytes[7], &sending_frame[frame_offset], CHUNK_BYTES);
            request.tx_size = sizeof(transfer_bytes);
            break;
        }
        case SSD1306_TURN_ON:
            transfer_bytes[0] = 0x00;
            transfer_bytes[1] = 0xAF;
            request.tx_size = 2U;
            break;
        case SSD1306_TURN_OFF:
            transfer_bytes[0] = 0x00;
            transfer_bytes[1] = 0xAE;
            transfer_bytes[2] = 0x8D;
            transfer_bytes[3] = 0x10;
            request.tx_size = 4U;
            break;
        default:
            break;
    }
}

void SSD1306_Task(void)
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
        ssd1306.ready = false;
        if (ssd1306.enabled)
        {
            ssd1306.error = convert_bus_error(I2C_BusGetStatus());
        }
        restart_pending = true;
    }
    if (request.result == I2C_BUS_PENDING)
    {
        return;
    }
    if (restart_pending && ssd1306.state == SSD1306_RELEASE_RESET && (uint32_t)(now - step_started_ms) < RESET_DELAY_MS)
    {
        return;
    }
    if (restart_pending)
    {
        request.result = I2C_BUS_IDLE;
        restart_pending = false;
        if (reset_port != NULL)
        {
            HAL_GPIO_WritePin(reset_port, SSD1306_RESET_PIN, GPIO_PIN_SET);
        }
        schedule_step(
            ssd1306.enabled ? SSD1306_ASSERT_RESET : SSD1306_TURN_OFF,
            ssd1306.enabled ? STARTUP_DELAY_MS : RESET_DELAY_MS,
            now
        );
    }
    if (request.result != I2C_BUS_IDLE)
    {
        I2C_BusResult result = request.result;
        request.result = I2C_BUS_IDLE;
        if (result == I2C_BUS_OK)
        {
            handle_completed_request(now);
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

    switch (ssd1306.state)
    {
        case SSD1306_ASSERT_RESET:
            if (reset_port != NULL)
            {
                HAL_GPIO_WritePin(reset_port, SSD1306_RESET_PIN, GPIO_PIN_RESET);
                schedule_step(SSD1306_RELEASE_RESET, RESET_DELAY_MS, now);
            }
            else if (ssd1306.enabled)
            {
                start_frame(0U, now);
            }
            else
            {
                schedule_step(SSD1306_TURN_OFF, 0U, now);
            }
            return;
        case SSD1306_RELEASE_RESET:
            HAL_GPIO_WritePin(reset_port, SSD1306_RESET_PIN, GPIO_PIN_SET);
            if (ssd1306.enabled)
            {
                start_frame(RESET_DELAY_MS, now);
            }
            else
            {
                schedule_step(SSD1306_TURN_OFF, RESET_DELAY_MS, now);
            }
            return;
        case SSD1306_WAIT_VISIBLE:
            ssd1306.ready = true;
            ssd1306.error = SSD1306_ERROR_NONE;
            ssd1306.last_frame_ms = now;
            schedule_step(SSD1306_IDLE, 0U, now);
            return;
        case SSD1306_IDLE:
            if (ssd1306.enabled && (refresh_requested || elapsed >= REFRESH_INTERVAL_MS))
            {
                start_frame(0U, now);
            }
            return;
        default:
            break;
    }

    if (elapsed - step_delay_ms >= BUS_ACCESS_MS)
    {
        I2C_BusResult status = I2C_BusGetStatus();
        report_error(
            status == I2C_BUS_IDLE || status == I2C_BUS_PENDING ? SSD1306_ERROR_TIMEOUT : convert_bus_error(status),
            now
        );
        return;
    }
    prepare_request();
    if (I2C_BusStart(&request) == HAL_ERROR)
    {
        request.result = I2C_BUS_IDLE;
        report_error(SSD1306_ERROR_CONFIG, now);
    }
}

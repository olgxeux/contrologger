#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"


#ifndef SSD1306_ADDRESS
#define SSD1306_ADDRESS 0x3CU
#endif
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT 64U
#endif
#define SSD1306_WIDTH 128U
#define SSD1306_TEXT_ROWS (SSD1306_HEIGHT / 8U)
#define SSD1306_TEXT_COLUMNS 21U

#ifndef SSD1306_RESET_PORT
#define SSD1306_RESET_PORT NULL
#endif
#ifndef SSD1306_RESET_PIN
#define SSD1306_RESET_PIN 0U
#endif

#if SSD1306_ADDRESS != 0x3C && SSD1306_ADDRESS != 0x3D
#error "SSD1306_ADDRESS must be 0x3C or 0x3D (7-bit)"
#endif
#if SSD1306_HEIGHT != 32 && SSD1306_HEIGHT != 64
#error "SSD1306_HEIGHT must be 32 or 64"
#endif

typedef enum {
    SSD1306_ASSERT_RESET,
    SSD1306_RELEASE_RESET,
    SSD1306_CONFIGURE,
    SSD1306_WRITE_FRAME,
    SSD1306_TURN_ON,
    SSD1306_WAIT_VISIBLE,
    SSD1306_IDLE,
    SSD1306_TURN_OFF
} SSD1306_State;

typedef enum {
    SSD1306_ERROR_NONE,
    SSD1306_ERROR_CONFIG,
    SSD1306_ERROR_NACK,
    SSD1306_ERROR_I2C,
    SSD1306_ERROR_TIMEOUT,
    SSD1306_ERROR_BUS_STUCK
} SSD1306_Error;

typedef struct {
    SSD1306_State state;
    SSD1306_Error error;
    uint32_t last_frame_ms;
    bool enabled;
    bool ready;
} SSD1306_Status;

extern SSD1306_Status ssd1306;

bool SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Task(void);
void SSD1306_SetEnabled(bool enabled);

void SSD1306_Clear(void);
bool SSD1306_WriteLine(uint8_t row, const char *text);

void SSD1306_Refresh(void);

#endif

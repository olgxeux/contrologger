#ifndef SG90_H
#define SG90_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


 /* These are calibration endpoints, not a guarantee of 180 mechanical degrees.
 *  Start with this limited pulse range; calibrate your servo without load.
 */
#define SG90_PULSE_AT_0_DEG_US    1000U
#define SG90_PULSE_AT_180_DEG_US  2000U
#define SG90_INPUT_TIMEOUT_MS     150U

typedef enum {
    SG90_ERROR_NONE,
    SG90_ERROR_CONFIG,
    SG90_ERROR_TIMER
} SG90_Error;

typedef enum {
    SG90_OFF,
    SG90_WAIT_INPUT,
    SG90_PWM,
    SG90_RETRY_TIMER
} SG90_State;

typedef struct {
    uint16_t min_angle_deg;
    uint16_t max_angle_deg;
    uint16_t target_angle_deg;
    uint16_t command_angle_deg;
    uint16_t pulse_us;
    SG90_Error error;
    SG90_State state;
    bool enabled;
} SG90_Data;

extern SG90_Data sg90;

bool SG90_Init(TIM_HandleTypeDef *timer);
void SG90_Task(void);
void SG90_SetEnabled(bool enabled);
bool SG90_SetLimits(uint16_t min_angle_deg, uint16_t max_angle_deg);
bool SG90_SetTargetAngle(uint16_t angle_deg);
void SG90_ClearTarget(void);                  

#endif

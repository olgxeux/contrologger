#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    bool aht20_on, bmp280_on, display_on, servo_on;
    uint8_t display_mode;
    uint16_t servo_min, servo_max;
} Settings;
bool settings_load(Settings *value);
bool settings_save(const Settings *value);
bool settings_valid(const Settings *value);
bool settings_equal(const Settings *a, const Settings *b);
extern bool settings_damaged; 
#endif

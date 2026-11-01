#ifndef APP_SENSORS_H
#define APP_SENSORS_H
#include <stdbool.h>
void sensors_init(void);
void sensors_process(void);
bool sensors_fault(void);
bool aht20_fresh(void);
bool bmp280_fresh(void);
#endif

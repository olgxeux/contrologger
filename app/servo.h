#ifndef APP_SERVO_H
#define APP_SERVO_H
#include <stdbool.h>
void servo_init(void);
void servo_process(void);
bool servo_fault(void);
bool servo_parking(void);
#endif

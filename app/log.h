#ifndef APP_LOG_H
#define APP_LOG_H
#include <stdbool.h>
void log_init(void);
void log_process(void);
void log_write(const char *format, ...) __attribute__((format(printf, 1, 2)));
bool log_idle(void);
#endif

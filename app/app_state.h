#ifndef APP_STATE_H
#define APP_STATE_H
#include "settings.h"
extern Settings app_settings;
extern bool app_save_pending, app_flash_error;
bool app_request_settings(const Settings *value);
#endif

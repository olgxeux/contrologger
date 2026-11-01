#include "cmd.h"
#include "app_state.h"
#include "log.h"
#include "usart.h"
#include <string.h>
static char line[48];
static unsigned length;
static bool discard;
static uint32_t last_byte;

static bool number(const char *s, uint16_t *value)
{
    unsigned n = 0;
    if (*s < '0' || *s > '9')
        return false;
    do
    {
        n = n * 10U + (unsigned)(*s++ - '0');
        if (n > 180U)
            return false;
    } while (*s >= '0' && *s <= '9');
    if (*s != '\0')
        return false;
    *value = (uint16_t)n;
    return true;
}
static void execute(const char *s)
{
    if (app_save_pending)
    {
        log_write("ERR busy: wait for OK/ERR save");
        return;
    }
    Settings next = app_settings;
    uint16_t n;
    if (strcmp(s, "aht20 on") == 0)
        next.aht20_on = true;
    else if (strcmp(s, "aht20 off") == 0)
        next.aht20_on = false;
    else if (strcmp(s, "bmp280 on") == 0)
        next.bmp280_on = true;
    else if (strcmp(s, "bmp280 off") == 0)
        next.bmp280_on = false;
    else if (strcmp(s, "display on") == 0)
        next.display_on = true;
    else if (strcmp(s, "display off") == 0)
        next.display_on = false;
    else if (strcmp(s, "servo on") == 0)
        next.servo_on = true;
    else if (strcmp(s, "servo off") == 0)
        next.servo_on = false;
    else if (strncmp(s, "display mode ", 13U) == 0 && number(s + 13, &n) && n >= 1U && n <= 4U)
        next.display_mode = (uint8_t)n;
    else if (strncmp(s, "servo min ", 10U) == 0 && number(s + 10, &n))
        next.servo_min = n;
    else if (strncmp(s, "servo max ", 10U) == 0 && number(s + 10, &n))
        next.servo_max = n;
    else
    {
        log_write("ERR command");
        return;
    }
    if (!settings_valid(&next))
    {
        log_write("ERR range: 0 <= min <= max <= 180; mode 1..4");
        return;
    }
    (void)app_request_settings(&next);
}
void cmd_init(void)
{
    length = 0U;
    discard = false;
}
void cmd_discard_input(void)
{
    uint32_t status = huart1.Instance->SR;
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U)
    {
        volatile uint32_t unused = huart1.Instance->DR;
        (void)unused;
        discard = true;
    }
    length = 0U;
}
void cmd_process(void)
{
    uint32_t now = HAL_GetTick();
    if (length != 0U && (uint32_t)(now - last_byte) > 1000U)
    {
        length = 0U;
        discard = true;
    }
    uint32_t status = huart1.Instance->SR;
    uint32_t errors = USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE;
    if ((status & (USART_SR_RXNE | errors)) == 0U)
        return;
    char c = (char)(uint8_t)huart1.Instance->DR;
    last_byte = now;
    if ((status & errors) != 0U)
    {
        length = 0U;
        discard = true;
        return;
    }
    if (c == '\r' || c == '\n')
    {
        if (discard)
            log_write("ERR RX: resend complete command");
        else if (length != 0U)
        {
            line[length] = '\0';
            execute(line);
        }
        length = 0U;
        discard = false;
        return;
    }
    if (discard)
        return;
    if (c < ' ' || c > '~' || length >= sizeof(line) - 1U)
    {
        length = 0U;
        discard = true;
        return;
    }
    line[length++] = c;
}

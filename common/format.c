#include "format.h"
#include <stdint.h>
#include <stdio.h>
void format_number(char *out, size_t size, float value)
{
    if (!(value >= -1000000.0f && value <= 1000000.0f))
    {
        (void)snprintf(out, size, "INVALID");
        return;
    }
    int32_t n = (int32_t)((double)value * 100.0 + (value < 0 ? -0.5 : 0.5));
    uint32_t magnitude = (uint32_t)(n < 0 ? -n : n);
    (void)snprintf(
        out,
        size,
        "%s%lu.%02lu",
        n < 0 ? "-" : "",
        (unsigned long)(magnitude / 100U),
        (unsigned long)(magnitude % 100U)
    );
}

void format_u64(char out[21], uint64_t value)
{
    char reversed[20];
    unsigned n = 0U;
    do
    {
        reversed[n++] = (char)('0' + (unsigned)(value % 10U));
        value /= 10U;
    } while (value);
    unsigned i = 0U;
    while (n)
        out[i++] = reversed[--n];
    out[i] = '\0';
}

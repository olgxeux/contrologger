#ifndef FORMAT_H
#define FORMAT_H
#include <stddef.h>
#include <stdint.h>
void format_u64(char out[21], uint64_t value);
void format_number(char *out, size_t size, float value);
#endif

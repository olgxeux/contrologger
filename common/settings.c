#include "settings.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define BASE0 0x08040000UL
#define BASE1 0x08060000UL
#define SECTOR_BYTES 0x20000UL
#define MAGIC 0x43464701UL
#define COMMIT 0xA55AC33CUL
#define WORDS 8U
#define SLOTS (SECTOR_BYTES / (WORDS * 4U))
static const uint32_t base[2] = {BASE0, BASE1};
static uint32_t next_slot[2], last_sequence;
static int active_sector;
bool settings_damaged;

bool settings_valid(const Settings *s)
{
    return s->display_mode >= 1U && s->display_mode <= 4U && s->servo_min <= s->servo_max && s->servo_max <= 180U;
}
bool settings_equal(const Settings *a, const Settings *b)
{
    return a->aht20_on == b->aht20_on && a->bmp280_on == b->bmp280_on && a->display_on == b->display_on &&
           a->servo_on == b->servo_on && a->display_mode == b->display_mode && a->servo_min == b->servo_min &&
           a->servo_max == b->servo_max;
}
static uint32_t crc32(const uint32_t *words)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (unsigned i = 0; i < 6U; ++i)
    {
        uint32_t word = words[i];
        for (unsigned byte = 0; byte < 4U; ++byte)
        {
            crc ^= word & 255U;
            word >>= 8;
            for (unsigned bit = 0; bit < 8U; ++bit)
                crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return ~crc;
}
static bool decode(const uint32_t *r, Settings *s)
{
    if (r[0] != MAGIC || r[7] != COMMIT || r[6] != crc32(r) || r[2] > 15U || r[3] < 1U || r[3] > 4U || r[4] > r[5] ||
        r[5] > 180U)
        return false;
    *s = (Settings
    ){(r[2] & 1U) != 0U,
      (r[2] & 2U) != 0U,
      (r[2] & 4U) != 0U,
      (r[2] & 8U) != 0U,
      (uint8_t)r[3],
      (uint16_t)r[4],
      (uint16_t)r[5]};
    return true;
}
bool settings_load(Settings *value)
{
    *value = (Settings){true, true, true, false, 1U, 30U, 150U};
    active_sector = -1;
    last_sequence = 0U;
    next_slot[0] = next_slot[1] = 0U;
    settings_damaged = false;
    for (unsigned bank = 0; bank < 2U; ++bank)
    {
        for (uint32_t slot = 0; slot < SLOTS; ++slot)
        {
            const uint32_t *r = (const uint32_t *)(uintptr_t)(base[bank] + slot * 32U);
            bool blank = true;
            for (unsigned i = 0; i < WORDS; ++i)
                if (r[i] != 0xFFFFFFFFU)
                    blank = false;
            if (blank)
                continue;
            next_slot[bank] = slot + 1U;
            Settings candidate;
            if (!decode(r, &candidate))
            {
                settings_damaged = true;
                continue;
            }
            uint32_t distance = r[1] - last_sequence;
            if (active_sector < 0 || (distance != 0U && distance < 0x80000000U))
            {
                active_sector = (int)bank;
                last_sequence = r[1];
                *value = candidate;
            }
        }
    }
    return active_sector >= 0;
}
static void reset_data_cache(void)
{
    bool enabled = (FLASH->ACR & FLASH_ACR_DCEN) != 0U;
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    if (enabled)
        __HAL_FLASH_DATA_CACHE_ENABLE();
}
bool settings_save(const Settings *value)
{
    if (!settings_valid(value))
        return false;
    Settings previous;
    bool found = settings_load(&previous);
    if (found && settings_equal(value, &previous))
        return true;
    unsigned bank = active_sector < 0 ? 0U : (unsigned)active_sector;
    bool erase = next_slot[bank] >= SLOTS;
    if (erase)
        bank ^= 1U;
    uint32_t slot = erase ? 0U : next_slot[bank];
    uint32_t address = base[bank] + slot * 32U;
    uint32_t r[WORDS] = {
        MAGIC,
        last_sequence + 1U,
        (value->aht20_on ? 1U : 0U) | (value->bmp280_on ? 2U : 0U) | (value->display_on ? 4U : 0U) |
            (value->servo_on ? 8U : 0U),
        value->display_mode,
        value->servo_min,
        value->servo_max,
        0U,
        COMMIT
    };
    r[6] = crc32(r);
    if (HAL_FLASH_Unlock() != HAL_OK)
        return false;
    __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR
    );
    bool ok = true;
    if (erase)
    {
        FLASH_EraseInitTypeDef e = {0};
        uint32_t failed_sector;
        e.TypeErase = FLASH_TYPEERASE_SECTORS;
        e.Sector = bank == 0U ? FLASH_SECTOR_6 : FLASH_SECTOR_7;
        e.NbSectors = 1U;
        e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        ok = HAL_FLASHEx_Erase(&e, &failed_sector) == HAL_OK;
        reset_data_cache();
    }
    for (unsigned i = 0; ok && i < WORDS; ++i)
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4U, r[i]) == HAL_OK;
    if (HAL_FLASH_Lock() != HAL_OK)
        ok = false;
    reset_data_cache();
    Settings readback;
    const uint32_t *stored = (const uint32_t *)(uintptr_t)address;
    return ok && memcmp(stored, r, sizeof(r)) == 0 && decode(stored, &readback) && settings_equal(value, &readback);
}

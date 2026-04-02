/* Feetech STS3215 — Utility functions */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Baud rate table — matches vtables.c baud_rate_config but with struct access */
struct baud_entry { uint32_t baud; uint32_t bit_time; };
const struct baud_entry baud_table_ext[8] = {
    { 1000000, 10 },
    {  500000, 20 },
    {  250000, 40 },
    {  128000, 79 },
    {  115107, 87 },
    {   76800, 131 },
    {   57600, 174 },
    {   38400, 261 },
};

/* Simple forward byte copy for register buffer copies. */
void __attribute__((noinline)) subsys_memcpy(void *dst, const void *src, int count)
{
    uint8_t *d = (uint8_t *)dst - 1;
    const uint8_t *s = (const uint8_t *)src;
    const uint8_t *end = s + count;
    for (; s != end; s++) {
        d++;
        *d = *s;
    }
}

/* Simple forward byte fill. */
void __attribute__((noinline)) subsys_memset(uint8_t *dst, uint8_t val, int count)
{
    uint8_t *end = dst + count;
    for (; dst != end; dst++) {
        *dst = val;
    }
}

/* Clamp baud rate index to valid range [0,7]. If > 7, returns 0. */
uint32_t __attribute__((noinline)) clamp_baud_index(uint32_t unused, uint32_t idx)
{
    (void)unused;
    if (idx > 7) {
        idx = 0;
    }
    return idx;
}

/* Return the TX bit time for the given baud index. */
uint32_t __attribute__((noinline)) baud_tx_time(uint32_t unused, uint32_t idx)
{
    (void)unused;
    uint32_t val;
    if (idx < 8) {
        val = baud_table_ext[idx].bit_time;
    } else {
        val = 10;
    }
    return val;
}

/* Write a byte at base + offset. */
void __attribute__((noinline)) byte_write(uint8_t *base, uint32_t offset, uint8_t val)
{
    base[offset] = val;
}


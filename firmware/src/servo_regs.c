/* Feetech STS3215 — Servo register management and EEPROM */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t eeprom_ctrl_arr[]; extern uint8_t * volatile eeprom_ctrl;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t reg_permissions[88];
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;

/* Functions from other files */
extern uint32_t eeprom_page_verify(uint8_t *regs, uint16_t *page);
extern void byte_write(uint8_t *base, uint32_t offset, uint8_t val);
extern void eeprom_regs_to_page(uint8_t *regs, uint16_t *page);
extern void eeprom_page_to_regs(uint8_t *regs, uint16_t *page);
extern void flash_page_erase(uint32_t page_addr);
extern void flash_write_halfword(uint16_t *addr, uint16_t value);
extern void servo_defaults_init(void);
extern void subsys_memcpy(void *dst, const void *src, int count);
extern void baud_rate_change(uint8_t *param, uint32_t baud_idx);

extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;
extern uint16_t position_lut[32];

/* Forward declarations within this file */
void regs_to_buf(uint8_t *src, uint8_t *dst, int32_t offset, int32_t count);
void buf_to_regs(uint8_t *dst, uint8_t *src, int32_t offset, int32_t count);
void eeprom_save_byte_internal(uint8_t *ee_ctrl);
uint16_t * eeprom_find_active_page(uint8_t *ee_ctrl, int32_t mode);

/* ================================================================
 * EEPROM group table — maps contiguous blocks of EEPROM-backed
 * registers to positions in the EEPROM flash page.
 *
 * Previously packed into pwm_ctrl_arr at magic offsets (EE_GRP_*),
 * limited to exactly 2 groups. Now a standalone struct supporting
 * up to MAX_EEPROM_GROUPS, so any register can be EEPROM-backed
 * without breaking the metadata layout.
 * ================================================================ */
#define MAX_EEPROM_GROUPS 8

static struct {
    uint8_t count;                        /* number of groups found */
    uint8_t start[MAX_EEPROM_GROUPS];     /* first register in each group */
    uint8_t size[MAX_EEPROM_GROUPS];      /* byte count per group */
    uint8_t offset[MAX_EEPROM_GROUPS];    /* cumulative byte offset in EEPROM page */
} ee_groups;

/* Map register address to EEPROM page byte offset.
 * Returns the offset, or 0 if not in any group. */
uint32_t __attribute__((noinline)) servo_apply_baud(uint8_t *param, int32_t addr)
{
    (void)param;
    for (uint8_t g = 0; g < ee_groups.count; g++) {
        uint32_t s = ee_groups.start[g];
        if ((uint32_t)addr >= s && (uint32_t)addr < s + ee_groups.size[g]) {
            return (addr - s + ee_groups.offset[g]) & 0xFF;
        }
    }
    return 0;
}

/* Reverse mapping — from EEPROM page byte offset back to register address.
 * Returns the mapped offset, or -1 if not in any group. */
int32_t __attribute__((noinline)) servo_apply_config(uint8_t *param, int32_t addr)
{
    (void)param;
    uint32_t adj = addr & 0xFF;
    for (uint8_t g = 0; g < ee_groups.count; g++) {
        uint32_t s = ee_groups.start[g];
        if (adj >= s && adj < s + ee_groups.size[g]) {
            return (ee_groups.offset[g] - s) + adj;
        }
    }
    return -1;
}

/* ================================================================
 * Scan reg_permissions to build EEPROM address mapping tables.
 * Groups consecutive EEPROM-backed registers and computes offset
 * mappings for servo_apply_baud / servo_apply_config.
 * ================================================================ */
void __attribute__((noinline)) servo_regs_load_eeprom(uint8_t *param)
{
    (void)param;
    uint8_t grp = 0;
    int was_eeprom = 0;
    uint8_t cumulative_offset = 0;

    /* Zero the table */
    ee_groups.count = 0;
    for (int g = 0; g < MAX_EEPROM_GROUPS; g++) {
        ee_groups.start[g] = 0;
        ee_groups.size[g] = 0;
        ee_groups.offset[g] = 0;
    }

    for (int i = 0; i < 0x57; i++) {
        if ((reg_permissions[i] & 4) == 0) {
            /* Not EEPROM-backed */
            if (was_eeprom && grp < MAX_EEPROM_GROUPS) {
                /* Close current group, start next */
                grp++;
                if (grp < MAX_EEPROM_GROUPS) {
                    ee_groups.offset[grp] = cumulative_offset;
                }
            }
            if (grp < MAX_EEPROM_GROUPS) {
                ee_groups.start[grp] = (uint8_t)(i + 1);
            }
            was_eeprom = 0;
        } else {
            /* EEPROM-backed — grow current group */
            if (grp < MAX_EEPROM_GROUPS) {
                ee_groups.size[grp]++;
                cumulative_offset++;
            }
            was_eeprom = 1;
        }
    }
    ee_groups.count = was_eeprom ? grp + 1 : grp;
    if (ee_groups.count > MAX_EEPROM_GROUPS)
        ee_groups.count = MAX_EEPROM_GROUPS;
}

/* ================================================================
 * Load register data from EEPROM page. If firmware version header
 * doesn't match (3, 10, 0, 9), resets to factory defaults.
 * ================================================================ */
void __attribute__((noinline)) eeprom_page_load_defaults(uint8_t *param)
{
    (void)param;
    uint8_t *sr = servo_regs_arr;
    uint8_t *ec = eeprom_ctrl_arr;

    /* Load all EEPROM groups from working buffer into servo_regs */
    for (uint8_t g = 0; g < ee_groups.count; g++) {
        regs_to_buf(ec, sr + ee_groups.start[g],
                    ee_groups.offset[g], ee_groups.size[g]);
    }

    /* Check EEPROM validity: version must be {3, ?, ?, 9}.
     * If not (blank or corrupt EEPROM), load firmware defaults. */
    if (sr[0] != 3 || sr[3] != 9) {
        servo_defaults_init();
        sr[0] = 3;
        sr[1] = 10;
        sr[2] = 0;
        sr[3] = 9;
        /* Sync defaults + version back to EEPROM working buffer.
         * Without this, the working buffer retains 0xFF from the
         * erased/invalid page, and any subsequent EEPROM save writes
         * corrupted version headers — causing defaults to reload on
         * every boot and making register writes non-persistent. */
        for (uint8_t g = 0; g < ee_groups.count; g++) {
            buf_to_regs(ec, sr + ee_groups.start[g],
                        ee_groups.offset[g], ee_groups.size[g]);
        }
    } else {
        sr[0] = 3;
        sr[1] = 10;
        sr[2] = 0;
        sr[3] = 9;
    }

    /* Fix uninitialized PI gains from old EEPROM pages that predate
     * regs 50-53 being EEPROM-backed. Those page positions were never
     * written, so they read as 0xFFFF. Replace with firmware defaults. */
    if (*(uint16_t *)(sr + SR_CURRENT_KP_LO) == 0xFFFF)
        *(uint16_t *)(sr + SR_CURRENT_KP_LO) = 3;
    if (*(uint16_t *)(sr + SR_CURRENT_KI_LO) == 0xFFFF)
        *(uint16_t *)(sr + SR_CURRENT_KI_LO) = 50;
}

/* Save current register state to EEPROM. */
void __attribute__((noinline)) eeprom_save_regs(uint8_t *param)
{
    (void)param;
    uint8_t *sr = servo_regs_arr;
    uint8_t *ec = eeprom_ctrl_arr;
    uint8_t saved_byte5 = sr[5];

    /* Compute total offset (where backup copies start in the mirror) */
    uint8_t total = 0;
    for (uint8_t g = 0; g < ee_groups.count; g++)
        total += ee_groups.size[g];

    /* Load backup copies from mirror into servo_regs */
    uint8_t save_off = total;
    for (uint8_t g = 0; g < ee_groups.count; g++) {
        regs_to_buf(ec, sr + ee_groups.start[g],
                    save_off, ee_groups.size[g]);
        save_off += ee_groups.size[g];
    }

    sr[0] = 3;
    sr[1] = 10;
    sr[2] = 0;
    sr[3] = 9;
    sr[5] = saved_byte5;

    /* Write servo_regs back to mirror at primary offsets */
    for (uint8_t g = 0; g < ee_groups.count; g++) {
        buf_to_regs(ec, sr + ee_groups.start[g],
                    ee_groups.offset[g], ee_groups.size[g]);
    }

    eeprom_save_byte_internal(ec);
}

/* Write firmware header to EEPROM page and all register groups to flash. */
void __attribute__((noinline, optimize("no-optimize-sibling-calls"))) eeprom_page_header_write(uint8_t *param)
{
    (void)param;
    uint8_t *sr = servo_regs_arr;
    uint8_t *ec = eeprom_ctrl_arr;

    sr[0] = 3;
    sr[1] = 10;
    sr[2] = 0;
    sr[3] = 9;

    /* Compute save offset (after primary data) */
    uint8_t save_off = 0;
    for (uint8_t g = 0; g < ee_groups.count; g++)
        save_off += ee_groups.size[g];

    /* Write all groups to mirror at save offsets */
    for (uint8_t g = 0; g < ee_groups.count; g++) {
        buf_to_regs(ec, sr + ee_groups.start[g],
                    save_off, ee_groups.size[g]);
        save_off += ee_groups.size[g];
    }

    eeprom_save_byte_internal(ec);
}

/* Copy 'count' bytes from (src + offset) to dst. */
void __attribute__((noinline)) regs_to_buf(uint8_t *src, uint8_t *dst, int32_t offset, int32_t count)
{
    if (count != 0) {
        uint32_t i = 0;
        do {
            dst[i] = src[offset + i];
            i = (i + 1) & 0xFFFF;
        } while ((int32_t)i < count);
    }
}

/* Copy 'count' bytes from src to (dst + offset). */
void __attribute__((noinline)) buf_to_regs(uint8_t *dst, uint8_t *src, int32_t offset, int32_t count)
{
    if (count != 0) {
        uint32_t i = 0;
        do {
            dst[offset + i] = src[i];
            i = (i + 1) & 0xFFFF;
        } while ((int32_t)i < count);
    }
}

/* ================================================================
 * Check if a register write overlaps the range [start, start+count).
 * Returns 1 if overlap, 0 if not.
 * ================================================================ */
uint32_t __attribute__((noinline, section(".text.keep"))) reg_change_detect(uint8_t *param, int32_t addr,
    int32_t start, int32_t count, uint8_t len)
{
    (void)param;
    volatile uint32_t base = (uint32_t)servo_regs_arr;
    uint32_t adj = ((uint32_t)addr - base) & 0xFF;
    if ((uint32_t)((count - 1 + start) & 0xFF) < adj) {
        return 0;
    }
    if ((int32_t)(((len - 1) + adj) & 0xFF) < start) {
        return 0;
    }
    return 1;
}

/* ================================================================
 * Register write side effects — apply EEPROM save, baud change, etc.
 * For each EEPROM-backed+writable register, remaps address and
 * writes to EEPROM page.
 * ================================================================ */
void __attribute__((noinline)) reg_write_side_effects(uint8_t *param, uint8_t *regs,
    int32_t start_addr, int32_t count)
{
    {
        volatile uint8_t *sr = servo_regs;
        if (sr[SR_LOCK] != 0) return;
    }
    if (count == 0) return;

    uint32_t i = 0;
    int any_saved = 0;
    do {
        int32_t idx = i + start_addr;
        if ((reg_permissions[idx] & 4) && (reg_permissions[idx] & 2)) {
            uint16_t mapped = servo_apply_baud(param, (start_addr + i) & 0xFF);
            byte_write(eeprom_ctrl_arr, mapped, regs[idx]);
            any_saved = 1;
        }
        i = (i + 1) & 0xFF;
    } while ((int32_t)i < count);

    if (any_saved) {
        eeprom_save_byte_internal(eeprom_ctrl_arr);
    }
}

/* Unlock flash, find active page, write changed regs, lock flash. */
void __attribute__((noinline)) eeprom_save_byte_internal(uint8_t *ee_ctrl)
{
    register volatile uint32_t *fmc __asm__("r4") = (volatile uint32_t *)0x40022000;
    register uint32_t key __asm__("r3") = 0x45670123;
    __asm__ __volatile__(
        "str %[key], [%[fmc], #4]\n\t"
        "add.w %[key], %[key], #0x88888888\n\t"
        "str %[key], [%[fmc], #4]"
        : [key] "+r" (key)
        : [fmc] "r" (fmc)
        : "memory"
    );

    uint16_t *page = eeprom_find_active_page(ee_ctrl, 2);
    eeprom_regs_to_page(ee_ctrl, page);

    __asm__ __volatile__(
        "ldr r3, [%[fmc], #16]\n\t"
        "orr.w r3, r3, #0x80\n\t"
        "str r3, [%[fmc], #16]"
        :
        : [fmc] "r" (fmc)
        : "r3", "memory"
    );
}

/* ================================================================
 * Scan two EEPROM pages in flash to find which is active.
 * Page status word: 0xFFFF = erased, 0x0000 = active.
 * ================================================================ */
uint16_t * __attribute__((noinline)) eeprom_find_active_page(uint8_t *ee_ctrl, int32_t mode)
{
    uint16_t *page0 = *(uint16_t **)(ee_ctrl + EE_PAGE0_PTR);
    uint16_t *page1 = *(uint16_t **)(ee_ctrl + EE_PAGE1_PTR);
    int16_t status0 = *page0;
    int16_t status1 = *page1;

    if (status0 == -1) {
        if (status1 == -1) {
            if (mode == 2) {
                flash_write_halfword(page0, 0);
            }
            return *(uint16_t **)(ee_ctrl + EE_PAGE0_PTR);
        }
    } else if (status0 == 0) {
        if (status1 == -1) {
            if (mode != 1) {
                int32_t verify = eeprom_page_verify(ee_ctrl, page0);
                if (verify == 0) {
                    flash_write_halfword(*(uint16_t **)(ee_ctrl + EE_PAGE1_PTR), 0);
                    return *(uint16_t **)(ee_ctrl + EE_PAGE1_PTR);
                }
                return *(uint16_t **)(ee_ctrl + EE_PAGE0_PTR);
            }
        } else if (status1 == 0) {
            page0 = page1;
            if (mode != 1) {
                int32_t verify = eeprom_page_verify(ee_ctrl, page1);
                if (verify == 0) {
                    flash_page_erase(*(uint32_t *)(ee_ctrl + EE_PAGE0_PTR));
                    flash_write_halfword(*(uint16_t **)(ee_ctrl + EE_PAGE0_PTR), 0);
                    return *(uint16_t **)(ee_ctrl + EE_PAGE0_PTR);
                }
                return *(uint16_t **)(ee_ctrl + EE_PAGE1_PTR);
            }
        } else {
            goto recovery;
        }
        return page0;
    }

recovery:
    if (mode == 1) {
        volatile uint32_t * volatile fmc = (volatile uint32_t *)0x40022000U;
        uint32_t key1 = 0x45670123U;
        fmc[1] = key1;
        fmc[1] = key1 - 0x77777778U;
        flash_page_erase(*(uint32_t *)(ee_ctrl + EE_PAGE0_PTR));
        fmc[4] |= 0x80U;
    } else {
        flash_page_erase((uint32_t)page0);
        flash_write_halfword(*(uint16_t **)(ee_ctrl + EE_PAGE0_PTR), 0);
    }
    return *(uint16_t **)(ee_ctrl + EE_PAGE0_PTR);
}

/* Inverse of position_linearize. */
int32_t __attribute__((noinline)) position_delinearize(uint8_t *param, int32_t val)
{
    (void)param;
    int16_t sval = (int16_t)val;

    if (val < ENCODER_HALF_REV) {
        return val;
    }
    if (val == ENCODER_HALF_REV) {
        return 0;
    }
    if (val < ENCODER_FULL_REV) {
        return (int32_t)(int16_t)(ENCODER_HALF_REV - sval);
    }
    if (val > (ENCODER_FULL_REV + ENCODER_HALF_REV - 1)) {
        if (val < (ENCODER_FULL_REV * 2)) {
            return (int32_t)(int16_t)(ENCODER_FULL_REV - sval);
        }
        return 0;
    }
    return (int32_t)(int16_t)(sval - ENCODER_HALF_REV);
}

/* Call eeprom_page_header_write on pwm_ctrl_arr. */
void __attribute__((noinline)) eeprom_header_write_wrap(void)
{
    eeprom_page_header_write(pwm_ctrl_arr);
}


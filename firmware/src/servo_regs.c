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


/* Map register address to EEPROM page offset. */
uint32_t __attribute__((noinline)) servo_apply_baud(uint8_t *param, int32_t addr)
{
    int idx;
    uint32_t start = (uint32_t)param[EE_GRP_START_BASE];

    if (addr >= (int32_t)start && addr < (int32_t)(param[EE_GRP_SIZE_BASE] + start)) {
        idx = 0;
    } else {
        start = (uint32_t)param[EE_GRP_START1];
        if (addr < (int32_t)start) {
            return 0;
        }
        if (addr >= (int32_t)(param[EE_GRP_SIZE2_BASE] + start)) {
            return 0;
        }
        idx = 1;
    }
    return (addr + (param[idx + EE_GRP_OFFSET_BASE] - start)) & 0xFF;
}

/* Reverse mapping — from EEPROM page offset back to register address. */
int32_t __attribute__((noinline)) servo_apply_config(uint8_t *param, int32_t addr)
{
    int idx;
    uint32_t adj = addr & 0xFF;
    uint32_t start = (uint32_t)param[EE_GRP_START_BASE];

    if (adj >= start && adj < param[EE_GRP_SIZE_BASE] + start) {
        idx = 0;
    } else {
        start = (uint32_t)param[EE_GRP_START1];
        if (adj < start) {
            return -1;
        }
        if (adj >= param[EE_GRP_SIZE2_BASE] + start) {
            return -1;
        }
        idx = 1;
    }
    return (param[idx + EE_GRP_OFFSET_BASE] - start) + adj;
}

/* ================================================================
 * Scan reg_permissions to build EEPROM address mapping tables.
 * Groups consecutive EEPROM-backed registers and computes offset
 * mappings for servo_apply_baud / servo_apply_config.
 * ================================================================ */
void __attribute__((noinline)) servo_regs_load_eeprom(uint8_t *param)
{
    uint32_t grp = 0;
    int was_eeprom = 0;

    for (int i = 0; i < 0x57; i++) {
        if ((reg_permissions[i] & 4) == 0) {
            /* Not EEPROM-backed */
            if (was_eeprom) {
                grp = (grp + 1) & 0xFF;
                param[grp + EE_GRP_OFFSET_BASE] = param[(grp - 1) + EE_GRP_SIZE_BASE] + param[(grp - 1) + EE_GRP_OFFSET_BASE];
            }
            param[grp + EE_GRP_START_BASE] = (uint8_t)(i + 1);
            was_eeprom = 0;
        } else {
            /* EEPROM-backed — increment size of current group */
            param[grp + EE_GRP_SIZE_BASE] = param[grp + EE_GRP_SIZE_BASE] + 1;
            was_eeprom = 1;
        }
    }
    /* Finalize: build total-offset and mirror entries */
    uint8_t last_sum = param[grp + EE_GRP_OFFSET_BASE] + param[grp + EE_GRP_SIZE_BASE];
    param[(grp + 1) + EE_GRP_OFFSET_BASE] = last_sum;
    param[(grp + 1) + EE_GRP_SIZE_BASE] = param[EE_GRP_SIZE_BASE];
    param[(grp + 2) + EE_GRP_OFFSET_BASE] = last_sum + param[EE_GRP_SIZE_BASE];
    param[(grp + 2) + EE_GRP_SIZE_BASE] = param[EE_GRP_SIZE2_BASE];
}

/* ================================================================
 * Load register data from EEPROM page. If firmware version header
 * doesn't match (3, 10, 0, 9), resets to factory defaults.
 * ================================================================ */
void __attribute__((noinline)) eeprom_page_load_defaults(uint8_t *param)
{
    uint8_t *sr = servo_regs_arr;
    uint8_t *ec = eeprom_ctrl_arr;
    regs_to_buf(ec, sr + param[EE_GRP_START_BASE],
                param[EE_GRP_OFFSET_BASE], param[EE_GRP_SIZE_BASE]);
    regs_to_buf(ec, sr + param[EE_GRP_START1],
                param[EE_GRP_OFFSET_BASE + 1], param[EE_GRP_SIZE2_BASE]);

    /* Check EEPROM validity: original at 0x068E checks sr[0]==3,
     * then at 0x06B0 checks sr[3]==9. Both must pass or defaults load.
     * After check, version is always set to {3, 10, 0, 9}. */
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
        buf_to_regs(ec, sr + param[EE_GRP_START_BASE],
                    param[EE_GRP_OFFSET_BASE], param[EE_GRP_SIZE_BASE]);
        buf_to_regs(ec, sr + param[EE_GRP_START1],
                    param[EE_GRP_OFFSET_BASE + 1], param[EE_GRP_SIZE2_BASE]);
    } else {
        sr[0] = 3;
        sr[1] = 10;
        sr[2] = 0;
        sr[3] = 9;
    }
}

/* Save current register state to EEPROM. */
void __attribute__((noinline)) eeprom_save_regs(uint8_t *param)
{
    uint8_t *sr = servo_regs_arr;
    uint8_t *ec = eeprom_ctrl_arr;
    uint8_t saved_byte5 = sr[5];

    regs_to_buf(ec, sr + param[EE_GRP_START_BASE],
                param[EE_GRP_SAVE_OFF0], param[EE_GRP_SAVE_SIZE0]);
    regs_to_buf(ec, sr + param[EE_GRP_START1],
                param[EE_GRP_SAVE_OFF1], param[EE_GRP_SAVE_SIZE1]);

    sr[0] = 3;
    sr[1] = 10;
    sr[2] = 0;
    sr[3] = 9;
    sr[5] = saved_byte5;

    buf_to_regs(ec, sr + param[EE_GRP_START_BASE],
                param[EE_GRP_OFFSET_BASE], param[EE_GRP_SIZE_BASE]);
    buf_to_regs(ec, sr + param[EE_GRP_START1],
                param[EE_GRP_OFFSET_BASE + 1], param[EE_GRP_SIZE2_BASE]);

    eeprom_save_byte_internal(ec);
}

/* Write firmware header to EEPROM page and both register groups to flash. */
void __attribute__((noinline, optimize("no-optimize-sibling-calls"))) eeprom_page_header_write(uint8_t *param)
{
    uint8_t *sr = servo_regs_arr;
    uint8_t *ec = eeprom_ctrl_arr;

    sr[0] = 3;
    sr[1] = 10;
    sr[2] = 0;
    sr[3] = 9;

    buf_to_regs(ec, sr + param[EE_GRP_START_BASE],
                param[EE_GRP_SAVE_OFF0], param[EE_GRP_SAVE_SIZE0]);
    buf_to_regs(ec, sr + param[EE_GRP_START1],
                param[EE_GRP_SAVE_OFF1], param[EE_GRP_SAVE_SIZE1]);

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


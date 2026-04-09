/* Flash Memory Controller (FMC) and EEPROM Emulation */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>

/* Use SPL's FMC register definitions from gd32f1x0_fmc.h */

extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t eeprom_ctrl_arr[];

/* ================================================================
 * eeprom_page_setup — Initialize EEPROM page pointers
 *
 * Sets up the page pointers at eeprom_ctrl+0x200 and +0x204.
 * Called as a constructor (via .init_array) before main().
 * Original stores: page0 = 0x08007400, page1 = 0x08007600, size = 0x200
 * ================================================================ */
void __attribute__((constructor)) eeprom_page_setup(void)
{
    *(uint32_t *)(eeprom_ctrl_arr + EE_PAGE0_PTR) = 0x08007400U;
    *(uint16_t *)(eeprom_ctrl_arr + EE_PAGE_SIZE) = 0x0200;
    *(uint32_t *)(eeprom_ctrl_arr + EE_PAGE1_PTR) = 0x08007600U;
}

/* ================================================================
 * flash_page_erase — Erase a 1KB flash page
 *
 * Sets PER bit, writes page address, triggers erase, waits for BSY.
 * ================================================================ */
void __attribute__((noinline)) flash_page_erase(uint32_t page_addr)
{
    volatile uint32_t *fmc = (volatile uint32_t *)0x40022000U;
    __asm__ volatile ("" : "+r"(fmc));
    fmc[4] |= 0x02;            /* CTL |= PER: page erase */
    fmc[5] = page_addr;        /* ADDR = target page address */
    fmc[4] |= 0x40;            /* CTL |= STRT: start erase */
    /* CRITICAL: BSY is not asserted instantly after STRT — there is a
     * propagation delay across the AHB→FMC bridge. If we read FMC_STAT
     * on the very next instruction, BSY may still read 0 and the loop
     * exits before the erase has even begun. We then clear PER, which
     * aborts the in-flight erase and leaves flash in an inconsistent
     * state — BSY then *genuinely* hangs on the NEXT flash op.
     * Force a few cycles of delay before the first STAT read. */
    __asm__ volatile ("nop\n nop\n nop\n nop\n nop\n nop\n");
    while (fmc[3] & 1) {}      /* Wait for BSY to clear */
    fmc[4] &= ~0x02U;          /* CTL &= ~PER */
}

/* ================================================================
 * flash_write_halfword — Write 16-bit value to flash
 *
 * Sets PG bit, writes halfword, waits for BSY.
 * ================================================================ */
void __attribute__((noinline)) flash_write_halfword(uint16_t *addr, uint16_t value)
{
    volatile uint32_t *fmc = (volatile uint32_t *)0x40022000U;
    __asm__ volatile ("" : "+r"(fmc));
    fmc[4] |= 0x01;            /* CTL |= PG: programming */
    *addr = value;              /* Write halfword to flash */
    /* See flash_page_erase — same BSY-propagation race applies here. */
    __asm__ volatile ("nop\n nop\n nop\n nop\n nop\n nop\n");
    while (fmc[3] & 1) {}      /* Wait for BSY */
    fmc[4] &= ~0x01U;          /* CTL &= ~PG */
}

/* ================================================================
 * eeprom_page_to_regs — Copy EEPROM page data to registers
 *
 * Reads 255 halfwords from flash page (skipping header) into
 * the register table as sequential bytes.
 *
 * Page format: [STATUS_HW][reg0|reg1][reg2|reg3]...[reg508|reg509]
 * ================================================================ */
void __attribute__((noinline)) eeprom_page_to_regs(uint8_t *regs, uint16_t *page)
{
    int i = 0;
    do {
        page = page + 1;  /* skip to next halfword */
        *(uint16_t *)(regs + i * 2) = *page;
        i++;
    } while (i != 0xFF);
}

/* ================================================================
 * eeprom_regs_to_page — Write changed registers to flash page
 *
 * Compares register table with page data. For each difference where
 * the flash is erased (0xFFFF), writes the register value.
 * ================================================================ */
void __attribute__((noinline)) eeprom_regs_to_page(uint8_t *regs, uint16_t *page)
{
    int i = 0;
    do {
        page = page + 1;
        uint16_t reg_val = *(uint16_t *)(regs + i * 2);
        if (reg_val != *page && *page == 0xFFFF) {
            flash_write_halfword(page, reg_val);
        }
        i++;
    } while (i != 0xFF);
}

/* ================================================================
 * eeprom_page_verify — Check if page matches registers
 *
 * Returns 1 if all non-erased entries match, 0 if mismatch found.
 * ================================================================ */
uint32_t __attribute__((noinline)) eeprom_page_verify(uint8_t *regs, uint16_t *page)
{
    uint32_t i = 0;
    while (1) {
        uint16_t page_val = page[1 + i];  /* skip header */
        uint16_t reg_val = *(uint16_t *)(regs + i * 2);
        if (reg_val != page_val && page_val != 0xFFFF) {
            return 0;  /* mismatch */
        }
        i = (i + 1) & 0xFFFF;
        if (i > 0xFE) return 1;
    }
}

extern uint16_t * eeprom_find_active_page(uint8_t *ee_ctrl, int32_t mode);

/* Find active page and load register values from it. */
void __attribute__((noinline, optimize("no-optimize-sibling-calls"))) eeprom_init(uint8_t *ee_ctrl)
{
    uint16_t *active = eeprom_find_active_page(ee_ctrl, 1);
    eeprom_page_to_regs(ee_ctrl, active);
}

/* ================================================================
 * eeprom_save_byte
 *
 * Saves a single register change to the active EEPROM page.
 * If the halfword at the register's position is erased (0xFFFF),
 * writes the new value directly. Otherwise triggers page swap.
 * ================================================================ */
void __attribute__((noinline, section(".text.keep"))) eeprom_save_byte(uint8_t *ee_ctrl, uint8_t reg_addr)
{
    uint16_t *active = eeprom_find_active_page(ee_ctrl, 0);
    uint32_t idx = reg_addr / 2;
    uint8_t *sr = servo_regs_arr;
    uint16_t reg_val = *(uint16_t *)(sr + idx * 2);
    uint16_t *slot = active + 1 + idx;

    if (*slot == 0xFFFF) {
        /* Slot is erased — write directly */
        flash_write_halfword(slot, reg_val);
    } else if (*slot != reg_val) {
        /* Slot has old value — need page swap */
        eeprom_regs_to_page(sr, active);
    }
}

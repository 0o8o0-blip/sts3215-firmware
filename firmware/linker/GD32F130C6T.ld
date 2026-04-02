/*
 * Linker script for GD32F130C6T6 — Feetech STS3215 servo
 *
 * Flash layout:
 *   0x08000000 - 0x080041AF  Application firmware (~16KB)
 *   0x08007400 - 0x080077FF  EEPROM emulation pages (1KB each)
 *   0x08007800 - 0x08007FFF  Bootloader (2KB)
 *
 * Application firmware occupies 0x08000000 to ~0x08004200.
 * The rest of flash is reserved for EEPROM and bootloader.
 */

ENTRY(Reset_Handler)

_estack = 0x20001000;    /* Top of 4KB SRAM */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS
{
    /* Vector table — must be at flash base */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Program code — Reset_Handler must be first (at 0xCC for bootloader) */
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.text.Reset_Handler))
        KEEP(*(.text.keep*))
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* Read-only data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* Initialized data — loaded from flash, copied to RAM at startup */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    /* Uninitialized data — zeroed at startup */
    . = ALIGN(4);
    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* Stack/heap check */
    ._user_heap_stack :
    {
        . = ALIGN(4);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + 0x100;   /* Min heap */
        . = . + 0x400;   /* Min stack */
        . = ALIGN(4);
    } >RAM

    .ARM.attributes 0 : { *(.ARM.attributes) }

    /* Bootloader (0x08007800-0x08007FFF) is NOT included in this build.
     * UART flashing preserves the existing bootloader on the servo.
     * For SWD flashing, use feetec_firmware_aux/bootloader/ separately. */
}

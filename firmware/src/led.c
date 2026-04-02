/* Feetech STS3215 — LED/GPIO subsystem */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t encoder_ctrl_arr[]; extern uint8_t * volatile encoder_ctrl;
extern uint8_t gpio_ctrl_arr[]; extern uint8_t * volatile gpio_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;

/* Vtable arrays (defined in vtables.c) */
extern const void *gpio_vtable[];
extern const void *led_obj_vtable[];

/* Functions from other files */
extern void subsys_memset(uint8_t *dst, uint8_t val, int count);
extern void encoder_set_initial_pos(int32_t *param);
extern void gpio_config_pin(uint8_t *cfg);


/* Set first byte to 5. */
void __attribute__((noinline)) subsys_init_5(uint8_t *param)
{
    *param = 5;
}

/* Call vtable function at offset 8. */
void __attribute__((noinline)) gpio_helper(int32_t **param)
{
    typedef void (*fn_t)(void);
    fn_t fn = (fn_t)(*(uint32_t *)((uint32_t)(*param) + 8));
    fn();
}

/* Dispatch to vtable[2] on the given object. */
void __attribute__((noinline)) led_vtable_dispatch(uint32_t *param)
{
    typedef void (*fn_t)(void);
    fn_t fn = (fn_t)(*(uint32_t *)((uint32_t)(*param) + 8));
    fn();
}

/* Initialize a GPIO/LED toggle structure with vtable. */
void __attribute__((noinline)) led_obj_init(uint32_t *param)
{
    extern const void *led_obj_vtable[];
    *param = (uint32_t)led_obj_vtable;
    *(uint8_t *)(param + 1) = 0;
    *(uint8_t *)((uint8_t *)param + 5) = 0;
}

/* LED tick — increment counter; every 100 ticks, toggle LED via vtable. */
void __attribute__((noinline)) led_tick(uint32_t *param)
{
    uint8_t *counter = (uint8_t *)param + 5;
    if (*counter > 99) {
        *counter = 0;
        uint8_t *state = (uint8_t *)(param + 1);
        typedef void (*fn_t)(void);
        if (*state == 0) {
            *state = 1;
            fn_t fn = (fn_t)(*(uint32_t *)((uint32_t)(*param) + 8));
            fn();
            __asm__ volatile ("nop" ::: "memory");
            return;
        }
        *state = 0;
        fn_t fn = (fn_t)(*(uint32_t *)((uint32_t)(*param) + 4));
        fn();
        __asm__ volatile ("" ::: "memory");
        return;
    }
    (*counter)++;
}

/* Initialize LED structure and clear vtable. */
void __attribute__((noinline, section(".text.keep"))) vtable_setup(void)
{
    uint32_t *p = (uint32_t *)gpio_ctrl_arr;
    led_obj_init(p);
    *p = 0;  /* vtable will be set by caller */
}

/* Enable GPIOF clock and call LED pin configuration. */
void __attribute__((noinline)) gpio_led_init(void)
{
    REG(GPIOF_BASE, 0x00) |= 1;   /* GPIOF_CTL bit 0 */
    gpio_helper((int32_t **)gpio_ctrl_arr);
}

/* Zero several sub-fields of a structure. */
int __attribute__((noinline, section(".text.keep"))) zero_init_helper(uint8_t *param)
{
    subsys_memset(param + 0x17, 0, 4);
    subsys_memset(param + 0x1B, 0, 4);
    subsys_memset(param + 0x1F, 0, 2);
    return (int)(uint32_t)param;
}

/* ================================================================
 * LED blink control — blinks LED based on alarm conditions.
 * If no alarm config, keeps LED on. Otherwise checks each alarm
 * bit against active errors.
 * ================================================================ */
void __attribute__((noinline)) led_blink_tick(uint8_t *param)
{
    uint32_t *gp = (uint32_t *)param;
    volatile uint8_t *sr;

    sr = servo_regs;
    if (sr[SR_ALARM_CONFIG] == 0) {
        /* No alarm config: keep LED on via vtable[2] */
        typedef void (*fn_t)(void);
        fn_t fn = (fn_t)(*(uint32_t *)((uint32_t)(*gp) + 8));
        fn();
        return;
    }

    uint8_t blink = 0;

    /* Check voltage error bit (bit 0) */
    sr = servo_regs;
    if ((sr[SR_ALARM_CONFIG] & 1) != 0) {
        sr = servo_regs;
        blink = sr[SR_ERROR_FLAGS] & 1;
    }

    /* Check overtemp bit (bit 2) */
    sr = servo_regs;
    if ((sr[SR_ALARM_CONFIG] & 4) != 0) {
        sr = servo_regs;
        if ((sr[SR_ERROR_FLAGS] & 4) != 0) {
            blink = 1;
        }
    }

    /* Check bit 5 (extended error) */
    sr = servo_regs;
    if ((sr[SR_ALARM_CONFIG] & ALARM_EXTENDED_ERR) != 0) {
        sr = servo_regs;
        if ((sr[SR_ERROR_FLAGS] & ALARM_EXTENDED_ERR) != 0) {
            blink = 1;
        }
    }

    /* Check I2C error bit (bit 1) + overload bit (bit 3) interaction */
    sr = servo_regs;
    if ((sr[SR_ALARM_CONFIG] & 2) == 0 || (sr[SR_ERROR_FLAGS] & 2) == 0) {
        sr = servo_regs;
        if ((sr[SR_ALARM_CONFIG] & 8) != 0) {
            goto check_overload;
        }
    } else {
        /* I2C error is configured AND active */
        if ((sr[SR_ALARM_CONFIG] & 8) == 0) {
            /* No overload check configured: blink for I2C error */
            goto do_blink;
        }
        blink = 1;
check_overload:
        sr = servo_regs;
        if ((sr[SR_ERROR_FLAGS] & 8) != 0) {
            goto do_blink;
        }
    }

    if (blink == 0) {
        /* No alarm: LED off via vtable[1] */
        typedef void (*fn_t)(void);
        fn_t fn = (fn_t)(*(uint32_t *)((uint32_t)(*gp) + 4));
        fn();
        return;
    }

do_blink:
    /* Active alarm: blink LED */
    led_tick(gp);
}

/* Turn LED OFF by clearing GPIOF bit 0 via BC register. */
void __attribute__((noinline)) gpio_led_alarm_set(void)
{
    *(volatile uint16_t *)(GPIOF_BASE + 0x28) = 1;  /* GPIOF BC: clear bit 0 */
}

/* Turn LED ON by setting GPIOF bit 0 via BOP register. */
void __attribute__((noinline)) gpio_led_state_set(void)
{
    *(volatile uint32_t *)(GPIOF_BASE + 0x18) = 1;  /* GPIOF BOP: set bit 0 */
}

/* Enable LED output and reset encoder position. */
void __attribute__((noinline)) gpio_led_enable(void)
{
    volatile uint32_t *gpiof = (volatile uint32_t *)GPIOF_BASE;
    gpiof[5] |= 1;  /* GPIOF OCTL: set output bit 0 (PF0 = LED) */
    encoder_set_initial_pos((int32_t *)encoder_ctrl_arr);
}


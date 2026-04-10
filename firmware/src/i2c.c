/* Feetech STS3215 — I2C subsystem */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;
extern uint8_t i2c_ctrl_arr[]; extern uint8_t * volatile i2c_ctrl;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t timer_ctrl_arr[]; extern uint8_t * volatile timer_ctrl;

/* Vtable arrays (defined in vtables.c) */
extern const void *i2c_obj_vtable[];

/* Functions from other files */
extern void pwm_apply_output(uint8_t *param, int32_t duty, int32_t brake_state, uint32_t extra);
extern void usart1_baud_init(uint8_t *param);
extern void subsys_memset(uint8_t *dst, uint8_t val, int count);
extern void subsys_init_5(uint8_t *param);
extern void servo_regs_load_eeprom(uint8_t *param);
extern void eeprom_page_load_defaults(uint8_t *param);
extern void gpio_config_pin(uint8_t *cfg);

/* Forward declarations within this file */
void i2c_send_addr_write(uint8_t *param);
void i2c_send_addr_read(uint8_t *param);
void i2c_master_reset(uint8_t *param);
void i2c_start_reinit(uint8_t *param);


/* Set I2C mode field to 0. */
void __attribute__((noinline)) i2c_set_mode(uint32_t *param)
{
    *param = 0;
}

/* Initialize I2C control object with vtable and zero buffers. */
uint32_t * __attribute__((noinline)) i2c_obj_init(uint32_t *param)
{
    extern const void *i2c_obj_vtable[];
    *param = (uint32_t)i2c_obj_vtable;
    subsys_memset((uint8_t *)(param + 5), 0, 8);
    subsys_memset((uint8_t *)(param + 7), 1, 8);
    return param;
}

/* ================================================================
 * I2C event handler — state machine for I2C master read sequence.
 * Handles address phase, data reception, and NACK/stop conditions.
 * ================================================================ */
void __attribute__((noinline)) i2c_event_handler(uint8_t *param)
{
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;

    switch (*param) {
    case 1:
        if ((i2c[5] & 1) == 0) {
            *param = 7;
            i2c[1] &= ~0x200U;
            i2c[1] &= ~0x100U;
            return;
        }
        /* Send slave address in WRITE mode */
        i2c_send_addr_write(param);
        *param = 4;
        return;

    case 2:
        if ((i2c[5] & 1) == 0) {
            *param = 7;
            i2c[1] &= ~0x200U;
            i2c[1] &= ~0x100U;
            return;
        }
        /* Send slave address in READ mode */
        i2c_send_addr_read(param);
        *param = 5;
        return;

    case 3:
        if ((i2c[5] & 4) == 0) {
            *param = 7;
            i2c[1] &= ~0x200U;
            i2c[1] &= ~0x100U;
            return;
        }
        /* Clear BTC: read STAT0 then DATA (original at 0x182A-0x182C) */
        (void)i2c[5];  /* read STAT0 */
        (void)i2c[4];  /* read DATA — clears BTC */
        i2c[0] |= 0x800;   /* Set POS */
        i2c[0] |= 0x100;   /* Generate repeated START */
        *param = 2;
        return;

    case 4:
        if ((i2c[5] & 2) == 0) {
            *param = 7;
            i2c[1] &= ~0x200U;
            i2c[1] &= ~0x100U;
            return;
        }
        /* Clear ADDR flag by reading STAT0 then STAT1 (required by I2C peripheral).
         * Original at 0x17F6: ldr [r3,#20]; ldr [r3,#24] before writing DATA. */
        (void)i2c[5];  /* read STAT0 */
        (void)i2c[6];  /* read STAT1 — clears ADDR */
        /* Write data byte (register address for read) */
        i2c[4] = (uint32_t)(uint8_t)param[5];
        *param = 3;
        return;

    case 5: {
        uint32_t stat0 = i2c[5];
        if ((stat0 & 2) == 0) {
            /* ADDR not set — check for BTC (transfer complete) */
            stat0 = i2c[5];  /* re-read STAT0 for BTC check */
            if ((stat0 & 4) != 0) {
                /* BTC set — read 2 received data bytes + generate STOP */
                i2c[0] |= 0x200;   /* STOP */
                uint8_t idx = param[4];
                param[idx + 1] = (uint8_t)i2c[4];
                uint32_t next = idx + 1;
                param[4] = (uint8_t)next;
                param[(next & 0xFF) + 1] = (uint8_t)i2c[4];
                param[4] = idx + 2;
                *param = 6;
                i2c[1] &= ~0x200U;
                i2c[1] &= ~0x100U;
                return;
            }
            *param = 7;
            return;
        }
        /* ADDR is set (slave ACKed read address) */
        if (param[4] == 0) {
            /* First entry: disable ACK (NACK on last byte) */
            i2c[0] &= ~0x400U;
            /* Clear ADDR flag: read STAT0 then STAT1 (required by I2C peripheral).
             * Original at 0x18A2: ldr [r3,#20]; ldr [r3,#24] */
            (void)i2c[5];  /* read STAT0 */
            (void)i2c[6];  /* read STAT1 — clears ADDR */
            return;
        }
        break;
    }

    default:
        /* Match factory: set error state, disable I2C interrupts */
        *param = 7;
        i2c[1] &= ~0x200U;
        i2c[1] &= ~0x100U;
        return;
    }
}

/* ================================================================
 * I2C + GPIO + DMA hardware init — configures I2C0 GPIO pins,
 * DMA channel 0 for I2C data transfer, and I2C timing registers.
 * ================================================================ */
void __attribute__((noinline)) i2c_hw_init(uint8_t *param)
{
    /* Cache peripheral base in a register */
    volatile uint32_t * volatile periph = (volatile uint32_t *)0x40012400U;

    /* Enable ADC clock (RCU APB2EN bit 9 = ADCEN) */
    *(volatile uint32_t *)(0x40021000 + 0x18) |= 0x200;

    uint32_t pin_idx = 0;
    int i = 0;
    uint32_t bit_pos = 1;
    char *pcfg = (char *)(param + I2C_ADC_CFG);

    do {
        if (*pcfg != '\0') {
            /* Set GPIOA CTL: two bits per pin (mode=AF)
             * Original sets lower mode bit first, then upper, as
             * two separate read-modify-write operations. */
            *(volatile uint32_t *)GPIOA_BASE |= (1U << ((i * 2) & 0xFFU));
            *(volatile uint32_t *)GPIOA_BASE |= (1U << (bit_pos & 0xFF));

            /* Compute ADC regular sequence register index.
             * Original builds address as: periph_base + idx*4, then
             * reads/writes at [reg, #0x2C] = ADC_RSQ registers.
             * periph_base = 0x40012400 (ADC), so addresses are:
             *   0x4001242C (RSQ2), 0x40012430 (RSQ1), 0x40012434 (RSQ0) */
            uint32_t prod5 = pin_idx * 5;
            int32_t div16 = (int32_t)(((int64_t)(int32_t)0x88888889 * (int64_t)(int32_t)prod5) >> 32);
            int32_t af_reg_idx = 2 - ((div16 + (int32_t)prod5) >> 4);
            uint32_t af_reg_off = 0x2C + (uint32_t)(af_reg_idx * 4);

            /* Compute bit position within the sequence register */
            uint32_t mod6 = pin_idx - (uint32_t)(((uint64_t)0xAAAAAAABULL * (uint64_t)pin_idx) >> 34) * 6;
            uint32_t shift = (mod6 & 0xFF) * 5;

            /* Write to ADC register space (periph base), not GPIOA */
            *((volatile uint32_t *)((uint32_t)periph + af_reg_off)) |=
                (uint32_t)i << (shift & 0xFF);

            /* Set ADC sample time register (SAMPT1, offset 0x10)
             * Original uses [r6, #7] i.e. pcfg[7] for sample time byte */
            periph[0x10/4] |=
                (uint32_t)(uint8_t)pcfg[7] << ((i * 3) & 0xFFU);

            pin_idx = (pin_idx + 1) & 0xFF;
        }
        i++;
        bit_pos += 2;
        pcfg++;
    } while (i != 8);

    /* Configure peripheral transfer count */
    periph[0x2C/4] |= (pin_idx - 1) * 0x100000;

    /* Configure ADC for software-triggered single conversions.
     * The TIMER0_BRK_UP_TRG_COM_IRQHandler in main.c uses SWRCST to
     * sample channels at precise times. No DMA, no scan, no external
     * trigger.
     *
     * CTL1: ETSRC=111 (software trigger required for SWRCST)
     *       ETERC=0 (don't enable external trigger)
     *       DMA=0  (no DMA, ISR reads RDATA directly)
     *       ADON=1 (power on) */
    periph[0x08/4] |= (0x07U << 17) | (1U << 20); /* CTL1: ETSRC=111 (software), ETERC=1 */
    periph[0x08/4] |= 1;                    /* ADON */
    periph[0x08/4] |= 8;                    /* RSTCLB */

    while ((periph[0x08/4] & 8) != 0) { }

    /* Calibration */
    periph[0x08/4] |= 4;
    while ((periph[0x08/4] & 4) != 0) { }

    /* Pre-load DMA buffer slots so existing code paths see something
     * sensible until the ISR has run a few times. */
    *(volatile uint16_t *)(param + 4 + 0) = 0;  /* slot 0: temp */
    *(volatile uint16_t *)(param + 4 + 2) = 0;  /* slot 1: current */
    *(volatile uint16_t *)(param + 4 + 4) = 0;  /* slot 2: voltage */
}

/* I2C0 event IRQ handler. */
void I2C0_EV_IRQHandler(void)
{
    i2c_event_handler(encoder_i2c_arr);
}


/* ================================================================
 * Configure I2C0 to start a read transaction for the encoder.
 * State machine: if idle (state==0), set up read; else retry count.
 * ================================================================ */
void __attribute__((noinline)) i2c_configure(uint8_t *param)
{
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;

    if (param[EI2C_STATE] == 0) {
        param[EI2C_STATE] = 1;
        param[EI2C_SLAVE_ADDR] = 2;   /* num bytes to read */
        param[EI2C_WRITE_DATA] = 0;   /* byte counter */
        param[EI2C_REG_ADDR] = 0x0C; /* register address (raw angle) */
        param[6] = 0x36; /* I2C slave address */
        /* Configure I2C0 for read */
        i2c[0] |= 0x400;        /* Enable ACK */
        i2c[0] &= ~0x800U;      /* Clear POS */
        i2c[1] |= 0x200;        /* Set ERRIE */
        i2c[1] |= 0x100;        /* Set EVIE */
        i2c[0] |= 0x100;        /* Generate START */
    } else {
        uint8_t retry = param[EI2C_DATA_IDX];
        param[EI2C_DATA_IDX] = retry + 1;
        if ((uint8_t)(retry + 1) > 10) {
            param[EI2C_STATE] = 7;  /* error state */
        }
    }
}

/* ================================================================
 * Return I2C state machine state.
 * If state==6 (complete): resets to 0 and clears retry counter.
 * If state==7 (error): resets and re-inits I2C.
 * ================================================================ */
uint8_t __attribute__((noinline)) i2c_get_state(uint8_t *param)
{
    uint8_t state = param[EI2C_STATE];
    if (state == 6) {
        param[EI2C_STATE] = 0;
        param[EI2C_DATA_IDX] = 0;
    } else if (state == 7) {
        param[EI2C_STATE] = 0;
        i2c_master_reset(param);
        i2c_start_reinit(param);
        param[EI2C_DATA_IDX] = 0;
    }
    return state;
}

/* Read 2 bytes from I2C buffer as 16-bit value (high:low). */
uint16_t __attribute__((noinline)) i2c_read_data(uint8_t *param)
{
    return ((uint16_t)param[1] << 8) | (uint16_t)param[2];
}

/* Software-driven "data ready" flag set by the TIMER0 ISR each time it
 * completes a full sweep (current + voltage + temperature). The main_tick
 * checks this and processes the latest readings. */
extern volatile uint8_t adc_isr_data_ready;

uint32_t __attribute__((noinline)) i2c_data_ready(uint8_t *ctrl)
{
    (void)ctrl;
    if (adc_isr_data_ready) {
        adc_isr_data_ready = 0;
        return 1;
    }
    return 0;
}

/* ================================================================
 * Stop motor on I2C encoder error.
 * If I2C error is enabled in alarm shutdown and active in error flags,
 * zeroes PWM output and calls pwm_apply_output.
 * ================================================================ */
void __attribute__((noinline)) i2c_error_stop(void)
{
    volatile uint8_t *sr = servo_regs;
    if (((sr[SR_ALARM_SHUTDOWN] & 2) != 0) && ((sr[SR_ERROR_FLAGS] & 2) != 0)) {
        volatile uint8_t *pc = pwm_ctrl;
        *(uint16_t *)(pc + PWM_OUTPUT) = 0;
        sr = servo_regs;
        pwm_apply_output(timer_ctrl_arr, 0, (int32_t)sr[SR_TORQUE_ENABLE], 0);
    }
}

/* Initialize i2c_ctrl_arr — runs as .init_array constructor before main(),
 * matching original binary constructor [7]. */
void __attribute__((constructor, noinline)) i2c_ctrl_init(void)
{
    uint32_t *p = (uint32_t *)i2c_ctrl_arr;
    i2c_obj_init(p);
    extern const void *i2c_init_vtable[];
    *p = (uint32_t)i2c_init_vtable;
    subsys_init_5((uint8_t *)(p + 9));
}

/* ================================================================
 * Check I2C0 STAT register for error flags (BERR, LOSTARB, AERR,
 * OUERR — bits 8-11). If any set: clears them, sets error state,
 * and disables interrupts.
 * ================================================================ */
void __attribute__((noinline, section(".text.keep"))) i2c_error_check(uint8_t *param)
{
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;

    if ((i2c[5] & 0xF00) == 0) {
        /* No error flags visible — but we're in the ER handler, so an error
         * DID occur. The flags may have been cleared by a prior STAT0 read.
         * Must still set error state and disable interrupts to prevent
         * infinite re-entry (the main loop will reinit I2C via i2c_get_state). */
        *param = 7;
        i2c[1] &= ~0x200U;
        i2c[1] &= ~0x100U;
        return;
    }

    /* Clear error and event flags */
    i2c[5] &= ~0xDF00U;

    /* Set state to error */
    *param = 7;

    /* Disable event and error interrupts in I2C peripheral */
    i2c[1] &= ~0x200U;
    i2c[1] &= ~0x100U;
}

/* I2C0 error interrupt handler.
 * Original firmware routes BOTH event and error interrupts through the
 * state machine (i2c_event_handler), NOT a separate error-check function.
 * The I2C0_EV vector is Default_Handler in the original (EV interrupt is
 * never enabled in NVIC). Only ERRIE triggers IRQ 23, and the state machine
 * handles both normal state transitions and error conditions. */
void I2C0_ER_IRQHandler(void)
{
    /* Match factory: route through event handler state machine.
     * Factory I2C0_EV vector is Default_Handler (unused); only ER is active.
     * The state machine handles both I2C events and error recovery. */
    i2c_event_handler(encoder_i2c_arr);
}

/* Reset encoder I2C state: clears state byte and data index. */
void __attribute__((noinline, section(".text.keep"))) i2c_state_reset(void)
{
    encoder_i2c_arr[EI2C_STATE] = 0;
    encoder_i2c_arr[EI2C_DATA_IDX] = 0;
}

/* Reset I2C0 by toggling peripheral enable, then re-init timing. */
void __attribute__((noinline)) i2c_master_reset(uint8_t *param)
{
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;

    /* Toggle I2C peripheral reset */
    *i2c |= 0x8000;      /* Set SWRST (bit 15) */
    *i2c &= ~0x8000U;    /* Clear SWRST */

    /* Re-initialize I2C with stored parameters */
    uint32_t speed = *(uint32_t *)(param + 8);
    (void)speed;

    /* Re-configure I2C: CTL1 frequency, CKCFG clock, RT rise time, enable */
    i2c[1] = 0x30;        /* CTL1: frequency = 48MHz / 1MHz = 0x30 */
    i2c[7] = 0xF0;        /* CKCFG: 100kHz I2C clock */
    i2c[8] = 0x30;        /* RT: rise time for 100kHz standard mode */
    *i2c |= 1;            /* CTL0: I2CEN */
}

/* I2C bus recovery — original at 0x080016B8.
 * Bit-bangs GPIO to clear stuck I2C bus, then re-enables I2C pins.
 * Called from i2c_get_state on error (state=7) to recover the bus
 * WITHOUT restarting the transaction (unlike i2c_configure). */
void __attribute__((noinline)) i2c_start_reinit(uint8_t *param)
{
    (void)param;
    volatile uint32_t *gpioa = (volatile uint32_t *)GPIOA_BASE;

    /* Toggle SCL/SDA pins to GPIO output mode, pulse them,
     * then restore to I2C AF mode. Original does a timed
     * bit-bang sequence with delay loops. */
    gpioa[0] = (gpioa[0] & ~0x80000U) | 0x40000U;  /* PA9 output */
    gpioa[0] = (gpioa[0] & ~0x200000U) | 0x100000U; /* PA10 output */

    /* Toggle clock 9 times to clear stuck slave */
    *(volatile uint16_t *)(GPIOA_BASE + 0x28) = 0x200;   /* SDA low */
    *(volatile uint16_t *)(GPIOA_BASE + 0x28) = 0x400;   /* SCL low */

    /* Brief delay (original loops 0x78 iterations) */
    for (volatile int i = 0; i < 120; i++) {}

    *(volatile uint32_t *)(GPIOA_BASE + 0x18) = 0x200;   /* SDA high */

    for (volatile int i = 0; i < 120; i++) {}

    /* Restore AF mode for I2C pins.
     * Original does separate set/clear for each pin (two RMW per pin).
     * PA10: set bit 21 (AF upper), then clear bit 20 (output lower) → [21:20]=10=AF
     * PA9:  set bit 19 (AF upper), then clear bit 18 (output lower) → [19:18]=10=AF */
    gpioa[0] |= 0x200000U;           /* PA10: set bit 21 */
    gpioa[0] &= ~0x100000U;          /* PA10: clear bit 20 → AF mode */
    gpioa[0] |= 0x80000U;            /* PA9: set bit 19 */
    gpioa[0] &= ~0x40000U;           /* PA9: clear bit 18 → AF mode */
}

/* Chain: servo_regs_load_eeprom -> eeprom_page_load_defaults -> usart1_baud_init */
void __attribute__((noinline)) pwm_composite_init(uint8_t *param)
{
    servo_regs_load_eeprom(param);
    eeprom_page_load_defaults(param);
    usart1_baud_init(param);
}

/* ================================================================
 * Configure I2C control state based on servo_regs config byte.
 * Bit 5 (0x20) selects between I2C encoder mode and ADC mode.
 * Then calls i2c_hw_init and sets initial values.
 * ================================================================ */
void __attribute__((noinline)) i2c_mode_init(uint8_t *param)
{
    if (servo_regs_arr[SR_CONFIG] & 0x20) {
        /* I2C encoder mode */
        param[I2C_ADC_PA3] = 1;
        param[I2C_ADC_PA5] = 1;
        param[I2C_ADC_PA4] = 1;
        param[I2C_TEMP_CH] = 2;
        param[I2C_CURR_CH] = 1;
        param[I2C_VOLT_CH] = 0;
    } else {
        /* ADC mode: PA1=temp, PA3=current, PA4=voltage */
        param[I2C_ADC_PA4] = 1;
        param[I2C_ADC_PA1] = 1;
        param[I2C_ADC_PA3] = 1;
        param[I2C_TEMP_CH] = 0;
        param[I2C_CURR_CH] = 1;
        param[I2C_VOLT_CH] = 2;
    }
    i2c_hw_init(param);
    *(uint16_t *)(param + I2C_CURR_BASELINE) = 0;
    param[I2C_WARMUP_CTR] = 0x10;
}

/* Send I2C slave address in WRITE mode (bit 0 = 0). */
void __attribute__((noinline)) i2c_send_addr_write(uint8_t *param)
{
    uint8_t addr = param[6];
    uint8_t val = (uint8_t)(addr << 1);
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;
    i2c[4] = (uint32_t)val;
}

/* Send I2C slave address in READ mode (bit 0 = 1). */
void __attribute__((noinline)) i2c_send_addr_read(uint8_t *param)
{
    uint8_t addr = param[6];
    uint8_t val = (uint8_t)((addr << 1) | 1);
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;
    i2c[4] = (uint32_t)val;
}


/* Feetech STS3215 — Encoder subsystem (AS5600 I2C magnetic encoder) */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t encoder_ctrl_arr[]; extern uint8_t * volatile encoder_ctrl;
extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;
extern uint8_t pid_state_arr[]; extern uint8_t * volatile pid_state;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;

/* Vtable arrays (defined in vtables.c) */
extern const void *encoder_vtable[];

/* Functions from other files */
extern uint16_t i2c_read_data(uint8_t *param);
extern uint8_t i2c_get_state(uint8_t *param);
extern void gpio_config_pin(uint8_t *cfg);
extern void i2c_configure(uint8_t *param);
extern void i2c_set_mode(uint32_t *param);
extern void timer0_set_pwm(uint8_t *state, uint32_t forward, uint32_t reverse);
extern void position_offset_set(uint8_t *param_unused, int16_t position);

/* Forward declarations within this file */
int32_t encoder_multiturn_track(int32_t *param);
int32_t encoder_iir_filter(int32_t *accum, int32_t new_val, int32_t alpha);
uint32_t encoder_set_value(uint32_t *param, uint32_t val);


/* Initialize encoder state with vtable pointer and zeros. */
void __attribute__((noinline)) encoder_vtable_init(uint32_t *param)
{
    extern const void *encoder_vtable[];
    *param = (uint32_t)encoder_vtable;
    param[3] = 0;
    param[2] = 0;
}

/* Store a value into the encoder accumulator. */
uint32_t __attribute__((noinline)) encoder_set_value(uint32_t *param, uint32_t val)
{
    *param = val;
    return val;
}

/* Encoder object constructor — initializes vtable and I2C mode. */
uint32_t * __attribute__((noinline)) encoder_obj_init(uint32_t *param)
{
    encoder_vtable_init(param);
    extern const void *encoder_vtable[];
    *param = (uint32_t)encoder_vtable;
    i2c_set_mode(param + 7);
    *(uint8_t *)(param + 6) = 0;
    return param;
}

/* ================================================================
 * Encoder subsystem init — configures I2C0 at 100kHz for AS5600.
 * ================================================================ */
void __attribute__((noinline)) encoder_init(void)
{
    /* I2C0 hardware initialization for AS5600 magnetic encoder.
     * Configures GPIO pins, I2C clock, timing, and NVIC interrupts.
     *
     * Original at 0x15B4: configures old-style I2C registers (CTL1, CKCFG, RT).
     * GD32F130 uses STM32F1-compatible I2C peripheral, NOT the new I2C_TIMING register.
     */
    volatile uint32_t *gpioa = (volatile uint32_t *)GPIOA_BASE;
    volatile uint32_t *rcu = (volatile uint32_t *)0x40021000;
    volatile uint32_t *i2c = (volatile uint32_t *)I2C0_BASE;

    /* Configure PA9 (I2C0 SCL) as AF4 open-drain */
    gpioa[0x24/4] |= 0x40;       /* AFSEL1: PA9 = AF4 (bits 7:4 = 0100) */
    gpioa[0x24/4] |= 0x400;      /* AFSEL1: PA10 = AF4 (bits 11:8 = 0100) */
    gpioa[0] |= 0x80000;         /* CTL: PA9 = AF mode (bits 19:18 = 10) */
    gpioa[1] |= 0x200;           /* OMODE: PA9 = open-drain */
    gpioa[2] |= 0x40000;         /* OSPD: PA9 = medium speed */
    gpioa[0] |= 0x200000;        /* CTL: PA10 = AF mode (bits 21:20 = 10) */
    gpioa[1] |= 0x400;           /* OMODE: PA10 = open-drain */
    gpioa[2] |= 0x100000;        /* OSPD: PA10 = medium speed */

    /* Enable I2C0 clock on APB1 (bit 21) */
    rcu[0x1C / 4] |= (1U << 21);

    /* Store I2C speed and mode in encoder_i2c struct (matches original) */
    *(uint32_t *)(encoder_i2c_arr + 8) = 100000;  /* I2C speed = 100kHz */
    encoder_i2c_arr[12] = 0;                       /* mode = standard */

    /* I2C clock configuration for 100kHz standard mode:
     * CTL1 FREQ = 48 (48MHz APB1 clock)
     * RT = 48 (rise time = 49 PCLK1 cycles ≈ 1us)
     * CKCFG = 240 (CLKC = 48MHz / (2 * 100kHz) = 240)
     */
    {
        uint32_t ctl1 = i2c[1];
        ctl1 = (ctl1 & ~0x3F) | 0x30;  /* FREQ = 48 */
        i2c[1] = ctl1;
    }
    i2c[8] = 0x30;                      /* RT: rise time for 100kHz standard mode */
    {
        /* Standard mode: CLKC = PCLK1 / (2 * SCL_freq) = 48000000 / 200000 = 240 */
        uint32_t ckcfg = i2c[7];        /* read CKCFG at offset 0x1C */
        ckcfg |= 240;                   /* CLKC = 240 = 0xF0 */
        i2c[7] = ckcfg;                 /* write CKCFG */
    }

    /* Configure NVIC for I2C0 error (IRQ 23) interrupt ONLY.
     * Factory firmware only enables IRQ 23 in NVIC — the EV vector (IRQ 22)
     * points to Default_Handler and is never enabled. On GD32F130, when both
     * EVIE+ERRIE are set in CTL1, I2C events trigger both EV and ER interrupt
     * lines. Enabling both NVIC IRQs causes the handler to fire twice per
     * event, corrupting the state machine (second call sees wrong state → error).
     * Only IRQ 23 (ER) is needed — it receives all I2C events and errors. */
    uint8_t nvic_er[4] = { 23, 2, 0, 1 };   /* IRQ 23 = I2C0_ER, priority 2 */
    gpio_config_pin(nvic_er);

    /* Enable I2C0 peripheral */
    i2c[0] |= 1;

    /* Initialize encoder I2C transaction state */
    encoder_i2c_arr[EI2C_STATE] = 0;    /* state = idle */
    encoder_i2c_arr[EI2C_SLAVE_ADDR] = 0x36; /* AS5600 I2C address */
    encoder_i2c_arr[EI2C_REG_ADDR] = 0x0C;  /* raw angle register */
}

/* Low-pass IIR filter: out = (prev * (10 - alpha) + new * alpha) / 10 */
int32_t __attribute__((noinline)) encoder_iir_filter(int32_t *accum, int32_t new_val, int32_t alpha)
{
    int32_t complement = 10 - alpha;
    int32_t product = new_val * alpha;
    int32_t result = product;
    result += (*accum) * complement;
    /* Divide by 10 using multiply-shift: (x * 0x66666667) >> 34 */
    result = (int32_t)(((int64_t)0x66666667 * (int64_t)result) >> 34) - (result >> 31);
    *accum = result;
    return result;
}

/* Read I2C encoder and handle operating modes. */
int32_t __attribute__((noinline)) encoder_read(int32_t *param)
{
    int32_t result;

    volatile uint8_t *sr = servo_regs;
    uint8_t mode = sr[SR_OPERATING_MODE];
    if (mode == 0 || mode == 3) {
        if ((uint8_t)param[6] == 0) {
            /* First read — use vtable read function directly */
            *(uint8_t *)(param + 6) = 1;
            typedef int32_t (*read_fn_t)(int32_t *, int);
            uint32_t vtable = param[0];
            read_fn_t fn = (read_fn_t)(*(uint32_t *)(vtable + 4));
            result = fn(param, 0);
            timer0_set_pwm(pid_state_arr, result, result);
        } else {
            /* Subsequent reads — use multiturn tracking + IIR filter */
            int32_t raw = encoder_multiturn_track(param);
            result = encoder_iir_filter(param + 7, raw, 8);
        }
    } else {
        /* Non-position modes: return last value, reset init flag */
        result = param[5];
        *(uint8_t *)(param + 6) = 0;
    }
    return result;
}

/* Read encoder via vtable, set initial position for multiturn. */
int32_t __attribute__((noinline)) encoder_set_initial(int32_t *param, int32_t revolutions)
{
    typedef int32_t (*read_fn_t)(void);
    uint32_t vtable = param[0];
    read_fn_t fn = (read_fn_t)(*(uint32_t *)(vtable + 8));
    int32_t raw = fn();
    param[3] = raw;
    param[4] = raw;
    param[2] = revolutions;
    param[1] = raw + revolutions * ENCODER_FULL_REV;
    encoder_set_value((uint32_t *)(((uint8_t *)param) + ENC_POSITION_VAL), param[1]);
    return param[1];
}

/* Track cumulative position across multiple revolutions. */
int32_t __attribute__((noinline)) encoder_multiturn_track(int32_t *param)
{
    typedef int32_t (*read_fn_t)(void);
    uint32_t vtable = param[0];
    read_fn_t fn = (read_fn_t)(*(uint32_t *)(vtable + 8));
    int32_t raw = fn();

    param[3] = raw;
    int32_t delta = raw - param[4];

    /* Wrap detection: threshold 4076 = 4096-20. Jump > 4076 counts means wrap. */
    if (delta > (int32_t)ENCODER_WRAP_THRESH) {
        param[2] = param[2] - 1;
    } else if (delta < (int32_t)-ENCODER_WRAP_THRESH) {
        param[2] = param[2] + 1;
    }

    param[4] = raw;
    param[1] = raw + param[2] * ENCODER_FULL_REV;
    return param[1];
}

/* Update multiturn position and store to encoder value register. */
void __attribute__((noinline)) encoder_update_position(int32_t *param)
{
    int32_t val = encoder_multiturn_track(param);
    encoder_set_value((uint32_t *)(((uint8_t *)param) + ENC_POSITION_VAL), val);
}

/* Read initial encoder and set the encoder value register. */
void __attribute__((noinline)) encoder_set_initial_pos(int32_t *param)
{
    encoder_set_initial(param, 0);
}

/* Start an I2C read of the AS5600 raw angle register. */
void __attribute__((noinline)) encoder_start_read(uint8_t *ctrl)
{
    (void)ctrl;
    i2c_configure(encoder_i2c_arr);
}

/* ================================================================
 * Check if I2C encoder read is complete.
 * If state==6 (done), reads data, validates, applies direction.
 * If state==7 (error), sets error flag. Returns 1 on success.
 * ================================================================ */
uint32_t __attribute__((noinline)) encoder_read_complete(uint8_t *ctrl)
{
    uint8_t state = i2c_get_state(encoder_i2c_arr);

    if (state != 6) {
        if (state == 7) {
            volatile uint8_t *sr = servo_regs;
            sr[SR_ERROR_FLAGS] |= 2;  /* set I2C error flag */
        }
        return 0;
    }

    int32_t val = (int32_t)i2c_read_data(encoder_i2c_arr);

    if (val > ENCODER_MAX_VAL) {
        volatile uint8_t *sr = servo_regs;
        sr[SR_ERROR_FLAGS] |= 2;  /* invalid data */
        return 0;
    }

    /* Apply direction reversal if config bit 7 is set */
    {
        volatile uint8_t *sr = servo_regs;
        if ((sr[SR_CONFIG] & 0x80) != 0) {
            val = ENCODER_MAX_VAL - val;
        }
    }

    /* Store raw encoder value */
    *(int32_t *)(ctrl + ENC_RAW_ANGLE) = val;
    {
        volatile uint8_t *sr = servo_regs;
        sr[SR_ERROR_FLAGS] &= ~2U;  /* clear I2C error flag */
    }
    return 1;
}

/* Initialize encoder_ctrl_arr with encoder_obj_init. */
void __attribute__((noinline, section(".text.keep"))) encoder_ctrl_init(void)
{
    encoder_obj_init((uint32_t *)encoder_ctrl_arr);
}

/* Reset encoder to position 0, copy to pwm_ctrl, clear torque enable. */
void __attribute__((noinline)) encoder_home_reset(void)
{
    encoder_set_initial((int32_t *)encoder_ctrl_arr, 0);
    volatile uint8_t *ec = encoder_ctrl;
    volatile uint8_t *pc = pwm_ctrl;
    *(uint32_t *)(pc + PWM_ENCODER_POS) = *(uint32_t *)(ec + ENC_ACCUM_POS);
    volatile uint8_t *sr = servo_regs;
    sr[SR_TORQUE_ENABLE] = 0;
}

/* Read raw encoder angle from stored value (original at 0x080002EC).
 * Called via encoder_vtable[2] from encoder_set_initial.
 * Reads the value stored by encoder_read_complete at ctrl+0x14,
 * subtracts reference at ctrl+0x20, wraps to ±4096 range.
 * NOTE: called with ctrl pointer in r0 (ARM calling convention). */
int32_t __attribute__((noinline)) encoder_get_raw_angle(int32_t *ctrl)
{
    int32_t raw = *(int32_t *)((uint8_t *)ctrl + ENC_RAW_ANGLE);
    int32_t ref = *(int32_t *)((uint8_t *)ctrl + ENC_DELINEAR);
    int32_t delta = raw - ref;
    if (delta >= ENCODER_FULL_REV) {
        delta -= ENCODER_FULL_REV;
    } else if (delta < 0) {
        delta += ENCODER_FULL_REV;
    }
    return delta;
}

/* Call i2c_configure with encoder_i2c_arr. */
void __attribute__((noinline, section(".text.keep"))) encoder_init_wrapper(void)
{
    i2c_configure(encoder_i2c_arr);
}


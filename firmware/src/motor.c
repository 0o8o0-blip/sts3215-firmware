/* Feetech STS3215 — Motor control and PWM output */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t motor_ctrl_arr[]; extern uint8_t * volatile motor_ctrl;
extern uint8_t pid_state_arr[]; extern uint8_t * volatile pid_state;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t timer_ctrl_arr[]; extern uint8_t * volatile timer_ctrl;

/* Vtable arrays (defined in vtables.c) */
extern const void *pwm_ctrl_vtable[];

/* Functions from other files */
extern void gpio_config_pin(uint8_t *cfg);
extern void i2c_error_stop(void);
extern void pid_compute(uint8_t *state);
extern void timer0_set_duty(uint8_t *ctrl, int32_t duty, int32_t is_brake);
extern void timer0_set_pwm(uint8_t *state, uint32_t forward, uint32_t reverse);
extern int zero_init_helper(uint8_t *param);
extern void ring_buf_init(uint8_t *ring);
extern void servo_apply_config(uint8_t *param, int32_t addr);
extern void position_profile_init(uint8_t *s, int32_t speed, int32_t accel);
extern void timer0_hw_init(uint8_t *ctrl);

extern uint8_t adc_state_arr[]; extern uint8_t * volatile adc_state;
extern uint8_t encoder_ctrl_arr[]; extern uint8_t * volatile encoder_ctrl;
extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;

/* Forward declarations within this file */
void pwm_ctrl_hw_init(uint32_t *param);
void timer15_tick_handler(uint8_t *param);
void pwm_apply_output(uint8_t *param, int32_t duty, int32_t brake_state, uint32_t extra);
extern void uart_tx_start_if_ready(uint8_t *s);
extern void usart1_dma_init(uint8_t *uart_st, uint32_t baud_idx);


/* Initialize PWM control structure with vtable and defaults. */
uint32_t * __attribute__((noinline)) pwm_ctrl_init(uint32_t *param)
{
    pwm_ctrl_hw_init(param);
    extern const void *pwm_ctrl_vtable[];
    *param = (uint32_t)pwm_ctrl_vtable;  /* vtable — original loads from literal pool */
    *(uint8_t *)(param + 2) = 0xFF;
    return param;
}

/* Set default period and mode flags. */
void __attribute__((noinline)) pwm_ctrl_hw_init(uint32_t *param)
{
    extern const void *pwm_ctrl_vtable[];
    *param = (uint32_t)pwm_ctrl_vtable;  /* vtable — original loads from literal pool */
    *(uint16_t *)(param + 1) = 1000;  /* default period */
    *(uint8_t *)((uint8_t *)param + 6) = 0;
    *(uint8_t *)((uint8_t *)param + 7) = 0;
}

/* TIMER15 IRQ handler. */
void TIMER15_IRQHandler(void)
{
    timer15_tick_handler(motor_ctrl_arr);
}

/* TIMER15 tick handler — if TX pending, trigger UART TX start. */
void __attribute__((noinline)) timer15_tick_handler(uint8_t *param)
{
    if (param[UART_TX_ACTIVE] == 1) {
        param[UART_ERROR] = 0;
        param[UART_TX_ACTIVE] = 0;
        uart_tx_start_if_ready(param);
    }
    /* Clear TIMER15 update interrupt flag */
    REG16(TIMER15_BASE, 0x10) = 0;
}

/* Initialize TIMER15 for motor control tick and configure NVIC (IRQ 21). */
void __attribute__((noinline)) motor_obj_init(uint8_t *param)
{
    (void)param;

    /* Enable TIMER15 clock: RCU_APB2EN |= TIMER15EN */
    volatile uint32_t * volatile rcu = (volatile uint32_t *)0x40021000;
    rcu[6] |= 0x00020000U;  /* offset 0x18 = APB2EN */

    volatile uint8_t * volatile tb = (volatile uint8_t *)TIMER15_BASE;

    *(volatile uint16_t *)(tb + 0x00) = 8;      /* CTL0: UPS */
    *(volatile uint32_t *)(tb + 0x2C) = 0xFFFF; /* CAR: max auto-reload */
    *(volatile uint16_t *)(tb + 0x28) = 0x002F; /* PSC: /48 -> 1MHz tick */

    /* Trigger update event: SWEVG |= UG (offset 0x14) */
    {
        uint16_t v = *(volatile uint16_t *)(tb + 0x14);
        v |= 1;
        *(volatile uint16_t *)(tb + 0x14) = v;
    }

    /* Pipeline sync NOPs */
    __asm volatile ("nop");
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* Clear interrupt enables */
    *(volatile uint16_t *)(tb + 0x10) = 0;

    /* Configure NVIC for TIMER15 interrupt (IRQ 21) */
    uint8_t nvic_cfg[4];
    nvic_cfg[0] = 0x15;
    nvic_cfg[3] = 1;
    nvic_cfg[1] = 0;
    nvic_cfg[2] = 0;
    gpio_config_pin(nvic_cfg);
}

/* Check TIMER0 update interrupt flag. Returns 1 if elapsed, 0 otherwise. */
uint32_t __attribute__((noinline)) timer_tick_elapsed(uint8_t *ctrl)
{
    (void)ctrl;
    volatile uint16_t *timer0_intf = (volatile uint16_t *)(TIMER0_BASE + 0x10);
    if ((*timer0_intf & 1) != 0) {
        *timer0_intf = *timer0_intf & 0xFFFE;
        return 1;
    }
    return 0;
}

/* ================================================================
 * Voltage monitoring — check voltage against min/max limits.
 * If out of range for 51+ ticks, sets voltage error flag.
 * ================================================================ */
void __attribute__((noinline)) timer_tick_update(uint8_t *param)
{
    volatile uint8_t *sr = servo_regs;
    if (sr[SR_VOLTAGE] <= sr[SR_MAX_VOLTAGE]) {
        /* Voltage reading <= max voltage: check min */
        sr = servo_regs;
        uint8_t flag;
        if (sr[SR_VOLTAGE] < sr[SR_MIN_VOLTAGE]) {
            sr = servo_regs;
            flag = sr[SR_ERROR_FLAGS] | 1;   /* below min: set voltage error */
        } else {
            sr = servo_regs;
            flag = sr[SR_ERROR_FLAGS] & 0xFE; /* within range: clear */
        }
        sr = servo_regs;
        sr[SR_ERROR_FLAGS] = flag;
        *(uint16_t *)(param + 4) = 0;
        return;
    }
    /* Voltage > max: count ticks */
    if (*(uint16_t *)(param + 4) < 0x33) {
        *(uint16_t *)(param + 4) = *(uint16_t *)(param + 4) + 1;
        return;
    }
    /* 51+ ticks: set voltage error */
    sr = servo_regs;
    sr[SR_ERROR_FLAGS] |= 1;
}

/* ================================================================
 * Apply motor safety checks based on operating mode.
 * Non-PWM modes: check I2C encoder errors.
 * PWM mode: compute PID and apply duty.
 * ================================================================ */
/* ================================================================
 * PWM mode output compute — original at 0x08003484.
 * Reads goal from servo_regs[44-45] (sign-magnitude, bit 10 = sign),
 * clamps to ±max_output (servo_regs[48-49]), writes to PWM_OUTPUT.
 * When torque_enable != 1: output = 0.
 * ================================================================ */
void __attribute__((noinline)) pwm_mode_compute(uint8_t *param)
{
    (void)param;
    volatile uint8_t *sr = servo_regs;
    uint16_t max_out_u = *(uint16_t *)(sr + SR_MAX_OUTPUT_LO);
    int16_t max_out = (int16_t)max_out_u;

    sr = servo_regs;
    uint8_t torque_en = sr[SR_TORQUE_ENABLE];

    int16_t goal;
    if (torque_en == 1) {
        /* Read PWM goal from servo_regs[44-45] (sign-magnitude) */
        sr = servo_regs;
        uint16_t raw = *(uint16_t *)(sr + SR_PWM_GOAL_LO);
        if (raw & PWM_GOAL_SIGN_BIT) {
            /* Bit 10 set: negative */
            sr = servo_regs;
            uint16_t raw2 = *(uint16_t *)(sr + SR_PWM_GOAL_LO);
            raw2 &= ~(uint16_t)PWM_GOAL_SIGN_BIT;
            goal = -(int16_t)raw2;
        } else {
            goal = (int16_t)raw;
        }
    } else {
        goal = 0;
    }

    /* Clamp to [-max_output, max_output] */
    if (goal < -max_out) {
        goal = -(int16_t)max_out_u;
    } else if (goal > max_out) {
        goal = max_out;
    }

    volatile uint8_t *pc = pwm_ctrl;
    *(int16_t *)(pc + PWM_OUTPUT) = goal;
}

void __attribute__((noinline)) motor_safety_apply(uint8_t *adc_st)
{
    (void)adc_st;
    volatile uint8_t *sr = servo_regs;
    if (sr[SR_OPERATING_MODE] != 2) {
        /* Position/speed/multi-turn/current mode: check I2C encoder errors */
        i2c_error_stop();
    } else {
        /* PWM mode: compute output from servo_regs[44-45] and apply */
        pwm_mode_compute(pid_state_arr);
        sr = servo_regs;
        int32_t duty = (int32_t)*(volatile int16_t *)(pwm_ctrl_arr + PWM_OUTPUT);
        int32_t torque = (int32_t)sr[SR_TORQUE_ENABLE];
        pwm_apply_output(timer_ctrl_arr, duty, torque, 0);
    }
}

/* Zero-init pwm_ctrl_arr. */
void __attribute__((noinline, section(".text.keep"))) pwm_ctrl_zero_init(void)
{
    zero_init_helper(pwm_ctrl_arr);
}

/* Initialize timer_ctrl_arr — runs as .init_array constructor before main(),
 * matching original binary constructor [4]. */
void __attribute__((constructor, noinline)) timer_ctrl_init(void)
{
    pwm_ctrl_init((uint32_t *)timer_ctrl_arr);
}

/* ================================================================
 * PWM apply output with GPIO direction control.
 * Sets GPIOA pin 6 (motor direction) based on brake state,
 * then delegates to timer0_set_duty.
 * ================================================================ */
void __attribute__((noinline)) pwm_apply_output(uint8_t *param, int32_t duty,
    int32_t brake_state, uint32_t extra)
{
    (void)extra;
    int bVar1;

    if (param[6] == 0) {
        /* Motor enabled */
        if (((uint32_t)(brake_state - 1) & 0xFF) < 2) {
            /* brake_state == 1 or 2: set GPIO direction pin */
            *(volatile uint32_t *)(GPIOA_BASE + 0x18) = 0x40;
        } else {
            /* brake_state == 0 or 3+: clear GPIO direction pin */
            *(volatile uint32_t *)(GPIOA_BASE + 0x28) = 0x40;
        }
        bVar1 = (param[8] != (uint8_t)brake_state);
        if (bVar1) {
            param[8] = (uint8_t)brake_state;
        }
        *(uint16_t *)(param + 4) = PWM_MAX_DUTY;  /* max duty = 970 */
    } else {
        bVar1 = (brake_state == 0);
    }

    /* Negate duty if config bit 0 is clear */
    {
        volatile uint8_t *sr = servo_regs;
        if ((sr[SR_CONFIG] & 1) == 0) {
            duty = -(int16_t)duty;
        }
    }

    timer0_set_duty(param, duty, bVar1);

    /* Force-load shadow registers on brake to ensure the H-bridge
     * bootstrap caps charge immediately, not after the next update
     * event (~1ms with CREP=23). Without this, the bootstrap may
     * drain during the wait, preventing motor startup from cold. */
    if (bVar1) {
        *(volatile uint16_t *)(0x40012C00U + 0x14) = 1; /* TIMER0_SWEVG = UG */
    }
}

/* Initialize ring buffer in timer_ctrl_arr motion sub-state. */
void __attribute__((noinline, section(".text.keep"))) timer_ctrl_ring_init(void)
{
    ring_buf_init((uint8_t *)(timer_ctrl_arr + TMR_MOTION_BASE));
}

/* ================================================================
 * Safety shutdown handler — checks alarm_shutdown vs error_flags.
 * Action 1: force stop (zero PWM, reset encoder).
 * Action 2: apply speed limit from max_speed register.
 * ================================================================ */
void __attribute__((noinline)) motor_safety_halt(void)
{
    int32_t action = 0;

    uint8_t alarm_mask = servo_regs_arr[SR_ALARM_SHUTDOWN];
    uint8_t alarm_flags = servo_regs_arr[SR_ERROR_FLAGS];

    /* Check bit 5: extended error protection */
    if ((alarm_mask & ALARM_EXTENDED_ERR) == 0) {
        /* Not enabled */
    } else {
        if ((alarm_flags & ALARM_EXTENDED_ERR) == 0) {
            /* Not active */
        } else {
            action = 2;
        }
    }

    /* Check bit 2: overtemp protection + bit 0: voltage protection */
    if ((alarm_mask & 4) == 0) {
        goto no_overtemp;
    }
    if ((alarm_flags & 4) == 0) {
no_overtemp:
        if ((alarm_mask & 1) != 0) {
            goto check_voltage;
        }
    } else {
        if ((alarm_mask & 1) == 0) {
            goto force_stop;
        }
        action = 1;
check_voltage:
        if ((alarm_flags & 1) != 0) {
            goto force_stop;
        }
    }

    if (action != 1) {
        if (action != 2) {
            return;
        }
        /* action == 2: apply speed limit */
        {
            uint8_t spd = servo_regs_arr[SR_MAX_SPEED];
            uint16_t spd10 = (uint16_t)(spd * 10);
            if (*(int16_t *)(pwm_ctrl_arr + PWM_OUTPUT) >= 0) {
                *(uint16_t *)(pwm_ctrl_arr + PWM_OUTPUT) = spd10;
            } else {
                *(uint16_t *)(pwm_ctrl_arr + PWM_OUTPUT) = -spd10;
            }
        }
        return;
    }

force_stop:
    /* Force stop: zero PWM output and reset encoder position */
    {
        *(uint16_t *)(pwm_ctrl_arr + PWM_OUTPUT) = 0;
        uint32_t pos = *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS);
        timer0_set_pwm(pid_state_arr, pos, pos);
    }
}

/* ================================================================
 * Configure timer_ctrl_arr based on config byte, then init TIMER0.
 * Original at 0xC4C: sets timer_ctrl[6,7] from servo_regs config,
 * optionally configures PA6 for analog, then calls timer0_hw_init.
 * TIMER15 is NOT configured here — that happens in motor_obj_init.
 * ================================================================ */
void __attribute__((noinline)) timer15_init_wrapper(uint8_t *param)
{
    if (servo_regs_arr[SR_CONFIG] & 0x20) {
        param[6] = 1;
        /* I2C encoder mode: PA6 is H-bridge enable (GPIO output).
         * Set PA6 to output mode: CTL bits [13:12] = 01 */
        volatile uint32_t *gpioa = (volatile uint32_t *)GPIOA_BASE;
        gpioa[0] = (gpioa[0] & ~0x2000U) | 0x1000U;  /* PA6 = output */
    } else {
        param[6] = 0;
        /* PA6: analog mode for current sense */
        *(volatile uint32_t *)(GPIOA_BASE + 0x00) |= 0x1000;
        *(volatile uint32_t *)(GPIOA_BASE + 0x08) |= 0x3000;
        *(volatile uint16_t *)(GPIOA_BASE + 0x28) = 0x40;
    }
    if (servo_regs_arr[SR_CONFIG] & 0x40) {
        param[7] = 1;
    }
    /* Original calls timer0_hw_init(timer_ctrl) here, NOT timer15 init */
    timer0_hw_init(param);
}

/* Standalone TIMER15 hardware init — enables clock, configures prescaler and period. */
void __attribute__((noinline, section(".text.keep"))) timer15_hw_init_standalone(void)
{
    RCU_APB2EN |= 0x00020000U;
    *(volatile uint16_t *)(TIMER15_BASE + 0x00) = 8;
    *(volatile uint32_t *)(TIMER15_BASE + 0x2C) = 0xFFFF;
    *(volatile uint16_t *)(TIMER15_BASE + 0x28) = 0x002F;
    *(volatile uint16_t *)(TIMER15_BASE + 0x14) |= 1;
    *(volatile uint16_t *)(TIMER15_BASE + 0x10) = 0;
}

/* Configure PA2 as AF1 (USART1 TX/RX half-duplex) and init USART1 DMA. */
void __attribute__((noinline)) servo_regs_init_wrapper(uint8_t *param)
{
    *(volatile uint32_t *)(GPIOA_BASE + 0x20) |= 0x100;   /* AFSEL0: PA2 = AF1 */
    *(volatile uint32_t *)(GPIOA_BASE + 0x00) |= 0x20;     /* CTL: PA2 = AF mode */
    *(volatile uint32_t *)(GPIOA_BASE + 0x08) |= 0x10;     /* OSPD: PA2 speed */
    usart1_dma_init(param, servo_regs_arr[SR_BAUD_RATE]);
}

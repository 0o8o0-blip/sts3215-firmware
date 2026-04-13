/* PID Controller, Motor Control, and USART1 Init */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>

/* Peripheral bases (some already in servo_types.h, keep local for DMA channels) */
#undef DMA_BASE
#define DMA_BASE     0x40020000U
#define DMA_CH3_BASE 0x40020044U  /* DMA channel 3 (USART1 TX) */
#define DMA_CH4_BASE 0x40020058U  /* DMA channel 4 (USART1 RX) */
#define TIMER0_BASE  0x40012C00U

#define REG(base, off)   (*(volatile uint32_t *)((base) + (off)))
#define REG16(base, off) (*(volatile uint16_t *)((base) + (off)))


extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern void gpio_config_pin(uint8_t *cfg);
extern void position_pid(int32_t *state);
extern void speed_pid(int32_t *state);
extern void position_profile_init(uint8_t *state);

/* Baud rate lookup table — shared from vtables.c */
extern const uint32_t baud_rate_config[];  /* {baud, tx_time} pairs */

/* Configure USART1 in half-duplex mode with DMA RX/TX. */
void __attribute__((noinline)) usart1_dma_init(uint8_t *uart_st, uint32_t baud_idx)
{
    uart_st[0x111] = 1;  /* Mark UART initialized */

    /* Look up baud rate */
    uint32_t baud;
    if (baud_idx <= 7) {
        baud = baud_rate_config[baud_idx * 2];
    } else {
        baud = 1000000;
    }

    /* Enable USART1 clock */
    volatile uint32_t * volatile rcu = (volatile uint32_t *)0x40021000U;
    rcu[7] |= 0x00020000U;

    /* Configure USART1 */
    uint32_t baud_val = 48000000U / baud;
    volatile uint32_t * volatile usart = (volatile uint32_t *)USART1_BASE;
    usart[3] = baud_val;                    /* BAUD register at +0x0C */
    usart[0] = 0x04;                        /* CTL0 */
    usart[2] = 0x08;                        /* CTL2: HDEN */

    /* Configure DMA channel 3 (USART1 TX) */
    volatile uint32_t * volatile dma = (volatile uint32_t *)DMA_CH3_BASE;
    dma[2] = (uint32_t)&usart[0x28/4];     /* PADDR: USART1 TDATA (0x28) */
    dma[3] = 0;                             /* MADDR */
    dma[1] = 0;                             /* CNT */
    dma[0] = 0x90;                          /* CTL: DIR=1 (mem→periph) | MINC */
    dma[0] |= 1;                            /* CHEN */

    /* Configure DMA channel 4 (USART1 RX) — advance by 0x14 bytes */
    volatile uint32_t *dma2 = (volatile uint32_t *)((char *)dma + 0x14);
    dma2[2] = (uint32_t)&usart[0x24/4];    /* PADDR: USART1 RDATA (0x24) */
    dma2[3] = (uint32_t)(uart_st + 0x10);  /* MADDR: RX ring buffer */
    dma2[1] = 0x80;                         /* CNT: 128 bytes */
    dma2[0] = 0x30A0;                       /* CTL: circular RX, MINC, high prio */
    dma2[0] |= 1;                           /* CHEN */

    /* Enable DMA + idle interrupt on USART1 */
    usart[2] |= 0xC0;                       /* CTL2: DMAR | DMAT */
    usart[0] |= 0x10;                       /* CTL0: IDLEIE */

    /* Enable NVIC IRQ 28 (USART1) */
    uint8_t nvic_cfg[4] = { 0x1C, 1, 0, 1 };
    gpio_config_pin(nvic_cfg);

    /* Enable USART1 */
    usart[0] |= 1;                          /* CTL0: UEN */
}

/* ================================================================
 * Set TIMER0 PWM duty cycle for H-bridge motor control.
 *
 * Forward (duty >= 0): CH0CV=0, CH2CV=duty  → CH2N drives motor
 * Reverse (duty < 0):  CH0CV=|duty|, CH2CV=0 → CH0N drives motor
 * Brake (is_brake):    both channels = period → coast/brake
 *
 * Duty is clamped to [-period, +period] where period = ctrl[4..5].
 * ================================================================ */
void __attribute__((noinline)) timer0_set_duty(uint8_t *ctrl, int32_t duty, int32_t is_brake)
{
    volatile uint32_t *timer0 = (volatile uint32_t *)TIMER0_BASE;

    if (is_brake) {
        uint16_t period = *(uint16_t *)(ctrl + 4);
        timer0[0x3C / 4] = period;    /* CH2CV = period */
        timer0[0x34 / 4] = period;    /* CH0CV = period */
        return;
    }

    int16_t period = *(int16_t *)(ctrl + 4);

    /* Clamp duty to [-period, +period] */
    if (duty < (int32_t)-period)
        duty = (int32_t)(int16_t)-period;
    else if (duty >= (int32_t)period)
        duty = (int32_t)period;

    if (duty >= 0) {
        /* Forward: CH0N off, CH2N drives */
        timer0[0x34 / 4] = 0;                /* CH0CV = 0 */
        timer0[0x3C / 4] = (uint16_t)duty;   /* CH2CV = duty */
    } else {
        /* Reverse: CH2N off, CH0N drives */
        timer0[0x3C / 4] = 0;                /* CH2CV = 0 */
        timer0[0x34 / 4] = (uint16_t)(-duty); /* CH0CV = |duty| */
    }
}

/* ================================================================
 * Scale and store PID output values as 64-bit products.
 * Uses punch divisor from servo_regs for normalization.
 * ================================================================ */
static inline void smull_store(uint8_t *dst_lo, uint8_t *dst_hi,
    uint32_t a, uint32_t b)
{
    /* UMULL + sign correction for signed 64-bit multiply */
    uint32_t lo, hi;
    __asm__ volatile ("umull %0, %1, %2, %3"
        : "=r"(lo), "=r"(hi) : "r"(a), "r"(b));
    /* Sign correction: hi += a*(b>>31) + b*(a>>31) */
    {
        uint32_t a_sign = (uint32_t)((int32_t)a >> 31);
        uint32_t b_sign = (uint32_t)((int32_t)b >> 31);
        uint32_t tmp = a * b_sign;
        tmp += b * a_sign;  /* sign correction accumulate → MLA */
        hi += tmp;
    }
    *(uint32_t *)dst_lo = lo;
    *(uint32_t *)dst_hi = hi;
}

void __attribute__((noinline)) timer0_set_pwm(uint8_t *state,
    uint32_t forward, uint32_t reverse)
{
    /* Use high register to match binary encoding */
    volatile uint32_t fwd_sign;
    fwd_sign = (uint32_t)((int32_t)forward >> 31);

    volatile int32_t *pdiv = (volatile int32_t *)(pwm_ctrl + 0x2C);
    uint32_t scale = (uint32_t)((*pdiv * 10000) / *pdiv);

    /* Forward channel — 64-bit target (state+0x20) */
    smull_store(state + 0x20, state + 0x24, scale, forward);

    /* Reverse channel — 64-bit integrator (state+0x18) */
    pdiv = (volatile int32_t *)(pwm_ctrl + 0x2C);
    scale = (uint32_t)((*pdiv * 10000) / *pdiv);
    smull_store(state + 0x18, state + 0x1C, scale, reverse);

    /* Detect goal change -> reset profile */
    if (forward != *(uint32_t *)(state + 0x30)) {
        *(uint32_t *)(state + 0x30) = forward;
        state[0x28] = 0;
    }
    (void)fwd_sign;
}

/* Clamp position to CW/CCW angle limits. */
uint32_t __attribute__((noinline)) pid_clamp_angle(uint8_t *state, uint32_t position)
{
    (void)state;

    /* Compare CW and CCW limits */
    uint16_t cw_u  = *(volatile uint16_t *)(servo_regs_arr + 9);
    uint16_t ccw_u = *(volatile uint16_t *)(servo_regs_arr + 11);

    if (cw_u == ccw_u) return position;

    /* Range clamp */
    uint32_t cw_limit = (uint32_t)*(volatile uint16_t *)(servo_regs_arr + 9);
    if ((int32_t)position < (int32_t)cw_limit) {
        return (uint32_t)*(volatile uint16_t *)(servo_regs_arr + 9);
    }
    uint32_t ccw_limit = (uint32_t)*(volatile uint16_t *)(servo_regs_arr + 11);
    if ((int32_t)position > (int32_t)ccw_limit) {
        return (uint32_t)*(volatile uint16_t *)(servo_regs_arr + 11);
    }
    return position;
}

/* Update speed/acceleration limit from servo registers. */
void __attribute__((noinline)) pid_update_speed(uint8_t *pid_state)
{
    uint8_t accel = servo_regs_arr[SR_ACCELERATION];
    if (accel == 0) {
        /* Use speed * acceleration product — reload for each */
        *(uint32_t *)(pid_state + PID_SPEED_LIMIT) =
            (uint32_t)servo_regs_arr[SR_SPEED_MULT] * (uint32_t)servo_regs_arr[SR_MOVING_SPEED];
    } else {
        /* Use moving speed (clamped) */
        uint8_t moving_speed = servo_regs_arr[SR_MOVING_SPEED];
        if (moving_speed != 0 && moving_speed < accel) {
            servo_regs_arr[SR_ACCELERATION] = moving_speed;
        }
        *(uint32_t *)(pid_state + PID_SPEED_LIMIT) = (uint32_t)servo_regs_arr[SR_ACCELERATION];
    }

    /* Detect speed change -> reset profile */
    if (*(int32_t *)(pid_state + PID_SPEED_LIMIT) != *(int32_t *)(pid_state + PID_PREV_SPEED)) {
        *(int32_t *)(pid_state + PID_PREV_SPEED) = *(int32_t *)(pid_state + PID_SPEED_LIMIT);
        pid_state[PID_PROFILE_RESET] = 0;
    }
}

/* Compute goal position from registers, clamped to +/-8388607. */
#define GOAL_MIN  ((int32_t)0xFF800001)  /* -8388607 */
#define GOAL_MAX  ((int32_t)0x007FFFFF)  /*  8388607 */

uint32_t __attribute__((noinline)) pid_goal_compute(void)
{
    /* Read goal position from registers (16-bit, potentially signed) */
    uint32_t goal = (uint32_t)*(uint16_t *)(servo_regs_arr + SR_GOAL_POS_LO);
    if ((*(uint16_t *)(servo_regs_arr + SR_GOAL_POS_LO) & SPEED_GOAL_SIGN_BIT) != 0) {
        goal = -(goal & ~SPEED_GOAL_SIGN_BIT);
    }

    /* Apply direction multiplier */
    if (servo_regs_arr[SR_PUNCH_DIVISOR] == 0) {
        servo_regs_arr[SR_PUNCH_DIVISOR] = 1;
    }
    uint8_t punch_div = servo_regs_arr[SR_PUNCH_DIVISOR];

    uint8_t mode = servo_regs_arr[SR_OPERATING_MODE];
    if (mode == 0) {
        goal = (uint32_t)((uint32_t)punch_div * goal);
    } else if (mode == 3) {
        {
            uint32_t dir = (uint32_t)punch_div;
            uint32_t base = *(uint32_t *)pwm_ctrl;  /* pwm_ctrl[0] = current goal_reg */
            goal = base + dir * goal;  /* dir*goal + base → MLA */
        }
    }

    /* Clamp to absolute min/max range */
    if ((int32_t)goal < GOAL_MIN) {
        goal = (uint32_t)GOAL_MIN;
    }
    if (GOAL_MAX <= (int32_t)goal) {
        goal = (uint32_t)GOAL_MAX;
    }

    return goal;
}

/* ================================================================
 * Rate-limit PID output change toward a target value.
 * Uses acceleration for ramping. Accumulates into 64-bit integrator.
 * ================================================================ */
void __attribute__((noinline)) motion_rate_limit(int32_t *state,
    int32_t target, int32_t accel, int32_t max_val)
{
    /* Clamp target to ±max_val */
    int32_t abs_target = target < 0 ? -target : target;
    if (abs_target < max_val) {
        if (target < 0) target = -max_val;
        if (target > 0) target = max_val;
    }

    int32_t current = state[3];  /* state + 0xC */
    int32_t delta = target - current;

    if (delta == 0) {
        goto accumulate;
    }

    if (delta < 0) {
        /* Target is below current — need to decrease */
        if (accel == 0) {
            state[3] = target;
            if (target > 0 || state[3] >= 0) goto accumulate;
            if (-(int32_t)max_val < state[3]) {
                state[3] = -max_val;
            }
        } else {
            state[3] = current - accel;
            if (target <= 0) {
                if (state[3] > 0 && state[3] < max_val) {
                    state[3] = 0;
                    goto accumulate;
                }
                if (state[3] < 0 && -(int32_t)max_val < state[3]) {
                    state[3] = -max_val;
                }
            }
        }
        if (state[3] < target) {
            state[3] = target;
        }
    } else {
        /* Target is above current — need to increase */
        if (accel == 0) {
            state[3] = target;
            if (target < 0 || state[3] <= 0) goto accumulate;
            if (state[3] < max_val) {
                state[3] = max_val;
            }
        } else {
            state[3] = current + accel;
            if (target >= 0) {
                if (state[3] < 0 && -(int32_t)max_val < state[3]) {
                    state[3] = 0;
                    goto accumulate;
                }
                if (state[3] > 0 && state[3] < max_val) {
                    state[3] = max_val;
                }
            }
        }
        if (target < state[3]) {
            state[3] = target;
        }
    }

accumulate:
    /* Accumulate output into 64-bit integrator */
    {
        uint32_t val = (uint32_t)state[3];
        uint32_t lo = (uint32_t)state[6];  /* state + 0x18 */
        uint32_t new_lo = lo + val;
        state[6] = (int32_t)new_lo;
        state[7] = state[7] + ((int32_t)val >> 31) + (new_lo < lo ? 1 : 0);
    }
}

/* ================================================================
 * Main PID controller tick — called when encoder + ADC data is ready.
 * Processes position control, speed control, and PWM output.
 * ================================================================ */
void __attribute__((noinline)) pid_compute(uint8_t *state)
{
    /* Flag [0]: position update */
    if (state[0] != 0) {
        state[0] = 0;

        /* Compute goal and clamp to angle limits */
        uint32_t goal = pid_goal_compute();
        goal = pid_clamp_angle(state, goal);

        /* Store clamped goal in pwm_ctrl_arr[0] */
        *(uint32_t *)pwm_ctrl_arr = goal;

        /* 64-bit signed multiply: scale * goal */
        volatile uint32_t *pdiv = (volatile uint32_t *)(pwm_ctrl_arr + PWM_PUNCH_DIV);
        uint32_t scale = (uint32_t)((int32_t)(*pdiv * 10000) / (int32_t)*pdiv);
        /* UMULL + sign correction */
        {
            uint32_t lo, hi;
            __asm__ volatile ("umull %0, %1, %2, %3"
                : "=r"(lo), "=r"(hi) : "r"(scale), "r"(goal));
            /* Sign correction */
            {
                uint32_t s_sign = (uint32_t)((int32_t)scale >> 31);
                uint32_t g_sign = (uint32_t)((int32_t)goal >> 31);
                uint32_t corr = scale * g_sign;
                corr += goal * s_sign;  /* sign correction accumulate → MLA */
                hi += corr;
            }
            *(uint32_t *)(state + PID_GOAL_64) = lo;
            *(uint32_t *)(state + PID_GOAL_64 + 4) = hi;
        }

        /* Detect goal change -> reset profile */
        if (goal != *(uint32_t *)(state + PID_PREV_GOAL)) {
            *(uint32_t *)(state + PID_PREV_GOAL) = goal;
            state[PID_PROFILE_RESET] = 0;
        }

        /* Trigger motion update for position/multi-turn modes */
        if (servo_regs_arr[SR_OPERATING_MODE] == 0) {
            servo_regs_arr[SR_TORQUE_ENABLE] = 1;
        } else if (servo_regs_arr[SR_OPERATING_MODE] == 3) {
            servo_regs_arr[SR_TORQUE_ENABLE] = 1;
        }
    }

    /* Flag [1]: speed/torque update */
    if (state[1] != 0) {
        state[1] = 0;

        if (servo_regs_arr[SR_OPERATING_MODE] == 0) {
            /* Position mode: run position PID */
            position_pid((int32_t *)state);
        } else if (servo_regs_arr[SR_OPERATING_MODE] == 3) {
            position_pid((int32_t *)state);
        } else if (servo_regs_arr[SR_OPERATING_MODE] == 1) {
            /* Speed mode */
            speed_pid((int32_t *)state);
            servo_regs_arr[SR_TORQUE_ENABLE] = 1;
        } else if (servo_regs_arr[SR_OPERATING_MODE] == 4) {
            /* Current mode: re-enable torque if overload cleared */
            servo_regs_arr[SR_TORQUE_ENABLE] = 1;
        }
    }

    /* Flag [2]: PWM mode */
    if (state[2] != 0) {
        state[2] = 0;
        if (servo_regs_arr[SR_OPERATING_MODE] == 2) {
            servo_regs_arr[SR_TORQUE_ENABLE] = 1;
        }
    }

    /* Flag [3]: speed limit update */
    if (state[3] != 0) {
        state[3] = 0;
        pid_update_speed(state);
    }
}

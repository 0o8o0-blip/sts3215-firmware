/* Feetech STS3215 — Motion profile (trapezoid velocity planning) */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t timer_ctrl_arr[]; extern uint8_t * volatile timer_ctrl;

/* Functions from other files */
extern void motion_rate_limit(int32_t *state, int32_t target, int32_t accel, int32_t max_val);


/* Reset the motion profile state to all zeros. */
void __attribute__((noinline, section(".text.keep"))) motion_helper_reset(uint8_t *param)
{
    *(uint32_t *)(param + 8) = 0;
    *(uint32_t *)(param + 0x0C) = 0;
    *(uint32_t *)(param + 0x10) = 0;
    param[4] = 0;
    param[PID_PROFILE_RESET] = 0;
    *(uint32_t *)(param + 0x2C) = 0;
    *(uint32_t *)(param + 0x30) = 0;
    *(uint32_t *)(param + 0x34) = 0;
    param[0] = 0;
    param[1] = 0;
    param[2] = 0;
    param[3] = 0;
}

/* Handle motion accumulation with 64-bit integrator clamping. */
void __attribute__((noinline)) motion_profile_accel(int32_t *s, uint32_t unused,
    int32_t delta_lo, int32_t delta_hi)
{
    (void)unused;
    if (delta_lo == 0 && delta_hi == 0) {
        return;
    }

    /* 64-bit pointers for ldrd/strd generation */
    volatile int64_t *integ64 = (volatile int64_t *)(s + 6);
    volatile int64_t *tgt64   = (volatile int64_t *)(s + 8);

    /* Check if delta64 >= 1 (positive direction) */
    if ((int32_t)(uint32_t)(delta_lo == 0) <= delta_hi) {
        /* Moving forward: speed = step, integrator += step */
        int32_t step = s[4];
        s[3] = step;
        *integ64 += (int64_t)step;
        /* Clamp: if integrator >= target, snap to target */
        if (*integ64 >= *tgt64) {
            *integ64 = *tgt64;
            s[3] = 0;
        }
    }

    /* Check if delta_hi < 0 (negative direction) */
    if (delta_hi < 0) {
        /* Moving backward: speed = -step, integrator -= step */
        int32_t step = s[4];
        s[3] = -step;
        *integ64 -= (int64_t)step;
        /* Clamp: if integrator <= target, snap to target */
        if (*integ64 <= *tgt64) {
            *integ64 = *tgt64;
            s[3] = 0;
        }
    }
}

/* ================================================================
 * Compute braking distance — distance needed to decelerate to zero.
 * Formula: q = speed / accel
 *   result = (q * (speed*2 - (q*accel + accel))) / 2
 * ================================================================ */
static inline int32_t __attribute__((always_inline)) brake_formula(int32_t speed, int32_t accel)
{
    int32_t q = speed / accel;
    int32_t tmp = q * accel + accel;
    int32_t tmp2 = speed * 2 - tmp;
    int32_t prod = q * tmp2;
    /* Round toward zero */
    int32_t rounding = (int32_t)((uint32_t)prod >> 31);
    return (prod + rounding) >> 1;
}


uint32_t __attribute__((noinline)) motion_braking_distance(uint32_t unused, int32_t accel, int32_t speed)
{
    (void)unused;

    if (accel == 0) return 0;

    if (accel > 0) {
        if (speed < 0) return 0;

        int32_t q = speed / accel;
        int32_t tmp = accel;
        tmp += q * accel;
        int32_t tmp2 = speed * 2 - tmp;
        int32_t prod = q * tmp2;
        int32_t rounding = (int32_t)((uint32_t)prod >> 31);
        int32_t raw = (prod + rounding) >> 1;

        if (speed >= 1) {
            raw = (int32_t)((uint32_t)raw & ~((uint32_t)((int32_t)raw >> 31)));
        }
        if (speed < 0) {
            raw = (int32_t)((uint32_t)raw & (uint32_t)((int32_t)raw >> 31));
        }
        return (uint32_t)raw;
    }

    if (speed > 0) return 0;

    int32_t q2 = speed / accel;
    int32_t tmp2b = accel;
    tmp2b += q2 * accel;
    int32_t tmp2c = speed * 2 - tmp2b;
    int32_t prod2 = q2 * tmp2c;
    int32_t rounding2 = (int32_t)((uint32_t)prod2 >> 31);
    int32_t raw2 = (prod2 + rounding2) >> 1;

    if (speed >= 1) {
        raw2 = (int32_t)((uint32_t)raw2 & ~((uint32_t)((int32_t)raw2 >> 31)));
    }
    if (speed < 0) {
        raw2 = (int32_t)((uint32_t)raw2 & (uint32_t)((int32_t)raw2 >> 31));
    }
    return (uint32_t)raw2;
}

/* ================================================================
 * Deceleration and braking distance check.
 * If current speed is in wrong direction vs target: decelerate.
 * Otherwise: compute braking distance and set braking flag if needed.
 * ================================================================ */
void __attribute__((noinline)) motion_profile_reverse(int32_t *s, uint32_t unused,
    uint32_t dist_lo, int32_t dist_hi, int32_t target_speed, int32_t accel_step, int32_t limit)
{
    (void)unused;
    int32_t speed = s[3];

    if ((speed < 0) && (0 < target_speed)) {
        /* Speed is negative but target is positive */
        s[3] = speed + accel_step;
        if (-limit <= speed + accel_step) {
            s[3] = 0;
        }
        if ((limit < s[3]) || (-1 < target_speed)) goto update_integrator;
    } else {
        if ((speed < 1) || (-1 < target_speed)) {
            /* Check braking distance */
            uint32_t brake = motion_braking_distance((uint32_t)s, accel_step, speed);
            if ((int32_t)brake < 0) {
                brake = -brake;
            }
            int32_t brake_hi = (int32_t)brake >> 31;

            /* Take abs of distance */
            uint32_t abs_lo = dist_lo;
            int32_t abs_hi = dist_hi;
            if (dist_hi < 0) {
                uint32_t borrow = (dist_lo != 0) ? 1 : 0;
                abs_lo = -dist_lo;
                abs_hi = -dist_hi - borrow;
            }

            /* 64-bit compare: dist vs brake */
            if (brake_hi > abs_hi ||
                (brake_hi == abs_hi && brake > abs_lo)) {
                /* brake > dist: not enough room */
            } else {
                /* dist >= brake: set braking flag */
                *(uint8_t *)((uint8_t *)s + PID_PROFILE_RESET) = 1;
                return;
            }

            /* Decelerate: reduce speed */
            speed = speed - accel_step;
            s[3] = speed;
            if (((speed < limit) && (0 < target_speed)) ||
                ((-limit < speed) && (target_speed < 0))) {
                s[3] = 0;
            }
            goto update_integrator;
        }
        /* Speed positive, target negative */
        s[3] = speed + accel_step;
        if ((-limit <= speed + accel_step) && (0 < target_speed)) {
            s[3] = 0;
        }
        if (limit < s[3]) goto update_integrator;
    }
    s[3] = 0;

update_integrator:
    if (*(uint8_t *)((uint8_t *)s + PID_PROFILE_RESET) != 0) {
        return;
    }
    /* 64-bit accumulate: integrator += speed (sign-extended) */
    uint32_t spd = (uint32_t)s[3];
    uint32_t pos_lo = (uint32_t)s[6];
    s[6] = (int32_t)(pos_lo + spd);
    s[7] = s[7] + ((int32_t)spd >> 31) + ((pos_lo + spd < pos_lo) ? 1 : 0);
}

/* ================================================================
 * Acceleration phase of motion profile.
 * Uses 64-bit comparisons for distance vs braking distance.
 * ================================================================ */
void __attribute__((noinline)) motion_profile_forward(int32_t *s, uint32_t unused,
    uint32_t dist_lo, int32_t dist_hi, uint32_t target_speed, int32_t accel, int32_t max_val)
{
    (void)unused;
    uint32_t uVar4 = (uint32_t)s[3];

    /* Compute braking distance from current speed */
    uint32_t uVar1 = motion_braking_distance((uint32_t)s, accel, uVar4);
    if ((int32_t)uVar1 < 0) {
        uVar1 = -uVar1;
    }
    int32_t iVar2 = (int32_t)uVar1 >> 31;

    /* abs(dist) */
    if (dist_hi < 0) {
        uint32_t bVar5 = (dist_lo != 0) ? 1 : 0;
        dist_lo = -dist_lo;
        dist_hi = -dist_hi - bVar5;
    }

    /* 64-bit signed comparison: brake >= dist? */
    if (iVar2 > dist_hi || (iVar2 == dist_hi && uVar1 >= dist_lo)) {
        /* Brake distance exceeds remaining distance: decelerate */
        s[3] = (int32_t)(uVar4 - (uint32_t)accel);
    } else if (uVar4 != target_speed) {
        /* Accelerate toward target speed */
        uVar4 = uVar4 + (uint32_t)accel;
        /* Clamp: if |new_speed| > |target_speed|, use target */
        int32_t abs_target = ((int32_t)target_speed ^ ((int32_t)target_speed >> 31))
                              - ((int32_t)target_speed >> 31);
        int32_t abs_new = ((int32_t)uVar4 ^ ((int32_t)uVar4 >> 31))
                           - ((int32_t)uVar4 >> 31);
        if (abs_target < abs_new) {
            uVar4 = target_speed;
        }
        s[3] = (int32_t)uVar4;
    }

    /* Apply minimum speed threshold */
    if ((0 < (int32_t)target_speed) && (s[3] < max_val)) {
        s[3] = max_val;
    }

    int64_t *integ64 = (int64_t *)(s + 6);
    int64_t *tgt64 = (int64_t *)(s + 8);

    if ((int32_t)target_speed < 0) {
        if (-max_val < s[3]) {
            s[3] = -max_val;
        }
        int64_t speed_ext = (int64_t)s[3];
        *integ64 += speed_ext;
        if (*integ64 < *tgt64) goto clamp_to_target;
    } else {
        int64_t speed_ext = (int64_t)s[3];
        *integ64 += speed_ext;
        if (*integ64 < *tgt64) {
            return;
        }
    }
    if ((int32_t)target_speed > 0) goto clamp_to_target;
    if (*tgt64 >= *integ64) {
        if ((int32_t)target_speed < 0) goto clamp_to_target;
    }
    return;

clamp_to_target:
    *integ64 = *tgt64;
    s[3] = 0;
}

/* ================================================================
 * Main motion profile dispatcher — decides between acceleration
 * (forward) and deceleration (reverse/braking check) phases.
 * ================================================================ */
void __attribute__((noinline)) motion_profile_update(int32_t *s, uint32_t unused,
    int32_t delta_lo, int32_t delta_hi)
{
    (void)unused;
    int32_t accel = s[4];

    if ((delta_lo == 0 && delta_hi == 0) || accel == 0) {
        /* No distance or no acceleration: use rate limiting */
        motion_rate_limit(s, 0, s[2], *(int32_t *)(pwm_ctrl + PWM_MIN_SPEED));
        return;
    }

    if (delta_hi < (int32_t)(uint32_t)(delta_lo == 0)) {
        /* Negative direction */
        if (delta_hi < 0) {
            if (*(uint8_t *)((uint8_t *)s + PID_PROFILE_RESET) != 0 ||
                (motion_profile_reverse(s, 0, delta_lo, delta_hi,
                    -accel, -s[2], *(int32_t *)(pwm_ctrl + PWM_MIN_SPEED)),
                 *(uint8_t *)((uint8_t *)s + PID_PROFILE_RESET) != 0)) {
                volatile int32_t neg_accel = -s[4];
                volatile int32_t neg_speed = -s[2];
                motion_profile_forward(s, 0, delta_lo, delta_hi, neg_accel,
                                       neg_speed, *(int32_t *)(pwm_ctrl + PWM_MIN_SPEED));
            }
        }
    } else {
        /* Positive direction */
        if (*(uint8_t *)((uint8_t *)s + PID_PROFILE_RESET) != 0 ||
            (motion_profile_reverse(s, 0, delta_lo, delta_hi,
                accel, s[2], *(int32_t *)(pwm_ctrl + PWM_MIN_SPEED)),
             *(uint8_t *)((uint8_t *)s + PID_PROFILE_RESET) != 0)) {
            volatile int32_t pos_accel = s[4];
            volatile int32_t pos_speed = s[2];
            motion_profile_forward(s, 0, delta_lo, delta_hi, pos_accel,
                                   pos_speed, *(int32_t *)(pwm_ctrl + PWM_MIN_SPEED));
        }
    }
}

/* motion_profile_update_main removed — logic inlined into
 * position_motion_guard (pid.c) to match original structure.
 * The original has two separate functions:
 *   0x08003178: position guard (integrator + GOAL_POS + SPEED_PROFILE)
 *   0x0800325C: speed guard (no integrator, only SPEED_PROFILE)
 */

/* ================================================================
 * Initialize motion profile with speed, acceleration, and
 * torque parameters from servo registers.
 * ================================================================ */
void __attribute__((noinline)) position_profile_init(uint8_t *s, int32_t param_2, int32_t param_3)
{
    /* Set speed limit */
    if (param_2 == 0) {
        *(uint32_t *)(s + 8) =
            (uint32_t)(servo_regs)[SR_SPEED_MULT] * (uint32_t)(servo_regs)[SR_MOVING_SPEED];
    } else {
        *(int32_t *)(s + 8) = param_2;
    }

    /* Set max torque / acceleration */
    if (param_3 == 0) {
        if ((servo_regs)[SR_OPERATING_MODE] == 1) {
            /* Speed mode: no acceleration limit */
            *(uint32_t *)(s + 0x10) = 0;
        } else {
            uint32_t uVar1;
            if (((servo_regs)[SR_CONFIG] & 8) == 0) {
                uVar1 = 0;
            } else {
                uVar1 = *(uint32_t *)(pwm_ctrl + PWM_MAX_TORQUE);
            }
            *(uint32_t *)(s + 0x10) = uVar1;
        }
    } else if (((servo_regs)[SR_CONFIG] & 4) == 0) {
        *(int32_t *)(s + 0x10) = param_3 * *(int32_t *)(pwm_ctrl + PWM_PUNCH_DIV) * 10;
    } else {
        *(int32_t *)(s + 0x10) =
            (param_3 * *(int32_t *)(pwm_ctrl + PWM_PUNCH_DIV) * 10)
            / *(int32_t *)(pwm_ctrl + PWM_PUNCH_DIV);
    }

    /* Initialize tracking values */
    *(uint32_t *)(s + 0x34) = *(uint32_t *)(s + 8);
    *(uint32_t *)(s + 0x2C) = *(uint32_t *)(s + 0x10);
    s[PID_PROFILE_RESET] = 0;
}

/* ================================================================
 * Temperature monitoring — if temp exceeds max_temp for 50+ ticks,
 * sets overtemp flag. Clears when temp drops 5 below limit.
 * ================================================================ */
void __attribute__((noinline)) motion_step(uint8_t *param)
{
    volatile uint8_t *sr = servo_regs;
    if ((sr[SR_ERROR_FLAGS] & 4) == 0) {
        /* Not in overtemp state */
        sr = servo_regs;
        if (sr[SR_MAX_TEMP] < sr[SR_TEMPERATURE]) {
            uint16_t *counter = (uint16_t *)(param + 6);
            if (*counter > 0x32) {
                /* Exceeded 50 ticks: set overtemp flag */
                sr = servo_regs;
                sr[SR_ERROR_FLAGS] |= 4;
                return;
            }
            *counter = *counter + 1;
            return;
        }
        /* Temp OK: reset counter */
        *(uint16_t *)(param + 6) = 0;
    } else {
        /* In overtemp state: clear when temp drops 5 below limit */
        sr = servo_regs;
        if (sr[SR_TEMPERATURE] + 5 < (uint32_t)sr[SR_MAX_TEMP]) {
            sr = servo_regs;
            sr[SR_ERROR_FLAGS] &= ~4U;
        }
    }
}

/* ================================================================
 * Overload protection — monitors motor current vs threshold.
 * If current exceeds threshold for (threshold*10) ticks,
 * sets overload flag.
 * ================================================================ */
void __attribute__((noinline)) overload_protect(uint8_t *param)
{
    uint8_t threshold_byte = servo_regs_arr[SR_RESERVED_26];
    int16_t current = *(int16_t *)(pwm_ctrl_arr + PWM_CURRENT_SENSE);
    uint16_t abs_current = (current < 0) ? -current : current;
    uint16_t max_torque = *(volatile uint16_t *)(servo_regs_arr + SR_MAX_CURRENT_LO);

    /* threshold * 10 */
    uint16_t threshold_10 = (uint16_t)(threshold_byte * 10);
    if (max_torque == 0 || threshold_10 == 0) {
        /* No torque limit or no threshold: clear overload */
        servo_regs_arr[SR_ERROR_FLAGS] &= ~8U;
        return;
    }

    if (abs_current < *(volatile uint16_t *)(servo_regs_arr + SR_MAX_CURRENT_LO)) {
        /* Current below threshold: reset counter */
        *(uint16_t *)(param + 8) = 0;
        if (servo_regs_arr[SR_TORQUE_ENABLE] == 1) {
            servo_regs_arr[SR_ERROR_FLAGS] &= ~8U;
        }
    } else {
        /* Current at or above threshold */
        if (*(uint16_t *)(param + 8) <= threshold_10) {
            *(uint16_t *)(param + 8) = *(uint16_t *)(param + 8) + 1;
            return;
        }
        /* Exceeded time limit: set overload flag */
        servo_regs_arr[SR_ERROR_FLAGS] |= 8;
        if ((servo_regs_arr[SR_ALARM_SHUTDOWN] & 8) != 0) {
            if ((servo_regs_arr[SR_ERROR_FLAGS] & 8) == 0) {
                return;
            }
            servo_regs_arr[SR_TORQUE_ENABLE] = 0;
        }
    }
}

/* ================================================================
 * Motion profile ramp — guard + ramp step.
 * Checks torque_enable == 1, then ramps state[0xC] toward
 * state[0x10] at rate state[8].
 * ================================================================ */
void __attribute__((noinline, section(".text.keep"))) motion_profile_ramp(int32_t *state)
{
    if (servo_regs_arr[SR_TORQUE_ENABLE] != 1) return;

    int32_t accel = state[2];
    if (accel == 0) {
        /* No acceleration: set directly */
        state[3] = state[4];
        int32_t punch = *(int32_t *)(pwm_ctrl_arr + PWM_PUNCH_DIV);
        int32_t p10 = punch + (punch << 2);
        p10 = p10 << 1;
        int32_t speed = state[3] / p10;
        *(int32_t *)(pwm_ctrl_arr + PWM_SPEED_PROFILE) = speed;
        return;
    }

    /* Ramp toward target */
    int32_t target = state[4];
    int32_t current = state[3];
    int32_t delta = target - current;

    if (delta > 0) {
        current += accel;
        if (current >= target) current = target;
        state[3] = current;
    } else if (delta < 0) {
        current -= accel;
        if (current <= target) current = target;
        state[3] = current;
    }

    /* Compute speed */
    int32_t punch = *(int32_t *)(pwm_ctrl_arr + PWM_PUNCH_DIV);
    int32_t p10 = punch + (punch << 2);
    p10 = p10 << 1;
    int32_t speed = state[3] / p10;
    *(int32_t *)(pwm_ctrl_arr + PWM_SPEED_PROFILE) = speed;
}

/* Zero the motion profile ramp state and PWM output. */
void __attribute__((noinline, section(".text.keep"))) motion_profile_zero(int32_t *state)
{
    state[3] = 0;
    *(int32_t *)(pwm_ctrl_arr + PWM_SPEED_PROFILE) = 0;
    *(uint16_t *)(pwm_ctrl_arr + PWM_OUTPUT) = 0;
}


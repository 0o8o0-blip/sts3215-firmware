/* Feetech STS3215 — PID controller */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t adc_state_arr[]; extern uint8_t * volatile adc_state;
extern uint8_t encoder_ctrl_arr[]; extern uint8_t * volatile encoder_ctrl;
extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;
extern uint8_t pid_state_arr[]; extern uint8_t * volatile pid_state;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t timer_ctrl_arr[]; extern uint8_t * volatile timer_ctrl;

/* Functions from other files */
extern void motion_profile_accel(int32_t *s, uint32_t unused,
    int32_t delta_lo, int32_t delta_hi);
extern void motion_profile_update(int32_t *s, uint32_t unused,
    int32_t delta_lo, int32_t delta_hi);
extern void motor_safety_halt(void);
extern void pwm_apply_output(uint8_t *param, int32_t duty, int32_t brake_state, uint32_t extra);
extern void timer0_set_duty(uint8_t *ctrl, int32_t duty, int32_t is_brake);
extern void motion_helper_reset(uint8_t *param);
extern uint32_t position_linearize(uint32_t position);

/* Forward declarations within this file */
void pid_full_compute(uint8_t *param);
void pid_speed_compute(uint8_t *param);
void pid_current_compute(uint8_t *param);
void position_pid_reset(uint8_t *state);
void speed_pid_reset(uint8_t *state);


/* Compute position PID torque limit from servo register values. */
void __attribute__((noinline)) position_pid(int32_t *s)
{
    uint8_t *sr = servo_regs_arr;

    /* Read torque command from registers (16-bit, potentially signed) */
    uint32_t uVar1 = (uint32_t)*(uint16_t *)(sr + SR_GOAL_SPEED_LO);
    if ((*(uint16_t *)(sr + SR_GOAL_SPEED_LO) & SPEED_GOAL_SIGN_BIT) != 0) {
        uVar1 = -(uVar1 & ~SPEED_GOAL_SIGN_BIT);
    }

    int32_t iVar2;
    if (uVar1 == 0) {
        /* Zero torque: check config bit for brake behavior */
        if ((sr[SR_CONFIG] & 8) == 0) {
            iVar2 = 0;
        } else {
            uint8_t *pc = pwm_ctrl_arr;
            iVar2 = *(int32_t *)(pc + PWM_MAX_TORQUE);
        }
    } else {
        /* Non-zero torque: take absolute value */
        if ((int32_t)uVar1 < 0) {
            uVar1 = -uVar1;
        }

        /* Apply punch divisor scaling */
        if ((sr[SR_CONFIG] & 4) == 0) {
            uint8_t *pc = pwm_ctrl_arr;
            iVar2 = (int32_t)(uVar1 * (uint32_t)*(int32_t *)(pc + PWM_PUNCH_DIV) * 10);
        } else {
            uint8_t *pc = pwm_ctrl_arr;
            iVar2 = (int32_t)(uVar1 * (uint32_t)*(int32_t *)(pc + PWM_PUNCH_DIV) * 10)
                     / *(int32_t *)(pc + PWM_PUNCH_DIV);
        }

        /* Clamp to min torque */
        {
            uint8_t *pc = pwm_ctrl_arr;
            if (iVar2 < *(int32_t *)(pc + PWM_MIN_SPEED)) {
                iVar2 = *(int32_t *)(pc + PWM_MIN_SPEED);
            }
        }
        /* Clamp to max torque */
        {
            uint8_t *pc = pwm_ctrl_arr;
            if (*(int32_t *)(pc + PWM_MAX_TORQUE) <= iVar2) {
                iVar2 = *(int32_t *)(pc + PWM_MAX_TORQUE);
            }
        }
    }

    s[4] = iVar2;   /* state + 0x10: max torque for profile */

    /* Detect torque change -> reset profile */
    if (iVar2 != s[0x0B]) {   /* state + 0x2C */
        s[0x0B] = iVar2;
        *(uint8_t *)(s + 0x0A) = 0;   /* state + 0x28: reset flag */
    }
}

/* Compute speed PID output — scale, abs, and clamp to torque range. */
void __attribute__((noinline)) speed_pid(int32_t *s)
{
    /* Read speed command from registers */
    uint32_t uVar3 = (uint32_t)*(uint16_t *)(servo_regs + SR_GOAL_SPEED_LO);
    if ((*(uint16_t *)(servo_regs + SR_GOAL_SPEED_LO) & SPEED_GOAL_SIGN_BIT) != 0) {
        uVar3 = -(uVar3 & ~SPEED_GOAL_SIGN_BIT);
    }

    /* Apply punch divisor scaling */
    if (((servo_regs)[SR_CONFIG] & 4) == 0) {
        uVar3 = uVar3 * (uint32_t)*(int32_t *)(pwm_ctrl + PWM_PUNCH_DIV) * 10;
    } else {
        uVar3 = (uint32_t)((int32_t)(uVar3 * (uint32_t)*(int32_t *)(pwm_ctrl + PWM_PUNCH_DIV) * 10)
                             / *(int32_t *)(pwm_ctrl + PWM_PUNCH_DIV));
    }

    /* Compute absolute value: abs = (x ^ (x>>31)) - (x>>31) */
    int32_t iVar2 = ((int32_t)uVar3 ^ ((int32_t)uVar3 >> 31)) - ((int32_t)uVar3 >> 31);

    uint32_t max_torque = *(uint32_t *)(pwm_ctrl + PWM_MAX_TORQUE);
    uint32_t min_torque = *(uint32_t *)(pwm_ctrl + PWM_MIN_SPEED);
    uint32_t result;

    if ((int32_t)max_torque < iVar2) {
        /* abs(speed) exceeds max_torque */
        if ((int32_t)uVar3 < 1) {
            /* Negative speed: clamp to -max_torque */
            result = -max_torque;
        } else {
            /* Positive speed: clamp to max_torque */
            result = max_torque;
            if ((int32_t)min_torque <= iVar2) {
                /* min is also satisfied, use max_torque */
            } else {
                /* Need to apply min_torque with sign */
                result = min_torque;
                if ((int32_t)uVar3 < 1) {
                    result = -min_torque;
                }
            }
        }
    } else {
        /* abs(speed) within max_torque */
        result = uVar3;
        if ((int32_t)min_torque <= iVar2 || uVar3 == 0) {
            /* Already in valid range or zero — use as-is */
        } else {
            /* Below minimum: apply min_torque with sign */
            result = min_torque;
            if ((int32_t)uVar3 < 1) {
                result = -min_torque;
            }
        }
    }

    s[4] = (int32_t)result;   /* state + 0x10 */
}

/* Scale value by 50 if config bit 2 is set, otherwise return unchanged. */
static int32_t __attribute__((noinline)) pid_scale_output(int32_t *param, int32_t val)
{
    (void)param;
    volatile uint8_t *sr = servo_regs;
    if ((sr[SR_CONFIG] & 4) != 0) {
        val = val * 25;
        val = val << 1;
        return (int32_t)(int16_t)val;
    }
    return val;
}

/* Convert signed position to 12-bit magnitude with direction. */
static uint16_t __attribute__((noinline)) position_to_12bit(uint8_t *enc, int32_t val)
{
    (void)enc;
    /* abs */
    int32_t abs_val = val ^ (val >> 31);
    abs_val = abs_val - (val >> 31);
    uint32_t result = abs_val & ENCODER_MAX_VAL;
    if (result != 0 && val < 0) {
        result = ENCODER_FULL_REV - result;
    }
    return (uint16_t)result;
}

/* ================================================================
 * Apply PID output to servo registers for reporting.
 * Computes present load, speed, position, velocity, goal, and output.
 * Uses sign-magnitude encoding for negative values.
 * ================================================================ */
/* ================================================================
 * Speed-based stall detection — original at 0x08001EAC.
 * param = timer_ctrl_arr + TMR_MOTION_BASE
 *
 * Offsets within param:
 *   +0x00 (uint16): stall counter
 *   +0x02 (uint8):  running flag
 *   +0x0A (int16):  speed snapshot (output at start of run)
 *   +0x0C (int32):  speed accumulator
 *
 * Loads from servo_regs at hex offsets:
 *   [0x23] = SR_ACCEL_LO (stall time threshold)
 *   [0x24] = SR_ACCEL_HI (output activation threshold)
 *   [0x22] = SR_MAX_SPEED (recovery threshold)
 * When abs(output) > accel_hi*10, marks motor "running".
 * While running with no error: increments counter each tick.
 *   If abs(speed_accum) > 50: motor moving, reset counter.
 *   If counter > accel_lo*10: stall detected, set error bit 5.
 * When error bit 5 active: checks recovery (output within ±max_speed*10
 *   AND speed_snapshot sign matches direction) to clear.
 * When error bit 5 active but torque_enable != 1: auto-clear.
 * ================================================================ */
void __attribute__((noinline)) speed_stall_check(uint8_t *param)
{
    /* Original uses hex offsets from servo_regs_arr:
     * [0x22] = decimal 34 = SR_MAX_SPEED
     * [0x23] = decimal 35 = SR_ACCEL_LO
     * [0x24] = decimal 36 = SR_ACCEL_HI */
    volatile uint8_t *sr = servo_regs;
    uint8_t stall_time = sr[SR_ACCEL_LO];       /* [0x23]: stall time threshold */
    sr = servo_regs;
    uint8_t output_thresh = sr[SR_ACCEL_HI];     /* [0x24]: output activation threshold */
    sr = servo_regs;
    uint8_t recovery_thresh = sr[SR_MAX_SPEED];   /* [0x22]: recovery threshold */

    /* abs(pwm output) — original calls abs() at 0x08003EAC */
    volatile uint8_t *pc = pwm_ctrl;
    int16_t output_raw = *(int16_t *)(pc + PWM_OUTPUT);
    int32_t abs_output = (int32_t)output_raw;
    if (abs_output < 0) abs_output = -abs_output;

    /* Check error_flags bit 5 (extended/stall error) */
    sr = servo_regs;
    if ((sr[SR_ERROR_FLAGS] & ALARM_EXTENDED_ERR) != 0) {
        /* Stall error active — check torque_enable */
        sr = servo_regs;
        uint8_t te = sr[SR_TORQUE_ENABLE];
        if (te != 1) {
            /* Torque disabled: auto-clear error bit 5, reset running */
            sr = servo_regs;
            sr[SR_ERROR_FLAGS] = sr[SR_ERROR_FLAGS] & ~ALARM_EXTENDED_ERR;
            param[2] = 0;  /* clear running flag */

            /* Fall through to check if output > punch_min*10 */
            goto check_output;
        }
        /* torque_enable == 1: go to check_running */
        goto check_running;
    }

    /* No stall error: check running flag */
check_running:
    if (param[2] != 0) {
        goto main_check;
    }

check_output:
    /* Check if abs(output) > output_thresh * 10 */
    {
        uint16_t thresh = (uint16_t)output_thresh * 10;
        uint16_t abs_out16 = (uint16_t)abs_output;
        if (abs_out16 <= thresh) {
            return;
        }
    }

    /* Output exceeds threshold: mark as running, snapshot speed, reset counter */
    param[2] = 1;  /* running = 1 */
    *(uint16_t *)param = 0;  /* counter = 0 */
    pc = pwm_ctrl;
    *(int16_t *)(param + 0x0A) = *(uint16_t *)(pc + PWM_OUTPUT);  /* speed snapshot */
    *(int32_t *)(param + 0x0C) = 0;  /* speed_accum = 0 */

main_check:
    /* Check if stall error is currently active */
    sr = servo_regs;
    if ((sr[SR_ERROR_FLAGS] & ALARM_EXTENDED_ERR) != 0) {
        /* Error active — check recovery conditions */
        goto recovery_check;
    }

    /* No error: increment counter */
    {
        uint16_t counter = *(uint16_t *)param;
        counter = (uint16_t)(counter + 1);
        *(uint16_t *)param = counter;

        /* Check speed accumulator — if motor is actually moving, reset */
        int32_t speed_acc = *(int32_t *)(param + 0x0C);
        if (speed_acc < 0) speed_acc = -speed_acc;
        if (speed_acc > 0x32) {
            /* Motor moving: reset counter and running flag */
            *(uint16_t *)param = 0;
            param[2] = 0;
            goto recovery_check;
        }

        /* Check stall threshold: counter > stall_time * 10 */
        uint16_t stall_thresh = (uint16_t)stall_time * 10;
        if (counter > stall_thresh) {
            /* STALL DETECTED — set error bit 5 */
            sr = servo_regs;
            sr[SR_ERROR_FLAGS] = sr[SR_ERROR_FLAGS] | ALARM_EXTENDED_ERR;
        }
    }

recovery_check:
    /* Check if output has dropped enough to clear stall error */
    {
        int32_t kd_thresh = (int32_t)recovery_thresh * 10;
        pc = pwm_ctrl;
        int16_t cur_output = *(int16_t *)(pc + PWM_OUTPUT);

        if (kd_thresh > (int32_t)cur_output) {
            /* kd_thresh > output: check positive snapshot */
            int16_t snap = *(int16_t *)(param + 0x0A);
            if (snap > 0) {
                goto clear_stall;
            }
        }

        /* Check negative direction */
        int32_t neg_thresh = -kd_thresh;
        if ((int32_t)cur_output <= neg_thresh) {
            return;
        }
        {
            int16_t snap = *(int16_t *)(param + 0x0A);
            if (snap >= 0) {
                return;
            }
        }

clear_stall:
        param[2] = 0;  /* clear running */
        sr = servo_regs;
        sr[SR_ERROR_FLAGS] = sr[SR_ERROR_FLAGS] & ~ALARM_EXTENDED_ERR;  /* clear bit 5 */
    }
}

void __attribute__((noinline)) pid_output_apply(uint8_t *param)
{
    int32_t *p = (int32_t *)param;

    /* === Present Load (servo_regs_arr[0x3A]) === */
    /* Check sign by calling pid_scale_output */
    volatile int32_t check = pid_scale_output(p, (int32_t)(int16_t)p[1]);
    uint16_t load;
    if (check >= 0) {
        /* Positive: call again, just truncate */
        volatile int32_t val = pid_scale_output(p, (int32_t)(int16_t)p[1]);
        load = (uint16_t)val;
    } else {
        /* Negative: call again, apply sign-magnitude */
        volatile int32_t val = pid_scale_output(p, (int32_t)(int16_t)p[1]);
        int32_t neg = -val;
        uint32_t tmp = ~((uint32_t)neg << 17);
        tmp = ~(tmp >> 17);
        load = (uint16_t)tmp;
    }
    *(uint16_t *)(servo_regs_arr + SR_PRESENT_LOAD_LO) = load;

    /* === Present Speed (servo_regs_arr[0x3C]) === */
    int16_t spd_raw = *(int16_t *)((uint8_t *)p + 6);
    uint16_t speed;
    if (spd_raw < 0) {
        speed = ((uint16_t)(-spd_raw)) | PWM_GOAL_SIGN_BIT;
    } else {
        speed = (uint16_t)spd_raw;
    }
    *(uint16_t *)(servo_regs_arr + SR_PRESENT_SPD_LO) = speed;

    /* Initialize punch divisor if zero */
    if (servo_regs_arr[SR_PUNCH_DIVISOR] == 0) {
        servo_regs_arr[SR_PUNCH_DIVISOR] = 1;
    }

    /* === Present Position (servo_regs_arr[0x38]) === */
    uint8_t mode = servo_regs_arr[SR_OPERATING_MODE];
    if (mode == 3) {
        /* Multi-turn mode: (p[0] - p[2]) / divisor */
        int32_t diff = p[0] - p[2];
        int32_t val = diff / (int32_t)(uint32_t)servo_regs_arr[SR_PUNCH_DIVISOR];
        if (val >= (int32_t)0x7FFE) val = (int32_t)0x7FFE;
        int32_t min_val = (int32_t)0xFFFF8002;
        if (val < min_val) val = min_val;
        if (val < 0) {
            uint32_t uval = (uint32_t)(-val);
            uval = ~(uval << 17);
            uval = ~(uval >> 17);
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_POS_LO) = (uint16_t)uval;
        } else {
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_POS_LO) = (uint16_t)val;
        }
    } else {
        uint8_t config = servo_regs_arr[SR_CONFIG];
        if ((config & 0x10) == 0) {
            /* 12-bit position mode */
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_POS_LO) = position_to_12bit(encoder_ctrl_arr, p[2]);
        } else {
            /* Extended mode: p[2] / divisor with sign-magnitude */
            int32_t val2 = p[2];
            /* Check sign first via initial divide, then divide again */
            volatile int32_t quotient = val2 / (int32_t)(uint32_t)servo_regs_arr[SR_PUNCH_DIVISOR];
            int32_t divided = quotient;
            if (divided < 0) {
                uint32_t uval = (uint32_t)(-divided);
                uval = ~(uval << 17);
                uval = ~(uval >> 17);
                *(uint16_t *)(servo_regs_arr + SR_PRESENT_POS_LO) = (uint16_t)uval;
            } else {
                *(uint16_t *)(servo_regs_arr + SR_PRESENT_POS_LO) = (uint16_t)divided;
            }
        }
    }

    /* === Present Velocity (servo_regs_arr[0x45]) === */
    {
        int32_t vel_val = (int32_t)(int16_t)p[5];
        if (vel_val < 0) {
            uint32_t uval = (uint32_t)(-vel_val);
            uval = ~(uval << 17);
            uval = ~(uval >> 17);
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_VEL_LO) = (uint16_t)uval;
        } else {
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_VEL_LO) = (uint16_t)vel_val;
        }
    }

    /* === Present Goal (servo_regs_arr[0x43]) === */
    mode = servo_regs_arr[SR_OPERATING_MODE];
    if (mode == 3) {
        /* Multi-turn mode: 3 separate divides for clamp checks. */
        int32_t raw = p[3];
        int32_t div1 = raw / (int32_t)(uint32_t)servo_regs_arr[SR_PUNCH_DIVISOR];
        int32_t min_val = (int32_t)0xFFFF8002;
        if (div1 < min_val) {
            uint32_t uval = (uint32_t)(-min_val);
            uval = ~(uval << 17);
            uval = ~(uval >> 17);
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = (uint16_t)uval;
        } else {
            int32_t div2 = raw / (int32_t)(uint32_t)servo_regs_arr[SR_PUNCH_DIVISOR];
            if (div2 > (int32_t)0x7FFE) {
                *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = 0x7FFE;
            } else {
                int32_t div3 = raw / (int32_t)(uint32_t)servo_regs_arr[SR_PUNCH_DIVISOR];
                if (div3 < 0) {
                    uint32_t uval = (uint32_t)(-div3);
                    uval = ~(uval << 17);
                    uval = ~(uval >> 17);
                    *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = (uint16_t)uval;
                } else {
                    *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = (uint16_t)div3;
                }
            }
        }
    } else {
        uint8_t config = servo_regs_arr[SR_CONFIG];
        if ((config & 0x10) == 0) {
            /* 12-bit mode */
            *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = position_to_12bit(encoder_ctrl_arr, p[3]);
        } else {
            /* Extended mode: p[3] / divisor with sign-magnitude */
            int32_t val = p[3];
            volatile int32_t quotient = val / (int32_t)(uint32_t)servo_regs_arr[SR_PUNCH_DIVISOR];
            int32_t divided = quotient;
            if (divided < 0) {
                uint32_t uval = (uint32_t)(-divided);
                uval = ~(uval << 17);
                uval = ~(uval >> 17);
                *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = (uint16_t)uval;
            } else {
                *(uint16_t *)(servo_regs_arr + SR_PRESENT_GOAL_LO) = (uint16_t)divided;
            }
        }
    }

    /* === Present Output (servo_regs_arr[0x47]) === */
    volatile int32_t out_check = pid_scale_output(p, (int32_t)(int16_t)p[4]);
    uint16_t out;
    if (out_check >= 0) {
        volatile int32_t oval = pid_scale_output(p, (int32_t)(int16_t)p[4]);
        out = (uint16_t)oval;
    } else {
        volatile int32_t oval = pid_scale_output(p, (int32_t)(int16_t)p[4]);
        int32_t neg = -oval;
        uint32_t tmp = ~((uint32_t)neg << 17);
        tmp = ~(tmp >> 17);
        out = (uint16_t)tmp;
    }
    *(uint16_t *)(servo_regs_arr + SR_PRESENT_OUT_LO) = out;
}


/* ================================================================
 * Position motion profile guard (original at 0x08003178).
 *
 * Only runs if torque enabled. Computes 64-bit delta from target,
 * dispatches to accel or trapezoidal profile, then:
 *   - Writes PWM_GOAL_POS from integrator (64-bit divide)
 *   - Writes PWM_SPEED_PROFILE from s[3]
 *   - Sets moving flag based on delta and speed
 * ================================================================ */
void __attribute__((noinline)) position_motion_guard(uint8_t *state)
{
    volatile uint8_t *sr = servo_regs;
    if (sr[SR_TORQUE_ENABLE] != 1) return;

    int32_t *s = (int32_t *)state;

    /* Compute 64-bit delta = goal - integrator */
    int32_t delta_lo = s[8] - s[6];
    int32_t delta_hi = s[9] - s[7] - (uint32_t)(s[8] < (uint32_t)s[6]);

    /* Dispatch to profile based on speed limit */
    if (s[2] == 0) {
        motion_profile_accel(s, 0, delta_lo, delta_hi);
    } else {
        motion_profile_update(s, 0, delta_lo, delta_hi);
    }

    /* Compute output position from integrator (64-bit divide) */
    int32_t punch_div = *(int32_t *)(pwm_ctrl_arr + PWM_PUNCH_DIV);
    int32_t scale = (punch_div * 10000) / punch_div;

    int64_t integrator = ((int64_t)s[7] << 32) | (uint32_t)s[6];
    int32_t position = (int32_t)(integrator / (int64_t)scale);
    *(int32_t *)(pwm_ctrl_arr + PWM_GOAL_POS) = position;

    /* Compute velocity from speed and punch divisor */
    *(int32_t *)(pwm_ctrl_arr + PWM_SPEED_PROFILE) = s[3] / (punch_div * 10);

    /* Set moving flag based on motion state */
    if (delta_lo == 0 && delta_hi == 0) {
        /* At target: flag depends on whether speed is zero */
        *(uint8_t *)(s + 1) = (s[3] != 0) ? 1 : 0;
        return;
    }
    if (s[4] == 0 && s[3] == 0) {
        *(uint8_t *)(s + 1) = 0;
        return;
    }
    *(uint8_t *)(s + 1) = 1;
}

/* ================================================================
 * Speed motion profile guard (original at 0x0800325C).
 *
 * Only runs if torque enabled. Simpler than position version:
 *   - Accelerates s[3] toward s[4] (speed target)
 *   - Writes PWM_SPEED_PROFILE from s[3]
 *   - Does NOT compute PWM_GOAL_POS (no integrator)
 * ================================================================ */
void __attribute__((noinline)) speed_motion_guard(uint8_t *state)
{
    volatile uint8_t *sr = servo_regs;
    if (sr[SR_TORQUE_ENABLE] != 1) return;

    int32_t *s = (int32_t *)state;
    int32_t accel = s[2];

    if (accel == 0) {
        /* No acceleration limit — snap to target speed */
        s[3] = s[4];
    } else {
        /* Accelerate s[3] toward s[4] */
        int32_t target = s[4];
        int32_t delta = target - s[3];
        if (delta > 0) {
            s[3] += accel;
            if (s[3] >= target) s[3] = target;
        }
        if (delta < 0) {
            s[3] -= accel;
            if (s[3] <= target) s[3] = target;
        }
    }

    /* Write PWM_SPEED_PROFILE only */
    int32_t punch_div = *(int32_t *)(pwm_ctrl_arr + PWM_PUNCH_DIV);
    *(int32_t *)(pwm_ctrl_arr + PWM_SPEED_PROFILE) = s[3] / (punch_div * 10);
}

/* Reset position PID state — copy encoder position, zero duty/speed/profile. */
void __attribute__((noinline)) position_pid_reset(uint8_t *state)
{
    volatile uint8_t *pc = pwm_ctrl;
    uint32_t pos = *(uint32_t *)(pc + PWM_ENCODER_POS);
    *(uint32_t *)pc = pos;
    *(uint32_t *)(pc + PWM_GOAL_POS) = pos;
    int32_t divisor = *(int32_t *)(pc + PWM_PUNCH_DIV);
    uint32_t scale = (uint32_t)((divisor * 10000) / divisor);
    /* 64-bit signed multiply: scale * pos */
    uint32_t lo, hi;
    __asm__ volatile ("umull %0, %1, %2, %3"
        : "=r"(lo), "=r"(hi) : "r"(scale), "r"(pos));
    hi += scale * ((int32_t)pos >> 31) + pos * ((int32_t)scale >> 31);
    *(uint32_t *)(state + PID_INTEG_64) = lo;
    *(uint32_t *)(state + PID_INTEG_64 + 4) = hi;
    *(uint32_t *)(state + PID_GOAL_64) = lo;
    *(uint32_t *)(state + PID_GOAL_64 + 4) = hi;
    pc = pwm_ctrl;
    *(uint16_t *)(pc + PWM_OUTPUT) = 0;
    *(uint32_t *)(pc + PWM_SPEED_PROFILE) = 0;
    *(uint32_t *)(state + 0x0C) = 0;
    state[PID_PROFILE_RESET] = 0;
    state[4] = 0;
}

/* Reset speed PID state — zero output and speed accumulator. */
void __attribute__((noinline)) speed_pid_reset(uint8_t *state)
{
    *(uint32_t *)(state + 0x0C) = 0;
    volatile uint8_t *pc = pwm_ctrl;
    *(uint32_t *)(pc + PWM_SPEED_PROFILE) = 0;
    *(uint16_t *)(pc + PWM_OUTPUT) = 0;
    volatile uint8_t *ei = encoder_i2c;
    *(uint32_t *)(ei + EI2C_SPEED_ACCUM) = 0;
}

/* Dispatch motor output based on operating mode. */
void __attribute__((noinline)) motor_output_apply(uint8_t *adc_st)
{
    (void)adc_st;

    uint8_t mode = servo_regs_arr[SR_OPERATING_MODE];
    if (mode != 0) {
        if (mode == 3) goto position_mode;
        if (mode == 1) goto speed_mode;
        if (mode == 4) goto current_mode;
        /* PWM mode (mode 2): output is handled by motor_safety_apply
         * in Step 3 (calls pwm_mode_compute + pwm_apply_output).
         * TODO: PWM mode doesn't drive motor — needs investigation. */
        speed_stall_check(timer_ctrl_arr + TMR_MOTION_BASE);
        motor_safety_halt();
        return;
    }
position_mode:
    /* Position or multi-turn mode */
    position_motion_guard(pid_state_arr);
    pid_full_compute(pid_state_arr + PID_POS_BASE);
    speed_stall_check(timer_ctrl_arr + TMR_MOTION_BASE);
    motor_safety_halt();
    {
        uint8_t torque_enable = servo_regs_arr[SR_TORQUE_ENABLE];
        if (torque_enable != 1) {
            position_pid_reset(pid_state_arr);
        }
        /* pwm_apply_output: r2=torque_enable controls PA6 (H-bridge enable).
         * torque_enable=1 → PA6 HIGH → motor driver ON.
         * torque_enable=0 → PA6 LOW → motor driver OFF. */
        pwm_apply_output(timer_ctrl_arr,
                         (int32_t)*(int16_t *)(pwm_ctrl_arr + PWM_OUTPUT),
                         (int32_t)torque_enable, 0);
    }
    return;

speed_mode:
    /* Speed mode */
    speed_motion_guard(pid_state_arr);
    pid_speed_compute(encoder_i2c_arr + EI2C_SPEED_STATE);
    speed_stall_check(timer_ctrl_arr + TMR_MOTION_BASE);
    motor_safety_halt();
    {
        uint8_t torque_enable = servo_regs_arr[SR_TORQUE_ENABLE];
        if (torque_enable != 1) {
            speed_pid_reset(pid_state_arr);
        }
        pwm_apply_output(timer_ctrl_arr,
                         (int32_t)*(int16_t *)(pwm_ctrl_arr + PWM_OUTPUT),
                         (int32_t)torque_enable, 0);
    }
    return;

current_mode:
    /* Current (torque) control mode — closes the loop on motor current.
     * Reuses the speed PID state area (EI2C_SPEED_STATE) since current
     * and speed modes are mutually exclusive. */
    pid_current_compute(encoder_i2c_arr + EI2C_SPEED_STATE);
    speed_stall_check(timer_ctrl_arr + TMR_MOTION_BASE);
    motor_safety_halt();
    {
        uint8_t torque_enable = servo_regs_arr[SR_TORQUE_ENABLE];
        if (torque_enable != 1) {
            speed_pid_reset(pid_state_arr);
        }
        pwm_apply_output(timer_ctrl_arr,
                         (int32_t)*(int16_t *)(pwm_ctrl_arr + PWM_OUTPUT),
                         (int32_t)torque_enable, 0);
    }
}

/* ================================================================
 * Speed PID sub-functions.
 * State layout: [0]=error, [1]=I-term integrator, [2]=prev error
 * ================================================================ */

/* Speed error = target_speed - actual_speed. */
static void __attribute__((noinline)) pid_spd_error(int32_t *s)
{
    volatile uint8_t *pc = pwm_ctrl;
    int32_t target = (int32_t)*(int16_t *)(pc + PWM_CURRENT_SPEED);
    int32_t actual = *(int32_t *)(pc + PWM_SPEED_PROFILE);
    s[0] = target - actual;
}

/* P-term = error * Ki_speed / 10. */
static int32_t __attribute__((noinline)) pid_spd_pterm(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t kp = (int32_t)(uint32_t)sr[SR_KI_SPEED];
    int32_t val = s[0] * kp;
    /* Divide by 10 using magic multiply */
    int32_t result = (int32_t)(((int64_t)0x66666667 * (int64_t)val) >> 34)
                     - (val >> 31);
    return result;
}

/* I-term integrate: if Ki != 0, integrator += error - prev_error. */
static void __attribute__((noinline)) pid_spd_iterm_integrate(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t ki = (int32_t)(uint32_t)sr[SR_OVERLOAD_RATIO];
    if (ki == 0) {
        s[1] = 0;
        return;
    }
    int32_t delta = s[0] - s[2];
    s[1] += delta;
}

/* I-term output = integrator * Ki / 1000. */
static int32_t __attribute__((noinline)) pid_spd_iterm(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t ki = (int32_t)(uint32_t)sr[SR_OVERLOAD_RATIO];
    int32_t val = s[1] * ki;
    /* Divide by 1000: (x * 0x10624dd3) >> 38 with sign correction */
    int32_t result = (int32_t)(((int64_t)0x10624dd3 * (int64_t)val) >> 38)
                     - (val >> 31);
    return result;
}

/* Speed PID compute — main dispatcher. */
void __attribute__((noinline)) pid_speed_compute(uint8_t *param)
{
    int32_t *s = (int32_t *)param;

    volatile uint8_t *sr = servo_regs;
    if (sr[SR_TORQUE_ENABLE] != 1) {
        /* Not running: zero PID state */
        s[1] = 0;  /* param + 4 */
        s[2] = 0;  /* param + 8 */
        return;
    }

    pid_spd_error(s);
    pid_spd_iterm_integrate(s);
    int32_t p = pid_spd_pterm(s);
    int32_t i = pid_spd_iterm(s);

    int32_t output = p + i;

    /* Add positive/negative bias from servo_regs_arr[0x18] */
    sr = servo_regs;
    if (output > 0) {
        output += (int32_t)(uint32_t)sr[SR_PUNCH_MIN];
    }
    sr = servo_regs;
    if (output < 0) {
        output -= (int32_t)(uint32_t)sr[SR_PUNCH_MIN];
    }

    /* Store prev error for next iteration */
    s[2] = s[0];

    /* Clamp to max torque */
    sr = servo_regs;
    int16_t max_torque = *(int16_t *)(sr + SR_MAX_OUTPUT_LO);
    int16_t neg_max = -max_torque;
    int16_t result;

    if (output < (int32_t)neg_max) {
        result = neg_max;
    } else if (output > (int32_t)max_torque) {
        result = max_torque;
    } else {
        result = (int16_t)output;
    }

    volatile uint8_t *pc = pwm_ctrl;
    *(int16_t *)(pc + PWM_OUTPUT) = result;
    s[2] = output - (int32_t)result;  /* integrator anti-windup */
}

/* ================================================================
 * Current (torque) PID — mode 4.
 *
 * Closes the loop on motor current measured by the ADC shunt resistor.
 * Goal current comes from servo_regs[44-45] (PWM goal registers, reused
 * as current goal in sign-magnitude format, same as PWM mode).
 * Feedback is PWM_CURRENT_SENSE (ADC delta from baseline).
 *
 * Uses PI control (same structure as speed PID):
 *   P = error * Kp / 10        (Kp from SR_KI_SPEED register 37)
 *   I = integrator * Ki / 1000 (Ki from SR_OVERLOAD_RATIO register 39)
 *
 * State layout (reuses speed PID state at EI2C_SPEED_STATE):
 *   s[0] = error, s[1] = I-term integrator, s[2] = prev error
 * ================================================================ */

/* Current error = goal_current - actual_current. */
static void __attribute__((noinline)) pid_cur_error(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    volatile uint8_t *pc = pwm_ctrl;

    /* Read goal from PWM goal registers (sign-magnitude, bit 10 = sign) */
    uint16_t raw = *(uint16_t *)(sr + SR_PWM_GOAL_LO);
    int32_t goal;
    if (raw & PWM_GOAL_SIGN_BIT) {
        goal = -(int32_t)(raw & ~(uint16_t)PWM_GOAL_SIGN_BIT);
    } else {
        goal = (int32_t)raw;
    }

    /* Read actual current from ADC */
    int32_t actual = (int32_t)*(int16_t *)(pc + PWM_CURRENT_SENSE);

    s[0] = goal - actual;
}

/* Current PID compute — PI controller on motor current. */
void __attribute__((noinline)) pid_current_compute(uint8_t *param)
{
    int32_t *s = (int32_t *)param;

    volatile uint8_t *sr = servo_regs;
    if (sr[SR_TORQUE_ENABLE] != 1) {
        s[1] = 0;
        s[2] = 0;
        return;
    }

    pid_cur_error(s);
    pid_spd_iterm_integrate(s);  /* Reuse speed I-term integration */
    int32_t p = pid_spd_pterm(s);  /* Reuse speed P-term (same gain registers) */
    int32_t i = pid_spd_iterm(s);  /* Reuse speed I-term (same gain registers) */

    int32_t output = p + i;

    /* Add punch bias (minimum drive force) */
    sr = servo_regs;
    if (output > 0) {
        output += (int32_t)(uint32_t)sr[SR_PUNCH_MIN];
    }
    sr = servo_regs;
    if (output < 0) {
        output -= (int32_t)(uint32_t)sr[SR_PUNCH_MIN];
    }

    s[2] = s[0];

    /* Clamp to max output */
    sr = servo_regs;
    int16_t max_torque = *(int16_t *)(sr + SR_MAX_OUTPUT_LO);
    int16_t neg_max = -max_torque;
    int16_t result;

    if (output < (int32_t)neg_max) {
        result = neg_max;
    } else if (output > (int32_t)max_torque) {
        result = max_torque;
    } else {
        result = (int16_t)output;
    }

    volatile uint8_t *pc = pwm_ctrl;
    *(int16_t *)(pc + PWM_OUTPUT) = result;
    s[2] = output - (int32_t)result;  /* Anti-windup */
}

/* Reset PID state by zeroing all motion helper fields. */
void __attribute__((noinline, section(".text.keep"))) pid_state_reset(void)
{
    motion_helper_reset(pid_state_arr);
}

/* ================================================================
 * Position PID sub-functions.
 * State: [0]=counter, [1]=error, [2]=velocity, [3]=prev error,
 *        [4]=I-term, [5]=anti-windup residual, [6]=D-term accum
 * ================================================================ */

/* Compute position error with deadband applied. */
static void __attribute__((noinline)) pid_pos_error_update(int32_t *s)
{
    volatile uint8_t *pc = pwm_ctrl;
    int32_t actual = *(int32_t *)(pc + PWM_ENCODER_POS);
    int32_t goal = *(int32_t *)(pc + PWM_GOAL_POS);
    int32_t error = actual - goal;  /* original: encoder - goal (not goal - encoder) */
    volatile uint8_t *sr = servo_regs;
    int32_t neg_dz = -(int32_t)(uint32_t)sr[SR_CCW_DEADZONE];

    if (error < neg_dz) {
        sr = servo_regs;
        error += (int32_t)(uint32_t)sr[SR_CCW_DEADZONE];
    } else {
        sr = servo_regs;
        int32_t pos_dz = (int32_t)(uint32_t)sr[SR_CW_DEADZONE];
        if (error > pos_dz) {
            sr = servo_regs;
            error -= (int32_t)(uint32_t)sr[SR_CW_DEADZONE];
        } else {
            error = 0;
        }
    }
    s[1] = error;  /* param + 4 */
}

/* Velocity tracking — increment counter, compute velocity when threshold met. */
static void __attribute__((noinline)) pid_pos_velocity_track(int32_t *s)
{
    int8_t counter = *(int8_t *)s;
    counter++;
    *(int8_t *)s = counter;

    volatile uint8_t *sr = servo_regs;
    int32_t threshold = (int32_t)(uint32_t)sr[SR_VEL_COUNTER];
    if (counter >= threshold) {
        *(int8_t *)s = 0;  /* reset counter */

        register int32_t cur_err = s[1];
        int32_t vel = cur_err - s[3];
        s[2] = vel;
        s[3] = cur_err;

        /* Apply dead-zone threshold */
        sr = servo_regs;
        int32_t dz = (int32_t)(uint32_t)sr[SR_SPEED_DEADZONE];
        int32_t abs_vel = (vel < 0) ? -vel : vel;
        if (abs_vel < dz) {
            s[2] = 0;
        }
    }
}

/* Conditional I-term integration with anti-windup. */
static void __attribute__((noinline)) pid_pos_iterm_integrate(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t ki = (int32_t)(uint32_t)sr[SR_KI_GAIN];
    if (ki == 0) {
        s[4] = 0;   /* param + 0x10 */
        return;
    }
    /* Skip integration when speed flag is set (adc_state[0] != 0).
     * Original: cbnz adc_state[0] → return (skip integration).
     * When speed flag is clear, check alarm conditions. */
    volatile uint8_t *as = adc_state;
    if (as[0] != 0) return;
    {
        sr = servo_regs;
        uint8_t alarms = sr[SR_ERROR_FLAGS];
        uint8_t mask = sr[SR_ALARM_SHUTDOWN];
        if ((alarms & mask) != 0) return;
    }
    /* Integrate: accumulator += error - anti-windup terms */
    int32_t error = s[1];
    int32_t residual = s[5];
    int32_t extra = s[6];
    int32_t prev = s[4];
    s[4] = prev + (error - residual - extra);
}

/* P-term = error * Kp / 4 (with rounding toward zero). */
static int32_t __attribute__((noinline)) pid_pos_pterm(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t kp = (int32_t)(uint32_t)sr[SR_KP_GAIN];
    int32_t result = s[1] * kp;
    if (result < 0) result += 3;  /* round toward zero */
    return result >> 2;
}

/* I-term = velocity * Kd_scale / 8 (with rounding toward zero). */
static int32_t __attribute__((noinline)) pid_pos_iterm(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t ki_scale = (int32_t)(uint32_t)sr[SR_KD_GAIN_SCALE];
    int32_t result = s[2] * ki_scale;
    if (result < 0) result += 7;  /* round toward zero */
    return result >> 3;
}

/* D-term with integrator clamping to prevent windup. */
static int32_t __attribute__((noinline)) pid_pos_dterm(int32_t *s)
{
    volatile uint8_t *sr = servo_regs;
    int32_t kd = (int32_t)(uint32_t)sr[SR_KD_GAIN];
    int32_t ki = (int32_t)(uint32_t)sr[SR_KI_GAIN];
    int32_t val = s[4] * ki;

    /* Divide by 500 using magic multiply */
    int32_t divided = (int32_t)(((int64_t)0x10624dd3 * (int64_t)val) >> 37)
                      - (val >> 31);

    int32_t clamped;
    if (kd == 0) {
        clamped = divided;
    } else {
        int32_t limit = kd * 4;
        int32_t neg_limit = -limit;
        if (divided < neg_limit) {
            clamped = neg_limit;
        } else if (limit >= divided) {
            clamped = divided;
        } else {
            clamped = limit;
        }
    }

    int32_t residual = divided - clamped;
    s[6] = residual;   /* param + 0x18 */
    return clamped;
}

/* Position PID compute — main dispatcher. */
void __attribute__((noinline)) pid_full_compute(uint8_t *param)
{
    int32_t *s = (int32_t *)param;

    if (servo_regs_arr[SR_TORQUE_ENABLE] != 1) {
        /* Not running: zero all PID state */
        s[4] = 0;   /* param + 0x10 */
        s[3] = 0;   /* param + 0x0C */
        s[5] = 0;   /* param + 0x14 */
        s[6] = 0;   /* param + 0x18 */
        return;
    }

    pid_pos_error_update(s);
    pid_pos_velocity_track(s);
    pid_pos_iterm_integrate(s);

    int32_t p = pid_pos_pterm(s);
    int32_t i = pid_pos_iterm(s);
    int32_t d = pid_pos_dterm(s);

    int32_t output = p + i + d;

    /* Add positive/negative bias from servo_regs_arr[0x18] */
    {
        volatile uint8_t *sr = servo_regs;
        if (output > 0) {
            output += (int32_t)(uint32_t)sr[SR_PUNCH_MIN];
        }
        sr = servo_regs;
        if (output < 0) {
            output -= (int32_t)(uint32_t)sr[SR_PUNCH_MIN];
        }
    }

    /* Clamp to ±max_torque from servo_regs_arr[0x30] */
    {
        volatile uint8_t *sr = servo_regs;
        int16_t max_torque = *(int16_t *)(sr + SR_MAX_OUTPUT_LO);
        int16_t neg_max = -max_torque;
        int16_t result;

        if (output < (int32_t)neg_max) {
            result = neg_max;
        } else if (output > (int32_t)max_torque) {
            result = max_torque;
        } else {
            result = (int16_t)output;
        }

        volatile uint8_t *pc = pwm_ctrl;
        *(int16_t *)(pc + PWM_OUTPUT) = result;
        s[5] = output - (int32_t)result;  /* anti-windup residual */
    }
}


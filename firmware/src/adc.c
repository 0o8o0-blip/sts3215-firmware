/* Feetech STS3215 — ADC subsystem (temperature, voltage, current) */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint16_t position_lut[32];
extern uint8_t adc_state_arr[]; extern uint8_t * volatile adc_state;
extern uint8_t i2c_ctrl_arr[]; extern uint8_t * volatile i2c_ctrl;
extern uint8_t pid_state_arr[]; extern uint8_t * volatile pid_state;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t timer_ctrl_arr[]; extern uint8_t * volatile timer_ctrl;
extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;

/* Forward declarations within this file */
uint8_t adc_linearize_temp(uint8_t *state, int32_t raw_adc);
uint8_t temp_lut_interpolate(uint8_t *prev, int32_t raw, uint32_t idx);
void adc_iir_filter(int32_t *accum, int32_t new_val);
uint32_t lut_bsearch(uint32_t value, uint16_t *lut, uint32_t max_idx);

/* Compute motor speed from encoder delta over time. */
void __attribute__((noinline)) adc_processing(uint8_t *param)
{
    if (param[1] == 0xFF) {
        /* First call — initialize */
        param[1] = 0;
        *(uint32_t *)(param + 4) = *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS);
    }

    uint8_t counter = param[1];
    param[1] = counter + 1;

    /* Check if sample period elapsed */
    int32_t punch_div = *(int32_t *)(pwm_ctrl_arr + PWM_PUNCH_DIV);
    if ((int32_t)(int8_t)(counter + 1) >= 1000 / punch_div) {
        param[1] = 0;

        /* Compute position delta */
        int16_t delta = (int16_t)*(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS) -
                        (int16_t)*(uint32_t *)(param + 4);
        int32_t speed = (int32_t)delta;
        *(uint32_t *)(param + 4) = *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS);

        /* Handle wraparound */
        if (speed < -ENCODER_HALF_REV) {
            speed = (int32_t)(int16_t)(delta + ENCODER_FULL_REV);
        } else if (speed > ENCODER_HALF_REV) {
            speed = (int32_t)(int16_t)(delta - ENCODER_FULL_REV);
        }

        /* Accumulate into speed register */
        *(int32_t *)(timer_ctrl_arr + TMR_SPEED_ACCUM) += speed;
        *(int16_t *)(pwm_ctrl_arr + PWM_CURRENT_SPEED) = (int16_t)speed;
    }

    /* Check if speed is below threshold */
    int32_t abs_speed;
    int32_t raw_speed = (int32_t)*(int16_t *)(pwm_ctrl_arr + PWM_CURRENT_SPEED);
    abs_speed = raw_speed < 0 ? -raw_speed : raw_speed;

    if (abs_speed < (int32_t)(uint32_t)servo_regs_arr[SR_SPEED_DEADZONE]) {
        *param = 0;
        *(uint16_t *)(pwm_ctrl_arr + PWM_CURRENT_SPEED) = 0;
        return;
    }
    *param = 1;
}

/* Read temperature from ADC DMA buffer, IIR-filter and linearize. */
uint8_t __attribute__((noinline)) adc_read_temp(uint8_t *param)
{
    uint8_t ch_idx = param[I2C_TEMP_CH];
    uint16_t *adc_buf = (uint16_t *)(param + I2C_DMA_BUF);
    uint16_t raw = adc_buf[ch_idx];
    int32_t val = (int32_t)(raw >> 2);

    return adc_linearize_temp(param + I2C_TEMP_FILTER, val);
}

/* Read voltage from ADC DMA buffer, return 8-bit value. */
uint8_t __attribute__((noinline)) adc_read_voltage(uint8_t *param)
{
    uint8_t ch_idx = param[I2C_VOLT_CH];
    uint16_t *adc_buf = (uint16_t *)(param + I2C_DMA_BUF);
    uint16_t raw = adc_buf[ch_idx];
    return (uint8_t)((raw << 4) >> 8);
}

/* ================================================================
 * Read motor current from ADC DMA buffer.
 * If config bit 5 is set, returns 0 (no current sensor).
 * Otherwise reads ADC channel, applies warmup baseline average,
 * then returns delta from baseline.
 * ================================================================ */
int16_t __attribute__((noinline)) adc_read_current(uint8_t *param)
{
    volatile uint8_t *sr = servo_regs;
    if ((sr[SR_CONFIG] & 0x20) != 0) {
        return 0;
    }

    uint8_t ch_idx = param[I2C_CURR_CH];
    uint16_t *adc_buf = (uint16_t *)(param + I2C_DMA_BUF);
    uint16_t raw = adc_buf[ch_idx];
    int8_t warmup = *(int8_t *)(param + I2C_WARMUP_CTR);

    if (warmup != 0) {
        /* Warmup phase: accumulate samples for baseline */
        *(int8_t *)(param + I2C_WARMUP_CTR) = warmup - 1;
        uint16_t accum = *(uint16_t *)(param + I2C_CURR_BASELINE);
        uint32_t sum = (uint32_t)(raw >> 2) + (uint32_t)accum;
        if (warmup != 1) {
            *(int16_t *)(param + I2C_CURR_BASELINE) = (int16_t)sum;
            return 0;
        }
        /* Last warmup sample: compute average (divide by 16) */
        *(int16_t *)(param + I2C_CURR_BASELINE) = (int16_t)((sum & 0xFFFF) >> 4);
        return 0;
    }

    /* Normal: return current delta from baseline */
    return (int16_t)((raw >> 2) - *(int16_t *)(param + I2C_CURR_BASELINE));
}

/* Set motor torque enable flag based on operating mode and overload detection. */
void __attribute__((noinline)) adc_overload_check(uint8_t *param)
{
    volatile uint8_t *sr = servo_regs;
    uint8_t mode = sr[SR_OPERATING_MODE];
    if (mode == 2 || mode == 1 || mode == 4 || mode == 5) {
        /* PWM, speed, current, or cascaded mode: copy overload flag directly */
        sr = servo_regs;
        sr[SR_RESERVED_42] = param[0];
        return;
    }
    /* Position/multi-turn mode */
    volatile uint8_t *ps = pid_state;
    if (ps[4] == 0) {
        /* Not moving */
        if (param[0] == 0) {
            sr = servo_regs;
            sr[SR_RESERVED_42] = 0;
            return;
        }
    } else {
        /* Moving */
        sr = servo_regs;
        sr[SR_RESERVED_42] = 1;
    }
}

/* IIR filter with alpha=31/32: result = (prev + new*31) >> 5 */
void __attribute__((noinline, section(".text.keep"))) adc_iir_filter(int32_t *accum, int32_t new_val)
{
    int32_t result = (*accum) + new_val * 31;
    result = result >> 5;
    *accum = result;
}

/* ================================================================
 * Binary search in descending uint16_t LUT.
 * Returns index where lut[index] is the closest >= value.
 * Original at 0x08003AD0.
 * ================================================================ */
uint32_t __attribute__((noinline)) lut_bsearch(uint32_t value, uint16_t *lut, uint32_t max_idx)
{
    /* Check if value >= lut[0] (highest entry) */
    if (lut[0] <= value) return 0;

    /* Check if value <= lut[max_idx-1] (lowest non-zero entry) */
    max_idx = (uint8_t)(max_idx - 1);
    if (lut[max_idx] >= value) return max_idx;

    /* Binary search */
    uint32_t lo = 0;
    uint32_t hi = max_idx;
    uint32_t mid = hi >> 1;
    uint16_t mid_val = lut[mid];

    if (value == mid_val) return mid;

    for (;;) {
        if (value > mid_val) {
            /* Descending table: value > lut[mid] means look left
             * (lower index = higher values). Only narrow on strict >
             * so equality is caught by the check below. */
            hi = (uint8_t)(mid - 1);
        }
        if (lo > hi) return hi;

        mid = (uint8_t)((lo + hi) >> 1);
        mid_val = lut[mid];
        if (value == mid_val) return mid;

        if (value >= mid_val) continue;
        lo = (uint8_t)(mid + 1);
    }
}

/* ================================================================
 * Temperature LUT interpolation — converts raw ADC thermistor value
 * to temperature in degrees C using position_lut for linearization.
 * ================================================================ */
uint8_t __attribute__((noinline)) temp_lut_interpolate(uint8_t *prev, int32_t raw, uint32_t idx)
{
    if (idx == 30) return 150;

    uint16_t entry = position_lut[idx];
    uint16_t next  = position_lut[idx + 1];
    uint8_t prev_val = prev[0];

    /* Clamp raw to entry */
    if (raw >= (int32_t)entry) raw = (int32_t)entry;

    int32_t numer, denom;
    if (raw >= (int32_t)next) {
        numer = (int32_t)entry - raw;
    } else {
        numer = (int32_t)entry - (int32_t)next;
    }

    denom = (int32_t)entry - (int32_t)next;
    int32_t result = ((int32_t)prev_val * numer) / denom;
    result += (int32_t)prev_val * (int32_t)idx;
    return (uint8_t)result;
}

/* Apply binary search + LUT interpolation to raw ADC temperature value.
 * Original uses binary search (0x08003AD0) to find LUT index,
 * then calls temp_lut_interpolate with (state, raw, index). */
uint8_t __attribute__((noinline)) adc_linearize_temp(uint8_t *state, int32_t raw_adc)
{
    uint32_t idx = lut_bsearch((uint32_t)raw_adc, position_lut, 31);
    return temp_lut_interpolate(state, raw_adc, idx);
}


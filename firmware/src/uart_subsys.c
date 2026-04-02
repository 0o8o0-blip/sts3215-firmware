/* Feetech STS3215 — UART subsystem (ring buffer, DMA TX, protocol helpers) */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>
#include <string.h>

/* Global state arrays (defined in main.c) */
extern uint8_t adc_state_arr[]; extern uint8_t * volatile adc_state;
extern uint8_t eeprom_ctrl_arr[]; extern uint8_t * volatile eeprom_ctrl;
extern uint8_t encoder_ctrl_arr[]; extern uint8_t * volatile encoder_ctrl;
extern uint8_t motor_ctrl_arr[]; extern uint8_t * volatile motor_ctrl;
extern uint8_t pid_state_arr[]; extern uint8_t * volatile pid_state;
extern uint8_t pwm_ctrl_arr[]; extern uint8_t * volatile pwm_ctrl;
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t uart_state_arr[]; extern uint8_t * volatile uart_state;

/* Vtable arrays (defined in vtables.c) */
extern const void *motor_calibrate_vtable[];
extern const void *motor_ctrl_vtable[];
extern const void *ring_buf_vtable[];
extern const void *uart_obj_vtable[];
extern const void *uart_state_vtable[];

/* Functions from other files */
extern int reg_write(uint32_t addr, uint8_t *data, uint8_t len);
extern int32_t encoder_set_initial(int32_t *param, int32_t revolutions);
extern int32_t position_delinearize(uint8_t *param, int32_t val);
extern uint32_t clamp_baud_index(uint32_t unused, uint32_t idx);
extern uint32_t uart_packet_parse(uint8_t *state);
extern void buf_to_regs(uint8_t *dst, uint8_t *src, int32_t offset, int32_t count);
extern void eeprom_save_byte_internal(uint8_t *ee_ctrl);
extern void eeprom_save_regs(uint8_t *param);
extern void position_profile_init(uint8_t *s, int32_t speed, int32_t accel);
extern void timer0_set_pwm(uint8_t *state, uint32_t forward, uint32_t reverse);
extern void uart_dispatch(uint8_t *state);
extern void subsys_memset(uint8_t *dst, uint8_t val, int count);
extern void subsys_memcpy(void *dst, const void *src, int count);
extern uint32_t baud_tx_time(uint32_t unused, uint32_t idx);
extern int32_t servo_apply_config(uint8_t *param, int32_t addr);
extern void encoder_update_position(int32_t *param);
extern void usart1_dma_init(uint8_t *uart_st, uint32_t baud_idx);
extern void reg_write_side_effects(uint8_t *param, uint8_t *regs, int32_t start_addr, int32_t count);
extern void usart1_baud_init(uint8_t *param);
extern void uart_send_response(uint8_t *state);
extern void regs_to_buf(uint8_t *src, uint8_t *dst, int32_t offset, int32_t count);
extern uint32_t reg_change_detect(uint8_t *param, int32_t addr, int32_t len, uint8_t *changes);
extern void pid_state_reset(void);
extern void motion_rate_limit(int32_t *state, int32_t target, int32_t accel, int32_t max_val);
extern void led_blink_tick(uint8_t *param);
extern void gpio_led_init(void);

extern uint8_t encoder_i2c_arr[]; extern uint8_t * volatile encoder_i2c;
extern uint8_t gpio_ctrl_arr[]; extern uint8_t * volatile gpio_ctrl;
extern uint8_t timer_ctrl_arr[]; extern uint8_t * volatile timer_ctrl;
extern uint8_t i2c_ctrl_arr[]; extern uint8_t * volatile i2c_ctrl;
extern uint8_t reg_permissions[88];

/* Baud rate table (defined in util.c) */
struct baud_entry { uint32_t baud; uint32_t bit_time; };
extern const struct baud_entry baud_table_ext[8];

/* Forward declarations within this file */
void baud_rate_change(uint8_t *param, uint32_t baud_idx);
void usart_idle_handler(uint8_t *param);
void uart_check_dispatch(uint8_t *s);
void uart_state_init(uint32_t *s);
void uart_tx_start_if_ready(uint8_t *s);
void uart_tx_force_start(uint8_t *s);
void dma_tx_start(uint8_t *s, uint32_t addr, uint32_t len);
void ring_buf_reset(uint8_t *ring);

/* ================================================================
 * ring buffer reset
 *
 * Resets read and write indices to 0.
 * ================================================================ */
void __attribute__((noinline)) ring_buf_reset(uint8_t *ring)
{
    *(uint16_t *)(ring + 4) = 0;
    *(uint16_t *)(ring + 6) = 0;
}

/* ================================================================
 * ring buffer init
 *
 * Zero-initializes the ring buffer structure.
 * ================================================================ */
void __attribute__((noinline, section(".text.keep"))) ring_buf_init(uint8_t *ring)
{
    uint16_t *p = (uint16_t *)ring;
    p[0] = 0;
    ((uint8_t *)ring)[2] = 0;
    p[2] = 0;
    p[3] = 0;
    p[4] = 0;
    p[5] = 0;
    *(uint32_t *)(ring + 12) = 0;
}

/* ================================================================
 * ring buffer empty check
 *
 * Updates the write index from DMA remaining count, then checks
 * if read == write (empty). Returns 1 if empty, 0 if data available.
 * ================================================================ */
uint32_t __attribute__((noinline)) ring_buf_empty(uint8_t *ring)
{
    /* Update write index from DMA channel remaining count:
     * wr_idx = buf_size - DMA_CH_CNT */
    volatile uint32_t *dma_ch4 = (volatile uint32_t *)0x40020058;
    *(int16_t *)(ring + 10) = 0x80 - (int16_t)(dma_ch4[1]);
    return (*(int16_t *)(ring + 8) == *(int16_t *)(ring + 10)) ? 1 : 0;
}

/* ================================================================
 * ring buffer object init
 *
 * Initializes a ring buffer with pointer to internal buffer,
 * size 0x80, mask 0x7F.
 * ================================================================ */
void __attribute__((noinline)) ring_buf_obj_init(uint32_t *param)
{
    extern const void *ring_buf_vtable[];
    *param = (uint32_t)ring_buf_vtable;  /* vtable — original loads from literal pool */
    ring_buf_reset((uint8_t *)(param + 1));
    param[1] = (uint32_t)(param + 4);   /* buffer pointer = internal buffer */
    *(uint16_t *)(param + 3) = 0x80;    /* size */
    *(uint16_t *)((uint8_t *)param + 0x0E) = 0x7F;  /* mask */
    *(uint8_t *)(param + 0x44) = 0;     /* tx_len = 0 */
}

/* ================================================================
 * USART1 DMA + baud init + register setup
 *
 * Called during pwm_init. Initializes runtime register state from
 * EEPROM-loaded servo_regs_arr, sets up delinearized position, speed
 * profile, and baud rate clamping.
 * ================================================================ */
void __attribute__((noinline)) usart1_baud_init(uint8_t *param)
{
    volatile uint8_t * volatile sr = servo_regs;

    /* Clear runtime state bytes — matches factory init at 0x0B48 */
    sr[SR_TORQUE_ENABLE] = 0;
    sr[SR_ACCELERATION] = 0;
    *(volatile uint16_t *)(sr + SR_GOAL_POS_LO) = 0;
    *(volatile uint16_t *)(sr + SR_GOAL_SPEED_LO) = 0;
    *(volatile uint16_t *)(sr + SR_MAX_OUTPUT_LO) = *(volatile uint16_t *)(sr + SR_MAX_TORQUE_LO);
    sr[SR_LOCK] = 1;
    sr[SR_ACTION_PENDING] = 0;
    sr[SR_ERROR_FLAGS] = 0;
    sr[SR_RESERVED_42] = 0;
    /* Original reads 0x43 then writes 0 (dead read, compiler artifact) */
    (void)sr[SR_PRESENT_GOAL_LO]; sr[SR_PRESENT_GOAL_LO] = 0;
    (void)sr[SR_PRESENT_GOAL_HI]; sr[SR_PRESENT_GOAL_HI] = 0;

    /* Copy operating mode to param */
    param[PWM_OP_MODE] = sr[SR_OPERATING_MODE];

    /* Set punch divisor from servo_regs_arr[SR_PUNCH_DIVISOR2] */
    if (sr[SR_PUNCH_DIVISOR2] == 0) {
        *(uint32_t *)(param + PWM_PUNCH_DIV) = 1;
    } else {
        *(uint32_t *)(param + PWM_PUNCH_DIV) = (uint32_t)sr[SR_PUNCH_DIVISOR2];
    }

    /* Reload servo_regs_arr after branch (original uses r4 from literal pool) */
    volatile uint8_t * volatile sr2 = servo_regs;

    /* Compute max torque and punch values */
    int32_t punch_div = *(int32_t *)(param + PWM_PUNCH_DIV);
    *(uint32_t *)(param + PWM_MIN_SPEED) = (uint32_t)sr2[SR_TORQUE_FACTOR] * punch_div * 10;
    *(int32_t *)(param + 0x28) = (int32_t)((uint32_t)sr2[SR_PUNCH_VALUE] * punch_div * 500) / punch_div;

    /* Delinearize initial goal position and store in encoder_ctrl_arr */
    uint32_t delinear = position_delinearize(param, (int32_t)*(int16_t *)(sr2 + SR_GOAL_OFFSET_LO));
    *(uint32_t *)(encoder_ctrl_arr + ENC_DELINEAR) = delinear;

    /* Initialize speed profile */
    position_profile_init(pid_state_arr, sr2[SR_ACCELERATION], *(uint16_t *)(sr2 + SR_GOAL_SPEED_LO));

    /* Clamp baud rate index */
    uint8_t baud = clamp_baud_index(0, sr2[SR_SERVO_ID]);
    sr2[SR_SERVO_ID] = baud;
}

/* ================================================================
 * uart TX enable
 *
 * Disables USART1 receiver, enables transmitter. Sets TX init flag.
 * ================================================================ */
void __attribute__((noinline)) uart_tx_enable(uint8_t *param)
{
    volatile uint32_t * volatile usart = (volatile uint32_t *)USART1_BASE;
    *usart = *usart & ~0x08U;   /* Clear TE (bit 3) */
    *usart = *usart | 0x04U;    /* Set RE (bit 2) */
    param[UART_SYNC_ID] = 1;
}

/* ================================================================
 * uart obj init
 *
 * Initializes UART object with ring buffer and vtable.
 * ================================================================ */
uint32_t * __attribute__((noinline)) uart_obj_init(uint32_t *param)
{
    ring_buf_obj_init(param);
    extern const void *uart_obj_vtable[];
    *param = (uint32_t)uart_obj_vtable;  /* vtable — original loads from literal pool */
    *(uint8_t *)((uint8_t *)param + UART_ERROR) = 0;
    return param;
}

/* ================================================================
 * uart TX mode switch — vtable[2] for uart_obj_vtable
 *
 * Switches USART from RX to TX mode and clears SYNC_ID.
 * Called from uart_tx_start_if_ready via vtable dispatch.
 * Original at 0x08000D4D: bic #4 (clear RE), orr #8 (set TE),
 * strb #0 to SYNC_ID.
 * ================================================================ */
void __attribute__((noinline)) uart_tx_mode_switch(uint8_t *param)
{
    volatile uint32_t * volatile usart = (volatile uint32_t *)USART1_BASE;
    *usart = *usart & ~0x04U;   /* Clear RE (bit 2) */
    *usart = *usart | 0x08U;    /* Set TE (bit 3) */
    param[UART_SYNC_ID] = 0;
}

/* ================================================================
 * uart TX dispatch (baud rate change after error response)
 *
 * If error flag is set and UART is initialized, clear error
 * and apply baud rate change. Called from main loop context.
 * ================================================================ */
void __attribute__((noinline)) uart_tx_dispatch(uint8_t *param)
{
    if (param[UART_ERROR] != 0 && param[UART_SYNC_ID] != 0) {
        param[UART_ERROR] = 0;
        baud_rate_change(param, servo_regs_arr[SR_SERVO_ID]);
    }
}

/* ================================================================
 * USART1_IRQHandler — USART1 IRQ
 *
 * Handles TC (transmission complete), ORE (overrun), and IDLE
 * line detection interrupts.
 * ================================================================ */
void USART1_IRQHandler(void)
{
    volatile uint32_t *usart = (volatile uint32_t *)USART1_BASE;

    /* Check TC (transmission complete) + TCIE */
    if ((usart[7] & 0x40) != 0 && (usart[0] & 0x40) != 0) {
        /* Clear TC flag */
        usart[0] &= ~0x40U;
        usart[8] = 0x40;   /* ICR: clear TC */
        uart_tx_enable(uart_state_arr);
    }

    /* Check ORE (overrun error) */
    if ((usart[7] & 8) != 0) {
        usart[8] = 8;   /* ICR: clear ORE */
    }

    /* Check IDLE line */
    if ((usart[7] & 0x10) != 0) {
        usart[8] = 0x10;   /* ICR: clear IDLE */
        usart_idle_handler(motor_ctrl_arr);
    }
}

/* ================================================================
 * timer setup for TX
 *
 * Configures TIMER15 for TX timing based on baud rate.
 * Sets period, prescaler, enables one-shot mode.
 * ================================================================ */
void __attribute__((noinline)) timer_setup_tx(uint8_t *param, int32_t bit_count, int16_t prescaler)
{
    (void)param;
    volatile uint8_t * volatile tb = (volatile uint8_t *)TIMER15_BASE;

    /* Clear UIE in DMAINTEN (offset 0x0C) */
    {
        uint16_t v = *(volatile uint16_t *)(tb + 0x0C);
        v &= ~1U;
        *(volatile uint16_t *)(tb + 0x0C) = v;
    }

    /* Set prescaler (offset 0x28) */
    *(volatile uint16_t *)(tb + 0x28) = (uint16_t)(prescaler - 1);

    /* Set period CAR (offset 0x2C) — 32-bit write */
    *(volatile uint32_t *)(tb + 0x2C) = bit_count - 1;

    /* Generate update event: SWEVG |= UG (offset 0x14) */
    {
        uint16_t v = *(volatile uint16_t *)(tb + 0x14);
        v |= 1;
        *(volatile uint16_t *)(tb + 0x14) = v;
    }

    /* NOPs matching original (3 NOPs for pipeline sync) */
    __asm volatile ("nop");
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* Clear INTF (offset 0x10) */
    *(volatile uint16_t *)(tb + 0x10) = 0;

    /* Enable CEN: CTL0 |= 1 (offset 0x00) */
    {
        uint16_t v = *(volatile uint16_t *)(tb + 0x00);
        v |= 1;
        *(volatile uint16_t *)(tb + 0x00) = v;
    }

    /* Enable UIE: DMAINTEN = 1 (offset 0x0C) */
    *(volatile uint16_t *)(tb + 0x0C) = 1;
}

/* ================================================================
 * TX byte count calc
 *
 * Calculates the total bit-time for the TX response packet.
 * Returns (tx_len + 3) * baud_bit_time.
 * ================================================================ */
int16_t __attribute__((noinline)) tx_byte_count_calc(uint8_t *param)
{
    volatile uint8_t *sr = servo_regs;
    int16_t bit_time = baud_tx_time(0, sr[SR_SERVO_ID]);
    return (param[UART_TX_STATE] + 3) * bit_time;
}

/* ================================================================
 * UART TX process
 *
 * Checks if UART is initialized, then triggers check+dispatch.
 * Disables USART1 IRQ (bit 28) around dispatch, then re-enables.
 * ================================================================ */
void __attribute__((noinline)) uart_tx_process(uint8_t *param)
{
    if (uart_state_arr[UART_SYNC_ID] != 0) {
        volatile uint32_t * volatile nvic = (volatile uint32_t *)0xE000E100U;
        *(volatile uint32_t *)((uint8_t *)nvic + 0x80) = 0x10000000;  /* NVIC_ICER: disable IRQ 28 */
        uart_check_dispatch(param);
        nvic[0] = 0x10000000;  /* NVIC_ISER: re-enable IRQ 28 */
    }
}

/* ================================================================
 * USART idle line handler
 *
 * Called when USART1 detects an idle line (end of RX frame).
 * Parses any received packets and initiates TX response if needed.
 * ================================================================ */
void __attribute__((noinline)) usart_idle_handler(uint8_t *param)
{
    uart_check_dispatch(param);

    if (param[UART_PKT_READY] != 0) {
        param[UART_PKT_READY] = 0;
        if (param[UART_TX_ACTIVE] == 0) {
            if (param[UART_ERROR] == 0) {
                /* No delay needed — start TX on uart_state.
                 * Original at 0x284A loads uart_state_arr from literal
                 * pool (NOT param/motor_ctrl) for the TX start call.
                 * uart_tx_start_if_ready clears SYNC_ID via vtable[2],
                 * which prevents uart_tx_process from re-parsing. */
                {
                    volatile uint8_t *us = (volatile uint8_t *)uart_state_arr;
                    uart_tx_start_if_ready((uint8_t *)us);
                }
                return;
            }
            /* Delay needed — set flag and configure timer */
            param[UART_TX_ACTIVE] = 1;
            int16_t bit_count = tx_byte_count_calc(param);
            timer_setup_tx(param, bit_count, param[UART_ERROR]);
        }
    }
}

/* ================================================================
 * uart check+dispatch
 *
 * Parse and dispatch if a complete packet is available.
 * ================================================================ */
void __attribute__((noinline)) uart_check_dispatch(uint8_t *s)
{
    int32_t result = uart_packet_parse(s);
    if (result != 0) {
        uart_dispatch(s);
    }
}

/* ================================================================
 * motor_uart_obj_init
 *
 * Initializes motor control UART object: calls uart_state_init,
 * stores motor_ctrl_vtable, clears TX pending flag.
 * ================================================================ */
uint32_t * __attribute__((noinline)) motor_uart_obj_init(uint32_t *param)
{
    extern const void *motor_ctrl_vtable[];

    /* Initialize UART state machine */
    uart_state_init(param);

    /* Set vtable to motor_ctrl_vtable */
    *param = (uint32_t)motor_ctrl_vtable;

    /* Clear TX pending flag */
    ((uint8_t *)param)[UART_TX_ACTIVE] = 0;

    return param;
}

/* ================================================================
 * uart_motor_init
 *
 * Initializes the motor control object from uart_motor context.
 * Calls motor_uart_obj_init with motor_ctrl_arr, then stores
 * motor_calibrate_vtable.
 * ================================================================ */
void __attribute__((noinline)) uart_motor_init(void)
{
    extern const void *motor_calibrate_vtable[];
    uint32_t *p = (uint32_t *)(void *)motor_ctrl_arr;
    motor_uart_obj_init(p);
    *p = (uint32_t)motor_calibrate_vtable;
}

/* ================================================================
 * factory reset handler
 *
 * Performs factory reset: saves EEPROM, reinitializes, resets encoder.
 * ================================================================ */
void __attribute__((noinline)) factory_reset_handler(void)
{
    eeprom_save_regs(pwm_ctrl_arr);
    usart1_baud_init(pwm_ctrl_arr);
    uint32_t enc = (uint32_t)encoder_set_initial((int32_t *)encoder_ctrl_arr, 0);
    *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS) = enc;
    uart_state_arr[UART_ERROR] = 1;
}

/* ================================================================
 * uart TX + LED tick
 *
 * Dispatches UART TX and processes LED blink.
 * ================================================================ */
void __attribute__((noinline)) uart_tx_led_tick(uint8_t *param)
{
    uart_tx_dispatch(uart_state_arr);
    uart_tx_process(param);
}

/* ================================================================
 * UART state init
 *
 * Initializes the full UART protocol state machine.
 * ================================================================ */
void __attribute__((noinline)) uart_state_init(uint32_t *s)
{
    uint8_t *p = (uint8_t *)s;
    extern const void *uart_state_vtable[];
    *s = (uint32_t)uart_state_vtable;  /* vtable — original loads from literal pool */
    subsys_memset(p + UART_PKT_ID, 0, 3);   /* history bytes */
    subsys_memset(p + UART_HISTORY_0, 0, 3);   /* history backup */
    p[UART_PARAM_IDX] = 0;   /* param_idx */
    p[UART_PARSE_STATE] = 0;   /* parse_state */
    p[UART_SERVO_ID] = 0;   /* servo_id */
    p[UART_SYNC_IDX] = 0;   /* sync_idx */
    *(uint8_t *)(s + 0x44) = 0;   /* tx_len */
    p[UART_ERROR] = 0;   /* error */
    p[UART_PKT_READY] = 0;   /* pkt_ready */
}

/* ================================================================
 * SYNC_WRITE accumulate
 *
 * Accumulates SYNC_WRITE packet data. Handles servo ID matching
 * and parameter extraction for multi-servo sync writes.
 * ================================================================ */
uint32_t __attribute__((noinline)) sync_write_accumulate(uint8_t *s, uint32_t byte_val, int32_t state_idx)
{
    uint8_t byte = (uint8_t)byte_val;

    if (state_idx == 0) {
        /* First param byte = start register address */
        s[UART_PACKET_BUF] = byte;
        return 0;
    }
    if (state_idx == 1) {
        /* Second param byte = data length per servo */
        if ((int32_t)byte_val > 0x7D) {
            return 1;  /* Too long */
        }
        s[UART_SYNC_IDX] = 0;
        s[UART_PACKET_BUF + 1] = byte;
        return 0;
    }

    /* Subsequent bytes: servo ID followed by data */
    uint32_t sync_idx = (uint32_t)s[UART_SYNC_IDX];
    if (sync_idx == 0) {
        /* This byte is a servo ID */
        s[UART_SYNC_ID] = byte;
        if (s[UART_SERVO_ID] == byte_val) {
            /* Our servo ID matches — start collecting */
            s[UART_PACKET_BUF + 2] = byte;
            s[UART_TX_LEN] = 0;
        }
    } else if (s[UART_SYNC_ID] == s[UART_SERVO_ID]) {
        /* Data byte for our servo */
        s[sync_idx + UART_PACKET_BUF + 2] = byte;
        s[UART_TX_LEN] = s[UART_TX_LEN] + 1;
    }

    s[UART_SYNC_IDX] = (uint8_t)(sync_idx + 1);
    if (((sync_idx + 1) & 0xFF) == s[UART_PACKET_BUF + 1] + 1) {
        s[UART_SYNC_IDX] = 0;  /* Reset for next servo */
    }
    return 0;
}

/* ================================================================
 * WRITE handler
 *
 * Handles WRITE instruction: writes data to registers and calls
 * side-effects handler via vtable.
 * ================================================================ */
int32_t __attribute__((noinline)) handle_write(uint32_t *param, uint8_t *params)
{
    uint8_t param_len = params[0x80];
    /* Original doesn't clear param[0x44] here — it's done by sync_write only */

    if ((uint8_t)param_len < 2) {
        return -1;
    }
    uint8_t addr = params[0];
    __asm__ volatile ("nop");  /* Original loads pwm_ctrl_arr from pool here as r0 for reg_write */
    int32_t result = reg_write(addr, params + 1, (uint8_t)param_len - 1);
    if (result != -1) {
        /* Call vtable side-effects handler */
        typedef void (*side_effect_fn_t)(uint32_t *, int32_t, int32_t);
        side_effect_fn_t fn = (side_effect_fn_t)(*(uint32_t *)(param[0] + 4));
        fn(param, addr, result);
    }
    return result;
}

/* ================================================================
 * SYNC_WRITE handler
 *
 * Handles SYNC_WRITE: extracts accumulated data for this servo
 * and writes to registers.
 * ================================================================ */
int32_t __attribute__((noinline)) handle_sync_write(uint32_t *param, uint8_t *params)
{
    uint8_t *s = (uint8_t *)param;
    int32_t len = *(uint8_t *)(param + 0x44);
    *(uint8_t *)(param + 0x44) = 0;
    int32_t result = 0;

    if ((uint8_t)len != 0) {
        uint8_t addr = s[UART_PACKET_BUF];
        result = reg_write(addr, params + 3, (uint8_t)len);
        if (result != -1) {
            typedef void (*side_effect_fn_t)(uint32_t *, int32_t, int32_t);
            side_effect_fn_t fn = (side_effect_fn_t)(*(uint32_t *)(param[0] + 4));
            fn(param, addr, result);
        }
    }
    return result;
}

/* ================================================================
 * baud rate change
 *
 * Changes USART1 baud rate. Disables USART, changes baud divider,
 * re-enables.
 * ================================================================ */
extern const uint32_t baud_rate_config[];
void __attribute__((noinline)) baud_rate_change(uint8_t *param, uint32_t baud_idx)
{
    (void)param;
    /* Original uses single USART1_BASE pointer with [r3,#0] and [r3,#12] */
    volatile uint32_t * volatile usart = (volatile uint32_t *)USART1_BASE;

    uint32_t baud;
    if (baud_idx <= 7) {
        baud = baud_table_ext[baud_idx].baud;
    } else {
        baud = 1000000;
    }

    /* Disable USART */
    usart[0] = usart[0] & ~1U;

    /* Set new baud rate */
    usart[3] = 48000000U / baud;

    /* Re-enable */
    usart[0] = usart[0] | 1;
}

/* ================================================================
 * DMA TX start
 *
 * Starts DMA transfer for UART TX. Clears TC flag, disables RX
 * DMA channel, configures TX DMA channel with address and count.
 * ================================================================ */
void __attribute__((noinline)) dma_tx_start(uint8_t *s, uint32_t addr, uint32_t len)
{
    (void)s;
    volatile uint32_t *usart = (volatile uint32_t *)USART1_BASE;
    volatile uint32_t *dma_ch3 = (volatile uint32_t *)0x40020044U;  /* DMA CH3 (USART1 TX) */

    /* Clear TC flag */
    usart[8] = 0x40;                   /* ICR = 0x40 (clear TC) */

    /* Enable TCIE */
    usart[0] = usart[0] | 0x40U;      /* CR1 |= TCIE */

    /* Disable DMA channel */
    dma_ch3[0] = dma_ch3[0] & ~1U;    /* CTL &= ~CHEN */

    /* Set memory address and count */
    dma_ch3[3] = addr;                 /* MADDR = addr */
    dma_ch3[1] = len;                  /* CNT = len */

    /* Enable DMA channel */
    dma_ch3[0] = dma_ch3[0] | 1U;     /* CTL |= CHEN */
}

/* ================================================================
 * uart TX start if ready
 *
 * Checks if UART is ready to transmit and has data. If so, calls
 * the vtable reset function and starts DMA TX.
 * ================================================================ */
void __attribute__((noinline)) uart_tx_start_if_ready(uint8_t *s)
{
    /* Original at 0x3AA0: guard on SYNC_ID and TX_LEN.
     * At 0x3AAE: call vtable[2] which on uart_state does:
     *   1. Clear USART RE (bit 2)
     *   2. Set USART TE (bit 3)
     *   3. Clear SYNC_ID = 0
     * Then DMA TX start, then clear TX_LEN.
     *
     * Called with uart_state_arr (NOT motor_ctrl_arr). */
    if (s[UART_SYNC_ID] == 0) return;
    if (s[UART_TX_LEN] == 0) return;

    /* Call vtable[2] to switch USART to TX mode and clear SYNC_ID.
     * Original at 0x3AAE: ldr r3,[r0,#0]; ldr r3,[r3,#8]; blx r3
     * vtable[2] = uart_tx_mode_switch (clears RE, sets TE, SYNC=0) */
    {
        typedef void (*vfn)(uint8_t *);
        vfn fn = ((vfn *)(*(uint32_t *)s))[2];
        fn(s);
    }

    dma_tx_start(s, (uint32_t)(s + UART_TX_BUF), s[UART_TX_LEN]);
    s[UART_TX_LEN] = 0;
}

/* ================================================================
 * uart TX force start
 *
 * Unconditionally starts UART DMA TX.
 * ================================================================ */
void __attribute__((noinline, section(".text.keep"))) uart_tx_force_start(uint8_t *s)
{
    ring_buf_reset(s + 4);
    uint8_t tx_len = *(uint8_t *)((uint32_t *)s + 0x44);
    dma_tx_start(s, (uint32_t)(s + UART_TX_BUF), tx_len);
    *(uint8_t *)((uint32_t *)s + 0x44) = 0;
}

/* ================================================================
 * uart state obj init wrapper (1B thunk)
 *
 * Calls uart_obj_init on uart_state_arr.
 * ================================================================ */
void __attribute__((constructor, noinline)) uart_state_obj_init(void)
{
    uart_obj_init((uint32_t *)uart_state_arr);
}

/* ================================================================
 * NVIC_SystemReset
 *
 * Triggers a full system reset via SCB AIRCR register.
 * Disables interrupts, preserves priority grouping, sets SYSRESETREQ.
 * Called from calibration vtable[2] after saving EEPROM.
 * ================================================================ */
void __attribute__((noinline)) nvic_system_reset(void)
{
    __asm__ volatile ("cpsid i" ::: "memory");  /* disable interrupts */
    __asm__ volatile ("dsb sy" ::: "memory");    /* memory barrier */

    volatile uint32_t *scb = (volatile uint32_t *)0xE000ED00U;
    uint32_t prigroup = scb[3] & 0x700U;  /* AIRCR: preserve PRIGROUP */
    scb[3] = 0x05FA0000U | prigroup | 0x04U;  /* VECTKEY + SYSRESETREQ */

    __asm__ volatile ("dsb sy" ::: "memory");
    while (1) {}  /* Wait for reset */
}

/* ================================================================
 * position offset set / calibration
 *
 * Sets the encoder position offset for position calibration.
 * Only processes if param_2 is within the valid 12-bit range
 * (0x001..0x1FFF, i.e., (param_2 + 0xFFF) < 0x1FFF).
 *
 * ================================================================ */
void __attribute__((noinline)) position_offset_set(uint8_t *param_unused, int16_t position)
{
    (void)param_unused;
    extern uint32_t position_linearize(uint32_t raw);

    if ((uint16_t)(position + 0xFFF) >= 0x1FFF) {
        return;
    }

    /* Clear encoder accumulator */
    {
        volatile uint8_t *ec = encoder_ctrl;
        *(uint32_t *)(ec + ENC_DELINEAR) = 0;
    }

    /* Read current encoder position */
    int32_t current = encoder_set_initial((int32_t *)encoder_ctrl_arr, 0);

    /* Compute offset using linearization */
    int16_t delta = (int16_t)(current - position);
    uint16_t linearized = (uint16_t)position_linearize((uint32_t)(int32_t)delta);

    /* Store goal position offset in servo_regs_arr[SR_GOAL_OFFSET_LO] */
    {
        volatile uint8_t *sr = servo_regs;
        *(uint16_t *)(sr + SR_GOAL_OFFSET_LO) = linearized;
    }

    /* Remap register address SR_GOAL_OFFSET_LO to EEPROM page offset and save */
    uint32_t mapped = servo_apply_config(pwm_ctrl_arr, SR_GOAL_OFFSET_LO);
    if (mapped != 0xFFFFFFFF) {
        volatile uint8_t *sr = servo_regs;
        buf_to_regs(eeprom_ctrl_arr, (uint8_t *)sr + SR_GOAL_OFFSET_LO, mapped & 0xFFFF, 2);
        eeprom_save_byte_internal(eeprom_ctrl_arr);
        {
            volatile uint8_t *ec = encoder_ctrl;
            *(int32_t *)(ec + ENC_DELINEAR) = (int32_t)delta;
            encoder_update_position((int32_t *)ec);
        }
        {
            volatile uint8_t *ec = encoder_ctrl;
            volatile uint8_t *pc = pwm_ctrl;
            int32_t pos = *(int32_t *)(ec + ENC_ACCUM_POS);
            *(int32_t *)(pc + PWM_ENCODER_POS) = pos;
            *(int32_t *)(pc + PWM_GOAL_POS) = pos;
            *(int32_t *)pc = pos;
            timer0_set_pwm(pid_state_arr, pos, pos);
        }
    }
}

/* ================================================================
 * register change detection
 *
 * Checks if a write to registers [write_start, write_start+write_count)
 * overlaps with a specific register field at reg_offset of field_size bytes.
 *
 * the SRAM address of the field and compute the offset the same way.
 *
 * Returns 1 if the field was modified, 0 otherwise.
 * ================================================================ */
static uint32_t __attribute__((noinline)) reg_changed(uint32_t unused,
    int32_t reg_sram_addr, int32_t write_start, int32_t write_count,
    uint8_t field_size)
{
    (void)unused;
    uint32_t field_offset = (uint32_t)(reg_sram_addr - (int32_t)(uint32_t)servo_regs_arr) & 0xFF;
    if (((uint32_t)(write_count - 1 + write_start) & 0xFF) < field_offset) {
        return 0;
    }
    if ((int32_t)(((uint32_t)(field_size - 1) + field_offset) & 0xFF) < write_start) {
        return 0;
    }
    return 1;
}

/* ================================================================
 * Register change detection after register writes.
 *
 * Register change detection function. Called from motor_calibrate_vtable
 * after register writes complete. Detects which register groups changed
 * and triggers appropriate reconfiguration.
 *
 *
 * param_1 = motor_ctrl_arr vtable struct pointer
 * param_2 = start register address of write
 * param_3 = count of bytes written
 * ================================================================ */
void __attribute__((noinline)) uart_parse_byte(uint8_t *param_1)
{
    /* Extract write_start and write_count from the calling convention.
     * In the original, these come as param_2 (r1) and param_3 (r2)
     * from the vtable dispatch in handle_write/handle_sync_write.
     * We receive them via the same register-based convention. */
    register uint32_t write_start __asm__("r1");
    register uint32_t write_count __asm__("r2");
    int32_t start = (int32_t)write_start;
    int32_t count = (int32_t)(write_count & 0xFF);

    /* Base SRAM address for register offset calculations */
    int32_t reg_2a_addr = (int32_t)(uint32_t)(servo_regs_arr + SR_GOAL_POS_LO);

    /* Check if CW angle limit (regs 0x2A-0x2B) changed -> flag[PID_CW_CHANGED] */
    uint8_t uVar3 = (uint8_t)reg_changed(
        (uint32_t)pwm_ctrl_arr, reg_2a_addr, start, count, 2);
    pid_state_arr[PID_CW_CHANGED] = uVar3;

    /* Check if CCW angle limit (regs 0x2C-0x2D) changed -> flag[PID_CCW_CHANGED] */
    uVar3 = (uint8_t)reg_changed(
        (uint32_t)pwm_ctrl_arr, reg_2a_addr + 2, start, count, 2);
    pid_state_arr[PID_CCW_CHANGED] = uVar3;

    /* Check if acceleration (reg 0x29) changed -> flag[PID_ACCEL_CHANGED] */
    uVar3 = (uint8_t)reg_changed(
        (uint32_t)pwm_ctrl_arr, reg_2a_addr - 1, start, count, 1);
    pid_state_arr[PID_ACCEL_CHANGED] = uVar3;

    /* Check if speed/torque (regs 0x2E-0x2F) changed -> flag[PID_SPEED_CHANGED] */
    uVar3 = (uint8_t)reg_changed(
        (uint32_t)pwm_ctrl_arr, reg_2a_addr + 4, start, count, 2);
    pid_state_arr[PID_SPEED_CHANGED] = uVar3;

    /* Check if angle offset (regs 0x1F-0x20) changed */
    int32_t changed = (int32_t)reg_changed(
        (uint32_t)pwm_ctrl_arr, reg_2a_addr - 0x0B, start, count, 2);
    if (changed != 0) {
        /* Re-delinearize position offset and update encoder */
        uint32_t delinear = (uint32_t)position_delinearize(
            pwm_ctrl_arr, (int32_t)*(int16_t *)(servo_regs_arr + SR_GOAL_OFFSET_LO));
        *(uint32_t *)(encoder_ctrl_arr + ENC_DELINEAR) = delinear;
        encoder_update_position((int32_t *)encoder_ctrl_arr);
        uint32_t pos = *(uint32_t *)(encoder_ctrl_arr + ENC_POSITION_VAL);
        *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS) = pos;
        *(uint32_t *)(pwm_ctrl_arr + PWM_GOAL_POS) = pos;
        timer0_set_pwm(pid_state_arr, *(uint32_t *)pwm_ctrl_arr, pos);
    }

    /* Check if operating mode (reg 0x21) changed */
    changed = (int32_t)reg_changed(
        (uint32_t)pwm_ctrl_arr, (int32_t)(uint32_t)(servo_regs_arr + SR_OPERATING_MODE),
        start, count, 1);
    if (changed != 0) {
        uint8_t mode = servo_regs_arr[SR_OPERATING_MODE];
        if (*(pwm_ctrl_arr + PWM_OP_MODE) != mode) {
            /* Mode changed: reset encoder, clear speed/accel, reinit profile */
            *(pwm_ctrl_arr + PWM_OP_MODE) = mode;
            uint32_t pos = (uint32_t)encoder_set_initial(
                (int32_t *)encoder_ctrl_arr, 0);
            *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS) = pos;
            servo_regs_arr[SR_GOAL_SPEED_LO] = 0;
            servo_regs_arr[SR_GOAL_SPEED_LO + 1] = 0;
            servo_regs_arr[SR_ACCELERATION] = 0;
            servo_regs_arr[SR_TORQUE_ENABLE] = 0;
            position_profile_init(pid_state_arr,
                (int32_t)servo_regs_arr[SR_ACCELERATION],
                (int32_t)*(uint16_t *)(servo_regs_arr + SR_GOAL_SPEED_LO));
            adc_state_arr[1] = 0xFF;
        }
    }

    /* Check if baud rate (reg 0x06) changed */
    {
        int32_t baud_changed = (int32_t)reg_changed(
            (uint32_t)pwm_ctrl_arr, (int32_t)(uint32_t)(servo_regs_arr + SR_SERVO_ID),
            start, count, 1);
        if (baud_changed != 0) {
            uint8_t clamped = (uint8_t)clamp_baud_index(
                (uint32_t)uart_state_arr, (uint32_t)servo_regs_arr[SR_SERVO_ID]);
            servo_regs_arr[SR_SERVO_ID] = clamped;
            uart_state_arr[UART_ERROR] = 1;
        }
    }

    /* Check if motion flag (reg 0x28) changed -- reboot detection */
    {
        int32_t motion_changed = (int32_t)reg_changed(
            (uint32_t)pwm_ctrl_arr, (int32_t)(uint32_t)(servo_regs_arr + SR_TORQUE_ENABLE),
            start, count, 1);
        if (motion_changed != 0) {
            if ((int8_t)servo_regs_arr[SR_TORQUE_ENABLE] == -0x80) {
                /* Reboot flag (0x80 in servo_regs_arr[0x28]): trigger system reboot
                 * via vtable[6] = (**(code **)(*param_1 + 0x18))(param_1, 0x800) */
                typedef void (*reboot_fn_t)(uint8_t *, int32_t);
                reboot_fn_t fn = (reboot_fn_t)(*(uint32_t *)(*(uint32_t *)param_1 + 0x18));
                fn(param_1, 0x800);
                servo_regs_arr[SR_TORQUE_ENABLE] = 0;
            }
        }
    }
}


/* Set UART sync flag (original at 0x080038FD, 6 bytes).
 * Called via ring_buf_vtable[1] to signal TX ready. */
void __attribute__((noinline)) uart_sync_set(uint8_t *state)
{
    state[UART_SYNC_ID] = 1;
}

/* Clear UART sync flag (original at 0x08003905, 6 bytes).
 * Called via ring_buf_vtable[2] to clear TX ready. */
void __attribute__((noinline)) uart_sync_clear(uint8_t *state)
{
    state[UART_SYNC_ID] = 0;
}

/*
 * Vtable (Virtual Function Table) definitions
 *
 * The original firmware uses function pointer tables stored in .rodata
 * for polymorphic dispatch. Each subsystem struct starts with a pointer
 * to its vtable. Different operating modes swap vtables to change behavior.
 *
 * This file defines all vtable arrays matching the original at 0x08003F84-0x080040FC.
 */

#include <stdint.h>

/* ================================================================
 * Forward declarations of all vtable-referenced functions
 * ================================================================ */

/* Encoder subsystem */
extern void encoder_read(uint8_t *);
extern void encoder_set_initial_pos(uint8_t *);
extern void encoder_set_initial(int32_t *, int32_t);
extern void encoder_read_complete(uint8_t *);
extern int32_t encoder_get_raw_angle(int32_t *);
extern void gpio_led_init(uint8_t *);
extern void led_blink_tick(uint8_t *);

/* PWM / Timer */
extern void timer15_init(uint8_t *);
extern void pwm_apply_output(uint8_t *);

/* UART */
extern void servo_regs_init(void);
extern void uart_tx_enable(uint8_t *);
extern void uart_tx_dispatch(uint8_t *);
extern void uart_tx_mode_switch(uint8_t *);
extern void i2c_init(uint8_t *);

/* I2C */
extern void i2c_hw_init(uint8_t *);
extern void i2c_data_ready(void);
extern void encoder_multiturn_track(int32_t *);
extern void encoder_set_initial(int32_t *, int32_t);

/* Motor control */
extern void usart_idle_handler(uint8_t *);
extern void uart_send_response(uint8_t *);
extern void motor_init(uint8_t *);
extern void timer15_tick_handler(uint8_t *);
extern void uart_tx_process(uint8_t *);
extern void uart_parse_byte(uint8_t *);
extern void uart_tx_led_tick(uint8_t *);
extern void motor_safety_apply(uint8_t *);
extern void uart_motor_init(void);
extern void factory_reset_handler(void);
extern void nvic_system_reset(void);
extern void eeprom_header_write_wrap(void);
extern void encoder_home_reset(uint8_t *);
extern void position_offset_set(uint8_t *, int16_t);
extern void timer0_set_duty(uint8_t *, int32_t, int32_t);

/* Ring buffer */
extern void usart1_dma_init(uint8_t *, uint32_t);
extern void uart_sync_set(uint8_t *);
extern void uart_sync_clear(uint8_t *);
extern void uart_check_dispatch(uint8_t *);
extern void timer0_hw_init(uint8_t *);

/* New wrapper functions */
extern void timer15_init_wrapper(uint8_t *);
extern void servo_regs_init_wrapper(uint8_t *);
extern void i2c_mode_init(uint8_t *);
extern void gpio_led_enable(void);
extern void gpio_led_alarm_set(void);
extern void gpio_led_state_set(void);
extern void led_blink_tick(uint8_t *);
extern void led_vtable_dispatch(uint32_t *);

/* Default handler (does nothing) — address 0x08003ACF in original */
extern void uart_tx_force_start(uint8_t *);

/* Use a no-op function for null vtable entries */
static void vtable_noop(void) { }

/* ================================================================
 * Vtable arrays — stored in .rodata (flash), never overwritten
 *
 * Each entry is a function pointer (Thumb address, bit 0 set by linker).
 * The subsystem init functions store a pointer to the appropriate vtable
 * in the first word of the subsystem struct.
 * ================================================================ */

typedef void (*vfunc_t)(void);

/* Encoder vtable (0x08003F84): read, set_initial, get_raw */
const vfunc_t encoder_vtable[] = {
    (vfunc_t)encoder_read,              /* [0] 0x080003D5 */
    (vfunc_t)encoder_set_initial_pos,   /* [1] 0x0800044D */
    (vfunc_t)encoder_get_raw_angle,     /* [2] 0x080002ED — raw angle reader, NOT encoder_read_complete */
};

/* GPIO/LED vtable (0x08003F98): enable, alarm_set, state_set, led_blink_tick */
const vfunc_t gpio_vtable[] = {
    (vfunc_t)gpio_led_enable,           /* [0] 0x08000491 */
    (vfunc_t)gpio_led_alarm_set,        /* [1] 0x08000461 */
    (vfunc_t)gpio_led_state_set,        /* [2] 0x0800046D */
    (vfunc_t)led_blink_tick,            /* [3] 0x08001999 */
};

/* PWM control vtable (0x08003FB0): timer15_init, apply_output */
const vfunc_t pwm_ctrl_vtable[] = {
    (vfunc_t)timer15_init_wrapper,      /* [0] 0x08000C4D */
    (vfunc_t)pwm_apply_output,          /* [1] 0x08000C95 */
};

/* UART object vtable (0x08003FC0): servo_regs_init, tx_enable, tx_mode_switch */
const vfunc_t uart_obj_vtable[] = {
    (vfunc_t)servo_regs_init_wrapper,   /* [0] 0x08000DBD */
    (vfunc_t)uart_tx_enable,            /* [1] 0x08000D2D — RX mode: clear TE, set RE, SYNC=1 */
    (vfunc_t)uart_tx_mode_switch,       /* [2] 0x08000D4D — TX mode: clear RE, set TE, SYNC=0 */
};

/* I2C init vtable (0x08003FD4): i2c_mode_init, i2c_data_ready */
const vfunc_t i2c_init_vtable[] = {
    (vfunc_t)i2c_mode_init,             /* [0] 0x08000E75 */
    (vfunc_t)i2c_data_ready,            /* [1] 0x080012C5 */
};

/* Encoder mode vtable (0x08003FE4): multiturn_track, set_initial, noop */
const vfunc_t encoder_mode_vtable[] = {
    (vfunc_t)encoder_multiturn_track,   /* [0] 0x0800111D */
    (vfunc_t)encoder_set_initial,       /* [1] 0x08001101 */
    (vfunc_t)vtable_noop,              /* [2] 0x08003ACF */
};

/* I2C object vtable (0x08003FF8): hw_init, data_ready */
const vfunc_t i2c_obj_vtable[] = {
    (vfunc_t)i2c_hw_init,              /* [0] 0x08001181 */
    (vfunc_t)i2c_data_ready,           /* [1] 0x080012C5 */
};

/* LED object vtable (0x08004008): dispatch, noop, noop, led_blink_tick */
const vfunc_t led_obj_vtable[] = {
    (vfunc_t)led_vtable_dispatch,       /* [0] 0x0800194D */
    (vfunc_t)vtable_noop,              /* [1] 0x08003ACF */
    (vfunc_t)vtable_noop,              /* [2] 0x08003ACF */
    (vfunc_t)led_blink_tick,           /* [3] 0x08001999 */
};

/* UART state vtable (0x08004030): check_dispatch + 6 noop (7 entries in original) */
const vfunc_t uart_state_vtable[] = {
    (vfunc_t)uart_check_dispatch,      /* [0] 0x080026C9 — parse+dispatch (NOT usart_idle_handler) */
    (vfunc_t)vtable_noop,              /* [1] */
    (vfunc_t)vtable_noop,              /* [2] */
    (vfunc_t)vtable_noop,              /* [3] */
    (vfunc_t)vtable_noop,              /* [4] */
    (vfunc_t)vtable_noop,              /* [5] */
    (vfunc_t)vtable_noop,              /* [6] */
};

/* Motor control vtable — normal mode (0x08004054): 6 entries in original */
const vfunc_t motor_ctrl_vtable[] = {
    (vfunc_t)usart_idle_handler,       /* [0] 0x0800280D — idle handler */
    (vfunc_t)vtable_noop,              /* [1] */
    (vfunc_t)vtable_noop,              /* [2] */
    (vfunc_t)vtable_noop,              /* [3] */
    (vfunc_t)vtable_noop,              /* [4] */
    (vfunc_t)vtable_noop,              /* [5] */
};

/* Motor control vtable — active mode (0x08004070): 3 entries in original */
const vfunc_t motor_active_vtable[] = {
    (vfunc_t)uart_motor_init,          /* [0] 0x080026DD — motor_init */
    (vfunc_t)timer15_tick_handler,     /* [1] 0x08002739 — tick handler */
    (vfunc_t)uart_tx_process,          /* [2] 0x080027E5 — TX process */
};

/* Motor control vtable — calibration mode (0x08004084): 10 entries in original */
const vfunc_t motor_calibrate_vtable[] = {
    (vfunc_t)usart_idle_handler,       /* [0] 0x0800280D — idle handler */
    (vfunc_t)uart_parse_byte,          /* [1] 0x08002985 — register change handler */
    (vfunc_t)nvic_system_reset,        /* [2] 0x08002859 — system reboot */
    (vfunc_t)factory_reset_handler,    /* [3] 0x080028A9 — save+reinit+reset encoder */
    (vfunc_t)eeprom_header_write_wrap, /* [4] 0x08002881 — save EEPROM before reboot */
    (vfunc_t)encoder_home_reset,       /* [5] 0x080028D9 — reset encoder home position */
    (vfunc_t)position_offset_set,      /* [6] 0x080028FD — position calibration (TODO: separate fn) */
    (vfunc_t)uart_motor_init,          /* [7] 0x080026DD — motor_init (same as active[0]) */
    (vfunc_t)timer15_tick_handler,     /* [8] 0x08002739 — tick handler (same as active[1]) */
    (vfunc_t)uart_tx_led_tick,         /* [9] 0x08002B1D — LED tick */
};

/* PWM output vtable (0x08004020): 2 entries in original */
const vfunc_t pwm_output_vtable[] = {
    (vfunc_t)timer0_hw_init,           /* [0] 0x08001BED — full TIMER0 hardware init */
    (vfunc_t)timer0_set_duty,          /* [1] 0x08001CB9 — set duty cycle */
};

/* Ring buffer vtable (0x080040F4) */
const vfunc_t ring_buf_vtable[] = {
    (vfunc_t)usart1_dma_init,          /* [0] 0x0800390D */
    (vfunc_t)uart_sync_set,            /* [1] 0x080038FD — set state[0x111]=1 */
    (vfunc_t)uart_sync_clear,          /* [2] 0x08003905 — clear state[0x111]=0 */
};

/* Baud rate configuration table (0x080040AC)
 * Format: [baud_rate, tx_time_per_byte] pairs
 * tx_time = (10 bits * 1000000) / baud = microseconds per byte */
const uint32_t baud_rate_config[] = {
    1000000, 10,     /* index 0: 1Mbps */
     500000, 20,     /* index 1: 500kbps */
     250000, 40,     /* index 2: 250kbps */
     128000, 79,     /* index 3: 128kbps */
     115107, 87,     /* index 4: ~115.2kbps (actual: 48MHz/87/16 ≈ 34483*10/3 = 115107) */
      76800, 131,    /* index 5: 76.8kbps */
      57600, 174,    /* index 6: 57.6kbps */
      38400, 261,    /* index 7: 38.4kbps */
};

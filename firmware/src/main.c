/*
 * Feetech STS3215 Servo Firmware
 *
 * Open source reimplementation targeting binary-level equivalence
 * with the original SCServo21-GD32-TTL-250306 firmware (16816 bytes).
 *
 * Target: GD32F130C6T6 (Cortex-M3, 48MHz, 32KB flash, 4KB SRAM)
 */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <string.h>

/* ================================================================
 * Register and peripheral base shortcuts
 * ================================================================ */
#define GPIOA_BASE   0x48000000U
#define GPIOB_BASE   0x48000400U
#define GPIOF_BASE   0x48001400U
#define TIMER0_BASE  0x40012C00U
#define TIMER15_BASE 0x40014400U
#define USART1_BASE  0x40004400U
#define I2C0_BASE    0x40005400U

/* Direct register access (matches original firmware's literal pool style) */
#define REG(base, off)  (*(volatile uint32_t *)((base) + (off)))
#define REG16_AT(base, off) (*(volatile uint16_t *)((base) + (off)))


/* ================================================================
 * Forward declarations
 * ================================================================ */
static void nvic_priority_group_config(uint32_t prigroup);
static void fwdgt_setup(uint32_t reload, uint32_t prescaler);
void __attribute__((noinline)) servo_defaults_init(void);
void timer0_hw_init(uint8_t *pwm_ctrl);
void __attribute__((noinline)) gpio_config_pin(uint8_t *cfg);

/* ================================================================
 * Global state (BSS — zero initialized)
 *
 * SRAM layout matches original firmware exactly.
 * Struct sizes derived from address spacing in the original.
 * ================================================================ */

/* 0x20000000: Register permission table + linearization LUT (.data section)
 * Permission bits: bit0=R, bit1=W, bit2=EEPROM-backed
 * 0x05=RO+EEPROM, 0x07=RW+EEPROM, 0x03=RW, 0x01=RO, 0x00=none
 */
uint8_t reg_permissions[88] = {
    /* regs 0-3: firmware version, servo model (RO+EEPROM) */
    0x05, 0x05, 0x05, 0x05,
    /* regs 4-15: baud, delay, ID, reserved, alarm, reserved, cw/ccw limits, temp, voltage (RW+EEPROM) */
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    /* regs 16-39: torque, config, PID, deadzone, punch, mode, accel (RW+EEPROM) */
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    /* regs 40-49: runtime RW (torque enable, goal, speed, max output) */
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03,
    /* regs 50-53: current PI gains (RW+EEPROM — persist across power cycles) */
    0x07, 0x07, 0x07, 0x07,
    /* regs 54-55: reserved, lock (runtime) */
    0x03, 0x03,
    /* regs 56-77: present position, speed, load, voltage, temp, etc. (RO) */
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    /* regs 78-86: RW+EEPROM (extended config) */
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    /* reg 87: no access */
    0x00,
};

/* Position linearization LUT (32 entries, 16-bit values)
 * Original: 64 bytes at SRAM 0x20000058 (after reg_permissions)
 * 31 data values + 1 zero terminator */
uint16_t position_lut[32] = {
    0x0250, 0x0216, 0x01DD, 0x01A7, 0x0175, 0x0147, 0x011E, 0x00F9,
    0x00D8, 0x00BC, 0x00A3, 0x008E, 0x007B, 0x006B, 0x005E, 0x0052,
    0x0048, 0x003F, 0x0038, 0x0031, 0x002B, 0x0027, 0x0022, 0x001F,
    0x001B, 0x0019, 0x0016, 0x0014, 0x0012, 0x0010, 0x000F,
    0x0000, /* terminator */
};

/* 0x200000BC: encoder control state (36 bytes) */
/* All arrays must be 4-byte aligned — the firmware uses strd/ldrd and
 * word-aligned accesses via casted pointers (vtable, uint32_t*, etc.).
 * Without alignment, GCC may place uint8_t arrays at odd addresses,
 * causing UsageFaults on hardware (Cortex-M3 enforces alignment for
 * multi-word instructions like strd). */
#define ALIGNED4 __attribute__((aligned(4)))

ALIGNED4 uint8_t encoder_ctrl_arr[36]; uint8_t * volatile encoder_ctrl = encoder_ctrl_arr;

/* 0x200000E0: GPIO/LED state (8 bytes) */
ALIGNED4 uint8_t gpio_ctrl_arr[8]; uint8_t * volatile gpio_ctrl = gpio_ctrl_arr;

/* 0x200000E8: Servo register table (88 bytes) */
ALIGNED4 uint8_t servo_regs_arr[88]; uint8_t * volatile servo_regs = servo_regs_arr;

/* 0x20000140: PWM control state (48 bytes) */
ALIGNED4 uint8_t pwm_ctrl_arr[48]; uint8_t * volatile pwm_ctrl = pwm_ctrl_arr;

/* 0x20000170: Timer control state (28 bytes + 84 bytes motion sub-state) */
ALIGNED4 uint8_t timer_ctrl_arr[28 + 84]; uint8_t * volatile timer_ctrl = timer_ctrl_arr;

/* 0x2000018C: UART protocol state (278 bytes) */
ALIGNED4 uint8_t uart_state_arr[278]; uint8_t * volatile uart_state = uart_state_arr;

/* 0x200002A0: I2C control state (48 bytes) */
ALIGNED4 uint8_t i2c_ctrl_arr[48]; uint8_t * volatile i2c_ctrl = i2c_ctrl_arr;

/* 0x200002D0: EEPROM emulation state (528 bytes) */
ALIGNED4 uint8_t eeprom_ctrl_arr[528]; uint8_t * volatile eeprom_ctrl = eeprom_ctrl_arr;

/* 0x200004DC: I2C encoder driver state (28 bytes) */
ALIGNED4 uint8_t encoder_i2c_arr[28]; uint8_t * volatile encoder_i2c = encoder_i2c_arr;

/* 0x200004F8: Motor/PID control state (280 bytes) */
ALIGNED4 uint8_t motor_ctrl_arr[280]; uint8_t * volatile motor_ctrl = motor_ctrl_arr;

/* 0x20000610: ADC state (8 bytes) */
ALIGNED4 uint8_t adc_state_arr[8]; uint8_t * volatile adc_state = adc_state_arr;

/* 0x20000618: PID controller state (84 bytes) */
ALIGNED4 uint8_t pid_state_arr[84]; uint8_t * volatile pid_state = pid_state_arr;

/* ================================================================
 * SystemInit — Clock configuration
 *
 * IRC8M → PLL ×12 → 48 MHz. Enables IRC14M, IRC40K. Sets VTOR.
 * ================================================================ */
void SystemInit(void)
{
    RCU_INT = 0x00FF0000U;
    RCU_CTL0 |= RCU_CTL0_IRC8MEN;
    RCU_CTL1 |= RCU_CTL1_IRC14MEN;
    RCU_RSTSCK |= RCU_RSTSCK_IRC40KEN;
    RCU_CTL0 &= 0x0000FFFFU;
    RCU_CFG0 = 0x00000000U;
    RCU_CFG0 |= (10U << 18);
    RCU_CTL0 |= RCU_CTL0_PLLEN;
    while (0U == (RCU_CTL0 & RCU_CTL0_PLLSTB)) {
    }
    RCU_CFG0 |= 0x00000002U;
    while ((RCU_CFG0 & 0x0CU) != 0x08U) {
    }
    SCB->VTOR = 0x08000000U;
}

/* ================================================================
 * nvic_priority_group_config
 * ================================================================ */
static void nvic_priority_group_config(uint32_t prigroup)
{
    SCB->AIRCR = prigroup | 0x05FA0000U;
}

/* ================================================================
 * fwdgt_setup
 * ================================================================ */
static void fwdgt_setup(uint32_t reload, uint32_t prescaler)
{
    FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;
    FWDGT_PSC = prescaler;
    FWDGT_RLD = reload;
    while (FWDGT_STAT == 0U) {
    }
    FWDGT_CTL = FWDGT_KEY_RELOAD;
    FWDGT_CTL = FWDGT_KEY_ENABLE;
}

/* ================================================================
 * gpio_config_pin
 *
 * Configure a GPIO pin. Config block layout:
 *   [0] = pin number (0-15 + port*32)
 *   [1] = mode value
 *   [2] = pull value
 *   [3] = 0=input/reset, 1=output/AF config
 * ================================================================ */
void __attribute__((noinline)) gpio_config_pin(uint8_t *cfg)
{
    uint32_t pin = cfg[0];
    uint32_t port_offset = (pin >> 5);
    uint32_t bit = pin & 0x1F;

    if (cfg[3] == 0) {
        /* Reset pin — write to BRR (bit reset register) */
        REG(0xE000E100, (port_offset + 0x20) * 4) = 1U << bit;
        return;
    }

    /* Configure NVIC priority for the pin's interrupt */
    uint32_t prigroup = (~REG(0xE000ED00, 0x0C) << 0x15) >> 0x1D;
    *(volatile uint8_t *)(0xE000E100 + 0x300 + pin) =
        (uint8_t)(((uint32_t)cfg[1] << (4 - prigroup)) | (0xFU >> prigroup & (uint32_t)cfg[2])) << 4;
    REG(0xE000E100, port_offset * 4) = 1U << bit;
}

/* ================================================================
 * servo_defaults_init
 *
 * Initialize servo register table with defaults.
 * These values are only used when EEPROM is blank or has an invalid
 * version header. On factory motors, EEPROM values override these
 * in eeprom_page_load_defaults(). Values here match a factory
 * STS3215 servo dump so blank-EEPROM motors work out of the box.
 * ================================================================ */
void __attribute__((noinline)) servo_defaults_init(void)
{
    volatile uint8_t *sr = servo_regs;
    sr[4]  = 0;       /* baud rate index (1Mbps) */
    sr[5]  = 1;       /* return delay */
    sr[6]  = 1;       /* servo ID (default 1) */
    sr[8]  = 1;       /* alarm LED enable */
    (void)sr[9];  sr[9]  = 0;       /* CW limit low */
    (void)sr[10]; sr[10] = 0;       /* CW limit high */
    (void)sr[11]; sr[11] = 0xFF;    /* CCW limit low (4095 = full range) */
    (void)sr[12]; sr[12] = 0x0F;    /* CCW limit high */
    sr[13] = 0x46;    /* max temp (70°C) */
    sr[14] = 0x8C;    /* max voltage (14.0V) */
    sr[15] = 0x28;    /* min voltage (4.0V) */
    *(volatile uint16_t *)(sr + 16) = 1000;  /* max torque */
    sr[18] = 0x0C;    /* config: ADC mode (no I2C encoder flag) */
    sr[19] = 0x2C;    /* alarm shutdown mask */
    sr[20] = 0x2F;    /* alarm config */
    sr[21] = 0x20;    /* KP gain */
    sr[22] = 0x20;    /* KD scale */
    sr[23] = 0;       /* KI gain */
    sr[24] = 0x10;    /* punch/min force */
    sr[25] = 0;       /* KD */
    sr[26] = 1;       /* CW dead zone */
    sr[27] = 1;       /* CCW dead zone */
    *(volatile uint16_t *)(sr + 28) = 310;  /* max current (0x0136) */
    sr[30] = 1;       /* punch divisor */
    (void)sr[31]; sr[31] = 0x55;   /* goal offset low (85) */
    (void)sr[32]; sr[32] = 0;      /* goal offset high */
    sr[33] = 0;       /* mode: position control */
    sr[34] = 0x14;    /* max speed (20) */
    sr[35] = 0xC8;    /* accel low */
    sr[36] = 0x50;    /* accel high */
    sr[37] = 0x0A;    /* speed I gain (10) */
    sr[38] = 0xC8;    /* reserved (factory=200) */
    sr[39] = 0xC8;    /* overload ratio (200) */

    /* Mode 4 current PI defaults — tuned on dyno:
     * Kp=3, Ki=50 tracks setpoints 0-30 within ±8% at stall,
     * recovers cleanly from free-spin→stall transitions.
     * Higher Kp (e.g. 100) causes oscillation at stall. */
    *(volatile uint16_t *)(sr + 50) = 3;    /* current Kp */
    *(volatile uint16_t *)(sr + 52) = 50;   /* current Ki */

    /* Lock must be 0 for EEPROM writes to work. On blank-EEPROM boot,
     * eeprom_page_load_defaults copies 0xFF from the erased page into
     * servo_regs[55], which locks the register table and silently
     * prevents all subsequent UART writes from persisting. */
    sr[55] = 0;

    /* Permission/flags table at offset 0x50 */
    sr[80] = 2;
    sr[81] = 0x14;
    sr[82] = 0x32;
    sr[83] = 1;
    sr[84] = 0x96;
    sr[85] = 100;
    sr[86] = 1;
}

/* ================================================================
 * timer0_hw_init
 *
 * Configure TIMER0 for PWM output on PA7 (CH0N) and PA8 (CH0).
 * H-bridge motor driver with dead-time insertion.
 * ================================================================ */
void __attribute__((noinline)) timer0_hw_init(uint8_t *ctrl)
{
    /* Match original: gpioa from mov.w, gpiob from literal pool */
    volatile uint32_t *gpioa = (volatile uint32_t *)0x48000000U;
    volatile uint32_t *gpiob = (volatile uint32_t *)0x48000400U;

    /* Configure PA7 as AF2 (TIMER0_CH0N) */
    gpioa[8] |= 0x20000000U;   /* AFSEL0: PA7 = AF2 */
    gpiob[8] |= 0x20U;         /* AFSEL0: PB0 = AF2 */
    gpioa[0] |= 0x8000U;       /* CTL: PA7 = AF */
    gpioa[2] &= ~0xC000U;      /* OSPD: PA7 speed clear */
    gpiob[0] |= 0x08U;         /* CTL: PB0 = AF */
    gpiob[2] &= ~0x0CU;        /* OSPD: PB0 speed clear */

    /* Enable TIMER0 clock: RCU_APB2EN |= 0x800 */
    volatile uint32_t *rcu = (volatile uint32_t *)0x40021000U;
    rcu[6] |= 0x00000800U;

    /* TIMER0 — keep base in register, use strh [base, #imm] */
    register volatile uint32_t *tb __asm__("r3") = (volatile uint32_t *)0x40012C00U;
    uint16_t v;

    /* CTL0: ARSE */
    v = 0x0080;
    __asm volatile ("strh %0, [%1, #0]" :: "r"(v), "r"(tb));

    /* CAR = period-1 */
    __asm volatile ("ldrsh %0, [%1, #4]" : "=r"(v) : "r"(ctrl));
    {
        uint32_t car = (uint32_t)(int32_t)(int16_t)v - 1;
        __asm volatile ("str %0, [%1, #44]" :: "r"(car), "r"(tb) : "memory");
    }

    /* CHCTL2: CH0E + CH2E */
    {
        uint16_t chctl2 = 0x0404;
        __asm volatile ("strh %0, [%1, #32]" :: "r"(chctl2), "r"(tb));
    }

    /* Dead-time and complementary output config */
    if (ctrl[7] != 0) {
        __asm volatile ("movs %0, #2" : "=r"(v));
        __asm volatile ("strh %0, [%1, #40]" :: "r"(v), "r"(tb));
        __asm volatile ("movs %0, #15" : "=r"(v));
        __asm volatile ("strh %0, [%1, #48]" :: "r"(v), "r"(tb));
    } else {
        __asm volatile ("movs %0, #1" : "=r"(v));
        __asm volatile ("strh %0, [%1, #40]" :: "r"(v), "r"(tb));
        __asm volatile ("movs %0, #23" : "=r"(v));
        __asm volatile ("strh %0, [%1, #48]" :: "r"(v), "r"(tb));
    }

    /* Channel polarity */
    if (ctrl[6] != 0) {
        __asm volatile ("movs %0, #112" : "=r"(v));
        __asm volatile ("strh %0, [%1, #24]" :: "r"(v), "r"(tb));
        __asm volatile ("strh %0, [%1, #28]" :: "r"(v), "r"(tb));
    } else {
        __asm volatile ("movs %0, #96" : "=r"(v));
        __asm volatile ("strh %0, [%1, #24]" :: "r"(v), "r"(tb));
        __asm volatile ("strh %0, [%1, #28]" :: "r"(v), "r"(tb));
    }

    /* Enable output compare shadow: CHCTL0 |= 8, CHCTL1 |= 8 */
    __asm volatile ("ldrh %0, [%1, #24]" : "=r"(v) : "r"(tb));
    v |= 0x0008;
    __asm volatile ("strh %0, [%1, #24]" :: "r"(v), "r"(tb));
    __asm volatile ("ldrh %0, [%1, #28]" : "=r"(v) : "r"(tb));
    v |= 0x0008;
    __asm volatile ("strh %0, [%1, #28]" :: "r"(v), "r"(tb));

    /* Clear compare values */
    {
        uint32_t zero = 0;
        __asm volatile ("str %0, [%1, #52]" :: "r"(zero), "r"(tb) : "memory"); /* 0x34 */
        __asm volatile ("str %0, [%1, #60]" :: "r"(zero), "r"(tb) : "memory"); /* 0x3C */
    }

    /* SWEVG: UG */
    {
        uint16_t one;
        __asm volatile ("movs %0, #1" : "=r"(one));
        __asm volatile ("strh %0, [%1, #20]" :: "r"(one), "r"(tb));
    }

    __asm volatile ("nop");
    __asm volatile ("nop");

    /* DMAINTEN: none */
    {
        uint16_t zero16 = 0;
        __asm volatile ("strh %0, [%1, #16]" :: "r"(zero16), "r"(tb));
    }

    /* Enable counter: CTL0 |= 1 */
    __asm volatile ("ldrh %0, [%1, #0]" : "=r"(v) : "r"(tb));
    v |= 1;
    __asm volatile ("strh %0, [%1, #0]" :: "r"(v), "r"(tb));

    /* Main output enable: CCHP_POEN at offset 0x44 */
    {
        uint16_t poen = 0x8000;
        __asm volatile ("strh.w %0, [%1, #68]" :: "r"(poen), "r"(tb));
    }
}




/* ================================================================
 * External function declarations
 * ================================================================ */

/* uart_protocol.c */

/* eeprom.c */
extern void eeprom_init(uint8_t *ee_ctrl);

/* pid_motor.c */
extern void pid_compute(uint8_t *state);

/* subsystems.c */
extern void encoder_obj_init(uint8_t *ctrl);
extern void encoder_init(void);
extern int32_t encoder_read(uint8_t *ctrl);
extern void encoder_set_initial_pos(uint8_t *ctrl);
extern void gpio_led_init(uint8_t *ctrl);
extern void servo_apply_config(uint8_t *ctrl);
extern void eeprom_page_load_defaults(uint8_t *ctrl);
extern void eeprom_save_regs(uint8_t *ctrl);
extern void reg_write_side_effects(uint8_t *ctrl, uint8_t *regs, int32_t addr, int32_t count);
extern void i2c_hw_init(uint8_t *ctrl);
extern void uart_tx_led_tick(uint8_t *state);
extern void adc_processing(uint8_t *state);
/* motion_profile_update_main removed — logic inlined into
 * position_motion_guard (pid.c) to match original structure. */
extern void motor_obj_init(uint8_t *ctrl);
extern void led_obj_init(uint8_t *ctrl);

/* New sub-functions for main_tick control loop */
extern uint32_t timer_tick_elapsed(uint8_t *ctrl);
extern void encoder_start_read(uint8_t *ctrl);
extern uint32_t encoder_read_complete(uint8_t *ctrl);
extern uint32_t i2c_data_ready(uint8_t *ctrl);
extern uint8_t adc_read_temp(uint8_t *param);
extern uint8_t adc_read_voltage(uint8_t *param);
extern int16_t adc_read_current(uint8_t *param);
extern void motion_step(uint8_t *param);
extern void timer_tick_update(uint8_t *param);
extern void overload_protect(uint8_t *param);
extern void led_blink_tick(uint8_t *param);
extern void pid_output_apply(uint8_t *param);
extern void motor_safety_apply(uint8_t *adc_st);
extern void adc_overload_check(uint8_t *param);
extern void motor_output_apply(uint8_t *adc_st);

/* ================================================================
 * reg_write — Write to servo register table
 *
 * Checks permission bit 1 (writable) for each byte.
 * Clamps address+length to register table size (0x57 = 87).
 * Returns number of bytes written, or -1 on error.
 * ================================================================ */
int __attribute__((noinline)) reg_write(uint32_t addr, uint8_t *data, uint8_t len)
{
    if (addr >= 0x57) return -1;

    uint32_t count = len;
    if (((addr - 1 + count) & 0xFF) > 0x56) {
        count = (0x57 - addr) & 0xFF;
    }

    if (count != 0) {
        uint32_t i = 0;
        do {
            if (reg_permissions[i + addr] & 2) {  /* writable? */
                servo_regs_arr[i + addr] = data[i];
            }
            i = (i + 1) & 0xFF;
        } while (i < count);
    }

    reg_write_side_effects(pwm_ctrl_arr, servo_regs_arr, addr, count);
    return (int)(int16_t)count;
}

/* ================================================================
 * reg_read — Read from servo register table
 *
 * Copies register data to output buffer.
 * Clamps address+length to register table size (0x57 = 87).
 * Returns number of bytes read, or -1 on error.
 * ================================================================ */
int __attribute__((noinline)) reg_read(uint32_t addr, uint8_t *out, uint8_t len)
{
    if (addr > 0x56) return -1;

    uint32_t count = len;
    if (((addr - 1 + count) & 0xFF) > 0x56) {
        count = (0x57 - addr) & 0xFF;
    }

    if (count != 0) {
        uint32_t i = 0;
        do {
            out[i] = servo_regs_arr[i + addr];
            i = (i + 1) & 0xFF;
        } while (i < count);
    }

    return (int)(int16_t)count;
}

/* Force-reference functions that are called via vtables or indirect paths
 * to prevent the linker from garbage-collecting them.
 * We use a volatile function pointer read in a never-executed path so the
 * linker sees a reference but no runtime cost. */
extern void encoder_set_initial_pos(uint8_t *ctrl);
extern void factory_reset_handler(uint8_t *ctrl);
extern void sync_write_accumulate(uint8_t *state, uint32_t byte, uint8_t idx);
extern void uart_tx_force_start(uint8_t *state);
extern void dma_tx_start(uint8_t *state, uint32_t addr, uint32_t len);
extern void eeprom_save_regs(uint8_t *ctrl);
extern void reg_change_detect(uint8_t *state, uint32_t addr);
extern void eeprom_page_header_write(uint8_t *ctrl);
extern uint32_t ring_buf_empty(uint8_t *ring);
extern uint32_t position_linearize(uint32_t raw);
extern void encoder_track_multiturn(int32_t *enc_state);
extern void encoder_multiturn_track(int32_t *enc_state);
extern void subsys_memcpy(void *dst, const void *src, int count);
extern void ring_buf_init(uint8_t *ring);
extern void vtable_setup(void);
extern int zero_init_helper(uint8_t *param);
extern void eeprom_save_byte_internal(uint8_t *ee_ctrl);
extern void eeprom_save_byte(uint8_t *ee_ctrl, uint8_t reg_addr);
extern uint32_t *i2c_obj_init(uint32_t *param);
extern void motion_helper_reset(uint8_t *param);

/* New missing functions from subsystems.c */
extern void pwm_composite_init(uint8_t *param);
extern void i2c_mode_init(uint8_t *param);
extern void i2c_send_addr_write(uint8_t *param);
extern void i2c_send_addr_read(uint8_t *param);
extern void gpio_led_alarm_set(void);
extern void gpio_led_state_set(void);
extern void gpio_led_enable(void);
extern void timer15_init_wrapper(uint8_t *param);
extern void encoder_init_wrapper(void);
extern void servo_regs_init_wrapper(uint8_t *param);
extern void motion_profile_ramp(int32_t *state);
extern void motion_profile_zero(int32_t *state);


/* GC'd init thunks — keep alive */
extern void encoder_ctrl_init(void);
extern void pwm_ctrl_zero_init(void);
extern void timer_ctrl_init(void);
extern void timer_ctrl_ring_init(void);
extern void uart_state_obj_init(void);
extern void i2c_ctrl_init(void);
extern void i2c_error_check(uint8_t *param);
extern void i2c_state_reset(void);
extern void adc_iir_filter(int32_t *accum, int32_t new_val);
extern void eeprom_header_write_wrap(void);
extern void pid_state_reset(void);

/* Indirectly-called functions are kept alive via section(".text.keep")
 * attribute + KEEP(.text.keep*) in the linker script. */

/* ================================================================
 * position_linearize — Apply linearization to raw encoder
 *
 * Converts 12-bit raw encoder value to linearized position.
 * Handles wraparound at 0x1000 (4096 counts per revolution).
 * ================================================================ */
uint32_t __attribute__((noinline)) position_linearize(uint32_t raw)
{
    /* Original takes param in r1, creates uxth r0=r1, uses r1 for signed tests.
     * Force separate signed/unsigned variables to match. */
    volatile int32_t sval = (int32_t)raw;
    volatile uint32_t uval = raw & 0xFFFF;

    if (((uval + ENCODER_MAX_VAL) & 0xFFFF) < ENCODER_HALF_REV) {
        return (int16_t)(ENCODER_FULL_REV - uval);
    }
    if (sval < 0) {
        return (int16_t)(ENCODER_HALF_REV - uval);
    }
    if (sval < ENCODER_HALF_REV) {
        /* Original: mov r0, r1; bx lr (separate return, not shared) */
        __asm__ volatile ("nop");
        return sval;
    }
    if (sval < ENCODER_FULL_REV) {
        return (int16_t)(uval + ENCODER_HALF_REV);
    }
    return 0;
}

/* ================================================================
 * encoder_track_multiturn — Multi-turn encoder tracking
 *
 * Tracks cumulative position across multiple revolutions.
 * Detects forward/backward wrap at 0xFED threshold.
 * ================================================================ */
void __attribute__((noinline, section(".text.keep"))) encoder_track_multiturn(int32_t *enc_state)
{
    /* enc_state[0] = vtable pointer
     * enc_state[1] = accumulated position (multi-turn)
     * enc_state[2] = revolution counter
     * enc_state[3] = current raw position
     * enc_state[4] = previous raw position */

    /* Read current encoder value via vtable */
    int32_t current = enc_state[3];  /* TODO: vtable call */

    enc_state[3] = current;
    int32_t delta = current - enc_state[4];

    if (delta < (int32_t)0xFED) {
        if (delta < -0x13) {  /* TODO: check threshold from literal pool */
            enc_state[2]++;  /* forward revolution */
        }
    } else {
        enc_state[2]--;  /* backward revolution */
    }

    enc_state[4] = current;
    enc_state[1] = current + enc_state[2] * ENCODER_FULL_REV;
}

/* ON-phase current peak — written by TIMER0 update ISR, read by mode 4 PID */
volatile uint16_t adc_on_phase_peak;

/* Software flag set by the ISR when a full sweep (current + temp + voltage)
 * has been completed. main_tick / i2c_data_ready check this. */
volatile uint8_t adc_isr_data_ready;

/* Software replacement for TIMER0 UPIF — the ISR clears the hardware flag,
 * so timer_tick_elapsed can't see it. This flag restores that signal path. */
volatile uint8_t timer_tick_flag;

/* TIMER0 update interrupt counter — used to time-multiplex voltage/temp
 * sampling at lower rates than current. */
static volatile uint16_t timer0_isr_counter;

/* ================================================================
 * TIMER0 update interrupt handler — runs at PWM cycle rate (24kHz).
 *
 * Fires at the start of each new PWM cycle (counter overflow).
 * Triggers a single ADC SWRCST conversion to sample motor current at
 * the very start of the ON phase. Time-multiplexes voltage and temp
 * sampling on every Nth cycle.
 *
 * Writes results into the existing i2c_ctrl_arr DMA buffer slots so
 * that adc_read_temp/voltage/current keep working unchanged:
 *   slot 0 = temperature  (PA1, ADC CH1)
 *   slot 1 = current      (PA3, ADC CH3)
 *   slot 2 = voltage      (PA4, ADC CH4)
 *
 * Sets adc_isr_data_ready every 8 cycles so the main_tick step 3 fires
 * at ~3kHz (matching the original ~24kHz ÷ 8 cadence).
 * ================================================================ */
void __attribute__((interrupt)) TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
    volatile uint32_t *timer0 = (volatile uint32_t *)0x40012C00U;
    volatile uint32_t *adc = (volatile uint32_t *)0x40012400U;

    /* Clear TIMER0 UPIF (bit 0 of INTF, rc_w0) */
    timer0[0x10/4] = ~1U;

    /* Set software tick flag — timer_tick_elapsed() checks this instead
     * of the hardware UPIF that we just cleared. */
    timer_tick_flag = 1;

    /* If a flash operation is in progress, skip this sample.
     * The flash controller stalls instruction fetch during erase/program,
     * which means our busy-wait below could interact unpredictably with
     * the previous EEPROM save's wait state. Just bail and try next time. */
    {
        volatile uint32_t *fmc = (volatile uint32_t *)0x40022000U;
        if (fmc[3] & 1) {  /* FMC_STAT BSY bit */
            return;
        }
    }

    /* Discard any leftover EOC from a previous interrupted conversion */
    if (adc[0] & 2) {
        (void)adc[0x4C/4];
        adc[0] &= ~2U;
    }

    /* Pick channel based on counter:
     *   Every 256 cycles → temperature  (~94 Hz)
     *   Every 8 cycles   → voltage      (~3 kHz)
     *   Otherwise        → current      (~21 kHz)
     */
    uint16_t cnt = timer0_isr_counter;
    uint8_t channel;
    uint8_t buf_slot;
    if ((cnt & 0xFF) == 0) {
        channel = 1;
        buf_slot = 0;
    } else if ((cnt & 7) == 0) {
        channel = 4;
        buf_slot = 2;
    } else {
        channel = 3;
        buf_slot = 1;
    }
    timer0_isr_counter = cnt + 1;

    /* Configure ADC for single conversion of selected channel */
    adc[0x2C/4] = 0;                       /* RSQ0: RL=0 (1 conversion) */
    adc[0x34/4] = (uint32_t)channel;       /* RSQ2: position 0 = channel */

    /* Trigger SWRCST (bit 22 of CTL1) via bit-banding */
    *(volatile uint32_t *)0x42248158U = 1U;

    /* Wait for end-of-conversion (EOC = bit 1 of STAT) — bounded busy-wait */
    for (int w = 0; w < 200; w++) {
        if (adc[0] & 2) break;
    }

    /* Read result and write to the i2c_ctrl_arr DMA buffer slot
     * (existing adc_read_* functions read from there). */
    if (adc[0] & 2) {
        uint16_t value = (uint16_t)(adc[0x4C/4] & 0xFFF);
        adc[0] &= ~2U;
        *(volatile uint16_t *)(i2c_ctrl_arr + 4 + buf_slot * 2) = value;
        if (channel == 3) {
            adc_on_phase_peak = value;
        }
    }

    /* Set data_ready flag every cycle — factory DMA fired at 24kHz
     * (TIMER0_CH1 triggered ADC scan, DMA completed once per PWM cycle).
     * The original "every 8 cycles" assumption was wrong and caused
     * the speed PID to run 8x too slowly. */
    adc_isr_data_ready = 1;
}

/* Main tick — called every iteration of the main loop. */
void __attribute__((noinline)) main_tick(void)
{
    /* Main tick flow matches original binary at 0x0F5C.
     * Step1_action → goto Step 2 (b F66)
     * Step2_action → goto Step 3 (b F6E)
     * Phase1 set → goto Step4_check
     * Step4_check: if PHASE0, step4; else step5 */

    /* Step 1: Check TIMER0 update interrupt for tick.
     * ON-phase current sensing now happens in TIMER0_IRQHandler at the
     * exact moment of the timer update event (no main_tick polling delay,
     * no race with the regular ADC scan). */
    {
        uint32_t result = timer_tick_elapsed(timer_ctrl_arr);
        if (result != 0) {
            encoder_start_read(encoder_ctrl_arr);
        }
    }

    /* Step 2: Check encoder read complete */
    {
        uint32_t result = encoder_read_complete(encoder_ctrl_arr);
        if (result != 0) {
            uint32_t enc_val = (uint32_t)encoder_read(encoder_ctrl_arr);
            *(uint32_t *)(pwm_ctrl_arr + PWM_ENCODER_POS) = enc_val;
            adc_processing(adc_state_arr);
            adc_overload_check(adc_state_arr);
            i2c_ctrl_arr[I2C_PHASE0] = 1;
            /* Original: b F6E — goto Step 3 (falls through naturally) */
        }
    }

    /* Step 3: Check I2C DMA data ready */
    {
        uint32_t result = i2c_data_ready(i2c_ctrl_arr);
        if (result != 0) {
            servo_regs_arr[SR_TEMPERATURE] = adc_read_temp(i2c_ctrl_arr);
            servo_regs_arr[SR_VOLTAGE] = adc_read_voltage(i2c_ctrl_arr);
            *(int16_t *)(pwm_ctrl_arr + PWM_CURRENT_SENSE) = adc_read_current(i2c_ctrl_arr);

            /* Expose ON-phase current peak at regs 73-74 (RO).
             * This is the raw 12-bit ADC reading from the shunt resistor,
             * sampled at the start of each PWM cycle by the TIMER0 ISR.
             * Used by mode 4 PI loop and readable by host for current
             * monitoring, calibration, and auto-tuning. */
            *(uint16_t *)(servo_regs_arr + 73) = adc_on_phase_peak;
            motion_step(timer_ctrl_arr + TMR_MOTION_BASE);
            timer_tick_update(timer_ctrl_arr + TMR_MOTION_BASE);
            overload_protect(timer_ctrl_arr + TMR_MOTION_BASE);
            led_blink_tick(gpio_ctrl_arr);
            pid_output_apply(pwm_ctrl_arr);
            motor_safety_apply(adc_state_arr);

            i2c_ctrl_arr[I2C_PHASE1] = 1;
            /* Fall through to step4_check (original: same) */
        }
    }

    /* Phase1 gate → Step 4 check (original: F76-F7C then 1002-1020) */
    if (i2c_ctrl_arr[I2C_PHASE1] != 0) {
        if (i2c_ctrl_arr[I2C_PHASE0] != 0) {
            i2c_ctrl_arr[I2C_PHASE1] = 0;
            i2c_ctrl_arr[I2C_PHASE0] = 0;

            pid_compute(pid_state_arr);
            motor_output_apply(adc_state_arr);
        }
    }

    /* Step 5: UART TX dispatch + process */
    {
        volatile uint8_t *mc = motor_ctrl;
        uart_tx_led_tick((uint8_t *)mc);
    }
}

/* ================================================================
 * main
 * ================================================================ */
/* ================================================================
 * vtable_store_all — Store vtable pointers into subsystem structs
 *
 * Separated from main for clarity.
 * The original firmware does this inside each subsystem's constructor,
 * but we do it in one place to prevent --gc-sections from removing
 * vtable-called functions.
 * ================================================================ */
static void __attribute__((noinline)) vtable_store_all(void)
{
    extern const void *encoder_vtable[];
    extern const void *uart_obj_vtable[];
    extern const void *i2c_init_vtable[];
    extern const void *motor_calibrate_vtable[];
    extern const void *gpio_vtable[];

    /* Store vtable pointers matching what the original's constructors write.
     * motor_ctrl gets motor_calibrate_vtable at boot (NOT motor_active_vtable).
     * motor_active_vtable is installed later when torque is enabled.
     * Using the wrong vtable here breaks UART register change detection:
     * motor_calibrate_vtable[1] = uart_parse_byte (detects reg changes)
     * motor_active_vtable[1]    = timer15_tick_handler (wrong!) */
    *(const void **)(encoder_ctrl_arr) = encoder_vtable;
    *(const void **)(uart_state_arr) = uart_obj_vtable;
    *(const void **)(i2c_ctrl_arr) = i2c_init_vtable;
    *(const void **)(motor_ctrl_arr) = motor_calibrate_vtable;
    *(const void **)(gpio_ctrl_arr) = (const void *)gpio_vtable;

    /* Set SYNC_ID=1 so the first UART TX can fire.
     * Original calls ring_buf_vtable[1] (uart_sync_set) during boot. */
    extern void uart_sync_set(uint8_t *);
    uart_sync_set(uart_state_arr);

}

/* ================================================================
 * pwm_init — Composite PWM initialization.
 *
 *. 22 bytes in original.
 * ================================================================ */
static void __attribute__((noinline)) pwm_init(uint8_t *param)
{
    /* Original 0xBFC calls: pwm_composite_init + 2 config helpers.
     * It does NOT call timer0_hw_init — that is called from timer15_init_wrapper. */
    pwm_composite_init(param);
}

/* ================================================================
 * encoder_init — Initialize encoder subsystem.
 *
 * ================================================================ */
static void __attribute__((noinline)) encoder_init_all(uint8_t *param)
{
    (void)param;
    encoder_obj_init(encoder_ctrl_arr);
    encoder_init();             /* I2C hardware setup (GPIO, clock, timing, enable) */
    /* Do NOT call encoder_init_wrapper/i2c_configure here.
     * The original does not start an I2C transaction during init.
     * The first transaction starts from main_tick → encoder_start_read. */
}

/* ================================================================
 * led_init — GPIO LED initialization.
 *
 * ================================================================ */
static void __attribute__((noinline)) led_init(uint8_t *param)
{
    extern const void *gpio_vtable[];
    led_obj_init(param);
    /* Original constructor at 0x478 stores gpio_vtable before gpio_led_init
     * is called, so vtable[2] = gpio_led_state_set (writes GPIOF_BOP = 1).
     * Without this, led_obj_vtable[2] = vtable_noop. */
    *(const void **)param = (const void *)gpio_vtable;
    gpio_led_init(param);
    servo_apply_config(pwm_ctrl_arr);
}

int main(void)
{
    /* Enable AHB + DMA clocks and pulse APB1 reset.
     * Original keeps RCU base in r3 for both AHBEN (+0x14) and APB1RST (+0x10). */
    {
        volatile uint32_t * volatile rcu = (volatile uint32_t *)0x40021000U;
        rcu[5] |= RCU_AHBEN_PAEN | RCU_AHBEN_PBEN | RCU_AHBEN_PFEN;
        rcu[5] |= RCU_AHBEN_DMAEN;
        rcu[4] |= 0x00020100U;
        rcu[4] &= ~0x00020100U;
    }

    /* Set NVIC priority grouping */
    nvic_priority_group_config(0x500U);

    /* Initialize subsystems in order (matching original at 0x1054)
     * Use _arr arrays directly so compiler embeds SRAM address in literal pool
     * (one LDR) instead of going through pointer variable (two LDRs).
     * Use volatile pointers to prevent constprop from inlining the address. */
    {
        volatile uint8_t *p;
        /* Original has constructors (.init_array) that pre-initialize
         * subsystem structs before main(). Call them here first. */
        /* .init_array constructors — these run before main() in the original
         * (ARMCC C++ static init). Our GCC build doesn't auto-call them,
         * so call explicitly here after .bss is zeroed. */
        /* uart_state_obj_init, timer_ctrl_init, i2c_ctrl_init now run as
         * .init_array constructors before main(), matching original binary. */

        /* Init sequence matching original at 0x1054:
         * 1. eeprom_init(eeprom_ctrl)
         * 2. pwm_init(pwm_ctrl) — composite init only, no timer0_hw_init
         * 3. timer15_init_wrapper(timer_ctrl) — sets ctrl[6,7], calls timer0_hw_init
         * 4. i2c_mode_init(i2c_ctrl) — ADC channel config + hw init
         * 5. encoder_init_all(encoder_ctrl)
         * 6. servo_regs_init_wrapper(uart_state)
         * 7. motor_obj_init(motor_ctrl) — enables TIMER15 clock + config
         * 8. led_init(gpio_ctrl) */
        p = (volatile uint8_t *)eeprom_ctrl_arr; eeprom_init((uint8_t *)p);
        p = pwm_ctrl;    pwm_init((uint8_t *)p);
        p = timer_ctrl;  timer15_init_wrapper((uint8_t *)p);

        /* Enable TIMER0 update interrupt BEFORE i2c_mode_init, so the
         * ADC ISR is running during the current-sense warmup baseline
         * calibration. Without this, the warmup reads zeros from the
         * pre-loaded DMA buffer, baseline calibrates to 0, and every
         * subsequent idle read looks like current → false overload. */
        *(volatile uint32_t *)0xE000E100 |= 0x00002000U;  /* NVIC ISER0 bit 13 */
        *(volatile uint16_t *)(0x40012C00U + 0x0C) |= 0x0001U;  /* TIMER0 UPIE */

        p = i2c_ctrl;    i2c_mode_init((uint8_t *)p);
        p = encoder_ctrl; encoder_init_all((uint8_t *)p);
        /* Original reads servo_regs[6] (baud index) and passes as r1 */
        {
            volatile uint8_t baud = servo_regs_arr[6];
            (void)baud;
            p = (volatile uint8_t *)uart_state_arr; servo_regs_init_wrapper((uint8_t *)p);
        }
        p = motor_ctrl;  motor_obj_init((uint8_t *)p);
        /* adc_state_init: set counter to 0xFF so first adc_processing call
         * captures the initial encoder position snapshot (original at 0x08002C18) */
        adc_state_arr[1] = 0xFF;
        adc_state_arr[0] = 0;
        *(uint32_t *)(adc_state_arr + 4) = 0;
        p = gpio_ctrl;   led_init((uint8_t *)p);
    }
    /* Original passes args explicitly (not constprop) */
    {
        volatile uint32_t reload = 0x96U;
        volatile uint32_t prescaler = 6U;
        fwdgt_setup(reload, prescaler);
    }

    /* Store vtable pointers (done after init, before main loop) */
    vtable_store_all();

    /* Ensure lock=0 AFTER all init (including eeprom_page_load_defaults).
     * On blank-EEPROM boot, the EEPROM load copies 0xFF into sr[55],
     * which disables all EEPROM writes via reg_write_side_effects.
     * Must happen after init so it overrides the garbage from EEPROM. */
    servo_regs_arr[SR_LOCK] = 0;

    /* Ensure USART1 IRQ is enabled in NVIC — the init sequence should
     * do this via gpio_config_pin in usart1_dma_init, but verify it
     * wasn't accidentally disabled by error handlers during boot. */
    *(volatile uint32_t *)0xE000E100 = 0x10000000U;  /* NVIC_ISER bit 28 */

    /* TIMER0 ISR already enabled before i2c_mode_init (see above). */


    /* Main loop — original: str 0xAAAA to FWDGT_CTL, bl main_tick, b loop */
    while (1) {
        FWDGT_CTL = FWDGT_KEY_RELOAD;
        main_tick();
    }
}

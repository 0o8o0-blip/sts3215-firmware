/*
 * Feetech STS3215 Servo Firmware — Type Definitions
 *
 * Packed struct definitions for all firmware state arrays.
 * These match the original firmware's SRAM layout exactly.
 *
 * Each struct corresponds to a global uint8_t[] array in the firmware.
 * The arrays are accessed via raw byte offsets in the original binary;
 * these structs provide named fields for readability.
 *
 * Target: GD32F130C6T6 (ARM Cortex-M3, 48MHz, 32KB flash, 4KB SRAM)
 */

#ifndef SERVO_TYPES_H
#define SERVO_TYPES_H

#include <stdint.h>
#include <stddef.h>

/* ================================================================
 * Peripheral base addresses
 * ================================================================ */
#ifndef GPIOA_BASE
#define GPIOA_BASE   0x48000000U
#endif
#ifndef GPIOB_BASE
#define GPIOB_BASE   0x48000400U
#endif
#ifndef GPIOF_BASE
#define GPIOF_BASE   0x48001400U
#endif
#ifndef TIMER0_BASE
#define TIMER0_BASE  0x40012C00U
#endif
#ifndef TIMER15_BASE
#define TIMER15_BASE 0x40014400U
#endif
#ifndef USART1_BASE
#define USART1_BASE  0x40004400U
#endif
#ifndef I2C0_BASE
#define I2C0_BASE    0x40005400U
#endif
#ifndef DMA_BASE
#undef DMA_BASE
#define DMA_BASE     0x40020000U
#endif
#define DMA_CH0_BASE 0x40020008U
#define DMA_CH1_BASE 0x40020044U   /* USART1 RX */
#define DMA_CH2_BASE 0x40020058U   /* USART1 TX */
#ifndef SYSCFG_BASE
#undef SYSCFG_BASE
#define SYSCFG_BASE  0x40010000U
#endif

/* ================================================================
 * Register access macros
 * ================================================================ */
#ifndef REG
#define REG(base, off)     (*(volatile uint32_t *)((base) + (off)))
#endif
#undef REG16
#define REG16(base, off)   (*(volatile uint16_t *)((base) + (off)))

/* RELOAD() macro removed — replaced by volatile pointer pattern.
 * Each global array (servo_regs_arr, pwm_ctrl_arr, etc.) has a
 * companion volatile pointer (servo_regs, pwm_ctrl, etc.) that
 * provides the same reload guarantee when read. */

/* ================================================================
 * Servo Register Map (servo_regs_arr, 88 bytes)
 *
 * This is the Feetech protocol register table. Each servo has
 * 88 registers accessible via the UART protocol (READ/WRITE).
 *
 * Registers 0-39 are EEPROM-backed (persist across power cycles).
 * Registers 40-55 are runtime RW (goal position, speed, etc.).
 * Registers 56-77 are read-only status (present position, temp, etc.).
 * Registers 78-86 are extended EEPROM-backed config.
 *
 * Original SRAM address: 0x200000E8
 * ================================================================ */

/* Servo register offset constants */
#define SR_FW_VERSION_0     0    /* Firmware version byte 0 */
#define SR_FW_VERSION_1     1    /* Firmware version byte 1 */
#define SR_FW_VERSION_2     2    /* Firmware version byte 2 (model low) */
#define SR_FW_VERSION_3     3    /* Firmware version byte 3 (model high) */
#define SR_BAUD_RATE        6    /* Baud rate index (0-7) — register 6, NOT 4 */
#define SR_RETURN_DELAY     5    /* Response delay time */
#define SR_SERVO_ID         6    /* Servo ID (0-253) */
#define SR_RESERVED_07      7
#define SR_ALARM_LED        8    /* Alarm LED enable */
#define SR_CW_ANGLE_LO      9    /* CW angle limit low byte */
#define SR_CW_ANGLE_HI     10    /* CW angle limit high byte */
#define SR_CCW_ANGLE_LO    11    /* CCW angle limit low byte */
#define SR_CCW_ANGLE_HI    12    /* CCW angle limit high byte */
#define SR_MAX_TEMP         13    /* Max temperature threshold (default 0x50 = 80C) */
#define SR_MAX_VOLTAGE      14    /* Max voltage threshold (default 0x50) */
#define SR_MIN_VOLTAGE      15    /* Min voltage threshold (default 0x28) */
#define SR_MAX_TORQUE_LO   16    /* Max torque low byte */
#define SR_MAX_TORQUE_HI   17    /* Max torque high byte */
#define SR_CONFIG           18    /* Config byte: bit5=direction, bit4=sign, bit2=scale */
#define SR_ALARM_SHUTDOWN   19    /* Alarm shutdown mask */
#define SR_ALARM_CONFIG     20    /* Which alarms blink LED */
#define SR_KP_GAIN          21    /* Position P gain (default 0x20) */
#define SR_KD_GAIN_SCALE    22    /* D gain scaling (default 0x20) */
#define SR_KI_GAIN          23    /* I gain coefficient (default 4) */
#define SR_PUNCH_MIN        24    /* Minimum punch/force (default 0x10) */
#define SR_KD_GAIN          25    /* D gain (default 0) */
#define SR_CW_DEADZONE      26    /* CW dead zone (default 2) */
#define SR_CCW_DEADZONE     27    /* CCW dead zone (default 2) */
#define SR_MAX_CURRENT_LO  28    /* Max current limit low byte */
#define SR_MAX_CURRENT_HI  29    /* Max current limit high byte */
#define SR_PUNCH_DIVISOR    30    /* Punch divisor (default 1, must not be 0) */
#define SR_GOAL_OFFSET_LO  31    /* Goal position offset low */
#define SR_GOAL_OFFSET_HI  32    /* Goal position offset high */
#define SR_OPERATING_MODE   33    /* 0=position, 1=speed, 2=PWM, 3=multi-turn, 4=current */
#define SR_MAX_SPEED        34    /* Max speed limit (default 0x28) */
#define SR_ACCEL_LO         35    /* Acceleration low byte */
#define SR_ACCEL_HI         36    /* Acceleration high byte */
#define SR_KI_SPEED         37    /* Speed mode I gain (default 0x19) */
#define SR_RESERVED_26      38
#define SR_OVERLOAD_RATIO   39    /* Overload ratio (default 0x32) */
#define SR_TORQUE_ENABLE    40    /* 0x28: torque enable (1=on, 0x80=reboot flag) */
#define SR_ACCELERATION     41    /* 0x29: acceleration for rate limiting */
#define SR_GOAL_POS_LO      42    /* 0x2A: goal position low byte */
#define SR_GOAL_POS_HI      43    /* 0x2B: goal position high byte */
#define SR_PWM_GOAL_LO      44    /* 0x2C: PWM mode goal (sign-magnitude, bit 10=sign) */
#define SR_PWM_GOAL_HI      45    /* 0x2D: PWM mode goal high byte */
#define SR_GOAL_SPEED_LO    46    /* 0x2E: goal speed low byte */
#define SR_GOAL_SPEED_HI    47    /* 0x2F: goal speed high byte */
#define SR_MAX_OUTPUT_LO    48    /* 0x30: max output torque low byte */
#define SR_MAX_OUTPUT_HI    49    /* 0x31: max output torque high byte */
#define SR_RESERVED_32      50
#define SR_RESERVED_33      51
#define SR_RESERVED_34      52
#define SR_RESERVED_35      53
#define SR_RESERVED_36      54
#define SR_LOCK             55    /* 0x37: lock flag (non-zero skips side effects) */
#define SR_PRESENT_POS_LO  56    /* 0x38: present position low (RO) */
#define SR_PRESENT_POS_HI  57    /* 0x39: present position high (RO) */
#define SR_PRESENT_LOAD_LO 58    /* 0x3A: present load low (RO) */
#define SR_PRESENT_LOAD_HI 59    /* 0x3B: present load high (RO) */
#define SR_PRESENT_SPD_LO  60    /* 0x3C: present speed low (RO) */
#define SR_PRESENT_SPD_HI  61    /* 0x3D: present speed high (RO) */
#define SR_VOLTAGE          62    /* 0x3E: present voltage (RO) */
#define SR_TEMPERATURE      63    /* 0x3F: present temperature (RO) */
#define SR_ACTION_PENDING   64    /* 0x40: action pending flag */
#define SR_ERROR_FLAGS      65    /* 0x41: error flags (voltage|I2C|temp|overload) */
#define SR_RESERVED_42      66
#define SR_PRESENT_GOAL_LO 67    /* 0x43: present goal low (RO) */
#define SR_PRESENT_GOAL_HI 68    /* 0x44: present goal high (RO) */
#define SR_PRESENT_VEL_LO  69    /* 0x45: present velocity low (RO) */
#define SR_PRESENT_VEL_HI  70    /* 0x46: present velocity high (RO) */
#define SR_PRESENT_OUT_LO  71    /* 0x47: present output low (RO) */
#define SR_PRESENT_OUT_HI  72    /* 0x48: present output high (RO) */
/* gap: 73-79 (0x49-0x4F) unused */
#define SR_SPEED_DEADZONE   80    /* 0x50: velocity tracking dead zone */
#define SR_VEL_COUNTER      81    /* 0x51: velocity sample counter threshold */
#define SR_PUNCH_DIVISOR2   82    /* 0x52: alternative punch divisor */
#define SR_TORQUE_FACTOR    83    /* 0x53: max torque = factor * punch_div * 10 */
#define SR_PUNCH_VALUE      84    /* 0x54: minimum drive force */
#define SR_MOVING_SPEED     85    /* 0x55: moving speed for rate limiting */
#define SR_SPEED_MULT       86    /* 0x56: speed multiplier */

#define SERVO_REGS_SIZE     88

/* ================================================================
 * Encoder Control (encoder_ctrl_arr, 36 bytes)
 *
 * Holds encoder state for multi-turn position tracking.
 * Functions receive this as int32_t* param (9 int32 fields).
 *
 * Original SRAM address: 0x200000BC
 * ================================================================ */
#define ENC_VTABLE           0    /* uint32_t: encoder vtable pointer */
#define ENC_ACCUM_POS        4    /* int32_t: accumulated position (param[1]) */
#define ENC_REVOLUTIONS      8    /* int32_t: revolution counter (param[2]) */
#define ENC_RAW_VALUE       12    /* int32_t: raw encoder value (param[3]) */
#define ENC_PREV_RAW        16    /* int32_t: previous raw value (param[4]) */
#define ENC_LAST_RESULT     20    /* int32_t: last filtered result (param[5]) */
#define ENC_INIT_FLAG       24    /* uint8_t: init flag at byte offset (param[6] as uint8) */
#define ENC_IIR_ACCUM       28    /* int32_t: IIR filter accumulator (param[7]) */
#define ENC_POSITION_VAL    0x1C  /* uint32_t: encoder value register */
#define ENC_DELINEAR        0x20  /* uint32_t: delinearized position target */

#define ENCODER_CTRL_SIZE   36

/* ================================================================
 * GPIO/LED Control (gpio_ctrl_arr, 8 bytes)
 *
 * LED blink state for status/alarm indication.
 * vtable at offset 0 dispatches to gpio_vtable or led_obj_vtable.
 *
 * Original SRAM address: 0x200000E0
 * ================================================================ */
#define GPIO_VTABLE          0    /* uint32_t: vtable pointer */
#define GPIO_LED_STATE       4    /* uint8_t: LED toggle state (0 or 1) */
#define GPIO_LED_COUNTER     5    /* uint8_t: blink tick counter (0-99) */

#define GPIO_CTRL_SIZE       8

/* ================================================================
 * PWM Control (pwm_ctrl_arr, 48 bytes)
 *
 * Motor PWM output state and position tracking.
 *
 * Original SRAM address: 0x20000140
 * ================================================================ */
#define PWM_VTABLE           0    /* uint32_t: vtable pointer */
#define PWM_CURRENT_SPEED    4    /* int16_t: current motor speed */
#define PWM_OUTPUT           6    /* int16_t: PWM output command */
#define PWM_MODE_FLAG        7    /* uint8_t: mode flag (overlaps output high byte) */
#define PWM_ENCODER_POS      8    /* uint32_t: encoder position snapshot */
#define PWM_GOAL_POS        0x0C  /* int32_t: goal position */
#define PWM_SPEED_PROFILE   0x10  /* int32_t: computed speed profile output */
#define PWM_CURRENT_SENSE   0x14  /* int16_t: motor current (ADC reading) */
#define PWM_OP_MODE         0x16  /* uint8_t: operating mode copy */
#define PWM_MIN_SPEED       0x24  /* uint32_t: minimum speed threshold */
#define PWM_PUNCH_DIV       0x2C  /* int32_t: punch divisor for speed calc */

#define PWM_MAX_TORQUE      0x28  /* int32_t: max torque output limit */

#define PWM_CTRL_SIZE        48

/* ================================================================
 * Timer Control (timer_ctrl_arr, 112 bytes)
 *
 * Timer state + motion profile sub-state at offset 0x0C.
 * The motion sub-state extends 84 bytes and overlaps with the next
 * array in the original SRAM layout.
 *
 * Original SRAM address: 0x20000170
 * ================================================================ */
#define TMR_VTABLE           0    /* uint32_t: vtable pointer */
#define TMR_RING_BUF         4    /* ring buffer struct embedded here */
#define TMR_MOTION_BASE     0x0C  /* motion profile sub-state base */

/* Offsets within the motion sub-state (relative to timer_ctrl_arr) */
#define TMR_MOT_COUNTER     0x0C  /* uint8_t: motion state counter */
#define TMR_MOT_VOLT_TICK   0x10  /* uint16_t: voltage check tick counter */
#define TMR_MOT_TEMP_TICK   0x12  /* uint16_t: overtemp counter */
#define TMR_MOT_LOAD_TICK   0x14  /* uint16_t: overload counter */
#define TMR_MOT_STALL_RUN   0x0E  /* uint8_t: stall running flag */
#define TMR_MOT_SPEED_SNAP  0x16  /* int16_t: speed snapshot at start */
#define TMR_SPEED_ACCUM     0x18  /* int32_t: accumulated speed */

#define TIMER_CTRL_SIZE      112  /* 28 + 84 motion sub-state */

/* ================================================================
 * UART Protocol State (uart_state_arr, 278 bytes)
 *
 * Ring buffer + protocol parser + TX state machine.
 * The named offsets are already defined in uart_protocol.c.
 *
 * Original SRAM address: 0x2000018C
 * ================================================================ */
#define UART_VTABLE         0x000  /* uint32_t: vtable pointer */
#define UART_RING_BUF       0x004  /* ring buffer struct (16 bytes) */
#define UART_PACKET_BUF     0x085  /* received packet data buffer */
#define UART_TX_BUF         0x090  /* transmit buffer */
#define UART_PARAMS_LEFT    0x105  /* remaining params to receive */
#define UART_SERVO_ID       0x106  /* this servo's ID for matching */
#define UART_PARSE_STATE    0x107  /* parser state (0-5) */
#define UART_PKT_ID         0x108  /* received packet ID */
#define UART_PKT_INST       0x109  /* received instruction */
#define UART_PKT_CHECKSUM   0x10A  /* running checksum */
#define UART_HISTORY_0      0x10B  /* byte history [0] */
#define UART_HISTORY_1      0x10C  /* byte history [1] */
#define UART_HISTORY_2      0x10D  /* byte history [2] */
#define UART_PARAM_IDX      0x10E  /* current param index */
#define UART_SYNC_IDX       0x10F  /* sync write sub-index */
#define UART_TX_LEN         0x110  /* bytes in TX buffer */
#define UART_SYNC_ID        0x111  /* sync write: current servo ID */
#define UART_ERROR          0x112  /* error byte for response */
#define UART_PKT_READY      0x113  /* packet dispatch complete flag */
#define UART_TX_STATE       0x114  /* TX state / byte counter */
#define UART_TX_ACTIVE      0x115  /* TX in progress flag */

#define UART_STATE_SIZE      278

/* ================================================================
 * I2C Control (i2c_ctrl_arr, 48 bytes)
 *
 * I2C peripheral state and ADC DMA buffer.
 *
 * Original SRAM address: 0x200002A0
 * ================================================================ */
#define I2C_VTABLE           0    /* uint32_t: vtable pointer */
#define I2C_DMA_BUF          4    /* uint16_t[4]: ADC DMA buffer (8 bytes) */
#define I2C_TEMP_FILTER     0x24  /* uint8_t[4]: temperature filter state */
#define I2C_CURR_BASELINE   0x26  /* int16_t: current sensor baseline */
#define I2C_WARMUP_CTR      0x28  /* int8_t: warmup counter */
#define I2C_TEMP_CH         0x29  /* uint8_t: temperature ADC channel index */
#define I2C_VOLT_CH         0x2A  /* uint8_t: voltage ADC channel index */
#define I2C_CURR_CH         0x2B  /* uint8_t: current ADC channel index */
#define I2C_PHASE0          0x2C  /* uint8_t: ADC phase 0 flag */
#define I2C_PHASE1          0x2D  /* uint8_t: ADC phase 1 flag */

#define I2C_CTRL_SIZE        48

/* ================================================================
 * EEPROM Control (eeprom_ctrl_arr, 528 bytes)
 *
 * EEPROM emulation working buffer + page pointers.
 * First 512 bytes: working buffer for page read/write.
 * Last 16 bytes: page pointers and size.
 *
 * Original SRAM address: 0x200002D0
 * ================================================================ */
#define EE_WORK_BUF          0      /* uint8_t[512]: working buffer */
#define EE_PAGE0_PTR        0x200   /* uint32_t: flash page 0 pointer */
#define EE_PAGE1_PTR        0x204   /* uint32_t: flash page 1 pointer */
#define EE_PAGE_SIZE        0x208   /* uint16_t: page size (512) */

#define EEPROM_CTRL_SIZE     528

/* ================================================================
 * Encoder I2C (encoder_i2c_arr, 28 bytes)
 *
 * I2C transaction state machine for AS5600 magnetic encoder.
 * The encoder is at I2C address 0x36, reading register 0x0C
 * (raw angle high byte, 12-bit value).
 *
 * Original SRAM address: 0x200004DC
 * ================================================================ */
#define EI2C_STATE           0    /* uint8_t: I2C state machine (0=idle, 1-7=active) */
#define EI2C_SLAVE_ADDR      3    /* uint8_t: I2C slave address (0x36 for AS5600) */
#define EI2C_WRITE_DATA      4    /* uint8_t: data byte to write */
#define EI2C_REG_ADDR        5    /* uint8_t: register address to read (0x0C) */
#define EI2C_READ_BUF        6    /* uint8_t[6]: read data buffer */
#define EI2C_DATA_IDX       0x0D  /* uint8_t: data index / frame counter */
#define EI2C_SPEED_STATE    0x10  /* speed computation sub-state (used by pid) */
#define EI2C_SPEED_ACCUM    0x18  /* int32_t: speed accumulator */

#define ENCODER_I2C_SIZE     28

/* ================================================================
 * Motor Control (motor_ctrl_arr, 280 bytes)
 *
 * UART protocol state for motor command channel.
 * Contains a full UART state machine (same layout as uart_state_arr)
 * plus motor-specific vtable.
 *
 * Original SRAM address: 0x200004F8
 * ================================================================ */
/* Uses same UART_* offsets as uart_state_arr since it contains
 * an embedded UART protocol handler */
#define MOTOR_CTRL_SIZE      280

/* ================================================================
 * ADC State (adc_state_arr, 8 bytes)
 *
 * ADC processing state and motion flags.
 *
 * Original SRAM address: 0x20000610
 * ================================================================ */
#define ADC_MOTION_FLAG      0    /* uint8_t: motion/enable flag */
#define ADC_SAMPLE_STATE     1    /* uint8_t: sample counter (0xFF = init) */

#define ADC_STATE_SIZE       8

/* ================================================================
 * PID Controller State (pid_state_arr, 84 bytes)
 *
 * PID controller state with position PID sub-state at offset 0x38.
 *
 * Original SRAM address: 0x20000618
 * ================================================================ */
#define PID_CW_CHANGED       0    /* uint8_t: CW angle limit changed */
#define PID_SPEED_CHANGED    1    /* uint8_t: speed changed flag */
#define PID_CCW_CHANGED      2    /* uint8_t: CCW angle limit changed */
#define PID_ACCEL_CHANGED    3    /* uint8_t: acceleration changed flag */
#define PID_SPEED_LIMIT      8    /* uint32_t: speed limit value */

/* Motion profile integration state */
#define PID_INTEG_64        0x18  /* int64_t: 64-bit integrator */
#define PID_GOAL_64         0x20  /* int64_t: scaled 64-bit goal */
#define PID_PROFILE_RESET   0x28  /* uint8_t: profile reset flag */
#define PID_PREV_GOAL       0x30  /* uint32_t: previous goal position */
#define PID_PREV_SPEED      0x34  /* uint32_t: previous speed */

/* Position PID sub-state at offset 0x38 */
#define PID_POS_BASE        0x38
#define PID_POS_COUNTER     0x38  /* int8_t: sample counter */
#define PID_POS_ERROR       0x3C  /* int32_t: position error */
#define PID_POS_VELOCITY    0x40  /* int32_t: velocity delta */
#define PID_POS_PREV_ERR    0x44  /* int32_t: previous error */
#define PID_POS_INTEGRATOR  0x48  /* int32_t: I-term integrator */
#define PID_POS_ANTIWINDUP  0x4C  /* int32_t: anti-windup residual */
#define PID_POS_DTERM       0x50  /* int32_t: D-term accumulator */

#define PID_STATE_SIZE       84

/* ================================================================
 * Compile-time layout verification
 *
 * These static asserts ensure the struct sizes and key offsets
 * match the original firmware's SRAM layout.
 * ================================================================ */
_Static_assert(SERVO_REGS_SIZE == 88, "servo_regs size mismatch");
_Static_assert(ENCODER_CTRL_SIZE == 36, "encoder_ctrl size mismatch");
_Static_assert(GPIO_CTRL_SIZE == 8, "gpio_ctrl size mismatch");
_Static_assert(PWM_CTRL_SIZE == 48, "pwm_ctrl size mismatch");
_Static_assert(TIMER_CTRL_SIZE == 112, "timer_ctrl size mismatch");
_Static_assert(UART_STATE_SIZE == 278, "uart_state size mismatch");
_Static_assert(I2C_CTRL_SIZE == 48, "i2c_ctrl size mismatch");
_Static_assert(EEPROM_CTRL_SIZE == 528, "eeprom_ctrl size mismatch");
_Static_assert(ENCODER_I2C_SIZE == 28, "encoder_i2c size mismatch");
_Static_assert(MOTOR_CTRL_SIZE == 280, "motor_ctrl size mismatch");
_Static_assert(ADC_STATE_SIZE == 8, "adc_state size mismatch");
_Static_assert(PID_STATE_SIZE == 84, "pid_state size mismatch");

/* Verify critical servo register offsets */
_Static_assert(SR_TORQUE_ENABLE == 40, "SR_TORQUE_ENABLE offset");
_Static_assert(SR_GOAL_POS_LO == 42, "SR_GOAL_POS_LO offset");
_Static_assert(SR_OPERATING_MODE == 33, "SR_OPERATING_MODE offset");
_Static_assert(SR_PRESENT_POS_LO == 56, "SR_PRESENT_POS_LO offset");
_Static_assert(SR_ERROR_FLAGS == 65, "SR_ERROR_FLAGS offset");

/* Verify critical PID offsets */
_Static_assert(PID_POS_BASE == 0x38, "PID_POS_BASE offset");

/* ================================================================
 * EEPROM group mapping table offsets (within pwm_ctrl_arr)
 *
 * servo_regs_load_eeprom builds these tables so that
 * servo_apply_baud / servo_apply_config can remap register
 * addresses to EEPROM page offsets and back.
 *
 * Group 0 = EEPROM registers 0-39, Group 1 = registers 78-86.
 * ================================================================ */
#define EE_GRP_SIZE_BASE    0x17  /* param[grp + 0x17]: group size (count of EEPROM regs) */
#define EE_GRP_SIZE2_BASE   0x18  /* param[grp + 0x18]: group 1 size */
#define EE_GRP_SAVE_SIZE0   0x19  /* param[0x19]: save group 0 size */
#define EE_GRP_SAVE_SIZE1   0x1A  /* param[0x1A]: save group 1 size */
#define EE_GRP_OFFSET_BASE  0x1B  /* param[grp + 0x1B]: group cumulative offset */
#define EE_GRP_SAVE_OFF0    0x1D  /* param[0x1D]: save group 0 offset */
#define EE_GRP_SAVE_OFF1    0x1E  /* param[0x1E]: save group 1 offset */
#define EE_GRP_START_BASE   0x1F  /* param[grp + 0x1F]: group start register index */
#define EE_GRP_START1       0x20  /* param[0x20]: group 1 start register index */

/* ================================================================
 * Encoder constants
 * ================================================================ */
#define ENCODER_FULL_REV    0x1000  /* 4096 counts per revolution (12-bit) */
#define ENCODER_HALF_REV    0x800   /* 2048 counts (half revolution) */
#define ENCODER_MAX_VAL     0xFFF   /* Maximum 12-bit encoder value */
#define ENCODER_WRAP_THRESH 4076    /* Wrap detection threshold (4096 - 20) */

/* Raw angle value offset in encoder_ctrl (stored by encoder_read_complete) */
#define ENC_RAW_ANGLE       0x14    /* int32_t: raw angle from I2C read */

/* ================================================================
 * PWM / protocol bit constants
 * ================================================================ */
#define PWM_GOAL_SIGN_BIT   0x400   /* Bit 10: sign bit in PWM goal (sign-magnitude) */
#define SPEED_GOAL_SIGN_BIT 0x8000  /* Bit 15: sign bit in speed goal (sign-magnitude) */
#define PWM_MAX_DUTY        0x3CA   /* Max duty cycle = 970 */

/* ================================================================
 * Alarm / error bit masks
 * ================================================================ */
#define ALARM_EXTENDED_ERR  0x20    /* Bit 5: extended error (stall detection) */

#endif /* SERVO_TYPES_H */

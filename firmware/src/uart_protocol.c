/*
 * Feetech UART Protocol — Parser, Dispatcher, Response Builder
 *
 * Implements the Feetech/Dynamixel-compatible half-duplex UART protocol.
 * Packet format: [0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMS...][CHECKSUM]
 */

#include "gd32f1x0.h"
#include "servo_types.h"
#include <stdint.h>

/* uart_state struct offsets */
#define UART_VTABLE         0x000   /* function pointer table */
#define UART_RING_BUF       0x004   /* ring buffer struct (16 bytes) */
#define UART_PACKET_BUF     0x085   /* received packet data buffer */
#define UART_PARAMS_BUF     0x085   /* alias: params start after header */
#define UART_TX_BUF         0x090   /* transmit buffer (128 bytes) */
#define UART_PARAMS_LEFT    0x105   /* remaining params to receive */
#define UART_SERVO_ID       0x106   /* this servo's ID for matching */
#define UART_PARSE_STATE    0x107   /* parser state machine (0-5) */
#define UART_PKT_ID         0x108   /* received packet ID */
#define UART_PKT_INST       0x109   /* received packet instruction */
#define UART_PKT_CHECKSUM   0x10A   /* running checksum */
#define UART_HISTORY_0      0x10B   /* byte history [0] */
#define UART_HISTORY_1      0x10C   /* byte history [1] */
#define UART_HISTORY_2      0x10D   /* byte history [2] */
#define UART_PARAM_IDX      0x10E   /* current param index */
#define UART_SYNC_IDX       0x10F   /* sync write sub-index */
#define UART_TX_LEN         0x110   /* bytes in TX buffer */
#define UART_SYNC_ID        0x111   /* sync write: current servo ID */
#define UART_ERROR          0x112   /* error byte for response */
#define UART_PKT_READY      0x113   /* packet dispatch complete flag */
#define UART_TX_STATE       0x114   /* TX state / byte counter */
#define UART_TX_ACTIVE      0x115   /* TX in progress flag */

/* Feetech protocol instructions */
#define INST_PING           0x01
#define INST_READ            0x02
#define INST_WRITE           0x03
#define INST_REG_WRITE       0x04
#define INST_ACTION          0x05
#define INST_FACTORY_RESET   0x06
#define INST_BOOTLOADER      0x08
#define INST_SYNC_WRITE_CLK  0x09
#define INST_SYNC_WRITE_CLK2 0x0A
#define INST_REBOOT          0x0B
#define INST_SYNC_READ       0x82
#define INST_SYNC_WRITE      0x83

/* External references */
extern uint8_t servo_regs_arr[]; extern uint8_t * volatile servo_regs;
extern uint8_t uart_state_arr[]; extern uint8_t * volatile uart_state;
extern uint8_t motor_ctrl_arr[]; extern uint8_t * volatile motor_ctrl;

/* ================================================================
 * uart_packet_parse — Protocol state machine
 *
 * Called from main loop. Reads bytes from ring buffer, parses
 * Feetech packets using a 6-state machine.
 *
 * States:
 *   0: Idle — waiting for 0xFF 0xFF sync
 *   1: Got sync — waiting for ID match
 *   2: Got ID — waiting for length
 *   3: Got length — waiting for instruction
 *   4: Receiving params
 *   5: Waiting for checksum
 *
 * Returns 1 when a complete valid packet is ready for dispatch.
 * ================================================================ */
/* Ring buffer read — returns next byte
 * Takes pointer to ring sub-struct (uart_state + 4).
 * Offsets: +0 = buf_ptr, +4 = rd_idx, +10 = mask */
static uint8_t ring_read(uint8_t *ring)
{
    uint32_t *buf_ptr = (uint32_t *)ring;
    uint16_t *rd_idx = (uint16_t *)(ring + 4);
    uint16_t *mask = (uint16_t *)(ring + 10);

    uint8_t val = ((uint8_t *)buf_ptr[0])[*rd_idx];
    *rd_idx = (*rd_idx + 1) & *mask;
    return val;
}

/* Ring buffer empty check — calls global ring_buf_empty which reads
 * DMA hardware to update wr_idx. */
extern uint32_t ring_buf_empty(uint8_t *ring);

uint32_t uart_packet_parse(uint8_t *s)
{
    uint8_t *ring_base = uart_state_arr;
    uint8_t *ring_sub = uart_state_arr + UART_RING_BUF;

    while (1) {
        /* Check if ring buffer has data */
        if (ring_buf_empty(ring_base)) {
            return 0;
        }

        /* Read next byte from ring sub-struct */
        uint8_t byte = ring_read(ring_sub);

        /* Shift byte history */
        s[UART_HISTORY_0] = s[UART_HISTORY_1];
        s[UART_HISTORY_1] = s[UART_HISTORY_2];
        s[UART_HISTORY_2] = byte;

        /* Check for 0xFF 0xFF sync */
        if (s[UART_HISTORY_0] == 0xFF && s[UART_HISTORY_1] == 0xFF) {
            s[UART_PARSE_STATE] = 1;
        }

        switch (s[UART_PARSE_STATE]) {
        case 1: /* Waiting for ID */
            s[UART_SERVO_ID] = servo_regs_arr[5];  /* our servo ID */
            if (s[UART_SERVO_ID] == byte || byte == 0xFE) {
                s[UART_PKT_ID] = byte;
                s[UART_PARSE_STATE] = 2;
            } else {
                s[UART_PARSE_STATE] = 0;
            }
            break;

        case 2: /* Waiting for length */
            if (byte < 2) {
                s[UART_PARSE_STATE] = 0;
            } else {
                s[UART_PARAMS_LEFT] = byte - 2;
                s[UART_PARSE_STATE] = 3;
            }
            break;

        case 3: /* Waiting for instruction */
            s[UART_PKT_INST] = byte;
            if (byte == INST_SYNC_WRITE || s[UART_PARAMS_LEFT] < 0x81) {
                if (s[UART_PARAMS_LEFT] == 0) {
                    s[UART_PARSE_STATE] = 5;
                } else {
                    s[UART_PARSE_STATE] = 4;
                }
                s[UART_PARAM_IDX] = 0;
                s[UART_TX_LEN] = 0;
                s[UART_PKT_CHECKSUM] = 0;
            } else {
                s[UART_PARSE_STATE] = 0;
            }
            break;

        case 4: /* Receiving params */
            if (s[UART_PKT_INST] == INST_SYNC_WRITE) {
                /* SYNC_WRITE accumulates data differently via accumulator */
                extern uint32_t sync_write_accumulate(uint8_t *s, uint32_t byte_val, int32_t state_idx);
                uint32_t result = sync_write_accumulate(s, byte, s[UART_PARAM_IDX]);
                if (result != 0) {
                    s[UART_PARSE_STATE] = 0;
                    break;
                }
            } else {
                s[UART_PACKET_BUF + s[UART_PARAM_IDX]] = byte;
            }
            s[UART_PKT_CHECKSUM] += byte;
            s[UART_PARAM_IDX]++;
            if (s[UART_PARAM_IDX] == s[UART_PARAMS_LEFT]) {
                s[UART_PARSE_STATE] = 5;
            }
            break;

        case 5: /* Checksum verify */
            {
                uint8_t expected = ~(s[UART_PKT_ID] + 2 +
                                     s[UART_PKT_INST] +
                                     s[UART_PARAMS_LEFT] +
                                     s[UART_PKT_CHECKSUM]);
                s[UART_PKT_CHECKSUM] = expected;
                s[UART_PARSE_STATE] = 0;
                if (expected == byte) {
                    return 1;  /* Valid packet ready */
                }
                s[UART_TX_LEN] = 0;
            }
            break;
        }
    }
}

/* Build status response packet: [0xFF][0xFF][ID][LEN=2][ERROR][CHECKSUM] */
void uart_send_response(uint8_t *state)
{
    volatile uint8_t * volatile us = (volatile uint8_t *)uart_state_arr;
    volatile uint8_t *sr = servo_regs;
    uint8_t bVar1 = us[UART_TX_LEN];
    uint32_t idx = (uint32_t)bVar1;

    if (idx < 0x80) {
        uint32_t next = (idx + 1) & 0xFF;
        us[UART_TX_LEN] = (uint8_t)(idx + 1);
        us[idx + UART_TX_BUF] = 0xFF;

        if (next > 0x7F) {
            /* Overflow path: original still reads servo_regs[0x41]
             * as dead side-effects (compiler didn't optimize them out) */
            (void)sr[SR_ERROR_FLAGS];
            (void)sr[SR_ERROR_FLAGS];
            goto done;
        }

        uint8_t bVar5 = bVar1 + 2;
        us[UART_TX_LEN] = bVar5;
        us[next + UART_TX_BUF] = 0xFF;

        if (bVar5 > 0x7F) {
            /* Overflow: dead reads from original */
            (void)sr[SR_ERROR_FLAGS];
            goto overflow_error;
        }

        uint8_t servo_id = state[UART_SERVO_ID];
        uint8_t bVar6 = bVar1 + 3;
        us[UART_TX_LEN] = bVar6;
        us[(uint32_t)bVar5 + UART_TX_BUF] = servo_id;

        if (bVar6 > 0x7F) goto overflow_error;

        us[UART_TX_LEN] = bVar1 + 4;
        us[(uint32_t)bVar6 + UART_TX_BUF] = 2;
    }

    /* Error byte section */
    {
        uint8_t error = sr[SR_ERROR_FLAGS];
        uint8_t idx2 = us[UART_TX_LEN];
        if (idx2 > 0x7F) {
overflow_error:
            /* Overflow: dead reads from original */
            (void)sr[SR_ERROR_FLAGS];
            goto done;
        }

        uint8_t next2 = idx2 + 1;
        us[UART_TX_LEN] = next2;
        us[(uint32_t)idx2 + UART_TX_BUF] = error;

        char cVar3 = (char)sr[SR_ERROR_FLAGS];
        uint8_t cVar4 = state[UART_SERVO_ID];
        char checksum = ~((char)cVar4 + cVar3 + 2);
        if (next2 < 0x80) {
            us[UART_TX_LEN] = idx2 + 2;
            us[(uint32_t)next2 + UART_TX_BUF] = (uint8_t)checksum;
        }
    }

done:
    state[UART_TX_STATE] = 6;
}

/* ================================================================
 * uart_dispatch — Instruction dispatcher
 *
 * Dispatches parsed packet based on instruction byte.
 * ================================================================ */
/* Register access (main.c) */
extern int reg_write(uint32_t addr, uint8_t *data, uint8_t len);
extern int reg_read(uint32_t addr, uint8_t *out, uint8_t len);

/* subsystems.c */
extern int32_t handle_write(uint32_t *param, uint8_t *params);
extern int32_t handle_sync_write(uint32_t *param, uint8_t *params);

/* WRITE instruction is handled by handle_write() in uart_subsys.c */

/* ================================================================
 * uart_handle_read — READ instruction handler
 *
 * Reads register data and builds response packet in TX buffer.
 * params[0] = start address, params[1] = length
 * ================================================================ */
static int __attribute__((noinline)) uart_handle_read(uint8_t *state)
{
    if (state[UART_PARAMS_LEFT] != 2) return -1;

    uint8_t addr = state[UART_PACKET_BUF];
    uint8_t len  = state[UART_PACKET_BUF + 1];

    /* Read registers into TX buffer in uart_state_arr (NOT motor_ctrl).
     * Original hardcodes uart_state_arr for the TX buffer at 0x2000018C. */
    uint8_t *tx = uart_state_arr + UART_TX_BUF;
    int count = reg_read(addr, tx + 5, len);
    if (count < 0) return -1;

    /* Build response header: FF FF ID LEN ERROR [DATA] CHECKSUM
     * TX buffer is in uart_state_arr, but TX_STATE counter is tracked
     * on state (motor_ctrl_arr) — original uses r4 for state tracking. */
    state[UART_ERROR] = 0;
    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = state[UART_SERVO_ID];
    tx[3] = (uint8_t)(count + 2);  /* LEN = data_count + error + checksum */
    tx[4] = servo_regs_arr[0x41];      /* error/status byte */

    /* Calculate checksum */
    uint8_t checksum = tx[2] + tx[3] + tx[4];
    state[UART_TX_STATE] = 5;
    for (int i = 0; i < count; i++) {
        checksum += tx[state[UART_TX_STATE]];
        state[UART_TX_STATE]++;
    }
    tx[state[UART_TX_STATE]] = ~checksum;
    state[UART_TX_STATE]++;
    /* TX_LEN goes to uart_state_arr, not motor_ctrl (original at 0x080023B0) */
    uart_state_arr[UART_TX_LEN] = state[UART_TX_STATE];

    return count;
}

/* ================================================================
 * uart_handle_sync_read — SYNC_READ handler
 *
 * Handles SYNC_READ instruction: finds our servo ID in the ID list,
 * reads requested registers, builds response packet.
 *
 * Packet format: [addr][len][id0][id1]...[idN]
 *   params[0x85] = start register address
 *   params[0x86] = data length
 *   params[0x87..] = list of servo IDs
 *   params[0x105] = total param count (including addr+len)
 *
 * Response: [FF][FF][ID][LEN][ERROR][DATA...][CHECKSUM]
 * ================================================================ */
static int __attribute__((noinline)) uart_handle_sync_read(uint8_t *state)
{
    uint8_t param_count = state[UART_PARAMS_LEFT];
    if (param_count < 3) return -1;

    int32_t id_count = (int32_t)(param_count - 2);
    if (id_count < 1) return -1;

    /* Find our servo ID in the ID list */
    uint32_t id_index;
    if (state[0x87] == state[UART_SERVO_ID]) {
        id_index = 0;
    } else {
        id_index = 0;
        do {
            id_index = (id_index + 1) & 0xFF;
            if (id_count <= (int32_t)id_index) {
                return -1;
            }
        } while (state[id_index + 0x87] != state[UART_SERVO_ID]);
    }

    /* Store index as response delay (to stagger servo responses) */
    state[UART_ERROR] = (uint8_t)id_index;

    /* Read registers into TX buffer in uart_state_arr (NOT motor_ctrl).
     * Original hardcodes uart_state_arr for the TX buffer. */
    uint8_t addr = state[UART_PACKET_BUF];      /* 0x85: start address */
    uint8_t len  = state[UART_PACKET_BUF + 1];  /* 0x86: data length */
    uint8_t *tx = uart_state_arr + UART_TX_BUF;
    int32_t count = reg_read(addr, tx + 5, len);

    if (count == -1) return -1;

    /* Build response header: FF FF ID LEN ERROR [DATA] CHECKSUM */
    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = state[UART_SERVO_ID];
    tx[3] = (uint8_t)(count + 2);           /* LEN = data + error + checksum */
    tx[4] = servo_regs_arr[0x41];               /* error/status byte */

    /* Compute checksum over header + data.
     * TX_STATE counter on state (motor_ctrl), TX buffer on uart_state. */
    uint8_t checksum = (uint8_t)count + state[UART_SERVO_ID]
                       + servo_regs_arr[SR_ERROR_FLAGS] + 2;
    state[UART_TX_STATE] = 5;

    if (count > 0) {
        uint32_t i = 0;
        do {
            checksum += tx[state[UART_TX_STATE]];
            state[UART_TX_STATE]++;
            i = (i + 1) & 0xFF;
        } while ((int32_t)i < count);
    }

    /* Write checksum and finalize.
     * TX_LEN goes to uart_state_arr (original hardcodes this). */
    uint8_t final_idx = state[UART_TX_STATE];
    tx[final_idx] = ~checksum;
    state[UART_TX_STATE] = final_idx + 1;
    uart_state_arr[UART_TX_LEN] = final_idx + 1;

    return count;
}

/* ================================================================
 * uart_dispatch — Instruction dispatcher
 * ================================================================ */
void uart_dispatch(uint8_t *state)
{
    uint8_t inst = state[UART_PKT_INST];

    /* Action instructions (execute then maybe respond) */
    switch (inst) {
    case INST_WRITE:        /* 0x03 */
        handle_write((uint32_t *)state, state + UART_PACKET_BUF);
        break;
    case INST_BOOTLOADER:   /* 0x08 */
        /* System reset to enter bootloader via vtable[2] */
        {
            typedef void (*boot_fn_t)(void);
            boot_fn_t fn = (boot_fn_t)(*(uint32_t *)(*(uint32_t *)state + 8));
            fn();
        }
        break;
    case INST_SYNC_WRITE:   /* 0x83 */
        handle_sync_write((uint32_t *)state, state + UART_PACKET_BUF);
        break;
    case INST_REG_WRITE:    /* 0x04 */
        /* Copy 0x81 bytes from packet buffer to pending write area (state+4) */
        {
            extern void subsys_memcpy(void *dst, const void *src, int count);
            subsys_memcpy((uint8_t *)state + 4, state + UART_PACKET_BUF, 0x81);
        }
        servo_regs_arr[0x40] = 1;  /* set action pending flag */
        break;
    case INST_ACTION:       /* 0x05 */
        if (servo_regs_arr[0x40] == 0) goto send_default;
        servo_regs_arr[0x40] = 0;
        handle_write((uint32_t *)state, (uint8_t *)(state + 4));
        break;
    case INST_FACTORY_RESET: /* 0x06 */
        /* vtable call: (**(code **)(*param_1 + 0xc))() */
        ((void (*)(void))(*(uint32_t *)(*(uint32_t *)state + 0x0C)))();
        break;
    case INST_SYNC_WRITE_CLK:  /* 0x09 */
        /* vtable call: (**(code **)(*param_1 + 0x10))() */
        ((void (*)(void))(*(uint32_t *)(*(uint32_t *)state + 0x10)))();
        break;
    case INST_SYNC_WRITE_CLK2: /* 0x0A */
        /* vtable call: (**(code **)(*param_1 + 0x14))() */
        ((void (*)(void))(*(uint32_t *)(*(uint32_t *)state + 0x14)))();
        break;
    case INST_REBOOT:       /* 0x0B */
        /* vtable call with position argument */
        {
            typedef void (*fn2_t)(uint8_t *, int32_t);
            fn2_t fn = (fn2_t)(*(uint32_t *)(*(uint32_t *)state + 0x18));
            if (state[UART_PARAMS_LEFT] == 0) {
                fn(state, 0x800);
            } else if (state[UART_PARAMS_LEFT] == 2) {
                fn(state, (int32_t)*(int16_t *)(state + UART_PACKET_BUF));
            }
        }
        break;
    default:
        break;
    }

    /* Response-generating instructions */
    if (inst == INST_PING) {  /* 0x01 */
        if (state[UART_PKT_ID] == 0xFE) {
            state[UART_ERROR] = state[UART_SERVO_ID];
        }
        uart_send_response(state);
        state[UART_PKT_READY] = 1;
        return;
    }

    if (inst == INST_READ) {  /* 0x02 */
        int r = uart_handle_read(state);
        if (r == -1) return;
        state[UART_PKT_READY] = 1;
        return;
    }

    if (inst == INST_SYNC_READ) {  /* 0x82 */
        int r = uart_handle_sync_read(state);
        if (r == -1) return;
        state[UART_PKT_READY] = 1;
        return;
    }

send_default:
    /* Send status response if not broadcast and response level allows */
    if (state[UART_PKT_ID] != 0xFE && servo_regs_arr[8] != 0) {
        state[UART_ERROR] = 0;
        uart_send_response(state);
        state[UART_PKT_READY] = 1;
    }
}

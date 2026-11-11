#!/usr/bin/env python3
"""
Turn-count tests — the revolution counter that position control keeps across the
encoder's wrap, what a homing-offset write does to it, and what clears it.

Background, measured on a factory STS3215 (firmware 3.10): a joint moved by hand through
the encoder's zero with the torque off keeps one turn in the revolution counter. Writing
the homing offset (registers 31-32) and the angle limits afterwards leaves the turn in
place: Present_Position (register 56) reads within one turn while the control position
is 4096 counts away, and the first goal drives the motor a whole turn. The RESET
instruction (0x0A, no parameter) zeroes the counter and keeps every register; so does a
power cycle, as the counter lives in RAM. These tests pin that behaviour here.

The raw angle is fed through a stub in the encoder vtable's read slot, as
golden_periph_test.py does for encoder_multiturn_track, so no I2C model is needed.

Usage:
    python3 test/turn_count_test.py
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from golden_periph_test import GoldenPeriphTests, FLASH_BASE, SRAM_BASE

# encoder_ctrl layout (servo_types.h)
ENC_ACCUM_POS = 4
ENC_REVOLUTIONS = 8
ENC_PREV_RAW = 16
ENC_POSITION_VAL = 0x1C

# pwm_ctrl layout
PWM_ENCODER_POS = 8
PWM_GOAL_POS = 0x0C

# servo registers
SR_ALARM_LED = 8          # response status level: non-zero answers a write
SR_CW_ANGLE_LO = 9
SR_CONFIG = 18            # bit 4: Present_Position shows the multi-turn value
SR_GOAL_OFFSET_LO = 31
SR_TORQUE_ENABLE = 40
SR_PRESENT_POS_LO = 56

# uart state inside motor_ctrl
UART_PARAMS_LEFT = 0x105
UART_PKT_ID = 0x108
UART_PKT_INST = 0x109
UART_ERROR = 0x112
UART_PKT_READY = 0x113

INST_RESET = 0x0A
FULL_REV = 4096

STUB_ADDR = 0x20001E00     # raw-angle stub: ldr r0, [pc]; bx lr; .word angle
STUB_VALUE = STUB_ADDR + 4  # the word the stub loads: rewritten between calls, never the code
VTABLE_ADDR = 0x20001D00   # encoder vtable with the stub in its read slot (+8)


class TurnCountTests(GoldenPeriphTests):
    # -- helpers ---------------------------------------------------------------

    def u32(self, addr):
        return struct.unpack('<I', bytes(self.uc.mem_read(addr, 4)))[0]

    def i32(self, addr):
        return struct.unpack('<i', bytes(self.uc.mem_read(addr, 4)))[0]

    def u16(self, addr):
        return struct.unpack('<H', bytes(self.uc.mem_read(addr, 2)))[0]

    def write_u32(self, addr, val):
        self.uc.mem_write(addr, struct.pack('<I', val & 0xFFFFFFFF))

    def write_u16(self, addr, val):
        self.uc.mem_write(addr, struct.pack('<H', val & 0xFFFF))

    def install_raw_angle_stub(self):
        """The encoder's read slot: ldr r0, [pc, #0]; bx lr; then the word it loads. The
        code is written once, as Unicorn keeps translated code; later changes go to the
        data word through set_raw_angle()."""
        self.uc.mem_write(STUB_ADDR, struct.pack('<HH', 0x4800, 0x4770))

    def set_raw_angle(self, angle):
        """Make the encoder's read slot answer `angle` on its next call."""
        self.write_u32(STUB_VALUE, angle)

    def flash_u32(self, addr):
        return struct.unpack_from('<I', self.fw_data, addr - FLASH_BASE)[0]

    def boot(self, raw_angle):
        """Fresh emulator with the objects as the firmware has them after boot: factory
        registers, the motor controller on its calibration vtable, and the encoder on a
        vtable whose read slot answers `raw_angle`."""
        self.setup()
        self.init_servo_regs()
        self.uc.mem_write(VTABLE_ADDR, b'\x00' * 16)
        self.write_u32(VTABLE_ADDR + 8, STUB_ADDR | 1)
        self.write_u32(self.arr['encoder_ctrl'], VTABLE_ADDR)
        self.write_u32(self.arr['motor_ctrl'], self.syms['motor_calibrate_vtable'])
        self.install_raw_angle_stub()
        self.set_raw_angle(raw_angle)

    def enc(self, off):
        return self.arr['encoder_ctrl'] + off

    def pwm(self, off):
        return self.arr['pwm_ctrl'] + off

    def reg(self, off):
        return self.arr['servo_regs'] + off

    def regs_snapshot(self, offsets):
        return {off: self.uc.mem_read(self.reg(off), 1)[0] for off in offsets}

    def count_a_turn(self):
        """A joint moved by hand through the encoder's zero: 4090 then 10 counts. Leaves
        the counter at one turn and the control position one turn above the angle."""
        set_initial = self.fn('encoder_set_initial')
        update = self.fn('encoder_update_position')
        if not set_initial or not update:
            return False
        self.call(set_initial, [self.arr['encoder_ctrl'], 0])
        self.check('turn: initial accumulated position', self.i32(self.enc(ENC_ACCUM_POS)) == 4090,
                   f'got {self.i32(self.enc(ENC_ACCUM_POS))}')
        self.set_raw_angle(10)
        self.call(update, [self.arr['encoder_ctrl']])
        self.check('turn: one revolution counted', self.i32(self.enc(ENC_REVOLUTIONS)) == 1,
                   f'got {self.i32(self.enc(ENC_REVOLUTIONS))}')
        self.check('turn: control position one turn up', self.i32(self.enc(ENC_POSITION_VAL)) == 10 + FULL_REV,
                   f'got {self.i32(self.enc(ENC_POSITION_VAL))}')
        return True

    def write_registers_and_notify(self, start, values):
        """Write registers as a WRITE instruction does, then run the register-change
        handler the calibration vtable installs (uart_parse_byte: r1 = start, r2 = count)."""
        notify = self.fn('uart_parse_byte')
        if not notify:
            return False
        self.uc.mem_write(self.reg(start), bytes(values))
        self.call(notify, [self.arr['motor_ctrl'], start, len(values)])
        return True

    # -- tests -----------------------------------------------------------------

    def test_reset_instruction_dispatches_to_encoder_home_reset(self):
        """0x0A reaches motor_calibrate_vtable[5], which must be encoder_home_reset."""
        vtable = self.syms.get('motor_calibrate_vtable')
        target = self.syms.get('encoder_home_reset')
        if not vtable or not target:
            self.skipped += 1
            return
        entry = self.flash_u32(vtable + 5 * 4) & ~1
        self.check('reset: vtable[5] = encoder_home_reset', entry == (target & ~1),
                   f'got {entry:#x}, expected {target & ~1:#x}')

    def test_turn_counted_through_the_wrap_survives_the_calibration_writes(self):
        """A homing offset and angle limits written after the crossing leave the turn in
        the counter and the control position, as on the factory firmware."""
        self.boot(4090)
        if not self.count_a_turn():
            return

        # The calibration writes the angle limits, then the homing offset (500 counts).
        # The offset moves the angle the encoder reports: 10 - 500 wraps to 3606.
        if not self.write_registers_and_notify(SR_CW_ANGLE_LO, struct.pack('<HH', 867, 3261)):
            return
        self.check('limits: revolution kept', self.i32(self.enc(ENC_REVOLUTIONS)) == 1,
                   f'got {self.i32(self.enc(ENC_REVOLUTIONS))}')
        self.set_raw_angle((10 - 500) % FULL_REV)
        if not self.write_registers_and_notify(SR_GOAL_OFFSET_LO, struct.pack('<H', 500)):
            return
        expected = (10 - 500) % FULL_REV + FULL_REV
        self.check('offset: revolution kept', self.i32(self.enc(ENC_REVOLUTIONS)) == 1,
                   f'got {self.i32(self.enc(ENC_REVOLUTIONS))}')
        self.check('offset: control position still one turn up',
                   self.i32(self.enc(ENC_POSITION_VAL)) == expected,
                   f'got {self.i32(self.enc(ENC_POSITION_VAL))}, expected {expected}')
        self.check('offset: PID reads the one-turn-up position',
                   self.i32(self.pwm(PWM_ENCODER_POS)) == expected,
                   f'got {self.i32(self.pwm(PWM_ENCODER_POS))}, expected {expected}')
        self.check('offset: goal snapped to it as well',
                   self.i32(self.pwm(PWM_GOAL_POS)) == expected,
                   f'got {self.i32(self.pwm(PWM_GOAL_POS))}, expected {expected}')

        # Present_Position hides the turn unless config bit 4 is set.
        apply = self.fn('pid_output_apply')
        if not apply:
            return
        self.call(apply, [self.arr['pwm_ctrl']])
        self.check('present position: within one turn with bit 4 clear',
                   self.u16(self.reg(SR_PRESENT_POS_LO)) == expected % FULL_REV,
                   f'got {self.u16(self.reg(SR_PRESENT_POS_LO))}, expected {expected % FULL_REV}')
        config = self.uc.mem_read(self.reg(SR_CONFIG), 1)[0]
        self.uc.mem_write(self.reg(SR_CONFIG), bytes([config | 0x10]))
        self.call(apply, [self.arr['pwm_ctrl']])
        self.check('present position: multi-turn value with bit 4 set',
                   self.u16(self.reg(SR_PRESENT_POS_LO)) == expected,
                   f'got {self.u16(self.reg(SR_PRESENT_POS_LO))}, expected {expected}')

    def test_encoder_home_reset_zeroes_the_turn_and_keeps_the_registers(self):
        """RESET's handler drops the revolution counter and moves the control position
        onto the angle; the limits, the offset and the config byte stay."""
        self.boot(4090)
        if not self.count_a_turn():
            return
        if not self.write_registers_and_notify(SR_CW_ANGLE_LO, struct.pack('<HH', 867, 3261)):
            return
        self.set_raw_angle((10 - 500) % FULL_REV)
        if not self.write_registers_and_notify(SR_GOAL_OFFSET_LO, struct.pack('<H', 500)):
            return
        kept = list(range(SR_CW_ANGLE_LO, SR_CW_ANGLE_LO + 4)) + [SR_CONFIG, SR_GOAL_OFFSET_LO, SR_GOAL_OFFSET_LO + 1]
        before = self.regs_snapshot(kept)
        self.uc.mem_write(self.reg(SR_TORQUE_ENABLE), b'\x01')

        reset = self.fn('encoder_home_reset')
        if not reset:
            return
        self.call(reset)

        angle = (10 - 500) % FULL_REV
        self.check('home reset: revolutions zero', self.i32(self.enc(ENC_REVOLUTIONS)) == 0,
                   f'got {self.i32(self.enc(ENC_REVOLUTIONS))}')
        self.check('home reset: accumulated position on the angle',
                   self.i32(self.enc(ENC_ACCUM_POS)) == angle,
                   f'got {self.i32(self.enc(ENC_ACCUM_POS))}, expected {angle}')
        self.check('home reset: PID position on the angle',
                   self.i32(self.pwm(PWM_ENCODER_POS)) == angle,
                   f'got {self.i32(self.pwm(PWM_ENCODER_POS))}, expected {angle}')
        self.check('home reset: torque off',
                   self.uc.mem_read(self.reg(SR_TORQUE_ENABLE), 1)[0] == 0)
        self.check('home reset: limits, offset and config kept',
                   self.regs_snapshot(kept) == before,
                   f'got {self.regs_snapshot(kept)}, expected {before}')

        apply = self.fn('pid_output_apply')
        if apply:
            self.call(apply, [self.arr['pwm_ctrl']])
            self.check('home reset: present position on the angle',
                       self.u16(self.reg(SR_PRESENT_POS_LO)) == angle,
                       f'got {self.u16(self.reg(SR_PRESENT_POS_LO))}, expected {angle}')

    def test_reset_instruction_over_the_bus_zeroes_the_turn(self):
        """A RESET packet (0x0A, no parameter) through uart_dispatch zeroes the counter and
        is answered with error 0, as the factory firmware answers FF FF 02 02 0A F1 with
        FF FF 02 02 00 FB."""
        self.boot(4090)
        if not self.count_a_turn():
            return
        dispatch = self.fn('uart_dispatch')
        if not dispatch:
            return
        motor = self.arr['motor_ctrl']
        self.uc.mem_write(motor + UART_PKT_ID, b'\x02')
        self.uc.mem_write(motor + UART_PKT_INST, bytes([INST_RESET]))
        self.uc.mem_write(motor + UART_PARAMS_LEFT, b'\x00')
        self.uc.mem_write(motor + UART_ERROR, b'\x55')
        self.uc.mem_write(self.reg(SR_ALARM_LED), b'\x01')

        self.call(dispatch, [motor])

        self.check('reset packet: revolutions zero', self.i32(self.enc(ENC_REVOLUTIONS)) == 0,
                   f'got {self.i32(self.enc(ENC_REVOLUTIONS))}')
        self.check('reset packet: control position on the angle',
                   self.i32(self.enc(ENC_POSITION_VAL)) == 10,
                   f'got {self.i32(self.enc(ENC_POSITION_VAL))}')
        self.check('reset packet: answered with error 0',
                   self.uc.mem_read(motor + UART_ERROR, 1)[0] == 0
                   and self.uc.mem_read(motor + UART_PKT_READY, 1)[0] == 1,
                   f'error {self.uc.mem_read(motor + UART_ERROR, 1)[0]}, '
                   f'ready {self.uc.mem_read(motor + UART_PKT_READY, 1)[0]}')

    def run_all(self):
        # Only this file's tests: the inherited harness also carries its own.
        methods = sorted(m for m in TurnCountTests.__dict__ if m.startswith('test_'))
        print(f"Running {len(methods)} turn-count tests...\n")
        for method_name in methods:
            try:
                getattr(self, method_name)()
            except Exception as e:
                self.failed += 1
                print(f"  ERROR {method_name}: {e}")
        print(f"\n{'=' * 60}")
        print(f"Results: {self.passed} passed, {self.failed} failed, {self.skipped} skipped")
        print(f"{'=' * 60}")
        return self.failed == 0


if __name__ == '__main__':
    sys.exit(0 if TurnCountTests().run_all() else 1)

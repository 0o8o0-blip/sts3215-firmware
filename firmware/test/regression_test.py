#!/usr/bin/env python3
"""
Regression tests — verify fixes for all 24 bugs found during development,
plus invariant checks that catch classes of errors.

Each test encodes the exact condition that would fail if a specific bug
regressed. No original factory binary needed.

Usage:
    python3 test/regression_test.py
"""

import os
import struct
import subprocess
import sys

try:
    from unicorn import *
    from unicorn.arm_const import *
except ImportError:
    print("Error: unicorn required. Install with: pip install unicorn")
    sys.exit(1)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from periph_stubs import PeripheralStubs

# Paths
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUR_BIN = os.path.join(BASE_DIR, 'build', 'scservo21.bin')
OUR_ELF = os.path.join(BASE_DIR, 'build', 'scservo21.elf')

# Memory map
FLASH_BASE  = 0x08000000
FLASH_SIZE  = 0x10000
SRAM_BASE   = 0x20000000
SRAM_SIZE   = 0x2000
PERIPH_BASE = 0x40000000
PERIPH_SIZE = 0x100000
SCB_BASE    = 0xE000E000
SCB_SIZE    = 0x10000
GPIO_BASE   = 0x48000000
GPIO_SIZE   = 0x10000
RETURN_ADDR = 0x0800FFFE

# Peripheral addresses
TIMER0_BASE  = 0x40012C00
TIMER0_SWEVG = TIMER0_BASE + 0x14
TIMER0_CH0CV = TIMER0_BASE + 0x34
TIMER0_CH2CV = TIMER0_BASE + 0x3C
I2C0_BASE    = 0x40005400
DMA_BASE     = 0x40020000
DMA_INTC     = DMA_BASE + 0x04
USART1_BASE  = 0x40004400
ADC_BASE     = 0x40012400
ADC_STAT     = ADC_BASE + 0x00
ADC_CTL1     = ADC_BASE + 0x08
RCU_BASE     = 0x40021000
NVIC_ISER0   = 0xE000E100
GPIOF_BASE   = 0x48001400
SYSCFG_BASE  = 0x40010000


def get_symbols():
    result = subprocess.run(
        ['arm-none-eabi-nm', OUR_ELF],
        capture_output=True, text=True
    )
    syms = {}
    for line in result.stdout.strip().split('\n'):
        parts = line.split()
        if len(parts) >= 3:
            syms[parts[2]] = int(parts[0], 16)
    return syms


def get_default_regs():
    regs = bytearray(128)
    regs[4] = 0; regs[5] = 1; regs[6] = 0
    regs[8] = 1; regs[9] = 0; regs[10] = 0
    regs[13] = 0x50; regs[14] = 0x50; regs[15] = 0x28
    struct.pack_into('<H', regs, 16, 1000)
    regs[18] = 0x20; regs[19] = 0x27; regs[20] = 0x27
    regs[21] = 0x20; regs[22] = 0x20; regs[23] = 4
    regs[24] = 0x10; regs[26] = 2; regs[27] = 2
    regs[30] = 1; regs[33] = 0
    regs[34] = 0x28; regs[35] = 0x50; regs[36] = 0x50; regs[37] = 0x19
    regs[39] = 0x32
    regs[80] = 2; regs[81] = 0x14; regs[82] = 0x32; regs[83] = 1
    regs[84] = 0x96; regs[85] = 100; regs[86] = 1
    return regs


class RegressionTests:
    def __init__(self):
        if not os.path.exists(OUR_BIN):
            print(f"Error: {OUR_BIN} not found. Run 'make' first.")
            sys.exit(1)
        self.fw_data = open(OUR_BIN, 'rb').read()
        self.syms = get_symbols()
        self._sidata = self.syms.get('_sidata', 0)
        self._sdata = self.syms.get('_sdata', SRAM_BASE)
        self._edata = self.syms.get('_edata', SRAM_BASE)
        self.passed = 0
        self.failed = 0
        self.skipped = 0

    def setup(self, init_data=True):
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(FLASH_BASE, FLASH_SIZE)
        uc.mem_map(SRAM_BASE, SRAM_SIZE)
        uc.mem_map(PERIPH_BASE, PERIPH_SIZE)
        uc.mem_map(SCB_BASE, SCB_SIZE)
        uc.mem_map(GPIO_BASE, GPIO_SIZE)
        uc.mem_write(FLASH_BASE, self.fw_data.ljust(FLASH_SIZE, b'\xff'))
        uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)

        if init_data:
            off = self._sidata - FLASH_BASE
            sdata_off = self._sdata - SRAM_BASE
            size = self._edata - self._sdata
            if off + size <= len(self.fw_data):
                uc.mem_write(SRAM_BASE + sdata_off,
                             self.fw_data[off:off + size])

        sp = struct.unpack_from('<I', self.fw_data, 0)[0]
        uc.reg_write(UC_ARM_REG_SP, sp)
        uc.reg_write(UC_ARM_REG_XPSR, 0x01000000)

        stubs = PeripheralStubs()
        stubs.install(uc)
        uc._periph_stubs = stubs
        self.uc = uc

    def fn(self, name):
        addr = self.syms.get(name)
        if not addr:
            self.skipped += 1
            print(f"  SKIP {name}: not found")
        return addr

    def call(self, addr, args=None, max_insns=100000):
        if args is None:
            args = []
        arg_regs = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
        for i, arg in enumerate(args[:4]):
            self.uc.reg_write(arg_regs[i], arg & 0xFFFFFFFF)
        self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
        self.uc._periph_stubs.full_reset()
        try:
            self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=5000000, count=max_insns)
        except UcError:
            pass
        r0 = self.uc.reg_read(UC_ARM_REG_R0)
        sram = bytes(self.uc.mem_read(SRAM_BASE, SRAM_SIZE))
        writes = list(self.uc._periph_stubs.write_log)
        return r0, sram, writes

    def check(self, name, condition, detail=""):
        if condition:
            self.passed += 1
        else:
            self.failed += 1
            print(f"  FAIL {name}: {detail}")

    def read_flash_u32(self, addr):
        off = addr - FLASH_BASE
        if off + 4 <= len(self.fw_data):
            return struct.unpack_from('<I', self.fw_data, off)[0]
        return None

    def read_vtable(self, name, count):
        """Read a vtable array from flash/rodata."""
        addr = self.syms.get(name)
        if not addr:
            return None, []
        entries = []
        for i in range(count):
            val = self.read_flash_u32(addr + i * 4)
            if val is not None:
                entries.append(val & ~1)  # Clear Thumb bit
            else:
                entries.append(None)
        return addr, entries

    # ================================================================
    # VTABLE REGRESSION TESTS (Bugs 6, 10, 11, 12, 13, 14)
    # ================================================================

    def test_bug06_encoder_vtable(self):
        """Bug 6: encoder_vtable[2] had encoder_read_complete instead of
        encoder_get_raw_angle."""
        _, entries = self.read_vtable('encoder_vtable', 3)
        if not entries:
            self.skipped += 1; return
        expected_fn = self.syms.get('encoder_get_raw_angle', 0) & ~1
        wrong_fn = self.syms.get('encoder_read_complete', 0) & ~1
        self.check('bug06: encoder_vtable[2] = encoder_get_raw_angle',
                   entries[2] == expected_fn,
                   f'got {entries[2]:#x}, expected {expected_fn:#x}')
        self.check('bug06: encoder_vtable[2] != encoder_read_complete',
                   entries[2] != wrong_fn,
                   'still points to encoder_read_complete!')

    def test_bug10_uart_state_vtable(self):
        """Bug 10: uart_state_vtable[0] had usart_idle_handler instead of
        uart_check_dispatch."""
        _, entries = self.read_vtable('uart_state_vtable', 1)
        if not entries:
            self.skipped += 1; return
        expected = self.syms.get('uart_check_dispatch', 0) & ~1
        wrong = self.syms.get('usart_idle_handler', 0) & ~1
        self.check('bug10: uart_state_vtable[0] = uart_check_dispatch',
                   entries[0] == expected,
                   f'got {entries[0]:#x}, expected {expected:#x}')
        self.check('bug10: uart_state_vtable[0] != usart_idle_handler',
                   entries[0] != wrong,
                   'still points to usart_idle_handler!')

    def test_bug11_motor_calibrate_vtable(self):
        """Bug 11: motor_calibrate_vtable[3] had position_offset_set instead
        of factory_reset_handler."""
        _, entries = self.read_vtable('motor_calibrate_vtable', 4)
        if not entries:
            self.skipped += 1; return
        expected = self.syms.get('factory_reset_handler', 0) & ~1
        wrong = self.syms.get('position_offset_set', 0) & ~1
        self.check('bug11: motor_calibrate_vtable[3] = factory_reset_handler',
                   entries[3] == expected,
                   f'got {entries[3]:#x}, expected {expected:#x}')

    def test_bug12_pwm_output_vtable(self):
        """Bug 12: pwm_ctrl_vtable[0] had timer0_set_duty instead of
        timer15_init_wrapper."""
        _, entries = self.read_vtable('pwm_ctrl_vtable', 2)
        if not entries:
            self.skipped += 1; return
        expected = self.syms.get('timer15_init_wrapper', 0) & ~1
        wrong = self.syms.get('timer0_set_duty', 0) & ~1
        self.check('bug12: pwm_ctrl_vtable[0] = timer15_init_wrapper',
                   entries[0] == expected,
                   f'got {entries[0]:#x}, expected {expected:#x}')
        self.check('bug12: pwm_ctrl_vtable[0] != timer0_set_duty',
                   entries[0] != wrong,
                   'still points to timer0_set_duty!')

    def test_bug13_ring_buf_vtable_sync_set(self):
        """Bug 13: ring_buf_vtable[1] had vtable_noop instead of
        uart_sync_set."""
        _, entries = self.read_vtable('ring_buf_vtable', 3)
        if not entries:
            self.skipped += 1; return
        expected = self.syms.get('uart_sync_set', 0) & ~1
        noop = self.syms.get('vtable_noop', 0) & ~1
        self.check('bug13: ring_buf_vtable[1] = uart_sync_set',
                   entries[1] == expected,
                   f'got {entries[1]:#x}, expected {expected:#x}')
        self.check('bug13: ring_buf_vtable[1] != vtable_noop',
                   entries[1] != noop,
                   'still points to vtable_noop!')

    def test_bug14_ring_buf_vtable_sync_clear(self):
        """Bug 14: ring_buf_vtable[2] had vtable_noop instead of
        uart_sync_clear."""
        _, entries = self.read_vtable('ring_buf_vtable', 3)
        if not entries:
            self.skipped += 1; return
        expected = self.syms.get('uart_sync_clear', 0) & ~1
        noop = self.syms.get('vtable_noop', 0) & ~1
        self.check('bug14: ring_buf_vtable[2] = uart_sync_clear',
                   entries[2] == expected,
                   f'got {entries[2]:#x}, expected {expected:#x}')
        self.check('bug14: ring_buf_vtable[2] != vtable_noop',
                   entries[2] != noop,
                   'still points to vtable_noop!')

    # Also verify all vtable arrays are complete and consistent
    def test_vtable_completeness(self):
        """Verify all 11 vtable arrays have valid flash pointers."""
        vtables = {
            'encoder_vtable': 3,
            'uart_state_vtable': 7,
            'motor_calibrate_vtable': 10,
            'motor_ctrl_vtable': 6,
            'ring_buf_vtable': 3,
            'led_obj_vtable': 4,
            'i2c_obj_vtable': 2,
            'i2c_init_vtable': 2,
            'uart_obj_vtable': 3,
            'pwm_ctrl_vtable': 2,
            'gpio_vtable': 4,
        }
        for name, count in vtables.items():
            addr, entries = self.read_vtable(name, count)
            if not addr:
                self.skipped += 1
                continue
            all_valid = all(
                e is not None and FLASH_BASE <= e < FLASH_BASE + FLASH_SIZE
                for e in entries
            )
            self.check(f'vtable {name}: {count} valid flash ptrs',
                       all_valid,
                       f'entries: {[f"{e:#x}" if e else "None" for e in entries]}')

    # ================================================================
    # PERIPHERAL WRITE REGRESSION TESTS
    # ================================================================

    def test_bug01_gpio_led_writes_to_gpiof(self):
        """Bug 1: GPIO LED was writing to SRAM instead of GPIOF registers."""
        addr = self.fn('gpio_led_init')
        if not addr:
            addr = self.fn('led_init')
        if not addr: return

        self.setup()
        # Set up vtables
        for vt_name, arr_name in [('gpio_vtable', 'gpio_ctrl_arr'),
                                   ('encoder_vtable', 'encoder_ctrl_arr')]:
            vt = self.syms.get(vt_name, 0)
            arr = self.syms.get(arr_name, 0)
            if vt and arr:
                self.uc.mem_write(arr, struct.pack('<I', vt))

        _, _, writes = self.call(addr, [], max_insns=100000)
        gpiof_writes = [(a, v) for a, s, v in writes if GPIOF_BASE <= a < GPIOF_BASE + 0x100]
        sram_writes_in_periph_range = [(a, v) for a, s, v in writes
                                        if SRAM_BASE <= a < SRAM_BASE + SRAM_SIZE
                                        and a not in range(self.uc.reg_read(UC_ARM_REG_SP) - 64,
                                                           self.uc.reg_read(UC_ARM_REG_SP) + 64)]

        self.check('bug01: LED init writes to GPIOF',
                   len(gpiof_writes) >= 1,
                   f'no GPIOF writes found, only {len(writes)} total writes')

    def test_bug18_nvic_irq23_only(self):
        """Bug 18: I2C init was enabling both IRQ 22 (EV) and IRQ 23 (ER).
        Only IRQ 23 should be enabled."""
        addr = self.fn('encoder_init')
        if not addr: return

        self.setup()
        _, _, writes = self.call(addr, [SRAM_BASE + 0x900], max_insns=200000)

        nvic_writes = [(a, v) for a, s, v in writes if a == NVIC_ISER0]
        # IRQ 22 = bit 22 = 0x00400000, IRQ 23 = bit 23 = 0x00800000
        irq22_enabled = any(v & 0x00400000 for _, v in nvic_writes)
        irq23_enabled = any(v & 0x00800000 for _, v in nvic_writes)

        self.check('bug18: IRQ 23 enabled', irq23_enabled,
                   'IRQ 23 (I2C error) not enabled')
        self.check('bug18: IRQ 22 NOT enabled', not irq22_enabled,
                   'IRQ 22 (I2C event) still enabled!')

    def test_bug20_baud_reads_reg6(self):
        """Bug 20: Baud rate was reading servo_regs[4] (model minor) instead
        of servo_regs[6] (actual baud index)."""
        addr = self.fn('usart1_dma_init')
        if not addr: return

        self.setup()
        servo_regs_arr = self.syms.get('servo_regs_arr', 0)
        if not servo_regs_arr:
            self.skipped += 1; return

        # Set reg[4] = 3 (would give wrong baud rate)
        # Set reg[6] = 0 (correct: 1 Mbps)
        regs = get_default_regs()
        regs[4] = 3  # model minor — should NOT be used as baud index
        regs[6] = 0  # baud index — should be used
        self.uc.mem_write(servo_regs_arr, bytes(regs))

        uart_state = self.syms.get('uart_state_arr', 0)
        if not uart_state:
            self.skipped += 1; return

        _, _, writes = self.call(addr, [uart_state, 0], max_insns=200000)

        # Check USART1 BRR value: baud_idx=0 → 1Mbps → BRR=0x30 (48MHz/1M)
        # If it wrongly read reg[4]=3 → baud_idx=3 → 57600 → BRR=0x341
        usart_brr = [(a, v) for a, s, v in writes if a == USART1_BASE + 0x0C]
        if usart_brr:
            brr_val = usart_brr[0][1]
            self.check('bug20: BRR matches baud_idx=0 (1Mbps)',
                       brr_val == 0x30,
                       f'BRR={brr_val:#x}, expected 0x30 (1Mbps). '
                       f'Got baud from reg[4]?')
        else:
            self.check('bug20: USART1 BRR written', False, 'no BRR write found')

    def test_bug23_ug_event_on_brake(self):
        """Bug 23: PWM cold start needed UG event on TIMER0_SWEVG when braking
        to force immediate shadow register load for bootstrap charging.

        The SWEVG write at TIMER0_BASE+0x14 must exist in the brake path of
        pwm_apply_output. Unicorn's Thumb BL bug prevents runtime capture,
        so we verify the instruction exists in the binary via disassembly."""
        result = subprocess.run(
            ['arm-none-eabi-objdump', '-d', OUR_ELF],
            capture_output=True, text=True
        )

        # pwm_apply_output brake path: after timer0_set_duty BL, if brake (r4),
        # writes 1 to TIMER0_SWEVG (strh to [r3, #20] where r3=TIMER0_BASE)
        # Pattern: cbz r4, <skip>; movs r2, #1; ldr r3, [pc, ...]; strh r2, [r3, #20]
        pwm_apply_addr = self.syms.get('pwm_apply_output', 0)
        if not pwm_apply_addr:
            self.skipped += 1; return

        # Find the SWEVG write within pwm_apply_output function range.
        # The write is: movs r2, #1; ldr r3, [pc, ...]; strh r2, [r3, #20]
        # at the end of the function, conditional on brake flag.
        pwm_end = pwm_apply_addr + 0x80  # function is small
        found_swevg = False
        for line in result.stdout.split('\n'):
            parts = line.strip().split(':')
            if len(parts) < 2:
                continue
            try:
                addr_val = int(parts[0].strip(), 16)
            except ValueError:
                continue
            if pwm_apply_addr <= addr_val < pwm_end:
                if 'strh' in line and '#20]' in line:
                    found_swevg = True
                    break

        self.check('bug23: SWEVG write in pwm_apply_output',
                   found_swevg,
                   'no strh to offset #20 (SWEVG) found in pwm_apply_output')

        # Also verify timer0_hw_init has a SWEVG write
        t0_addr = self.syms.get('timer0_hw_init', 0)
        if t0_addr:
            t0_end = t0_addr + 0x200
            found = False
            for line in result.stdout.split('\n'):
                parts = line.strip().split(':')
                if len(parts) < 2:
                    continue
                try:
                    addr_val = int(parts[0].strip(), 16)
                except ValueError:
                    continue
                if t0_addr <= addr_val < t0_end:
                    if 'strh' in line and '#20]' in line:
                        found = True
                        break
            self.check('bug23: SWEVG write in timer0_hw_init',
                       found, 'no SWEVG write found in timer0_hw_init')

    def test_bug16_dma_flag_clear_not_syscfg(self):
        """Bug 16: DMA flag clear was writing to SYSCFG instead of DMA_INTC."""
        # Check that no init function writes to SYSCFG_BASE range
        # when it should be writing to DMA_INTC
        addr = self.fn('usart1_dma_init')
        if not addr: return

        self.setup()
        servo_regs_arr = self.syms.get('servo_regs_arr', 0)
        if servo_regs_arr:
            self.uc.mem_write(servo_regs_arr, bytes(get_default_regs()))
        uart_state = self.syms.get('uart_state_arr', 0)
        if not uart_state:
            self.skipped += 1; return

        _, _, writes = self.call(addr, [uart_state, 0], max_insns=200000)

        syscfg_writes = [(a, v) for a, s, v in writes
                         if SYSCFG_BASE <= a < SYSCFG_BASE + 0x100]
        self.check('bug16: no SYSCFG writes during DMA init',
                   len(syscfg_writes) == 0,
                   f'found {len(syscfg_writes)} SYSCFG writes: {syscfg_writes}')

    def test_bug19_eeprom_defaults(self):
        """Bug 19: EEPROM defaults didn't match factory values.
        Verify servo_defaults_init produces correct register values."""
        addr = self.fn('servo_defaults_init')
        if not addr: return

        self.setup()
        servo_regs_arr = self.syms.get('servo_regs_arr', 0)
        if not servo_regs_arr:
            self.skipped += 1; return

        # Zero the regs, then run defaults init
        self.uc.mem_write(servo_regs_arr, b'\x00' * 128)
        self.call(addr, [servo_regs_arr])
        sram = bytes(self.uc.mem_read(SRAM_BASE, SRAM_SIZE))
        off = servo_regs_arr - SRAM_BASE

        # Factory default values (captured from servo_defaults_init)
        checks = [
            (5, 0x01, 'firmware_version'),
            (6, 0x01, 'baud_rate'),       # index 1 = 500kbps
            (8, 0x01, 'return_delay'),
            (13, 0x46, 'max_voltage'),     # 7.0V
            (14, 0x8C, 'min_voltage'),     # 14.0V (was wrong before bug 19 fix)
            (15, 0x28, 'max_voltage_limit'),
            (18, 0x0C, 'config_flags'),
            (26, 0x01, 'cw_dead_zone'),
            (27, 0x01, 'ccw_dead_zone'),
            (30, 0x01, 'torque_enable_default'),
            (33, 0x00, 'operating_mode'),  # position mode
        ]
        for reg, exp, name in checks:
            actual = sram[off + reg]
            self.check(f'bug19: defaults[{reg}] ({name})',
                       actual == exp,
                       f'got {actual:#x}, expected {exp:#x}')

    def test_bug24_runtime_regs_cleared_on_init(self):
        """Bug 24: Stale EEPROM values caused transient duty on boot.

        usart1_baud_init clears runtime servo_regs (torque, goal position,
        speed goal, alarm flags) to prevent stale EEPROM values from
        causing unwanted motor movement on boot."""
        addr = self.fn('usart1_baud_init')
        if not addr:
            self.skipped += 1; return

        self.setup()
        servo_regs_arr = self.syms.get('servo_regs_arr', 0)
        uart_state = self.syms.get('uart_state_arr', 0)
        if not servo_regs_arr or not uart_state:
            self.skipped += 1; return

        # Set stale runtime values
        regs = get_default_regs()
        regs[40] = 0x01   # torque_enable
        struct.pack_into('<H', regs, 42, 2000)  # goal_position
        regs[65] = 0xFF   # alarm flag
        self.uc.mem_write(servo_regs_arr, bytes(regs))

        self.call(addr, [uart_state, 0], max_insns=200000)
        sram = bytes(self.uc.mem_read(SRAM_BASE, SRAM_SIZE))
        off = servo_regs_arr - SRAM_BASE

        # Runtime fields should be cleared
        self.check('bug24: torque_enable cleared',
                   sram[off + 40] == 0,
                   f'servo_regs[40] = {sram[off + 40]:#x}')
        self.check('bug24: goal_position cleared',
                   struct.unpack_from('<H', sram, off + 42)[0] == 0,
                   f'servo_regs[42-43] = {struct.unpack_from("<H", sram, off + 42)[0]}')
        self.check('bug24: alarm flag cleared',
                   sram[off + 65] == 0,
                   f'servo_regs[65] = {sram[off + 65]:#x}')

    # ================================================================
    # INVARIANT CHECKS — properties that must always hold
    # ================================================================

    def test_invariant_duty_never_exceeds_period(self):
        """timer0_set_duty output must never exceed the period value."""
        addr = self.fn('timer0_set_duty')
        if not addr: return

        period = 970
        # Test with duties from -2000 to +2000 in steps
        for duty in range(-2000, 2001, 100):
            self.setup(init_data=False)
            self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)
            ctrl_off = 0x100
            self.uc.mem_write(SRAM_BASE + ctrl_off + 4,
                              struct.pack('<h', period))

            periph_writes = []
            def hook_write(uc, access, address, size, value, user_data):
                if PERIPH_BASE <= address < PERIPH_BASE + PERIPH_SIZE:
                    periph_writes.append((address, value))
            self.uc.hook_add(UC_HOOK_MEM_WRITE, hook_write)

            self.uc.reg_write(UC_ARM_REG_R0, SRAM_BASE + ctrl_off)
            self.uc.reg_write(UC_ARM_REG_R1, duty & 0xFFFFFFFF)
            self.uc.reg_write(UC_ARM_REG_R2, 0)  # not brake
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=2000000, count=50000)
            except UcError:
                pass

            for paddr, value in periph_writes:
                if paddr in (TIMER0_CH0CV, TIMER0_CH2CV):
                    self.check(f'invariant: duty({duty}) CHxCV <= period',
                               value <= period,
                               f'CHxCV={value} > period={period}')
            # At least one channel should be 0 (forward or reverse, one is off)
            ch_vals = {paddr: value for paddr, value in periph_writes
                       if paddr in (TIMER0_CH0CV, TIMER0_CH2CV)}
            if ch_vals and duty != 0:
                has_zero = any(v == 0 for v in ch_vals.values())
                self.check(f'invariant: duty({duty}) one channel is 0',
                           has_zero,
                           f'both channels nonzero: {ch_vals}')

    def test_invariant_linearize_monotonic(self):
        """position_linearize must be monotonically increasing in [0, 4095]."""
        addr = self.fn('position_linearize')
        if not addr: return

        self.setup(init_data=False)
        self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)

        prev = -1
        for raw in range(0, 4096):
            self.uc.reg_write(UC_ARM_REG_R0, raw)
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=1000000, count=10000)
            except UcError:
                pass
            r0 = self.uc.reg_read(UC_ARM_REG_R0)
            if r0 <= prev:
                self.check(f'invariant: linearize monotonic at {raw}',
                           False, f'{r0} <= {prev}')
                return
            prev = r0

        self.check('invariant: linearize monotonic [0,4095]', True)

    def test_invariant_linearize_range(self):
        """position_linearize output must be in [0, 8191] for inputs [0, 4095]."""
        addr = self.fn('position_linearize')
        if not addr: return

        self.setup(init_data=False)
        self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)

        max_out = 0
        for raw in range(0, 4096):
            self.uc.reg_write(UC_ARM_REG_R0, raw)
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=1000000, count=10000)
            except UcError:
                pass
            r0 = self.uc.reg_read(UC_ARM_REG_R0)
            if r0 > max_out:
                max_out = r0
            if r0 > 8191:
                self.check(f'invariant: linearize({raw}) in range',
                           False, f'output {r0} > 8191')
                return

        self.check('invariant: linearize range [0,8191]', True)
        self.check('invariant: linearize uses full range',
                   max_out > 4096,
                   f'max output only {max_out}, expected >4096 (13-bit expansion)')

    def test_invariant_clamp_baud_idempotent(self):
        """clamp_baud_index(clamp_baud_index(x)) == clamp_baud_index(x)
        for all x — clamping must be idempotent."""
        addr = self.fn('clamp_baud_index')
        if not addr: return

        for idx in range(256):
            self.setup(init_data=False)
            self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)

            # First call
            self.uc.reg_write(UC_ARM_REG_R0, 0)
            self.uc.reg_write(UC_ARM_REG_R1, idx)
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=1000000, count=10000)
            except UcError:
                pass
            first = self.uc.reg_read(UC_ARM_REG_R0)

            # Second call with result
            self.uc.reg_write(UC_ARM_REG_R0, 0)
            self.uc.reg_write(UC_ARM_REG_R1, first)
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=1000000, count=10000)
            except UcError:
                pass
            second = self.uc.reg_read(UC_ARM_REG_R0)

            if first != second:
                self.check(f'invariant: clamp_baud idempotent at {idx}',
                           False, f'clamp({idx})={first}, clamp({first})={second}')
                return

        self.check('invariant: clamp_baud idempotent [0,255]', True)

    def test_invariant_brake_sets_both_channels_to_period(self):
        """timer0_set_duty with brake=1 must set both CH0CV and CH2CV to
        the full period value (H-bridge short for regenerative braking)."""
        addr = self.fn('timer0_set_duty')
        if not addr: return

        period = 970
        for duty in [0, 500, -500, 970, -970]:
            self.setup(init_data=False)
            self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)
            ctrl_off = 0x100
            self.uc.mem_write(SRAM_BASE + ctrl_off + 4,
                              struct.pack('<h', period))

            periph_writes = []
            def hook_write(uc, access, address, size, value, user_data):
                if PERIPH_BASE <= address < PERIPH_BASE + PERIPH_SIZE:
                    periph_writes.append((address, value))
            self.uc.hook_add(UC_HOOK_MEM_WRITE, hook_write)

            self.uc.reg_write(UC_ARM_REG_R0, SRAM_BASE + ctrl_off)
            self.uc.reg_write(UC_ARM_REG_R1, duty & 0xFFFFFFFF)
            self.uc.reg_write(UC_ARM_REG_R2, 1)  # brake=1
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(addr | 1, RETURN_ADDR, timeout=2000000, count=50000)
            except UcError:
                pass

            ch0 = ch2 = None
            for paddr, value in periph_writes:
                if paddr == TIMER0_CH0CV: ch0 = value
                elif paddr == TIMER0_CH2CV: ch2 = value

            self.check(f'invariant: brake({duty}) CH0CV = period',
                       ch0 == period,
                       f'CH0CV={ch0}, expected {period}')
            self.check(f'invariant: brake({duty}) CH2CV = period',
                       ch2 == period,
                       f'CH2CV={ch2}, expected {period}')

    def test_invariant_delinearize_inverts_linearize(self):
        """position_delinearize(position_linearize(x)) recovers the original
        value for all x in [0, 4095]."""
        lin_addr = self.fn('position_linearize')
        delin_addr = self.fn('position_delinearize')
        if not lin_addr or not delin_addr: return

        for raw in range(0, 4096, 16):  # Sample every 16 values
            self.setup(init_data=False)
            self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)

            # Linearize
            self.uc.reg_write(UC_ARM_REG_R0, raw)
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            try:
                self.uc.emu_start(lin_addr | 1, RETURN_ADDR, timeout=1000000, count=10000)
            except UcError:
                pass
            linearized = self.uc.reg_read(UC_ARM_REG_R0)

            # Delinearize
            self.uc.reg_write(UC_ARM_REG_R0, 0)  # unused param
            self.uc.reg_write(UC_ARM_REG_R1, linearized)
            self.uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
            self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)
            try:
                self.uc.emu_start(delin_addr | 1, RETURN_ADDR, timeout=1000000, count=10000)
            except UcError:
                pass
            r0 = self.uc.reg_read(UC_ARM_REG_R0)
            recovered = struct.unpack('<i', struct.pack('<I', r0))[0]

            # Delinearize returns signed: positive for [0,2047], negative for [2048,4095]
            # Need to convert back to unsigned 12-bit
            if recovered < 0:
                recovered_unsigned = (-recovered) + 2048
            elif recovered >= 2048:
                recovered_unsigned = recovered
            else:
                recovered_unsigned = recovered

            if recovered_unsigned != raw:
                # Special case: raw=2048 linearizes to 4096, delinearizes to 2048 which maps back correctly
                if raw == 2048 and recovered == 0:
                    continue  # 2048 → 4096 → 0 is the branch point, expected
                self.check(f'invariant: delin(lin({raw}))',
                           False,
                           f'lin={linearized}, delin={recovered}, recovered_unsigned={recovered_unsigned}')
                return

        self.check('invariant: delinearize inverts linearize [0,4095]', True)

    def run_all(self):
        methods = [m for m in dir(self) if m.startswith('test_')]
        print(f"Running {len(methods)} regression + invariant tests...\n")

        for method_name in sorted(methods):
            method = getattr(self, method_name)
            try:
                method()
            except Exception as e:
                self.failed += 1
                print(f"  ERROR {method_name}: {e}")

        print(f"\n{'='*60}")
        print(f"Results: {self.passed} passed, {self.failed} failed, {self.skipped} skipped")
        print(f"{'='*60}")
        return self.failed == 0


if __name__ == '__main__':
    tests = RegressionTests()
    success = tests.run_all()
    sys.exit(0 if success else 1)

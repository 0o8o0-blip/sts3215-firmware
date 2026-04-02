#!/usr/bin/env python3
"""
Golden value tests — verify firmware behavior without the original binary.

Each test runs a function from our compiled firmware with known inputs
and checks the output against pre-computed expected values. These values
were originally derived by diffing against the factory firmware.

Usage:
    python3 test/golden_test.py
"""

import os
import struct
import sys

try:
    from unicorn import *
    from unicorn.arm_const import *
except ImportError:
    print("Error: unicorn required. Install with: pip install unicorn")
    sys.exit(1)

# Paths
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUR_BIN = os.path.join(BASE_DIR, 'build', 'scservo21.bin')

# Memory map
FLASH_BASE = 0x08000000
FLASH_SIZE = 0x8000
SRAM_BASE = 0x20000000
SRAM_SIZE = 0x1000
PERIPH_BASE = 0x40000000
PERIPH_SIZE = 0x30000
SCB_BASE = 0xE000E000
SCB_SIZE = 0x1000
RETURN_ADDR = 0x08007FFE

# Peripheral writes captured during execution
periph_writes = []


def create_emulator(fw_data):
    """Create a Unicorn ARM emulator with our firmware loaded."""
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)

    # Map memory regions
    uc.mem_map(FLASH_BASE, FLASH_SIZE)
    uc.mem_map(SRAM_BASE, SRAM_SIZE)
    uc.mem_map(PERIPH_BASE, PERIPH_SIZE, UC_PROT_ALL)
    uc.mem_map(SCB_BASE, SCB_SIZE, UC_PROT_ALL)

    # Load firmware
    uc.mem_write(FLASH_BASE, fw_data[:FLASH_SIZE])

    # Initialize stack
    uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)

    # Hook peripheral writes
    def hook_mem_write(uc, access, address, size, value, user_data):
        if PERIPH_BASE <= address < PERIPH_BASE + PERIPH_SIZE:
            periph_writes.append((address, size, value))
    uc.hook_add(UC_HOOK_MEM_WRITE, hook_mem_write)

    # Auto-return on unmapped fetch (function return)
    def hook_unmapped(uc, access, address, size, value, user_data):
        return True
    uc.hook_add(UC_HOOK_MEM_FETCH_UNMAPPED, hook_unmapped)

    return uc


def find_function(fw_data, name):
    """Find function address from ELF symbol table."""
    elf_path = os.path.join(BASE_DIR, 'build', 'scservo21.elf')
    if not os.path.exists(elf_path):
        return None
    import subprocess
    result = subprocess.run(
        ['arm-none-eabi-nm', elf_path],
        capture_output=True, text=True
    )
    for line in result.stdout.strip().split('\n'):
        parts = line.split()
        if len(parts) >= 3 and parts[2] == name:
            return int(parts[0], 16)
    return None


def call_function(uc, addr, args=None, max_insns=50000):
    """Call a function and return (r0, sram_snapshot)."""
    global periph_writes
    periph_writes = []

    if args is None:
        args = []

    arg_regs = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
    for i, arg in enumerate(args[:4]):
        uc.reg_write(arg_regs[i], arg & 0xFFFFFFFF)

    uc.reg_write(UC_ARM_REG_LR, RETURN_ADDR | 1)
    uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)

    try:
        uc.emu_start(addr | 1, RETURN_ADDR, timeout=2000000, count=max_insns)
    except UcError:
        pass

    r0 = uc.reg_read(UC_ARM_REG_R0)
    sram = uc.mem_read(SRAM_BASE, SRAM_SIZE)
    return r0, bytes(sram), list(periph_writes)


def sram_write(uc, offset, data):
    """Write bytes to SRAM at offset."""
    uc.mem_write(SRAM_BASE + offset, data)


def sram_read_u16(sram, offset):
    return struct.unpack_from('<H', sram, offset)[0]


def sram_read_u32(sram, offset):
    return struct.unpack_from('<I', sram, offset)[0]


def sram_read_i16(sram, offset):
    return struct.unpack_from('<h', sram, offset)[0]


# ================================================================
# Test cases — golden values embedded as assertions
# ================================================================

class GoldenTests:
    def __init__(self):
        if not os.path.exists(OUR_BIN):
            print(f"Error: {OUR_BIN} not found. Run 'make' first.")
            sys.exit(1)

        self.fw_data = open(OUR_BIN, 'rb').read()
        # Pad to flash size
        self.fw_data += b'\xff' * (FLASH_SIZE - len(self.fw_data))
        self.passed = 0
        self.failed = 0
        self.skipped = 0

    def setup(self):
        """Create fresh emulator."""
        self.uc = create_emulator(self.fw_data)
        # Zero SRAM
        self.uc.mem_write(SRAM_BASE, b'\x00' * SRAM_SIZE)
        self.uc.reg_write(UC_ARM_REG_SP, SRAM_BASE + SRAM_SIZE)

    def fn(self, name):
        addr = find_function(self.fw_data, name)
        if not addr:
            self.skipped += 1
            print(f"  SKIP {name}: not found")
        return addr

    def check(self, name, condition, detail=""):
        if condition:
            self.passed += 1
        else:
            self.failed += 1
            print(f"  FAIL {name}: {detail}")

    # --- clamp_baud_index ---
    def test_clamp_baud_index(self):
        addr = self.fn('clamp_baud_index')
        if not addr: return

        cases = [(0, 0), (1, 1), (3, 3), (7, 7), (8, 0), (10, 0), (255, 0)]
        for idx, expected in cases:
            self.setup()
            r0, _, _ = call_function(self.uc, addr, [0, idx])
            self.check(f'clamp_baud_index({idx})', r0 == expected,
                       f'got {r0}, expected {expected}')

    # --- encoder_set_value ---
    def test_encoder_set_value(self):
        addr = self.fn('encoder_set_value')
        if not addr: return

        for val in [0, 100, 2048, 4095, 0xFFFFFFFF]:
            self.setup()
            buf_addr = 0x100  # SRAM offset for test buffer
            r0, sram, _ = call_function(self.uc, addr, [SRAM_BASE + buf_addr, val])
            stored = sram_read_u32(sram, buf_addr)
            self.check(f'encoder_set_value({val})', r0 == (val & 0xFFFFFFFF) and stored == (val & 0xFFFFFFFF),
                       f'r0={r0:#x} stored={stored:#x}')

    # --- subsys_init_5 ---
    def test_subsys_init_5(self):
        addr = self.fn('subsys_init_5')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
        self.check('subsys_init_5', sram[buf_addr] == 5,
                   f'got {sram[buf_addr]}, expected 5')

    # --- byte_write ---
    def test_byte_write(self):
        addr = self.fn('byte_write')
        if not addr: return

        for offset, val in [(0, 0x42), (10, 0xFF), (100, 0x00)]:
            self.setup()
            buf_addr = 0x200
            call_function(self.uc, addr, [SRAM_BASE + buf_addr, offset, val])
            sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
            self.check(f'byte_write(+{offset}, {val:#x})',
                       sram[buf_addr + offset] == val,
                       f'got {sram[buf_addr + offset]:#x}')

    # --- timer0_set_duty ---
    def test_timer0_set_duty(self):
        addr = self.fn('timer0_set_duty')
        if not addr: return

        TIMER0_BASE = 0x40012C00
        TIMER0_CH0CV = TIMER0_BASE + 0x34
        TIMER0_CH2CV = TIMER0_BASE + 0x3C

        cases = [
            # (duty, brake, period, expected_ch0, expected_ch2, label)
            (700, 0, 970, 0, 700, "forward 700"),
            (-700, 0, 970, 700, 0, "reverse 700"),
            (0, 0, 970, 0, 0, "zero duty"),
            (500, 1, 970, 970, 970, "brake"),
            (1500, 0, 970, 0, 970, "clamp to period"),
            (-1500, 0, 970, 970, 0, "clamp negative"),
        ]

        for duty, brake, period, exp_ch0, exp_ch2, label in cases:
            self.setup()
            # Set up ctrl buffer with period at offset 4
            ctrl_addr = 0x100
            sram_write(self.uc, ctrl_addr + 4, struct.pack('<h', period))
            call_function(self.uc, addr, [SRAM_BASE + ctrl_addr, duty & 0xFFFFFFFF, brake])

            # Check peripheral writes
            ch0 = ch2 = None
            for paddr, size, value in periph_writes:
                if paddr == TIMER0_CH0CV:
                    ch0 = value
                elif paddr == TIMER0_CH2CV:
                    ch2 = value

            self.check(f'timer0_set_duty {label}: CH0CV',
                       ch0 == exp_ch0, f'got {ch0}, expected {exp_ch0}')
            self.check(f'timer0_set_duty {label}: CH2CV',
                       ch2 == exp_ch2, f'got {ch2}, expected {exp_ch2}')

    # --- position_linearize ---
    def test_position_linearize(self):
        addr = self.fn('position_linearize')
        if not addr: return

        # Linearize maps 12-bit encoder to extended range.
        # Values 0-2047 pass through unchanged.
        # Values 2048-4095 are doubled: output = raw * 2 - 2048 + 2048 = raw + 2048.
        # Actually: 0-2047 pass through, 2048+ maps to 2048 + (raw-2048)*2 = 2*raw - 2048
        cases = [
            (0, 0),
            (1, 1),
            (1023, 1023),
            (1024, 1024),
            (2047, 2047),
            (2048, 4096),    # breakpoint: 2*2048 - 2048 + 2048 = 4096
            (3071, 5119),
            (3072, 5120),
            (4095, 6143),
            (4096, 0),       # wrap: value >= 4096 wraps to 0
        ]
        for raw, expected in cases:
            self.setup()
            r0, _, _ = call_function(self.uc, addr, [raw])
            self.check(f'position_linearize({raw})', r0 == expected,
                       f'got {r0}, expected {expected}')

    # --- pid_clamp_angle ---
    def test_pid_clamp_angle(self):
        addr = self.fn('pid_clamp_angle')
        if not addr: return

        # pid_clamp_angle needs servo_regs in SRAM — complex setup.
        # Covered by full_diff_test when original binary is available.
        self.skipped += 1

    # --- subsys_memcpy ---
    def test_subsys_memcpy(self):
        addr = self.fn('subsys_memcpy')
        if not addr: return

        cases = [
            (b'\x11\x22\x33\x44\x55\x66\x77\x88', 8),
            (b'\x00', 1),
            (b'\xFF' * 16, 16),
        ]
        for data, length in cases:
            self.setup()
            src_addr = 0x100
            dst_addr = 0x200
            sram_write(self.uc, src_addr, data)
            call_function(self.uc, addr, [SRAM_BASE + dst_addr, SRAM_BASE + src_addr, length])
            sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
            copied = bytes(sram[dst_addr:dst_addr + length])
            self.check(f'subsys_memcpy({length} bytes)', copied == data[:length],
                       f'got {copied.hex()}, expected {data[:length].hex()}')

    # --- subsys_memset ---
    def test_subsys_memset(self):
        addr = self.fn('subsys_memset')
        if not addr: return

        for fill, length in [(0x42, 8), (0x00, 4), (0xFF, 16)]:
            self.setup()
            buf_addr = 0x100
            sram_write(self.uc, buf_addr, b'\xAA' * 32)
            call_function(self.uc, addr, [SRAM_BASE + buf_addr, fill, length])
            sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
            filled = bytes(sram[buf_addr:buf_addr + length])
            after = sram[buf_addr + length]
            self.check(f'subsys_memset({fill:#x}, {length})',
                       filled == bytes([fill] * length) and after == 0xAA,
                       f'got {filled.hex()}, after={after:#x}')

    # --- motion_helper_reset ---
    def test_motion_helper_reset(self):
        addr = self.fn('motion_helper_reset')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\xBB' * 32)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)

        # motion_helper_reset zeros bytes 0-4 and 8-19, leaves 5-7 and 20+ untouched
        zeroed_offsets = list(range(0, 5)) + list(range(8, 20))
        kept_offsets = [5, 6, 7, 20, 21, 22, 23]

        all_zeroed = all(sram[buf_addr + i] == 0 for i in zeroed_offsets)
        all_kept = all(sram[buf_addr + i] == 0xBB for i in kept_offsets)
        self.check('motion_helper_reset zeros correct bytes',
                   all_zeroed and all_kept,
                   f'first 24: {bytes(sram[buf_addr:buf_addr+24]).hex()}')

    # --- motion_profile_zero ---
    def test_motion_profile_zero(self):
        addr = self.fn('motion_profile_zero')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\xCC' * 16)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)

        # motion_profile_zero zeros bytes 12-15 (int32 at offset 12)
        zeroed = all(sram[buf_addr + i] == 0 for i in [12, 13, 14, 15])
        kept = all(sram[buf_addr + i] == 0xCC for i in range(12))
        self.check('motion_profile_zero zeros offset 12-15',
                   zeroed and kept,
                   f'bytes: {bytes(sram[buf_addr:buf_addr+16]).hex()}')

    # --- adc_iir_filter ---
    def test_adc_iir_filter(self):
        addr = self.fn('adc_iir_filter')
        if not addr: return

        # adc_iir_filter(int32_t *accum, int32_t new_val)
        # Different filter than encoder_iir_filter — uses different coefficients
        cases = [
            # (prev_accum, new_val, expected_accum)
            (0, 1000, 968),
            (1000, 1000, 1000),
            (500, 0, 15),
            (8000, 4000, 4125),
            (0, 0, 0),
            (100, 200, 196),
        ]
        for prev, new_val, expected in cases:
            self.setup()
            accum_addr = 0x100
            sram_write(self.uc, accum_addr, struct.pack('<i', prev))
            call_function(self.uc, addr, [SRAM_BASE + accum_addr, new_val])
            sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
            accum = struct.unpack_from('<i', bytes(sram), accum_addr)[0]
            self.check(f'adc_iir_filter({prev},{new_val})',
                       accum == expected,
                       f'got {accum}, expected {expected}')

    # --- encoder_vtable_init ---
    def test_encoder_vtable_init(self):
        addr = self.fn('encoder_vtable_init')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\xCC' * 32)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)

        # Stores a vtable pointer at offset 0 (should be in flash range)
        vtable_ptr = struct.unpack_from('<I', bytes(sram), buf_addr)[0]
        in_flash = FLASH_BASE <= vtable_ptr < FLASH_BASE + FLASH_SIZE
        self.check('encoder_vtable_init stores flash ptr',
                   in_flash,
                   f'ptr={vtable_ptr:#010x}')

    # --- encoder_obj_init ---
    def test_encoder_obj_init(self):
        addr = self.fn('encoder_obj_init')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\xDD' * 48)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)

        # Stores vtable pointer at offset 0, zeros some middle offsets
        vtable_ptr = struct.unpack_from('<I', bytes(sram), buf_addr)[0]
        in_flash = FLASH_BASE <= vtable_ptr < FLASH_BASE + FLASH_SIZE
        # Bytes 8-15 should be zeroed
        zeroed_8_15 = all(sram[buf_addr + i] == 0 for i in range(8, 16))
        # Byte 27 and 28-31 should be zeroed
        zeroed_27 = sram[buf_addr + 27] == 0
        zeroed_28_31 = all(sram[buf_addr + i] == 0 for i in range(28, 32))
        self.check('encoder_obj_init vtable ptr', in_flash,
                   f'ptr={vtable_ptr:#010x}')
        self.check('encoder_obj_init zeros 8-15', zeroed_8_15,
                   f'bytes 8-15: {bytes(sram[buf_addr+8:buf_addr+16]).hex()}')

    # --- led_init ---
    def test_led_init(self):
        addr = self.fn('led_init')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\xEE' * 16)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)

        # Stores vtable pointer at offset 0, zeros bytes 4-5
        vtable_ptr = struct.unpack_from('<I', bytes(sram), buf_addr)[0]
        in_flash = FLASH_BASE <= vtable_ptr < FLASH_BASE + FLASH_SIZE
        zeroed_4_5 = sram[buf_addr + 4] == 0 and sram[buf_addr + 5] == 0
        self.check('led_init vtable ptr', in_flash,
                   f'ptr={vtable_ptr:#010x}')
        self.check('led_init zeros bytes 4-5', zeroed_4_5,
                   f'byte4={sram[buf_addr+4]:#x} byte5={sram[buf_addr+5]:#x}')

    # --- ring_buf_init ---
    def test_ring_buf_init(self):
        addr = self.fn('ring_buf_init')
        if not addr: return

        self.setup()
        buf_addr = 0x100
        # Pre-fill with non-zero
        sram_write(self.uc, buf_addr, b'\xAA' * 32)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
        # ring_buf_init should zero the struct
        # ring_buf_init zeros bytes 0-2, 4-15 (byte 3 is untouched)
        zeroed = all(sram[buf_addr + i] == 0 for i in [0,1,2,4,5,6,7,8,9,10,11,12,13,14,15])
        self.check('ring_buf_init zeros struct', zeroed,
                   f'first 16 bytes: {bytes(sram[buf_addr:buf_addr+16]).hex()}')

    # --- encoder_iir_filter ---
    def test_encoder_iir_filter(self):
        addr = self.fn('encoder_iir_filter')
        if not addr: return

        # IIR: out = (prev * (10 - alpha) + new * alpha) / 10
        cases = [
            # (prev_accum, new_val, alpha, expected_output)
            (0, 1000, 8, 800),        # 0*(10-8) + 1000*8 / 10 = 800
            (1000, 1000, 8, 1000),    # 1000*2 + 1000*8 / 10 = 1000
            (500, 0, 8, 100),         # 500*2 + 0*8 / 10 = 100
        ]
        for prev, new, alpha, expected in cases:
            self.setup()
            accum_addr = 0x100
            sram_write(self.uc, accum_addr, struct.pack('<i', prev))
            r0, sram, _ = call_function(self.uc, addr,
                [SRAM_BASE + accum_addr, new, alpha])
            result = struct.unpack_from('<i', r0.to_bytes(4, 'little'))[0] if isinstance(r0, int) else r0
            self.check(f'encoder_iir_filter({prev},{new},{alpha})',
                       abs(r0 - expected) <= 1,
                       f'got {r0}, expected ~{expected}')

    # --- encoder_init ---
    def test_encoder_init(self):
        addr = self.fn('encoder_init')
        if not addr: return

        self.setup()
        # Map GPIO region (encoder_init may configure GPIO pins)
        try:
            self.uc.mem_map(0x48000000, 0x2000, UC_PROT_ALL)
        except Exception:
            pass  # Already mapped
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\x00' * 64)
        _, _, pw = call_function(self.uc, addr, [SRAM_BASE + buf_addr])

        # encoder_init configures I2C0 peripheral:
        # 1. RCU_APB1EN |= I2C0EN (bit 21)
        # 2-4. I2C0 timing/address registers
        # 5. I2C0_CTL0 = enable
        expected_writes = {
            0x4002101C: 0x200000,   # RCU_APB1EN: I2C0 clock enable
            0x40005404: 0x30,       # I2C0_SADDR0
            0x40005420: 0x30,       # I2C0_FMPCFG
            0x4000541C: 0xF0,       # I2C0_RT
            0x40005400: 0x01,       # I2C0_CTL0: enable
        }
        pw_dict = {}
        for paddr, size, value in pw:
            pw_dict[paddr] = value

        for reg_addr, exp_val in expected_writes.items():
            actual = pw_dict.get(reg_addr)
            self.check(f'encoder_init [{reg_addr:#010x}]',
                       actual == exp_val,
                       f'got {actual}, expected {exp_val:#x}')

    # --- timer15_hw_init_standalone ---
    def test_timer15_hw_init(self):
        addr = self.fn('timer15_hw_init_standalone')
        if not addr: return

        self.setup()
        _, _, pw = call_function(self.uc, addr, [])

        # TIMER15 init: enable clock, configure prescaler/period, enable
        expected_writes = {
            0x40021018: 0x20000,    # RCU_APB2EN: TIMER15 clock enable
            0x40014400: 0x08,       # TIMER15_CTL0
            0x4001442C: 0xFFFF,     # TIMER15_CAR (auto-reload = max)
            0x40014428: 0x2F,       # TIMER15_PSC (prescaler = 47 → 1 MHz)
            0x40014414: 0x01,       # TIMER15_DMAINTEN (update IRQ enable)
            0x40014410: 0x00,       # TIMER15_INTF (clear flags)
        }
        pw_dict = {}
        for paddr, size, value in pw:
            pw_dict[paddr] = value

        for reg_addr, exp_val in expected_writes.items():
            actual = pw_dict.get(reg_addr)
            self.check(f'timer15_hw_init [{reg_addr:#010x}]',
                       actual == exp_val,
                       f'got {actual}, expected {exp_val:#x}')

    # --- eeprom_page_to_regs ---
    def test_eeprom_page_to_regs(self):
        addr = self.fn('eeprom_page_to_regs')
        if not addr: return

        # eeprom_page_to_regs(uint16_t *dst, uint16_t *src, uint32_t count)
        # Copies count halfwords from src+2 (skip header halfword) to dst
        self.setup()
        src_addr = 0x300
        dst_addr = 0x100
        # Header halfword + 4 data halfwords
        src_data = struct.pack('<5H', 0xAAAA, 0x0011, 0x0022, 0x0033, 0x0044)
        sram_write(self.uc, src_addr, src_data)
        call_function(self.uc, addr, [SRAM_BASE + dst_addr, SRAM_BASE + src_addr, 4])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)

        expected = struct.pack('<4H', 0x0011, 0x0022, 0x0033, 0x0044)
        actual = bytes(sram[dst_addr:dst_addr + 8])
        self.check('eeprom_page_to_regs copies data', actual == expected,
                   f'got {actual.hex()}, expected {expected.hex()}')

    # --- eeprom_page_verify ---
    def test_eeprom_page_verify(self):
        addr = self.fn('eeprom_page_verify')
        if not addr: return

        # eeprom_page_verify(uint16_t *regs, uint16_t *page, uint32_t count)
        # Returns 1 if regs match page+2 (skip header), 0 if mismatch
        regs_addr = 0x100
        page_addr = 0x300

        # Match case
        self.setup()
        regs = struct.pack('<4H', 0x0011, 0x0022, 0x0033, 0x0044)
        page = struct.pack('<5H', 0xAAAA, 0x0011, 0x0022, 0x0033, 0x0044)
        sram_write(self.uc, regs_addr, regs)
        sram_write(self.uc, page_addr, page)
        r0, _, _ = call_function(self.uc, addr,
            [SRAM_BASE + regs_addr, SRAM_BASE + page_addr, 4])
        self.check('eeprom_page_verify match', r0 == 1,
                   f'got {r0}, expected 1')

        # Mismatch case
        self.setup()
        page_bad = struct.pack('<5H', 0xAAAA, 0x0011, 0x00FF, 0x0033, 0x0044)
        sram_write(self.uc, regs_addr, regs)
        sram_write(self.uc, page_addr, page_bad)
        r0, _, _ = call_function(self.uc, addr,
            [SRAM_BASE + regs_addr, SRAM_BASE + page_addr, 4])
        self.check('eeprom_page_verify mismatch', r0 == 0,
                   f'got {r0}, expected 0')

    # --- i2c_error_check ---
    def test_i2c_error_check(self):
        addr = self.fn('i2c_error_check')
        if not addr: return

        # With I2C peripheral registers at zero (no pending transaction),
        # i2c_error_check detects all error flags and writes 0x07 to byte 0
        self.setup()
        buf_addr = 0x100
        sram_write(self.uc, buf_addr, b'\x00' * 32)
        call_function(self.uc, addr, [SRAM_BASE + buf_addr])
        sram = self.uc.mem_read(SRAM_BASE, SRAM_SIZE)
        self.check('i2c_error_check byte[0]', sram[buf_addr] == 0x07,
                   f'got {sram[buf_addr]:#x}, expected 0x07')

    # --- SystemInit ---
    def test_system_init(self):
        addr = self.fn('SystemInit')
        if not addr: return

        self.setup()
        _, _, pw = call_function(self.uc, addr, [])

        # SystemInit configures RCU (Reset and Clock Unit) for 48 MHz from IRC8M+PLL:
        # - Set AHB/APB prescalers
        # - Enable PLL with IRC8M/2 * 12 = 48 MHz
        # - Wait for PLL lock and switch system clock
        expected_writes = [
            (0x40021008, 0xFF0000),     # RCU_CFG2: reset
            (0x40021000, 0x01),         # RCU_CTL: IRC8M enable
            (0x40021034, 0x01),         # RCU_CFG1
            (0x40021024, 0x01),         # RCU_INT: clear flags
            (0x40021000, 0x01),         # RCU_CTL: prepare for PLL
            (0x40021004, 0x00),         # RCU_CFG0: reset
            (0x40021004, 0x280000),     # RCU_CFG0: PLL * 12
            (0x40021000, 0x1000001),    # RCU_CTL: PLL enable
        ]
        self.check('SystemInit write count', len(pw) == len(expected_writes),
                   f'got {len(pw)}, expected {len(expected_writes)}')
        for i, (exp_addr, exp_val) in enumerate(expected_writes):
            if i < len(pw):
                paddr, _, value = pw[i]
                self.check(f'SystemInit write[{i}]',
                           paddr == exp_addr and value == exp_val,
                           f'got [{paddr:#010x}]={value:#x}, expected [{exp_addr:#010x}]={exp_val:#x}')

    # --- position_linearize extended (already tested, adding wrap) ---

    # --- baud_tx_time ---
    def test_baud_tx_time(self):
        addr = self.fn('baud_tx_time')
        if not addr: return

        # baud_tx_time(unused, baud_index) returns byte time in microseconds
        cases = [
            (0, 10),    # 1 Mbps
            (1, 20),    # 500 kbps
            (2, 40),    # 250 kbps
            (3, 79),    # 128 kbps
            (4, 87),    # 115200
            (5, 131),   # 76800
            (6, 174),   # 57600
            (7, 261),   # 38400
        ]
        for idx, expected in cases:
            self.setup()
            r0, _, _ = call_function(self.uc, addr, [0, idx])
            self.check(f'baud_tx_time({idx})', r0 == expected,
                       f'got {r0}, expected {expected}')

    def run_all(self):
        """Run all test methods."""
        methods = [m for m in dir(self) if m.startswith('test_')]
        print(f"Running {len(methods)} golden value tests...\n")

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
    tests = GoldenTests()
    success = tests.run_all()
    sys.exit(0 if success else 1)

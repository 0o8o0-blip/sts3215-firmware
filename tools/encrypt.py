#!/usr/bin/env python3
"""
Feetech Servo Firmware Encryption Tool

Encrypts a raw firmware binary with AES-256-ECB so the servo's
bootloader can decrypt and flash it. Same key used for all Feetech
GD32/STM32 servo products.

Usage:
    python3 encrypt.py firmware/build/scservo21.bin -o encrypted_output.bin
    python3 encrypt.py firmware/build/scservo21.bin  # writes to stdout

The bootloader expects the encrypted firmware in 64-byte blocks
transferred via XMODEM-CRC protocol. This tool just does the
AES-256-ECB encryption — the flash_tool.py handles the transport.
"""
import sys
import struct
import argparse
from pathlib import Path

try:
    from Crypto.Cipher import AES
except ImportError:
    try:
        from Cryptodome.Cipher import AES
    except ImportError:
        print("Error: pycryptodome required. Install with: pip install pycryptodome")
        sys.exit(1)

# AES-256-ECB key — shared across all Feetech servo products
# Found at bootloader flash address 0x08007F81
KEY = bytes.fromhex(
    "d1841f7c203625582170f38735a876ed"
    "eeba3a7426e9a02956a248371ac0382b"
)


def encrypt_firmware(data: bytes) -> bytes:
    """Encrypt firmware binary with AES-256-ECB.

    Pads to 16-byte boundary with 0xFF (erased flash value).
    """
    # Pad to AES block size (16 bytes) with 0xFF
    padded_len = (len(data) + 15) & ~15
    padded = data + b'\xFF' * (padded_len - len(data))

    cipher = AES.new(KEY, AES.MODE_ECB)
    encrypted = b""
    for i in range(0, len(padded), 16):
        encrypted += cipher.encrypt(padded[i:i+16])

    return encrypted


def validate_firmware(data: bytes) -> bool:
    """Basic validation that this looks like ARM Cortex-M firmware."""
    if len(data) < 256:
        print(f"Warning: firmware is only {len(data)} bytes", file=sys.stderr)
        return False

    # Check vector table
    sp = struct.unpack_from("<I", data, 0)[0]
    reset = struct.unpack_from("<I", data, 4)[0]

    valid_sp = 0x20000000 <= sp <= 0x20010000
    valid_reset = 0x08000000 <= reset <= 0x08080000 and (reset & 1)

    if not valid_sp:
        print(f"Warning: SP=0x{sp:08X} doesn't look like SRAM", file=sys.stderr)
    if not valid_reset:
        print(f"Warning: Reset=0x{reset:08X} doesn't look like flash+Thumb", file=sys.stderr)

    return valid_sp and valid_reset


def main():
    parser = argparse.ArgumentParser(
        description="Encrypt firmware for Feetech servo bootloader")
    parser.add_argument("input", help="Raw firmware binary (.bin)")
    parser.add_argument("-o", "--output", help="Output encrypted file (default: stdout)")
    parser.add_argument("--no-validate", action="store_true",
                       help="Skip firmware validation")
    args = parser.parse_args()

    # Read input
    data = Path(args.input).read_bytes()
    print(f"Input: {args.input} ({len(data)} bytes)", file=sys.stderr)

    # Validate
    if not args.no_validate:
        if not validate_firmware(data):
            print("Firmware validation failed. Use --no-validate to skip.",
                  file=sys.stderr)
            sys.exit(1)
        print(f"Validation: OK (SP=0x{struct.unpack_from('<I', data, 0)[0]:08X}, "
              f"Reset=0x{struct.unpack_from('<I', data, 4)[0]:08X})", file=sys.stderr)

    # Encrypt
    encrypted = encrypt_firmware(data)
    print(f"Encrypted: {len(encrypted)} bytes "
          f"({len(encrypted) // 64} blocks of 64 bytes)", file=sys.stderr)

    # Verify round-trip
    cipher = AES.new(KEY, AES.MODE_ECB)
    decrypted = b""
    for i in range(0, len(encrypted), 16):
        decrypted += cipher.decrypt(encrypted[i:i+16])
    if decrypted[:len(data)] != data:
        print("ERROR: Round-trip verification failed!", file=sys.stderr)
        sys.exit(1)
    print("Verify: round-trip OK", file=sys.stderr)

    # Output
    if args.output:
        Path(args.output).write_bytes(encrypted)
        print(f"Output: {args.output}", file=sys.stderr)
    else:
        sys.stdout.buffer.write(encrypted)


if __name__ == "__main__":
    main()

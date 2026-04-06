#!/usr/bin/env python3
"""
Rescue a bricked STS3215 servo by catching the bootloader during power-up
and flashing factory firmware.

Usage:
    1. Connect ONLY the bricked servo to the bus (disconnect all others)
    2. Run this script
    3. Power cycle the servo (unplug and replug power) when prompted
    4. The script catches the bootloader and flashes factory firmware

    python3 tools/rescue.py --port /dev/ttyACM0

Why servos get bricked:
    - Flashing with multiple servos on the bus (bootloader has no ID)
    - Power loss during firmware transfer
    - Flashing corrupted or wrong firmware file

The bootloader is never overwritten by UART flashing — it lives at
0x08007800-0x08007FFF and is always available for recovery.
"""

import argparse
import os
import serial
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from flash_tool import (send_firmware_block, read_block_response,
                        BLOCK_ACK, BLOCK_SIZE)

CACHE_DIR = os.path.join(SCRIPT_DIR, '..', 'firmware', 'cache')


def find_factory_firmware():
    """Try to locate or download the factory firmware.

    Checks for a cached encrypted copy first, then tries to download
    from the Feetech update server. Downloaded firmware is cached for
    future use.
    """
    # Check for cached encrypted firmware
    os.makedirs(CACHE_DIR, exist_ok=True)
    cache_path = os.path.join(CACHE_DIR, 'factory_SCServo21-GD32-TTL.enc')
    if os.path.exists(cache_path):
        print(f"  Using cached firmware: {cache_path}")
        return open(cache_path, 'rb').read()

    # Try downloading from Feetech server
    try:
        from flash_tool import download_firmware
        print("  Downloading from Feetech server...")
        result = download_firmware('SCServo21-GD32-TTL')
        if result and result.get('data'):
            fw_enc = result['data']  # Already encrypted from server
            # Cache for next time
            open(cache_path, 'wb').write(fw_enc)
            print(f"  Cached to {cache_path}")
            return fw_enc
    except Exception as e:
        print(f"  Download failed: {e}")

    return None


def catch_bootloader(ser, timeout=30):
    """Send bootloader magic repeatedly until ACK received."""
    print("Waiting for bootloader... POWER CYCLE THE SERVO NOW!")
    print("(unplug power, then plug it back in)")
    print()

    ser.reset_input_buffer()
    start = time.time()
    while time.time() - start < timeout:
        ser.write(b'1fBVA')
        time.sleep(0.02)
        data = ser.read(100)
        if data and 0x06 in data:
            elapsed = time.time() - start
            print(f"  Bootloader ACK after {elapsed:.1f}s")

            # Handshake
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(bytes([0x01]))
            time.sleep(0.2)
            resp = ser.read(10)
            if resp and 0x06 in resp:
                print("  Handshake OK")
                return True
            else:
                print(f"  Handshake failed: {resp.hex() if resp else 'no response'}")
                return False

    print("  No bootloader response (timeout).")
    print("  Make sure the servo has power and the data cable is connected.")
    return False


def flash_firmware(ser, fw_enc):
    """Send firmware blocks to bootloader."""
    if len(fw_enc) % BLOCK_SIZE:
        fw_enc += b'\xff' * (BLOCK_SIZE - len(fw_enc) % BLOCK_SIZE)
    num_blocks = len(fw_enc) // BLOCK_SIZE
    print(f"  Flashing {num_blocks} blocks...")

    block_num = 1
    retries = 0
    while block_num <= num_blocks:
        offset = (block_num - 1) * BLOCK_SIZE
        block_data = fw_enc[offset:offset + BLOCK_SIZE]
        is_last = (block_num == num_blocks)

        send_firmware_block(ser, block_num, block_data, is_last, 'bin')
        status, _ = read_block_response(ser, mode='software')

        if status == BLOCK_ACK:
            if block_num % 16 == 0:
                time.sleep(0.05)
            if block_num % 50 == 0 or is_last:
                pct = block_num * 100 // num_blocks
                print(f"  Block {block_num}/{num_blocks} ({pct}%)")
            block_num += 1
            retries = 0
        else:
            retries += 1
            if retries > 3:
                print(f"  Block {block_num} failed after 3 retries. Aborting.")
                return False
            time.sleep(0.1)

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Rescue a bricked STS3215 servo via bootloader")
    parser.add_argument('--port', required=True, help='Serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('--firmware', help='Encrypted firmware file (default: factory)')
    args = parser.parse_args()

    # Load firmware
    if args.firmware:
        print(f"Loading firmware from {args.firmware}...")
        fw_enc = open(args.firmware, 'rb').read()
        # Auto-encrypt raw .bin files (same fix as flash_tool.py)
        if args.firmware.endswith(".bin"):
            try:
                from encrypt import encrypt_firmware
                print("  Encrypting for bootloader...")
                fw_enc = encrypt_firmware(fw_enc)
                print(f"  Encrypted: {len(fw_enc)} bytes")
            except ImportError:
                print("Error: encrypt.py or pycryptodome not found.")
                print("Cannot flash unencrypted .bin — bootloader would decrypt to garbage.")
                print("Install: pip install pycryptodome")
                sys.exit(1)
    else:
        print("Loading factory firmware...")
        fw_enc = find_factory_firmware()
        if not fw_enc:
            print("Error: Could not find or download factory firmware.")
            print("Use --firmware to specify an encrypted firmware file,")
            print("or ensure internet access for Feetech server download.")
            print("Previously downloaded firmware is cached in firmware/cache/")
            sys.exit(1)

    print(f"  {len(fw_enc)} bytes ready")
    print()

    # Safety check
    print("=" * 60)
    print("IMPORTANT: Only ONE servo should be on the bus!")
    print("The bootloader has no servo ID — if multiple servos are")
    print("connected, they will ALL enter bootloader mode and the")
    print("flash will fail or brick additional servos.")
    print("=" * 60)
    print()

    input("Press Enter when ready (single servo connected, power off)...")
    print()

    # Catch bootloader
    ser = serial.Serial(args.port, 500000, timeout=0.01)
    if not catch_bootloader(ser):
        ser.close()
        sys.exit(1)

    # Flash
    if flash_firmware(ser, fw_enc):
        print()
        print("Rescue complete! Power cycle the servo to boot.")
        print("Then verify with: python3 tools/flash_tool.py --port {} --scan".format(
            args.port))
    else:
        print()
        print("Flash failed. Try again — the bootloader survives failed flashes.")

    ser.close()


if __name__ == '__main__':
    main()

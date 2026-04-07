#!/usr/bin/env python3
"""
Feetech STS/SCS Servo Firmware Flash Tool for Linux

Reverse-engineered from FD.exe v1.9.8.3 (Feetech Debug Software).
Flashes official encrypted firmware to Feetech bus servos (STS3251, STS3215, etc.)
over the standard half-duplex UART bus.

Usage:
    python3 flash_tool.py --port /dev/ttyACM1 --servo-id 1
    python3 flash_tool.py --port /dev/ttyACM1 --servo-id 1 --firmware local_file.bin
    python3 flash_tool.py --port /dev/ttyACM1 --list-firmware
    python3 flash_tool.py --port /dev/ttyACM1 --scan

Protocol summary (reverse-engineered from FD.exe 0x40FC80-0x4100B0):

    Path A (DTR mode - Feetech USB adapters):
        1. Send instruction 0x08 to servo
        2. Switch to 500k baud, toggle DTR
        3. Wait for 'C' (0x43) at baud 50
        4. Send magic "1fBVA", read 7-byte response
        5. Transfer blocks, read 7-byte responses (status at byte[1])

    Path B (Software mode - generic USB-serial like Waveshare):
        1. Send instruction 0x08 to servo
        2. Send servo_id byte raw at current baud
        3. Switch to 500k baud, sleep 100ms
        4. Send magic "1fBVA", read 1-byte response
        5. Transfer blocks, read 1-byte responses (byte IS the status)

    Block format: [block_num][~block_num][flag][64 data bytes][CRC16_hi][CRC16_lo][type]
    CRC scope for .bin:  bytes [0:64] of packet (header + first 61 data bytes)
    CRC scope for .xbin: bytes [3:67] of packet (all 64 data bytes)

Note: Firmware from Feetech's server is encrypted (ECB block cipher).
      Decryption happens on the motor's bootloader, not in this tool.
"""

import argparse
import base64
import json
import struct
import sys
import time
import urllib.request

import serial


# === Feetech Protocol Constants ===

FEETECH_HEADER = bytes([0xFF, 0xFF])
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_BOOTLOADER = 0x08  # Enter bootloader mode

# Feetech firmware download API (reverse-engineered from FD.exe v1.9.8.3)
#
# The server URL is parameterized by the servo's model version string,
# read from registers 3 (SERVO_MAJOR) and 4 (SERVO_MINOR) over UART.
# No hashing — the model string is appended directly to the URL path.
#
# Example:
#   Servo reports SERVO_MAJOR=9, SERVO_MINOR=2  →  version string "9.2"
#   Version check:  GET http://www.scservo.com:9048/ftgetzuixinbanben/9.2
#   Download:       GET http://www.scservo.com:9048/ftgetzuixinwenjian/9.2
#
# The server returns JSON with Chinese field names:
#   banben   = firmware version (e.g., "3.10")
#   xinghao  = model number (e.g., "9.2")
#   filename = firmware filename (e.g., "SCServo21-GD32-TTL-250306.bin")
#   wenjian  = firmware binary, base64-encoded (AES-256-ECB encrypted)
#
# Known model IDs and their firmware:
#   9.2   STS3215 (GD32, TTL)  →  SCServo21-GD32-TTL-250306.bin
#   9.3   STS3251 (GD32, TTL)  →  SCServo21-GD32-TTL-250306.bin (same firmware)
#   9.15  Older SCS/STS         →  SCServo2.20-GD32-TTL(200824).bin
#   10.3  HTS (GD32, TTL)      →  FT-HTS-GD32-TTL-240319.bin
#   10.8  HLS (GD32, TTL)      →  FT-HLS-GD32-TTL-250326.bin
#   10.6  ST (STM32, TTL)      →  STServo3.20-STM32-TTL(220714).bin
#   6.16  SMS (STM32, RS485)   →  SMServo1.0-STM32-485(200710).bin
#   8.0   SMS v2 (STM32, RS485)→  SMServo2.40-STM32-485(220714).bin
#   20.3  SMS MODBUS            →  SMServo3.40-STM32-485-MODBUS(220715).mbin
#
FIRMWARE_API_BASE = "http://www.scservo.com:9048"
FIRMWARE_API_VERSION = "/ftgetzuixinbanben/"    # "get latest version" (最新版本)
FIRMWARE_API_DOWNLOAD = "/ftgetzuixinwenjian/"  # "get latest file" (最新文件)

# Servo register addresses (STS/SMS/SCS memory map)
REG_FIRMWARE_MAJOR = 0
REG_FIRMWARE_MINOR = 1
REG_SERVO_MAJOR = 3
REG_SERVO_MINOR = 4
REG_ID = 5
REG_BAUD_RATE = 6

# Bootloader magic sequences
BOOTLOADER_MAGIC_BUS = b"1fBVA"   # For .bin firmware (bus servos)
BOOTLOADER_MAGIC_XBIN = b"ABV1f"  # For .xbin firmware (open servos)

BOOTLOADER_ACK = 0x43   # 'C' - bootloader ready (XMODEM-CRC style)
BLOCK_SIZE = 64          # Firmware data bytes per block
BLOCK_ACK = 0x06         # Block accepted
BLOCK_NAK = 0x15         # Block rejected, retry


# === CRC16-CCITT (XMODEM variant) ===

def crc16_ccitt(data, init=0x0000):
    """CRC16-CCITT as used by XMODEM-CRC protocol."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


# === Feetech Protocol Helpers ===

def feetech_checksum(servo_id, length, instruction, params=b""):
    """Calculate Feetech protocol checksum: ~(ID + Length + Instruction + Params)."""
    total = servo_id + length + instruction
    for b in params:
        total += b
    return (~total) & 0xFF


def feetech_send(ser, servo_id, instruction, params=b""):
    """Send a Feetech protocol packet."""
    length = len(params) + 2  # instruction + params + checksum
    checksum = feetech_checksum(servo_id, length, instruction, params)
    packet = FEETECH_HEADER + bytes([servo_id, length, instruction]) + params + bytes([checksum])
    ser.write(packet)
    ser.flush()
    return packet


def feetech_receive(ser, timeout=0.5):
    """Receive a Feetech protocol response packet."""
    ser.timeout = timeout
    # Read header
    header = ser.read(2)
    if header != FEETECH_HEADER:
        return None
    # Read ID + Length
    id_len = ser.read(2)
    if len(id_len) < 2:
        return None
    resp_id = id_len[0]
    resp_len = id_len[1]
    # Read payload (includes error byte + data + checksum)
    payload = ser.read(resp_len)
    if len(payload) < resp_len:
        return None
    error = payload[0]
    data = payload[1:-1]
    return {"id": resp_id, "error": error, "data": data}


def ping_servo(ser, servo_id):
    """Ping a servo and return model number, or None if no response."""
    ser.reset_input_buffer()
    feetech_send(ser, servo_id, INST_PING)
    resp = feetech_receive(ser, timeout=0.1)
    if resp is not None:
        return resp
    return None


def read_register(ser, servo_id, address, length=1):
    """Read bytes from a servo's register table."""
    ser.reset_input_buffer()
    feetech_send(ser, servo_id, INST_READ, bytes([address, length]))
    resp = feetech_receive(ser, timeout=0.1)
    if resp and resp["data"]:
        return resp["data"]
    return None


def read_servo_info(ser, servo_id):
    """Read firmware version and servo model version from a servo."""
    info = {}

    data = read_register(ser, servo_id, REG_FIRMWARE_MAJOR, 2)
    if data and len(data) >= 2:
        info["fw_major"] = data[0]
        info["fw_minor"] = data[1]

    data = read_register(ser, servo_id, REG_SERVO_MAJOR, 2)
    if data and len(data) >= 2:
        info["servo_major"] = data[0]
        info["servo_minor"] = data[1]

    return info


# === Firmware Download ===

def get_servo_version_string(ser, servo_id):
    """Get the servo version string used for firmware API queries (e.g., '9.3')."""
    info = read_servo_info(ser, servo_id)
    if "servo_major" in info and "servo_minor" in info:
        return f"{info['servo_major']}.{info['servo_minor']}"
    return None


def check_firmware_version(version_string):
    """Check the latest firmware version available for a given servo model."""
    url = f"{FIRMWARE_API_BASE}{FIRMWARE_API_VERSION}{version_string}"
    try:
        with urllib.request.urlopen(url, timeout=10) as r:
            resp = json.loads(r.read())
        if resp.get("status") == 200:
            return resp["data"]
        return None
    except Exception as e:
        print(f"Error checking firmware version: {e}")
        return None


def download_firmware(version_string):
    """Download firmware binary from Feetech server."""
    url = f"{FIRMWARE_API_BASE}{FIRMWARE_API_DOWNLOAD}{version_string}"
    try:
        with urllib.request.urlopen(url, timeout=30) as r:
            resp = json.loads(r.read())
        if resp.get("status") == 200:
            data = resp["data"]
            fw_b64 = data["wenjian"]
            fw_bytes = base64.b64decode(fw_b64)
            return {
                "filename": data["filename"],
                "version": data["banben"],
                "model": data["xinghao"],
                "data": fw_bytes,
            }
        return None
    except Exception as e:
        print(f"Error downloading firmware: {e}")
        return None


# === Bootloader Protocol ===

def enter_bootloader(ser, servo_id, bus_baudrate=1000000, mode="software"):
    """
    Send the bootloader entry command.

    Protocol (reverse-engineered from FD.exe at 0x40FE50):

    Path A (mode="dtr"): For Feetech USB adapters with DTR wired to BOOT0.
        1. Send instruction 0x08 to the servo
        2. Set baud 500000, toggle DTR
        3. Wait for 'C' (0x43) at baud 50

    Path B (mode="software"): For generic USB-serial adapters (Waveshare, etc.).
        1. Send instruction 0x08 to the servo
        2. Send the servo_id byte raw at current baud rate
        3. Switch baud to 500000
        4. Sleep 100ms (no 'C' wait)
    """
    print(f"  Sending bootloader entry command to servo ID {servo_id} (mode={mode})...")

    # Step 1: Send instruction 0x08 (enter bootloader) at current baud rate
    feetech_send(ser, servo_id, INST_BOOTLOADER)
    time.sleep(0.015)  # Small delay for servo to process

    if mode == "dtr":
        # === Path A: DTR hardware mode ===
        ser.baudrate = 500000
        time.sleep(0.01)

        # Toggle DTR to reset the motor with BOOT0 pulled high
        ser.dtr = True
        time.sleep(0.1)
        ser.dtr = False
        time.sleep(0.1)

        # Wait for 'C' at low baud rate
        ser.baudrate = 50
        ser.reset_input_buffer()
        ser.timeout = 0.5

        print("  Waiting for bootloader 'C' response...")
        for attempt in range(20):
            data = ser.read(1)
            if data and data[0] == BOOTLOADER_ACK:
                print(f"  Bootloader responded on attempt {attempt + 1}!")
                return True
            if data:
                print(f"  Got unexpected byte: 0x{data[0]:02x} (attempt {attempt + 1})")

        print("  ERROR: Bootloader did not respond after 20 attempts")
        return False

    else:
        # === Path B: Software mode (generic USB-serial adapters) ===
        # The firmware's instruction 0x08 handler jumps directly to the
        # Feetech bootloader at 0x08007800. Switch to 500k immediately
        # since the bootloader communicates at that baud rate.
        ser.baudrate = 500000
        time.sleep(0.05)

        ser.timeout = 0.5
        ser.reset_input_buffer()

        print("  Bootloader entry sent (software mode)")
        return True


def send_bootloader_magic(ser, firmware_type="bin"):
    """Send the bootloader magic sequence.

    From FD.exe 0x4100B0:
      .bin mode (bus servos):  sends "1fBVA"
      .xbin mode (open servos): sends "ABV1f"
    """
    if firmware_type == "xbin":
        magic = BOOTLOADER_MAGIC_XBIN
    else:
        magic = BOOTLOADER_MAGIC_BUS

    # Ensure we're at 500000 baud (should already be set by enter_bootloader)
    ser.baudrate = 500000
    time.sleep(0.01)
    ser.reset_input_buffer()

    print(f"  Sending bootloader magic: {magic!r}")
    ser.write(magic)
    ser.flush()
    time.sleep(0.05)


def send_firmware_block(ser, block_num, data_block, is_last_block=False, firmware_type="bin"):
    """
    Send a single 64-byte firmware block.

    Packet format (reverse-engineered):
        [block_num & 0xFF]      - Block sequence number
        [~block_num & 0xFF]     - Complement of block number
        [flag]                  - Transfer flag (4=normal, 5=last block, 6=xbin)
        [64 bytes data]         - Firmware data (encrypted)
        [CRC16_hi]              - CRC16 high byte
        [CRC16_lo]              - CRC16 low byte
        [type_byte]             - End-of-transfer flag (4=last, 6=normal for .bin)
    """
    # Pad block to 64 bytes with 0xFF if needed
    if len(data_block) < BLOCK_SIZE:
        data_block = data_block + b'\xff' * (BLOCK_SIZE - len(data_block))

    # Build packet
    block_byte = block_num & 0xFF
    complement = (~block_num) & 0xFF

    # Determine flag byte
    # From disassembly at 0x410020: lea eax, [eax + eax + 4]
    # where eax = setne (1 if not last block, 0 if last)
    # So: not_last=1 -> 1*2+4=6; last=0 -> 0*2+4=4
    if is_last_block:
        flag = 4
    else:
        flag = 6

    # End-of-transfer flag in byte 69 (the bootloader checks THIS byte for == 4,
    # NOT byte 2). Byte 69 is outside the CRC scope (CRC covers bytes 0-63).
    # FD.exe puts the same flag value here: 4 = last block, 6 = normal block.
    # For .xbin firmware, normal blocks use 0x01 (different protocol variant).
    if firmware_type == "xbin":
        type_byte = 0x01
    else:
        type_byte = flag  # 4 for last block, 6 for normal

    # Build the header + data portion first
    header = bytes([block_byte, complement, flag])

    # Calculate CRC16 - CRITICAL: scope differs between .bin and .xbin!
    # From disassembly at 0x40FC80 (build_block_packet):
    #   .bin mode:  CRC16 over packet[0:64] = header(3) + first 61 data bytes
    #   .xbin mode: CRC16 over packet[3:67] = all 64 data bytes
    if firmware_type == "xbin":
        crc = crc16_ccitt(data_block)  # CRC over 64 data bytes
    else:
        crc_input = (header + data_block)[:64]  # CRC over first 64 bytes of packet
        crc = crc16_ccitt(crc_input)

    crc_hi = (crc >> 8) & 0xFF
    crc_lo = crc & 0xFF

    packet = header + data_block + bytes([crc_hi, crc_lo, type_byte])

    ser.write(packet)
    ser.flush()

    return packet


def read_block_response(ser, timeout=2.0, mode="software"):
    """
    Read the bootloader's response to a firmware block.
    Returns the status code (0x06=ACK, 0x15=NAK, -1=timeout).

    From FD.exe 0x40FD70 (send_and_read):
      Path 1 (DTR mode): reads 7 bytes, status at response[1]
      Path 2 (software mode / fallback): reads 1 byte, returns it directly
    """
    ser.timeout = timeout

    if mode == "dtr":
        # Path 1: read 7 bytes, status at offset 1
        response = ser.read(7)
        if len(response) < 2:
            return -1, response
        status = response[1]
        return status, response
    else:
        # Path 2: read 1 byte, that byte IS the status
        response = ser.read(1)
        if len(response) < 1:
            return -1, response
        status = response[0]
        return status, response


# === Flash Operation ===

def flash_firmware(ser, servo_id, firmware_data, firmware_type="bin",
                   bus_baudrate=1000000, mode="software"):
    """
    Flash firmware to a servo.

    Args:
        ser: Serial port object
        servo_id: Target servo ID
        firmware_data: Raw firmware bytes (encrypted, from server or file)
        firmware_type: "bin" for bus servos, "xbin" for open servos
        bus_baudrate: Current bus baud rate
        mode: "software" (generic adapters) or "dtr" (Feetech USB adapters)
    """
    # Firmware data starts at byte 0 of the file (no header to skip).
    # FD.exe internally prepends a 2-byte file-size header in its buffer,
    # but the .bin file itself is pure firmware data from byte 0.
    fw_payload = firmware_data
    num_blocks = (len(fw_payload) + BLOCK_SIZE - 1) // BLOCK_SIZE
    print(f"\nFirmware: {len(firmware_data)} bytes, {num_blocks} blocks")

    # Step 0: Silence other servos on the bus
    # Factory firmware responds to garbage bytes when the bus switches to 500k
    # for bootloader communication, causing bus contention. Writing return_level=0
    # via broadcast disables UART responses on all servos, preventing collisions.
    print("\n[Step 0] Silencing other servos on bus...")
    REG_RETURN_LEVEL = 8
    feetech_send(ser, 0xFE, INST_WRITE, bytes([REG_RETURN_LEVEL, 0]))
    time.sleep(0.01)
    ser.reset_input_buffer()

    # Step 1: Enter bootloader
    print("\n[Step 1] Entering bootloader mode...")
    if not enter_bootloader(ser, servo_id, bus_baudrate, mode=mode):
        print("Failed to enter bootloader mode.")
        print("Make sure the servo is powered and connected.")
        return False

    # Step 2: Send magic sequence
    print("\n[Step 2] Sending bootloader magic...")
    send_bootloader_magic(ser, firmware_type)

    # Step 3: Read magic response
    print("\n[Step 3] Reading magic response...")
    if mode == "dtr":
        ser.timeout = 2.0
        init_resp = ser.read(7)
        if len(init_resp) < 7:
            print(f"  Warning: Short initial response ({len(init_resp)} bytes): {init_resp.hex() if init_resp else 'empty'}")
        else:
            print(f"  Magic response: {init_resp.hex()}")
    else:
        # Software mode: read 1 byte response to magic
        ser.timeout = 1.0
        init_resp = ser.read(1)
        if init_resp:
            print(f"  Magic response: 0x{init_resp[0]:02x}")
            if init_resp[0] == BLOCK_ACK:
                print("  Got ACK - bootloader ready!")
            elif init_resp[0] == BLOCK_NAK:
                print("  Got NAK - bootloader rejected magic, aborting")
                return False
        else:
            print("  Warning: No response to magic (timeout)")

    # Step 3b: Send 0x01 handshake byte (from FD.exe 0x40FFE7)
    # This tells the bootloader to prepare for block reception
    print("\n[Step 3b] Sending start-transfer handshake (0x01)...")
    ser.write(bytes([0x01]))
    ser.flush()
    if mode == "dtr":
        ser.timeout = 2.0
        handshake_resp = ser.read(7)
        if len(handshake_resp) >= 2:
            print(f"  Handshake response: {handshake_resp.hex()} (status=0x{handshake_resp[1]:02x})")
        else:
            print(f"  Handshake response: {handshake_resp.hex() if handshake_resp else 'empty'}")
    else:
        ser.timeout = 1.0
        handshake_resp = ser.read(1)
        if handshake_resp:
            print(f"  Handshake response: 0x{handshake_resp[0]:02x}")
        else:
            print("  Warning: No handshake response (timeout)")

    # Step 4: Transfer firmware blocks (data starts at firmware[2])
    print(f"\n[Step 4] Transferring {num_blocks} blocks...")
    block_num = 1
    retries = 0
    max_retries = 3

    while block_num <= num_blocks:
        offset = (block_num - 1) * BLOCK_SIZE
        block_data = fw_payload[offset:offset + BLOCK_SIZE]
        is_last = (block_num == num_blocks)

        # Send block
        send_firmware_block(ser, block_num, block_data, is_last, firmware_type)

        # Read response
        status, response = read_block_response(ser, mode=mode)

        # Inter-block delay: the bootloader needs time to decrypt and write
        # to flash before it can receive the next block. GD32F130 flash write
        # takes ~150µs per halfword (32 writes = 4.8ms per block). Page erase
        # takes ~40ms (every 16 blocks). Without this delay, the next block's
        # bytes can arrive while the bootloader is still writing flash,
        # causing byte loss on the 1-byte-deep UART RX buffer.
        if status == BLOCK_ACK:
            if block_num % 16 == 0:
                time.sleep(0.05)   # extra delay at page boundary
            else:
                time.sleep(0.01)   # 10ms inter-block delay

        if status == BLOCK_ACK:
            progress = block_num * 100 // num_blocks
            print(f"\r  Block {block_num}/{num_blocks} ({progress}%) - OK", end="", flush=True)
            block_num += 1
            retries = 0
        elif status == BLOCK_NAK:
            retries += 1
            print(f"\n  Block {block_num} NAK'd, retry {retries}/{max_retries}")
            if retries >= max_retries:
                print(f"\n  ERROR: Block {block_num} failed after {max_retries} retries")
                return False
        else:
            retries += 1
            resp_hex = response.hex() if response else "empty"
            print(f"\n  Block {block_num} unexpected response (status={status}): {resp_hex}")
            print(f"  Retry {retries}/{max_retries}")
            if retries >= max_retries:
                print(f"\n  ERROR: Block {block_num} failed after {max_retries} retries")
                return False

    print(f"\n\n[Step 5] Firmware transfer complete!")

    # Restore original baud rate and re-enable responses on other servos
    ser.baudrate = bus_baudrate
    time.sleep(0.5)
    feetech_send(ser, 0xFE, INST_WRITE, bytes([REG_RETURN_LEVEL, 1]))
    time.sleep(0.01)
    ser.reset_input_buffer()

    return True


# === Scan ===

def scan_servos(ser, max_id=20):
    """Scan for servos on the bus and display their info."""
    print(f"Scanning for servos (ID 0-{max_id})...\n")
    found = []
    for sid in range(max_id + 1):
        resp = ping_servo(ser, sid)
        if resp is not None:
            info = read_servo_info(ser, sid)
            version_str = f"{info.get('servo_major', '?')}.{info.get('servo_minor', '?')}"
            fw_str = f"{info.get('fw_major', '?')}.{info.get('fw_minor', '?')}"

            # Check for firmware update
            update_info = check_firmware_version(version_str) if version_str != "?.?" else None
            update_str = ""
            if update_info:
                update_str = f" -> latest: v{update_info['banben']} ({update_info['filename']})"
                if update_info["banben"] == fw_str:
                    update_str = " [up to date]"

            print(f"  ID {sid:3d}: servo={version_str}  firmware=v{fw_str}  error={resp['error']}{update_str}")
            found.append(sid)

    if not found:
        print("  No servos found!")
    return found


# === Main ===

def main():
    parser = argparse.ArgumentParser(
        description="Feetech Servo Firmware Flash Tool (Linux)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Scan for servos:
    %(prog)s --port /dev/ttyACM1 --scan

  Check available firmware:
    %(prog)s --port /dev/ttyACM1 --servo-id 1 --check

  Flash latest firmware from server:
    %(prog)s --port /dev/ttyACM1 --servo-id 1 --flash

  Flash from local file:
    %(prog)s --port /dev/ttyACM1 --servo-id 1 --flash --firmware file.bin

  List known firmware versions:
    %(prog)s --list-firmware
""",
    )

    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyACM1)")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Bus baud rate (default: 1000000)")
    parser.add_argument("--servo-id", type=int, help="Target servo ID")
    parser.add_argument("--scan", action="store_true", help="Scan for servos on the bus")
    parser.add_argument("--check", action="store_true", help="Check for firmware updates")
    parser.add_argument("--flash", action="store_true", help="Flash firmware to servo")
    parser.add_argument("--firmware", help="Local firmware file (skip download)")
    parser.add_argument("--firmware-type", choices=["bin", "xbin"], default="bin",
                        help="Firmware type (default: bin)")
    parser.add_argument("--mode", choices=["software", "dtr"], default="software",
                        help="Bootloader entry mode: 'software' for generic USB-serial "
                             "adapters (Waveshare, CH340, etc.), 'dtr' for Feetech USB "
                             "adapters with DTR wired to BOOT0 (default: software)")
    parser.add_argument("--list-firmware", action="store_true",
                        help="List known firmware versions from server")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="Skip confirmation prompts")

    args = parser.parse_args()

    # List firmware mode (no serial port needed)
    if args.list_firmware:
        print("Querying Feetech firmware server...\n")
        known_models = [
            ("6.16", "SMS (STM32, RS485)"),
            ("8.0", "SMS v2 (STM32, RS485)"),
            ("9.2", "SCS/STS - STS3215"),
            ("9.3", "SCS/STS - STS3251"),
            ("9.15", "SCS/STS - older"),
            ("10.3", "HTS (GD32, TTL)"),
            ("10.8", "HLS (GD32, TTL)"),
            ("10.6", "ST (STM32, TTL)"),
            ("20.3", "SMS MODBUS (STM32, RS485)"),
        ]
        print(f"  {'Model':8s} {'Description':30s} {'Latest FW':10s} {'Filename'}")
        print(f"  {'─' * 8} {'─' * 30} {'─' * 10} {'─' * 40}")
        for model, desc in known_models:
            info = check_firmware_version(model)
            if info:
                print(f"  {model:8s} {desc:30s} v{info['banben']:9s} {info['filename']}")
            else:
                print(f"  {model:8s} {desc:30s} {'N/A':10s}")
        return

    # Open serial port
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
    except serial.SerialException as e:
        print(f"Error opening serial port {args.port}: {e}")
        sys.exit(1)

    try:
        # Scan mode
        if args.scan:
            scan_servos(ser)
            return

        # Need servo ID for remaining operations
        if args.servo_id is None:
            print("Error: --servo-id is required for --check and --flash operations")
            sys.exit(1)

        # Read servo info
        print(f"Reading servo ID {args.servo_id} info...")
        info = read_servo_info(ser, args.servo_id)
        if not info:
            print(f"Error: No response from servo ID {args.servo_id}")
            sys.exit(1)

        version_str = f"{info['servo_major']}.{info['servo_minor']}"
        fw_str = f"{info['fw_major']}.{info['fw_minor']}"
        print(f"  Servo model version: {version_str}")
        print(f"  Current firmware:    v{fw_str}")

        # Check mode
        if args.check:
            print(f"\nChecking for updates (model {version_str})...")
            update = check_firmware_version(version_str)
            if update:
                print(f"  Latest firmware: v{update['banben']}")
                print(f"  Filename: {update['filename']}")
                if update["banben"] == fw_str:
                    print("  Status: Already up to date!")
                else:
                    print(f"  Status: Update available (v{fw_str} -> v{update['banben']})")
            else:
                print("  No firmware found on server for this model.")
            return

        # Flash mode
        if args.flash:
            # Get firmware data
            if args.firmware:
                # Load from local file
                print(f"\nLoading firmware from {args.firmware}...")
                with open(args.firmware, "rb") as f:
                    fw_data = f.read()
                print(f"  Loaded {len(fw_data)} bytes")

                # Auto-encrypt raw .bin files for the bootloader.
                # Server-downloaded firmware is already encrypted, but local
                # builds are plaintext. The bootloader always decrypts, so
                # sending unencrypted data results in garbage.
                if args.firmware.endswith(".bin"):
                    try:
                        from encrypt import encrypt_firmware
                        print("  Encrypting for bootloader...")
                        fw_data = encrypt_firmware(fw_data)
                        print(f"  Encrypted: {len(fw_data)} bytes")
                    except ImportError:
                        print("Error: encrypt.py or pycryptodome not found.")
                        print("Cannot flash unencrypted .bin — bootloader would decrypt to garbage.")
                        print("Install: pip install pycryptodome")
                        sys.exit(1)
            else:
                # Download from server
                print(f"\nDownloading firmware for model {version_str}...")
                fw = download_firmware(version_str)
                if not fw:
                    print("Error: Failed to download firmware")
                    sys.exit(1)
                print(f"  Filename: {fw['filename']}")
                print(f"  Version:  v{fw['version']}")
                print(f"  Size:     {len(fw['data'])} bytes")
                fw_data = fw["data"]

            # Confirmation
            if not args.yes:
                print(f"\n*** WARNING ***")
                print(f"This will flash firmware to servo ID {args.servo_id}.")
                print(f"If the process is interrupted, the servo may be bricked.")
                print(f"Make sure power is stable and do not disconnect during flashing.")
                resp = input("\nProceed? [y/N] ")
                if resp.lower() != "y":
                    print("Aborted.")
                    return

            # Flash!
            success = flash_firmware(
                ser, args.servo_id, fw_data,
                firmware_type=args.firmware_type,
                bus_baudrate=args.baudrate,
                mode=args.mode,
            )

            if success:
                print("\nFirmware flash completed successfully!")
                print("The servo should reboot automatically.")
                print("Wait a few seconds, then verify with --scan or --check.")
            else:
                print("\nFirmware flash FAILED!")
                print("The servo may need to be recovered via SWD/JTAG.")
                sys.exit(1)

            return

        # Default: show info
        print("\nUse --scan, --check, or --flash to perform an operation.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()

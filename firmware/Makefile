# Feetech STS3215 Firmware — GD32F130C6T6
#
# Build: make
# Build with GCC 6.3.1 (via QEMU): make gcc6
# Build hybrid (no -ffunction-sections for src/): make gcc6-hybrid
# Clean: make clean
# Compare with original: make compare
# Compare GCC 6.3.1 build: make compare-gcc6
# Disassemble: make disasm

TARGET = scservo21

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

# GCC 6.3.1 toolchain (via QEMU wrappers, set by gcc6 target)
GCC6_CC = /tmp/gcc6-wrapper.sh
GCC6_AS = /tmp/as6-wrapper.sh
GCC6_LD = /tmp/ld6-wrapper.sh
GCC6_OBJCOPY = /tmp/objcopy6-wrapper.sh
GCC6_SIZE = /tmp/size6-wrapper.sh
GCC6_EXTRA_CFLAGS = -fno-crossjumping -fno-optimize-sibling-calls -fno-gcse -fno-if-conversion2 -fno-peephole2

# Directories
BUILD_DIR = build
SRC_DIR = src
LIB_DIR = lib
LINKER_DIR = linker

# Source files
# Note: SPL library sources removed — no SPL functions are called.
# SPL headers (lib/SPL/Include/) are still included for register definitions.
SRC_SOURCES = \
    $(SRC_DIR)/main.c \
    $(SRC_DIR)/uart_protocol.c \
    $(SRC_DIR)/pid_motor.c \
    $(SRC_DIR)/eeprom.c \
    $(SRC_DIR)/vtables.c \
    $(SRC_DIR)/util.c \
    $(SRC_DIR)/encoder.c \
    $(SRC_DIR)/i2c.c \
    $(SRC_DIR)/led.c \
    $(SRC_DIR)/adc.c \
    $(SRC_DIR)/motion.c \
    $(SRC_DIR)/pid.c \
    $(SRC_DIR)/motor.c \
    $(SRC_DIR)/uart_subsys.c \
    $(SRC_DIR)/servo_regs.c

C_SOURCES = $(SRC_SOURCES)

ASM_SOURCES = \
    $(LINKER_DIR)/startup_gd32f130.s

# Include paths
C_INCLUDES = \
    -I$(SRC_DIR) \
    -I$(LIB_DIR)/CMSIS \
    -I$(LIB_DIR)/SPL/Include

# MCU flags
MCU = -mcpu=cortex-m3 -mthumb

# C flags — match original firmware's likely build config
CFLAGS = $(MCU) -Os -Wall
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -DGD32F130_150
CFLAGS += $(C_INCLUDES)

# Assembler flags
ASFLAGS = $(MCU)

# Linker flags
LDSCRIPT = $(LINKER_DIR)/GD32F130C6T.ld
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT)
LDFLAGS += -lc -lm -lnosys
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref

# Reference binary for comparison
REFERENCE = ../firmware/decrypted/decrypted_SCServo21-GD32-TTL-250306.bin

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Build rules
all: $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@
	@python3 -c "\
	import subprocess; \
	nm=subprocess.check_output(['$(PREFIX)nm','$<']).decode(); \
	syms={l.split()[2]:int(l.split()[0],16) for l in nm.strip().split('\n') if len(l.split())>=3}; \
	fw_end=syms['_sidata']+syms['_edata']-syms['_sdata']-0x08000000; \
	d=bytearray(open('$@','rb').read()[:fw_end]); \
	d+=b'\xff'*((64-len(d)%64)%64); \
	open('$@','wb').write(d); \
	print(f'  Firmware: {fw_end} bytes, padded to {len(d)} ({len(d)//64} blocks)')"

$(BUILD_DIR):
	mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	@$(SIZE) $<

compare: $(BUILD_DIR)/$(TARGET).bin
	@echo "=== Size comparison ==="
	@echo -n "Original:  "; wc -c < $(REFERENCE)
	@echo -n "Ours:      "; wc -c < $<
	@echo ""

disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -d -S $< > $(BUILD_DIR)/$(TARGET).dis
	@echo "Disassembly written to $(BUILD_DIR)/$(TARGET).dis"

clean:
	rm -rf $(BUILD_DIR)

# GCC 6.3.1 build (slow, ~3-5 min via QEMU; uses -j1 to avoid parallel failures)
gcc6: clean
	$(MAKE) -j1 all \
		CC="$(GCC6_CC)" \
		AS="$(GCC6_AS)" \
		LD="$(GCC6_LD)" \
		OBJCOPY="$(GCC6_OBJCOPY)" \
		SIZE="$(GCC6_SIZE)" \
		CFLAGS="$(MCU) -Os -Wall -fdata-sections -ffunction-sections -DGD32F130_150 $(C_INCLUDES) $(GCC6_EXTRA_CFLAGS)"

compare-gcc6: gcc6
	@echo "=== Size comparison (GCC 6.3.1 build) ==="
	@echo -n "Original:  "; wc -c < $(REFERENCE)
	@echo -n "Ours:      "; wc -c < $(BUILD_DIR)/$(TARGET).bin
	@echo ""

# GCC 6.3.1 hybrid build: src/ without -ffunction-sections (literal pool sharing).
# Note: SPL sources removed — no SPL functions are called.
GCC6_HYBRID_SRC_CFLAGS = $(MCU) -Os -Wall -fdata-sections -DGD32F130_150 $(C_INCLUDES) $(GCC6_EXTRA_CFLAGS)

SRC_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC_SOURCES:.c=.o)))
ASM_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

gcc6-hybrid: clean | $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)
	@echo "=== Compiling src/ WITHOUT -ffunction-sections ==="
	@for f in $(SRC_SOURCES); do \
		obj=$(BUILD_DIR)/$$(basename $${f%.c}.o); \
		echo "  CC $$f -> $$obj"; \
		$(GCC6_CC) -c $(GCC6_HYBRID_SRC_CFLAGS) $$f -o $$obj || exit 1; \
	done
	@echo "=== Assembling ==="
	@for f in $(ASM_SOURCES); do \
		obj=$(BUILD_DIR)/$$(basename $${f%.s}.o); \
		echo "  AS $$f -> $$obj"; \
		$(GCC6_AS) -c $(ASFLAGS) $$f -o $$obj || exit 1; \
	done
	@echo "=== Linking ==="
	$(GCC6_LD) $(SRC_OBJECTS) $(ASM_OBJECTS) $(LDFLAGS) -o $(BUILD_DIR)/$(TARGET).elf
	$(GCC6_OBJCOPY) -O binary $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	@$(GCC6_SIZE) $(BUILD_DIR)/$(TARGET).elf

compare-gcc6-hybrid: gcc6-hybrid
	@echo "=== Size comparison (GCC 6.3.1 hybrid build) ==="
	@echo -n "Original:  "; wc -c < $(REFERENCE)
	@echo -n "Ours:      "; wc -c < $(BUILD_DIR)/$(TARGET).bin
	@echo ""

# Encrypt firmware for flashing via bootloader
ENCRYPT = python3 ../tools/encrypt.py

encrypt: $(BUILD_DIR)/$(TARGET).bin
	$(ENCRYPT) $< -o $(BUILD_DIR)/$(TARGET).enc.bin

# Flash custom firmware to servo (requires: pip install pyserial pycryptodome)
# Usage: make flash PORT=/dev/ttyACM0 ID=1
PORT ?= /dev/ttyACM0
ID ?= 1
FLASH_TOOL = python3 ../tools/flash_tool.py

flash: encrypt
	$(FLASH_TOOL) --port $(PORT) --servo-id $(ID) --firmware $(BUILD_DIR)/$(TARGET).enc.bin

# Flash factory firmware (downloads from Feetech servers)
flash-factory:
	$(FLASH_TOOL) --port $(PORT) --servo-id $(ID)

# Run all tests
test: all
	@echo "=== full_diff_test ===" && python3 test/full_diff_test.py 2>&1 | tail -3
	@echo "=== system_test ===" && python3 test/system_test.py 2>&1 | tail -3
	@echo "=== periph_diff_test ===" && python3 test/periph_diff_test.py 2>&1 | tail -3
	@echo "=== coverage_test ===" && python3 test/coverage_test.py 2>&1 | tail -3
	@echo "=== fuzz_test ===" && python3 test/fuzz_test.py 2>&1 | tail -3

.PHONY: all clean size compare disasm gcc6 compare-gcc6 gcc6-hybrid compare-gcc6-hybrid encrypt flash flash-factory test

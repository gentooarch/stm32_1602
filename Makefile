# =============================================================
# STM32F103 Bare Metal Makefile (NO STD PERIPH, NO HAL)
# Target: STM32F103C8T6 — Pure Register-Level Code Only
# =============================================================

ARMGNU = arm-none-eabi

CC = $(ARMGNU)-gcc
AS = $(ARMGNU)-as
OBJCOPY = $(ARMGNU)-objcopy
OBJDUMP = $(ARMGNU)-objdump
SIZE = $(ARMGNU)-size

PROJECT = STM32F103_LED_I2C
TARGET = $(PROJECT).elf
BINFILE = $(PROJECT).bin
HEXFILE = $(PROJECT).hex

SRC_DIR = src
INC_DIRS = inc CMSIS/device CMSIS/core
STARTUP_DIR = startup

STARTUP_SOURCES = $(STARTUP_DIR)/startup_stm32.s
C_SOURCES = $(SRC_DIR)/main.c

STARTUP_OBJECTS = $(STARTUP_SOURCES:.s=.o)
C_OBJECTS = $(C_SOURCES:.c=.o)

# --- 关键修改：移除所有 StdPeriph 宏和路径 ---
CFLAGS = -Og -g3 -Wall -Wextra -Wno-unused-parameter \
         -DSTM32F10X_MD \
         -Iinc -ICMSIS/device -ICMSIS/core \
         -mcpu=cortex-m3 -march=armv7-m -mthumb -mfloat-abi=soft -nostdlib

LDFLAGS = -TLinkerScript.ld -Wl,--gc-sections

all: $(TARGET) $(BINFILE) $(HEXFILE) size

%.o: %.c
	@echo "🔧 Compiling $< ..."
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	@echo "🧩 Assembling $< ..."
	$(AS) -mcpu=cortex-m3 -march=armv7-m -mthumb -mfloat-abi=soft -c $< -o $@

$(TARGET): $(STARTUP_OBJECTS) $(C_OBJECTS)
	@echo "🔗 Linking $@ ..."
	$(CC) $(LDFLAGS) $^ -o $@ -nostdlib -nostartfiles

$(BINFILE): $(TARGET)
	@echo "💾 Generating $@ ..."
	$(OBJCOPY) -O binary $< $@

$(HEXFILE): $(TARGET)
	@echo "🖨️  Generating $@ ..."
	$(OBJCOPY) -O ihex $< $@

size: $(TARGET)
	@echo "📊 Size of $(TARGET):"
	$(SIZE) $<

clean:
	rm -f $(TARGET) $(BINFILE) $(HEXFILE) $(STARTUP_OBJECTS) $(C_OBJECTS)
	@echo "🗑️  Cleaned all build files."

flash: $(BINFILE)
	@echo "🚀 Flashing $< to 0x08000000..."
	st-flash --reset write $< 0x08000000

.PHONY: all clean size flash

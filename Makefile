
# Project
TARGET    := contrologger
BUILD_DIR := build

# Toolchain
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

# MCU
MCU := -mcpu=cortex-m4 \
       -mthumb \
       -mfpu=fpv4-sp-d16 \
       -mfloat-abi=hard

# Preprocessor definitions
C_DEFS := \
    -DUSE_HAL_DRIVER \
    -DSTM32F411xE \
    -DHSE_VALUE=25000000U

# Directories
PLATFORM_DIR := platform/stm32f411
HAL_DIR      := external/stm32_hal
CMSIS_DIR    := external/cmsis

# Include paths
C_INCLUDES := \
    -Iapp \
    -Icommon \
    -Idrivers/mcu \
    -Idrivers/devices \
    -Idrivers/bus \
    -I$(PLATFORM_DIR) \
    -I$(HAL_DIR)/Inc \
    -I$(HAL_DIR)/Inc/Legacy \
    -I$(CMSIS_DIR)/Include \
    -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include


# Application and peripheral sources
PROJECT_SOURCES := \
    main.c \
    $(wildcard app/*.c) \
    $(wildcard common/*.c) \
    $(wildcard drivers/mcu/*.c) \
    $(wildcard drivers/devices/*.c) \
    $(wildcard drivers/bus/*.c) \
    $(wildcard $(PLATFORM_DIR)/*.c)

# STM32 HAL sources
HAL_SOURCES := \
    $(HAL_DIR)/Src/stm32f4xx_hal.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_rcc_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_pwr.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_pwr_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_flash.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_flash_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_dma.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_dma_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_adc.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_adc_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_i2c.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_i2c_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_spi.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_tim.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_tim_ex.c \
    $(HAL_DIR)/Src/stm32f4xx_hal_uart.c

C_SOURCES := $(PROJECT_SOURCES) $(HAL_SOURCES)


# Startup
ASM_SOURCES := \
    $(PLATFORM_DIR)/startup_stm32f411xe.s

# Compiler flags
CFLAGS := \
    $(MCU) \
    $(C_DEFS) \
    $(C_INCLUDES) \
    -std=c11 \
    -Og \
    -g3 \
    -Wall \
    -Wextra \
    -ffunction-sections \
    -fdata-sections \
    -MMD \
    -MP

ASFLAGS := \
    $(MCU) \
    -x assembler-with-cpp \
    -MMD \
    -MP

# Linker
LDSCRIPT := $(PLATFORM_DIR)/STM32F411xx_FLASH.ld

LDFLAGS := \
    $(MCU) \
    -T$(LDSCRIPT) \
    -specs=nano.specs \
    -specs=nosys.specs \
    -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
    -Wl,--gc-sections \
    -Wl,--print-memory-usage

LIBS := -lc -lm

# Output files
ELF := $(BUILD_DIR)/$(TARGET).elf
HEX := $(BUILD_DIR)/$(TARGET).hex
BIN := $(BUILD_DIR)/$(TARGET).bin

# Object files
C_OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES))
ASM_OBJECTS := $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASM_SOURCES))

OBJECTS := $(C_OBJECTS) $(ASM_OBJECTS)

# Default target
all: $(ELF) $(HEX) $(BIN)

# Compile C
$(BUILD_DIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup
$(BUILD_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

# Link
$(ELF): $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) \
	    -Wl,--start-group $(LIBS) -Wl,--end-group \
	    -o $@
	$(SIZE) $@

# Generate HEX
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

# Generate BIN
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

# OpenOCD
OPENOCD := openocd

OPENOCD_INTERFACE := interface/stlink.cfg
OPENOCD_TARGET    := target/stm32f4x.cfg
SWD_SPEED         := 2000

# Flash using ST-Link V2
flash: $(ELF)
	$(OPENOCD) \
	    -f $(OPENOCD_INTERFACE) \
	    -f $(OPENOCD_TARGET) \
	    -c "adapter speed $(SWD_SPEED)" \
	    -c "program $(ELF) verify reset exit"

# Start GDB server
debug:
	$(OPENOCD) \
	    -f $(OPENOCD_INTERFACE) \
	    -f $(OPENOCD_TARGET) \
	    -c "adapter speed $(SWD_SPEED)"

# Firmware size
size: $(ELF)
	$(SIZE) $<

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Automatic header dependencies
-include $(OBJECTS:.o=.d)

.PHONY: all flash debug size clean

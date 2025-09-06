# -------- Project --------
TARGET      := nuc140_music_player
BUILD       := build
CROSS       ?= arm-none-eabi-
CC          := $(CROSS)gcc
AS          := $(CROSS)gcc
AR          := $(CROSS)ar
OBJCOPY     := $(CROSS)objcopy
OBJDUMP     := $(CROSS)objdump
SIZE        := $(CROSS)size

LDSCRIPT    ?= nuc140.ld

# -------- Sources --------
SRCS := \
  Library/ff8/src/ff.c \
  Library/ff8/src/diskio.c \
  Library/NUC1xx-LB_002/Source/LCD.c \
  Library/NUC1xx-LB_002/Source/SDCard.c \
  Library/NUC1xx-LB_002/Source/Scankey.c \
  Library/misc_gcc_compatibility/retarget_gcc.c \
  Library/libadpcm/adpcm_ima.c \
  Library/NUC1xx/Source/GPIO.c \
  Library/NUC1xx/Source/SPI.c \
  Library/NUC1xx/Source/SYS.c \
  Library/NUC1xx/Source/I2C.c \
  Library/NUC1xx/Source/I2S.c \
  Library/NUC1xx/Source/UART.c \
  Library/Device/Nuvoton/NUC1xx/Source/system_NUC1xx.c

SRCS += $(wildcard src/nuc140_music_player/*.c)

ASMS := \
  Library/misc_gcc_compatibility/startup_gcc.S

# -------- Includes --------
INCLUDES := \
  -ILibrary/ff8/src \
  -ILibrary/CMSIS/Include \
  -ILibrary/Device/Nuvoton/NUC1xx/Include \
  -ILibrary/Device/Nuvoton/NUC1xx/Source \
  -ILibrary/NUC1xx/Include \
  -ILibrary/NUC1xx-LB_002/Include \
  -Isrc/nuc140_music_player/ \
  -Isrc/nuc140_music_player/include 

# -------- Flags --------
COMMON := -mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -fno-builtin -fno-common
DEFS   := -D__EVAL -D__UVISION_VERSION=542
CSTD   := -std=gnu99
OPT    ?= -O1
DBG    ?= -g3

CFLAGS := $(COMMON) $(CSTD) $(OPT) $(DBG) $(DEFS) $(INCLUDES) -MMD -MP -Wall -Wextra
ASFLAGS:= $(COMMON) $(OPT) $(DBG) $(DEFS)
LDFLAGS:= $(COMMON) -Wl,--gc-sections -Wl,-Map,$(BUILD)/$(TARGET).map -T $(LDSCRIPT)
LIBS   := -lc -lm -lnosys

# -------- Objects --------
COBJS := $(SRCS:%=$(BUILD)/%.o)
AOBJS := $(ASMS:%=$(BUILD)/%.o)
OBJS  := $(COBJS) $(AOBJS)
DEPS  := $(OBJS:.o=.d)

ifneq ($(strip $(ADPCM_LIB)),)
  LIBS += $(ADPCM_LIB)
endif

# -------- Rules --------
.PHONY: all clean size objdump hex bin

all: $(BUILD)/$(TARGET).elf hex bin size

hex: $(BUILD)/$(TARGET).hex
bin: $(BUILD)/$(TARGET).bin

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "LD  $@"
	@$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@echo "HEX $@"
	@$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@echo "BIN $@"
	@$(OBJCOPY) -O binary $< $@

size: $(BUILD)/$(TARGET).elf
	@$(SIZE) --format=berkeley $<

objdump: $(BUILD)/$(TARGET).elf
	@$(OBJDUMP) -d -S $< > $(BUILD)/$(TARGET).lst

clean:
	@echo "CLEAN"
	@rm -rf $(BUILD)

# -------- Patterns --------
$(BUILD)/%.c.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.s.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/%.S.o: %.S
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) $(ASFLAGS) -c $< -o $@


# -------- Dependency includes --------
-include $(DEPS)

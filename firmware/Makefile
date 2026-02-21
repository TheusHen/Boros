CROSS_COMPILE ?= arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy

SRC_DIR := src
TARGET := firmware

CFLAGS  ?= -mcpu=cortex-m0plus -mthumb -O2 -Wall -Wextra -std=c11
CFLAGS  += -ffunction-sections -fdata-sections -I$(SRC_DIR)
LDFLAGS ?= -T$(SRC_DIR)/link.ld -nostartfiles -Wl,--gc-sections
LDLIBS  ?= -lc -lm -lnosys

ASM_SRCS := $(SRC_DIR)/startup_stm32g071.s
C_SRCS := $(sort $(wildcard $(SRC_DIR)/*.c))
SRCS := $(ASM_SRCS) $(C_SRCS)

OBJS := $(SRCS:.c=.o)
OBJS := $(OBJS:.s=.o)

.PHONY: all clean

all: $(TARGET).elf $(TARGET).bin

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
ifeq ($(OS),Windows_NT)
	-@for %%f in ($(OBJS) $(TARGET).elf $(TARGET).bin) do @if exist "%%f" del /Q /F "%%f"
else
	-$(RM) $(OBJS) $(TARGET).elf $(TARGET).bin
endif

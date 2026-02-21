CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS  ?= -mcpu=cortex-m0plus -mthumb -O2 -Wall -Wextra -std=c11
CFLAGS  += -ffunction-sections -fdata-sections -I. -Isrc
LDFLAGS ?= -Tsrc/link.ld -nostartfiles -Wl,--gc-sections
LDLIBS  ?= -lc -lm -lnosys

SRCS = \
	src/startup_stm32g071.s \
	src/system.c \
	src/gpio.c \
	src/systick.c \
	src/uart.c \
	spi.c \
	flash_w25q.c \
	src/bmp390.c \
	src/icm42688.c \
	log.c \
	src/main.c

OBJS = $(SRCS:.c=.o)
OBJS := $(OBJS:.s=.o)

all: firmware.elf firmware.bin

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

firmware.elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@

clean:
	$(RM) $(OBJS) firmware.elf firmware.bin

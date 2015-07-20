TOOLCHAIN_ROOT := /usr0/home/acolin/ti/msp430-gcc

DEVICE  = msp430f5340
PREFIX = $(TOOLCHAIN_ROOT)/bin/msp430-elf-
CC      = $(PREFIX)gcc
GDB     = $(PREFIX)gdb

SRC_DIR := ../src
LIBDEBUG_ROOT := ../../libdebug

CFLAGS = \
	-mmcu=$(DEVICE) \
	-O1 \
	-g \
	-std=c99 \
	-pedantic \
	-I $(TOOLCHAIN_ROOT)/include \
	-I $(LIBDEBUG_ROOT)/src/include \
	-I $(SRC_DIR) \
	-I $(SRC_DIR)/adc12 \
	-I $(SRC_DIR)/gpio \
	-I $(SRC_DIR)/rfid \
	-I $(SRC_DIR)/timers \
	-I $(SRC_DIR)/uart \
	-I $(SRC_DIR)/i2c \

LFLAGS = -L $(TOOLCHAIN_ROOT)/include

VPATH = $(SRC_DIR)

EXEC = monitor

OBJECTS = \
	catchall.o \
	main.o \
	error.o \
	uart/uart.o \
	i2c/i2c.o \
	timers/pwm.o \
	timers/timeLog.o \
	rfid/rfid.o \
	rfid/rfid_decoder.o \
	adc12/adc12.o \

%.o: %.c
	mkdir -p "./$(shell dirname $@)"
	$(CC) -c $(CFLAGS) $< -o $@

$(EXEC): $(OBJECTS)
	$(CC) $(CFLAGS) $(LFLAGS) $^ -o $(EXEC)

clean:
	rm -f $(OBJECTS) $(EXEC)

flash: $(EXEC)
	mspdebug tilib "prog $(EXEC)"

debug: $(EXEC)
	$(GDB) $(EXEC)

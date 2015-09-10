include ../../../Makefile.env

DEVICE  = msp430f5340
PREFIX = $(TOOLCHAIN_ROOT)/bin/msp430-elf-
CC      = $(PREFIX)gcc
GDB     = $(PREFIX)gdb

SRC_DIR := ../src

CFLAGS = \
	-mmcu=$(DEVICE) \
	-O1 \
	-g \
	-std=c99 \
	-pedantic \
	-Wall \
	-I $(TOOLCHAIN_ROOT)/include \
	-I $(LIBEDB_ROOT)/src/include \
	-I $(SRC_DIR) \
	-I $(SRC_DIR)/adc12 \
	-I $(SRC_DIR)/gpio \
	-I $(SRC_DIR)/rfid \
	-I $(SRC_DIR)/timers \
	-I $(SRC_DIR)/uart \
	-I $(SRC_DIR)/i2c \
	-I $(SRC_DIR)/rfid \
	-I $(SRC_DIR)/mspware \

LFLAGS = -L $(TOOLCHAIN_ROOT)/include

VPATH = $(SRC_DIR)

EXEC = edb

OBJECTS = \
	catchall.o \
	main.o \
	error.o \
	uart.o \
	i2c.o \
	pwm.o \
	systick.o \
	rfid/rfid.o \
	rfid/rfid_decoder.o \
	adc.o \
	mspware/pmm.o \
	params.o \
	charge.o \
	comparator.o \
	codepoint.o \
	host_comm_impl.o \
	target_comm_impl.o \
	tether.o \
	clock.o \

all: $(EXEC)

-include $(OBJECTS:.o=.d)

%.o: %.c
	mkdir -p "./$(shell dirname $@)"
	$(CC) -c -MD $(CFLAGS) $< -o $@

$(EXEC): $(OBJECTS)
	$(CC) $(CFLAGS) $(LFLAGS) $^ -o $(EXEC)

clean:
	rm -f $(OBJECTS) $(OBJECTS:.o=.d) $(EXEC)

flash: $(EXEC)
	mspdebug tilib "prog $(EXEC)"

debug: $(EXEC)
	$(GDB) $(EXEC)

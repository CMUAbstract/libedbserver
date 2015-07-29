ROOT := /usr0/home/acolin
DEV_ROOT := $(ROOT)/dev
TOOLCHAIN_ROOT := $(ROOT)/ti/msp430-gcc

DEVICE  = msp430fr5969
PREFIX = $(TOOLCHAIN_ROOT)/bin/msp430-elf-
CC      = $(PREFIX)gcc
GDB     = $(PREFIX)gdb

SRC_DIR := ../src
LIBDEBUG_ROOT := ../../libdebug
WISP_BASE_ROOT := $(DEV_ROOT)/wisp5/CCS/wisp-base

CFLAGS = \
	-mmcu=$(DEVICE) \
	-O1 \
	-g \
	-std=c99 \
	-pedantic \
	-Wall \
	-I $(TOOLCHAIN_ROOT)/include \
	-I $(WISP_BASE_ROOT) \
	-I $(SRC_DIR) \
	-I $(SRC_DIR)/include/libdebug \

VPATH = $(SRC_DIR)

BINARY = libdebug.a

OBJECTS = \
	debug.o \

all: $(BINARY)

-include $(OBJECTS:.o=.d)

%.o: %.c
	mkdir -p "./$(shell dirname $@)"
	$(CC) -c -MD $(CFLAGS) $< -o $@

$(BINARY): $(OBJECTS)
	$(AR) rcs $@ $^ 

clean:
	rm -f $(OBJECTS) $(OBJECTS:.o=.d) $(BINARY)

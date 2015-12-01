EXEC = edb.out

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
	profile.o \
	payload.o \

BOARD = edb

include $(MAKER_ROOT)/Makefile.gcc

SRC_DIR = ../../src

CFLAGS += \
	-I$(SRC_DIR) \
	-I$(SRC_DIR)/rfid \
	-I$(SRC_DIR)/mspware \
	-I$(LIBEDB_ROOT)/src/include \

VPATH = $(SRC_DIR)

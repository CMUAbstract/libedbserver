EXEC = edb.out

BOARD = sprite-edb-socket-rgz

# Configure the build here (by default all flags are unset)
ifeq ($(BOARD),edb)
CONFIG_SYSTICK = 1
endif

OBJECTS = \
	catchall.o \
	main.o \
	error.o \
	uart.o \
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

ifeq ($(CONFIG_ENABLE_RF_PROTOCOL_MONITORING),1)
	OBJECTS += rfid/rfid.o rfid/rfid_decoder.o
endif
ifeq ($(CONFIG_ENABLE_I2C_MONITORING),1)
	OBJECTS += i2c.o
endif
ifeq ($(CONFIG_SYSTICK),1)
	OBJECTS += systick.o
endif
ifeq ($(CONFIG_PWM_CHARGING),1)
	OBJECTS += pwm.o
endif

include ../Makefile.config

include $(MAKER_ROOT)/Makefile.gcc

SRC_DIR = ../../src

CFLAGS += \
	-I$(SRC_DIR) \
	-I$(SRC_DIR)/rfid \
	-I$(SRC_DIR)/mspware \
	-I$(LIBEDB_ROOT)/src/include \

VPATH = $(SRC_DIR)

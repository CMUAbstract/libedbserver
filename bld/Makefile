LIB = libedbserver

SRC_ROOT = ../../src
export LIB_ROOT = $(abspath ../../ext)

export LIBEDB_ROOT = $(LIB_ROOT)/libedb

include ../Makefile.config

OBJECTS = \
	catchall.o \
	main.o \
	error.o \
	adc.o \
	mspware/pmm.o \
	params.o \
	comparator.o \
	codepoint.o \
	tether.o \
	profile.o \
	payload.o \
	sched.o \
	delay.o \


ifeq ($(or $(CONFIG_HOST_UART),$(CONFIG_TARGET_UART)),1)
	OBJECTS += uart.o
endif

ifeq ($(CONFIG_HOST_UART),1)
	OBJECTS += host_comm_impl.o
endif

ifeq ($(CONFIG_TARGET_UART),1)
	OBJECTS += target_comm_impl.o
endif

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

ifeq ($(CONFIG_CHARGE_MANIP),1)
	OBJECTS += charge.o
endif

# To build the dependee that uses the headers, we need the config macros here
include $(LIB_ROOT)/libmsp/bld/Makefile.config

override DEPS += \
	libmsp \

# libedb is not a regular dependency: we only need headers (for
# debugger<->target protocol)
override CFLAGS += -I$(LIBEDB_ROOT)/src/include

ifeq ($(CONFIG_DEV_CONSOLE),1)
override CFLAGS += -DCONFIG_DEV_CONSOLE
CONFIG_PRINTF_LIB = libmspsoftuart
include $(MAKER_ROOT)/Makefile.console
endif

override CFLAGS += \
	-I$(SRC_ROOT) \
	-I$(SRC_ROOT)/rfid \
	-I$(LIBEDB_ROOT)/src/include \

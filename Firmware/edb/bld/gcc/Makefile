EXEC = edb.out

# Expects BOARD env var to be set to one supported by 'maker' (see Maker.board)

# Configure the build here (see Makefile.config, by default all flags are unset)
CONFIG_STARTUP_VOLTAGE_WORKAROUND = disable-pmm
CONFIG_ENABLE_WATCHPOINTS = 1
CONFIG_TARGET_UART = 1
CONFIG_ENABLE_DEBUG_MODE = 1
CONFIG_PULL_DOWN_ON_SIG_LINE = 1
CONFIG_MAIN_LOOP_LED = 1
CONFIG_DEBUG_MODE_LED = 1

ifeq ($(BOARD),edb)
CONFIG_HOST_UART = 1
CONFIG_SYSTICK = 1
CONFIG_ENABLE_WATCHPOINT_STREAM = 1
CONFIG_ENABLE_VOLTAGE_STREAM = 1
CONFIG_ENABLE_TARGET_SIDE_DEBUG_MODE = 1
CONFIG_POWER_TARGET_IN_DEBUG_MODE = 1

else ifeq ($(BOARD),sprite-edb-socket-rgz)
CONFIG_ENABLE_PAYLOAD = 1
CONFIG_COLLECT_ENERGY_PROFILE = 1
CONFIG_COLLECT_APP_OUTPUT = 1
CONFIG_PERIODIC_PAYLOAD = auto
CONFIG_AUTO_ENABLED_WATCHPOINTS = 2

# TEMPORARY: for debugging
CONFIG_STATE_PINS = 1
CONFIG_EVENT_PINS = 1

endif # BOARD

CONFIG_DEV_CONSOLE = 1


OBJECTS = \
	catchall.o \
	main.o \
	error.o \
	adc.o \
	mspware/pmm.o \
	params.o \
	charge.o \
	comparator.o \
	codepoint.o \
	tether.o \
	clock.o \
	profile.o \
	payload.o \


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

include ../Makefile.config

include $(MAKER_ROOT)/Makefile.gcc

ifeq ($(CONFIG_DEV_CONSOLE),1)
CFLAGS += -DCONFIG_DEV_CONSOLE
CONFIG_PRINTF_LIB = libmspsoftuart
include $(MAKER_ROOT)/Makefile.console
endif

SRC_DIR = ../../src

CFLAGS += \
	-I$(SRC_DIR) \
	-I$(SRC_DIR)/rfid \
	-I$(SRC_DIR)/mspware \
	-I$(LIBEDB_ROOT)/src/include \

VPATH = $(SRC_DIR)

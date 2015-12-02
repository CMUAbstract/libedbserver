EXEC = edb.out

OBJECTS = \
	catchall.o \
	main.o \
	error.o \
	uart.o \
	systick.o \
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

BOARD = sprite-edb-socket-rgz

# Flags that affect which sources are included go here, not in config.h

# Enable code for decoding the RF protocol
# 		Currently, this is disabled because it causes spurious interrupts.
#
#       TODO: setup RF pins only upon cmd to monitor RF because otherwise we
#       get spurious interrupts on RX pin
#
ifeq ($(CONFIG_ENABLE_RF_PROTOCOL_MONITORING),1)
	CFLAGS += -DCONFIG_ENABLE_RF_PROTOCOL_MONITORING
	OBJECTS += rfid/rfid.o rfid/rfid_decoder.o
endif

# Enable listening and recording communication on I2C bus
# 		TODO: this is not currently implemented
ifeq ($(CONFIG_ENABLE_I2C_MONITORING),1)
	CFLAGS += -DCONFIG_ENABLE_I2C_MONITORING
	OBJECTS += i2c.o
endif

# Capacitor charge implemented by a PWM with a control loop around the duty-cycle
# 		The alternative is an "valve" method of raising a GPIO high and watching
#       the voltage level with ADC or comparator and pulling the GPIO low once
#       the target threshold is crossed.
#
ifeq ($(CONFIG_PWM_CHARGING),1)
	CFLAGS += -DCONFIG_PWM_CHARGING
	OBJECTS += pwm.o
endif

include $(MAKER_ROOT)/Makefile.gcc

SRC_DIR = ../../src

CFLAGS += \
	-I$(SRC_DIR) \
	-I$(SRC_DIR)/rfid \
	-I$(SRC_DIR)/mspware \
	-I$(LIBEDB_ROOT)/src/include \

VPATH = $(SRC_DIR)

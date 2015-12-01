include ../../../Makefile.env

CG_TOOL_BIN := $(CG_TOOL_ROOT)/bin
AS = $(CG_TOOL_BIN)/cl430
CC = $(CG_TOOL_BIN)/cl430
LD = $(CG_TOOL_BIN)/cl430
HEX = $(CG_TOOL_BIN)/hex430
FLASHER := LD_LIBRARY_PATH=$(FLASHER_ROOT) $(FLASHER_ROOT)/MSP430Flasher

SRC_ROOT = ../src
LD_SCRIPT = lnk_msp430f5340.cmd

EXEC = edb

COMMON_FLAGS = \
	-g \
	-O1 \
	--opt_for_speed=0 \
	--use_hw_mpy=F5 \
	-vmspx \
	--abi=eabi \
	--data_model=small \
	--define=__MSP430F5340__ \
	--printf_support=minimal \
	--silicon_errata=CPU21 \
	--silicon_errata=CPU22 \
	--silicon_errata=CPU23 \
	--silicon_errata=CPU40 \
	--advice:power="none" \
	--diag_warning=225 \
	--display_error_number \
	--diag_wrap=off \

CFLAGS = \
	--preproc_with_compile \
	--include_path="$(CG_TOOL_ROOT)/include" \
	--include_path="$(PLATFORM_ROOT)/include" \
	--include_path="$(LIBEDB_ROOT)/src/include" \
	--include_path="$(SRC_ROOT)" \
	--include_path="$(SRC_ROOT)/mspware" \
	--include_path="$(SRC_ROOT)/adc12" \
	--include_path="$(SRC_ROOT)/gpio" \
	--include_path="$(SRC_ROOT)/rfid" \
	--include_path="$(SRC_ROOT)/timers" \
	--include_path="$(SRC_ROOT)/uart" \
	--include_path="$(SRC_ROOT)/i2c" \

ASFLAGS = $(CFLAGS)

LDFLAGS = \
	--rom_model \
	--heap_size=160 \
	--stack_size=160 \
	--cinit_hold_wdt=on \
	--reread_libs \
	--warn_sections \
	-i"$(CG_TOOL_ROOT)/lib" \
	-i"$(CG_TOOL_ROOT)/include" \
	-i"$(PLATFORM_ROOT)/include" \
	-i"$(PLATFORM_ROOT)/lib/5xx_6xx_FRxx" \

HEXFLAGS = \
	--memwidth=8 \
	--romwidth=8

# Out-of-tree build: place build artifacts (object files) in the build dir
VPATH += $(SRC_ROOT)

# Specify the source files list explicitly avoiding wildcards, in order to make
# sure the build (esp. linker) doesn't pick up stray files
C_SRCS = \
	catchall.c \
	mspware/pmm.c \
	main.c \
	error.c \
	uart.c \
	i2c.c \
	pwm.c \
	systick.c \
	rfid/rfid.c \
	rfid/rfid_decoder.c \
	adc12.c \
	charge.c \
	comparator.c \
	codepoint.c \
	host_comm_impl.c \
	target_comm_impl.c \
	tether.c \
	clock.c \

ASM_SRCS = \

LIBS = \

OBJS = ${C_SRCS:.c=.obj} ${ASM_SRCS:.asm=.obj}

all: $(EXEC).out

-include ${C_SRCS:.c=.d} ${ASM_SRCS:.asm=.d}

%.obj: %.asm
	mkdir -p "./$(shell dirname $@)"
	$(AS) $(COMMON_FLAGS) -c $(ASFLAGS) \
		--preproc_dependency="${@:.obj=.d}" --asm_directory="$(shell dirname $@)" \
		--output_file="$@" "$<"

%.obj: %.c
	mkdir -p "./$(shell dirname $@)"
	$(CC) $(COMMON_FLAGS) -c $(CFLAGS) \
		--preproc_dependency="${@:.obj=.d}" --asm_directory="$(shell dirname $@)" \
		--output_file="$@" "$<"

$(EXEC).out: $(OBJS)
	$(LD) $(COMMON_FLAGS) -z $(LDFLAGS) --output_file="$@" $^ $(LD_SCRIPT) $(LIBS)

%.hex: %.out
	$(HEX) $(HEXFLAGS) -i -o "$@" "$<"

clean:
	rm -f $(EXEC).out $(EXEC).hex $(OBJS) ${OBJS:.obj=.d}
	echo $(OBJS) | xargs -n 1 dirname | sort | uniq | xargs -n 1 -I {} \
		bash -c "[ -d {} ] && find {} -empty -type d -mindepth 0 -maxdepth 0" | \
		xargs -r rmdir -p

flash: $(EXEC).hex
	$(FLASHER) -w $< -g -v -z [VCC=3300]

.PHONY: clean flash

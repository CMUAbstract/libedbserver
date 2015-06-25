ROOT := /usr0/home/acolin
TI_ROOT := $(ROOT)/ti
DEV_ROOT := $(ROOT)/dev
CCS_ROOT := $(TI_ROOT)/ccsv6
CG_TOOL_ROOT := $(CCS_ROOT)/tools/compiler/ti-cgt-msp430_4.4.3
CG_TOOL_BIN := $(CG_TOOL_ROOT)/bin
FLASHER_ROOT := $(TI_ROOT)/MSP430Flasher_1.3.4
PLATFORM_ROOT := $(CCS_ROOT)/ccs_base/msp430
AS = $(CG_TOOL_BIN)/cl430
CC = $(CG_TOOL_BIN)/cl430
LD = $(CG_TOOL_BIN)/cl430
HEX = $(CG_TOOL_BIN)/hex430
FLASHER := LD_LIBRARY_PATH=$(FLASHER_ROOT) $(FLASHER_ROOT)/MSP430Flasher

SRC_ROOT = ../src
LD_SCRIPT = $(SRC_ROOT)/lnk_msp430fr5969.cmd

WISP_BASE_ROOT := $(DEV_ROOT)/wisp5/CCS/wisp-base

EXEC = wisp5_test_fw

COMMON_FLAGS = \
	-g \
	-vmspx \
	--abi=eabi \
	--data_model=restricted \
	--use_hw_mpy=F5 \
	--define=__MSP430FR5969__ \
	--printf_support=minimal \
	--silicon_errata=CPU21 \
	--silicon_errata=CPU22 \
	--silicon_errata=CPU40 \
	--advice:power="none" \
	--diag_warning=225 \
	--display_error_number \
	--diag_wrap=off \


CFLAGS = \
	--preproc_with_compile \
	--include_path="$(CG_TOOL_ROOT)/include" \
	--include_path="$(PLATFORM_ROOT)/include" \
	--include_path="$(WISP_BASE_ROOT)" \
	--include_path="$(SRC_ROOT)" \

ASFLAGS = $(CFLAGS)

LDFLAGS = \
	--rom_model \
	--heap_size=160 \
	--stack_size=160 \
	--cinit_hold_wdt=on \
	--reread_libs \
	--warn_sections \
	-m=$(EXEC).map \
	-i"$(CG_TOOL_ROOT)/lib" \
	-i"$(CG_TOOL_ROOT)/include" \
	-i"$(PLATFORM_ROOT)/include" \
	-i"$(PLATFORM_ROOT)/lib/5xx_6xx_FRxx" \
	-i"$(WISP_BASE_ROOT)/make" \

HEXFLAGS = \
	--memwidth=8 \
	--romwidth=8

# Out-of-tree build: place build artifacts (object files) in the build dir
VPATH += $(SRC_ROOT)

# Specify the source files list explicitly avoiding wildcards, in order to make
# sure the build (esp. linker) doesn't pick up stray files
C_SRCS = \
	main.c \
	catchall.c \
	debug.c \

ASM_SRCS = \
	isr-link.asm \

LIBS = \
	-l"wisp-base.lib" \

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

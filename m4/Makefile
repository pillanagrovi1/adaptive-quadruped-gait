# Cortex-M4 firmware build (STM32F446RE-sized memory map, runs in QEMU).
#   make -C m4                       default config in m4/bench_cfg.h
#   make -C m4 CFG_DIR=... OUT=...   what scripts/measure_m4.py does
#   make -C m4 run                   run the default build in QEMU

CC       = arm-none-eabi-gcc
SIZE     = arm-none-eabi-size
ROOT     = ..
CFG_DIR ?= default_cfg
OUT     ?= ../build/m4/default
OPT     ?= -O2

CFLAGS   = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft $(OPT) -std=c11 \
           -ffunction-sections -fdata-sections -Wall -Wextra -g -DTR_NO_DIAG
INC      = -I$(CFG_DIR) -I. -I$(ROOT)/c/include -I$(ROOT)/c/src
SRC      = bench_main.c startup.c \
           $(ROOT)/c/src/tr_agent.c $(ROOT)/c/src/tr_tabular.c \
           $(ROOT)/c/src/tr_trace.c
LDFLAGS  = -T link.ld -nostartfiles --specs=nano.specs -Wl,--gc-sections \
           -Wl,-Map=$(OUT)/fw.map

QEMU     = qemu-system-arm -M netduinoplus2 -nographic -monitor none \
           -semihosting-config enable=on,target=native

.PHONY: all run clean

all: $(OUT)/fw.elf

$(OUT)/fw.elf: $(SRC) $(CFG_DIR)/bench_cfg.h link.ld
	mkdir -p $(OUT)
	$(CC) $(CFLAGS) $(INC) $(SRC) $(LDFLAGS) -lgcc -o $@
	$(SIZE) $@

run: $(OUT)/fw.elf
	$(QEMU) -kernel $<

clean:
	rm -rf ../build/m4

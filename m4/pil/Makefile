# Processor-in-the-loop firmware.
#   make -C m4/pil              QEMU build (USART1, no cycle counter)  -> build/pil/qemu/pil.elf
#   make -C m4/pil nucleo       STM32F446RE Nucleo build (USART2 = ST-LINK VCP, DWT cycles, HSI 16 MHz)
#   make -C m4/pil nucleo180    same at 180 MHz (PLL + over-drive)
# Flash the Nucleo: st-flash write ../../build/pil/nucleo/pil.bin 0x8000000
# (or copy pil.bin onto the NUCLEO USB drive).  NOT yet tested on hardware.

CC     = arm-none-eabi-gcc
OBJCOPY= arm-none-eabi-objcopy
SIZE   = arm-none-eabi-size
ROOT   = ../..
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -O2 -std=c11 -ffunction-sections -fdata-sections \
         -Wall -Wextra -g -DTR_NO_DIAG
INC    = -I. -I.. -I$(ROOT)/c/include -I$(ROOT)/c/src
SRC    = pil_main.c ../startup.c $(ROOT)/c/src/tr_agent.c $(ROOT)/c/src/tr_tabular.c \
         $(ROOT)/c/src/tr_trace.c
LD     = -T ../link.ld -nostartfiles --specs=nano.specs -Wl,--gc-sections -lgcc

.PHONY: all qemu nucleo nucleo180 clean
all: qemu

define build
	mkdir -p $(ROOT)/build/pil/$(1)
	$(CC) $(CFLAGS) $(2) $(INC) $(SRC) $(LD) -o $(ROOT)/build/pil/$(1)/pil.elf
	$(OBJCOPY) -O binary $(ROOT)/build/pil/$(1)/pil.elf $(ROOT)/build/pil/$(1)/pil.bin
	$(SIZE) $(ROOT)/build/pil/$(1)/pil.elf
endef

qemu:
	$(call build,qemu,-DBOARD_QEMU)
nucleo:
	$(call build,nucleo,-DBOARD_NUCLEO)
nucleo180:
	$(call build,nucleo180,-DBOARD_NUCLEO -DCLOCK_180MHZ)
clean:
	rm -rf $(ROOT)/build/pil

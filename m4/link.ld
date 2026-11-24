/*
 * Linker script sized like the STM32F446RE: 512 KB flash, 128 KB SRAM.
 * QEMU runs it on the netduinoplus2 machine (STM32F405, also Cortex-M4,
 * flash at 0x08000000, SRAM at 0x20000000), so the addresses match too.
 *
 * SRAM layout (low -> high):
 *   .data | .bss | ......free / stack painting zone...... | stack (grows down) | _estack
 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text : {
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

  _sidata = LOADADDR(.data);
  .data : {
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) : {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Fail the link if static data leaves less than 4 KB for the stack. */
  ASSERT(_ebss + 4096 <= _estack, "static RAM too large: less than 4 KB left for the stack")
}

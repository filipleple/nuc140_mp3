/* nuc140.ld — minimal GCC linker script for NUC140VE3AN */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
  SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* End of RAM for initial stack pointer */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Provide common CMSIS symbols */
PROVIDE(__stack = _estack);
PROVIDE(__Vectors = __isr_vector_start__);

SECTIONS
{
  .isr_vector :
  {
    __isr_vector_start__ = .;
    KEEP(*(.isr_vector))              /* GCC/CMSIS default */
    KEEP(*(.vectors))                 /* some BSPs use .vectors */
    KEEP(*(.vector_table))            /* fallback */
    __isr_vector_end__ = .;
  } > FLASH

  .text :
  {
    *(.text*) 
    *(.rodata*)
    *(.glue_7) *(.glue_7t)
    *(.gcc_except_table*)
    *(.eh_frame*)
    . = ALIGN(4);
  } > FLASH

  /* Initialized data */
  _sidata = LOADADDR(.data);
  .data : ALIGN(4)
  {
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > SRAM AT > FLASH

  /* Zero-initialized data */
  .bss (NOLOAD) : ALIGN(4)
  {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > SRAM

  /* Optional tiny heap/stack symbols for C runtime */
  PROVIDE(end = _ebss);
  PROVIDE(_end = _ebss);

  /* Discard unused sections */
  /DISCARD/ : { *(.note*) *(.comment*) }
}


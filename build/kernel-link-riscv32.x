/* Kernel link script for riscv32 targets.

   Derived from kernel-link.x with the Cortex-M machinery removed: RISC-V has
   no hardware vector table (the trap vector, mtvec, is programmed at
   runtime), and we do not use cortex-m-rt or an equivalent runtime crate.
   The kernel entry point is a hand-written `_start` provided by the app
   crate, placed first in .text so it sits at a known offset right after the
   image header.
*/

INCLUDE memory.x

ENTRY(_start);

SECTIONS
{
  PROVIDE(_stack_start = ORIGIN(STACK) + LENGTH(STACK));

  /* Header containing data needed by the loader. We specify
     _HUBRIS_IMAGE_HEADER_SIZE and _HUBRIS_IMAGE_HEADER_ALIGN in memory.x at
     build time, then reserve enough space for the header here. There is no
     vector table on RISC-V, so the header sits at the very start of the
     image and code follows it. */
  .header ORIGIN(FLASH) :
  {
    ASSERT(. == ALIGN(_HUBRIS_IMAGE_HEADER_ALIGN), "error: header alignment is invalid");
    HEADER = .;
    . = . + _HUBRIS_IMAGE_HEADER_SIZE;
  } > FLASH

  /* ### .text */
  .text : ALIGN(4)
  {
    __stext = .;
    *(.text.start*); /* pull the _start routine to the beginning */
    *(.text .text.*);
    . = ALIGN(4);
    __etext = .;
  } > FLASH

  /* ### .rodata */
  .rodata __etext : ALIGN(4)
  {
    __srodata = .;
    *(.rodata .rodata.*);
    /* We move this into a special section so we can ensure it is always
       included in the build */
    KEEP(*(.hubris_id));
    /* 4-byte align the end (VMA) of this section.
       This is required by LLD to ensure the LMA of the following .data
       section will have the correct alignment. */
    . = ALIGN(4);
    __erodata = .;
  } > FLASH

  /* ## Sections in RAM */
  /* ### .data */
  .data : ALIGN(4)
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    /* RISC-V small-data sections */
    *(.sdata .sdata.* .sdata2 .sdata2.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
  } > RAM AT>FLASH
  . = ALIGN(4);
  __edata = .;

  /* LMA of .data */
  __sidata = LOADADDR(.data);

  /* RISC-V global pointer: mid-range of the small-data area, so gp-relative
     addressing (+/- 2 KiB) covers as much of it as possible. */
  PROVIDE(__global_pointer$ = __sdata + 0x800);

  /*
   * Fill the remaining flash space with a known value
   */
  .fill : ALIGN(1) {
    . = (ORIGIN(FLASH) + LENGTH(FLASH));
  } > FLASH =0xffffffff

  /* ### .bss */
  .bss (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    /* RISC-V small-bss sections */
    *(.sbss .sbss.*);
    *(COMMON); /* Uninitialized C statics */
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
  } > RAM
  . = ALIGN(4);
  __ebss = .;

  /* ### .uninit */
  .uninit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __suninit = .;
    *(.uninit .uninit.*);
    . = ALIGN(4);
    __euninit = .;
  } > RAM

  /* Place the heap right after `.uninit` in RAM */
  PROVIDE(__sheap = __euninit);

  /* ## .got */
  /* Dynamic relocations are unsupported. This section is only used to detect
     relocatable code in the input files and raise an error if relocatable
     code is found */
  .got (NOLOAD) :
  {
    KEEP(*(.got .got.*));
  }

  /* ## Discarded sections */
  /DISCARD/ :
  {
    /* Unwinding info only wastes space; the kernel aborts on panic.
       (The call-frame info Humility unwinds with is emitted into
       .debug_frame instead -- non-alloc, so it rides in the ELF without
       ever being part of the image; see dist.rs.) */
    *(.eh_frame .eh_frame_hdr);
  }
}

/* Do not exceed this mark in the error messages below                                    | */
/* # Alignment checks */
ASSERT(ORIGIN(FLASH) % 4 == 0, "
ERROR(kernel-link-riscv32): the start of the FLASH region must be 4-byte aligned");

ASSERT(ORIGIN(RAM) % 4 == 0, "
ERROR(kernel-link-riscv32): the start of the RAM region must be 4-byte aligned");

ASSERT(__sdata % 4 == 0 && __edata % 4 == 0, "
BUG(kernel-link-riscv32): .data is not 4-byte aligned");

ASSERT(__sidata % 4 == 0, "
BUG(kernel-link-riscv32): the LMA of .data is not 4-byte aligned");

ASSERT(__sbss % 4 == 0 && __ebss % 4 == 0, "
BUG(kernel-link-riscv32): .bss is not 4-byte aligned");

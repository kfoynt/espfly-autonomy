ENTRY(ofProcess)
SECTIONS {
  . = 0x40000000;
  .text : { *(.literal .literal.*) *(.text .text.*) }
  . = 0x3fc80000;
  .rodata : { *(.rodata .rodata.*) *(.data .data.*) *(.bss .bss.*) }
}

/* memmap */

PROVIDE(estack = 0x20000000+32*1024);

MEMORY
{
    rom(RX)   : ORIGIN = 0x00000000, LENGTH = 32*1024
    ram(WAIL) : ORIGIN = 0x20000000, LENGTH = 32*1024
}

SECTIONS
{
    .text : { *(.text*) } > rom
    .data : { *(.data*) } > ram
}

INCLUDE msp430m0_symbols.ld

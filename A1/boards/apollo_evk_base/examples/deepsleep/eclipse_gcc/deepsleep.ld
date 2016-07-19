/******************************************************************************
 *
 * deepsleep.ld - Linker script for applications using startup_gcc.c
 *
 *****************************************************************************/
ENTRY(am_reset_isr)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512K
    SRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .stack (NOLOAD):
    {
        *(.stack*)
    } > SRAM

    .data :
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
        . = ALIGN(4);
    } > SRAM AT>FLASH

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

}



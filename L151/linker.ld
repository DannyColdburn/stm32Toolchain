ENTRY(Reset_Handler)
/* STM32 uses full descending stack*/
_estack = ORIGIN(RAM) + LENGTH(RAM);
__heap_size = 0x800;
__stack_size = 0x800;

_Min_Heap_Size = 0x1000;
_Min_Stack_Size = 0x1000;

/*If you ever wonder why it 0x0800000 then look in boot config in user manual
  When boot mode is configured to main flash memory 0x0800000 aliased to 0x0 */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 16K
}

SECTIONS{
    . = ORIGIN(FLASH);

    .isr_vector : {
        KEEP(*(.isr_vector));
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH    

    .rodata : 
    {
        . = ALIGN(4);
        *(.rodata);
        . = ALIGN(4);
    } > FLASH

    .ARM.extab  : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    _basedata = LOADADDR(.data);
    .data : 
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data);
        
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    
    . = ALIGN(4);
    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    ._user_heap_stack : {
        . = ALIGN(8);
        end = .;
        _end = .;

        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    

    /DISCARD/ : {
        *(.comment)
        *(.ARM.attributes)
    }

}

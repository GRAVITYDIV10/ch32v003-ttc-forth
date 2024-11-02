ENTRY(_start)

MEMORY
{
	ROM (rx) : ORIGIN = 0x00000000, LENGTH = 16K
	RAM (rwx): ORIGIN = 0x20000000, LENGTH = 2K
}

SECTIONS
{
	.init :
	{
		*(.init)
	} >ROM AT>ROM

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text*)
		. = ALIGN(4);
	} >ROM AT>ROM

	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >ROM AT>ROM =0xFF

	.forth_rom :
	{
		. = ALIGN(4);
		*(.etb_rom)
		. = ALIGN(4);
		*(.wtb_rom)
		. = ALIGN(4);
		*(.dict_rom)
		. = ALIGN(4);
	} >ROM AT>ROM

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>ROM

	.dlalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_lma = .);
	} >ROM AT>ROM

	.data :
	{
		. = ALIGN(4);
		*(.data)
		*(.data*)
		. = ALIGN(4);
		*(.wtb_ram)
		. = ALIGN(4);
		*(.dict_ram)
		. = ALIGN(4);
		PROVIDE(_edata = .);
	} >RAM AT>ROM =0xFF

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		*(.bss)
		. = ALIGN(4);
	} >RAM AT>ROM

	PROVIDE(SRAM_BASE = ORIGIN(RAM));
	PROVIDE(SRAM_SIZE = LENGTH(RAM));
	PROVIDE(SRAM_END  = SRAM_BASE + SRAM_SIZE);
}

#define ip s0
#define rsp s1
#define psp a5
#define psb a4
#define up a3
#define wp a0
#define xp a1
#define yp a2


	.equ STKSIZE, 18
	.equ USRSIZE, 8
	.equ TIBSIZE, 32

	.section .init
	.global _start
_start:
	.option norvc;
	j reset
	.option rvc;


	.section .text
reset:
copy_data:
	la a0, _data_lma
        la a1, _data_vma
        la a2, _edata
        bgeu a1, a2, 2f
1:
        lw t0, (a0)
        sw t0, (a1)
        addi a0, a0, 4
        addi a1, a1, 4
        bltu a1, a2, 1b
2:

pb2clk_init:
	.equ RCC_BASE, 0x40021000
	.equ RCC_AFIOEN, (1 << 0)
	.equ RCC_IOPDEN, (1 << 5)
	.equ RCC_USART1EN, (1 << 14)
	.equ RCC_APB2PCENR, 0x18
	li a0, RCC_BASE
	li a1, (RCC_AFIOEN | RCC_IOPDEN | RCC_USART1EN)
	sw a1, RCC_APB2PCENR(a0)

afio_init:
	.equ AFIO_BASE, 0x40010000
	.equ AFIO_PCFR1, 0x04
	.equ AFIO_UART_D6TX, (1 << 21) | (0 << 2)
	li a0, AFIO_BASE
	li a1, AFIO_UART_D6TX
	sw a1, AFIO_PCFR1(a0)

gpiod_init:
	.equ GPIOD_BASE, 0x40011400
	.equ GPIO_CFGLR, 0x00
	.equ GPIO_INDR, 0x08
	.equ GPIO_OUTDR, 0x0C
	.equ GPIO_CFG_MUODOUT_30M, 0xF
	.equ PD6_CFG, (GPIO_CFG_MUODOUT_30M << 24)
	.equ GPIO_CFG_FLOATIN, 0x4
	.equ PD1_CFG, (GPIO_CFG_FLOATIN << 4)
	li a0, GPIOD_BASE
	li a1, PD1_CFG | PD6_CFG
	sw a1, GPIO_CFGLR(a0)

uart1_init:
	.equ UART1_BASE, 0x40013800
	.equ UART_BAUD_115200, ((4 << 4) | (5 << 0))
	.equ UART_BRR, 0x08
	.equ UART_UE, (1 << 13)
	.equ UART_TE, (1 << 3)
	.equ UART_RE, (1 << 2)
	.equ UART_CTLR1, 0x0C
	.equ UART_HDSEL, (1 << 3)
	.equ UART_CTLR3, (0x14)
	.equ UART_DATAR, (0x04)
	.equ UART_STATR, (0x00)
	.equ UART_TC, (1 << 6)
	.equ UART_RXNE, (1 << 5)
	li a0, UART1_BASE
	li a1, UART_BAUD_115200
	sw a1, UART_BRR(a0)
	li a1, UART_UE | UART_TE | UART_RE
	sw a1, UART_CTLR1(a0)
	li a1, UART_HDSEL
	sw a1, UART_CTLR3(a0)
1:
	lw a1, UART_STATR(a0)
	andi a1, a1, UART_TC
	beqz a1, 1b
	li a1, 'B'
	sw a1, UART_DATAR(a0)


goto_forth:
	la a0, forth
	jr a0

	.set lastword, 0
	.set lastcode, 0

	.section .wtb_rom
	.word 0xFFFFFF00
wtb_rom_base:

	.section .ctb_rom
	.word 0xFFFFFF00
ctb_rom_base:

	.section .text

	.macro defword label, name, entry, attr
		.section .rodata
	name_\label:
		.ascii "\name"
	name_end_\label:
		.set nlen_\label, name_end_\label - name_\label
		.byte nlen_\label + \attr
	f_\label:
		.byte \entry
	body_\label:
		.set t_\label, lastword
		.set lastword, lastword + 1
		.section .wtb_rom
		.word f_\label
		.set wtb_rom_end, .
		.section .rodata
	.endm

	.macro defcode label
		.section .ctb_rom
		.word c_\label
		.set code_\label, lastcode
		.set lastcode, lastcode + 1
		.section .text
	c_\label:
	.endm

_next:
	lbu wp, 0(ip)
	addi ip, ip, 1
	slli wp, wp, 2
	la xp, wtb_rom_base
	add wp, xp, wp
	lw wp, 0(wp)
	li xp, 0
	lbu yp, 0(wp)
	or xp, xp, yp
	lbu yp, 1(wp)
	slli yp, yp, 8
	or xp, xp, yp
	lbu yp, 3(wp)
	slli yp, yp, 24
	or xp, xp, yp
	jr xp

	.macro next
		j _next
	.endm


/*
	defword sysrst, "sysrst", a_sysrst, 0


	.section .text

code_sysrst:
	.equ PFIC_BASE, 0xE000E000
	.equ PFIC_SCTLR, 0xD10
	.equ PFIC_SYSRST, (1 << 31)
	li a0, PFIC_BASE + PFIC_SCTLR
	li a1, PFIC_SYSRST
	sw a1, 0(a0)
1:
	j 1b

	.macro dpush reg
		sw \reg, 0(psp)
		addi psp, psp, 4
	.endm

	.macro dpop reg
		addi psp, psp, -4
		lw \reg, 0(psp)
	.endm

	defword lit8, "lit8", a_lit8, 0
	.section .text
a_lit8:
	lbu xp, 0(ip)
	addi ip, ip, 1
	dpush xp
	next

	.macro rpush reg
		sw \reg, 0(rsp)
		addi rsp, rsp, 4
	.endm

	.macro rpop reg
		addi rsp, rsp, -4
		lw \reg, 0(rsp)
	.endm

_call:
	rpush ip
	addi ip, wp, 4
	next

	defword exit, "exit", a_exit, 0
	.section .text
a_exit:	
	rpop ip
	next

	defword branch, "branch", a_branch, 0
	.section .text
a_branch:
	lb xp, 0(ip)
	add ip, ip, xp
	next

	defword 0branch, "0branch", a_0branch, 0
	.section .text
a_0branch:
	lb xp, 0(ip)
	dpop wp
	bnez wp, 1f
	add ip, ip, xp
	next
1:
	addi ip, ip, 1
	next

	defword equ, "=", a_equ, 0
	.section .text
a_equ:
	dpop wp
	dpop xp
	li yp, -1
	beq wp, xp, 1f
	li yp, 0
1:
	dpush yp
	next

_doconst8u:
	lbu xp, 4(wp)
	dpush xp
	next

	defword 0x0, "0x0", _doconst8u, 0
	.byte 0

	defword 0x1, "0x1", _doconst8u, 0
	.byte 1

	defword 0x2, "0x2", _doconst8u, 0
	.byte 2

	defword 0x3, "0x3", _doconst8u, 0
	.byte 3

	defword 0x4, "0x4", _doconst8u, 0
	.byte 4

	defword 0x5, "0x5", _doconst8u, 0
	.byte 5

	defword 0x6, "0x6", _doconst8u, 0
	.byte 6

	defword 0x7, "0x7", _doconst8u, 0
	.byte 7

	defword 0x8, "0x8", _doconst8u, 0
	.byte 8

	defword 0x9, "0x9", _doconst8u, 0
	.byte 9

	defword 0xA, "0xA", _doconst8u, 0
	.byte 0xA

	defword 0xB, "0xB", _doconst8u, 0
	.byte 0xB

	defword 0xC, "0xC", _doconst8u, 0
	.byte 0xC

	defword 0xD, "0xD", _doconst8u, 0
	.byte 0xD

	defword 0xE, "0xE", _doconst8u, 0
	.byte 0xE

	defword 0xF, "0xF", _doconst8u, 0
	.byte 0xF

	defword 0x10, "0x10", _doconst8u, 0
	.byte 0x10

	defword 0x20, "0x20", _doconst8u, 0
	.byte 0x20

	defword 0x7F, "0x7F", _doconst8u, 0
	.byte 0x7F

	defword 0x80, "0x80", _doconst8u, 0
	.byte 0x80

	defword 0xFF, "0xFF", _doconst8u, 0
	.byte 0xFF

	defword plus, "+", a_plus, 0
	.section .text
a_plus:	
	dpop wp
	dpop xp
	add wp, wp, xp
	dpush wp
	next

	defword inc, "1+", _call, 0
	.byte t_0x1
	.byte t_plus
	.byte t_exit

	defword okay, "okay", _call, 0
	.byte t_lit8
	.byte '@'
	.byte t_emit
	.byte t_branch
	.byte -1

	defword fail, "fail", _call, 0
	.byte t_lit8
	.byte '!'
	.byte t_emit
	.byte t_branch
	.byte -1

	defword failne, "failne", _call, 0
	.byte t_equ
	.byte t_0branch
	.byte 1f - .
	.byte t_exit
1:
	.byte t_fail

	defword dzchk, "dzchk", a_dzchk, 0
	.section .text
a_dzchk:
	beq psp, psb, 1f
	la ip, dzchk_fail
1:
	next

	.section .rodata
dzchk_fail:
	.byte t_fail

	.section .text
_doconst8s:
	lb xp, 4(wp)
	dpush xp
	next

	defword true, "true", _doconst8s, 0
	.byte -1

	defword false, "false", _doconst8u, 0
	.byte 0

	defword 0xFFFFFFFF, "0xFFFFFFFF", _doconst8s, 0
	.byte -1

	defword cload, "c@", a_cload, 0
	.section .text
a_cload:
	dpop wp
	lbu wp, 0(wp)
	dpush wp
	next

	defword lit16, "lit16", a_lit16, 0
	.section .text
a_lit16:
	lbu wp, 0(ip)
	lbu xp, 1(ip)
	addi ip, ip, 2
	slli xp, xp, 8
	or wp, wp, xp
	dpush wp
	next

	defword lit32, "lit32", a_lit32, 0
	.section .text
a_lit32:
	lbu wp, 0(ip)
	lbu xp, 1(ip)
	slli xp, xp, 8
	or wp, wp, xp
	lbu xp, 2(ip)
	slli xp, xp, 16
	or wp, wp, xp
	lbu xp, 3(ip)
	slli xp, xp, 24
	or wp, wp, xp
	addi ip, ip, 4
	dpush wp
	next

	defword logand, "&", a_logand, 0
	.section .text
a_logand:
	dpop wp
	dpop xp
	and wp, wp, xp
	dpush wp
	next

	.section .rodata
xdigits:
	.ascii "0123456789ABCDEF"

	defword num2hex, "num2hex", _call, 0
	.byte t_0xF
	.byte t_logand
	.byte t_lit32
	.word xdigits
	.byte t_plus
	.byte t_cload
	.byte t_exit

	defword hex4, "hex4", _call, 0
	.byte t_num2hex
	.byte t_emit
	.byte t_exit

	defword rshift, "rshift", a_rshift, 0
	.section .text
a_rshift:
	dpop wp
	dpop xp
	srl xp, xp, wp
	dpush xp
	next

	defword dup, "dup", a_dup, 0
	.section .text
a_dup:
	dpop wp
	dpush wp
	dpush wp
	next

	defword swap, "swap", a_swap, 0
	.section .text
a_swap:
	dpop wp
	dpop xp
	dpush wp
	dpush xp
	next

	defword hex8, "hex8", _call, 0
	.byte t_dup
	.byte t_0x4
	.byte t_rshift
	.byte t_hex4
	.byte t_hex4
	.byte t_exit

	defword hex16, "hex16", _call, 0
	.byte t_dup
	.byte t_0x8
	.byte t_rshift
	.byte t_hex8
	.byte t_hex8
	.byte t_exit

	defword hex32, "hex32", _call, 0
	.byte t_dup
	.byte t_0x10
	.byte t_rshift
	.byte t_hex16
	.byte t_hex16
	.byte t_exit

	defword psp_load, "psp@", a_psp_load, 0
	.section .text
a_psp_load:
	dpush psp
	next

	defword psb_load, "psb@", a_psb_load, 0
	.section .text
a_psb_load:
	dpush psb
	next

	defword 4div, "4/", _call, 0
	.byte t_0x2
	.byte t_rshift
	.byte t_exit

	defword logxor, "^", a_logxor, 0
	.section .text
a_logxor:
	dpop wp
	dpop xp
	xor xp, xp, wp
	dpush xp
	next

	defword invert, "invert", _call, 0
	.byte t_0xFFFFFFFF
	.byte t_logxor
	.byte t_exit

	defword negate, "negate", _call, 0
	.byte t_invert
	.byte t_inc
	.byte t_exit

	defword minus, "-", _call, 0
	.byte t_negate
	.byte t_plus
	.byte t_exit

	defword depth, "depth", _call, 0
	.byte t_psp_load
	.byte t_psb_load
	.byte t_minus
	.byte t_4div
	.byte t_exit


	defword aligned, "aligned", _call, 0
	.byte t_0x3
	.byte t_plus
	.byte t_0x4
	.byte t_negate
	.byte t_logand
	.byte t_exit

	defword is_aligned, "aligned?", _call, 0
	.byte t_dup
	.byte t_aligned
	.byte t_equ
	.byte t_exit

	defword tor, ">r", a_tor, 0
	.section .text
a_tor:
	dpop wp
	rpush wp
	next

	defword fromr, "r>", a_fromr, 0
	.section .text
a_fromr:
	rpop wp
	dpush wp
	next

	defword over, "over", _call, 0
	.byte t_tor
	.byte t_dup
	.byte t_fromr
	.byte t_swap
	.byte t_exit

	defword dec, "1-", _call, 0
	.byte t_0x1
	.byte t_minus
	.byte t_exit

	defword space, "space", _call, 0
	.byte t_0x20
	.byte t_emit
	.byte t_exit

	defword wcnt_rom, "wcnt-rom", _doconst32, 0
	.word wcnt_rom

	defword lshift, "lshift", a_lshift, 0
	.section .text
a_lshift:
	dpop wp
	dpop xp
	sll xp, xp, wp
	dpush xp
	next

	defword logor, "|", a_logor, 0
	.section .text
a_logor:
	dpop wp
	dpop xp
	or xp, xp, wp
	dpush xp
	next

	defword wload, "w@", _call, 0
	.byte t_dup
	.byte t_cload
	.byte t_swap
	.byte t_inc
	.byte t_cload
	.byte t_0x8
	.byte t_lshift
	.byte t_logor
	.byte t_exit

	defword load, "@", _call, 0
	.byte t_dup
	.byte t_wload
	.byte t_swap
	.byte t_0x2
	.byte t_plus
	.byte t_wload
	.byte t_0x10
	.byte t_lshift
	.byte t_logor
	.byte t_exit	

	defword drop, "drop", a_drop, 0
	.section .text
a_drop:
	dpop wp
	next

	defword 2drop, "2drop", _call, 0
	.byte t_drop
	.byte t_drop
	.byte t_exit

	defword dsdump, ".s", _call, 0
	.byte t_lit8
	.byte '<'
	.byte t_emit
	.byte t_depth
	.byte t_hex8
	.byte t_lit8
	.byte '>'
	.byte t_emit

	.byte t_depth
	.byte t_psb_load
	.byte t_swap
2:
	.byte t_dup
	.byte t_0branch
	.byte 1f - .
	.byte t_space
	.byte t_dec
	.byte t_swap
	.byte t_dup
	.byte t_load
	.byte t_hex32
	.byte t_0x4
	.byte t_plus
	.byte t_swap
	.byte t_branch
	.byte 2b - .
1:
	.byte t_2drop
	.byte t_exit

	defword mmio_load, "mmio-@", a_mmio_load, 0
	.section .text
a_mmio_load:
	dpop wp
	lw wp, 0(wp)
	dpush wp
	next

	defword mmio_store, "mmio-!", a_mmio_store, 0
	.section .text
a_mmio_store:
	dpop wp
	dpop xp
	sw xp, 0(wp)
	next

_doconst32:
	lbu xp, 4(wp)
	lbu yp, 5(wp)
	slli yp, yp, 8
	or xp, xp, yp
	lbu yp, 6(wp)
	slli yp, yp, 16
	or xp, xp, yp
	lbu yp, 7(wp)
	slli yp, yp, 24
	or xp, xp, yp
	dpush xp
	next

	defword uart1, "uart1", _doconst32, 0
	.word UART1_BASE

	defword uart_offset_statr, "uart-offset-statr", _doconst8u, 0
	.byte UART_STATR

	defword uart_stat_load, "uart-stat@", _call, 0
	.byte t_uart_offset_statr
	.byte t_plus
	.byte t_mmio_load
	.byte t_exit

	defword uart_flag_tc, "uart-flag-tc", _doconst32, 0
	.word UART_TC

	defword eqz, "0=", _call, 0
	.byte t_0x0
	.byte t_equ
	.byte t_exit

	defword nez, "0<>", _call, 0
	.byte t_eqz
	.byte t_invert
	.byte t_exit

	defword uart_tx_stat, "uart-tx?", _call, 0
	.byte t_uart_stat_load
	.byte t_uart_flag_tc
	.byte t_logand
	.byte t_nez
	.byte t_exit

	.equ USER_OFFSET_NEXT, (0 * 4)
	.equ USER_OFFSET_IP, (1 * 4)
	.equ USER_OFFSET_PSP, (2 * 4)
	.equ USER_OFFSET_PSB, (3 * 4)
	.equ USER_OFFSET_RSP, (4 * 4)


_usersave:
	sw ip, USER_OFFSET_IP(up)
	sw psp, USER_OFFSET_PSP(up)
	sw psb, USER_OFFSET_PSB(up)
	sw rsp, USER_OFFSET_RSP(up)
	ret

_userload:
	lw ip, USER_OFFSET_IP(up)
	lw psp, USER_OFFSET_PSP(up)
	lw psb, USER_OFFSET_PSB(up)
	lw rsp, USER_OFFSET_RSP(up)
	ret

	defword pause, "pause", a_pause, 0
	.section .text
a_pause:
	la wp, ycnt
	lw xp, 0(wp)
	addi xp, xp, 1
	sw xp, 0(wp)

	call _usersave
	lw up, USER_OFFSET_NEXT(up)
	call _userload
	next

	defword ycnt, "ycnt", a_ycnt, 0
	.section .text
a_ycnt:	
	la wp, ycnt
	lw xp, 0(wp)
	dpush xp
	next

	defword uart_tx_wait, "uart-txwait", _call, 0
1:
	.byte t_pause
	.byte t_dup
	.byte t_uart_tx_stat
	.byte t_0branch
	.byte 1b - .
	.byte t_drop
	.byte t_exit

	defword uart_offset_datar, "uart-offset-datar", _doconst8u, 0
	.byte UART_DATAR

	defword uart_data_store, "uart-data!", _call, 0
	.byte t_uart_offset_datar
	.byte t_plus
	.byte t_mmio_store
	.byte t_exit

	defword uart_tx, "uart-tx", _call, 0
	.byte t_dup
	.byte t_uart_tx_wait
	.byte t_uart_data_store
	.byte t_exit

	defword emit, "emit", _call, 0
	.byte t_uart1
	.byte t_uart_tx
	.byte t_exit

	defword uart_flag_rxne, "uart-flag-rxne", _doconst32, 0
	.word UART_RXNE

	defword uart_rx_stat, "uart-rx-stat", _call, 0
	.byte t_uart_stat_load
	.byte t_uart_flag_rxne
	.byte t_logand
	.byte t_nez
	.byte t_exit

	defword uart_rx_wait, "uart-rx-wait", _call, 0
1:
	.byte t_pause
	.byte t_dup
	.byte t_uart_rx_stat
	.byte t_0branch
	.byte 1b - .
	.byte t_drop
	.byte t_exit

	defword uart_data_load, "uart-data@", _call, 0
	.byte t_uart_offset_datar
	.byte t_plus
	.byte t_mmio_load
	.byte t_exit

	defword uart_rx, "uart-rx", _call, 0
	.byte t_dup
	.byte t_uart_rx_wait
	.byte t_uart_data_load
	.byte t_exit

	defword key, "key", _call, 0
	.byte t_uart1
	.byte t_uart_rx
	.byte t_exit

	defword is_del, "del?", _call, 0
	.byte t_dup
	.byte t_lit8
	.byte '\b'
	.byte t_equ
	.byte t_swap
	.byte t_0x7F
	.byte t_equ
	.byte t_logor
	.byte t_exit

	defword is_cr, "cr?", _call, 0
	.byte t_dup
	.byte t_lit8
	.byte '\n'
	.byte t_equ
	.byte t_swap
	.byte t_lit8
	.byte '\r'
	.byte t_equ
	.byte t_logor
	.byte t_exit


	defword is_space, "space?", _call, 0
	.byte t_0x20
	.byte t_equ
	.byte t_exit

	defword is_tab, "tab?", _call, 0
	.byte t_dup
	.byte t_lit8
	.byte '\t'
	.byte t_equ
	.byte t_swap
	.byte t_lit8
	.byte '\v'
	.byte t_equ
	.byte t_logor
	.byte t_exit

	defword is_delim, "delim?", _call, 0
	.byte t_dup
	.byte t_is_cr
	.byte t_swap
	.byte t_dup
	.byte t_is_space
	.byte t_swap
	.byte t_is_tab
	.byte t_logor
	.byte t_logor
	.byte t_exit

	defword tib, "tib", _doconst32, 0
	.word tib

	defword toin, ">in", _doconst32, 0
	.word toin

	defword toin_load, ">in@", _call, 0
	.byte t_toin
	.byte t_load
	.byte t_exit

	defword cstore, "c!", a_cstore, 0
	.section .text
a_cstore:
	dpop wp
	dpop xp
	sb xp, 0(wp)
	next

	defword 2dup, "2dup", _call, 0
	.byte t_over
	.byte t_over
	.byte t_exit

	defword rot, "rot", _call, 0
	.byte t_tor
	.byte t_swap
	.byte t_fromr
	.byte t_swap
	.byte t_exit

	defword 2swap, "2swap", _call, 0
	.byte t_rot
	.byte t_tor
	.byte t_rot
	.byte t_fromr
	.byte t_exit

	defword 2over, "2over", _call, 0
	.byte t_tor
	.byte t_tor
	.byte t_2dup
	.byte t_fromr
	.byte t_fromr
	.byte t_2swap
	.byte t_exit

	defword wstore, "w!", _call, 0
	.byte t_2dup
	.byte t_cstore
	.byte t_inc
	.byte t_swap
	.byte t_0x8
	.byte t_rshift
	.byte t_swap
	.byte t_cstore
	.byte t_exit

	defword store, "!", _call, 0
	.byte t_2dup
	.byte t_wstore
	.byte t_0x2
	.byte t_plus
	.byte t_swap
	.byte t_0x10
	.byte t_rshift
	.byte t_swap
	.byte t_wstore
	.byte t_exit

	defword toin_store, ">in!", _call, 0
	.byte t_toin
	.byte t_store
	.byte t_exit

	defword toin_max, ">in-max", _doconst8u, 0
	.byte TIBSIZE - 1

	defword toin_min, ">in-min", _doconst8u, 0
	.byte 0x0

	defword toin_rst, ">in-rst", _call, 0
	.byte t_toin_min
	.byte t_toin_store
	.byte t_exit

	defword lt, "<", a_lt, 0
	.section .text
a_lt:
	dpop wp
	dpop xp
	li yp, -1
	blt xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword gt, ">", _call, 0
	.byte t_swap
	.byte t_lt
	.byte t_exit

	defword ge, ">=", _call, 0
	.byte t_2dup
	.byte t_gt
	.byte t_rot
	.byte t_rot
	.byte t_equ
	.byte t_logor
	.byte t_exit

	defword le, "<=", _call, 0
	.byte t_swap
	.byte t_ge
	.byte t_exit

	defword within, "within", _call, 0
	.byte t_tor
	.byte t_over
	.byte t_le
	.byte t_fromr
	.byte t_rot
	.byte t_gt
	.byte t_logand
	.byte t_exit

	defword toin_chk, ">in-chk", _call, 0
	.byte t_toin_load
	.byte t_toin_min
	.byte t_toin_max
	.byte t_within
	.byte t_exit

	defword tib_push, "tib-push", _call, 0
	.byte t_toin_chk
	.byte t_0branch
	.byte 1f - .
	.byte t_tib
	.byte t_toin_load
	.byte t_plus
	.byte t_cstore
	.byte t_toin_load
	.byte t_inc
	.byte t_toin_store
	.byte t_exit
1:
	.byte t_toin_rst
	.byte t_drop
	.byte t_exit

	defword tib_drop, "tib-drop", _call, 0
	.byte t_toin_load
	.byte t_dec
	.byte t_toin_store
	.byte t_toin_chk
	.byte t_0branch
	.byte 1f - .
	.byte t_exit
1:
	.byte t_toin_rst
	.byte t_exit

	defword _token, "_token", _call, 0
2:
	.byte t_key
	.byte t_dup
	.byte t_is_delim
	.byte t_0branch
	.byte 1f - .
	.byte t_drop
	.byte t_exit
1:
	.byte t_dup
	.byte t_is_del
	.byte t_0branch
	.byte 1f - .
	.byte t_tib_drop
	.byte t_drop
1:
	.byte t_tib_push
	.byte t_branch
	.byte 2b - .

	defword token, "token", _call, 0
	.byte t_toin_rst
1:
	.byte t__token
	.byte t_toin_load
	.byte t_0branch
	.byte 1b - .
	.byte t_exit

	defword type, "type", _call, 0
1:
	.byte t_dup
	.byte t_0branch
	.byte 1f - .
	.byte t_dec
	.byte t_swap
	.byte t_dup
	.byte t_cload
	.byte t_emit
	.byte t_inc
	.byte t_swap
	.byte t_branch
	.byte 1b - .
1:
	.byte t_2drop
	.byte t_exit

	defword nip, "nip", _call, 0
	.byte t_swap
	.byte t_drop
	.byte t_exit

	defword min, "min", _call, 0
	.byte t_2dup
	.byte t_lt
	.byte t_0branch
	.byte 1f - .
	.byte t_drop
	.byte t_exit
1:
	.byte t_nip
	.byte t_exit

	defword ne, "<>", _call, 0
	.byte t_equ
	.byte t_invert
	.byte t_exit

	defword compare, "compare", _call, 0
	.byte t_rot
	.byte t_min

	.byte t_dup
	.byte t_0branch
	.byte 1f - .

3:
	.byte t_dup
	.byte t_0branch
	.byte 1f - .

	.byte t_rot
	.byte t_dup
	.byte t_cload
	.byte t_tor
	.byte t_inc

	.byte t_rot
	.byte t_dup
	.byte t_cload
	.byte t_tor
	.byte t_inc
	
	.byte t_rot
	.byte t_dec

	.byte t_fromr
	.byte t_fromr
	.byte t_minus
	.byte t_dup
	.byte t_0branch
	.byte 2f - .
	.byte t_nip
	.byte t_nip
	.byte t_nip
	.byte t_exit
2:
	.byte t_drop
	.byte t_branch
	.byte 3b - .

1:
	.byte t_nip
	.byte t_nip
	.byte t_exit

	defword word_offset_nlen, "word-offset-nlen", _doconst8u, 0
	.byte WORD_OFFSET_NLEN

	defword word_nlen_load, "word-nlen@", _call, 0
	.byte t_word_offset_nlen
	.byte t_plus
	.byte t_cload
	.byte t_exit

	defword word_name_load, "word-name@", _call, 0
	.byte t_dup
	.byte t_word_nlen_load
	.byte t_minus
	.byte t_exit

	defword word_name_type, "word-name-type", _call, 0
	.byte t_dup
	.byte t_word_name_load
	.byte t_swap
	.byte t_word_nlen_load
	.byte t_type
	.byte t_exit


	defword ttc_rom_base, "ttc-rom-base", _doconst32, 0
	.word ttc_rom_base

	defword 4mul, "4*", _call, 0
	.byte t_0x2
	.byte t_lshift
	.byte t_exit

	defword ttc_rom_end, "ttc-rom-end", _call, 0
	.byte t_ttc_rom_base
	.byte t_wcnt_rom
	.byte t_load
	.byte t_4mul
	.byte t_plus
	.byte t_exit

	defword cr, "cr", _call, 0
	.byte t_lit8
	.byte '\n'
	.byte t_lit8
	.byte '\r'
	.byte t_emit
	.byte t_emit
	.byte t_exit

	defword dump, "dump", _call, 0
1:
	.byte t_dup
	.byte t_0branch
	.byte 1f - .
	.byte t_dec
	.byte t_swap
	.byte t_dup
	.byte t_hex32
	.byte t_lit8
	.byte ':'
	.byte t_emit
	.byte t_space
	.byte t_dup
	.byte t_load
	.byte t_hex32
	.byte t_cr
	.byte t_0x4
	.byte t_plus
	.byte t_swap
	.byte t_branch
	.byte 1b - .
1:
	.byte t_2drop
	.byte t_exit

	defword words, "words", _call, 0
	.byte t_ttc_rom_end
	.byte t_0x4
	.byte t_minus

	.byte t_wcnt_rom
	.byte t_load

1:
	.byte t_dup
	.byte t_0branch
	.byte 1f - .
	.byte t_dec
	.byte t_swap
	.byte t_dup
	.byte t_load
	.byte t_word_name_type
	.byte t_space
	.byte t_0x4
	.byte t_minus
	.byte t_swap
	.byte t_branch
	.byte 1b - .
1:
	.byte t_2drop
	.byte t_exit

	defword find, "find", _call, 0
	.byte t_dup
	.byte t_eqz
	.byte t_0branch
	.byte 1f - .
	.byte t_2drop
	.byte t_false
	.byte t_exit
1:

	.section .rodata
boot_human:
//#define TEST
#ifdef TEST
test_find:
	.byte t_0x0
	.byte t_0x0
	.byte t_find
	.byte t_false
	.byte t_failne
	.byte t_dzchk

test_4mul:
	.byte t_0x1
	.byte t_4mul
	.byte t_0x4
	.byte t_failne

	.byte t_0x2
	.byte t_4mul
	.byte t_0x8
	.byte t_failne

	.byte t_0x0
	.byte t_4mul
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_word_name_type:
	.byte t_lit32
	.word entry_lit8
	.byte t_word_name_type

	.byte t_dzchk

test_word_name_load:
	.byte t_lit32
	.word entry_lit8
	.byte t_word_name_load
	.byte t_lit32
	.word name_lit8
	.byte t_failne
	.byte t_dzchk

test_word_nlen_load:
	.byte t_lit32
	.word entry_lit8
	.byte t_word_nlen_load
	.byte t_lit8
	.byte nlen_lit8
	.byte t_failne
	.byte t_dzchk


test_compare:
	.byte t_0x0
	.byte t_0x1
	.byte t_0x0
	.byte t_0x0
	.byte t_compare
	.byte t_0x0
	.byte t_failne

	.byte t_lit32
	.word name_lit8
	.byte t_0x3
	.byte t_lit32
	.word name_lit32
	.byte t_0x3
	.byte t_compare
	.byte t_0x0
	.byte t_failne

	.byte t_lit32
	.word name_lit8
	.byte t_0x4
	.byte t_lit32
	.word name_lit32
	.byte t_0x4
	.byte t_compare
	.byte t_0x0
	.byte t_lt
	.byte t_true
	.byte t_failne

	.byte t_lit32
	.word name_lit32
	.byte t_0x4
	.byte t_lit32
	.word name_lit8
	.byte t_0x4
	.byte t_compare
	.byte t_0x0
	.byte t_gt
	.byte t_true
	.byte t_failne

	.byte t_dzchk

test_ne:
	.byte t_0x0
	.byte t_0x0
	.byte t_ne
	.byte t_false
	.byte t_failne

	.byte t_0x0
	.byte t_0x1
	.byte t_ne
	.byte t_true
	.byte t_failne


	.byte t_dzchk

test_min:
	.byte t_0x0
	.byte t_0x1
	.byte t_min
	.byte t_0x0
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_min
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_nip:
	.byte t_0x0
	.byte t_0x1
	.byte t_nip
	.byte t_0x1
	.byte t_failne
	.byte t_dzchk

test_type:
	.byte t_lit32
	.word name_type
	.byte t_lit32
	.word nlen_type
	.byte t_type
	.byte t_dzchk

test_tib_push_drop:
	.byte t_toin_rst
	.byte t_0x20
	.byte t_tib_push
	.byte t_toin_load
	.byte t_0x1
	.byte t_failne
	.byte t_tib_drop
	.byte t_toin_load
	.byte t_0x0
	.byte t_failne
	.byte t_tib_drop
	.byte t_toin_chk
	.byte t_true
	.byte t_failne
	.byte t_toin_max
	.byte t_toin_store
	.byte t_0x80
	.byte t_tib_push
	.byte t_toin_load
	.byte t_toin_min
	.byte t_failne
	.byte t_dzchk

test_within:
	.byte t_0x0
	.byte t_0x1
	.byte t_0x2
	.byte t_within
	.byte t_false
	.byte t_failne

	.byte t_0x1
	.byte t_0x1
	.byte t_0x2
	.byte t_within
	.byte t_true
	.byte t_failne

	.byte t_0x2
	.byte t_0x1
	.byte t_0x2
	.byte t_within
	.byte t_false
	.byte t_failne

	.byte t_0x3
	.byte t_0x1
	.byte t_0x2
	.byte t_within
	.byte t_false
	.byte t_failne

	.byte t_0x3
	.byte t_0x1
	.byte t_0x4
	.byte t_within
	.byte t_true
	.byte t_failne

	.byte t_dzchk

test_le:
	.byte t_0x0
	.byte t_0x0
	.byte t_le
	.byte t_true
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_le
	.byte t_false
	.byte t_failne

	.byte t_0x0
	.byte t_0x1
	.byte t_le
	.byte t_true
	.byte t_failne
	.byte t_dzchk

test_ge:
	.byte t_0x0
	.byte t_0x0
	.byte t_ge
	.byte t_true
	.byte t_failne

	.byte t_0x0
	.byte t_0x1
	.byte t_ge
	.byte t_false
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_ge
	.byte t_true
	.byte t_failne
	.byte t_dzchk

test_gt:
	.byte t_0x0
	.byte t_0x0
	.byte t_gt
	.byte t_false
	.byte t_failne

	.byte t_0x0
	.byte t_0x1
	.byte t_gt
	.byte t_false
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_gt
	.byte t_true
	.byte t_failne
	.byte t_dzchk

test_lt:
	.byte t_0x0
	.byte t_0x0
	.byte t_lt
	.byte t_false
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_lt
	.byte t_false
	.byte t_failne

	.byte t_0x0
	.byte t_0x1
	.byte t_lt
	.byte t_true
	.byte t_failne
	.byte t_dzchk

test_toin:	
	.byte t_0x20
	.byte t_toin_store
	.byte t_toin_load
	.byte t_0x20
	.byte t_failne
	.byte t_toin_rst
	.byte t_toin_load
	.byte t_0x0
	.byte t_failne

	.byte t_toin_max
	.byte t_toin_store
	.byte t_toin_chk
	.byte t_false
	.byte t_failne

	.byte t_toin_min
	.byte t_dec
	.byte t_toin_store
	.byte t_toin_chk
	.byte t_false
	.byte t_failne

	.byte t_toin_rst
	.byte t_toin_chk
	.byte t_true
	.byte t_failne

	.byte t_dzchk

test_store:
	.byte t_lit32
	.word 0x55AA00FF
	.byte t_tib
	.byte t_store
	.byte t_tib
	.byte t_load
	.byte t_lit32
	.word 0x55AA00FF
	.byte t_failne
	.byte t_dzchk

test_wstore:
	.byte t_lit16
	.2byte 0x55AA
	.byte t_tib
	.byte t_wstore
	.byte t_tib
	.byte t_wload
	.byte t_lit16
	.2byte 0x55AA
	.byte t_failne
	.byte t_dzchk

test_2over:
	.byte t_0x1
	.byte t_0x2
	.byte t_0x3
	.byte t_0x4
	.byte t_2over
	.byte t_0x2
	.byte t_failne
	.byte t_0x1
	.byte t_failne
	.byte t_0x4
	.byte t_failne
	.byte t_0x3
	.byte t_failne
	.byte t_0x2
	.byte t_failne
	.byte t_0x1
	.byte t_failne
	.byte t_dzchk

test_rot:
	.byte t_0x1
	.byte t_0x2
	.byte t_0x3
	.byte t_rot
	.byte t_0x1
	.byte t_failne
	.byte t_0x3
	.byte t_failne
	.byte t_0x2
	.byte t_failne
	.byte t_dzchk

test_2swap:
	.byte t_0x1
	.byte t_0x2
	.byte t_0x3
	.byte t_0x4
	.byte t_2swap
	.byte t_0x2
	.byte t_failne
	.byte t_0x1
	.byte t_failne
	.byte t_0x4
	.byte t_failne
	.byte t_0x3
	.byte t_failne
	.byte t_dzchk

test_2dup:
	.byte t_0x1
	.byte t_0x2
	.byte t_2dup
	.byte t_0x2
	.byte t_failne
	.byte t_0x1
	.byte t_failne
	.byte t_0x2
	.byte t_failne
	.byte t_0x1
	.byte t_failne
	.byte t_dzchk

test_cstore:
	.byte t_0x10
	.byte t_tib
	.byte t_cstore
	.byte t_tib
	.byte t_cload
	.byte t_0x10
	.byte t_failne
	.byte t_dzchk

test_is_delim:
	.byte t_lit8
	.byte ' '
	.byte t_is_delim
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte '\n'
	.byte t_is_delim
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte '\r'
	.byte t_is_delim
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte '\t'
	.byte t_is_delim
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte '\v'
	.byte t_is_delim
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte 'A'
	.byte t_is_delim
	.byte t_false
	.byte t_failne
	.byte t_dzchk

test_is_tab:
	.byte t_lit8
	.byte '\t'
	.byte t_is_tab
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte '\v'
	.byte t_is_tab
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte 'A'
	.byte t_is_tab
	.byte t_false
	.byte t_failne
	.byte t_dzchk

test_is_space:
	.byte t_lit8
	.byte ' '
	.byte t_is_space
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte 'A'
	.byte t_is_space
	.byte t_false
	.byte t_failne
	.byte t_dzchk

test_is_cr:
	.byte t_lit8
	.byte '\r'
	.byte t_is_cr
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte '\n'
	.byte t_is_cr
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte 'A'
	.byte t_is_cr
	.byte t_false
	.byte t_failne
	.byte t_dzchk

test_is_del:
	.byte t_lit8
	.byte '\b'
	.byte t_is_del
	.byte t_true
	.byte t_failne

	.byte t_0x7F
	.byte t_is_del
	.byte t_true
	.byte t_failne

	.byte t_lit8
	.byte 'A'
	.byte t_is_del
	.byte t_false
	.byte t_failne
	.byte t_dzchk

test_uart_tx:
	.byte t_uart1
	.byte t_hex32
	.byte t_space

	.byte t_uart_offset_statr
	.byte t_hex32
	.byte t_space

	.byte t_uart_flag_tc
	.byte t_hex32
	.byte t_space

	.byte t_uart1
	.byte t_uart_stat_load
	.byte t_hex32
	.byte t_space

	.byte t_uart1
	.byte t_uart_tx_stat
	.byte t_hex32
	.byte t_space

	.byte t_uart1
	.byte t_uart_tx_wait
	.byte t_uart1
	.byte t_uart_tx_stat
	.byte t_hex32
	.byte t_space

	.byte t_lit8
	.byte 'U'
	.byte t_uart1
	.byte t_uart_tx

	.byte t_lit8
	.byte 'A'
	.byte t_uart1
	.byte t_uart_tx

	.byte t_lit8
	.byte 'R'
	.byte t_uart1
	.byte t_uart_tx

	.byte t_lit8
	.byte 'T'
	.byte t_uart1
	.byte t_uart_tx
	.byte t_dzchk

test_nez:
	.byte t_0x0
	.byte t_nez
	.byte t_false
	.byte t_failne

	.byte t_0x1
	.byte t_nez
	.byte t_true
	.byte t_failne
	.byte t_dzchk

test_eqz:	
	.byte t_0x0
	.byte t_eqz
	.byte t_true
	.byte t_failne

	.byte t_0x1
	.byte t_eqz
	.byte t_false
	.byte t_failne

	.byte t_dzchk

test_dsdump:
	.byte t_dsdump
	.byte t_dzchk

	.byte t_0x1
	.byte t_dsdump

	.byte t_0x2
	.byte t_dsdump

	.byte t_0x3
	.byte t_dsdump

	.byte t_0x4
	.byte t_dsdump

	.byte t_2drop
	.byte t_2drop
	.byte t_dzchk


test_drop_2drop:
	.byte t_0x0
	.byte t_drop
	.byte t_dzchk
	.byte t_0x0
	.byte t_0x0
	.byte t_2drop
	.byte t_dzchk

test_load:
	.byte t_lit32
1:
	.word 0x89ABCDEF
	.byte t_lit32
	.word 1b
	.byte t_load
	.byte t_failne
	.byte t_dzchk

test_wload:
	.byte t_lit16
1:
	.2byte 0x55AA
	.byte t_lit32
	.word 1b
	.byte t_wload
	.byte t_failne
	.byte t_dzchk

test_logor:
	.byte t_0x0
	.byte t_0x0
	.byte t_logor
	.byte t_0x0
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_logor
	.byte t_0x1
	.byte t_failne

	.byte t_0x1
	.byte t_0x1
	.byte t_logor
	.byte t_0x1
	.byte t_failne
	.byte t_dzchk

test_lshift:
	.byte t_0x1
	.byte t_0x1
	.byte t_lshift
	.byte t_0x2
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_lshift
	.byte t_0x1
	.byte t_failne

	.byte t_0x1
	.byte t_0x2
	.byte t_lshift
	.byte t_0x4
	.byte t_failne
	.byte t_dzchk

test_dec:
	.byte t_0x1
	.byte t_dec
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_over:
	.byte t_0x0
	.byte t_0x1
	.byte t_over
	.byte t_0x0
	.byte t_failne
	.byte t_0x1
	.byte t_failne
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_tor_fromr:
	.byte t_0x0
	.byte t_tor
	.byte t_dzchk
	.byte t_fromr
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_is_aligned:
	.byte t_0x0
	.byte t_is_aligned
	.byte t_true
	.byte t_failne

	.byte t_0x1
	.byte t_is_aligned
	.byte t_false
	.byte t_failne

	.byte t_0x4
	.byte t_is_aligned
	.byte t_true
	.byte t_failne
	.byte t_dzchk

test_aligned:
	.byte t_0x0
	.byte t_aligned
	.byte t_0x0
	.byte t_failne

	.byte t_0x1
	.byte t_aligned
	.byte t_0x4
	.byte t_failne

	.byte t_0x2
	.byte t_aligned
	.byte t_0x4
	.byte t_failne

	.byte t_0x3
	.byte t_aligned
	.byte t_0x4
	.byte t_failne

	.byte t_0x4
	.byte t_aligned
	.byte t_0x4
	.byte t_failne

	.byte t_0x5
	.byte t_aligned
	.byte t_0x8
	.byte t_failne

test_depth:
	.byte t_depth
	.byte t_0x0
	.byte t_failne

	.byte t_0x0
	.byte t_depth
	.byte t_0x1
	.byte t_failne
	.byte t_0x0
	.byte t_failne

	.byte t_0x0
	.byte t_0x0
	.byte t_depth
	.byte t_0x2
	.byte t_failne
	.byte t_failne

	.byte t_dzchk

test_minus:
	.byte t_0x5
	.byte t_0x2
	.byte t_minus
	.byte t_0x3
	.byte t_failne

	.byte t_0x1
	.byte t_0x1
	.byte t_minus
	.byte t_0x0
	.byte t_failne

test_negate:	
	.byte t_0x0
	.byte t_negate
	.byte t_0x0
	.byte t_failne

	.byte t_0x1
	.byte t_negate
	.byte t_lit32
	.word -1
	.byte t_failne

	.byte t_0x2
	.byte t_negate
	.byte t_lit32
	.word -2
	.byte t_failne

	.byte t_lit32
	.word -2
	.byte t_negate
	.byte t_0x2
	.byte t_failne
	
test_invert:	
	.byte t_0x0
	.byte t_invert
	.byte t_lit32
	.word -1
	.byte t_failne

	.byte t_0x1
	.byte t_invert
	.byte t_lit32
	.word -2
	.byte t_failne
	.byte t_dzchk

test_logxor:
	.byte t_0x1
	.byte t_0x1
	.byte t_logxor
	.byte t_0x0
	.byte t_failne

	.byte t_0x1
	.byte t_0x0
	.byte t_logxor
	.byte t_0x1
	.byte t_failne

	.byte t_0x0
	.byte t_0x0
	.byte t_logxor
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_4div:
	.byte t_0x4
	.byte t_4div
	.byte t_0x1
	.byte t_failne

	.byte t_0x8
	.byte t_4div
	.byte t_0x2
	.byte t_failne

	.byte t_0x0
	.byte t_4div
	.byte t_0x0
	.byte t_failne
	.byte t_dzchk

test_psp_psb_load:
	.byte t_psp_load
	.byte t_psb_load
	.byte t_failne

test_hex32:
	.byte t_lit32
	.word 0x01234567
	.byte t_hex32
	.byte t_lit32
	.word 0x89ABCDEF
	.byte t_hex32

test_hex16:
	.byte t_lit16
	.2byte 0x0123
	.byte t_hex16
	.byte t_lit16
	.2byte 0x4567
	.byte t_hex16

test_hex8:
	.byte t_0x20
	.byte t_hex8
	.byte t_0x7F
	.byte t_hex8
	.byte t_dzchk

test_swap:
	.byte t_0x6
	.byte t_0x7
	.byte t_swap
	.byte t_0x6
	.byte t_failne
	.byte t_0x7
	.byte t_failne
	.byte t_dzchk

test_dup:
	.byte t_0x5
	.byte t_dup
	.byte t_failne
	.byte t_dzchk

test_rshift:
	.byte t_0x2
	.byte t_0x1
	.byte t_rshift
	.byte t_0x1
	.byte t_failne

	.byte t_0x4
	.byte t_0x1
	.byte t_rshift
	.byte t_0x2
	.byte t_failne

	.byte t_dzchk

test_hex4:
	.byte t_0x0
	.byte t_hex4
	.byte t_0x9
	.byte t_hex4
	.byte t_0xA
	.byte t_hex4
	.byte t_0xF
	.byte t_hex4
	.byte t_dzchk

test_num2hex:	
	.byte t_0x0
	.byte t_num2hex
	.byte t_lit8
	.byte '0'
	.byte t_failne

	.byte t_0x9
	.byte t_num2hex
	.byte t_lit8
	.byte '9'
	.byte t_failne

	.byte t_0xA
	.byte t_num2hex
	.byte t_lit8
	.byte 'A'
	.byte t_failne

	.byte t_0xF
	.byte t_num2hex
	.byte t_lit8
	.byte 'F'
	.byte t_failne

	.byte t_0x10
	.byte t_num2hex
	.byte t_lit8
	.byte '0'
	.byte t_failne
	.byte t_dzchk

test_logand:
	.byte t_0x1
	.byte t_0x1
	.byte t_logand
	.byte t_0x1
	.byte t_failne

	.byte t_0x0
	.byte t_0x1
	.byte t_logand
	.byte t_0x0
	.byte t_failne

	.byte t_0x0
	.byte t_0x0
	.byte t_logand
	.byte t_0x0
	.byte t_failne

	.byte t_dzchk


test_cload:
	.byte t_lit32
	.word 1f
	.byte t_cload
	.byte t_lit8
1:
	.byte 0x55
	.byte t_failne
	.byte t_dzchk


test_true_false:
	.byte t_false
	.byte t_0x0
	.byte t_failne
	.byte t_true
	.byte t_0xFFFFFFFF
	.byte t_failne
	.byte t_dzchk

test_inc:
	.byte t_0x3
	.byte t_inc
	.byte t_0x4
	.byte t_failne
	.byte t_dzchk

test_plus:
	.byte t_0x3
	.byte t_0x4
	.byte t_plus
	.byte t_0x7
	.byte t_failne
	.byte t_dzchk

test_equ:
	.byte t_0x3
	.byte t_0x3
	.byte t_failne
	.byte t_dzchk

show_words:
	.byte t_cr
	.byte t_words
	.byte t_cr
	.byte t_dzchk

show_ttc_rom:
	.byte t_space
	.byte t_ttc_rom_base
	.byte t_hex32

	.byte t_space
	.byte t_ttc_rom_end
	.byte t_hex32
/*
	.byte t_ttc_rom_base
	.byte t_wcnt_rom
	.byte t_load
	.byte t_dump
	.byte t_dzchk
*/

/*
test_echo:
1:
	.byte t_key
	.byte t_emit
	.byte t_dzchk
	.byte t_branch
	.byte 1b - .
*/

/*
test_token:
1:
	.byte t_token
	.byte t_lit8
	.byte '['
	.byte t_emit
	.byte t_tib
	.byte t_toin_load
	.byte t_type
	.byte t_lit8
	.byte ']'
	.byte t_emit
	.byte t_dzchk
	.byte t_branch
	.byte 1b - .
*/
	.byte t_pause

	.byte t_space
	.byte t_ycnt
	.byte t_hex32

	.byte t_space
	.byte t_wcnt_rom
	.byte t_load
	.byte t_hex8

	.byte t_space
#endif

	.byte t_okay
	.byte t_sysrst
	.byte 0x55
	.byte 0xAA
	.section .text

forth:
	la wp, ycnt
	sw zero, 0(wp)

	la wp, wcnt_rom
	la xp, lastno
	sw xp, 0(wp)

	la up, user_human
	la wp, user_human
	sw wp, USER_OFFSET_NEXT(up)
	la ip, boot_human
	la psp, dstk_human
	mv psb, psp
	la rsp, rstk_human
	call _usersave
	next

	.section .bss
ycnt:
	.fill 1, 4, 0
wcnt_rom:
	.fill 1, 4, 0
user_human:
	.fill USRSIZE, 4, 0
rstk_human:
	.fill STKSIZE, 4, 0
dstk_human:
	.fill STKSIZE, 4, 0
user_root:
	.fill USRSIZE, 4, 0
rstk_root:
	.fill STKSIZE, 4, 0
dstk_root:
	.fill STKSIZE, 4, 0
toin:
	.fill 1, 4, 0
tib:
	.fill TIBSIZE, 4, 0
	.p2align 2, 0x0

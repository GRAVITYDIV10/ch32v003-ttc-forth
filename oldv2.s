#define ip s0
#define rsp s1
#define psp a5
#define psb a4
#define up a3
#define wp a2
#define xp a1
#define yp a0

	.equ STKSIZE, 32
	.equ USRSIZE, 8
	.equ TIBSIZE, 32

	.equ USER_OFFSET_NEXT, (0 * 4)
	.equ USER_OFFSET_IP,   (1 * 4)
	.equ USER_OFFSET_RSP,  (3 * 4)
	.equ USER_OFFSET_PSP,  (4 * 4)
	.equ USER_OFFSET_PSB,  (5 * 4)

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
	.set lastentr, 0

	.section .wtb_rom
	.word 0xFFFFFF00
wtb_rom_base:
	.section .etb_rom
	.word 0xFFFFFF00
etb_rom_base:
	.section .text

	.macro defentr label
		.section .text
	entr_\label:
		.section .etb_rom
		.word entr_\label

		.set e_\label, lastentr
		.set lastentr, lastentr + 1
		.section .text
	.endm

/*
  FEDCBA9876543210FEDCBA9876543210
0b00100000000000000011111111111111
  MMANNNNNEEEEEEEE  AAAAAAAAAAAAAA

M:	attribute
N:	name len
E:	entry id
A:	address
*/

	.equ ADDR_MASK, 0x20003FFF
	.equ NLEN_MASK, 0x1F
	.equ NLEN_SHIFT, 24
	.equ ENTR_MASK, 0xFF
	.equ ENTR_SHIFT, 16
	.macro defword label, name, entry, attr
		.section .rodata
	name_\label:
		.ascii "\name"
	name_end_\label:
		.set nlen_\label, name_end_\label - name_\label
		.set nlen_shifted_\label, nlen_\label << NLEN_SHIFT
		.set entr_shifted_\label, \entry << ENTR_SHIFT
	word_\label:

		.section .wtb_rom
		.word word_\label + entr_shifted_\label + nlen_shifted_\label + \attr

		.set w_\label,lastword
		.set lastword, lastword + 1
		.section .rodata
	.endm

	defentr panic
	li a0, UART1_BASE
1:
	lw a1, UART_STATR(a0)
	andi a1, a1, UART_TC
	beqz a1, 1b
	li a1, '!'
	sw a1, UART_DATAR(a0)
1:
	j 1b

	defword panic, "panic", e_panic, 0

	defentr halt
	li a0, UART1_BASE
1:
	lw a1, UART_STATR(a0)
	andi a1, a1, UART_TC
	beqz a1, 1b
	li a1, '@'
	sw a1, UART_DATAR(a0)
1:
	j 1b

	defword halt, "halt", e_halt, 0

	defentr next
	lbu wp, 0(ip)
	addi ip, ip, 1
	j _next

_next:
	slli wp, wp, 2
	la xp, wtb_rom_base
	add wp, wp, xp
	lw wp, 0(wp)

	li xp, (ENTR_MASK << ENTR_SHIFT)
	and  yp, wp, xp
	srli yp, yp, ENTR_SHIFT - 2
	andi yp, yp, (ENTR_MASK << 2)

	la xp, etb_rom_base
	add yp, yp, xp
	lw yp, 0(yp)
	li xp, ADDR_MASK
	and yp, yp, xp
	and wp, wp, xp

	jr yp

	defword next, "next", e_next, 0

	.macro rpush reg
		sw \reg, 0(rsp)
		addi rsp, rsp, 4
	.endm

	defentr call
	rpush ip
	mv ip, wp
	j entr_next

	defword call, "call", e_call, 0

	.macro rpop reg
		addi rsp, rsp, -4
		lw \reg, 0(rsp)
	.endm

	defentr exit
	rpop ip
	j entr_next

	defword exit, "exit", e_exit, 0

	defword noop, "noop", e_call, 0
	.byte w_exit
	
	defentr branch
	lb xp, 0(ip)
	add ip, ip, xp
	j entr_next

	defword branch, "branch", e_branch, 0

	.macro dpop reg
		addi psp, psp, -4
		lw \reg, 0(psp)
	.endm

	defentr 0branch
	lb xp, 0(ip)
	dpop wp
	bnez wp, 1f
	add ip, ip, xp
	j entr_next
1:
	addi ip, ip, 1
	j entr_next

	defword 0branch, "0branch", e_0branch, 0

	.macro dpush reg
		sw \reg, 0(psp)
		addi psp, psp, 4
	.endm

	defentr lit8u
	lbu xp, 0(ip)
	addi ip, ip, 1
	dpush xp
	j entr_next

	defword lit8u, "lit8u", e_lit8u, 0

	defentr logxor
	dpop wp
	dpop xp
	xor wp, wp, xp
	dpush wp
	j entr_next

	defword logxor, "^", e_logxor, 0

	defentr doconst8u
	lbu xp, 0(wp)
	dpush xp
	j entr_next

	defword doconst8u, "doconst8u", e_doconst8u, 0

	defword 0x0, "0x0", e_doconst8u, 0
	.byte 0x0

	defword 0x1, "0x1", e_doconst8u, 0
	.byte 0x1

	defword 0x2, "0x2", e_doconst8u, 0
	.byte 0x2

	defword 0x3, "0x3", e_doconst8u, 0
	.byte 0x3

	defword 0x4, "0x4", e_doconst8u, 0
	.byte 0x4

	defword 0x5, "0x5", e_doconst8u, 0
	.byte 0x5

	defword 0x6, "0x6", e_doconst8u, 0
	.byte 0x6

	defword 0x7, "0x7", e_doconst8u, 0
	.byte 0x7

	defword 0x8, "0x8", e_doconst8u, 0
	.byte 0x8

	defword 0x9, "0x9", e_doconst8u, 0
	.byte 0x9

	defword 0xA, "0xA", e_doconst8u, 0
	.byte 0xA

	defword 0xB, "0xB", e_doconst8u, 0
	.byte 0xB

	defword 0xC, "0xC", e_doconst8u, 0
	.byte 0xC

	defword 0xD, "0xD", e_doconst8u, 0
	.byte 0xD

	defword 0xE, "0xE", e_doconst8u, 0
	.byte 0xE

	defword 0xF, "0xF", e_doconst8u, 0
	.byte 0xF

	defword 0x10, "0x10", e_doconst8u, 0
	.byte 0x10

	defentr doconst8s
	lb xp, 0(wp)
	dpush xp
	j entr_next

	defword doconst8s, "doconst8s", e_doconst8s, 0

	defword true, "true", e_doconst8s, 0
	.byte -1

	defword false, "false", e_doconst8u, 0
	.byte 0x0

	defword invert, "invert", e_call, 0
	.byte w_true
	.byte w_logxor
	.byte w_exit

	defword equ, "=", e_call, 0
	.byte w_logxor
	.byte w_0branch
	.byte 1f - .
	.byte w_false
	.byte w_exit
1:
	.byte w_true
	.byte w_exit

	defword nepanic, "<>panic", e_call, 0
	.byte w_logxor
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
	.byte w_exit

	defword ne, "<>", e_call, 0
	.byte w_equ
	.byte w_invert
	.byte w_exit

	defentr plus
	dpop wp
	dpop xp
	add wp, wp, xp
	dpush wp
	j entr_next

	defword plus, "+", e_plus, 0

	defentr dzchk
	bne psp, psb, entr_panic
	j entr_next

	defword dzchk, "dzchk", e_dzchk, 0

	defword inc, "1+", e_call, 0
	.byte w_0x1
	.byte w_plus
	.byte w_exit

	defword negate, "negate", e_call, 0
	.byte w_invert
	.byte w_inc
	.byte w_exit

	defentr lit8s
	lb xp, 0(ip)
	addi ip, ip, 1
	dpush xp
	j entr_next

	defword lit8s, "lit8s", e_lit8s, 0

	defword minus, "-", e_call, 0
	.byte w_negate
	.byte w_plus
	.byte w_exit

	defword dec, "1-", e_call, 0
	.byte w_0x1
	.byte w_minus
	.byte w_exit

	defentr logand
	dpop wp
	dpop xp
	and wp, wp, xp
	dpush wp
	j entr_next

	defword logand, "&", e_logand, 0

	defentr logor
	dpop wp
	dpop xp
	or wp, wp, xp
	dpush wp
	j entr_next

	defword logor, "|", e_logor, 0

	defentr lshift
	dpop wp
	dpop xp
	sll xp, xp, wp
	dpush xp
	j entr_next

	defword lshift, "<<", e_lshift, 0

	defentr rshift
	dpop wp
	dpop xp
	srl xp, xp, wp
	dpush xp
	j entr_next

	defword rshift, ">>", e_rshift, 0

	defword 4div, "4/", e_call, 0
	.byte w_0x2
	.byte w_rshift
	.byte w_exit

	defword 4mul, "4*", e_call, 0
	.byte w_0x2
	.byte w_lshift
	.byte w_exit

	defentr psp_load
	dpush psp
	j entr_next

	defword psp_load, "psp@", e_psp_load, 0

	defentr psb_load
	dpush psb
	j entr_next

	defword psb_load, "psb@", e_psb_load, 0

	defword depth, "depth", e_call, 0
	.byte w_psp_load
	.byte w_psb_load
	.byte w_minus
	.byte w_4div
	.byte w_exit

	defentr doconst32
	lbu xp, 0(wp)
	lbu yp, 1(wp)
	slli yp, yp, 8
	or xp, xp, yp
	lbu yp, 2(wp)
	slli yp, yp, 16
	or xp, xp, yp
	lbu yp, 3(wp)
	slli yp, yp, 24
	or xp, xp, yp
	dpush xp
	j entr_next

	defword doconst32, "doconst32", e_doconst32, 0

	defentr lit32
	lbu xp, 0(ip)
	lbu yp, 1(ip)
	slli yp, yp, 8
	or xp, xp, yp
	lbu yp, 2(ip)
	slli yp, yp, 16
	or xp, xp, yp
	lbu yp, 3(ip)
	slli yp, yp, 24
	or xp, xp, yp
	addi ip, ip, 4
	dpush xp
	j entr_next

	defword lit32, "lit32", e_lit32, 0

	defentr cload
	dpop wp
	lbu wp, 0(wp)
	dpush wp
	j entr_next

	defword cload, "c@", e_cload, 0

	defentr drop
	dpop wp
	j entr_next

	defword drop, "drop", e_drop, 0

	defentr dup
	dpop wp
	dpush wp
	dpush wp
	j entr_next

	defword dup, "dup", e_dup, 0

	defentr swap
	dpop wp
	dpop xp
	dpush wp
	dpush xp
	j entr_next

	defword swap, "swap", e_swap, 0

	defentr tor
	dpop wp
	rpush wp
	j entr_next

	defword tor, ">r", e_tor, 0

	defentr fromr
	rpop wp
	dpush wp
	j entr_next

	defword fromr, "r>", e_fromr, 0

	defword over, "over", e_call, 0
	.byte w_tor
	.byte w_dup
	.byte w_fromr
	.byte w_swap
	.byte w_exit

	defword rot, "rot", e_call, 0
	.byte w_tor
	.byte w_swap
	.byte w_fromr
	.byte w_swap
	.byte w_exit

	defword 2drop, "2drop", e_call, 0
	.byte w_drop
	.byte w_drop
	.byte w_exit

	defword 2dup, "2dup", e_call, 0
	.byte w_over
	.byte w_over
	.byte w_exit

	defword 2swap, "2swap", e_call, 0
	.byte w_rot
	.byte w_tor
	.byte w_rot
	.byte w_fromr
	.byte w_exit

	defword 2over, "2over", e_call, 0
	.byte w_tor
	.byte w_tor
	.byte w_2dup
	.byte w_fromr
	.byte w_fromr
	.byte w_2swap
	.byte w_exit

	defword 2rot, "2rot", e_call, 0
	.byte w_tor
	.byte w_tor
	.byte w_2swap
	.byte w_fromr
	.byte w_fromr
	.byte w_2swap
	.byte w_exit


xdigits:
	.ascii "0123456789ABCDEF"

	.macro n2x
	.byte w_0xF
	.byte w_logand
	.byte w_lit32
	.word xdigits
	.byte w_plus
	.byte w_cload
	.endm

	defentr mmio_load
	dpop wp
	lw wp, 0(wp)
	dpush wp
	j entr_next

	defword mmio_load, "mmio-@", e_mmio_load, 0

	defword uart1, "uart1", e_doconst32, 0
	.word UART1_BASE

	.macro uart_stat_load
	.byte w_uart1
	.byte w_lit8u
	.byte UART_STATR
	.byte w_plus
	.byte w_mmio_load
	.endm

	defword nez, "0<>", e_call, 0
	.byte w_0x0
	.byte w_ne
	.byte w_exit

	defword uart_stat_tx, "uart-tx?", e_call, 0
	uart_stat_load
	.byte w_lit32
	.word UART_TC
	.byte w_logand
	.byte w_nez
	.byte w_exit

	.section .text
user_save:
	sw ip, USER_OFFSET_IP(up)
	sw rsp, USER_OFFSET_RSP(up)
	sw psp, USER_OFFSET_PSP(up)
	sw psb, USER_OFFSET_PSB(up)
	ret

user_load:
	lw ip, USER_OFFSET_IP(up)
	lw rsp, USER_OFFSET_RSP(up)
	lw psp, USER_OFFSET_PSP(up)
	lw psb, USER_OFFSET_PSB(up)
	ret

	defentr pause
	call user_save
	lw up, USER_OFFSET_NEXT(up)
	call user_load
	j entr_next

	defword pause, "pause", e_pause, 0

	defword uart_wait_tx, "uart-wait-tx", e_call, 0
1:
	.byte w_pause
	.byte w_uart_stat_tx
	.byte w_0branch
	.byte 1b - .
	.byte w_exit

	defentr mmio_store
	dpop wp
	dpop xp
	sw xp, 0(wp)
	j entr_next

	defword mmio_store, "mmio-!", e_mmio_store, 0

	defword uart_data_store, "uart-data!", e_call, 0
	.byte w_uart1
	.byte w_lit8u
	.byte UART_DATAR
	.byte w_plus
	.byte w_mmio_store
	.byte w_exit

	defword uart_cstore, "uart-c!", e_call, 0
	.byte w_uart_wait_tx
	.byte w_uart_data_store
	.byte w_exit

	defword uart_stat_rx, "uart-rx?", e_call, 0
	uart_stat_load
	.byte w_lit32
	.word UART_RXNE
	.byte w_logand
	.byte w_nez
	.byte w_exit

	defword uart_wait_rx, "uart-wait-rx", e_call, 0
1:
	.byte w_pause
	.byte w_uart_stat_rx
	.byte w_0branch
	.byte 1b - .
	.byte w_exit

	defword uart_data_load, "uart-data@", e_call, 0
	.byte w_uart1
	.byte w_lit8u
	.byte UART_DATAR
	.byte w_plus
	.byte w_mmio_load
	.byte w_exit

	defword uart_cload, "uart-c@", e_call, 0
	.byte w_uart_wait_rx
	.byte w_uart_data_load
	.byte w_exit

	defentr execute
	dpop wp
	j _next

	defword execute, "execute", e_execute, 0

	defword wload, "w@", e_call, 0
	.byte w_dup
	.byte w_cload
	.byte w_swap
	.byte w_inc
	.byte w_cload
	.byte w_0x8
	.byte w_lshift
	.byte w_logor
	.byte w_exit

	defword load, "@", e_call, 0
	.byte w_dup
	.byte w_wload
	.byte w_swap
	.byte w_0x2
	.byte w_plus
	.byte w_wload
	.byte w_0x10
	.byte w_lshift
	.byte w_logor
	.byte w_exit

	defword v_emit, "v-emit", e_doconst32, 0
	.word v_emit

	defword emit, "emit", e_call, 0
	.byte w_v_emit
	.byte w_cload
	.byte w_execute
	.byte w_exit

	defword v_keyava, "v-key?", e_doconst32, 0
	.word v_keyava

	defword v_key, "v-key", e_doconst32, 0
	.word v_key

	defword keyava, "key?", e_call, 0
	.byte w_v_keyava
	.byte w_cload
	.byte w_execute
	.byte w_exit

	defword key, "key", e_call, 0
	.byte w_v_key
	.byte w_cload
	.byte w_execute
	.byte w_exit

	defword type, "type", e_call, 0
1:
	.byte w_dup
	.byte w_0branch
	.byte 1f - .
	.byte w_dec
	.byte w_swap
	.byte w_dup
	.byte w_cload
	.byte w_emit
	.byte w_inc
	.byte w_swap
	.byte w_branch
	.byte 1b - .
1:
	.byte w_2drop
	.byte w_exit

	.macro hex4
	n2x
	.byte w_emit
	.endm

	.macro hex8
	.byte w_dup
	.byte w_0x4
	.byte w_rshift
	hex4
	hex4
	.endm

	.macro hex16
	.byte w_dup
	.byte w_0x8
	.byte w_rshift
	hex8
	hex8
	.endm

	defword dot, ".", e_call, 0
	.byte w_dup
	.byte w_0x10
	.byte w_rshift
	hex16
	hex16
	.byte w_exit

	.macro space
	.byte w_lit8u
	.byte ' '
	.byte w_emit
	.endm

	defword dsdump, ".s", e_call, 0
	.byte w_depth
	.byte w_dup
	.byte w_lit8u
	.byte '<'
	.byte w_emit
	.byte w_dot
	.byte w_lit8u
	.byte '>'
	.byte w_emit

	.byte w_psb_load
	.byte w_swap

1:
	.byte w_dup
	.byte w_0branch
	.byte 1f - .
	.byte w_dec
	.byte w_swap
	.byte w_dup
	.byte w_load
	space
	.byte w_dot
	.byte w_0x4
	.byte w_plus
	.byte w_swap
	.byte w_branch
	.byte 1b - .
1:
	.byte w_2drop
	.byte w_exit

	defword nip, "nip", e_call, 0
	.byte w_swap
	.byte w_drop
	.byte w_exit

	defword ltz, "0<", e_call, 0
	.byte w_lit32
	.word (1 << 31)
	.byte w_logand
	.byte w_nez
	.byte w_exit

	defword gez, "0>=", e_call, 0
	.byte w_ltz
	.byte w_invert
	.byte w_exit

	defword lt, "<", e_call, 0
	.byte w_minus
	.byte w_ltz
	.byte w_exit

	defword gt, ">", e_call, 0
	.byte w_swap
	.byte w_lt
	.byte w_exit

	defword min, "min", e_call, 0
	.byte w_2dup
	.byte w_lt
	.byte w_0branch
	.byte 1f - .
	.byte w_drop
	.byte w_exit
1:
	.byte w_nip
	.byte w_exit

	defword compare, "compare", e_call, 0
	.byte w_rot
	.byte w_min

	.byte w_dup
	.byte w_0branch
	.byte 1f - .

3:
	.byte w_dup
	.byte w_0branch
	.byte 1f - .

	.byte w_rot
	.byte w_dup
	.byte w_cload
	.byte w_tor
	.byte w_inc

	.byte w_rot
	.byte w_dup
	.byte w_cload
	.byte w_tor
	.byte w_inc
	
	.byte w_rot
	.byte w_dec

	.byte w_fromr
	.byte w_fromr
	.byte w_minus
	.byte w_dup
	.byte w_0branch
	.byte 2f - .
	.byte w_nip
	.byte w_nip
	.byte w_nip
	.byte w_exit
2:
	.byte w_drop
	.byte w_branch
	.byte 3b - .

1:
	.byte w_nip
	.byte w_nip
	.byte w_exit

	defword xt_load, "xt@", e_call, 0
	.byte w_4mul
	.byte w_lit32
	.word wtb_rom_base
	.byte w_plus
	.byte w_load
	.byte w_exit

	defword xt2body, "xt2body", e_call, 0
	.byte w_xt_load
	.byte w_lit32
	.word ADDR_MASK
	.byte w_logand
	.byte w_exit

	defword xt2nlen, "xt2nlen", e_call, 0
	.byte w_xt_load
	.byte w_lit32
	.word NLEN_SHIFT
	.byte w_rshift
	.byte w_lit32
	.word NLEN_MASK
	.byte w_logand
	.byte w_exit

	defword xt2name, "xt2name", e_call, 0
	.byte w_dup
	.byte w_xt2body
	.byte w_swap
	.byte w_xt2nlen
	.byte w_minus
	.byte w_exit

	defword rom_latest, "rom-latest", e_call, 0
	.byte w_lit32
	.word rom_latest
	.byte w_cload
	.byte w_exit

	defword eqz, "0=", e_call, 0
	.byte w_0x0
	.byte w_equ
	.byte w_exit

	defword words, "words", e_call, 0
	.byte w_rom_latest
1:
	.byte w_dec
	.byte w_dup
	.byte w_xt2name
	.byte w_over
	.byte w_xt2nlen
	.byte w_type
	space
	.byte w_dup
	.byte w_eqz
	.byte w_0branch
	.byte 1b - .
	.byte w_drop
	.byte w_exit

	// (addr u -- xt)
	defword find, "find", e_call, 0
	.byte w_rom_latest

1:
	.byte w_dup
	.byte w_eqz
	.byte w_0branch
	.byte 2f - .
	.byte w_nip
	.byte w_nip
	.byte w_exit
2:
	.byte w_dec
	.byte w_2dup
	.byte w_xt2nlen
	.byte w_equ
	.byte w_0branch
	.byte 2f - .
	.byte w_dup
	.byte w_xt2name
	.byte w_2over
	.byte w_swap
	.byte w_over
	.byte w_compare
	.byte w_eqz
	.byte w_0branch
	.byte 2f - .
	.byte w_nip
	.byte w_nip
	.byte w_exit
2:
	.byte w_branch
	.byte 1b - .

	defword tib, "tib", e_doconst32, 0
	.word tib

	defword toin, ">in", e_doconst32, 0
	.word toin

	defentr cstore
	dpop wp
	dpop xp
	sb xp, 0(wp)
	j entr_next

	defword cstore, "c!", e_cstore, 0

	defword wstore, "w!", e_call, 0
	.byte w_2dup
	.byte w_cstore
	.byte w_inc
	.byte w_swap
	.byte w_0x8
	.byte w_rshift
	.byte w_swap
	.byte w_cstore
	.byte w_exit

	defword store, "!", e_call, 0
	.byte w_2dup
	.byte w_wstore
	.byte w_0x2
	.byte w_plus
	.byte w_swap
	.byte w_0x10
	.byte w_rshift
	.byte w_swap
	.byte w_wstore
	.byte w_exit

	defword le, "<=", e_call, 0
	.byte w_2dup
	.byte w_lt
	.byte w_tor
	.byte w_equ
	.byte w_fromr
	.byte w_logor
	.byte w_exit

	defword within, "within", e_call, 0
	.byte w_tor
	.byte w_over
	.byte w_le
	.byte w_fromr
	.byte w_rot
	.byte w_gt
	.byte w_logand
	.byte w_exit

	defword toin_chk, ">in-chk", e_call, 0
	.byte w_toin
	.byte w_cload
	.byte w_0x0
	.byte w_lit8u
	.byte TIBSIZE
	.byte w_within
	.byte w_exit

	defword tib_push, "tib-push", e_call, 0
	.byte w_toin_chk
	.byte w_0branch
	.byte 1f - .
	.byte w_tib
	.byte w_toin
	.byte w_cload
	.byte w_plus
	.byte w_cstore

	.byte w_toin
	.byte w_cload
	.byte w_inc
	.byte w_toin
	.byte w_cstore

	.byte w_exit
1:
	.byte w_drop
	.byte w_0x0
	.byte w_toin
	.byte w_cstore
	.byte w_exit

	defword tib_drop, "tib-drop", e_call, 0
	.byte w_toin
	.byte w_cload
	.byte w_dec
	.byte w_toin
	.byte w_cstore
	.byte w_toin_chk
	.byte w_0branch
	.byte 1f - .
	.byte w_exit
1:
	.byte w_0x0
	.byte w_toin
	.byte w_cstore
	.byte w_exit

	.section .rodata
boot_human:
#define TEST
#ifdef TEST
show_words:
	.byte w_words
	.byte w_dzchk

test_toin:
	.byte w_0x5
	.byte w_dup
	.byte w_tib_push
	.byte w_tib
	.byte w_cload
	.byte w_nepanic
/*
	.byte w_toin
	.byte w_cload
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk
*/

	.byte w_lit8u
	.byte TIBSIZE
	.byte w_toin
	.byte w_cstore
	.byte w_toin_chk
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_toin
	.byte w_cstore
	.byte w_toin_chk
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_within:
	.byte w_0x0
	.byte w_0x1
	.byte w_0x2
	.byte w_within
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_0x1
	.byte w_0x2
	.byte w_within
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_0x2
	.byte w_within
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_le:
	.byte w_0x0
	.byte w_0x0
	.byte w_le
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_le
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_le
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_store:
	.byte w_lit32
	.word 0x89ABCDEF
	.byte w_dup
	.byte w_tib
	.byte w_store
	.byte w_tib
	.byte w_load
	.byte w_nepanic
	.byte w_dzchk

test_wstore:
	.byte w_lit32
	.word 0x55AA
	.byte w_dup
	.byte w_tib
	.byte w_wstore
	.byte w_tib
	.byte w_wload
	.byte w_nepanic
	.byte w_dzchk

test_cstore:
	.byte w_lit8u
	.byte 0x1F
	.byte w_dup
	.byte w_tib
	.byte w_cstore
	.byte w_tib
	.byte w_cload
	.byte w_nepanic
	.byte w_dzchk


	.byte w_lit8u
	.byte 0x7F
	.byte w_dup
	.byte w_tib
	.byte w_cstore
	.byte w_tib
	.byte w_cload
	.byte w_nepanic
	.byte w_dzchk


test_find:
	.byte w_lit32
	.word name_next
	.byte w_lit8u
	.byte nlen_next
	.byte w_find
	.byte w_lit8u
	.byte w_next
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.word name_next
	.byte w_lit8u
	.byte nlen_next + 1
	.byte w_find
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk



test_eqz:
	.byte w_0x0
	.byte w_eqz
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_eqz
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk


test_xt2name:
	.byte w_lit8u
	.byte w_next
	.byte w_xt2name
	.byte w_lit32
	.word name_next
	.byte w_nepanic
	.byte w_dzchk

test_xt2nlen:
	.byte w_lit8u
	.byte w_next
	.byte w_xt2nlen
	.byte w_lit8u
	.byte nlen_next
	.byte w_nepanic
	.byte w_dzchk

test_xt2body:
	.byte w_lit8u
	.byte w_next
	.byte w_xt2body
	.byte w_lit32
	.word word_next
	.byte w_nepanic
	.byte w_dzchk

test_xt_load:
	.byte w_lit8u
	.byte w_next
	.byte w_xt_load
	.byte w_dot
	space
	.byte w_dzchk

test_compare:
	.byte w_0x0
	.byte w_dup
	.byte w_2dup
	.byte w_compare
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.word name_lit8u
	.byte w_0x3
	.byte w_lit32
	.word name_lit32
	.byte w_0x3
	.byte w_compare
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.word name_lit8u
	.byte w_0x4
	.byte w_lit32
	.word name_lit32
	.byte w_0x4
	.byte w_compare
	.byte w_ltz
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.word name_lit32
	.byte w_0x4
	.byte w_lit32
	.word name_lit8u
	.byte w_0x4
	.byte w_compare
	.byte w_0x0
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_min:
	.byte w_0x0
	.byte w_0x1
	.byte w_min
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_min
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk


test_gt:
	.byte w_0x1
	.byte w_0x0
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_lit8s
	.byte -1
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_lit8s
	.byte -1
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte -1
	.byte w_lit8s
	.byte -2
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_gt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_gt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte -1
	.byte w_0x1
	.byte w_gt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_lt:
	.byte w_0x1
	.byte w_0x1
	.byte w_lt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_lt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_lt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte -1
	.byte w_0x0
	.byte w_lt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte -1
	.byte w_0x1
	.byte w_lt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_lit8s
	.byte -1
	.byte w_lt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_gez:
	.byte w_lit8s
	.byte -1
	.byte w_gez
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte 0
	.byte w_gez
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte 1
	.byte w_gez
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_ltz:
	.byte w_lit8s
	.byte -1
	.byte w_ltz
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte 0
	.byte w_ltz
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8s
	.byte 1
	.byte w_ltz
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_nip:
	.byte w_0x0
	.byte w_0x1
	.byte w_nip
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_dsdump:
	.byte w_dsdump
	.byte w_0x0
	.byte w_dsdump
	.byte w_0x1
	.byte w_dsdump
	.byte w_0x2
	.byte w_dsdump
	.byte w_0x3
	.byte w_dsdump
	.byte w_2drop
	.byte w_2drop
	.byte w_dzchk

test_dot:
	.byte w_lit32
	.word 0x01234567
	.byte w_dot
	.byte w_lit32
	.word 0x89ABCDEF
	.byte w_dot

test_type:
	.byte w_lit32
	.word name_type
	.byte w_lit8u
	.byte nlen_type
	.byte w_type
	.byte w_dzchk

test_emit:
	.byte w_lit8u
	.byte 0x23
	.byte w_emit
	.byte w_dzchk

test_load:
	.byte w_lit32
1:
	.word 0x55AA00FF
	.byte w_lit32
	.word 1b
	.byte w_load
	.byte w_nepanic
	.byte w_dzchk

test_wload:
	.byte w_lit32
1:
	.word 0x55AA
	.byte w_lit32
	.word 1b
	.byte w_wload
	.byte w_nepanic
	.byte w_dzchk

test_execute:
	.byte w_lit8u
	.byte w_true
	.byte w_execute
	.byte w_true
	.byte w_nepanic

	.byte w_lit8u
	.byte w_false
	.byte w_execute
	.byte w_false
	.byte w_nepanic

	.byte w_lit8u
	.byte w_0x3
	.byte w_execute
	.byte w_0x3
	.byte w_nepanic

	.byte w_dzchk

test_uart_tx:
	.byte w_uart_wait_tx
	.byte w_dzchk
	.byte w_lit8u
	.byte 0x55
	.byte w_uart_cstore
	.byte w_dzchk

test_pause:
	.byte w_pause
	.byte w_dzchk

test_nez:
	.byte w_0x0
	.byte w_nez
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_nez
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_2rot:
	.byte w_0x1
	.byte w_0x2
	.byte w_0x3
	.byte w_0x4
	.byte w_0x5
	.byte w_0x6
	.byte w_2rot
	.byte w_0x2
	.byte w_nepanic
	.byte w_0x1
	.byte w_nepanic
	.byte w_0x6
	.byte w_nepanic
	.byte w_0x5
	.byte w_nepanic
	.byte w_0x4
	.byte w_nepanic
	.byte w_0x3
	.byte w_nepanic
	.byte w_dzchk

test_2over:
	.byte w_0x1
	.byte w_0x2
	.byte w_0x3
	.byte w_0x4
	.byte w_2over
	.byte w_0x2
	.byte w_nepanic
	.byte w_0x1
	.byte w_nepanic
	.byte w_0x4
	.byte w_nepanic
	.byte w_0x3
	.byte w_nepanic
	.byte w_0x2
	.byte w_nepanic
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_2swap:
	.byte w_0x1
	.byte w_0x2
	.byte w_0x3
	.byte w_0x4
	.byte w_2swap
	.byte w_0x2
	.byte w_nepanic
	.byte w_0x1
	.byte w_nepanic
	.byte w_0x4
	.byte w_nepanic
	.byte w_0x3
	.byte w_nepanic
	.byte w_dzchk

test_2dup:
	.byte w_0x3
	.byte w_0x4
	.byte w_2dup
	.byte w_0x4
	.byte w_nepanic
	.byte w_0x3
	.byte w_nepanic
	.byte w_0x4
	.byte w_nepanic
	.byte w_0x3
	.byte w_nepanic
	.byte w_dzchk

test_2drop:
	.byte w_0x1
	.byte w_0x0
	.byte w_2drop
	.byte w_dzchk

test_rot:
	.byte w_0x7
	.byte w_0x8
	.byte w_0x9
	.byte w_rot
	.byte w_0x7
	.byte w_nepanic
	.byte w_0x9
	.byte w_nepanic
	.byte w_0x8
	.byte w_nepanic
	.byte w_dzchk

test_over:	
	.byte w_0x5
	.byte w_0x6
	.byte w_over
	.byte w_0x5
	.byte w_nepanic
	.byte w_0x6
	.byte w_nepanic
	.byte w_0x5
	.byte w_nepanic
	.byte w_dzchk

test_tor_fromr:
	.byte w_0x9
	.byte w_tor
	.byte w_dzchk
	.byte w_fromr
	.byte w_0x9
	.byte w_nepanic
	.byte w_dzchk

test_swap:
	.byte w_0x1
	.byte w_0x5
	.byte w_swap
	.byte w_0x1
	.byte w_nepanic
	.byte w_0x5
	.byte w_nepanic
	.byte w_dzchk

test_dup:
	.byte w_0x1
	.byte w_dup
	.byte w_0x1
	.byte w_nepanic
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_drop:
	.byte w_0x0
	.byte w_drop
	.byte w_dzchk

test_cload:
	.byte w_lit8u
1:
	.byte 0x55
	.byte w_lit32
	.word 1b
	.byte w_cload
	.byte w_nepanic
	.byte w_dzchk

test_depth:
	.byte w_depth
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_depth
	.byte w_nepanic
	.byte w_dzchk

test_psp_psb_load:
	.byte w_psp_load
	.byte w_psb_load
	.byte w_nepanic
	.byte w_dzchk

	.byte w_psb_load
	.byte w_psp_load
	.byte w_0x4
	.byte w_minus
	.byte w_nepanic
	.byte w_dzchk

test_4mul:
	.byte w_0x1
	.byte w_4mul
	.byte w_0x4
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_4mul
	.byte w_0x8
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x4
	.byte w_4mul
	.byte w_0x10
	.byte w_nepanic
	.byte w_dzchk

test_4div:
	.byte w_0x4
	.byte w_4div
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x8
	.byte w_4div
	.byte w_0x2
	.byte w_nepanic
	.byte w_dzchk

test_rshift:
	.byte w_0x1
	.byte w_0x0
	.byte w_rshift
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_0x1
	.byte w_rshift
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x4
	.byte w_0x1
	.byte w_rshift
	.byte w_0x2
	.byte w_nepanic
	.byte w_dzchk

test_lshift:
	.byte w_0x1
	.byte w_0x0
	.byte w_lshift
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_lshift
	.byte w_0x2
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x2
	.byte w_lshift
	.byte w_0x4
	.byte w_nepanic
	.byte w_dzchk

test_logor:
	.byte w_0x0
	.byte w_0x0
	.byte w_logor
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_logor
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_logor
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_logor
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_logand:
	.byte w_0x0
	.byte w_0x0
	.byte w_logand
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_logand
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_logand
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_logand
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_dec:
	.byte w_0x0
	.byte w_dec
	.byte w_lit8s
	.byte -1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_dec
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_dec
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_minus:
	.byte w_0x0
	.byte w_0x2
	.byte w_minus
	.byte w_lit8s
	.byte -2
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_0x1
	.byte w_minus
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_0x0
	.byte w_minus
	.byte w_0x2
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_0x2
	.byte w_minus
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

test_lit8s_negte:
	.byte w_lit8s
	.byte -1
	.byte w_negate
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_negate
	.byte w_lit8s
	.byte -1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x2
	.byte w_negate
	.byte w_lit8s
	.byte -2
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_negate
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

test_inc:
	.byte w_0x0
	.byte w_inc
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

test_plus:
	.byte w_0x0
	.byte w_0x1
	.byte w_plus
	.byte w_0x1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_plus
	.byte w_0x2
	.byte w_nepanic
	.byte w_dzchk

	.byte w_true
	.byte w_0x1
	.byte w_plus
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

test_ne:
	.byte w_0x0
	.byte w_0x0
	.byte w_ne
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_ne
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_nepanic:
	.byte w_0x0
	.byte w_0x0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_equ
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_equ_failne:
	.byte w_0x1
	.byte w_0x0
	.byte w_equ
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_equ
	.byte w_0branch
	.byte panic - .
	.byte w_dzchk

test_invert_doconst8s:
	.byte w_true
	.byte w_invert
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
	.byte w_dzchk

test_xor_doconst8u:
	.byte w_0x0
	.byte w_0x0
	.byte w_logxor
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x1
	.byte w_logxor
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
	.byte w_dzchk

	.byte w_0x0
	.byte w_0x1
	.byte w_logxor
	.byte w_0branch
	.byte panic - .
	.byte w_dzchk

	.byte w_0x1
	.byte w_0x0
	.byte w_logxor
	.byte w_0branch
	.byte panic - .
	.byte w_dzchk

test_lit8u_0branch:
	.byte w_lit8u
	.byte 0
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
	.byte w_lit8u
	.byte 1
	.byte w_0branch
	.byte panic - .

test_branch:
	.byte w_branch
	.byte 1f - .
	.byte w_panic
1:

test_call_exit:
	.byte w_noop
	.byte w_next

show_rom_latest:
	space
	.byte w_rom_latest
	.byte w_dot
	space
	.byte w_dzchk

/*
test_uart_echo:
1:
	.byte w_uart_cload
	.byte w_uart_cstore
	.byte w_dzchk
	.byte w_branch
	.byte 1b - .
*/
/*
test_echo:
1:
	.byte w_key
	.byte w_emit
	.byte w_dzchk
	.byte w_branch
	.byte 1b - .
*/

#endif

	.byte w_halt
panic:
	.byte w_panic


boot_root:
1:
	.byte w_pause
	.byte w_branch
	.byte 1b - .

	.section .text
forth:
	la wp, v_emit
	li xp, w_uart_cstore
	sb xp, 0(wp)

	la wp, v_keyava
	li xp, w_uart_stat_rx
	sb xp, 0(wp)

	la wp, v_key
	li xp, w_uart_cload
	sb xp, 0(wp)

	la wp, rom_latest
	la xp, lastword
	sb xp, 0(wp)

	la wp, toin
	sb zero, 0(wp)

	la up, user_root
	la wp, user_human
	sw wp, USER_OFFSET_NEXT(up)
	la ip, boot_root
	la rsp, rstk_root
	la psp, dstk_root
	mv psb, psp
	call user_save

	la up, user_human
	la wp, user_root
	sw wp, USER_OFFSET_NEXT(up)
	la ip, boot_human
	la rsp, rstk_human
	la psp, dstk_human
	mv psb, psp
	call user_save

	j entr_next

	.section .bss
user_root:
	.fill USRSIZE, 4, 0
rstk_root:
	.fill STKSIZE, 4, 0
dstk_root:
	.fill STKSIZE, 4, 0
user_human:
	.fill USRSIZE, 4, 0
rstk_human:
	.fill STKSIZE, 4, 0
dstk_human:
	.fill STKSIZE, 4, 0
v_emit:
	.fill 1, 1, 0
v_keyava:
	.fill 1, 1, 0
v_key:
	.fill 1, 1, 0
rom_latest:
	.fill 1, 1, 0
tib:
	.fill TIBSIZE, 1, 0
toin:
	.fill 1, 1, 0
	.p2align 2

#define ip s0
#define rsp s1
#define psp a5
#define psb a4
#define up tp
#define wp a0
#define xp a1
#define yp a2
#define zp a3

	.equ VERSION, 0x1

	.equ STKSIZE, 21
	.equ USRSIZE, 8
	.equ TIBMASK, 0x1F
	.equ RWORDS, 63
	.equ RAM_XTMIN, (0xFF - RWORDS)
	.equ URXMASK, 0x1F
	.equ KBUFMASK, 0x7F

	.equ USER_OFFSET_NEXT, (0 * 4)
	.equ USER_OFFSET_IP,   (1 * 4)
	.equ USER_OFFSET_RSP,  (3 * 4)
	.equ USER_OFFSET_PSP,  (4 * 4)
	.equ USER_OFFSET_PSB,  (5 * 4)

	.section .init
	.global _start
_start:
	.option norvc;
_vector:
        j reset
        .word   0
        .word   NMI_Handler                  /* NMI Handler */
        .word   HardFault_Handler            /* Hard Fault Handler */
        .word   0
        .word   0
        .word   0
        .word   0
        .word   0
        .word   0
        .word   0
        .word   0
        .word   SysTick_Handler             /* SysTick Handler */
        .word   0
        .word   SW_Handler                  /* SW Handler */
        .word   0
        /* External Interrupts */
        .word   WWDG_IRQHandler             /* Window Watchdog */
        .word   PVD_IRQHandler              /* PVD through EXTI Line detect */
        .word   FLASH_IRQHandler            /* Flash */
        .word   RCC_IRQHandler              /* RCC */
        .word   EXTI7_0_IRQHandler          /* EXTI Line 7..0 */
        .word   AWU_IRQHandler              /* AWU */
        .word   DMA1_Channel1_IRQHandler    /* DMA1 Channel 1 */
        .word   DMA1_Channel2_IRQHandler    /* DMA1 Channel 2 */
        .word   DMA1_Channel3_IRQHandler    /* DMA1 Channel 3 */
        .word   DMA1_Channel4_IRQHandler    /* DMA1 Channel 4 */
        .word   DMA1_Channel5_IRQHandler    /* DMA1 Channel 5 */
        .word   DMA1_Channel6_IRQHandler    /* DMA1 Channel 6 */
        .word   DMA1_Channel7_IRQHandler    /* DMA1 Channel 7 */
        .word   ADC1_IRQHandler             /* ADC1 */
        .word   I2C1_EV_IRQHandler          /* I2C1 Event */
        .word   I2C1_ER_IRQHandler          /* I2C1 Error */
        .word   USART1_IRQHandler           /* USART1 */
        .word   SPI1_IRQHandler                 /* SPI1 */
        .word   TIM1_BRK_IRQHandler             /* TIM1 Break */
        .word   TIM1_UP_IRQHandler          /* TIM1 Update */
        .word   TIM1_TRG_COM_IRQHandler     /* TIM1 Trigger and Commutation */
        .word   TIM1_CC_IRQHandler          /* TIM1 Capture Compare */
        .word   TIM2_IRQHandler             /* TIM2 */
_vector_end:
        .equ _vector_size, _vector_end - _vector
        .option rvc;

	.macro next
		j entr_next
	.endm

	.section .text
NMI_Handler:
HardFault_Handler:
SW_Handler:
WWDG_IRQHandler:
PVD_IRQHandler:
FLASH_IRQHandler:
RCC_IRQHandler:
EXTI7_0_IRQHandler:
AWU_IRQHandler:
DMA1_Channel1_IRQHandler:
DMA1_Channel2_IRQHandler:
DMA1_Channel3_IRQHandler:
DMA1_Channel4_IRQHandler:
DMA1_Channel5_IRQHandler:
DMA1_Channel6_IRQHandler:
DMA1_Channel7_IRQHandler:
ADC1_IRQHandler:
I2C1_EV_IRQHandler:
I2C1_ER_IRQHandler:
SPI1_IRQHandler:
TIM1_BRK_IRQHandler:
TIM1_UP_IRQHandler:
TIM1_TRG_COM_IRQHandler:
TIM1_CC_IRQHandler:
TIM2_IRQHandler:

save_regs:
	addi sp, sp, -(15 * 4)
	sw ra, 0(sp)
	sw gp, (1 * 4)(sp)
	sw tp, (2 * 4)(sp)
	sw t0, (3 * 4)(sp)
	sw t1, (4 * 4)(sp)
	sw t2, (5 * 4)(sp)
	sw s0, (6 * 4)(sp)
	sw s1, (7 * 4)(sp)
	sw a0, (8 * 4)(sp)
	sw a1, (9 * 4)(sp)
	sw a2, (10 * 4)(sp)
	sw a3, (11 * 4)(sp)
	sw a4, (12 * 4)(sp)
	sw a5, (13 * 4)(sp)

	la tp, user_irq
	call user_load
	next

restore_regs:
	lw ra, 0(sp)
	lw gp, (1 * 4)(sp)
	lw tp, (2 * 4)(sp)
	lw t0, (3 * 4)(sp)
	lw t1, (4 * 4)(sp)
	lw t2, (5 * 4)(sp)
	lw s0, (6 * 4)(sp)
	lw s1, (7 * 4)(sp)
	lw a0, (8 * 4)(sp)
	lw a1, (9 * 4)(sp)
	lw a2, (10 * 4)(sp)
	lw a3, (11 * 4)(sp)
	lw a4, (12 * 4)(sp)
	lw a5, (13 * 4)(sp)
	addi sp, sp, (15 * 4)
	mret

	.section .text
reset:
set_sp:
	la sp, _eusrstack

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

	.equ RCC_BASE, 0x40021000
	li a0, RCC_BASE
pllclk_init:
        .equ RCC_CTLR, 0x00
        .equ RCC_PLLON, (1 << 24)
        .equ RCC_PLLRDY, (1 << 25)
	lw a1, RCC_CTLR(a0)
	li a2, RCC_PLLON
	or a1, a1, a2
	sw a1, RCC_CTLR(a0)

	li a2, RCC_PLLRDY
1:
	lw a1, RCC_CTLR(a0)
	and a1, a1, a2
	beqz a1, 1b

sysclk_init:
        .equ RCC_CFGR0, 0x04
        .equ RCC_SW_PLL, (0x2 << 0)
        lw a1, RCC_CFGR0(a0)
        ori a1, a1, RCC_SW_PLL
        sw a1, RCC_CFGR0(a0)

        .equ RCC_SWS_MASK, (0x3 << 2)
        .equ RCC_SWS_PLL, (0x2 << 2)
        li a2, RCC_SWS_PLL
1:
        lw a1, RCC_CFGR0(a0)
        and a1, a1, a2
        bne a1, a2, 1b

hclk_init:
        .equ RCC_HPRE_MASK, (0xF << 4)
        lw a1, RCC_CFGR0(a0)
        li a2, RCC_HPRE_MASK # no div
        xori a2, a2, -1
        and a1, a1, a2
        sw a1, RCC_CFGR0(a0)
        // HCLK: 48Mhz

mco_init:
	.equ RCC_MCO_SYSCLK, (0x4 << 24)
	lw a1, RCC_CFGR0(a0)
	li a2, RCC_MCO_SYSCLK
	or a1, a1, a2
	sw a1, RCC_CFGR0(a0)

lsi_init:
        .equ RCC_RSTSCKR, 0x24
        .equ RCC_LSION, (1 << 0)
        .equ RCC_LSIRDY, (1 << 1)
        lw a1, RCC_RSTSCKR(a0)
        or a1, a1, RCC_LSION
        sw a1, RCC_RSTSCKR(a0)

1:
        lw a1, RCC_RSTSCKR(a0)
        andi a1, a1, RCC_LSIRDY
        beqz a1, 1b

hbclk_init:
	.equ RCC_AHBPCENR, 0x14
	.equ RCC_DMA1_EN, (1 << 0)
	.equ RCC_SRAM_EN, (1 << 2)
	li a1, (RCC_DMA1_EN | RCC_SRAM_EN)
	sw a1, RCC_AHBPCENR(a0)

pb1clk_init:
	.equ RCC_APB1PCENR, 0x1C
	.equ RCC_PWR_EN, (1 << 28)
	.equ RCC_I2C1_EN, (1 << 21)
	.equ RCC_WWDG_EN, (1 << 11)
	.equ RCC_TIM2_EN, (1 << 0)
	li a1, (RCC_PWR_EN | RCC_I2C1_EN | RCC_WWDG_EN | RCC_TIM2_EN)
	sw a1, RCC_APB1PCENR(a0)

pb2clk_init:
	.equ RCC_AFIOEN, (1 << 0)
	.equ RCC_IOPAEN, (1 << 2)
	.equ RCC_IOPCEN, (1 << 4)
	.equ RCC_IOPDEN, (1 << 5)
	.equ RCC_ADC1EN, (1 << 9)
	.equ RCC_TIM1EN, (1 << 11)
	.equ RCC_SPI1EN, (1 << 12)
	.equ RCC_USART1EN, (1 << 14)
	.equ RCC_APB2PCENR, 0x18
	li a1, (RCC_AFIOEN | RCC_IOPAEN | RCC_IOPCEN | RCC_IOPDEN | RCC_ADC1EN | RCC_TIM1EN | RCC_SPI1EN | RCC_USART1EN)
	sw a1, RCC_APB2PCENR(a0)

afio_init:
	.equ AFIO_BASE, 0x40010000
	.equ AFIO_PCFR1, 0x04
	.equ AFIO_UART_D6TX, (1 << 21) | (0 << 2)
	li a0, AFIO_BASE
	li a1, AFIO_UART_D6TX
	sw a1, AFIO_PCFR1(a0)

gpioa_init:
	.equ GPIOA_BASE, 0x40010800
	.equ GPIO_CFGLR, 0x00
	.equ GPIO_INDR, 0x08
	.equ GPIO_OUTDR, 0x0C
	.equ GPIO_CFG_AIN, 0x0
	.equ PA1_CFG, (GPIO_CFG_AIN << 4)
	la a0, GPIOA_BASE
	li a1, PA1_CFG
	sw a1, GPIO_CFGLR(a0)

gpioc_init:
	.equ GPIOC_BASE, 0x40011000
	.equ GPIO_CFG_PPOUT_2M, 0x2
	.equ GPIO_CFG_MUODOUT_30M, 0xF
	.equ GPIO_CFG_MUPPOUT_30M, 0xB
	.equ PC1_CFG, (GPIO_CFG_MUODOUT_30M << 4)
	.equ PC2_CFG, (GPIO_CFG_MUODOUT_30M << 8)
	.equ PC3_CFG, (GPIO_CFG_PPOUT_2M << 12)
	.equ PC4_CFG, (GPIO_CFG_MUPPOUT_30M << 16)
	li a0, GPIOC_BASE
	li a1, PC1_CFG | PC2_CFG | PC3_CFG | PC4_CFG
	sw a1, GPIO_CFGLR(a0)

gpiod_init:
	.equ GPIOD_BASE, 0x40011400
	.equ PD6_CFG, (GPIO_CFG_MUODOUT_30M << 24)
	.equ GPIO_CFG_FLOATIN, 0x4
	.equ PD1_CFG, (GPIO_CFG_FLOATIN << 4)
	li a0, GPIOD_BASE
	li a1, PD1_CFG | PD6_CFG
	sw a1, GPIO_CFGLR(a0)

uart_init:
	.equ UART1_BASE, 0x40013800
	.equ UART_BAUD_115200, ((26 << 4) | (0 << 0))
	.equ UART_BRR, 0x08
	.equ UART_UE, (1 << 13)
	.equ UART_RXNEIE, (1 << 5)
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
	li a1, UART_UE | UART_RXNEIE | UART_TE | UART_RE
	sw a1, UART_CTLR1(a0)
	li a1, UART_HDSEL
	sw a1, UART_CTLR3(a0)
/*
1:
	lw a1, UART_STATR(a0)
	andi a1, a1, UART_TC
	beqz a1, 1b
	li a1, 'B'
	sw a1, UART_DATAR(a0)
*/

	la a0, urxhead
	sb zero, 0(a0)
	la a0, urxnum
	sb zero, 0(a0)

i2c_init:
	.equ I2C1_BASE, 0x40005400
	.equ I2C_CTLR1_R16, 0x00
	.equ I2C_CTLR2_R16, 0x04
	.equ I2C_OADDR1_R16, 0x08
	.equ I2C_OADDR2_R16, 0x0C
	.equ I2C_DATAR_R16, 0x10
	.equ I2C_STATR1_R16, 0x14
	.equ I2C_STATR2_R16, 0x18
	.equ I2C_CKCFGR_R16, 0x1C
	.equ I2C_PE, (1 << 0)
	.equ I2C_START, (1 << 8)
	.equ I2C_STOP,  (1 << 9)
	.equ I2C_ACK_EN, (1 << 10)
	.equ I2C_ACK_ADDR_7BIT, (0x4000)

	.equ I2C_BUSY, (1 << (1 + 16))

	.equ I2C_MSL, (1 << (0 + 16))
	.equ I2C_SB, (1 << 0)

	.equ I2C_STAT_START, (I2C_BUSY | I2C_MSL | I2C_SB)

	la a0, I2C1_BASE
	lhu a1, I2C_CTLR2_R16(a0)
	ori a1, a1, 48
	sh a1, I2C_CTLR2_R16(a0)

	lhu a1, I2C_CTLR1_R16(a0)
	li a2, I2C_PE
	xori a2, a2, -1
	and a1, a1, a2
	sh a1, I2C_CTLR1_R16(a0)
	
	li a1, 240
	sh a1, I2C_CKCFGR_R16(a0)
	
	lhu a1, I2C_CTLR1_R16(a0)
	ori a1, a1, I2C_PE | I2C_ACK_EN
	sh a1, I2C_CTLR1_R16(a0)

	li a1, I2C_ACK_ADDR_7BIT
	sh a1, I2C_OADDR1_R16(a0)

stk_init:
        .equ STK_BASE, 0xE000F000
        .equ STK_CTLR, 0x00
        .equ STK_SR,   0x04
        .equ STK_CNTL, 0x08
        .equ STK_CMPLR,0x10

        .equ STK_SWIE, (1 << 31)
        .equ STK_STRE, (1 << 3)
        .equ STK_HCLK, (1 << 2)
        .equ STK_HCLKDIV8, (0 << 2)
        .equ STK_STIE, (1 << 1)
        .equ STK_STEN, (1 << 0)

	li a0, STK_BASE
	li a1, 6000 // 1ms
	sw a1, STK_CMPLR(a0)
	sw zero, STK_CNTL(a0)
	li a1, STK_STRE | STK_HCLKDIV8 | STK_STIE | STK_STEN	
        sw a1, STK_CTLR(a0)

	la a0, stkcnt
	sw zero, 0(a0)
	sw zero, 4(a0)

pfic_init:
        .equ PFIC_BASE, 0xE000E000
	.equ PFIC_IENR1,0x100
        .equ PFIC_IENR2,0x104
        li a0, PFIC_BASE
        lw a1, PFIC_IENR1(a0)
        .equ IRQ_STK, (1 << 12)
	li a2, IRQ_STK
	or a1, a1, a2
        sw a1, PFIC_IENR1(a0)
        lw a1, PFIC_IENR2(a0)
        .equ IRQ_UART1, (1 << 0)
	li a2, IRQ_UART1
	or a1, a1, a2
        sw a1, PFIC_IENR2(a0)

iwdg_init:
        .equ IWDG_BASE, 0x40003000
        .equ IWDG_CTLR_R16, 0x00
        .equ IWDG_PSCR_R16, 0x04
        .equ IWDG_RLDR_R16, 0x08
        .equ IWDG_STATR_R16, 0xC
        .equ IWDG_RVU, (1 << 1)
        .equ IWDG_PVU, (1 << 0)
        .equ IWDG_KEYFEED, 0xAAAA
        .equ IWDG_KEYUNLOCK, 0x5555
        .equ IWDG_KEYON, 0xCCCC
        li a0, IWDG_BASE
        li a1, IWDG_KEYUNLOCK
        sh a1, IWDG_CTLR_R16(a0)

        .equ IWDG_DIVMASK, 0x7
        .equ IWDG_DIV256, 0x6
1:
        lhu a1, IWDG_STATR_R16(a0)
        andi a1, a1, IWDG_PVU
        bnez a1, 1b

        li a2, IWDG_DIVMASK
        xori a2, a2, -1
        lhu a1, IWDG_PSCR_R16(a0)
        and a1, a1, a2
        li a2, IWDG_DIV256
        or a1, a1, a2
        sh a1, IWDG_PSCR_R16(a0)

1:
        lw a1, IWDG_STATR_R16(a0)
        andi a1, a1, IWDG_RVU
        bnez a1, 1b

        li a1, -1
        sh a1, IWDG_RLDR_R16(a0)

iwdg_on:
        li a1, IWDG_KEYON
        sh a1, IWDG_CTLR_R16(a0)

irq_init:
	li a0, 0x1880
	csrw mstatus, a0
	la a0, _vector
	ori a0, a0, 3
	csrw mtvec, a0

goto_forth:
	la a0, forth
	csrw mepc, a0
	mret

SysTick_Handler:
	li t0, STK_BASE
	sw zero, STK_SR(t0)
        la t0, stkcnt

        lw t1, 0(t0)
        li t2, -1
        beq t1, t2, 1f
        addi t1, t1, 1
        sw t1, 0(t0)
        mret
1:
        sw zero, 0(t0)
        lw t1, 4(t0)
        addi t1, t1, 1
        sw t1, 4(t0)

	mret

USART1_IRQHandler:
	li t0, UART1_BASE
	lw t1, UART_STATR(t0)
	andi t1, t1, UART_RXNE
	beqz t1, 1f
	lw t2, UART_DATAR(t0)

	la t0, urxhead
	lbu t0, 0(t0)
	la t1, urxnum
	lbu t1, 0(t1)
	add t0, t0, t1
	andi t0, t0, URXMASK
	la t1, urxfifo
	add t1, t1, t0
	sb t2, 0(t1)

	la t0, urxnum
	lbu t1, 0(t0)
	addi t1, t1, 1
	sb t1, 0(t0)
1:
	mret

	.set lastword, 0
	.set lastentr, 0

	.section .wtb_rom
	.asciz "WORDTAB"
	.p2align 2
wtb_rom_base:
	.section .etb_rom
	.asciz "ENTRTAB"
	.p2align 2
etb_rom_base:
	.section .text

	.macro defentr label
	.section .etb_rom
	.4byte entr_\label
	.set e_\label, lastentr
	.set lastentr, lastentr + 1
	.section .text
	entr_\label:
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
	.equ ATTR_IMMED, (1 << 31)
	.macro defword lab, nam, entr, attr
	.section .rodata
	nam_\lab:
	.ascii "\nam"
	nam_end_\lab:
	.set nlen_\lab, nam_end_\lab - nam_\lab
	.set nlen_shifted_\lab, nlen_\lab << NLEN_SHIFT
	.set entr_shifted_\lab, \entr << ENTR_SHIFT
	body_\lab:
	.section .wtb_rom
	wtb_\lab:
	.4byte body_\lab + entr_shifted_\lab + nlen_shifted_\lab + \attr
	.set w_\lab, lastword
	.set lastword, lastword + 1
	.section .rodata
	.endm

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
	li xp, RAM_XTMIN
	bge wp, xp, 1f
	la xp, wtb_rom_base
	j 2f
1:
	la xp, wtb_ram_base
	addi wp, wp, -RAM_XTMIN
2:
	slli wp, wp, 2
	add wp, wp, xp
	lw wp, 0(wp)

	li zp, (ENTR_MASK << ENTR_SHIFT)
	and  yp, wp, zp
	srli yp, yp, ENTR_SHIFT - 2
	andi yp, yp, (ENTR_MASK << 2)

	la zp, etb_rom_base
	add yp, yp, zp
	lw yp, 0(yp)
	li zp, ADDR_MASK
	and yp, yp, zp
	and wp, wp, zp

	jr yp

	defword next, "next", e_next, 0

	.macro rpush reg
	sw \reg, 0(rsp)
	addi rsp, rsp, 4
	.endm

	.macro rpop reg
	addi rsp, rsp, -4
	lw \reg, 0(rsp)
	.endm

	defentr call
	rpush ip
	mv ip, wp
	next

	defword call, "call", e_call, 0

	defentr exit
	rpop ip
	next

	defword exit, "exit", e_exit, 0

	.macro dpush reg
	sw \reg, 0(psp)
	addi psp, psp, 4
	.endm

	.macro dpop reg
	addi psp, psp, -4
	lw \reg, 0(psp)
	.endm

	defentr lit8
	lbu xp, 0(ip)
	addi ip, ip, 1
	dpush xp
	next

	defword lit8, "lit8", e_lit8, 0

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
	next

	defword lit32, "lit32", e_lit32, 0

	defentr 0branch
	lb xp, 0(ip)
	dpop wp
	bnez wp, 1f
	add ip, ip, xp
	next
1:
	addi ip, ip, 1
	next

	defword 0branch, "0branch", e_0branch, 0

	defentr equ
	dpop wp
	dpop xp
	li yp, -1
	beq wp, xp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword equ, "=", e_equ, 0

	defword nepanic, "<>", e_call, 0
	.byte w_equ
	.byte w_0branch
	.byte 1f - .
	.byte w_exit
1:
	.byte w_panic

	defentr plus
	dpop wp
	dpop xp
	add wp, wp, xp
	dpush wp
	next

	defword plus, "+", e_plus, 0

	defentr logand
	dpop wp
	dpop xp
	and wp, wp, xp
	dpush wp
	next

	defword logand, "&", e_logand, 0

	defentr logor
	dpop wp
	dpop xp
	or wp, wp, xp
	dpush wp
	next

	defword logor, "|", e_logor, 0

	defentr logxor
	dpop wp
	dpop xp
	xor wp, wp, xp
	dpush wp
	next

	defword logxor, "^", e_logxor, 0

	defentr lshift
	dpop wp
	dpop xp
	sll xp, xp, wp
	dpush xp
	next

	defword lshift, "<<", e_lshift, 0

	defentr rshift
	dpop wp
	dpop xp
	srl xp, xp, wp
	dpush xp
	next

	defword rshift, ">>", e_rshift, 0

	defentr lt
	dpop wp
	dpop xp
	li yp, -1
	blt xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword lt, "<", e_lt, 0

	defentr gt
	dpop wp
	dpop xp
	li yp, -1
	bgt xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword gt, ">", e_gt, 0

	defword true, "true", e_call, 0
	.byte w_lit32
	.4byte -1
	.byte w_exit

	defword false, "false", e_call, 0
	.byte w_lit8
	.byte 0
	.byte w_exit

	defentr psp_load
	dpush psp
	next

	defword psp_load, "psp@", e_psp_load, 0

	defentr psb_load
	dpush psb
	next
	
	defword psb_load, "psb@", e_psb_load, 0

	defword dzchk, "dzchk", e_call, 0
	.byte w_psp_load
	.byte w_psb_load
	.byte w_nepanic
	.byte w_exit

	defword invert, "invert", e_call, 0
	.byte w_true
	.byte w_logxor
	.byte w_exit

	defword negate, "negate", e_call, 0
	.byte w_invert
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_exit

	defword minus, "-", e_call, 0
	.byte w_negate
	.byte w_plus
	.byte w_exit

	defword depth, "depth", e_call, 0
	.byte w_psp_load
	.byte w_psb_load
	.byte w_minus
	.byte w_lit8
	.byte 2
	.byte w_rshift
	.byte w_exit

	defentr load
	dpop wp
	lw wp, 0(wp)
	dpush wp
	next

	defword load, "@", e_load, 0

	defentr store
	dpop wp
	dpop xp
	sw xp, 0(wp)
	next

	defword store, "!", e_store, 0

	defword dp, "dp", e_call, 0
	.byte w_lit32
	.4byte dp
	.byte w_exit

	defentr wload
	dpop wp
	lhu wp, 0(wp)
	dpush wp
	next

	defword wload, "w@", e_wload, 0

	defentr wstore
	dpop wp
	dpop xp
	sh xp, 0(wp)
	next

	defword wstore, "w!", e_wstore, 0

	defentr cload
	dpop wp
	lbu wp, 0(wp)
	dpush wp
	next

	defword cload, "c@", e_cload, 0

	defentr cstore
	dpop wp
	dpop xp
	sb xp, 0(wp)
	next

	defword cstore, "c!", e_cstore, 0

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
	next

	defword pause, "pause", e_pause, 0

	defword uart_txstat, "uart-tx?", e_call, 0
	.byte w_lit32
	.4byte UART1_BASE + UART_STATR
	.byte w_load
	.byte w_lit32
	.4byte UART_TC
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_invert
	.byte w_exit

	defword uart_txwait, "uart-txwait", e_call, 0
1:
	.byte w_pause
	.byte w_uart_txstat
	.byte w_0branch
	.byte 1b - .
	.byte w_exit

	defword uart_cstore, "uart-c!", e_call, 0
	.byte w_uart_txwait
	.byte w_lit32
	.4byte UART1_BASE + UART_DATAR
	.byte w_store
	.byte w_exit

	defword uart_rxstat, "uart-rx?", e_call, 0
	.byte w_lit32
	.4byte urxnum
	.byte w_cload
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_invert
	.byte w_exit

/*
	.byte w_lit32
	.4byte UART1_BASE + UART_STATR
	.byte w_load
	.byte w_lit32
	.4byte UART_RXNE
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_invert
	.byte w_exit
*/

	defword uart_rxwait, "uart-rxwait", e_call, 0
1:
	.byte w_pause
	.byte w_uart_rxstat
	.byte w_0branch
	.byte 1b - .
	.byte w_exit


	defentr irqoff
	li wp, 0x88
	csrc mstatus, wp
	ret

	defword irqoff, "irqoff", e_irqoff, 0

	defentr irqon
	li wp, 0x88
	csrs mstatus, wp
	ret

	defword irqon, "irqon", e_irqon, 0

	defword uart_cload, "uart-c@", e_call, 0
	.byte w_uart_rxwait
	.byte w_irqoff
	.byte w_lit32
	.4byte urxhead
	.byte w_cload
	.byte w_lit8
	.byte 0x1F
	.byte w_logand
	.byte w_lit32
	.4byte urxfifo
	.byte w_plus
	.byte w_cload

	.byte w_lit32
	.4byte urxhead
	.byte w_cload
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_lit8
	.byte 0x1F
	.byte w_logand
	.byte w_lit32
	.4byte urxhead
	.byte w_cstore

	.byte w_lit32
	.4byte urxnum
	.byte w_cload
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_lit8
	.byte 0x1F
	.byte w_logand
	.byte w_lit32
	.4byte urxnum
	.byte w_cstore

	.byte w_irqon
	.byte w_exit

/*

	.byte w_lit32
	.4byte UART1_BASE + UART_DATAR
	.byte w_load
	.byte w_exit
*/

	defword xt_emit, "xt-emit", e_call, 0
	.byte w_lit32
	.4byte xt_emit
	.byte w_exit

	defword xt_keyava, "xt-key?", e_call, 0
	.byte w_lit32
	.4byte xt_keyava
	.byte w_exit

	defword xt_key, "xt-key", e_call, 0
	.byte w_lit32
	.4byte xt_key
	.byte w_exit

	defentr execute
	dpop wp
	j _next

	defword execute, "execute", e_execute, 0

	defword emit, "emit", e_call, 0
	.byte w_xt_emit
	.byte w_load
	.byte w_execute
	.byte w_exit

	defword keyava, "key?", e_call, 0
	.byte w_xt_keyava
	.byte w_load
	.byte w_execute
	.byte w_exit

	defword key, "key", e_call, 0
	.byte w_xt_key
	.byte w_load
	.byte w_execute
	.byte w_exit

	defentr drop
	dpop wp
	next

	defword drop, "drop", e_drop, 0

	defentr dup
	dpop wp
	dpush wp
	dpush wp
	next

	defword dup, "dup", e_dup, 0

	defentr swap
	dpop wp
	dpop xp
	dpush wp
	dpush xp
	next

	defword swap, "swap", e_swap, 0

	defentr tor
	dpop wp
	rpush wp
	next

	defword tor, ">r", e_tor, 0

	defentr fromr
	rpop wp
	dpush wp
	next

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

xdigits:
	.ascii "0123456789ABCDEF"

	defword dot, ".", e_call, 0
	.byte w_lit8
	.byte 32
1:
	.byte w_lit8
	.byte 4
	.byte w_minus

	.byte w_over
	.byte w_over
	.byte w_rshift
	.byte w_lit8
	.byte 0xF
	.byte w_logand
	.byte w_lit32
	.4byte xdigits
	.byte w_plus
	.byte w_cload
	.byte w_emit
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 1b - .
	.byte w_drop
	.byte w_drop
	.byte w_exit

	defword tib, "tib", e_call, 0
	.byte w_lit32
	.4byte tib
	.byte w_exit

	defword toin, ">in", e_call, 0
	.byte w_lit32
	.4byte toin
	.byte w_exit

	.macro tib_rst
	.byte w_lit8
	.byte 0
	.byte w_toin
	.byte w_cstore
	.endm

	.macro tib_chk
	.byte w_toin
	.byte w_cload
	.byte w_lit8
	.byte ~(TIBMASK)
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_equ
	.endm

	defword tib_push, "tib-push", e_call, 0
	tib_chk
	.byte w_0branch
	.byte 1f - .
	.byte w_tib
	.byte w_toin
	.byte w_cload
	.byte w_plus
	.byte w_cstore

	.byte w_toin
	.byte w_cload
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_toin
	.byte w_cstore

	.byte w_exit
1:
	tib_rst
	.byte w_drop
	.byte w_exit

	defword tib_drop, "tib-drop", e_call, 0
	.byte w_toin
	.byte w_cload
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_toin
	.byte w_cstore
	tib_chk
	.byte w_0branch
	.byte 1f - .
	.byte w_exit
1:
	tib_rst
	.byte w_exit

	defword type, "type", e_call, 0
	.byte w_dup
	.byte w_0branch
	.byte 1f - .
1:
	.byte w_swap
	.byte w_dup
	.byte w_cload
	.byte w_emit
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_swap
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 1b - .
1:
	.byte w_drop
	.byte w_drop
	.byte w_exit

	defword is_delim, "delim?", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte ' '
	.byte w_equ

	.byte w_swap
	.byte w_dup
	.byte w_lit8
	.byte '\t'
	.byte w_equ

	.byte w_swap
	.byte w_dup
	.byte w_lit8
	.byte '\n'
	.byte w_equ

	.byte w_swap
	.byte w_lit8
	.byte '\r'
	.byte w_equ

	.byte w_logor
	.byte w_logor
	.byte w_logor

	.byte w_exit

	defword is_delete, "delete?", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte '\b'
	.byte w_equ

	.byte w_swap
	.byte w_lit8
	.byte 0x7F
	.byte w_equ
	.byte w_logor
	.byte w_exit

	defword _token, "_token", e_call, 0
2:
	.byte w_key
	.byte w_dup
	.byte w_is_delim
	.byte w_0branch
	.byte 1f - .
	.byte w_drop
	.byte w_exit
1:
	.byte w_dup
	.byte w_is_delete
	.byte w_0branch
	.byte 1f - .
	.byte w_drop
	.byte w_tib_drop
	.byte w_false
	.byte w_0branch
	.byte 2b - .
1:
	.byte w_tib_push
	.byte w_false
	.byte w_0branch
	.byte 2b - .

	defword token, "token", e_call, 0
	tib_rst
1:
	.byte w__token
	.byte w_toin
	.byte w_cload
	.byte w_0branch
	.byte 1b - .
	.byte w_exit

	defword dsdump, ".s", e_call, 0
	.byte w_depth
	.byte w_lit8
	.byte '<'
	.byte w_emit
	.byte w_dot
	.byte w_lit8
	.byte '>'
	.byte w_emit

	.byte w_depth
	.byte w_psb_load
	.byte w_swap

	.byte w_dup
	.byte w_0branch
	.byte 1f - .

2:
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_swap
	.byte w_dup
	.byte w_load
	.byte w_dot
	.byte w_lit8
	.byte ' '
	.byte w_emit
	.byte w_lit8
	.byte 4
	.byte w_plus
	.byte w_swap

	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 2b - .
1:
	.byte w_drop
	.byte w_drop
	.byte w_exit

	defword nip, "nip", e_call, 0
	.byte w_swap
	.byte w_drop
	.byte w_exit

	defword min, "min", e_call, 0
	.byte w_over
	.byte w_over
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
	.byte w_rot
	.byte w_dup
	.byte w_cload
	.byte w_tor
	.byte w_lit8
	.byte 1
	.byte w_plus

	.byte w_rot
	.byte w_dup
	.byte w_cload
	.byte w_tor
	.byte w_lit8
	.byte 1
	.byte w_plus

	.byte w_rot
	.byte w_lit8
	.byte 1
	.byte w_minus

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
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 3b - .
1:
	.byte w_nip
	.byte w_nip
	.byte w_exit

	defword ge, ">=", e_call, 0
	.byte w_2dup
	.byte w_gt
	.byte w_tor
	.byte w_equ
	.byte w_fromr
	.byte w_logor
	.byte w_exit

	defword xt2desc, "xt2desc", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte RAM_XTMIN
	.byte w_ge
	.byte w_0branch
	.byte 1f - .

	.byte w_lit8
	.byte RAM_XTMIN
	.byte w_minus
	.byte w_lit8
	.byte 2
	.byte w_lshift
	.byte w_lit32
	.4byte wtb_ram_base
	.byte w_plus
	.byte w_load
	.byte w_exit
1:

	.byte w_lit8
	.byte 2
	.byte w_lshift
	.byte w_lit32
	.4byte wtb_rom_base
	.byte w_plus
	.byte w_load

	.byte w_exit


	defword xt2nlen, "xt2nlen", e_call, 0
	.byte w_xt2desc
	.byte w_lit8
	.byte NLEN_SHIFT
	.byte w_rshift
	.byte w_lit32
	.4byte NLEN_MASK
	.byte w_logand
	.byte w_exit

	defword xt2body, "xt2body", e_call, 0
	.byte w_xt2desc
	.byte w_lit32
	.4byte ADDR_MASK
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
	.4byte rom_latest
	.byte w_load
	.byte w_exit

	defword ram_xtmin, "ram-xtmin", e_call, 0
	.byte w_lit32
	.4byte RAM_XTMIN
	.byte w_exit

	defword ram_latest, "ram-latest", e_call, 0
	.byte w_lit32
	.4byte ram_latest
	.byte w_load
	.byte w_exit

	defword rom_words, "rom-words", e_call, 0
	.byte w_rom_latest

1:
	.byte w_dup
	.byte w_xt2name
	.byte w_over
	.byte w_xt2nlen
	.byte w_type
	.byte w_lit8
	.byte ' '
	.byte w_emit

	.byte w_lit8
	.byte 1
	.byte w_minus

	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 1b - .
	.byte w_drop
	.byte w_exit

	defword ram_words, "ram-words", e_call, 0
	.byte w_ram_latest

1:
	.byte w_dup
	.byte w_xt2name
	.byte w_over
	.byte w_xt2nlen
	.byte w_type
	.byte w_lit8
	.byte ' '
	.byte w_emit

	.byte w_lit8
	.byte 1
	.byte w_minus

	.byte w_dup
	.byte w_lit8
	.byte RAM_XTMIN
	.byte w_lt
	.byte w_0branch
	.byte 1b - .
	.byte w_drop
	.byte w_exit

	defword words, "words", e_call, 0
	.byte w_ram_words
	.byte w_rom_words
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

	defword rom_find, "rom-find", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 1f - .
	.byte w_nip
	.byte w_exit
1:
	.byte w_rom_latest
1:
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
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 2f - .
	.byte w_nip
	.byte w_nip
	.byte w_exit
2:
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 1b - .
	.byte w_nip
	.byte w_nip
	.byte w_exit

	defword ram_find, "ram-find", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 1f - .
	.byte w_nip
	.byte w_exit
1:
	.byte w_ram_latest
1:
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
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 2f - .
	.byte w_nip
	.byte w_nip
	.byte w_exit
2:
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_dup
	.byte w_lit8
	.byte RAM_XTMIN
	.byte w_lt
	.byte w_0branch
	.byte 1b - .
	.byte w_drop
	.byte w_drop
	.byte w_drop
	.byte w_false
	.byte w_exit

	defword find, "find", e_call, 0
	.byte w_2dup
	.byte w_ram_find
	.byte w_dup
	.byte w_0branch
	.byte 1f - .
	.byte w_nip
	.byte w_nip
	.byte w_exit
1:
	.byte w_drop
	.byte w_rom_find
	.byte w_exit

	defword here, "here", e_call, 0
	.byte w_dp
	.byte w_load
	.byte w_exit

	defword ccomma, "c,", e_call, 0
	.byte w_here
	.byte w_cstore

	.byte w_here
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_dp
	.byte w_store
	.byte w_exit

	defword aligned, "aligned", e_call, 0
	.byte w_lit32
	.4byte (4 - 1)
	.byte w_plus
	.byte w_lit32
	.4byte -4
	.byte w_logand
	.byte w_exit

	defword align, "align", e_call, 0
	.byte w_here
	.byte w_aligned
	.byte w_dp
	.byte w_store
	.byte w_exit

	defword comma, ",", e_call, 0
	.byte w_dup
	.byte w_ccomma

	.byte w_lit8
	.byte 8
	.byte w_rshift
	.byte w_dup
	.byte w_ccomma

	.byte w_lit8
	.byte 8
	.byte w_rshift
	.byte w_dup
	.byte w_ccomma

	.byte w_lit8
	.byte 8
	.byte w_rshift
	.byte w_ccomma

	.byte w_exit

	defword is_aligned, "aligned?", e_call, 0
	.byte w_dup
	.byte w_aligned
	.byte w_equ
	.byte w_exit

	# ( addr u )
	defword dump, "dump", e_call, 0
	.byte w_dup
	.byte w_0branch
	.byte 1f - .
2:
	.byte w_swap

	.byte w_dup
	.byte w_dot
	.byte w_lit8
	.byte ':'
	.byte w_emit
	.byte w_lit8
	.byte ' '
	.byte w_emit
	.byte w_dup
	.byte w_load
	.byte w_dot
	.byte w_lit8
	.byte '\n'
	.byte w_emit
	.byte w_lit8
	.byte '\r'
	.byte w_emit
	.byte w_lit8
	.byte 4
	.byte w_plus

	.byte w_swap
	.byte w_lit8
	.byte 1
	.byte w_minus

	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 2b - .
1:
	.byte w_drop
	.byte w_drop
	.byte w_exit

	defword le, "<=", e_call, 0
	.byte w_2dup
	.byte w_lt
	.byte w_rot
	.byte w_rot
	.byte w_equ
	.byte w_logor
	.byte w_exit

	defword within "within", e_call, 0
	.byte w_tor
	.byte w_over
	.byte w_le
	.byte w_fromr
	.byte w_rot
	.byte w_gt
	.byte w_logand
	.byte w_exit

	defword is_hex, "hex?", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte '0'
	.byte w_lit8
	.byte '9' + 1
	.byte w_within
	.byte w_swap
	.byte w_lit8
	.byte 'A'
	.byte w_lit8
	.byte 'F' + 1
	.byte w_within
	.byte w_logor
	.byte w_exit

	# ( addr u -- flag )
	defword is_number, "number?", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte 2
	.byte w_lt
	.byte w_0branch
	.byte 1f - .
	.byte w_drop
	.byte w_drop
	.byte w_false
	.byte w_exit
1:
	
	.byte w_lit8
	.byte 2
	.byte w_minus

	.byte w_swap
	.byte w_dup
	.byte w_cload
	.byte w_lit8
	.byte '0'
	.byte w_equ
	.byte w_over
	.byte w_cload
	.byte w_lit8
	.byte 'x'
	.byte w_equ
	.byte w_logand
	.byte w_0branch
	.byte 1f - .
	.byte w_drop
	.byte w_drop
	.byte w_false
	.byte w_exit
1:
	.byte w_lit8
	.byte 2
	.byte w_plus
	.byte w_swap

	.byte w_dup
	.byte w_0branch
	.byte 1f - .

3:
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_swap
	.byte w_dup
	.byte w_cload
	.byte w_is_hex
	.byte w_invert
	.byte w_0branch
	.byte 2f - .
	.byte w_drop
	.byte w_drop
	.byte w_false
	.byte w_exit
2:
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_swap
	.byte w_dup
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_0branch
	.byte 3b - .
	.byte w_drop
	.byte w_drop
	.byte w_true
	.byte w_exit
1:
	.byte w_nip
	.byte w_exit

	defword x2n, "x2n", e_call, 0
	.byte w_dup
	.byte w_lit8
	.byte '9'
	.byte w_le
	.byte w_0branch
	.byte 1f - .
	.byte w_lit8
	.byte '0'
	.byte w_minus
	.byte w_exit
1:	
	.byte w_lit8
	.byte 'A' - 0xA
	.byte w_minus
	.byte w_exit

	# ( addr u - u )
	defword number, "number", e_call, 0
	.byte w_2dup
	.byte w_is_number
	.byte w_invert
	.byte w_0branch
	.byte 1f - .
	.byte w_drop
	.byte w_drop
	.byte w_false
	.byte w_exit
1:

	.byte w_lit8
	.byte 2
	.byte w_minus
	.byte w_swap
	.byte w_lit8
	.byte 2
	.byte w_plus
	.byte w_swap

	.byte w_dup
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_lit8
	.byte 2
	.byte w_lshift
	.byte w_lit8
	.byte 0

1:
	.byte w_2swap
	.byte w_dup
	.byte w_0branch
	.byte 2f - . 

	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_swap
	.byte w_dup
	.byte w_cload
	.byte w_x2n
	.byte w_tor
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_swap

	.byte w_2swap
	.byte w_over
	.byte w_fromr
	.byte w_swap
	.byte w_lshift
	.byte w_logor

	.byte w_swap
	.byte w_lit8
	.byte 4
	.byte w_minus
	.byte w_swap
	
	.byte w_false
	.byte w_0branch
	.byte 1b - .

2:
	.byte w_drop
	.byte w_drop
	.byte w_nip
	.byte w_exit


	defentr psp_rst
	mv psp, psb
	next

	defword psp_rst, "psp-rst", e_psp_rst, 0

	defword psp_chk, "psp-chk", e_call, 0
	.byte w_psp_load
	.byte w_psb_load
	.byte w_lt
	.byte w_exit

	defword execute_with_chk, "execute-with-chk", e_call, 0
	.byte w_execute
	.byte w_psp_chk
	.byte w_0branch
	.byte 1f - .
	.byte w_lit32
	.4byte 2f
	.byte w_lit8
	.byte 3f - 2f
	.byte w_type
	.byte w_psp_rst
1:
	.byte w_exit

2:
	.ascii "\n\rdata stack error\n\r"
3:

	defword state, "state", e_call, 0
	.byte w_lit32
	.4byte state
	.byte w_exit

	defword bic, "bic", e_call, 0
	.byte w_invert
	.byte w_logand
	.byte w_exit

	.equ FLAG_COMP, (1 << 0)
	defword compon, "]", e_call, 0
	.byte w_state
	.byte w_load
	.byte w_lit8
	.byte FLAG_COMP
	.byte w_logor
	.byte w_state
	.byte w_store
	.byte w_exit

	defword compoff, "[", e_call, ATTR_IMMED
	.byte w_state
	.byte w_load
	.byte w_lit8
	.byte FLAG_COMP
	.byte w_bic
	.byte w_state
	.byte w_store
	.byte w_exit

	defword is_comp, "comp?", e_call, 0
	.byte w_state
	.byte w_load
	.byte w_lit8
	.byte FLAG_COMP
	.byte w_logand
	.byte w_lit8
	.byte FLAG_COMP
	.byte w_equ
	.byte w_exit

	defword quest, "?", e_call, 0
	.byte w_load
	.byte w_dot
	.byte w_exit


	defword xt_is_immed, "xtimmed?", e_call, 0
	.byte w_xt2desc
	.byte w_lit32
	.4byte ATTR_IMMED
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_equ
	.byte w_invert
	.byte w_exit

	defword wtb_comma, "wtb,", e_call, 0
	.byte w_ram_latest
	.byte w_lit8
	.byte RAM_XTMIN
	.byte w_minus
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_lit8
	.byte 2
	.byte w_lshift
	.byte w_lit32
	.4byte wtb_ram_base
	.byte w_plus
	.byte w_store

	.byte w_ram_latest
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_lit32
	.4byte ram_latest
	.byte w_store

	.byte w_exit

	defword cmove, "cmove", e_call, 0
2:
	.byte w_dup
	.byte w_0branch
	.byte 1f - .
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_tor

	.byte w_over
	.byte w_cload
	.byte w_over
	.byte w_cstore
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_swap
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_swap
	.byte w_fromr

	.byte w_false
	.byte w_0branch
	.byte 2b - .

1:
	.byte w_drop
	.byte w_drop
	.byte w_drop
	.byte w_exit

	# ( name u )
	defword defword, "defword", e_call, 0
	.byte w_here
	.byte w_swap
	.byte w_dup
	.byte w_tor
	.byte w_cmove
	.byte w_fromr

	.byte w_dup
	.byte w_here
	.byte w_plus
	.byte w_dp
	.byte w_store

	.byte w_lit8
	.byte NLEN_SHIFT
	.byte w_lshift

	.byte w_lit8
	.byte e_call
	.byte w_lit8
	.byte ENTR_SHIFT
	.byte w_lshift
	.byte w_logor
	.byte w_here
	.byte w_logor
	.byte w_wtb_comma

	.byte w_exit

	defword docom, ":", e_call, 0
	.byte w_token
	.byte w_tib
	.byte w_toin
	.byte w_cload
	.byte w_defword
	.byte w_compon
	.byte w_exit

	defword doend, ";", e_call, ATTR_IMMED
	.byte w_lit32
	.4byte nam_exit
	.byte w_lit8
	.byte nlen_exit
	.byte w_find
	.byte w_ccomma
	.byte w_compoff
	.byte w_exit

	defword if, "if", e_call, ATTR_IMMED
	.byte w_lit32
	.4byte nam_0branch
	.byte w_lit8
	.byte nlen_0branch
	.byte w_find
	.byte w_ccomma
	.byte w_here
	.byte w_lit8
	.byte 0xFF
	.byte w_ccomma
	.byte w_exit

	defword then, "then", e_call, ATTR_IMMED
	.byte w_here
	.byte w_over
	.byte w_minus
	.byte w_swap
	.byte w_cstore
	.byte w_exit

	defword begin, "begin", e_call, ATTR_IMMED
	.byte w_here
	.byte w_exit

	defword until, "until", e_call, ATTR_IMMED
	.byte w_lit32
	.4byte nam_0branch
	.byte w_lit8
	.byte nlen_0branch
	.byte w_find
	.byte w_ccomma
	.byte w_here
	.byte w_minus
	.byte w_ccomma
	.byte w_exit

	defword feed_dog, "feed-dog", e_call, 0
	.byte w_lit32
	.4byte IWDG_KEYFEED
	.byte w_lit32
	.4byte IWDG_BASE + IWDG_CTLR_R16
	.byte w_wstore
	.byte w_exit

	defword chip_uid, "chip-uid", e_call, 0
        .equ ESIG_BASE, 0x1FFFF700
        .equ ESIG_UNIID1, 0xE8
        .equ ESIG_UNIID2, 0xEC
        .equ ESIG_UNIID3, 0xF0
	.byte w_lit32
	.4byte ESIG_BASE + ESIG_UNIID3
	.byte w_load
	.byte w_lit32
	.4byte ESIG_BASE + ESIG_UNIID2
	.byte w_load
	.byte w_lit32
	.4byte ESIG_BASE + ESIG_UNIID1
	.byte w_load
	.byte w_exit

	defentr mcause_load
	csrr wp, mcause
	dpush wp
	next

	defword mcause_load, "mcause@", e_mcause_load, 0

	defentr mtval_load
	csrr wp, mtval
	dpush wp
	next

	defword mtval_load, "mtval@", e_mtval_load, 0

	defentr mepc_load
	csrr wp, mepc
	dpush wp
	next

	defword mepc_load, "mepc@", e_mepc_load, 0

	defword millis, "millis", e_call, 0
	.byte w_lit32
	.4byte stkcnt
	.byte w_dup
	.byte w_load
	.byte w_swap
	.byte w_lit8
	.byte 4
	.byte w_plus
	.byte w_load
	.byte w_swap
	.byte w_exit

	defword sysrst, "sysrst", e_call, 0
        .equ PFIC_SCTLR, 0xD10
        .equ PFIC_SYSRST, (1 << 31)
	.byte w_lit32
	.4byte PFIC_SYSRST
	.byte w_lit32
	.4byte PFIC_BASE + PFIC_SCTLR
	.byte w_store
1:
	.byte w_false
	.byte w_0branch
	.byte 1b - .

	defword ledon, "ledon", e_call, 0
	.byte w_lit32
	.4byte GPIOC_BASE + GPIO_OUTDR
	.byte w_dup
	.byte w_load
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 3
	.byte w_lshift
	.byte w_logor
	.byte w_swap
	.byte w_store
	.byte w_exit

	defword ledoff, "ledoff", e_call, 0
	.byte w_lit32
	.4byte GPIOC_BASE + GPIO_OUTDR
	.byte w_dup
	.byte w_load
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 3
	.byte w_lshift
	.byte w_bic
	.byte w_swap
	.byte w_store
	.byte w_exit

	defword kermit_buf, "kermit-buf", e_call, 0
	.byte w_lit32
	.4byte kermit_buf
	.byte w_exit

	defword kermit_bufsize, "kermit-bufsize", e_call, 0
	.byte w_lit8
	.byte KBUFMASK
	.byte w_exit

	defword kermit_bufi, "kermit-bufi", e_call, 0
	.byte w_lit32
	.4byte kermit_bufi
	.byte w_exit

	defword kermit_rst, "kermit-rst", e_call, 0
	.byte w_lit8
	.byte 0
	.byte w_kermit_bufi
	.byte w_cstore
	.byte w_exit

	defword kermit_used, "kermit-used", e_call, 0
	.byte w_kermit_bufi
	.byte w_cload
	.byte w_exit

	defword kermit_tochar, "kermit-tochar", e_call, 0
	.byte w_lit8
	.byte 32
	.byte w_plus
	.byte w_exit

	defword kermit_unchar, "kermit-unchar", e_call, 0
	.byte w_lit8
	.byte 32
	.byte w_minus
	.byte w_exit

	defword kermit_push, "kermit-push", e_call, 0
	.byte w_kermit_buf
	.byte w_kermit_used
	.byte w_plus
	.byte w_cstore

	.byte w_kermit_used
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_kermit_bufsize
	.byte w_logand
	.byte w_kermit_bufi
	.byte w_cstore

	.byte w_exit

	defword kermit_mark_chk, "kermit-mark-chk", e_call, 0
	.byte w_lit8
	.byte 0x1
	.byte w_equ
	.byte w_exit

	defword kermit_len_chk, "kermit-len-chk", e_call, 0
	.byte w_lit8
	.byte 3
	.byte w_kermit_tochar
	.byte w_lit8
	.byte 94 + 1
	.byte w_kermit_tochar
	.byte w_within
	.byte w_exit

	defword kermit_seq_chk, "kermit-seq-chk", e_call, 0
	.byte w_lit8
	.byte 0
	.byte w_kermit_tochar
	.byte w_lit8
	.byte 63 + 1
	.byte w_kermit_tochar
	.byte w_within
	.byte w_exit

	defword kermit_mark_wait, "kermit-mark-wait", e_call, 0
	.byte w_kermit_used
	.byte w_0branch
	.byte 2f - .
	.byte w_exit
2:
	.byte w_key
	.byte w_dup
	.byte w_kermit_mark_chk
	.byte w_0branch
	.byte 1f - .
	.byte w_kermit_push
	.byte w_exit
1:
	.byte w_drop
	.byte w_false
	.byte w_0branch
	.byte 2b - .
	.byte w_exit

	defword interpret, "interpret", e_call, 0
	.byte w_token
	.byte w_tib
	.byte w_toin
	.byte w_cload

	.byte w_find
	.byte w_dup
	.byte w_0branch
	.byte interpret_number - .

	# state immed action
	# true  false compile
	# false true  execute
	# true  true  execute 
	# false false execute

	.byte w_dup
	.byte w_xt_is_immed
	.byte w_0branch
	.byte 1f - .
	.byte w_execute_with_chk
	.byte w_exit
1:
	.byte w_is_comp
	.byte w_0branch
	.byte 1f - .
	.byte w_ccomma
	.byte w_exit
1:
	.byte w_execute_with_chk
	.byte w_exit

interpret_number:
	.byte w_drop
	.byte w_tib
	.byte w_toin
	.byte w_cload
	.byte w_2dup
	.byte w_is_number
	.byte w_0branch
	.byte interpret_badword - .
	.byte w_number
	.byte w_is_comp
	.byte w_0branch
	.byte 1f - .

	.byte w_dup
	.byte w_lit8
	.byte 0xFF
	.byte w_logand
	.byte w_over
	.byte w_equ
	.byte w_0branch
	.byte 2f - .
	.byte w_lit32
	.4byte nam_lit8
	.byte w_lit8
	.byte nlen_lit8
	.byte w_find
	.byte w_ccomma
	.byte w_ccomma
	.byte w_exit
2:
	.byte w_lit32
	.4byte nam_lit32
	.byte w_lit8
	.byte nlen_lit32
	.byte w_find
	.byte w_ccomma
	.byte w_comma
1:
	.byte w_exit

interpret_badword:
	.byte w_type
	.byte w_lit32
	.4byte 1f
	.byte w_lit8
	.byte 2f - 1f
	.byte w_type
	.byte w_psp_rst
	.byte w_exit

1:
	.ascii " not found\n\r"
2:

	.section .rodata
boot_human:
#ifdef TEST
test_cmove:
	.byte w_lit32
	.4byte nam_words
	.byte w_here
	.byte w_lit8
	.byte nlen_words
	.byte w_cmove

	.byte w_lit32
	.4byte nam_words
	.byte w_lit8
	.byte nlen_words
	.byte w_here
	.byte w_over
	.byte w_compare
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

test_ram_find:
	.byte w_lit32
	.4byte nam_version
	.byte w_lit8
	.byte nlen_version
	.byte w_find
	.byte w_lit8
	.byte w_version
	.byte w_nepanic
	.byte w_dzchk


test_ram_xt2name:
	.byte w_lit8
	.byte w_version
	.byte w_xt2name
	.byte w_lit32
	.4byte nam_version
	.byte w_nepanic
	.byte w_dzchk

test_ram_version:
	.byte w_version
	.byte w_lit32
	.4byte VERSION
	.byte w_nepanic
	.byte w_dzchk

test_ram_noop:
	.byte w_noop
	.byte w_dzchk

test_ge:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_ge
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_ge
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_ge
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk
	
test_xt_is_immed:
	.byte w_lit8
	.byte w_compoff
	.byte w_xt_is_immed
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte w_compon
	.byte w_xt_is_immed
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_comp:
	.byte w_compon
	.byte w_is_comp
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_compoff
	.byte w_is_comp
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_bic:
	.byte w_lit8
	.byte 0x81
	.byte w_lit8
	.byte 0x01
	.byte w_bic
	.byte w_lit8
	.byte 0x80
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0x81
	.byte w_lit8
	.byte 0x80
	.byte w_bic
	.byte w_lit8
	.byte 0x01
	.byte w_nepanic
	.byte w_dzchk

test_psp_rst:
	.byte w_true
	.byte w_psp_rst
	.byte w_dzchk

test_number:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_number
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 1f
	.byte w_lit8
	.byte 2f - 1f
	.byte w_number
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk


	.byte w_lit32
	.4byte 2f
	.byte w_lit8
	.byte 3f - 2f
	.byte w_number
	.byte w_lit8
	.byte 0xF2
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 3f
	.byte w_lit8
	.byte 4f - 3f
	.byte w_number
	.byte w_lit32
	.4byte 0x01234567
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 4f
	.byte w_lit8
	.byte 5f - 4f
	.byte w_number
	.byte w_lit32
	.4byte 0x89ABCDEF
	.byte w_nepanic
	.byte w_dzchk

	.byte w_false
	.byte w_0branch
	.byte 10f - .
1:
	.ascii "0x0"
2:
	.ascii "0xF2"
3:
	.ascii "0x01234567"
4:
	.ascii "0x89ABCDEF"
5:
10:

test_x2n:
	.byte w_lit8
	.byte '0'
	.byte w_x2n
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte '1'
	.byte w_x2n
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte '9'
	.byte w_x2n
	.byte w_lit8
	.byte 9
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'A'
	.byte w_x2n
	.byte w_lit8
	.byte 0xA
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'F'
	.byte w_x2n
	.byte w_lit8
	.byte 0xF
	.byte w_nepanic
	.byte w_dzchk

test_is_number:
	.byte w_lit8
	.byte 0
	.byte w_dup
	.byte w_is_number
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 2
	.byte w_is_number
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 1f
	.byte w_lit8
	.byte 2f - 1f
	.byte w_is_number
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 2f
	.byte w_lit8
	.byte 3f - 2f
	.byte w_is_number
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 3f
	.byte w_lit8
	.byte 4f - 3f
	.byte w_is_number
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 4f
	.byte w_lit8
	.byte 5f - 4f
	.byte w_is_number
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 5f
	.byte w_lit8
	.byte 6f - 5f
	.byte w_is_number
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_false
	.byte w_0branch
	.byte 10f - .
1:
	.ascii "0x"
2:
	.ascii "0X"
3:
	.ascii "0xa"
4:	
	.ascii "0x0M"
5:
	.ascii "0x123456789ABCDEF"
6:
10:

test_is_hex:
	.byte w_lit8
	.byte '0'
	.byte w_is_hex
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk


	.byte w_lit8
	.byte '0' - 1
	.byte w_is_hex
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte '9'
	.byte w_is_hex
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk


	.byte w_lit8
	.byte '9' + 1
	.byte w_is_hex
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk


	.byte w_lit8
	.byte 'A' - 1
	.byte w_is_hex
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'A'
	.byte w_is_hex
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'F'
	.byte w_is_hex
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'F' + 1
	.byte w_is_hex
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_within:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 2
	.byte w_within
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 2
	.byte w_within
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 2
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 2
	.byte w_within
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_le:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_le
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_le
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_le
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk


test_dump:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_dump
	.byte w_dzchk

	.byte w_align
	.byte w_here
	.byte w_lit8
	.byte 1
	.byte w_dump
	.byte w_dzchk

	.byte w_align
	.byte w_here
	.byte w_lit8
	.byte 3
	.byte w_dump
	.byte w_dzchk

test_is_aligned:
	.byte w_lit8
	.byte 0
	.byte w_is_aligned
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_is_aligned
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 4
	.byte w_is_aligned
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 8
	.byte w_is_aligned
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_comma:
	.byte w_lit32
	.4byte 0xFFAA7733
	.byte w_comma
	.byte w_dzchk

	.byte w_here
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_cload
	.byte w_lit8
	.byte 0xFF
	.byte w_nepanic

	.byte w_here
	.byte w_lit8
	.byte 2
	.byte w_minus
	.byte w_cload
	.byte w_lit8
	.byte 0xAA
	.byte w_nepanic

	.byte w_here
	.byte w_lit8
	.byte 3
	.byte w_minus
	.byte w_cload
	.byte w_lit8
	.byte 0x77
	.byte w_nepanic

	.byte w_here
	.byte w_lit8
	.byte 4
	.byte w_minus
	.byte w_cload
	.byte w_lit8
	.byte 0x33
	.byte w_nepanic



test_aligned:
	.byte w_lit8
	.byte 0
	.byte w_aligned
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_aligned
	.byte w_lit8
	.byte 4
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 2
	.byte w_aligned
	.byte w_lit8
	.byte 4
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 3
	.byte w_aligned
	.byte w_lit8
	.byte 4
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 4
	.byte w_aligned
	.byte w_lit8
	.byte 4
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 5
	.byte w_aligned
	.byte w_lit8
	.byte 8
	.byte w_nepanic
	.byte w_dzchk

test_ccomma:

	.byte w_lit8
	.byte 0x77
	.byte w_ccomma

	.byte w_here
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_cload
	.byte w_lit8
	.byte 0x77
	.byte w_nepanic
	.byte w_dzchk

test_find:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_find
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte nam_next
	.byte w_lit8
	.byte 4
	.byte w_find
	.byte w_lit8
	.byte w_next
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte nam_next
	.byte w_lit8
	.byte 10
	.byte w_find
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte nam_next
	.byte w_lit8
	.byte 1
	.byte w_find
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk
	
test_2over:
	.byte w_lit8
	.byte 10
	.byte w_lit8
	.byte 90
	.byte w_lit8
	.byte 43
	.byte w_lit8
	.byte 47
	.byte w_2over
	.byte w_lit8
	.byte 90
	.byte w_nepanic
	.byte w_lit8
	.byte 10
	.byte w_nepanic
	.byte w_lit8
	.byte 47
	.byte w_nepanic
	.byte w_lit8
	.byte 43
	.byte w_nepanic
	.byte w_lit8
	.byte 90
	.byte w_nepanic
	.byte w_lit8
	.byte 10
	.byte w_nepanic
	.byte w_dzchk

test_2swap:
	.byte w_lit8
	.byte 10
	.byte w_lit8
	.byte 90
	.byte w_lit8
	.byte 43
	.byte w_lit8
	.byte 47
	.byte w_2swap
	.byte w_lit8
	.byte 90
	.byte w_nepanic
	.byte w_lit8
	.byte 10
	.byte w_nepanic
	.byte w_lit8
	.byte 47
	.byte w_nepanic
	.byte w_lit8
	.byte 43
	.byte w_nepanic
	.byte w_dzchk

test_2dup:
	.byte w_lit8
	.byte 5
	.byte w_lit8
	.byte 19
	.byte w_2dup
	.byte w_lit8
	.byte 19
	.byte w_nepanic
	.byte w_lit8
	.byte 5
	.byte w_nepanic
	.byte w_lit8
	.byte 19
	.byte w_nepanic
	.byte w_lit8
	.byte 5
	.byte w_nepanic
	.byte w_dzchk

show_words:
	.byte w_words
	.byte w_lit8
	.byte '\n'
	.byte w_emit
	.byte w_lit8
	.byte '\r'
	.byte w_emit
	.byte w_dzchk

show_rom_word_count:
	.byte w_lit32
	.4byte 1f
	.byte w_lit8
	.byte 2f - 1f
	.byte w_type
	.byte w_rom_latest
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_dot
	.byte w_lit8
	.byte '\n'
	.byte w_emit
	.byte w_lit8
	.byte '\r'
	.byte w_emit
	.byte w_false
	.byte w_0branch
	.byte 2f - .

1:
	.ascii "ROM WORD COUNT: 0x"
2:
	.byte w_dzchk

test_rom_latest:
	.byte w_rom_latest
	.byte w_lit32
	.4byte lastword - 1
	.byte w_nepanic
	.byte w_dzchk

test_xt2name:
	.byte w_lit8
	.byte w_next
	.byte w_xt2name
	.byte w_lit32
	.4byte nam_next
	.byte w_nepanic
	.byte w_dzchk

test_xt2body:
	.byte w_lit8
	.byte w_next
	.byte w_xt2body
	.byte w_lit32
	.4byte body_next
	.byte w_nepanic
	.byte w_dzchk


test_xt2nlen:
	.byte w_lit8
	.byte w_next
	.byte w_xt2nlen
	.byte w_lit8
	.byte nlen_next
	.byte w_nepanic
	.byte w_dzchk

test_xt2desc:
	.byte w_lit8
	.byte w_next
	.byte w_xt2desc
	.byte w_lit32
	.4byte wtb_next
	.byte w_load
	.byte w_nepanic
	.byte w_dzchk

test_compare:
	.byte w_lit8
	.byte 0
	.byte w_dup
	.byte w_dup
	.byte w_dup
	.byte w_compare
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte nam_lit8
	.byte w_lit8
	.byte 3
	.byte w_lit32
	.4byte nam_lit32
	.byte w_lit8
	.byte 3
	.byte w_compare
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte nam_lit8
	.byte w_lit8
	.byte 4
	.byte w_lit32
	.4byte nam_lit32
	.byte w_lit8
	.byte 4
	.byte w_compare
	.byte w_lit8
	.byte 0
	.byte w_lt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte nam_lit32
	.byte w_lit8
	.byte 4
	.byte w_lit32
	.4byte nam_lit8
	.byte w_lit8
	.byte 4
	.byte w_compare
	.byte w_lit8
	.byte 0
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_min:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_min
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_min
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

test_nip:
	.byte w_lit8
	.byte 2
	.byte w_lit8
	.byte 3
	.byte w_nip
	.byte w_lit8
	.byte 3
	.byte w_nepanic
	.byte w_dzchk

test_dsdump:
	.byte w_dsdump
	.byte w_false
	.byte w_dsdump
	.byte w_true
	.byte w_dsdump
	.byte w_drop
	.byte w_dsdump
	.byte w_drop
	.byte w_dsdump
	.byte w_dzchk

test_is_delete:
	.byte w_lit8
	.byte '\b'
	.byte w_is_delete
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0x7F
	.byte w_is_delete
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'A'
	.byte w_is_delete
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_is_delim:
	.byte w_lit8
	.byte ' '
	.byte w_is_delim
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte '\t'
	.byte w_is_delim
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte '\n'
	.byte w_is_delim
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte '\r'
	.byte w_is_delim
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 'A'
	.byte w_is_delim
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

test_type:
	.byte w_lit32
	.4byte nam_type
	.byte w_lit8
	.byte nlen_type
	.byte w_type
	.byte w_dzchk

test_tib:
	tib_rst
	.byte w_lit8
	.byte 0x55
	.byte w_tib_push
	.byte w_toin
	.byte w_cload
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_tib
	.byte w_cload
	.byte w_lit8
	.byte 0x55
	.byte w_nepanic
	.byte w_dzchk
	.byte w_tib_drop
	.byte w_toin
	.byte w_cload
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_tib_drop
	.byte w_toin
	.byte w_cload
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

test_tib_chk:
	.byte w_lit8
	.byte TIBMASK + 1
	.byte w_toin
	.byte w_cstore
	tib_chk
	.byte w_false
	.byte w_nepanic

	tib_rst
	tib_chk
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_dot:
	.byte w_lit32
	.4byte 0x01234567
	.byte w_dot
	.byte w_dzchk
	.byte w_lit32
	.4byte 0x89ABCDEF
	.byte w_dot
	.byte w_dzchk

test_over:
	.byte w_lit8
	.byte 17
	.byte w_lit8
	.byte 21
	.byte w_over
	.byte w_lit8
	.byte 17
	.byte w_nepanic
	.byte w_lit8
	.byte 21
	.byte w_nepanic
	.byte w_lit8
	.byte 17
	.byte w_nepanic
	.byte w_dzchk

test_tor_fromr:
	.byte w_lit8
	.byte 15
	.byte w_tor
	.byte w_dzchk
	.byte w_fromr
	.byte w_lit8
	.byte 15
	.byte w_nepanic
	.byte w_dzchk

test_swap:	
	.byte w_lit8
	.byte 9
	.byte w_lit8
	.byte 13
	.byte w_swap
	.byte w_lit8
	.byte 9
	.byte w_nepanic
	.byte w_lit8
	.byte 13
	.byte w_nepanic
	.byte w_dzchk

test_dup_drop:
	.byte w_lit8
	.byte 3
	.byte w_dup
	.byte w_nepanic
	.byte w_true
	.byte w_drop
	.byte w_dzchk

test_uart_tx:
	.byte w_uart_txwait
	.byte w_lit8
	.byte 'U'
	.byte w_uart_cstore
	.byte w_dzchk

test_pause:
	.byte w_pause
	.byte w_dzchk

test_cload_cstore:
	.byte w_lit8
	.byte 0x33
	.byte w_here
	.byte w_cstore
	.byte w_here
	.byte w_cload
	.byte w_lit8
	.byte 0x33
	.byte w_nepanic
	.byte w_dzchk

test_wload_wstore:
	.byte w_align
	.byte w_lit32
	.4byte 0xAAAA
	.byte w_here
	.byte w_wstore
	.byte w_here
	.byte w_wload
	.byte w_lit32
	.4byte 0xAAAA
	.byte w_nepanic
	.byte w_dzchk

test_load_store:
	.byte w_align
	.byte w_lit32
	.4byte 0x55555555
	.byte w_here
	.byte w_store
	.byte w_here
	.byte w_load
	.byte w_lit32
	.4byte 0x55555555
	.byte w_nepanic
	.byte w_dzchk


test_depth:
	.byte w_depth
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_depth
	.byte w_nepanic
	.byte w_dzchk

test_minus:
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 2
	.byte w_lit8
	.byte 1
	.byte w_minus
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

test_negate:
	.byte w_lit32
	.4byte 0
	.byte w_negate
	.byte w_lit32
	.4byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 1
	.byte w_negate
	.byte w_lit32
	.4byte -1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte -1
	.byte w_negate
	.byte w_lit32
	.4byte 1
	.byte w_nepanic
	.byte w_dzchk

test_invert:
	.byte w_false
	.byte w_invert
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte 0x55555555
	.byte w_invert
	.byte w_lit32
	.4byte 0xAAAAAAAA
	.byte w_nepanic
	.byte w_dzchk

test_gt:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_gt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_gt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_gt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_lt:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_lt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_lt
	.byte w_false
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_lt
	.byte w_true
	.byte w_nepanic
	.byte w_dzchk

test_rshift:
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_rshift
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 4
	.byte w_lit8
	.byte 2
	.byte w_rshift
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit32
	.4byte (1 << 31)
	.byte w_lit8
	.byte 31
	.byte w_rshift
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

test_lshift:
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_lshift
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 2
	.byte w_lshift
	.byte w_lit8
	.byte 4
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 31
	.byte w_lshift
	.byte w_lit32
	.4byte (1 << 31)
	.byte w_nepanic
	.byte w_dzchk

test_logxor:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_logxor
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_logxor
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_logxor
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_logxor
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

test_logor:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_logor
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_logor
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_logor
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_logor
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

test_logand:
	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 0
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 0
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 0
	.byte w_lit8
	.byte 1
	.byte w_logand
	.byte w_lit8
	.byte 0
	.byte w_nepanic
	.byte w_dzchk

	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_logand
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

test_plus:
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_plus
	.byte w_lit8
	.byte 2
	.byte w_nepanic
	.byte w_dzchk

test_equ:
	.byte w_lit8
	.byte 1
	.byte w_lit8
	.byte 1
	.byte w_nepanic
	.byte w_dzchk

test_0branch:
	.byte w_lit8
	.byte 1
	.byte w_0branch
	.byte panic - .
	.byte w_lit8
	.byte 0
	.byte w_0branch
	.byte 1f - .
	.byte w_panic
1:
/*
test_echo:
1:
	.byte w_key
	.byte w_emit
	.byte w_false
	.byte w_0branch
	.byte 1b - .

*/

/*
test_token:
1:
	.byte w_token
	.byte w_dzchk

	.byte w_lit8
	.byte '['
	.byte w_emit
	.byte w_tib
	.byte w_toin
	.byte w_cload
	.byte w_type
	.byte w_lit8
	.byte ']'
	.byte w_emit

	.byte w_false
	.byte w_0branch
	.byte 1b - .
*/

#endif
	.byte w_next
	.byte w_next
	.byte w_next

1:
	.byte w_interpret
	.byte w_false
	.byte w_0branch
	.byte 1b - .

	.byte w_halt
panic:
	.byte w_panic


boot_root:
1:	
	.byte w_feed_dog
	.byte w_pause
	.byte w_false
	.byte w_0branch
	.byte 1b - . 

boot_irq:
	.byte w_lit32
	.4byte 2f
	.byte w_lit8
	.byte 3f - 2f
	.byte w_type

	.byte w_chip_uid
	.byte w_dot
	.byte w_dot
	.byte w_dot

	.byte w_lit32
	.4byte 3f
	.byte w_lit8
	.byte 4f - 3f
	.byte w_type

	.byte w_mcause_load
	.byte w_dot

	.byte w_lit32
	.4byte 4f
	.byte w_lit8
	.byte 5f - 4f
	.byte w_type

	.byte w_mtval_load
	.byte w_dot

	.byte w_lit32
	.4byte 5f
	.byte w_lit8
	.byte 6f - 5f
	.byte w_type

	.byte w_mepc_load
	.byte w_dot

	.byte w_lit32
	.4byte 1f
	.byte w_lit8
	.byte 2f - 1f
	.byte w_type

10:
	.byte w_false
	.byte w_0branch
	.byte 10b - .

1:
	.ascii "\n\r!!! UNHANDLE IRQ !!!\n\r"
2:
	.ascii "\n\rCHIP UID : "
3:
	.ascii "\n\rMCAUSE : "
4:
	.ascii "\n\rMTVAL : "
5:
	.ascii "\n\rMEPC : "
6:

	.section .text
forth:
	la wp, xt_emit
	la xp, w_uart_cstore
	sw xp, 0(wp)

	la wp, xt_keyava
	la xp, w_uart_rxstat
	sw xp, 0(wp)

	la wp, xt_key
	la xp, w_uart_cload
	sw xp, 0(wp)

	la wp, rom_latest
	la xp, lastword - 1
	sw xp, 0(wp)

	la wp, state
	sw zero, 0(wp)

	la wp, kermit_bufi
	sb zero, 0(wp)

	la up, user_irq
	sw up, USER_OFFSET_NEXT(up)
	la ip, boot_irq
	la rsp, rstk_irq
	la psp, dstk_irq
	mv psb, psp
	call user_save

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

	next

	.section .bss
stkcnt:
	.word 0
	.word 0
user_human:	
	.fill USRSIZE, 4, 0
state:
	.fill 1, 4, 0
rom_latest:
	.fill 1, 4, 0
xt_emit:
	.fill 1, 4, 0
xt_key:
	.fill 1, 4, 0
xt_keyava:
	.fill 1, 4, 0
toin:
	.fill 1, 1, 0
tib:
	.fill TIBMASK + 1, 1, 0
urxhead:
	.fill 1, 1, 0
urxnum:
	.fill 1, 1, 0
urxfifo:
	.fill URXMASK + 1, 1, 0

	.p2align 2
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
user_irq:
	.fill USRSIZE, 4, 0
rstk_irq:
	.fill STKSIZE, 4, 0
dstk_irq:
	.fill STKSIZE, 4, 0
kermit_buf:
	.fill KBUFMASK, 1, 0
kermit_bufi:
	.fill 1, 1, 0

	.section .wtb_ram
	.p2align 2, 0
wtb_ram_base:

	.section .data

	.set lastword_ram, RAM_XTMIN

	.macro rdefword lab, nam, entr, attr
	.section .dict_ram
	nam_\lab:
	.ascii "\nam"
	nam_end_\lab:
	.set nlen_\lab, nam_end_\lab - nam_\lab
	.set nlen_shifted_\lab, nlen_\lab << NLEN_SHIFT
	.set entr_shifted_\lab, \entr << ENTR_SHIFT
	body_\lab:
	.section .wtb_ram
	wtb_\lab:
	.4byte body_\lab + entr_shifted_\lab + nlen_shifted_\lab + \attr
	.set w_\lab, lastword_ram
	.set lastword_ram, lastword_ram + 1
	.section .dict_ram
	.endm

	rdefword noop, "noop", e_next, 0
	rdefword version, "version", e_call, 0
	.byte w_lit32
	.4byte VERSION
	.byte w_exit

	.section .wtb_ram
	.p2align 8, 0xFF
	# must reverse some space used for runtime word add
	.asciz "RWTBEND"

	.section .dict_ram
	.p2align 2, 0xFF
dp:
	.4byte dict
ram_latest:
	.4byte lastword_ram - 1


	.section .bss
	.p2align 2, 0xFF
dict:



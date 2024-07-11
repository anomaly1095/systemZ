
@ # SystemZ Kernel <PRODUCTION BRANCH>

@ Copyright (C) 2024 Connexion Nord, Inc. or its affiliates. All Rights Reserved.

@ SPDX-License-Identifier: MIT

@ Permission is hereby granted, free of charge, to any person obtaining a copy of
@ this software and associated documentation files (the "Software"), to deal in
@ the Software without restriction, including without limitation the rights to
@ use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
@ the Software, and to permit persons to whom the Software is furnished to do so,
@ subject to the following conditions:

@ The above copyright notice and this permission notice shall be included in all
@ copies or substantial portions of the Software.

@ THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
@ IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
@ FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
@ COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
@ IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
@ CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

@ <https://github.com/anomaly1095/systemZ>


.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

@ RCC register details provided in STM32F401's ref manual page 103

@----------------------------------------------------------------------
@---------------------------------------------------------------------- startup section
.section .text.sysinit, "ax", %progbits

@-----------------------------------
@ main function called by system initialization to take care of clocks
@-----------------------------------
  .global _RCC_config
  .type _RCC_config, %function
_RCC_config:
  PUSH    {lr}
  LDR     r0, =RCC_BASE
  BL      _HSI_enable
  BL      _PLL_enable
  BL      _SYSCLK_config
  BL      _BUS_prescaler_config
  BL      _RCC_interrupt_config
  @ consult these 3 functions
  @ & configure the masks to suit the needs of the system
  BL      _AHB1_enable_periph_clks
  BL      _APB1_enable_periph_clks
  BL      _APB2_enable_periph_clks
  POP     {lr}
  MOV     r0, #0          @ return 0
  BX      lr
  .align  4
  .size _RCC_config, .-_RCC_config


@-----------------------------------
@ Enable the high speed internal osciallator
@-----------------------------------
  .type _HSI_enable, %function
_HSI_enable:
  LDR     r1, [r0]        @ RCC_CR
  MOV     r2, #0b1
  ORR     r1, r1, r2
  STR     r1, [r0]        @ RCC_CR
  LSL     r2, r2, #1      @ make the mask ready for next op
  @ Wait for the high speed internal oscillator to be anabled
__HSI_wait:
  LDR     r1, [r0]        @ RCC_CR
  TST     r1, r2          @ use previously shifted mask to check if HSIRDY
  BEQ     __HSI_wait
  BX      lr
  .align  4
  .size _HSI_enable, .-_HSI_enable


@-----------------------------------
@ Configure then enable the Phase locked loop
@-----------------------------------
  .type _PLL_enable, %function
_PLL_enable:
  @ Configure PLL
  LDR     r1, [r0, #0x04] @ RCC_PLLCFGR
  MOVW    r2, #16         @ PLLM prescaler = 16
  ORR     r1, r1, r2
  MOVW    r2, #(336 << 6) @ PLLN prescaler = 336
  MOVT    r2, #0b01       @ PLLP prescaler = 4
  ORR     r1, r1, r2
  MOV     r2, #0
  MOVT    r2, #(0b1 << 6) @ PLLSRC
  BIC     r1, r1, r2      @ clear bit to set HSI as PLL's input
  STR     r1, [r0, #0x04]
  @ Enable PLL and clock security system
  LDR     r1, [r0]        @ RCC_CR
  MOV     r2, #0
  MOVT    r2, #0x108      @ PLLON and CSSON bit set
  ORR     r1, r1, r2
  STR     r1, [r0]        @ RCC_CR
  LSL     r2, r2, #1      @ make the mask ready for the next op
__PLL_wait:
  LDR     r1, [r0]        @ RCC_CR
  TST     r1, r2          @ use previously shifted mask to check if PLLRDY
  BEQ     __PLL_wait
  BX      lr
  .align  4
  .size _PLL_enable, .-_PLL_enable


@-----------------------------------
@ Set PLL as sysclk input thru the provided multiplexer
@-----------------------------------
  .type _SYSCLK_config, %function
_SYSCLK_config:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  MOVW    r2, #0b11
  BIC     r1, r1, r2      @ clear the bits
  MOVW    r2, #0b10
  ORR      r1, r1, r2
  STR     r1, [r0, #0x08] @ PLL_CFGR
  LSL     r2, r2, #1      @ shift the bits to allign sith SWS bits
  @ Wait for the multiplexer to select the PLL as SYSCLK input source
__SYSCLK_wait:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  TST     r1, r2
  BEQ     __SYSCLK_wait
  BX      lr
  .align  4
  .size _SYSCLK_config, .-_SYSCLK_config

@-----------------------------------
@ Set AHB APB1 APB2 prescalers
@-----------------------------------
  .type _BUS_prescaler_config, %function
_BUS_prescaler_config:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  MOVW    r2, #0b1111     
  MOVW    r2, #0xE0F0     @ clear HPRE and PPRE2 bits (AHB & APB2 prescaler bits)
  BIC     r1, r1, r2
  MOVW    r2, #(0b100 << 10)@ set PPRE2 = /2 (APB1 prescaler)
  ORR     r1, r1, r2
  STR     r1, [r0, #0x08] @ PLL_CFGR
  BX      lr
  .align  4
  .size _BUS_prescaler_config, .-_BUS_prescaler_config

@-----------------------------------
@ enable rcc interrupts
@-----------------------------------
  .type _RCC_interrupt_config, %function
_RCC_interrupt_config:
  LDR     r1, [r0, #0x0C] @ RCC_CIR
  MOVW    r2, #(0b111111 << 8)  @ enable all RCC interrupts
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x0C] @ RCC_CIR
  BX      lr
  .align  4
  .size _RCC_interrupt_config, .-_RCC_interrupt_config

@-----------------------------------
@ Enable AHB1 essencial peripherals clocks
@-----------------------------------
  .type _AHB1_enable_periph_clks, %function
_AHB1_enable_periph_clks:
  LDR     r1, [r0, #0x30]   @ RCC_AHB1ENR
  MOVW    r2, #0b11         @ enable GPIOA GPIOB 
  MOVT    r2, #(0b11 << 5)  @ enable DMA1 DMA2
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x30]   @ RCC_AHB1ENR
  BX      lr
  .align  4
  .size _AHB1_enable_periph_clks, .-_AHB1_enable_periph_clks


@-----------------------------------
@ Enable APB1 essencial peripheral clocks
@-----------------------------------
  .type _APB1_enable_periph_clks, %function
_APB1_enable_periph_clks:
  LDR     r1, [r0, #0x40]   @ RCC_APB1ENR
  MOVW    r2, #0b1001       @ enable TIM2 TIM5
  MOVT    r2, #0x1000       @ enable PWR interface
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x40]   @ RCC_APB1ENR
  BX      lr
  .align  4
  .size _APB1_enable_periph_clks, .-_APB1_enable_periph_clks


@-----------------------------------
@ Enable APB2 essential peripheral clocks
@-----------------------------------
  .type _APB2_enable_periph_clks, %function
_APB2_enable_periph_clks:
  LDR     r1, [r0, #0x44]   @ RCC_AHB1ENR
  MOVW    r2, #0x4001       @ enable SYSCFG aand TIM1
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x44]   @ RCC_AHB1ENR
  BX      lr
  .align  4
  .size _APB2_enable_periph_clks, .-_APB2_enable_periph_clks


@-----------------------------------------------
.section .rodata.registers.RCC, "a", %progbits
  .equ RCC_BASE, 0x40023800       @ RCC bit-band base address
  .equ RCC_BASE_BB, 0x42470000    @ RCC base address
  .equ RCC_PLLCFGR_MASK, 0xF437FFF
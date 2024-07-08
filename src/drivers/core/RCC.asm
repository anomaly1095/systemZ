
@ RCC register details provided in STM32F401's ref manual page 103

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
  #include "../../common/registers.asm"
  #include "../../common/macros.asm"


.section .text.drivers.rcc, "ax", %progbits

@-----------------------------------
  .global RCC_config
  .type RCC_config, %function
@ main function called by system initialization to take care of clocks
@-----------------------------------
RCC_config:
  PUSH    {lr}
  LDR     r0, =RCC_BASE
  BL      _HSI_enable
  BL      _PLL_enable
  BL      _SYSCLK_config
  BL      _BUS_prescaler_config
  POP     {lr}
  BX      lr
  .align  4
  .size RCC_config, .-RCC_config


@-----------------------------------
@ Enable the high speed internal osciallator
  .type _HSI_enable, %function
@-----------------------------------
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
  .align 4
  .size _HSI_enable, .-_HSI_enable


@-----------------------------------
@ Configure then enable the Phase locked loop
  .type _PLL_enable, %function
@-----------------------------------
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
  .align 4
  .size _PLL_enable, .-_PLL_enable


@-----------------------------------
@ Set PLL as sysclk input thru the provided multiplexer
  .type _SYSCLK_config, %function
@-----------------------------------
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
  .align 4
  .size _SYSCLK_config, .-_SYSCLK_config

@-----------------------------------
@ Set AHB APB1 APB2 prescalers
@-----------------------------------
  .type _BUS_prescaler_config, %function
_BUS_prescaler_config:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  MOVW    r2, #0b1111     
  MOVW    r2, #0xE0F0       @ clear HPRE and PPRE2 bits (AHB & APB2 prescaler bits)
  BIC     r1, r1, r2
  MOVW    r2, #(0b100 << 10)@ set PPRE2 = /2 (APB1 prescaler)
  ORR     r1, r1, r2
  STR     r1, [r0, #0x08] @ PLL_CFGR
  BX      lr
  .align 4
  .size _BUS_prescaler_config, .-_BUS_prescaler_config

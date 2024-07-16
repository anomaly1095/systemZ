
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
@ Author: Youssef Azaiez

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
.include "include.s"


@----------------------------------------------
@ Default Handler Section
.section .text.default_handler, "ax", %progbits
	.type _default_handler, %function
  .weak _default_handler
	.global _default_handler

_default_handler:
  B       .                   @ Infinite loop for debugging
  .align 4
.size _default_handler, .-_default_handler

@----------------------------------------------
@ Reset Handler Section
.section .text.reset_handler, "ax", %progbits
	.type _reset_handler, %function
	.global _reset_handler
	.extern _start

_reset_handler:
  @ disable interrupts with configurable priority levels
  LDR     sp, _ekstack       @ Load kernel stack pointer

  CPSID   I         @ disable interrupts till end of system initialization
  
  @ Disable fp instructions executing out of order
  @ Disables IT folding
  @ Disable write buffer use
  @ Disables interruption of load/store and multiply/divide operations
  DIS_OUTOFORDER_EXEC #1

  @ set stack alignement
  BL      stack_align_4
  
  @ Copy .kdata section from FLASH to SRAM
  LDR     r0, =_sikdata
  LDR     r1, =_skdata
  LDR     r2, =_ekdata
  .kdata_copy:
    LDR     r3, [r0], #4        @ Load word from .kdata Flash section
    CMP     r1, r2
    IT      NE
    STRNE   r3, [r1], #4        @ Store word in .kdata SRAM section
    BNE     .kdata_copy

  @ Zero out .kbss section in SRAM
  LDR     r0, =_skbss
  LDR     r1, =_ekbss
  MOV     r2, #0
  .kbss_zero:
    CMP     r0, r1
    ITT     NE
    STRNE   r2, [r0], #4
    BNE     .kbss_zero

  @ System initialization
  BL      _sysinit

  @ Copy .data section from FLASH to SRAM
  LDR     r0, =_sidata
  LDR     r1, =_sdata
  LDR     r2, =_edata
  .data_copy:
    LDR     r3, [r0], #4        @ Load word from .data Flash section
    CMP     r1, r2
    IT      NE
    STRNE   r3, [r1], #4        @ Store word in .data SRAM section
    BNE     .data_copy
  
	@ Zero out .bss section in SRAM
  LDR     r0, =_sbss
  LDR     r1, =_ebss
  MOV     r2, #0
  .bss_zero:
    CMP     r0, r1
    ITT     NE
    STRNE   r2, [r0], #4
    BNE     .bss_zero

  @ enable interrupts with configurable priority levels
  CPSIE   I

  @ Configure CONTROL register and switch to unprivileged thread mode
  MOV     r1, #0b011
  MRS     r0, CONTROL
  ORR     r0, r0, r1
  MSR     CONTROL, r0
  LDR     sp, =_estack        @ Load app stack pointer in PSP
  BL      _start              @ Branch to application code
  B       _default_handler    @ Debug: if return from application
  .align 4
.size _reset_handler, .-_reset_handler

@----------------------------------------------
@ System Initialization
.section .text.sysinit, "ax", %progbits
	.global _sysinit
	.type _sysinit, %function

_sysinit:
	PUSH    {lr}              @ Preserve the link register
  
  @ CLOCK configuration
	BL      _RCC_config
  CBNZ    r0, _default_handler  @ Branch to default handler on error
  
  @ POWER configuration
	BL      _PWR_config
  CBNZ    r0, _default_handler

	@ MPU configuration
	BL      _MPU_config
  CBNZ    r0, _default_handler

	@ FLASH configuration
	BL      _FLASH_config
  CBNZ    r0, _default_handler

	@ NVIC configuration
  BL      _NVIC_config
  CBNZ    r0, _default_handler
  
  @ SYSTICK configuration
  BL      _SYSTICK_config
  CBNZ    r0, _default_handler

  POP     {pc}               @ Restore the link register
  .align  4
.size _sysinit, .-_sysinit   @ Define the size of the function



@ SYSTICK register details provided in stm32-cortex-M4 page 246 
@-----------------------------------
@ main function called by system initialization to configure the SYSTICK
@-----------------------------------
  .type _SYSTICK_config, %function
_SYSTICK_config:
  @ save link register 
  LDR     r0, =SYSTICK_BASE
  
  @ load the systick counter value (10499) in ST_LOAD to prooduce a tick each 1ms  
  MOVW    r1, =SYSTICK_COUNTER
  STR     r1, [r0, #0x04]           @ STK_LOAD
  MOVW    r1, #0
  STR     r1, [r0, #0x08]           @ STK_VAL
  @ CLKSOURCE = AHB/8, enable SYSTICK interrupt, enable SYSTICK
  MOVW    r1, #0b011
  STR     r1, [r0, #0x08]           @ STK_VAL

  @ recover link register and return 0
  MOV     r0, #0
  BX      lr
  .align  4
  .size _SYSTICK_config, .-_SYSTICK_config

@ RCC register details provided in STM32F401's ref manual page 103
@-----------------------------------
@ main function called by system initialization to take care of clocks
@-----------------------------------
  .type _RCC_config, %function
_RCC_config:
  LDR     r0, =RCC_BASE
@ Enable the high speed internal osciallator
.HSI_enable:
  LDR     r1, [r0]        @ RCC_CR
  ORR     r1, r1, #0b01
  STR     r1, [r0]        @ RCC_CR
  @ Wait for the high speed internal oscillator to be anabled
.HSI_wait:
  LDR     r1, [r0]        @ RCC_CR
  TST     r1, #0b10          @ use previously shifted mask to check if HSIRDY
  BEQ     .HSI_wait

@ Configure then enable the Phase locked loop
.PLL_enable:
  @ Configure PLL
  LDR     r1, [r0, #0x04] @ RCC_PLLCFGR
  ORR     r1, r1, #16     @ PLLM prescaler = 16
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
.PLL_wait:
  LDR     r1, [r0]        @ RCC_CR
  TST     r1, r2          @ use previously shifted mask to check if PLLRDY
  BEQ     .PLL_wait

@ Set PLL as sysclk input thru the provided multiplexer
.SYSCLK_config:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  BIC     r1, r1, #0b11      @ clear the bits
  ORR     r1, r1, #0b10
  STR     r1, [r0, #0x08] @ PLL_CFGR
  @ Wait for the multiplexer to select the PLL as SYSCLK input source
.SYSCLK_wait:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  TST     r1, #0b100
  BEQ     .SYSCLK_wait

@ Set AHB APB1 APB2 prescalers
.BUS_prescaler_config:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  MOVW    r2, #0b1111     
  MOVW    r2, #0xE0F0     @ clear HPRE and PPRE2 bits (AHB & APB2 prescaler bits)
  BIC     r1, r1, r2
  MOVW    r2, #(0b100 << 10)@ set PPRE2 = /2 (APB1 prescaler)
  ORR     r1, r1, r2
  STR     r1, [r0, #0x08] @ PLL_CFGR

@ enable rcc interrupts
.RCC_interrupt_config:
  LDR     r1, [r0, #0x0C] @ RCC_CIR
  MOVW    r2, #(0b111111 << 8)  @ enable all RCC interrupts
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x0C] @ RCC_CIR

@ Enable AHB1 essencial peripherals clocks
.AHB1_enable_periph_clks:
  LDR     r1, [r0, #0x30]   @ RCC_AHB1ENR
  MOVW    r2, #0b11         @ enable GPIOA GPIOB 
  MOVT    r2, #(0b11 << 5)  @ enable DMA1 DMA2
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x30]   @ RCC_AHB1ENR
@ Enable APB1 essencial peripheral clocks
.APB1_enable_periph_clks:
  LDR     r1, [r0, #0x40]   @ RCC_APB1ENR
  MOVW    r2, #0b1001       @ enable TIM2 TIM5
  MOVT    r2, #0x1000       @ enable PWR interface
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x40]   @ RCC_APB1ENR

@ Enable APB2 essential peripheral clocks
  LDR     r1, [r0, #0x44]   @ RCC_AHB1ENR
  MOVW    r2, #0x4001       @ enable SYSCFG aand TIM1
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x44]   @ RCC_AHB1ENR
  MOV     r0, #0            @ return 0
  BX      lr
  .align  4
  .size _RCC_config, .-_RCC_config


@ PWR register details provided in STM32F401's ref manual page 87
@-----------------------------------
@ main function called by system initialization to take care of power moes an config
@-----------------------------------
  .type _PWR_config, %function
_PWR_config:
  LDR     r1, [r0]
  @ Disable access to RTC and it's backup domain
  @ Enter Stop mode when CPU enters DEEPSLEED
  @ Low power voltage regulator on during stop mode
  @ Set Voltage regulator in scale 2 mode (adequate for 84Mhz clock)
  @ Set main regulator in low voltage during deep sleep
  @ Set Low power regulator low voltage during deep sleep
  @ FLASH power down during deep-sleep 
  @ Enable power voltage detector and set the threshhold at 2.7 V
  LDR     r2, =PWR_CR_MASK
  ORR     r1, r1, r2
  STR     r1, [r0]
  MOV     r0, #0
  BX      lr
  .align  4
  .size _PWR_config, .-_PWR_config


@ NVIC register details provided in stm32-cortex-M4 Referance Manual page 208

@-----------------------------------
@ main function called by system initialization to configure the NVIC
@-----------------------------------
  .type _NVIC_config, %function
_NVIC_config:
  @ enable RCC / FLASH / FPU
  LDR     r0, =NVIC_ISER0
  LDR     r1, [r0]              @ NVIC_ISER0
  ORR     r1, r1, #(0b11 << 4)  @ set FLASH and RCC bits
  STR     r1, [r0]              @ NVIC_ISER0
  LDR     r1, [r0, #0x08]       @ NVIC_ISER2
  ORR     r1, r1, #0b10000      @ set FPU bit
  STR     r1, [r0, #0x08]       @ NVIC_ISER2
  MOV     r0, #0
  BX      lr
  .align  4
  .size _NVIC_config, .-_NVIC_config


@ FLASH register details provided in STM32F401RE Referance Manual page 60

@-----------------------------------
@ main function called by system initialization to configure the FLASH
@-----------------------------------
  .type _FLASH_config, %function
_FLASH_config:
  LDR     r0, =FLASH_BASE
@ reset & enable caches, enable frefetch
@ Set latency at 2ws for 84mhz AHB clock
.FLASH_access_control:
  LDR     r1, [r0]
  ORR     r1, r1, #0b11   @ reset both instruction and data caches
  STR     r1, [r0]        @ save changes
  MOVW    r2, #0x0702     @ enable caches, prefetch and set latency = 2Ws
  ORR     r1, r1, r2
  STR     r1, [r0]        @ save changes

.FLASH_cr_optcr_config:
  @ Unlock the FLASH_CR
  LDR     r1, =FLASH_KEY1
  STR     r1, [r0, #0x04]     @ store key 1
  LDR     r1, =FLASH_KEY2
  STR     r1, [r0, #0x04]     @ store key 2

  @ FLASH_CR register config
  LDR     r1, [r0, #0x10]     @ load contents of FLASH_CR
  MOVW    r2, #(0b10 << 8)    @ set program size at 32 bits
  MOVT    r2, #(0b11 << 8)    @ enable interrupts
  ORR     r1, r1, r2
  STR     r1, [r0, #0x10]     @ load contents of FLASH_CR

  @ LOCK access to FLASH_CR
  MOV     r2, #1
  ROR     r2, r2, #1          @ set bit 31
  ORR     r1, r1, r2
  STR     r1, [r0, #0x10]     @ load contents of FLASH_CR
  
  @ decide whether we branch in based off the kernel's mode (PROD or DEV)
  .ifdef PRODUCTION_MODE
    BL       __FLASH_opt_config      @ configure FLASH option bytes
  .endif

  MOV     r0, #0      @ RETURN 0
  BX      lr
  .align 4
  .size _FLASH_config, .-_FLASH_config

@-----------------------------------
@ write flash options
@-----------------------------------
  .type __FLASH_opt_config, %function
__FLASH_opt_config:
  PUSH     {lr}
  @ Unlock the option bytes
  LDR     r1, =FLASH_OPTKEY1
  STR     r1, [r0, #0x08]         @ Store opt-key 1
  LDR     r1, =FLASH_OPTKEY2
  STR     r1, [r0, #0x08]         @ Store opt-key 2

  @ Read current FLASH_OPTCR value
  LDR     r1, [r0, #0x14]         @ FLASH_OPTCR

  @ Example: Clear nWRP for sectors 0-5 (protect K_FLASH) and keep sectors 6-7 unprotected
  MOV     r2, #0
  MOVT    r2, #0xFF
  BIC     r1, r1, r2              @ Clear nWRP[7:0]
  MOVT    r2, #0xC0           
  ORR     r1, r1, r2              @ Set nWRP[7:0] to 0xC0 (protect sectors 0-5 for kernel)
  
  @ set BOR (brownout level)
  BIC     r1, r1, #0b1100              @ cler BOR_LEV bits
  ORR     r1, r1, #0b0100              @ set new BOR_LEV bits at BOR level 2

  @ Write modified value back to FLASH_OPTCR
  STR     r1, [r0, #0x14]         @ FLASH_OPTCR
  
  @ Start the option byte programming
  LDR     r1, [r0, #0x14]         @ FLASH_OPTCR
  ORR     r1, r1, #0b10           @ Set OPTSTRT bit
  STR     r1, [r0, #0x14]         @ FLASH_OPTCR

  @ LOCK access to option bytes
  LDR     r1, [r0, #0x14]         @ FLASH_OPTCR
  ORR     r1, r1, #0b1            @ Set OPTLOCK bit
  STR     r1, [r0, #0x14]         @ FLASH_OPTCR
  POP      {pc}
  .align 4
  .size __FLASH_opt_config, .-__FLASH_opt_config



  @ MPU register details provided in ARM cortex-M7 Referance Manual page 200
@-----------------------------------
@ main function called by system initialization to configure the MPU
@-----------------------------------
  .type _MPU_config, %function
_MPU_config:
  LDR     r0, =MPU_BASE
@ Check for system's support for MPU
@ Check support for separate or unified sections
.MPU_type:
  LDR     r1, [r0]
  CMP     r2, #0
  IT      EQ
  BXEQ    lr            @ we return to system init
@ Configure the 8 sections for the system
@ (macro used is defined in src/common/macros.asm)
.MPU_sections_config:
  @ Configure REGION0: 0x00000000 to 0x1FFFFFFF (FLASH for kernel and apps)
  MPU_CONFIG_REGION SECTION0_BASE, 0, SECTION0_MASK

  @ Configure REGION1: 0x20000000 to 0x3FFFFFFF (SRAM)
  MPU_CONFIG_REGION SECTION1_BASE, 1, SECTION1_MASK
  
  @ Configure REGION2: 0x20010000 to 0x20017FFF (Kernel SRAM)
  MPU_CONFIG_REGION SECTION2_BASE, 2, SECTION2_MASK

  @ Configure REGION3: 0x22200000 to 0x222FFFFF (Bit-Band Area of Kernel SRAM)
  MPU_CONFIG_REGION SECTION3_BASE, 3, SECTION3_MASK
  
  @ Configure REGION4: 0x40000000 to 0x5FFFFFFF (Peripheral Registers)
  MPU_CONFIG_REGION SECTION4_BASE, 4, SECTION4_MASK
  
  @ Configure REGION5: 0x40026000 to 0x40026FFF (DMA Controller)
  MPU_CONFIG_REGION SECTION5_BASE, 5, SECTION5_MASK

  @ Configure REGION6: 0x424C0000 to 0x424DFFFF (Bit-Band Area of DMA Controller)
  MPU_CONFIG_REGION SECTION6_BASE, 6, SECTION6_MASK

  @ Configure REGION7: 0xE0000000 to 0xE00FFFFF (System Peripheral Space)
  MPU_CONFIG_REGION SECTION7_BASE, 7, SECTION7_MASK
@  Enable background map, enable mpu during NMI AND FAULTS, enable MPU
.MPU_enable:
  LDR     r1, [r0, #0x04]       @ MPU_CTRL reg
  ORR     r1, r1, #0b111        @ Set ENABLE, PRIVDEFENA, and HFNMIENA bits
  STR     r1, [r0, #0x04]       @ MPU_CTRL reg
  MOV     r0, #0
  BX      lr
  .align 4
  .size _MPU_config, .-_MPU_config

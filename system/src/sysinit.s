
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
// system interrupt vector table for stm32F401xx
  .section .isr_vectors, "a", %progbits
  .type  g_pfnVectors, %object
  .extern _reset_handler
  .extern _default_handler
@----------------------------------------------
 
g_pfnVectors:
  .word  _ekstack     @ end of kernel stack (lowest address)  
  .word  _reset_handler @ initial address of PC
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  
  /* External Interrupts */
  .word     WWDG_IRQHandler
  .word     PVD_IRQHandler
  .word     TAMP_STAMP_IRQHandler
  .word     RTC_WKUP_IRQHandler
  .word     FLASH_IRQHandler                  
  .word     RCC_IRQHandler                    
  .word     EXTI0_IRQHandler
  .word     EXTI1_IRQHandler 
  .word     EXTI2_IRQHandler 
  .word     EXTI3_IRQHandler 
  .word     EXTI4_IRQHandler 
  .word     DMA1_Stream0_IRQHandler 
  .word     DMA1_Stream1_IRQHandler 
  .word     DMA1_Stream2_IRQHandler 
  .word     DMA1_Stream3_IRQHandler 
  .word     DMA1_Stream4_IRQHandler 
  .word     DMA1_Stream5_IRQHandler 
  .word     DMA1_Stream6_IRQHandler 
  .word     ADC_IRQHandler
  .word     0               				  
  .word     0              					  
  .word     0                                 
  .word     0                                 
  .word     EXTI9_5_IRQHandler
  .word     TIM1_BRK_TIM9_IRQHandler
  .word     TIM1_UP_TIM10_IRQHandler
  .word     TIM1_TRG_COM_TIM11_IRQHandler
  .word     TIM1_CC_IRQHandler
  .word     TIM2_IRQHandler
  .word     TIM3_IRQHandler
  .word     TIM4_IRQHandler
  .word     I2C1_EV_IRQHandler
  .word     I2C1_ER_IRQHandler
  .word     I2C2_EV_IRQHandler
  .word     I2C2_ER_IRQHandler
  .word     SPI1_IRQHandler
  .word     SPI2_IRQHandler
  .word     USART1_IRQHandler
  .word     USART2_IRQHandler
  .word     0               				  
  .word     EXTI15_10_IRQHandler
  .word     RTC_Alarm_IRQHandler
  .word     OTG_FS_WKUP_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     DMA1_Stream7_IRQHandler
  .word     0           
  .word     SDIO_IRQHandler
  .word     TIM5_IRQHandler
  .word     SPI3_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     DMA2_Stream0_IRQHandler
  .word     DMA2_Stream1_IRQHandler
  .word     DMA2_Stream2_IRQHandler
  .word     DMA2_Stream3_IRQHandler
  .word     DMA2_Stream4_IRQHandler
  .word     0		  
  .word     0	  
  .word     0	  
  .word     0	  
  .word     0	  
  .word     0	  
  .word     OTG_FS_IRQHandler
  .word     DMA2_Stream5_IRQHandler 
  .word     DMA2_Stream6_IRQHandler 
  .word     DMA2_Stream7_IRQHandler 
  .word     USART6_IRQHandler
  .word     I2C3_EV_IRQHandler
  .word     I2C3_ER_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     FPU_IRQHandler
  .word     0
  .word     0
  .word     SPI4_IRQHandler

  .size  g_pfnVectors, .-g_pfnVectors

@  isr vectors will be overrided by _default_handler or by a stronger definition of the isr 

  .weak      NMI_Handler

  .weak      HardFault_Handler

  .weak      MemManage_Handler

  .weak      BusFault_Handler

  .weak      UsageFault_Handler

  .weak      SVC_Handler

  .weak      DebugMon_Handler

  .weak      PendSV_Handler

  .weak      SysTick_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler, _default_handler
                
  .weak      PVD_IRQHandler
  .thumb_set PVD_IRQHandler, _default_handler
            
  .weak      TAMP_STAMP_IRQHandler
  .thumb_set TAMP_STAMP_IRQHandler, _default_handler
          
  .weak      RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler, _default_handler
          
  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler, _default_handler
                
  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler, _default_handler
                
  .weak      EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler, _default_handler
                
  .weak      EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler, _default_handler
                  
  .weak      EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler, _default_handler
              
  .weak      EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler, _default_handler
                      
  .weak      EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler, _default_handler
                
  .weak      DMA1_Stream0_IRQHandler
  .thumb_set DMA1_Stream0_IRQHandler, _default_handler
      
  .weak      DMA1_Stream1_IRQHandler
  .thumb_set DMA1_Stream1_IRQHandler, _default_handler
                
  .weak      DMA1_Stream2_IRQHandler
  .thumb_set DMA1_Stream2_IRQHandler, _default_handler
                
  .weak      DMA1_Stream3_IRQHandler
  .thumb_set DMA1_Stream3_IRQHandler, _default_handler
              
  .weak      DMA1_Stream4_IRQHandler
  .thumb_set DMA1_Stream4_IRQHandler, _default_handler
                
  .weak      DMA1_Stream5_IRQHandler
  .thumb_set DMA1_Stream5_IRQHandler, _default_handler
                
  .weak      DMA1_Stream6_IRQHandler
  .thumb_set DMA1_Stream6_IRQHandler, _default_handler
                
  .weak      ADC_IRQHandler
  .thumb_set ADC_IRQHandler, _default_handler
          
  .weak      EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler, _default_handler
          
  .weak      TIM1_BRK_TIM9_IRQHandler
  .thumb_set TIM1_BRK_TIM9_IRQHandler, _default_handler
          
  .weak      TIM1_UP_TIM10_IRQHandler
  .thumb_set TIM1_UP_TIM10_IRQHandler, _default_handler
    
  .weak      TIM1_TRG_COM_TIM11_IRQHandler
  .thumb_set TIM1_TRG_COM_TIM11_IRQHandler, _default_handler
    
  .weak      TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler, _default_handler
                
  .weak      TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler, _default_handler
                
  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler, _default_handler
                
  .weak      TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler, _default_handler
                
  .weak      I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler, _default_handler
                  
  .weak      I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler, _default_handler
                  
  .weak      I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler, _default_handler
                
  .weak      I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler, _default_handler
                        
  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, _default_handler
                      
  .weak      SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler, _default_handler
                
  .weak      USART1_IRQHandler
  .thumb_set USART1_IRQHandler, _default_handler
                  
  .weak      USART2_IRQHandler
  .thumb_set USART2_IRQHandler, _default_handler

  .weak      EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler, _default_handler

  .weak      RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler, _default_handler
          
  .weak      OTG_FS_WKUP_IRQHandler
  .thumb_set OTG_FS_WKUP_IRQHandler, _default_handler
          
  .weak      DMA1_Stream7_IRQHandler
  .thumb_set DMA1_Stream7_IRQHandler, _default_handler
                  
  .weak      SDIO_IRQHandler
  .thumb_set SDIO_IRQHandler, _default_handler
                  
  .weak      TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler, _default_handler
                  
  .weak      SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler, _default_handler
                  
  .weak      DMA2_Stream0_IRQHandler
  .thumb_set DMA2_Stream0_IRQHandler, _default_handler
            
  .weak      DMA2_Stream1_IRQHandler
  .thumb_set DMA2_Stream1_IRQHandler, _default_handler
                
  .weak      DMA2_Stream2_IRQHandler
  .thumb_set DMA2_Stream2_IRQHandler, _default_handler
          
  .weak      DMA2_Stream3_IRQHandler
  .thumb_set DMA2_Stream3_IRQHandler, _default_handler
          
  .weak      DMA2_Stream4_IRQHandler
  .thumb_set DMA2_Stream4_IRQHandler, _default_handler
          
  .weak      OTG_FS_IRQHandler
  .thumb_set OTG_FS_IRQHandler, _default_handler
                  
  .weak      DMA2_Stream5_IRQHandler
  .thumb_set DMA2_Stream5_IRQHandler, _default_handler
                
  .weak      DMA2_Stream6_IRQHandler
  .thumb_set DMA2_Stream6_IRQHandler, _default_handler
                
  .weak      DMA2_Stream7_IRQHandler
  .thumb_set DMA2_Stream7_IRQHandler, _default_handler
                
  .weak      USART6_IRQHandler
  .thumb_set USART6_IRQHandler, _default_handler

  .weak      I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler, _default_handler

  .weak      I2C3_ER_IRQHandler   
  .thumb_set I2C3_ER_IRQHandler, _default_handler

  .weak      FPU_IRQHandler
  .thumb_set FPU_IRQHandler, _default_handler

  .weak      SPI4_IRQHandler
  .thumb_set SPI4_IRQHandler, _default_handlers

@----------------------------------------------
@ Default Handler Section
.section .text.default_handler, "ax", %progbits
	.type _default_handler, %function
  .weak _default_handler
	.global _default_handler

_default_handler:
  B       .                   @ Infinite loop for debugging
  .align  2
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
  
  @ check definition in include.asm
  DIS_OUTOFORDER_EXEC #1

  @ set stack alignement
  BL      stack_align_4
  
  @ Copy .kdata section from FLASH to SRAM
  LDR     r0, =_sikdata
  LDR     r1, =_skdata
  LDR     r2, =_ekdata
  1:
    LDR     r3, [r0], #4        @ Load word from .kdata Flash section
    CMP     r1, r2
    IT      NE
    STRNE   r3, [r1], #4        @ Store word in .kdata SRAM section
    BNE     1f

  @ Zero out .kbss section in SRAM
  LDR     r0, =_skbss
  LDR     r1, =_ekbss
  MOV     r2, #0
  2:
    CMP     r0, r1
    ITT     NE
    STRNE   r2, [r0], #4
    BNE     2f

  @ System initialization
  BL      _sysinit

  @ Copy .data section from FLASH to SRAM
  LDR     r0, =_sidata
  LDR     r1, =_sdata
  LDR     r2, =_edata
  3:
    LDR     r3, [r0], #4        @ Load word from .data Flash section
    CMP     r1, r2
    IT      NE
    STRNE   r3, [r1], #4        @ Store word in .data SRAM section
    BNE     3f
  
	@ Zero out .bss section in SRAM
  LDR     r0, =_sbss
  LDR     r1, =_ebss
  MOV     r2, #0
  4:
    CMP     r0, r1
    ITT     NE
    STRNE   r2, [r0], #4
    BNE     4f

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
  .align  2
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
  .align  2
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
  .align  2
  .size _SYSTICK_config, .-_SYSTICK_config

@ RCC register details provided in STM32F401's ref manual page 103
@-----------------------------------
@ main function called by system initialization to take care of clocks
@-----------------------------------
  .type _RCC_config, %function
_RCC_config:
  LDR     r0, =RCC_BASE
@ Enable the high speed internal osciallator
1:
  LDR     r1, [r0]        @ RCC_CR
  ORR     r1, r1, #0b01
  STR     r1, [r0]        @ RCC_CR
  @ Wait for the high speed internal oscillator to be anabled
2:
  LDR     r1, [r0]        @ RCC_CR
  TST     r1, #0b10          @ use previously shifted mask to check if HSIRDY
  BEQ     2f

@ Configure then enable the Phase locked loop
3:
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
4:
  LDR     r1, [r0]        @ RCC_CR
  TST     r1, r2          @ use previously shifted mask to check if PLLRDY
  BEQ     4f

@ Set PLL as sysclk input thru the provided multiplexer
5:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  BIC     r1, r1, #0b11      @ clear the bits
  ORR     r1, r1, #0b10
  STR     r1, [r0, #0x08] @ PLL_CFGR
  @ Wait for the multiplexer to select the PLL as SYSCLK input source
6:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  TST     r1, #0b100
  BEQ     6f

@ Set AHB APB1 APB2 prescalers
7:
  LDR     r1, [r0, #0x08] @ PLL_CFGR
  MOVW    r2, #0b1111     
  MOVW    r2, #0xE0F0     @ clear HPRE and PPRE2 bits (AHB & APB2 prescaler bits)
  BIC     r1, r1, r2
  MOVW    r2, #(0b100 << 10)@ set PPRE2 = /2 (APB1 prescaler)
  ORR     r1, r1, r2
  STR     r1, [r0, #0x08] @ PLL_CFGR

@ enable rcc interrupts
8:
  LDR     r1, [r0, #0x0C] @ RCC_CIR
  MOVW    r2, #(0b111111 << 8)  @ enable all RCC interrupts
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x0C] @ RCC_CIR

@ Enable AHB1 essencial peripherals clocks
9:
  LDR     r1, [r0, #0x30]   @ RCC_AHB1ENR
  MOVW    r2, #0b11         @ enable GPIOA GPIOB 
  MOVT    r2, #(0b11 << 5)  @ enable DMA1 DMA2
  ORR     r1, r1, r2
  LDR     r1, [r0, #0x30]   @ RCC_AHB1ENR
@ Enable APB1 essencial peripheral clocks
10:
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
  .align  2
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
  .align  2
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
  .align  2
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
1:
  LDR     r1, [r0]
  ORR     r1, r1, #0b11   @ reset both instruction and data caches
  STR     r1, [r0]        @ save changes
  MOVW    r2, #0x0702     @ enable caches, prefetch and set latency = 2Ws
  ORR     r1, r1, r2
  STR     r1, [r0]        @ save changes

2:
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
  .ifndef DEVELOPMENT_MODE
    BL       __FLASH_opt_config      @ configure FLASH option bytes
  .endif

  MOV     r0, #0      @ RETURN 0
  BX      lr
  .align  2
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
  POP     {pc}
  .align  2
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
1:
  LDR     r1, [r0]
  CMP     r2, #0
  IT      EQ
  BXEQ    lr            @ we return to system init
@ Configure the 8 sections for the system
@ (macro used is defined in src/common/macros.asm)
2:
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
3:
  LDR     r1, [r0, #0x04]       @ MPU_CTRL reg
  ORR     r1, r1, #0b111        @ Set ENABLE, PRIVDEFENA, and HFNMIENA bits
  STR     r1, [r0, #0x04]       @ MPU_CTRL reg
  MOV     r0, #0
  BX      lr
  .align  2
  .size _MPU_config, .-_MPU_config

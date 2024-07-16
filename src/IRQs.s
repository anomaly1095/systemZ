
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
.include "syscalls.s"

@----------------------------------------------------------------------
@----------------------------------------------------------------------
@---------------------------------------------------------------------- 
@ Interrupt Request handlers for exception handling are 
@ all executed in privileged handler mode

  .section .text.IRQs, "ax", %progbits

@ System exception handlers

  .global NMI_Handler
  .type NMI_Handler, %function
NMI_Handler:

  .global HardFault_Handler
  .type HardFault_Handler, %function
HardFault_Handler:

  .global MemManage_Handler
  .type MemManage_Handler, %function
MemManage_Handler:

  .global BusFault_Handler
  .type BusFault_Handler, %function
BusFault_Handler:

  .global UsageFault_Handler
  .type UsageFault_Handler, %function
UsageFault_Handler:

  .global SVC_Handler
  .type SVC_Handler, %function
SVC_Handler:
  @ All Syscalls are emplemented inside src/sys-calls.asm
  @ All the Syscalls either return 0 or the expected return value
  @ All function wrappers should expect an uint32_t return value
  @ r0, r1, r2, r3, r12, LR, PC, xPSR will all be saved automatically during context switch
  PUSH    {r4-r5, lr}       @ Save r4, r5, and lr to the kernel stack
  CPSID   I                 @ disable interrupts 
  MRS     r4, PSP           @ Get the address of the process stack pointer
  LDR     r4, [r4, #0x18]   @ Get the value of the PC saved on the process stack
  LDRB    r4, [r4, #-2]     @ Load the byte of the SVC instruction
  LDR     r5, =SVC_MASK 
  BIC     r4, r4, r5        @ Extract the immediate value from the SVC instruction

  @ Check which syscall to call
  CMP     r4, #0
  BEQ     _NVIC_enable_irq
  CMP     r4, #1
  BEQ     _NVIC_disable_irq
  CMP     r4, #2
  BEQ     _NVIC_set_pend_irq
  CMP     r4, #3
  BEQ     _NVIC_clear_pend_irq
  CMP     r4, #4
  BEQ     _NVIC_check_active_irq
  CMP     r4, #5
  BEQ     _NVIC_set_pri_irq
  CMP     r4, #6
  BEQ     _NVIC_get_pri_irq
  CMP     r4, #7
  BEQ     _NVIC_soft_trigger_irq
  CMP     r4, #8
  BEQ     _sbrk
  CMP     r4, #9
  BEQ     _sbrk_free

  MRS     r4, PSP           @ Get the address of the process stack pointer
  STR     r0, [r4]          @ Store the return value of the syscalls in the process stack
  CPSIE   I                 @ enable interrupts 
  POP     {r4-r5, pc}       @ Restore r4, r5, and set pc = lr
  .align  4
  .size SVC_Handler, .-SVC_Handler

  .global DebugMon_Handler
  .type DebugMon_Handler, %function
DebugMon_Handler:

  .global PendSV_Handler
  .type PendSV_Handler, %function
PendSV_Handler:


  .global SysTick_Handler
  .type SysTick_Handler, %function
SysTick_Handler:
  LDR     r0, =stk_cntrs
  LDRH    r1, [r0]            @ Load number of milliseconds
  LDR     r1, [r0, #0x02]     @ Load number of seconds
  ADD     r1, r1, #1
  
  @ add callbacks (use for os scheduler)
  PUSH    {r0-r3, lr}
  LDR     r0, =stk_clbk
  LDR     r1, [r0]
  BLX     r1
  LDR     r1, [r0, #0x04]
  BLX     r1
  LDR     r1, [r0, #0x08]
  BLX     r1
  LDR     r1, [r0, #0x0C]
  BLX     r1
  POP     {r0-r3, lr}

  CMP     r1, #1000
  ITT     EQ
  ADDEQ   r2, r2, #1          @ Store new number of seconds
  MOVEQ   r1, #0              @ reset millisecond counter
  STRH    r1, [r0]            @ Store new number of milliseconds
  STR     r2, [r0, #0x02]     @ Store new number of seconds
  BX      lr
  .align  4
  .size SysTick_Handler, .-SysTick_Handler

/* External Interrupts */
@ WWDG_IRQHandler
@ PVD_IRQHandler
@ TAMP_STAMP_IRQHandler
@ RTC_WKUP_IRQHandler
@ FLASH_IRQHandler
@ RCC_IRQHandler
@ EXTI0_IRQHandler
@ EXTI1_IRQHandler
@ EXTI2_IRQHandler
@ EXTI3_IRQHandler
@ EXTI4_IRQHandler
@ DMA1_Stream0_IRQHandler
@ DMA1_Stream1_IRQHandler
@ DMA1_Stream2_IRQHandler
@ DMA1_Stream3_IRQHandler
@ DMA1_Stream4_IRQHandler
@ DMA1_Stream5_IRQHandler
@ DMA1_Stream6_IRQHandler
@ ADC_IRQHandler
@ EXTI9_5_IRQHandler
@ TIM1_BRK_TIM9_IRQHandler
@ TIM1_UP_TIM10_IRQHandler
@ TIM1_TRG_COM_TIM11_IRQHandler
@ TIM1_CC_IRQHandler
@ TIM2_IRQHandler
@ TIM3_IRQHandler
@ TIM4_IRQHandler
@ I2C1_EV_IRQHandler
@ I2C1_ER_IRQHandler
@ I2C2_EV_IRQHandler
@ I2C2_ER_IRQHandler
@ SPI1_IRQHandler
@ SPI2_IRQHandler
@ USART1_IRQHandler
@ USART2_IRQHandler	  
@ EXTI15_10_IRQHandler
@ RTC_Alarm_IRQHandler
@ OTG_FS_WKUP_IRQHandler
@ DMA1_Stream7_IRQHandler
@ SDIO_IRQHandler
@ TIM5_IRQHandler
@ SPI3_IRQHandler
@ DMA2_Stream0_IRQHandler
@ DMA2_Stream1_IRQHandler
@ DMA2_Stream2_IRQHandler
@ DMA2_Stream3_IRQHandler
@ DMA2_Stream4_IRQHandler
@ OTG_FS_IRQHandler
@ DMA2_Stream5_IRQHandler 
@ DMA2_Stream6_IRQHandler 
@ DMA2_Stream7_IRQHandler 
@ USART6_IRQHandler
@ I2C3_EV_IRQHandler
@ I2C3_ER_IRQHandler
@ FPU_IRQHandler
@ SPI4_IRQHandler


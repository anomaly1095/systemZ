      
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
  #include "../../common/define.asm"
  #include "../../common/macros.asm"

@ NVIC register details provided in stm32-cortex-M4 Referance Manual page 208

.section .text.drivers.NVIC, "ax", %progbits

@-----------------------------------
@ main function called by system initialization to configure the NVIC
@-----------------------------------
  .global _NVIC_config
  .type _NVIC_config, %function
_NVIC_config:
  PUSH    {lr}
  @ enable RCC / FLASH / FPU
  LDR     r0, =NVIC_ISER0
  LDR     r1, [r0]              @ NVIC_ISER0
  MOV     r2, #(0b11 << 4)      @ set FLASH and RCC bits
  ORR     r1, r1, r2
  STR     r1, [r0]              @ NVIC_ISER0
  LDR     r1, [r0, #0x08]       @ NVIC_ISER2
  MOV     r2, #(0b1 << 4)       @ set FPU bit
  ORR     r1, r1, r2
  STR     r1, [r0, #0x08]       @ NVIC_ISER2
  POP     {lr}
  MOV     r0, #0
  BX      lr
  .align 4
  .size _NVIC_config, .-_NVIC_config


@-----------------------------------
@ syscall used by apps (called by SVC)
@ called by software to enable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_enable_irq, %function
_NVIC_enable_irq:

  .align 4
  .size _NVIC_enable_irq, .-_NVIC_enable_irq

@-----------------------------------
@ syscall used by apps (called by SVC)
@ called by software to disable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_disable_irq, %function
_NVIC_disable_irq:

  .align 4
  .size _NVIC_disable_irq, .-_NVIC_disable_irq

@-----------------------------------
@ syscall used by apps (called by SVC)
@ called by software to set an interrupt as pending
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_set_pend_irq, %function
_NVIC_set_pend_irq:

  .align 4
  .size _NVIC_set_pend_irq, .-_NVIC_set_pend_irq

@-----------------------------------
@ syscall used by apps (called by SVC)
@ called by software to remove an interrupt from pending list
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_clear_pend_irq, %function
_NVIC_clear_pend_irq:


  .align 4
  .size _NVIC_clear_pend_irq, .-_NVIC_clear_pend_irq

@-----------------------------------
@ syscall used by apps (called by SVC)
@ called by software to check interrupt if the interrupt is active
@ arg0: number of the IRQ (0..239)
@ return: 1 if active / 0 if idle
@-----------------------------------
  .type _NVIC_check_active_irq, %function
_NVIC_check_active_irq:


  .align 4
  .size _NVIC_check_active_irq, .-_NVIC_check_active_irq
  
@-----------------------------------
@ syscall used by apps (called by SVC)
@ called by software to set the priority of the interrupt
@ arg0: number of the IRQ (0..239)
@ arg1: priority number
@-----------------------------------
  .type _NVIC_set_pri_irq, %function
_NVIC_set_pri_irq:


  .align 4
  .size _NVIC_set_pri_irq, .-_NVIC_set_pri_irq
  
@-----------------------------------
@ syscall used by apps
@ access to this register can be thru unpriviledged thread mode
@ check SCR reg in page 230 of the stm32-cortex-M4 Referance Manual
@ called by software to trigger an interrupt on the mask specified in arg0
@-----------------------------------
  .type _NVIC_soft_trigger_irq, %function
_NVIC_soft_trigger_irq:


  .align 4
  .size _NVIC_soft_trigger_irq, .-_NVIC_soft_trigger_irq


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
.include "src/include.s"

@----------------------------------------------------------------------
@----------------------------------------------------------------------
@----------------------------------------------------------------------ISR handlers
@----------------------------------------------------------------------
@---------------------------------------------------------------------- 
  .section .text.system, "ax", %progbits


  .global NMI_Handler
  .type NMI_Handler, %function
NMI_Handler:

@---------------------------------------------------------------------- 

  .global HardFault_Handler
  .type HardFault_Handler, %function
HardFault_Handler:

@---------------------------------------------------------------------- 
  .global MemManage_Handler
  .type MemManage_Handler, %function
MemManage_Handler:

@---------------------------------------------------------------------- 
  .global BusFault_Handler
  .type BusFault_Handler, %function
BusFault_Handler:

@---------------------------------------------------------------------- 
  .global UsageFault_Handler
  .type UsageFault_Handler, %function
UsageFault_Handler:


@---------------------------------------------------------------------- 
  .global SVC_Handler
  .type SVC_Handler, %function
SVC_Handler:
  PUSH    {r4-r5, lr}           @ Save r4, r5, and lr to the kernel stack
  MRS     r4, PSP               @ Get the address of the process stack pointer
  LDR     r4, [r4, #0x18]       @ Get the value of the PC saved on the process stack
  LDRB    r4, [r4, #-2]         @ Load the byte of the SVC instruction
  AND     r4, r4, #0xFF         @ Mask to get the immediate value (0x00FF)
  LDR     r5, =SVC_Table        @ Load the address of the SVC table
  LDR     r4, [r5, r4, LSL #2]  @ Get the address of the SVC handler from the table
  BLX     r4                    @ Branch to the SVC handler

  MRS     r4, PSP               @ Get the address of the process stack pointer
  STR     r0, [r4]              @ Store the return value in the process stack
  POP     {r4-r5, pc}           @ Restore r4, r5, and set pc = lr

  @-------NVIC--------@
  SVC0_Handler:
    _NVIC_enable_irq
    BX      lr

  SVC1_Handler:
    _NVIC_disable_irq
    BX      lr

  SVC2_Handler:
    _NVIC_set_pend_irq
    BX      lr

  SVC3_Handler:
    _NVIC_clear_pend_irq
    BX      lr

  SVC4_Handler:
    _NVIC_check_active_irq
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC5_Handler:
    _NVIC_set_prio_irq
    BX      lr

  SVC6_Handler:
    _NVIC_get_prio_irq
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC7_Handler:
    _NVIC_soft_trigger_irq
    BX      lr

  @-------Memory management--------@
  SVC8_Handler:
    _sbrk
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC9_Handler:
    _sbrk_free
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC10_Handler:
    _malloc
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC11_Handler:
    _free
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC12_Handler:
  
    BX      lr

  SVC13_Handler:
    BX      lr

  @-------SCB--------@

  SVC14_Handler:
    _DIS_OUTOFORDER_EXEC #1
    BX      lr

  SVC15_Handler:
    _DIS_OUTOFORDER_EXEC #0
    BX      lr

  SVC16_Handler:
    _GET_CPUID
    BX      lr

  SVC17_Handler:
    _NMI_set_pending
    BX      lr

  SVC18_Handler:
    _PENDSV_set_pending
    BX      lr

  SVC19_Handler:
    _PENDSV_clear_pending
    BX      lr

  SVC20_Handler:
    _SYSTICK_set_pending
    BX      lr

  SVC21_Handler:
    _SYSTICK_clear_pending
    BX      lr

  SVC22_Handler:
    _SYSTICK_check_pending
    BX      lr

  SVC23_Handler:
    _ISR_check_pending
    BX      lr

  SVC24_Handler:
    _prio_split16_0
    BX      lr

  SVC25_Handler:
    _prio_split8_2
    BX      lr

  SVC26_Handler:
    _prio_split4_4
    BX      lr

  SVC27_Handler:
    _prio_split2_8
    BX      lr

  SVC28_Handler:
    _prio_split0_16
    BX      lr

  SVC29_Handler:
    _reset_request
    BX      lr

  SVC30_Handler:
    _sev_on_pend
    BX      lr

  SVC31_Handler:
    _sleep_deep
    BX      lr

  SVC32_Handler:
    _sleep_on_exit
    BX      lr

  SVC33_Handler:
    _stack_align_4
    BX      lr

  SVC34_Handler:
    _stack_align_8
    BX      lr

  SVC35_Handler:
    _NMI_HARDFAULT_dis_bus_fault_handling
    BX      lr

  SVC36_Handler:
    _NMI_HARDFAULT_en_bus_fault_handling
    BX      lr

  SVC37_Handler:
    _div0_notrap
    BX      lr

  SVC38_Handler:
    _div0_trap
    BX      lr

  SVC39_Handler:
    _unalign_notrap
    BX      lr

  SVC40_Handler:
    _unalign_trap
    BX      lr

  SVC41_Handler:
    _app_access_STIR
    BX      lr

  SVC42_Handler:
    _no_enter_thread_mode_on_active_exc
    BX      lr
  
  SVC43_Handler:
    _enter_thread_mode_on_active_exc
    BX      lr
  
  SVC44_Handler:
    _set_UsageFault_prio 
    BX      lr
    
  SVC45_Handler:
    _set_MemMan_fault_prio
    BX      lr

  SVC46_Handler:
    _set_SVC_prio
    BX      lr

  SVC47_Handler:
    _set_SYSTICK_prio
    BX      lr

  SVC48_Handler:
    _set_PendSV_prio
    BX      lr

  SVC49_Handler:
    _enable_UsageFault
    BX      lr

  SVC50_Handler:
    _enable_BusFault
    BX      lr

  SVC51_Handler:
    _enable_MemMan_fault
    BX      lr

  SVC52_Handler:
    _disable_UsageFault
    BX      lr

  SVC53_Handler:
    _disable_BusFault
    BX      lr

  SVC54_Handler:
    _disable_MemMan_fault
    BX      lr

  SVC55_Handler:
    _is_SVC_pending
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC56_Handler:
    _is_BusFault_pending
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC57_Handler:
    _is_MemMan_fault_pending
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC58_Handler:
    _is_UsageFault_pending
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC59_Handler:
    _is_SYSTICK_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC60_Handler:
    _is_PendSV_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC61_Handler:
    _is_DBGMon_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC62_Handler:
    _is_SVC_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC63_Handler:
    _is_UsageFault_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC64_Handler:
    _is_BusFault_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC65_Handler:
    _is_MemMan_fault_active
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC66_Handler:
    _div_by0_UsageFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC67_Handler:
    _unalignement_UsageFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC68_Handler:
    _coprocessor_UsageFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC69_Handler:
    _invPC_UsageFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC70_Handler:
    _invEPSR_UsageFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC71_Handler:
    _BFAR_valid_addr
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC72_Handler:
    _FP_LazyState_BusFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC73_Handler:
    _push_BusFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC74_Handler:
    _pop_BusFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC75_Handler:
    _imprecise_BusFault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC76_Handler:
    _precise_DBus_error
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC77_Handler:
    _IBus_error
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC78_Handler:
    _MMAR_valid_addr
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  SVC79_Handler:
    _FP_LazyState_MemMan_fault
    STORE_R0_TO_PSP   @ Store r0 value for return
    BX      lr

  @ SVC80_Handler:
  @   _push_MemMan_fault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC81_Handler:
  @   _pop_MemMan_fault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC82_Handler:
  @   _DataAccess_MemMan_fault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC83_Handler:
  @   _ExecNot_section_MemMan_fault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC84_Handler:
  @   _forced_HardFault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC85_Handler:
  @   _push_MemMan_fault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC86_Handler:
  @   _vect_table_HardFault
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC87_Handler:
  @   _get_MemManFault_addr
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC88_Handler:
  @   _get_BusFault_addr
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC89_Handler:
  @   _get_AuxFault_addr
  @   STORE_R0_TO_PSP   @ Store r0 value for return
  @   BX      lr

  @ SVC90_Handler:
  @   Check if IRQ number is valid
  @   CMP     r0, #85               @ Compare with the maximum IRQ number (84)
  @   IT      HS
  @   BXHS    lr                    @ Branch if the IRQ number is out of range

  @   LDR     r3, =IRQ_head_nodes   @ Load address of IRQ_head_nodes
  @   LDR     r0, [r3, r0, LSL #3]  @ Load the head node pointer (8 bytes per node)

  @   _add_callback                 @ Call the macro to add the node

  @   BX      lr                    @ Return

  @ SVC91_Handler:
  @   Check if IRQ number is valid
  @   CMP     r0, #85               @ Compare with the maximum IRQ number (84)
  @   IT      HS
  @   BXHS    lr                    @ Branch if the IRQ number is out of range

  @   LDR     r3, =IRQ_head_nodes   @ Load address of IRQ_head_nodes
  @   LDR     r0, [r3, r0, LSL #3]  @ Load the head node pointer (8 bytes per node)

  @   _rem_callback                 @ Call the macro to remove the node

  @   BX      lr                    @ Return

  SVC92_Handler:
    NOP
    BX      lr

  SVC93_Handler:
    NOP
    BX      lr

  SVC94_Handler:
    NOP
    BX      lr

  SVC95_Handler:
    NOP
    BX      lr

  SVC96_Handler:
    NOP
    BX      lr

  SVC97_Handler:
    NOP
    BX      lr

  SVC98_Handler:
    NOP
    BX      lr

  SVC99_Handler:
    NOP
    BX      lr


  .align  2
  .size SVC_Handler, .-SVC_Handler


@---------------------------------------------------------------------- 
  .global DebugMon_Handler
  .type DebugMon_Handler, %function
DebugMon_Handler:

@---------------------------------------------------------------------- 
  .global PendSV_Handler
  .type PendSV_Handler, %function
PendSV_Handler:


@---------------------------------------------------------------------- 
  .global SysTick_Handler
  .type SysTick_Handler, %function
SysTick_Handler:
  ENTER_CRITICAL                    @ enble interrupts
  LDR     r0, =SYSTICK_cntrs
  LDRH    r1, [r0, #0x04]           @ Load number of milliseconds
  LDR     r2, [r0]                  @ Load number of seconds
  ADD     r1, r1, #1                @ add 1 millisecond

  @ add functions to call (use for os scheduler)

  CMP     r1, #1000
  ITT     EQ
  ADDEQ   r2, r2, #1          @ Store new number of seconds
  MOVEQ   r1, #0              @ reset millisecond counter
  STRH    r1, [r0, #0x04]     @ Store new number of milliseconds
  STR     r2, [r0]            @ Store new number of seconds
  EXIT_CRITICAL               @ enable interrupts
  BX      lr
  .align  2
  .size SysTick_Handler, .-SysTick_Handler


@---------------------------------------------------------------------- 
@---------------------------------------------------------------------- 

@ dont forget to make each isr check for it's callback linked list during emplementation 
@ dont forget to go back to unprivileged mode before going to the callback for security reasons

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

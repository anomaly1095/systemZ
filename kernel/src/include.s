
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

#include "src/data.s"

.section .text.system, "ax", %progbits

/**
  * Functions that will be rarely used or get called by 
  * the SVC handler are written as macro 
  * Functions Frequently used like memory management and used by the kernel 
  * will be normal branches and called directly by kernel code
  */

@-----------------------------------------------------
@-----------------------------------------------------
@-----------------------------------------------------NVIC 
@-----------------------------------------------------
@----------------------------------------------------- 

#define LITTLE_ENDIAN

@ used by FLASH set_options function
#define DEVELOPMENT_MODE 

.macro ENTER_CRITICAL
  CPSID I
.endm

.macro EXIT_CRITICAL
  CPSIE I
.endm

.macro STORE_R0_TO_PSP
  MRS     r2, PSP                  @ Get the address of the process stack pointer
  STR     r0, [r2, #0]             @ Store the value of r0 at the top of the process stack
.endm

.macro LOAD_R0_FROM_PSP
  MRS     r2, PSP                  @ Get the address of the process stack pointer
  LDR     r0, [r2, #0]             @ Load the value of r0 from the top of the process stack
.endm

.macro MPU_CONFIG_REGION region_base:req, region_number:req, region_mask:req
  @ Enter critical section to ensure exclusive access to MPU registers
  ENTER_CRITICAL
  
  LDR     r0, =MPU_BASE              @ Load the base address of the MPU
  LDR     r1, =\region_base          @ Load the base address for the MPU region
  MOVW    r2, #(0b10000 | \region_number) @ Configure the region number and VALID bit
  ORR     r1, r1, r2                @ Combine base address with the region number
  @ Ensure all previous memory accesses are complete
  DSB
  ISB

  STR     r1, [r0, #0x0C]           @ Write to the MPU_RBAR register
  LDR     r2, =\region_mask          @ Load the region attributes mask
  STR     r2, [r0, #0x10]           @ Write to the MPU_RASR register
  
  @ Ensure all MPU configuration is complete before exiting critical section
  DSB
  ISB
  
  @ Exit critical section
  EXIT_CRITICAL
.endm



@-----------------------------------
@ Macro used to select which register to select in the NVIC (0..7)
@ Applies on NVIC_ISER, NVIC_ICER, NVIC_ISPR, NVIC_ICPR, NVIC_IABR
@ arg0: interrupt position (irq_num)
@ arg1: address of register 0 (NVIC0_addr)
@ return: sets address of register to work on in r2 and normalizes irq_num in r0
@ example: NVIC_REG_SELECT7 5, 0xE000E100 
@-----------------------------------
.macro NVIC_REG_SELECT7 irq_num:req, NVIC0_addr:req
  LDR     r2, =\NVIC0_addr         @ Load the base address into r2

  CMP     \irq_num, #31            @ Compare irq_num with 31
  IT      GT                       @ If irq_num > 31, then...
  ADDGT   r2, r2, #0x04            @ Adjust address if irq_num > 31

  CMP     \irq_num, #63            @ Compare irq_num with 63
  ITTE    GT                       @ If irq_num > 63, then...
  ADDGT   r2, r2, #0x04            @ Adjust address if irq_num > 63
  SUBGT   \irq_num, \irq_num, #64  @ Normalize the bit offset in irq_num to start at 0 if irq_num > 63
  SUBLE   \irq_num, \irq_num, #32  @ Normalize the bit offset in irq_num to start at 0 if irq_num <= 63
.endm


@-----------------------------------
@ Macro used to select which register to select in the NVIC (0..59)
@ Applies on NVIC_IPR
@ arg0: interrupt position (irq_num)
@ arg1: base address of NVIC_IPR registers (NVIC0_IPR_addr)
@ return: sets the address of the register to work on in r2 and normalizes irq_num in \irq_num
@-----------------------------------
.macro NVIC_REG_SELECT59 irq_num:req, NVIC0_IPR_addr:req
  LDR     r2, =\NVIC0_IPR_addr     @ Load the base address into r2
  LSR     \irq_num, \irq_num, #2   @ Divide irq_num by 4 to get the byte offset
  ADD     r2, r2, \irq_num         @ Add the offset to the base address
.endm


@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to enable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_enable_irq
  @ Enter critical section to ensure exclusive access to NVIC registers
  ENTER_CRITICAL
  
  @ Macro sets the address of the register in r2
  @ Normalizes the IRQ number in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ISER0  @ Select the appropriate NVIC_ISER register
  
  @ Create a mask for the IRQ bit position
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ Shift the mask to the IRQ bit position
  
  @ Load the current value of the NVIC_ISER register
  LDR     r1, [r2]
  
  @ Set the bit for the IRQ
  ORR     r1, r1, r3
  
  @ Ensure all previous memory accesses are complete
  DSB
  ISB

  @ Store the updated value back to the NVIC_ISER register
  STR     r1, [r2]
  
  @ Ensure all NVIC updates are complete before exiting critical section
  DSB
  ISB
  
  @ Exit critical section
  EXIT_CRITICAL
.endm

  
@-----------------------------------
@ Macro used by apps (called by SVC)
@ called by software to disable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_disable_irq
  @ Enter critical section to ensure exclusive access to NVIC registers
  ENTER_CRITICAL
  
  @ Macro sets the address of the register in r2
  @ Normalizes the irq_num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ICER0   @ Select the appropriate NVIC_ICER register
  
  @ Create a mask for the IRQ bit position
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ Shift the mask to the IRQ bit position
  
  @ Load the current value of the NVIC_ICER register
  LDR     r1, [r2]
  
  @ Set the bit to disable the IRQ
  ORR     r1, r1, r3
  
  @ Ensure all previous memory accesses are complete
  DSB
  ISB

  @ Store the updated value back to the NVIC_ICER register
  STR     r1, [r2]
  
  @ Ensure all NVIC updates are complete before exiting critical section
  DSB
  ISB
  
  @ Exit critical section
  EXIT_CRITICAL
.endm


@-----------------------------------
@ Macro used by apps (called by SVC)
@ called by software to set an interrupt as pending
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_set_pend_irq
  @ Enter critical section to ensure exclusive access to NVIC registers
  ENTER_CRITICAL
  
  @ Macro sets the address of the register in r2
  @ Normalizes the irq_num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ISPR0   @ Select the appropriate NVIC_ISPR register
  
  @ Create a mask for the IRQ bit position
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ Shift the mask to the IRQ bit position
  
  @ Load the current value of the NVIC_ISPR register
  LDR     r1, [r2]
  
  @ Set the bit to make the IRQ pending
  ORR     r1, r1, r3
  
  @ Ensure all previous memory accesses are complete
  DSB
  ISB

  @ Store the updated value back to the NVIC_ISPR register
  STR     r1, [r2]
  
  @ Ensure all NVIC updates are complete before exiting critical section
  DSB
  ISB
  
  @ Exit critical section
  EXIT_CRITICAL
.endm


@-----------------------------------
@ Macro used by apps (called by SVC)
@ called by software to remove an interrupt from the pending list
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_clear_pend_irq
  @ Enter critical section to ensure exclusive access to NVIC registers
  ENTER_CRITICAL

  @ Macro sets the address of the register in r2
  @ Normalizes the irq_num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ICPR0   @ Select the appropriate NVIC_ICPR register
  
  @ Create a mask for the IRQ bit position
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ Shift the mask to the IRQ bit position
  
  @ Load the current value of the NVIC_ICPR register
  LDR     r1, [r2]
  
  @ Set the bit to clear the IRQ from pending
  ORR     r1, r1, r3
  
  @ Ensure all previous memory accesses are complete
  DSB
  ISB

  @ Store the updated value back to the NVIC_ICPR register
  STR     r1, [r2]
  
  @ Ensure all NVIC updates are complete before exiting critical section
  DSB
  ISB
  
  @ Exit critical section
  EXIT_CRITICAL
.endm


@-----------------------------------
@ SYSCALL
@ Used by apps (called by SVC)
@ Checks if the interrupt is active
@ arg0: number of the IRQ (0..239)
@ return: 1 if active / 0 if idle
@-----------------------------------
.macro _NVIC_check_active_irq
  @ Enter critical section to ensure exclusive access to NVIC registers
  ENTER_CRITICAL

  @ Macro sets the address of the register in r2
  @ Normalizes the irq_num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_IABR0   @ Select the appropriate NVIC_IABR register

  @ Shift the mask to the IRQ bit position
  MOV     r3, #0b1
  LSL     r3, r3, r0

  @ Load the value of the NVIC_IABR register
  LDR     r1, [r2]

  @ Check if bit is set (IRQ active)
  TST     r1, r3
  ITE     NE
  MOVNE   r0, #1                @ Bit is set (IRQ active)
  MOVEQ   r0, #0                @ Bit is not set (IRQ idle)
  
  @ Exit critical section
  EXIT_CRITICAL
.endm

@-----------------------------------
@ SYSCALL
@ Used by apps (called by SVC)
@ Sets the priority of the interrupt
@ arg0: number of the IRQ (0..239)
@ arg1: priority number
@-----------------------------------
.macro _NVIC_set_prio_irq
  @ Enter critical section to ensure exclusive access to NVIC registers
  ENTER_CRITICAL

  @ Select the appropriate NVIC_IPR register
  NVIC_REG_SELECT59 r0, NVIC_IPR0   @ Select the appropriate NVIC_IPR register
  
  @ Ensure all previous memory accesses are complete
  DSB
  ISB

  @ Store the priority number in the selected register byte
  STRB    r1, [r2]
  
  @ Ensure all NVIC updates are complete before exiting critical section
  DSB
  ISB
  
  @ Exit critical section
  EXIT_CRITICAL
.endm


@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to get the priority of the interrupt
@ arg0: number of the IRQ (0..239)
@ return: priority number of the IRQ
@-----------------------------------
.macro _NVIC_get_prio_irq
  NVIC_REG_SELECT59 r0, NVIC_IPR0   @ Select the appropriate NVIC_IPR register
  LDRB    r0, [r2]                    @ Load the priority number from the selected register byte
.endm

@-----------------------------------SYSCALL
@ function used directly by apps or thru syscall
@ access to this register can be thru unpriviledged thread mode
@ check SCR reg in page 230 of the stm32-cortex-M4 Referance Manual
@ called by software to trigger an interrupt on the mask specified in arg0
@ arg0: IRQ number (0..239)
@-----------------------------------
.macro _NVIC_soft_trigger_irq
  LDR     r1, =NVIC_STIR
  STR     r0, [r1]
.endm

@-----------------------------------------------------
@-----------------------------------------------------
@----------------------------------------------------- SCB 
@-----------------------------------------------------
@-----------------------------------------------------

/*--------ACTLR---------*/
/*--------ACTLR---------*/

.macro _DIS_OUTOFORDER_EXEC state:req
  ENTER_CRITICAL
  LDR     r0, =ACTLR
  LDR     r1, [r0]
  LDR     r2, =\state
  CBZ     r2, 1f   @ check if state is 0 (enable)
  @ Disable features: Set bits
  MOVW    r2, #0x207
  ORR     r1, r1, r2  @ mask for disabling features
  B       2f
  1:
  @ Enable features: Clear bits
  MOVW    r2, #0x207
  BIC     r1, r1, r2 @ mask for enabling features
  2:
  DSB                 @ Data Synchronization Barrier
  ISB                 @ Instruction Synchronization Barrier
  STR     r1, [r0]    @ write back the modified value to ACTLR
  DSB                 @ Ensure the write is complete before continuing
  ISB                 @ Ensure the new instructions are fetched correctly
  EXIT_CRITICAL
.endm

/*--------CPUID---------*/
/*--------CPUID---------*/

@ returns CPUID in r0
.macro _GET_CPUID
  LDR     r0, =CPUID
  LDR     r0, [r0]
.endm


/*--------ICSR---------*/
/*--------ICSR---------*/

.macro SET_PENDING_BIT bit_mask:req
  ENTER_CRITICAL
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  ORR     r1, r1, #\bit_mask    @ Set the specified pending bit
  STR     r1, [r0]         @ Write back the modified value to ICSR
  EXIT_CRITICAL
.endm

.macro CLEAR_PENDING_BIT bit_mask:req
  ENTER_CRITICAL
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  BIC     r1, r1, #\bit_mask    @ Clear the specified pending bit
  STR     r1, [r0]         @ Write back the modified value to ICSR
  EXIT_CRITICAL
.endm

.macro CHECK_PENDING_BIT bit_mask:req
  ENTER_CRITICAL
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  MOV     r0, #0
  TST     r1, #\bit_mask        @ Check the specified pending bit
  IT      NE
  MOVNE   r0, #1
  EXIT_CRITICAL
.endm


.macro _NMI_set_pending
  SET_PENDING_BIT 0x80000000 @ Set the NMI pending bit (bit 28)
.endm

.macro _PENDSV_set_pending
  SET_PENDING_BIT 0x10000000 @ Set the PENDSV pending bit (bit 28)
.endm

.macro _PENDSV_clear_pending
  SET_PENDING_BIT 0x8000000 @ Clear the PENDSV pending bit (bit 27)
.endm

.macro _PENDSV_check_pending
  CHECK_PENDING_BIT 0x10000000 @ Check the PENDSV pending bit (bit 28)
.endm

.macro _SYSTICK_set_pending
  SET_PENDING_BIT 0x4000000 @ Set the SYSTICK pending bit (bit 26)
.endm

.macro _SYSTICK_clear_pending
  SET_PENDING_BIT 0x2000000 @ Clear the SYSTICK pending bit (bit 25)
.endm

.macro _SYSTICK_check_pending
  CHECK_PENDING_BIT 0x4000000 @ Check the SYSTICK pending bit (bit 26)
.endm

.macro _ISR_check_pending
  CHECK_PENDING_BIT 0x400000 @ Check if any ISR is pending excluding NMI and Faults.
.endm

.macro _ISR_highest_pending
  ENTER_CRITICAL                 @ Disable interrupts
  LDR     r0, =ICSR         @ Load the address of ICSR
  LDR     r1, [r0]          @ Load the current value of ICSR
  MOVW    r2, #0xF000       @ bottom 4 bits of mask for VECTPENDING
  MOVT    r2, #7            @ top 3 bits of mask
  AND     r0, r1, r2        @ Check the specified pending bit
  EXIT_CRITICAL                 @ Enable interrupts
.endm

.macro _ISR_check_preempted
  CHECK_PENDING_BIT 0x800   @ checks if there are any preempted interrupt routines
.endm

.macro _ISR_str_actv_num
  ENTER_CRITICAL
  MRS     r0, IPSR                  @ Read IPSR into r0 (contains active exception number)
  MOV     r1, r0                    @ Move exception number to r1 (to ensure 8-bit value)
  LDRB    r0, =last_IRQ              @ load the address of the byte thjat will contain the last ISR num
  STRB    r1, r0                    @ Store active exception number in global byte variable
  EXIT_CRITICAL
.endm

/*--------AIRCR---------*/
/*--------AIRCR---------*/

@ Macro used in SCB manip to set the binary point separation
@ that allows the config of number of groups prio and subgroup prio
.macro SET_GRP_SPLIT grp_prios:req
  ENTER_CRITICAL                  @ Disable interrupts
  LDR     r0, =AIRCR         @ Load address of AIRCR register
  LDR     r1, [r0]           @ Load current value of AIRCR
  BIC     r1, r1, #0x0700    @ Clear old PRIGROUP bits
  MOV     r2, #\grp_prios    @ Store the input priority grouping value in r2
  
  @ Check different priority grouping configurations and set the corresponding PRIGROUP bits
  CMP     r2, #0
  IT      EQ
  ORREQ   r1, r1, #0x700      @ Set new PRIGROUP bits: 0 groups - 16 subgroups
  BEQ     1f

  CMP     r2, #2
  IT      EQ
  ORREQ   r1, r1, #0x600      @ Set new PRIGROUP bits: 2 groups - 8 subgroups
  BEQ     1f

  CMP     r2, #4
  IT      EQ
  ORREQ   r1, r1, #0x500      @ Set new PRIGROUP bits: 4 groups - 4 subgroups
  BEQ     1f
  
  CMP     r2, #8
  IT      EQ
  ORREQ   r1, r1, #0x400      @ Set new PRIGROUP bits: 8 groups - 2 subgroups
  BEQ     1f

  @ Default: 16 groups - 0 subgroups (no additional MOV needed)

  1:
    MOVT    r1, #0x5FA         @ Write key to allow write access to AIRCR
    STR     r1, [r0]           @ Store modified value back to AIRCR
    EXIT_CRITICAL                  @ Enable interrupts
.endm

@ Set new PRIGROUP bits: 0 groups - 16 subgroups
.macro _prio_split16_0
  SET_GRP_SPLIT 16
.endm
@ Set new PRIGROUP bits: 2 groups - 8 subgroups
.macro _prio_split8_2
  SET_GRP_SPLIT 8
.endm
@ Set new PRIGROUP bits: 4 groups - 4 subgroups
.macro _prio_split4_4
  SET_GRP_SPLIT 4
.endm
@ Set new PRIGROUP bits: 4 groups - 4 subgroups
.macro _prio_split2_8
  SET_GRP_SPLIT 2
.endm
@ Set new PRIGROUP bits: 8 groups - 2 subgroups
.macro _prio_split0_16
  SET_GRP_SPLIT 0
.endm


@ Generic macro to manipulate a register using BIC or ORR operations
@ Args:
@   reg: Register to manipulate
@   mask: Bitmask to apply
@   operation: Operation to perform (BIC or ORR)
.macro manipulate_register reg:req, mask:req, operation:req
  ENTER_CRITICAL
  LDR     r0, =\reg       @ Load address of the register
  LDR     r1, [r0]        @ Load current value from the register
  \operation r1, r1, #\mask   @ Perform specified operation with the bitmask
  STR     r1, [r0]        @ Store modified value back to the register
  EXIT_CRITICAL
.endm

@ Macro to set the reset request in AIRCR
.macro _reset_request
  manipulate_register AIRCR, 0x4, ORR   @ Set bit 2 in AIRCR (Reset Request)
.endm


/*--------SCR---------*/
/*--------SCR---------*/

@ Macro to set the SEVONPEND bit in SCR
.macro _sev_on_pend
  manipulate_register SCR, 0b10000, ORR    @ Set SEVONPEND bit in SCR
.endm

@ Macro to set the SLEEPDEEP bit in SCR
.macro _sleep_deep
  manipulate_register SCR, 0b100, ORR      @ Set SLEEPDEEP bit in SCR
.endm

@ Macro to set the SLEEPONEXIT bit in SCR
.macro _sleep_on_exit
  manipulate_register SCR, 0b10, ORR       @ Set SLEEPONEXIT bit in SCR
.endm

/*--------CCR---------*/
/*--------CCR---------*/

@ Macro to clear stack alignment to 4 bytes aligned in CCR
.macro _stack_align_4
  manipulate_register CCR, 0x200, BIC
.endm

@ Macro to set stack alignment to 8 bytes aligned in CCR
.macro _stack_align_8
  manipulate_register CCR, 0x200, ORR
.endm

@ Macro to Disable NMI and HardFault handlers to ignore bus faults caused by load and store instructions
.macro _NMI_HARDFAULT_dis_bus_fault_handling
  manipulate_register CCR, 0x2000000, BIC
.endm

@ Macro to enable NMI and HardFault handlers to ignore bus faults caused by load and store instructions

.macro _NMI_HARDFAULT_en_bus_fault_handling
  manipulate_register CCR, 0x2000000, ORR
.endm

@ Macro to disable trapping on division by zero in CCR
.macro _div0_notrap
  manipulate_register CCR, 0b10000, BIC
.endm

@ Macro to enable trapping on division by zero in CCR
.macro _div0_trap
  manipulate_register CCR, 0b10000, ORR
.endm

@ Macro to disable trapping on unaligned halfword and word accesses in CCR
.macro _unalign_notrap
  manipulate_register CCR, 0b1000, BIC
.endm

@ Macro to enable trapping on unaligned halfword and word accesses in CCR
.macro _unalign_trap
  manipulate_register CCR, 0b1000, ORR
.endm

@ Macro to allow access to the STIR register in CCR
.macro _app_access_STIR
  manipulate_register CCR, 0b10, ORR
.endm

@ Macro to disable returning to thread mode when some exceptions are active
.macro _no_enter_thread_mode_on_active_exc
  manipulate_register CCR, 0b1, BIC
.endm

@ Macro to enable returning to thread mode from any level under an EXC_RETURN in CCR
.macro _enter_thread_mode_on_active_exc
  manipulate_register CCR, 0b1, ORR
.endm


/*--------SHPRx---------*/
/*--------SHPRx---------*/

@ Macro to set priority for various system exceptions
.macro set_exception_priority reg:req, bit_offset:req
  ENTER_CRITICAL                  @ Disable interrupts to ensure atomic operation
  LDR     r1, =\reg               @ Load the address of the priority register
  LDR     r2, [r1]                @ Load the current value of the register
  BIC     r2, r2, #(0xFF << \bit_offset)  @ Clear the current priority bits
  ORR     r2, r2, r0, LSL \bit_offset     @ Set new priority bits from r0
  STR     r2, [r1]                @ Store the new value back into the register
  EXIT_CRITICAL                   @ Enable interrupts after operation
.endm

@ Macro to set Usage Fault priority in SHPR1
.macro _set_UsageFault_prio
  set_exception_priority SHPR1, 20        @ Set Usage Fault priority (bit offset 20)
.endm

@ Macro to set Bus Fault priority in SHPR1
.macro _set_BusFault_prio
  set_exception_priority SHPR1, 12        @ Set Bus Fault priority (bit offset 12)
.endm

@ Macro to set Memory Management Fault priority in SHPR1
.macro _set_MemMan_fault_prio
  set_exception_priority SHPR1, 4         @ Set Memory Management Fault priority (bit offset 4)
.endm

@ Macro to set SVCall priority in SHPR2
.macro _set_SVC_prio 
  set_exception_priority SHPR2, 28        @ Set SVCall priority (bit offset 28)
.endm

@ Macro to set SysTick priority in SHPR3
.macro _set_SYSTICK_prio
  set_exception_priority SHPR3, 28        @ Set SysTick priority (bit offset 28)
.endm

@ Macro to set PendSV priority in SHPR3
.macro _set_PendSV_prio
  set_exception_priority SHPR3, 20        @ Set PendSV priority (bit offset 20)
.endm


/*--------SHCSR---------*/
/*--------SHCSR---------*/

@ Macro to enable a specific system handler
.macro enable_handler bit_offset:req
  ENTER_CRITICAL
  LDR     r0, =SHCSR
  LDR     r1, [r0]
  ORR     r1, r1, #(1 << \bit_offset)
  STR     r1, [r0]
  EXIT_CRITICAL
.endm

@ Macro to disable a specific system handler
.macro disable_handler bit_offset:req
  ENTER_CRITICAL
  LDR     r0, =SHCSR
  LDR     r1, [r0]
  BIC     r1, r1, #(1 << \bit_offset)
  STR     r1, [r0]
  EXIT_CRITICAL
.endm

@ Macro to check if a specific system handler is pending
.macro is_handler_pending bit_offset:req
  LDR     r0, =SHCSR
  LDR     r1, [r0]
  TST     r1, #(1 << \bit_offset)
  ITE     EQ
  MOVEQ   r0, #0
  MOVNE   r0, #1
.endm

@ Macro to check if a specific system handler is active
.macro is_handler_active bit_offset:req
  LDR     r0, =SHCSR
  LDR     r1, [r0]
  TST     r1, #(1 << \bit_offset)
  ITE     EQ
  MOVEQ   r0, #0
  MOVNE   r0, #1
.endm

@ Macros to enable specific system handlers
.macro _enable_UsageFault
  enable_handler 18
.endm

.macro _enable_BusFault
  enable_handler 17
.endm

.macro _enable_MemMan_fault
  enable_handler 16
.endm

@ Macros to disable specific system handlers
.macro _disable_UsageFault
  disable_handler 18
.endm

.macro _disable_BusFault
  disable_handler 17
.endm

.macro _disable_MemMan_fault
  disable_handler 16
.endm

@ Macros to check if specific system handlers are pending
.macro _is_SVC_pending
  is_handler_pending 15
.endm

.macro _is_BusFault_pending
  is_handler_pending 14
.endm

.macro _is_MemMan_fault_pending
  is_handler_pending 13
.endm

.macro _is_UsageFault_pending
  is_handler_pending 12
.endm

@ Macros to check if specific system handlers are active
.macro _is_SYSTICK_active
  is_handler_active 11
.endm

.macro _is_PendSV_active
  is_handler_active 10
.endm

.macro _is_DBGMon_active
  is_handler_active 8
.endm

.macro _is_SVC_active
  is_handler_active 7
.endm

.macro _is_UsageFault_active
  is_handler_active 3
.endm

.macro _is_BusFault_active
  is_handler_active 1
.endm

.macro _is_MemMan_fault_active
  is_handler_active 0
.endm


/*--------UFSR---------*/
/*--------UFSR---------*/

.macro check_fault_bit reg:req, bit_offset:req
  LDR     r0, =\reg
  LDR     r1, [r0]
  TST     r1, #(0b1 << \bit_offset)
  ITE     NE
  MOVNE   r0, #1  @ bit set
  MOVEQ   r0, #0  @ bit not set
.endm

.macro _div_by0_UsageFault
  check_fault_bit UFSR, 25
.endm

.macro _unalignement_UsageFault
  check_fault_bit UFSR, 24
.endm

.macro _coprocessor_UsageFault
  check_fault_bit UFSR, 19
.endm

.macro _invPC_UsageFault
  check_fault_bit UFSR, 18
.endm

.macro _invEPSR_UsageFault
  check_fault_bit UFSR, 17
.endm

.macro _undef_instr_UsageFault
  check_fault_bit UFSR, 16
.endm

/*--------BFSR---------*/
/*--------BFSR---------*/

.macro _BFAR_valid_addr
  check_fault_bit BFSR, 15
.endm

.macro _FP_LazyState_BusFault
  check_fault_bit BFSR, 13
.endm

.macro _push_BusFault
  check_fault_bit BFSR, 12
.endm

.macro _pop_BusFault
  check_fault_bit BFSR, 11
.endm

.macro _imprecise_BusFault
  check_fault_bit BFSR, 10
.endm

.macro _precise_DBus_error
  check_fault_bit BFSR, 9
.endm

.macro _IBus_error
  check_fault_bit BFSR, 8
.endm

/*--------MMFSR---------*/
/*--------MMFSR---------*/

.macro _MMAR_valid_addr
  check_fault_bit MMFSR, 7
.endm

.macro _FP_LazyState_MemMan_fault
  check_fault_bit MMFSR, 5
.endm

.macro _push_MemMan_fault
  check_fault_bit MMFSR, 4
.endm

.macro _pop_MemMan_fault
  check_fault_bit MMFSR, 3
.endm

.macro _DataAccess_MemMan_fault
  check_fault_bit MMFSR, 1
.endm

.macro _ExecNot_section_MemMan_fault
  check_fault_bit MMFSR, 0
.endm

/*--------HFSR---------*/
/*--------HFSR---------*/

.macro _forced_HardFault
  check_fault_bit HFSR, 30         @ Check for Forced Hard Fault
.endm

.macro _vect_table_HardFault
  check_fault_bit HFSR, 1          @ Check for Vector Table Read Fault
.endm

/*--------MMFAR---------*/
/*--------MMFAR---------*/

.macro _get_MemManFault_addr
  LDR     r0, =MMFAR
  LDR     r0, [r0]      @ load the address of the memory management fault
.endm

/*--------BFAR---------*/
/*--------BFAR---------*/

.macro _get_BusFault_addr
  LDR     r0, =BFAR
  LDR     r0, [r0]      @ load the address of the BUS fault
.endm

/*--------AFSR---------*/
/*--------AFSR---------*/

.macro _get_AuxFault_addr
  LDR     r0, =AFSR
  LDR     r0, [r0]      @ load the Auxiliary fault status register
.endm



@-----------------------------------------------------
@-----------------------------------------------------
@----------------------------------------------------- Linked list functions
@-----------------------------------------------------
@-----------------------------------------------------


@ brief: Adds new node at the start of the linked list
@ this function puts new data in the head node in .bss section
@ and allocates a new node in the heap with malloc and sets old data in new node 
@ example before calling function do: LDR r0, =head_ptrs then offset it by n bytes
@ n being multiple of 8
@  r0: Address of head ptr 
@  r1: Value to add to linked list
@   malloc_addr: _malloc or _kmalloc
.macro _ll_add_node malloc_addr:req
  PUSH    {r4, lr}

  LDR     r2, [r0]             @ Load the value of data field from head_ptr
  LDR     r3, [r0, #4]         @ Load the value of next_ptr field from head_ptr

  ENTER_CRITICAL
  CMP     r2, #0x0             @ Check if data field is null
  IT      EQ                   @ If-Then condition for EQ (Equal)
  STREQ   r1, [r0]             @ If data field is null, store the new value in head_ptr
  BEQ     1f                   @ If data was null, exit the function

  PUSH    {r0}                 @ Save the address of head ptr
  MOVS    r0, #8               @ Move size 8 to r0 (malloc argument)
  BL      \malloc_addr         @ Call malloc to allocate a new node
  CMP     r0, #0               @ Check if malloc returned NULL
  BEQ     1f                   @ If malloc failed, exit the function
  POP     {r4}                 @ Recover the address of head ptr, set it in r4

  STR     r2, [r0]             @ Store old data in the new node's data field
  STR     r3, [r0, #4]         @ Store old next_ptr in the new node's next_ptr field

  STR     r1, [r4]             @ Store new data in head_ptr's data field
  STR     r0, [r4, #4]         @ Store address of new node in head_ptr's next_ptr field

1:
  EXIT_CRITICAL
  POP     {r4, lr}          @ Restore link register
.endm



@ brief: Removes node from the linked list that has the value in r1
@ This function iterates over the linked list 
@   - if head ptr is null it exits 
@   - if head ptr is the one to remove and no next node we nullify it
@   - if head ptr is the one to remove and theres a next node we copy next in head and free next from heap
@   - else we iterate and when found we do the same thing copy next in node to remove and free it 
@   - if none found we exit
@ example before calling macro do: LDR r0, =head_ptrs then offset it by n bytes n being  multiple of 8
@ and set r1 to the data field of node to remove
@   r0: Address of head ptr 
@   r1: Value to remove from linked list
@   free_addr: _free or _kfree
.macro _ll_rem_node free_addr:req  
  PUSH    {lr}
  LDR     r2, [r0]              @ Load the value of data field from head_ptr
  CBZ     r2, 5f                @ Exit if data field of head pointer is null

  CMP     r2, r1                @ Check if node to remove is the head node
  BEQ     2f                    @ If head node is the node to remove

@ loop
1:
  MOV     r3, r0                @ Save the address of the current node in r3
  LDR     r0, [r0, #4]          @ Load the value of next_ptr field
  CBZ     r0, 5f                @ Exit if no more nodes
  LDR     r2, [r0]              @ Load the data field of the next node
  CMP     r2, r1                @ Check if data field = data to remove
  BEQ     3f                    @ Branch if data matches

  LDR     r2, [r0, #4]          @ Load the value of next_ptr field of the next node
  CBZ     r2, 5f                @ Exit if no more nodes
  B       1b                    @ Continue loop

@ remove_head
2:
  ENTER_CRITICAL
  LDR     r2, [r0, #4]          @ Load the value of next_ptr field
  CBZ     r2, 4f                @ If there's no next node, nullify head

  @ Free the next node and set head node to the next node
  LDR     r3, [r2]              @ Load the value of data field of next node
  STR     r3, [r0]              @ Store the data field of next node in head node
  LDR     r3, [r2, #4]          @ Load the value of next_ptr field of next node
  STR     r3, [r0, #4]          @ Store the next_ptr field of next node in head node
  MOV     r0, r2                @ Load the address of next node in r0
  BL      \free_addr            @ Free the old head node
  B       5f                    @ Exit

@ remove_node
3:
  ENTER_CRITICAL
  LDR     r1, [r0, #4]          @ Load the value of next_ptr field of node to remove
  STR     r1, [r3, #4]          @ Link previous node to the next node
  MOV     r0, r3                @ Load the address of node to remove in r0
  BL      \free_addr            @ Free the node
  B       5f                    @ Exit

@ nullify_head
4:
  ENTER_CRITICAL
  MOVS    r2, #0                @ Set head node data to 0
  STR     r2, [r0]
  STR     r2, [r0, #4]          @ Set head node next_ptr to 0

@ exit
5:
  EXIT_CRITICAL
  POP    {lr}
.endm


@ arg0: address of head node
@ arg1: value to search
@ return: address of node if found
@ return 0(null): if not found
.macro _ll_search_node
@ search
1:
  LDR     r2, [r0]       @ Load the value of the data field in r2
  CMP     r2, r1         @ Compare it with the search value
  BEQ     3f             @ Branch if found

  LDR     r0, [r0, #4]   @ Load the next_ptr field into r0
  CMP     r0, #0         @ Check if next_ptr is null
  BEQ     2f             @ Branch if not found

  B       1b             @ Continue searching
@ return success
2:
  MOVS    r0, #0         @ Set return value to 0 (null)
@ return fail
3:

.endm


@-----------------------------------------------------
@-----------------------------------------------------
@----------------------------------------------------- Memory management functions
@-----------------------------------------------------
@-----------------------------------------------------


@-----------------------------------
@ SYSCALL: Used by the app to expand the APP process heap towards the top
@ arg0: amount of SRAM needed
@ returns: address of old system break, or NULL if failed to allocate SRAM
@-----------------------------------
.macro _sbrk
  ENTER_CRITICAL
  DSB
  MRS     r1, PSP             @ Get PSP (Process Stack Pointer)
  
  LDR     r2, =p_brk
  LDR     r2, [r2]            @ Load the address of app system break
  
  ADD     r0, r2, r0          @ Calculate new system break address
  CMP     r0, r1              @ Compare with PSP
  BGE     1f                  @ Return 0 if new system break is beyond PSP

  LDR     r2, =p_brk
  STR     r0, [r2]            @ Store the new system break

  DSB
  EXIT_CRITICAL
  BX      lr                 @ Return new system break address

1:
  MOVS    r0, #0             @ Return 0 on failure
  DSB
  EXIT_CRITICAL
  BX      lr
.endm

@-----------------------------------
@ SYSCALL: Used by the app to collapse the APP process heap towards the bottom freeing memory
@ arg0: amount of SRAM to free
@ returns: address of new system break, or NULL on error
@-----------------------------------
.macro _sbrk_free
  ENTER_CRITICAL
  DSB
  LDR     r1, =_edata         @ Load the address of the end of .data in SRAM

  LDR     r2, =p_brk
  LDR     r2, [r2]            @ Load the address of app system break
  
  SUBS    r0, r2, r0          @ Compute new system break address
  BLE     1f                  @ Return 0 if new system break is invalid (less than current break)

  CMP     r0, r1              @ Compare new system break with end of .data
  BLT     1f                  @ Return 0 if new system break is below end of .data

  STR     r0, [r2]            @ Store the new system break

  DSB
  EXIT_CRITICAL
  BX      lr                 @ Return new system break address

1:
  MOVS    r0, #0             @ Return 0 on error
  DSB
  EXIT_CRITICAL
  BX      lr
.endm


@-----------------------------------
@ Used by the kernel to expand the KERNEL heap towards the top
@ arg0: amount of SRAM needed
@ returns address of old system break
@ returns NULL if failed to allocate SRAM
@-----------------------------------
.type _ksbrk, %function
_ksbrk:
  ENTER_CRITICAL
  DSB
  LDR     r2, =k_brk
  LDR     r2, [r2]           @ Load the address of kernel system break
  ADD     r0, r2, r0         @ Add the system break address to the requested amount of memory
  CMP     r0, r12            @ Compare new system break to MSP address (CURRENT DEFAULT SP)
  BGE     1f                 @ If new system break is beyond MSP, allocation failed

  LDR     r2, =k_brk
  STR     r0, [r2]           @ Update the system break address

  DSB
  EXIT_CRITICAL
  BX      lr                 @ Return new system break address

1:
  MOVS    r0, #0             @ Return 0 on failure
  DSB
  EXIT_CRITICAL
  BX      lr
  .align  2
  .size _ksbrk, .-_ksbrk


@-----------------------------------
@ Used by the kernel to collapse the KERNEL heap towards the bottom freeing memory
@ arg0: amount of SRAM to free
@ returns address of new system break
@-----------------------------------
.type _ksbrk_free, %function
_ksbrk_free:
  ENTER_CRITICAL
  DSB
  LDR     r1, =_ekdata       @ Load the address of the end of .kdata in SRAM

  LDR     r2, =k_brk
  LDR     r2, [r2]           @ Load the address of kernel system break

  SUBS    r0, r2, r0         @ Compute new system break address
  CMP     r0, r1             @ Check if new system break is valid
  BLS     1f                 @ If new system break <= _ekdata, allocation failed

  STR     r0, [r2]           @ Update the system break address

  DSB
  EXIT_CRITICAL
  BX      lr                 @ Return new system break address

1:
  MOVS    r0, #0             @ Return 0 on error
  DSB
  EXIT_CRITICAL
  BX      lr
  .align  2
  .size _ksbrk_free, .-_ksbrk_free

@-----------------------------------SYSCALL
@ Used by the apps as syscall to allocate memory from heap dynamically
@ Uses the arrays in .bss.system to manage free heap blocks
@ arg0: amount of SRAM to allocate
@ returns pointer to the newly allocated block
@ returns NULL on failure
@-----------------------------------
.macro _malloc
  CMP     r0, #0                      @ Exit if mem to allocate is 0 and return NULL
  IT      EQ
  BXEQ    lr

  PUSH    {r4-r8}                     @ Save registers and return address

  LDR     r1, =blocks_addr            @ Load the address of the array list
  LDR     r2, =blocks_sizes           @ Load the address of the sizes of the blocks
  LDR     r3, =MAX_HEAP_BLOCKS        @ Load the number of indexes (number of blocks)
  LDR     r8, =blocks_cntr            @ Load the free blocks counter
  LDRB    r8, [r8]
  CMP     r8, #0                      @ If no free blocks, expand heap
  BEQ     4f

  MOVW    r4, #0xFFFF                 @ Initialize best fit size to max possible value
  MOV     r5, #0                      @ Initialize index for best fit block
  MOV     r6, #0                      @ Initialize index counter

@ Loop through the array to find an adequate block using best fit
1:
  LDRH    r7, [r2, r6, LSL #1]        @ Load the size of the current block
  CBZ     r7, 2f                      @ If size is 0, skip (optional for performance)
  CMP     r7, r0                      @ Compare size with required size
  BEQ     3f                          @ If found perfect fit, branch
  BLT     2f                          @ If block is too small, skip
  CMP     r7, r4                      @ Compare current block size with best fit size
  ITT     LO                          @ If current block is a better fit
  MOVLO   r4, r7                      @ Update best fit size
  MOVLO   r5, r6                      @ Update best fit index
2:
  ADD     r6, r6, #1                  @ Increment index
  CMP     r6, r3                      @ Check if index reached max
  BNE     1b                          @ If not, continue searching

@ No suitable block found, expand the heap
  MOV     r6, #0xFFFF
  CMP     r4, r6                      @ Check if no free blocks found at all
  BEQ     4f                          @ Expand heap if no block found

@ Use the best fit block found
3:
  ENTER_CRITICAL
  LDR     r0, [r1, r5, LSL #2]        @ Load the address of the block to return it in r0
  MOV     r7, #0
  STRH    r7, [r2, r5, LSL #1]        @ Nullify the size to indicate it's no longer free
  SUB     r8, r8, #1                  @ Decrement the free block counter
  LDR     r7, =blocks_cntr
  STRB    r8, [r7]                    @ Store the updated counter value
  EXIT_CRITICAL
  B       5f                          @ Exit

@ Expand kernel system break with r0 as arg0
4:
  PUSH    {lr}                        @ save lr before branching with link 
  BL      _sbrk                       @ Call _ksbrk to expand the heap
  POP     {lr}

@ Exit
5:
  POP     {r4-r8}                 @ Restore registers and return
.endm

@-----------------------------------SYSCALL
@ Used by the apps as syscall to free memory allocated from the heap
@ Uses the arrays in .bss.system to manage free heap blocks
@ arg0: pointer to the start of block
@ arg1: size of the block to free
@ returns 0 on success
@ returns 1 on failure
@-----------------------------------
.macro _free
  CMP     r0, #0                      @ Exit if the pointer to free is NULL
  IT      EQ
  BXEQ    lr
  CMP     r1, #0                      @ Exit if size is 0
  IT      EQ
  BXEQ    lr

  PUSH    {r4-r7}                     @ Save registers and return address

  LDR     r1, =blocks_addr            @ Load the address of the array list
  LDR     r2, =blocks_sizes           @ Load the address of the sizes of the blocks
  LDR     r3, =MAX_HEAP_BLOCKS        @ Load the number of indexes (number of blocks)


  @ Find the index of the block to free
  MOV     r4, #0                      @ Initialize index counter

1:
  LDR     r5, [r1, r4, LSL #2]        @ Load the address of the block
  CMP     r5, r0                      @ Compare the block address
  BEQ     2f                          @ If found, branch to restore it
  ADD     r4, r4, #1                  @ Increment index
  CMP     r4, r3                      @ Check if index reached max
  BNE     1b                          @ If not, continue searching

  @ If block was not found, return (or handle error if needed)
  POP     {r4-r5}                 @ Restore registers and return
  BX      lr

@ Restore the block to the free list
2:
  ENTER_CRITICAL
  @ Restore the block's size back in the sizes array
  STRH    r1, [r2, r4, LSL #1]        @ Store the block size in the list
  @ Update the free block counter
  LDR     r3, =blocks_cntr
  LDRB    r4, [r3]                    @ Load the free blocks counter
  ADD     r4, r4, #1                  @ Increment the free block counter
  STRB    r4, [r3]                    @ Store the updated counter value
  EXIT_CRITICAL

  POP     {r4-r5}                 @ Restore registers and return
.endm


@-----------------------------------
@ Used by the kernel to allocate memory from heap dynamically
@ Uses the arrays in .bss.system to manage free heap blocks
@ arg0: amount of SRAM to allocate
@ returns pointer to the newly allocated block
@ returns NULL on failure
@-----------------------------------
.global _kmalloc
.type _kmalloc, %function
_kmalloc:
  CMP     r0, #0                      @ Exit if mem to allocate is 0 and return NULL
  IT      EQ
  BXEQ    lr

  PUSH    {r4-r8, lr}                 @ Save registers and return address

  LDR     r1, =kblocks_addr           @ Load the address of the array list
  LDR     r2, =kblocks_sizes          @ Load the address of the sizes of the blocks
  LDR     r3, =KMAX_HEAP_BLOCKS       @ Load the number of indexes (number of blocks)
  LDR     r8, =kblocks_cntr           @ Load the free blocks counter
  LDRB    r8, [r8]

  CMP     r8, #0                      @ If no free blocks, expand heap
  BEQ     4f

  MOVW    r4, #0xFFFF                 @ Initialize best fit size to max possible value
  MOV     r5, #0                      @ Initialize index for best fit block
  MOV     r6, #0                      @ Initialize index counter

@ Loop through the array to find an adequate block using best fit
1:
  LDRH    r7, [r2, r6, LSL #1]        @ Load the size of the current block
  CBZ     r7, 2f                      @ If size is 0, skip (optional for performance)
  CMP     r7, r0                      @ Compare size with required size
  BEQ     3f                          @ If found perfect fit, branch
  BLT     2f                          @ If block is too small, skip
  CMP     r7, r4                      @ Compare current block size with best fit size
  ITT     LO                          @ If current block is a better fit
  MOVLO   r4, r7                      @ Update best fit size
  MOVLO   r5, r6                      @ Update best fit index
2:
  ADD     r6, r6, #1                  @ Increment index
  CMP     r6, r3                      @ Check if index reached max
  BNE     1b                          @ If not, continue searching

@ No suitable block found, expand the heap
  MOV     r6, #0xFFFF
  CMP     r4, r6                      @ Check if no free blocks found at all
  BEQ     4f                          @ Expand heap if no block found

@ Use the best fit block found
3:
  ENTER_CRITICAL
  LDR     r0, [r1, r5, LSL #2]        @ Load the address of the block to return it in r0
  MOV     r7, #0
  STRH    r7, [r2, r5, LSL #1]        @ Nullify the size to indicate it's no longer free
  SUB     r8, r8, #1                  @ Decrement the free block counter
  LDR     r8, =kblocks_cntr           @ Load the free blocks counter
  STRB    r8, [r7]           @ Store the updated counter value
  EXIT_CRITICAL
  B       5f                          @ Exit

@ Expand kernel system break with r0 as arg0
4:
  BL      _ksbrk                      @ Call _ksbrk to expand the heap

@ Exit
5:
  POP     {r4-r8, pc}                 @ Restore registers and return
  .align  2
  .size _kmalloc, .-_kmalloc

@-----------------------------------
@ Used by the kernel to free memory allocated from the heap
@ Uses the arrays in .bss.system to manage free heap blocks
@ arg0: pointer to the start of block
@ arg1: size of the block to free
@ returns 0 on success
@ returns 1 on failure
@-----------------------------------
.global _kfree
.type _kfree, %function
_kfree:
  CMP     r0, #0                      @ Exit if the pointer to free is NULL
  IT      EQ
  BXEQ    lr
  CMP     r1, #0                      @ Exit if size is 0
  IT      EQ
  BXEQ    lr

  PUSH    {r4-r7, lr}                 @ Save registers and return address

  LDR     r1, =kblocks_addr           @ Load the address of the array list
  LDR     r2, =kblocks_sizes          @ Load the address of the sizes of the blocks
  LDR     r3, =KMAX_HEAP_BLOCKS       @ Load the number of indexes (number of blocks)


  @ Find the index of the block to free
  MOV     r4, #0                      @ Initialize index counter

1:
  LDR     r5, [r1, r4, LSL #2]        @ Load the address of the block
  CMP     r5, r0                      @ Compare the block address
  BEQ     2f                          @ If found, branch to restore it
  ADD     r4, r4, #1                  @ Increment index
  CMP     r4, r3                      @ Check if index reached max
  BNE     1b                          @ If not, continue searching

  @ If block was not found, return (or handle error if needed)
  POP     {r4-r7, pc}                 @ Restore registers and return
  BX      lr

@ Restore the block to the free list
2:
  ENTER_CRITICAL
  @ Restore the block's size back in the sizes array
  STRH    r1, [r2, r4, LSL #1]        @ Store the block size in the list
  @ Update the free block counter
  LDR     r3, =kblocks_cntr
  LDRB    r4, [r3]                    @ Load the free blocks counter
  ADD     r4, r4, #1                  @ Increment the free block counter
  STRB    r4, [r3]                    @ Store the updated counter value
  EXIT_CRITICAL

  POP     {r4-r7, pc}                 @ Restore registers and return
  .align  2
  .size _kfree, .-_kfree



@ This function can be used by kernel or by app and does not require SVC
@ Function copies 1 word at a time so buffer needs to be 4 bytes aligned
@ Arguments:
@ r0: src (source address)
@ r1: dest (destination address)
@ r2: length (number of bytes to copy, assumed to be multiple of 4)
.global memcpy_4
.type memcpy_4, %function
memcpy_4:
  ENTER_CRITICAL
  DSB
  LDR     r3, [r0], #4      @ Load word from src and increment src by 4
  STR     r3, [r1], #4      @ Store word to dest and increment dest by 4
  SUBS    r2, r2, #4        @ Decrement length counter by 4
  BNE     memcpy_4          @ If length is not 0, continue loop
  DSB
  EXIT_CRITICAL
  BX      lr                @ Return from function
  .align    2
  .size memcpy_4, .-memcpy_4

@ This function copies 1 byte at a time so no buffer alignment required
@ Arguments:
@ r0: src (source address)
@ r1: dest (destination address)
@ r2: length (number of bytes to copy)
.global memcpy_1
.type memcpy_1, %function
memcpy_1:
  ENTER_CRITICAL
  DSB
  @ Loop to copy bytes
1:
  LDRB    r3, [r0], #1      @ Load byte from src and increment src by 1
  STRB    r3, [r1], #1      @ Store byte to dest and increment dest by 1
  SUBS    r2, r2, #1        @ Decrement length counter by 1
  BNE     1b                @ If length is not 0, continue loop
  DSB
  EXIT_CRITICAL
  BX      lr                @ Return from function
  .align    2
  .size memcpy_1, .-memcpy_1


@ This function sets memory with a 4-byte aligned value.
@ Arguments:
@ r0: dest (destination address)
@ r1: value (byte value to set)
@ r2: length (number of bytes to set, assumed to be multiple of 4)
.global memset_4
.type memset_4, %function
memset_4:
  ENTER_CRITICAL
  DSB
  @ Make a word ready containing 4 bytes of the required byte value
  MOV     r3, r1              @ Move the byte value into r3
  ORR     r3, r3, r3, LSL #8  @ Set byte 2
  ORR     r3, r3, r3, LSL #16 @ Set byte 3 and byte 4
  @ Loop to set memory
1:
  STR     r3, [r0], #4        @ Store word to dest and increment dest by 4
  SUBS    r2, r2, #4          @ Decrement length counter by 4
  BNE     1b                 @ If length is not 0, continue loop
  DSB
  EXIT_CRITICAL
  BX      lr                  @ Return from function
  .align    2
  .size memset_4, .-memset_4


@ This function sets 1 byte at a time so no buffer alignment required
@ Arguments:
@ r0: dest (destination address)
@ r1: value (byte value to set)
@ r2: length (number of bytes to set)
.global memset_1
.type memset_1, %function
memset_1:
  @ Check if length is zero
  CMP     r2, #0
  BEQ     1f                 @ If length is zero, exit
  ENTER_CRITICAL

  @ Set up loop
  MOV     r3, r1             @ Load the byte value into r3

  1: 
    STRB    r3, [r0], #1     @ Store byte to dest and increment dest by 1
    SUBS    r2, r2, #1       @ Decrement length counter by 1
    BNE     1b               @ If length is not zero, continue loop

  @ Exit critical section
  EXIT_CRITICAL
  BX      lr                 @ Return from function
  .align  2
  .size memset_1, .-memset_1


@ This function can be used by kernel or by app and does not require SVC
@ It assumes the memory size to zero out is 4 bytes aligned
@ Arguments:
@ r0: dest (destination address)
@ r2: length (number of bytes to zero out, assumed to be multiple of 4)
.global memzero_4
.type memzero_4, %function
memzero_4:
  MOV     r3, #0                @ Load zero into r3
  DSB
  CMP     r2, #0                @ Check if length is 0
  BEQ     2f                  @ If length is 0, skip loop
@ loop
1:
  STR     r3, [r0], #4         @ Store zero to dest and increment dest by 4
  SUBS    r2, r2, #4           @ Decrement length counter by 4
  BNE     1b                 @ If length is not 0, continue loop
@ exit
2:
  DSB
  BX      lr                   @ Return from function
  .align  2
  .size memzero_4, .-memzero_4


@ This function can be used by kernel or by app and does not require SVC
@ Arguments:
@ r0: dest (destination address)
@ r2: length (number of bytes to zero out)
.global memzero_1
.type memzero_1, %function
memzero_1:
  DSB
  MOV     r3, #0              @ Load zero into r3
  1:
    STRB    r3, [r0], #1      @ Store zero to dest and increment dest by 1
    SUBS    r2, r2, #1        @ Decrement length counter by 1
    BNE     1b                @ If length is not 0, continue loop
    DSB
    BX      lr                @ Return from function
  .align    2
    .size memzero_1, .-memzero_1

@-----------------------------------
@ Compares two memory blocks byte by byte.
@ Arguments:
@   r0: Pointer to the first memory block (src1).
@   r1: Pointer to the second memory block (src2).
@   r2: Number of bytes to compare.
@ Returns:
@   r0: 0 if the memory blocks are equal, non-zero otherwise.
@-----------------------------------
.global memcmp_1
.type memcmp_1, %function
memcmp_1:
  DSB
  PUSH    {r4}
  LDRB    r3, [r0], #1   @ Load byte from src1 and increment src1
  LDRB    r4, [r1], #1   @ Load byte from src2 and increment src2
  CMP     r3, r4         @ Compare bytes
  BNE     1f          @ Exit loop if bytes are not equal
  SUBS    r2, r2, #1     @ Decrement length counter
  BNE     memcmp_1          @ Loop if length is not zero
  MOV     r0, #0         @ If all bytes are equal, return 0
  BX      lr
  1:
    MOV     r0, #1         @ If bytes are not equal, return non-zero
    DSB
    POP    {r4}
    BX      lr
  .align    2
  .size memcmp_1, .-memcmp_1

@-----------------------------------
@ Memory Compare (memcmp)
@ Compares two memory blocks word by word.
@ Arguments:
@   r0: Pointer to the first memory block (src1).
@   r1: Pointer to the second memory block (src2).
@   r2: Number of bytes to compare.
@ Returns:
@   r0: 0 if the memory blocks are equal, non-zero otherwise.
@-----------------------------------
.global memcmp_4
.type memcmp_4, %function
memcmp_4:
  DSB
  PUSH    {r4}
  LDR     r3, [r0], #4   @ Load word from src1 and increment src1 by 4
  LDR     r4, [r1], #4   @ Load word from src2 and increment src2 by 4
  CMP     r3, r4         @ Compare words
  BNE     1f          @ Exit loop if words are not equal
  SUBS    r2, r2, #4     @ Decrement length counter by 4
  BGE     memcmp_4          @ Loop if length is not zero or negative
  MOV     r0, #0         @ If all words are equal, return 0
  BX      lr             @ Return from function
  1:
    DSB
    MOV     r0, #1         @ If words are not equal, return non-zero
    POP    {r4}
    BX      lr             @ Return from function
  .align    2
  .size memcmp_4, .-memcmp_4


@ This mechanism is used as syscall by user app to add functions
@ as callbacks to get called when IRQs happen
@ arg0: Address of Head node of the linked list to add function to 
@ arg1: Function to call when the IRQ gets called
.macro _add_callback 
  _ll_add_node _kmalloc
.endm

@ This mechanism is used as syscall by user app to remove functions
@ as callbacks to no longer get called when IRQs happen
@ arg0: Address of Head node of the linked list to remove function from 
@ arg1: Function to remove from callback list
.macro _rem_callback 
  _ll_rem_node _kfree
.endm


@-----------------------------------------------------
@-----------------------------------------------------
@----------------------------------------------------- Tasks
@-----------------------------------------------------
@-----------------------------------------------------

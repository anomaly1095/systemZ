
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

.include "data.s"


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

.define LITTLE_ENDIAN

@ used by FLASH set_options function
.define DEVELOPMENT_MODE 


@ Macro used for configuring the various regions used by the system
.macro MPU_CONFIG_REGION region_base:req, region_number:req, region_mask:req
  LDR     r0, =MPU_BASE
	LDR     r1, =\region_base
	MOVW    r2, #(0b10000 | \region_number) @ Region number, VALID bit
	ORR     r1, r1, r2
	STR     r1, [r0, #0x0C]                 @ MPU_RBAR reg

	LDR     r2, =\region_mask
	STR     r2, [r0, #0x10]                 @ MPU_RASR reg
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
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ISER0  @ Select the appropriate NVIC_ISER register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
.endm
  
@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to disable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_disable_irq
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ICER0   @ Select the appropriate NVIC_ICER register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
.endm

@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to set an interrupt as pending
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_set_pend_irq
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ISPR0   @ Select the appropriate NVIC_ISPR register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
.endm

@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to remove an interrupt from pending list
@ arg0: number of the IRQ (0..239)
@-----------------------------------
.macro _NVIC_clear_pend_irq
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_ICPR0   @ Select the appropriate NVIC_ICPR register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
.endm

@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to check interrupt if the interrupt is active
@ arg0: number of the IRQ (0..239)
@ return: 1 if active / 0 if idle
@-----------------------------------
.macro _NVIC_check_active_irq
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT7  r0, NVIC_IABR0   @ Select the appropriate NVIC_IABR register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  TST     r1, r3                @ check if bit is set (IRQ active)
  ITE     NE
  MOVNE   r0, #1                @ bit is set (IRQ active)
  MOVEQ   r0, #0                @ bit is not set (IRQ idle)
.endm  

@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to set the priority of the interrupt
@ arg0: number of the IRQ (0..239)
@ arg1: priority number
@-----------------------------------
.macro _NVIC_set_prio_irq
  NVIC_REG_SELECT59 r0, NVIC_IPR0   @ Select the appropriate NVIC_IPR register
  STRB    r1, [r2]                @ Store the priority number in the selected register byte
  MOV     r0, #0
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
  MOV     r0, #0
.endm

@-----------------------------------------------------
@-----------------------------------------------------
@----------------------------------------------------- SCB 
@-----------------------------------------------------
@-----------------------------------------------------

/*--------ACTLR---------*/
/*--------ACTLR---------*/

.macro DIS_OUTOFORDER_EXEC state:req
  LDR     r0, =ACTLR
  LDR     r1, [r0]
  LDR     r2, =\state
  CBZ     r2, .enable   @ check if state is 0 (enable)
  .disable:
  @ Disable features: Set bits
  ORR     r1, r1, 0x207 @ mask for disabling features
  B       .exit
  .enable:
  @ Enable features: Clear bits
  BIC     r1, r1, 0x207 @ mask for enabling features
  .exit:
  DSB                 @ Data Synchronization Barrier
  ISB                 @ Instruction Synchronization Barrier
  STR     r1, [r0]    @ write back the modified value to ACTLR
  DSB                 @ Ensure the write is complete before continuing
  ISB                 @ Ensure the new instructions are fetched correctly
.endm

/*--------CPUID---------*/
/*--------CPUID---------*/

@ returns CPUID in r0
.macro GET_CPUID
  LDR     r0, =CPUID
  LDR     r0, [r0]
.endm


/*--------ICSR---------*/
/*--------ICSR---------*/

.macro SET_PENDING_BIT bit_mask:req
  CPSID   I                @ Disable interrupts
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  ORR     r1, r1, #\bit_mask    @ Set the specified pending bit
  STR     r1, [r0]         @ Write back the modified value to ICSR
  CPSIE   I                @ Enable interrupts
.endm

.macro CLEAR_PENDING_BIT bit_mask:req
  CPSID   I                @ Disable interrupts
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  BIC     r1, r1, #\bit_mask    @ Clear the specified pending bit
  STR     r1, [r0]         @ Write back the modified value to ICSR
  CPSIE   I                @ Enable interrupts
.endm

.macro CHECK_PENDING_BIT bit_mask:req
  LDR     r0, =ICSR        @ Load the address of ICSR
  LDR     r1, [r0]         @ Load the current value of ICSR
  MOV     r0, #0
  TST     r1, #\bit_mask        @ Check the specified pending bit
  IT      NE
  MOVNE   r0, #1
.endm

.macro NMI_set_pending
  SET_PENDING_BIT 0x80000000 @ Set the NMI pending bit (bit 31)
.endm

.macro PENDSV_set_pending
  SET_PENDING_BIT 0x10000000 @ Set the PENDSV pending bit (bit 28)
.endm

.macro PENDSV_clear_pending
  SET_PENDING_BIT 0x8000000 @ Clear the PENDSV pending bit (bit 27)
.endm

.macro PENDSV_check_pending
  CHECK_PENDING_BIT 0x10000000 @ Check the PENDSV pending bit (bit 28)
.endm

.macro SYSTICK_set_pending
  SET_PENDING_BIT 0x4000000 @ Set the SYSTICK pending bit (bit 26)
.endm

.macro SYSTICK_clear_pending
  SET_PENDING_BIT 0x2000000 @ Clear the SYSTICK pending bit (bit 25)
.endm

.macro SYSTICK_check_pending
  CHECK_PENDING_BIT 0x4000000 @ Check the SYSTICK pending bit (bit 26)
.endm

.macro ISR_check_pending
  CHECK_PENDING_BIT 0x400000 @ Check if any ISR is pending excluding NMI and Faults.
.endm

.macro ISR_highest_pending
  CPSID   I                 @ Disable interrupts
  LDR     r0, =ICSR         @ Load the address of ICSR
  LDR     r1, [r0]          @ Load the current value of ICSR
  MOVW    r2, #0xF000       @ bottom 4 bits of mask for VECTPENDING
  MOVT    r2, #7            @ top 3 bits of mask
  AND     r0, r1, r2        @ Check the specified pending bit
  CPSIE   I                 @ Enable interrupts
.endm

.macro ISR_check_preempted
  CHECK_PENDING_BIT 0x800   @ checks if there are any preempted interrupt routines
.endm

.macro ISR_str_actv_num
  MRS     r0, IPSR                  @ Read IPSR into r0 (contains active exception number)
  MOV     r1, r0                    @ Move exception number to r1 (to ensure 8-bit value)
  LDRB    r0, last_IRQ              @ load the address of the byte thjat will contain the last ISR num
  STRB    r1, r0                    @ Store active exception number in global byte variable
.endm

/*--------AIRCR---------*/
/*--------AIRCR---------*/

@ Macro used in SCB manip to set the binary point separation
@ that allows the config of number of groups prio and subgroup prio
.macro SET_GRP_SPLIT grp_prios:req
  CPSID   I                  @ Disable interrupts
  LDR     r0, =AIRCR         @ Load address of AIRCR register
  LDR     r1, [r0]           @ Load current value of AIRCR
  BIC     r1, r1, #0x0700    @ Clear old PRIGROUP bits
  MOV     r2, #\grp_prios    @ Store the input priority grouping value in r2
  
  @ Check different priority grouping configurations and set the corresponding PRIGROUP bits
  CMP     r2, #0
  ITT     EQ
  ORREQ   r1, r1, #0x700      @ Set new PRIGROUP bits: 0 groups - 16 subgroups
  BEQ     .exit

  CMP     r2, #2
  ITT     EQ
  ORREQ   r1, r1, #0x600      @ Set new PRIGROUP bits: 2 groups - 8 subgroups
  BEQ     .exit

  CMP     r2, #4
  ITT     EQ
  ORREQ   r1, r1, #0x500      @ Set new PRIGROUP bits: 4 groups - 4 subgroups
  BEQ     .exit
  
  CMP     r2, #8
  ITT     EQ
  ORREQ   r1, r1, #0x400      @ Set new PRIGROUP bits: 8 groups - 2 subgroups
  BEQ     .exit

  @ Default: 16 groups - 0 subgroups (no additional MOV needed)

.exit:
  MOVT    r1, #0x5FA         @ Write key to allow write access to AIRCR
  STR     r1, [r0]           @ Store modified value back to AIRCR
  CPSIE   I                  @ Enable interrupts
.endm

@ Set new PRIGROUP bits: 0 groups - 16 subgroups
.macro prio_split16_0
  SET_GRP_SPLIT 16
.endm
@ Set new PRIGROUP bits: 2 groups - 8 subgroups
.macro prio_split8_2
  SET_GRP_SPLIT 8
.endm
@ Set new PRIGROUP bits: 4 groups - 4 subgroups
.macro prio_split4_4
  SET_GRP_SPLIT 4
.endm
@ Set new PRIGROUP bits: 4 groups - 4 subgroups
.macro prio_split2_8
  SET_GRP_SPLIT 2
.endm
@ Set new PRIGROUP bits: 8 groups - 2 subgroups
.macro prio_split0_16
  SET_GRP_SPLIT 0
.endm


@ Generic macro to manipulate a register using BIC or ORR operations
@ Args:
@   reg: Register to manipulate
@   mask: Bitmask to apply
@   operation: Operation to perform (BIC or ORR)
.macro manipulate_register reg:req, mask:req, operation:req
  LDR     r0, =\reg       @ Load address of the register
  LDR     r1, [r0]        @ Load current value from the register
  \operation r1, r1, #\mask   @ Perform specified operation with the bitmask
  STR     r1, [r0]        @ Store modified value back to the register
.endm

@ Macro to set the reset request in AIRCR
.macro reset_request
  manipulate_register AIRCR, 0b100, ORR   @ Set bit 2 in AIRCR (Reset Request)
.endm

/*--------SCR---------*/
/*--------SCR---------*/

@ Macro to set the SEVONPEND bit in SCR
.macro sev_on_pend
  manipulate_register SCR, 0b10000, ORR    @ Set SEVONPEND bit in SCR
.endm

@ Macro to set the SLEEPDEEP bit in SCR
.macro sleep_deep
  manipulate_register SCR, 0b100, ORR      @ Set SLEEPDEEP bit in SCR
.endm

@ Macro to set the SLEEPONEXIT bit in SCR
.macro sleep_on_exit
  manipulate_register SCR, 0b10, ORR       @ Set SLEEPONEXIT bit in SCR
.endm

/*--------CCR---------*/
/*--------CCR---------*/

@ Macro to clear stack alignment to 4 bytes aligned in CCR
.macro stack_align_4
  manipulate_register CCR, 0x200, BIC
.endm

@ Macro to set stack alignment to 8 bytes aligned in CCR
.macro stack_align_8
  manipulate_register CCR, 0x200, ORR
.endm

@ Macro to Disable NMI and HardFault handlers to ignore bus faults caused by load and store instructions
.macro NMI_HARDFAULT_dis_bus_fault_handling
  manipulate_register CCR, 0x2000000, BIC
.endm

@ Macro to enable NMI and HardFault handlers to ignore bus faults caused by load and store instructions
.macro NMI_HARDFAULT_en_bus_fault_handling
  manipulate_register CCR, 0x2000000, ORR
.endm

@ Macro to disable trapping on division by zero in CCR
.macro div0_notrap
  manipulate_register CCR, 0b10000, BIC
.endm

@ Macro to enable trapping on division by zero in CCR
.macro div0_trap
  manipulate_register CCR, 0b10000, ORR
.endm

@ Macro to disable trapping on unaligned halfword and word accesses in CCR
.macro unalign_notrap
  manipulate_register CCR, 0b1000, BIC
.endm

@ Macro to enable trapping on unaligned halfword and word accesses in CCR
.macro unalign_trap
  manipulate_register CCR, 0b1000, ORR
.endm

@ Macro to allow access to the STIR register in CCR
.macro app_access_STIR
  manipulate_register CCR, 0b10, ORR
.endm

@ Macro to enable returning to thread mode from any level under an EXC_RETURN in CCR
.macro exit_nested_irqs_on_return
  manipulate_register CCR, 0b1, ORR
.endm


/*--------SHPRx---------*/
/*--------SHPRx---------*/

@ Macro to set priority for various system exceptions
.macro set_exception_priority reg:req, bit_offset:req, prio:req
  LDR     r0, =\reg                              @ Load the address of the priority register
  LDR     r1, [r0]                               @ Load the current value of the register
  BIC     r1, r1, #(0xFF << \bit_offset)         @ Clear the current priority bits
  ORR     r1, r1, #(\prio << \bit_offset)        @ Set new priority bits
  STR     r1, [r0]                               @ Store the new value back into the register
.endm

@ Macro to set Usage Fault priority in SHPR1
.macro set_UsageFault_prio prio:req
  set_exception_priority SHPR1, 20, \prio        @ Set Usage Fault priority (bit offset 20)
.endm

@ Macro to set Bus Fault priority in SHPR1
.macro set_BusFault_prio prio:req
  set_exception_priority SHPR1, 12, \prio        @ Set Bus Fault priority (bit offset 12)
.endm

@ Macro to set Memory Management Fault priority in SHPR1
.macro set_MemMan_fault_prio prio:req
  set_exception_priority SHPR1, 4, \prio         @ Set Memory Management Fault priority (bit offset 4)
.endm

@ Macro to set SVCall priority in SHPR2
.macro set_SVC_prio prio:req
  set_exception_priority SHPR2, 28, \prio        @ Set SVCall priority (bit offset 28)
.endm

@ Macro to set SysTick priority in SHPR3
.macro set_SYSTICK_prio prio:req
  set_exception_priority SHPR3, 28, \prio        @ Set SysTick priority (bit offset 28)
.endm

@ Macro to set PendSV priority in SHPR3
.macro set_PendSV_prio prio:req
  set_exception_priority SHPR3, 20, \prio        @ Set PendSV priority (bit offset 20)
.endm


/*--------SHCSR---------*/
/*--------SHCSR---------*/

@ Macro to enable a specific system handler
.macro enable_handler bit_offset:req
  LDR     r0, =SHCSR
  LDR     r1, [r0]
  ORR     r1, r1, #(1 << \bit_offset)
  STR     r1, [r0]
.endm

@ Macro to disable a specific system handler
.macro disable_handler bit_offset:req
  LDR     r0, =SHCSR
  LDR     r1, [r0]
  BIC     r1, r1, #(1 << \bit_offset)
  STR     r1, [r0]
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
.macro enable_UsageFault
  enable_handler 18
.endm

.macro enable_BusFault
  enable_handler 17
.endm

.macro enable_MemMan_fault
  enable_handler 16
.endm

@ Macros to disable specific system handlers
.macro disable_UsageFault
  disable_handler 18
.endm

.macro disable_BusFault
  disable_handler 17
.endm

.macro disable_MemMan_fault
  disable_handler 16
.endm

@ Macros to check if specific system handlers are pending
.macro is_SVC_pending
  is_handler_pending 15
.endm

.macro is_BusFault_pending
  is_handler_pending 14
.endm

.macro is_MemMan_fault_pending
  is_handler_pending 13
.endm

.macro is_UsageFault_pending
  is_handler_pending 12
.endm

@ Macros to check if specific system handlers are active
.macro is_SYSTICK_active
  is_handler_active 11
.endm

.macro is_PendSV_active
  is_handler_active 10
.endm

.macro is_DBGMon_active
  is_handler_active 8
.endm

.macro is_SVC_active
  is_handler_active 7
.endm

.macro is_UsageFault_active
  is_handler_active 3
.endm

.macro is_BusFault_active
  is_handler_active 1
.endm

.macro is_MemMan_fault_active
  is_handler_active 0
.endm


/*--------UFSR---------*/
/*--------UFSR---------*/

.macro check_fault_bit reg:req, bit_offset:req
  LDR     r0, =\reg
  LDR     r1, [r0]
  TST     r1, r1, #(0b1 << bit_offset)
  ITE     NE
  MOVNE   r0, #1  @ bit set
  MOVEQ   r0, #0  @ bit not set
.endm

.macro div_by0_UsageFault
  check_fault_bit UFSR, 25
.endm

.macro unalignement_UsageFault
  check_fault_bit UFSR, 24
.endm

.macro coprocessor_UsageFault
  check_fault_bit UFSR, 19
.endm

.macro invPC_UsageFault
  check_fault_bit UFSR, 18
.endm

.macro invEPSR_UsageFault
  check_fault_bit UFSR, 17
.endm

.macro undef_instr_UsageFault
  check_fault_bit UFSR, 16
.endm

/*--------BFSR---------*/
/*--------BFSR---------*/

.macro BFAR_valid_addr
  check_fault_bit BFSR, 15
.endm

.macro FP_LazyState_BusFault
  check_fault_bit BFSR, 13
.endm

.macro push_BusFault
  check_fault_bit BFSR, 12
.endm

.macro pop_BusFault
  check_fault_bit BFSR, 11
.endm

.macro imprecise_BusFault
  check_fault_bit BFSR, 10
.endm

.macro precise_DBus_error
  check_fault_bit BFSR, 9
.endm

.macro IBus_error
  check_fault_bit BFSR, 8
.endm

/*--------MMFSR---------*/
/*--------MMFSR---------*/

.macro MMAR_valid_addr
  check_fault_bit MMFSR, 7
.endm

.macro FP_LazyState_MemMan_fault
  check_fault_bit MMFSR, 5
.endm

.macro push_MemMan_fault
  check_fault_bit MMFSR, 4
.endm

.macro pop_MemMan_fault
  check_fault_bit MMFSR, 3
.endm

.macro DataAccess_MemMan_fault
  check_fault_bit MMFSR, 1
.endm

.macro ExecNot_section_MemMan_fault
  check_fault_bit MMFSR, 0
.endm

/*--------HFSR---------*/
/*--------HFSR---------*/

.macro forced_HardFault
  check_fault_bit HFSR, 30         @ Check for Forced Hard Fault
.endm

.macro vect_table_HardFault
  check_fault_bit HFSR, 1          @ Check for Vector Table Read Fault
.endm

/*--------MMFAR---------*/
/*--------MMFAR---------*/

.macro get_MemManFault_addr
  LDR     r0, =MMFAR
  LDR     r0, [r0]      @ load the address of the memory management fault
.endm

/*--------BFAR---------*/
/*--------BFAR---------*/

.macro get_BusFault_addr
  LDR     r0, =BFAR
  LDR     r0, [r0]      @ load the address of the BUS fault
.endm

/*--------AFSR---------*/
/*--------AFSR---------*/

.macro get_AuxFault_addr
  LDR     r0, =AFSR
  LDR     r0, [r0]      @ load the Auxiliary fault status register
.endm



@-----------------------------------------------------
@-----------------------------------------------------
@----------------------------------------------------- Memory management
@-----------------------------------------------------
@-----------------------------------------------------


@-----------------------------------
@ SYSCALL: Used by the app to expand the APP process heap towards the top
@ arg0: amount of SRAM needed
@ returns: pointer (address) of start of the allocated space, or 0 if failed to allocate SRAM
@-----------------------------------
.macro _sbrk
  MRS     r1, PSP             @ Get PSP (Process Stack Pointer)
  
  LDR     r2, =p_brk
  LDR     r2, [r2]            @ Load the address of app system break
  
  ADD     r0, r2, r0          @ Calculate new system break address
  CMP     r0, r1              @ Compare with PSP
  BGE     .exit                @ Return 0 if failed to allocate

  LDR     r2, =p_brk
  STR     r0, [r2]            @ Store the new system break
  .exit:
.endm

@-----------------------------------
@ SYSCALL: Used by the app to collapse the APP process heap towards the bottom freeing memory
@ arg0: amount of SRAM to free
@ returns: pointer (address) of new system break, or 0 on error
@-----------------------------------
.macro _sbrk_free
  LDR     r1, =_edata         @ Load the address of the end of .data in SRAM

  LDR     r2, =p_brk
  LDR     r2, [r2]            @ Load the address of app system break
  
  SUBS    r0, r2, r0          @ Subtract the requested amount of memory from the system break address
  BLE     .exit                @ Return 0 if error (requested amount exceeds current heap size)

  CMP     r0, r1              @ Compare new system break with end of .data
  BLT     .exit                @ Return 0 if error (new system break is below end of .data)

  LDR     r2, =p_brk
  STR     r0, [r2]            @ Store the new system break
  .exit:
.endm


@-----------------------------------
@ Used by the kernel to expand the KERNEL heap towards the top
@ arg0: amount of SRAM needed
@ returns pointer (address) of start of the allocated space
@ returns 0 ((void*)(0x0)) if failed to allocate SRAM
@-----------------------------------
.type _ksbrk, %function
_ksbrk:

  LDR     r2, =k_brk
  LDR     r2, [r2]           @ Load the address of kernel system break
  ADD     r0, r2, r0         @ Add the system break address to the requested amount of memory
  CMP     r0, r12            @ Compare new system break to MSP address (CURRENT DEFAULT SP)
  BGE     .exit               @ Return 0 if failed to allocate

  LDR     r2, =k_brk
  STR     r0, [r2]           @ Store the value of the process's new system break 

  .exit:
    BX      lr
  .align  2
  .size _ksbrk, .-_ksbrk



@-----------------------------------
@ Used by the kernel to collapse the KERNEL heap towards the bottom freeing memory
@ arg0: amount of SRAM to free
@ returns pointer (address) of new system break
@-----------------------------------
.type _ksbrk_free, %function
_ksbrk_free:
  LDR     r1, =_ekdata       @ Load the address of the end of .kdata in SRAM

  LDR     r2, =k_brk
  LDR     r2, [r2]           @ Load the address of kernel system break
  
  SUBS    r0, r2, r0         @ Subtract the requested amount of memory from the system break address
  CMP     r0, r1             @ Compare new system break to end of .kdata
  BLS     .error             @ If new BRK <= _ekdata, return 0

  STR     r0, [r2]           @ Store the value of the KERNEL's new system break 
  CPSIE   i                  @ Enable interrupts (CPSIE i clears PRIMASK)
  BX      lr                 @ Return with the new system break address

  .error:
    MOVS    r0, #0             @ Return 0 on error
    BX      lr

  .align    2
.size _ksbrk_free, .-_ksbrk_free


@ This function can be used by kernel or by app and does not require SVC
@ Function copies 1 word at a time so buffer needs to be 4 bytes aligned
@ Arguments:
@ r0: src (source address)
@ r1: dest (destination address)
@ r2: length (number of bytes to copy, assumed to be multiple of 4)
.global memcpy_4
.type memcpy_4, %function
memcpy_4:
    LDR     r3, [r0], #4      @ Load word from src and increment src by 4
    STR     r3, [r1], #4      @ Store word to dest and increment dest by 4
    SUBS    r2, r2, #4        @ Decrement length counter by 4
    BNE     memcpy_4          @ If length is not 0, continue loop
    BX      lr                @ Return from function
  .align    2
  .size memcpy_4, .-memcpy_4

@ This function can be used by kernel or by app and does not require SVC
@ Function copies 1 byte at a time so no buffer alignment required
@ Arguments:
@ r0: src (source address)
@ r1: dest (destination address)
@ r2: length (number of bytes to copy)
.global memcpy_1
.type memcpy_1, %function
memcpy_1:
    LDRB    r3, [r0], #1      @ Load byte from src and increment src by 1
    STRB    r3, [r1], #1      @ Store byte to dest and increment dest by 1
    SUBS    r2, r2, #1        @ Decrement length counter by 1
    BNE     memcpy_1             @ If length is not 0, continue loop
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
  @ Make a word ready containing 4 bytes of the required byte value
  MOV     r3, r1              @ Move the byte value into r3
  ORR     r3, r3, r3, LSL #8  @ Set byte 2
  ORR     r3, r3, r3, LSL #16 @ Set byte 3 and byte 4
  .loop:
    STR     r3, [r0], #4        @ Store word to dest and increment dest by 4
    SUBS    r2, r2, #4          @ Decrement length counter by 4
    BNE     .loop               @ If length is not 0, continue loop

  .exit:
    BX      lr                  @ Return from function
  .align    2
  .size memset_4, .-memset_4

@ This function can be used by kernel or by app and does not require SVC
@ Function sets 1 byte at a time so no buffer alignment required
@ Arguments:
@ r0: dest (destination address)
@ r1: value (byte value to set)
@ r2: length (number of bytes to set)
.global memset_1
.type memset_1, %function
memset_1:
    STRB    r1, [r0], #1        @ Store byte to dest and increment dest by 1
    SUBS    r2, r2, #1          @ Decrement length counter by 1
    BNE     memset_1               @ If length is not 0, continue loop
    BX      lr                  @ Return from function
  .align    2
    .size memset_1, .-memset_1

@ This function can be used by kernel or by app and does not require SVC
@ It assumes the memory size to zero out is 4 bytes aligned
@ Arguments:
@ r0: dest (destination address)
@ r2: length (number of bytes to zero out, assumed to be multiple of 4)
.global memzero_4
.type memzero_4, %function
memzero_4:
  MOV     r3, #0             @ Load zero into r3
  .loop:
    STR     r3, [r0], #4       @ Store zero to dest and increment dest by 4
    SUBS    r2, r2, #4         @ Decrement length counter by 4
    BNE     .loop              @ If length is not 0, continue loop

  .exit:
    BX      lr                 @ Return from function
  .align    2
    .size memzero_4, .-memzero_4

@ This function can be used by kernel or by app and does not require SVC
@ Arguments:
@ r0: dest (destination address)
@ r2: length (number of bytes to zero out)
.global memzero_1
.type memzero_1, %function
memzero_1:
  MOV     r3, #0             @ Load zero into r3
  .loop:
    STRB    r3, [r0], #1       @ Store zero to dest and increment dest by 1
    SUBS    r2, r2, #1         @ Decrement length counter by 1
    BNE     .loop              @ If length is not 0, continue loop

  .exit:
    BX      lr                 @ Return from function
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
    LDRB    r3, [r0], #1   @ Load byte from src1 and increment src1
    LDRB    r4, [r1], #1   @ Load byte from src2 and increment src2
    CMP     r3, r4         @ Compare bytes
    BNE     .fail          @ Exit loop if bytes are not equal
    SUBS    r2, r2, #1     @ Decrement length counter
    BNE     memcmp_1          @ Loop if length is not zero
    MOV     r0, #0         @ If all bytes are equal, return 0
    BX      lr
  .fail:
    MOV     r0, #1         @ If bytes are not equal, return non-zero
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
    LDR     r3, [r0], #4   @ Load word from src1 and increment src1 by 4
    LDR     r4, [r1], #4   @ Load word from src2 and increment src2 by 4
    CMP     r3, r4         @ Compare words
    BNE     .fail          @ Exit loop if words are not equal
    SUBS    r2, r2, #4     @ Decrement length counter by 4
    BGE     memcmp_4          @ Loop if length is not zero or negative
    MOV     r0, #0         @ If all words are equal, return 0
    BX      lr             @ Return from function
  .fail:
    MOV     r0, #1         @ If words are not equal, return non-zero
    BX      lr             @ Return from function
  .align    2
  .size memcmp_4, .-memcmp_4


.macro SVC_HANDLERS
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
  BX      lr

SVC5_Handler:
  _NVIC_set_prio_irq
  BX      lr

SVC6_Handler:
  _NVIC_get_prio_irq
  BX      lr

SVC7_Handler:
  _NVIC_soft_trigger_irq
  BX      lr

@-------Memory management--------@
SVC8_Handler:
  _sbrk
  BX      lr

SVC9_Handler:
  _sbrk_free
  BX      lr

SVC10_Handler:
  BX      lr

SVC11_Handler:
  BX      lr

SVC12_Handler:
  BX      lr

SVC13_Handler:
  BX      lr

SVC14_Handler:
  BX      lr

@-------SCB--------@
SVC15_Handler:
  DIS_OUTOFORDER_EXEC #0
  BX      lr

SVC16_Handler:
  GET_CPUID
  BX      lr

SVC17_Handler:
  NMI_set_pending
  BX      lr

SVC18_Handler:
  PENDSV_set_pending
  BX      lr

SVC19_Handler:
  PENDSV_clear_pending
  BX      lr

SVC20_Handler:
  SYSTICK_set_pending
  BX      lr

SVC21_Handler:
  SYSTICK_clear_pending
  BX      lr

SVC22_Handler:
  SYSTICK_check_pending
  BX      lr

SVC23_Handler:
  ISR_check_pending
  BX      lr

SVC24_Handler:
  prio_split16_0
  BX      lr

SVC25_Handler:
  prio_split8_2
  BX      lr

SVC26_Handler:
  prio_split4_4
  BX      lr

SVC27_Handler:
  prio_split2_8
  BX      lr

SVC28_Handler:
  prio_split0_16
  BX      lr

SVC29_Handler:
  reset_request
  BX      lr

SVC30_Handler:
  sev_on_pend
  BX      lr

SVC31_Handler:
  sleep_deep
  BX      lr

SVC32_Handler:
  sleep_on_exit
  BX      lr

SVC33_Handler:
  stack_align_4
  BX      lr

SVC34_Handler:
  stack_align_8
  BX      lr

SVC35_Handler:
  NMI_HARDFAULT_dis_bus_fault_handling
  BX      lr

SVC36_Handler:
  NMI_HARDFAULT_en_bus_fault_handling
  BX      lr

SVC37_Handler:
  div0_notrap
  BX      lr

SVC38_Handler:
  div0_trap
  BX      lr

SVC39_Handler:
  unalign_notrap
  BX      lr

SVC40_Handler:
  unalign_trap
  BX      lr

SVC41_Handler:
  app_access_STIR
  BX      lr

SVC42_Handler:
  exit_nested_irqs_on_return
  BX      lr

SVC43_Handler:
  set_UsageFault_prio 
  BX      lr

SVC44_Handler:
  set_MemMan_fault_prio
  BX      lr

SVC45_Handler:
  set_SVC_prio
  BX      lr

SVC46_Handler:
  set_SYSTICK_prio
  BX      lr

SVC47_Handler:
  set_PendSV_prio
  BX      lr

SVC48_Handler:
  enable_UsageFault
  BX      lr

SVC49_Handler:
  enable_BusFault
  BX      lr

SVC50_Handler:
  enable_MemMan_fault
  BX      lr

SVC51_Handler:
  disable_UsageFault
  BX      lr

SVC52_Handler:
  disable_BusFault
  BX      lr

SVC53_Handler:
  disable_MemMan_fault
  BX      lr

SVC54_Handler:
  is_SVC_pending
  BX      lr

SVC55_Handler:
  is_BusFault_pending
  BX      lr

SVC56_Handler:
  is_MemMan_fault_pending
  BX      lr

SVC57_Handler:
  is_UsageFault_pending
  BX      lr

SVC58_Handler:
  is_SYSTICK_active
  BX      lr

SVC59_Handler:
  is_PendSV_active
  BX      lr

SVC60_Handler:
  is_DBGMon_active
  BX      lr

SVC61_Handler:
  is_SVC_active
  BX      lr

SVC62_Handler:
  is_UsageFault_active
  BX      lr

SVC63_Handler:
  is_BusFault_active
  BX      lr

SVC64_Handler:
  is_MemMan_fault_active
  BX      lr

SVC65_Handler:
  div_by0_UsageFault
  BX      lr

SVC66_Handler:
  unalignement_UsageFault
  BX      lr

SVC67_Handler:
  coprocessor_UsageFault
  BX      lr

SVC68_Handler:
  invPC_UsageFault
  BX      lr

SVC69_Handler:
  invEPSR_UsageFault
  BX      lr

SVC70_Handler:
  BFAR_valid_addr
  BX      lr

SVC71_Handler:
  FP_LazyState_BusFault
  BX      lr

SVC72_Handler:
  push_BusFault
  BX      lr

SVC73_Handler:
  pop_BusFault
  BX      lr

SVC74_Handler:
  imprecise_BusFault
  BX      lr

SVC75_Handler:
  precise_DBus_error
  BX      lr

SVC76_Handler:
  IBus_error
  BX      lr

SVC77_Handler:
  MMAR_valid_addr
  BX      lr

SVC78_Handler:
  FP_LazyState_MemMan_fault
  BX      lr

SVC79_Handler:
  push_MemMan_fault
  BX      lr

SVC80_Handler:
  pop_MemMan_fault
  BX      lr

SVC81_Handler:
  DataAccess_MemMan_fault
  BX      lr

SVC82_Handler:
  ExecNot_section_MemMan_fault
  BX      lr

SVC83_Handler:
  forced_HardFault
  BX      lr

SVC84_Handler:
  push_MemMan_fault
  BX      lr

SVC85_Handler:
  vect_table_HardFault
  BX      lr

SVC86_Handler:
  get_MemManFault_addr
  BX      lr

SVC87_Handler:
  get_BusFault_addr
  BX      lr

SVC88_Handler:
  get_AuxFault_addr
  BX      lr

SVC89_Handler:
  unassigned
  BX      lr

SVC90_Handler:
  unassigned
  BX      lr

SVC91_Handler:
  unassigned
  BX      lr

SVC92_Handler:
  unassigned
  BX      lr

SVC93_Handler:
  unassigned
  BX      lr

SVC94_Handler:
  unassigned
  BX      lr

SVC95_Handler:
  unassigned
  BX      lr

SVC96_Handler:
  unassigned
  BX      lr

SVC97_Handler:
  unassigned
  BX      lr

SVC98_Handler:
  unassigned
  BX      lr

SVC99_Handler:
  unassigned
  BX      lr
.endm

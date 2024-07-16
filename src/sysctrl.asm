
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


@-----------------------------------------------------
@----------------------------------------------------- Memory 
@-----------------------------------------------------


@ This function can be used by kernel or by app and does not require SVC
@ Function copies 1 word at a time so buffer needs to be 4 bytes aligned
@ Arguments:
@ r0: src (source address)
@ r1: dest (destination address)
@ r2: length (number of bytes to copy, assumed to be multiple of 4)
.global memcpy_4
.type memcpy_4, %function
memcpy_4:
  .loop:
    LDR     r3, [r0], #4      @ Load word from src and increment src by 4
    STR     r3, [r1], #4      @ Store word to dest and increment dest by 4
    SUBS    r2, r2, #4        @ Decrement length counter by 4
    BNE     .loop             @ If length is not 0, continue loop
    BX      lr                @ Return from function
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
  .loop:
    LDRB    r3, [r0], #1      @ Load byte from src and increment src by 1
    STRB    r3, [r1], #1      @ Store byte to dest and increment dest by 1
    SUBS    r2, r2, #1        @ Decrement length counter by 1
    BNE     .loop             @ If length is not 0, continue loop
    BX      lr                @ Return from function
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
  .loop:
    STRB    r1, [r0], #1        @ Store byte to dest and increment dest by 1
    SUBS    r2, r2, #1          @ Decrement length counter by 1
    BNE     .loop               @ If length is not 0, continue loop
    BX      lr                  @ Return from function
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
  .loop:
    LDRB    r3, [r0], #1   @ Load byte from src1 and increment src1
    LDRB    r4, [r1], #1   @ Load byte from src2 and increment src2
    CMP     r3, r4         @ Compare bytes
    BNE     .fail          @ Exit loop if bytes are not equal
    SUBS    r2, r2, #1     @ Decrement length counter
    BNE     .loop          @ Loop if length is not zero
    MOV     r0, #0         @ If all bytes are equal, return 0
    BX      lr
  .fail:
    MOV     r0, #1         @ If bytes are not equal, return non-zero
    BX      lr
  .size memcmp81, .-memcmp_1

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
.global memcmp
.type memcmp, %function
memcmp:
  .loop:
    LDR     r3, [r0], #4   @ Load word from src1 and increment src1 by 4
    LDR     r4, [r1], #4   @ Load word from src2 and increment src2 by 4
    CMP     r3, r4         @ Compare words
    BNE     .fail          @ Exit loop if words are not equal
    SUBS    r2, r2, #4     @ Decrement length counter by 4
    BGE     .loop          @ Loop if length is not zero or negative
    MOV     r0, #0         @ If all words are equal, return 0
    BX      lr             @ Return from function
  .fail:
    MOV     r0, #1         @ If words are not equal, return non-zero
    BX      lr             @ Return from function
  .size memcmp, .-memcmp



@-----------------------------------------------------
@----------------------------------------------------- SCB 
@-----------------------------------------------------

/*--------ACTLR---------*/
/*--------ACTLR---------*/

@ Macro used for configuring the various regions used by the system
@ state: 1 to disable  or 0 to enable
.macro DIS_OUTOFORDER_EXEC state:req
  LDR     r0, =ACTLR
  LDR     r1, [r0]
  MOVW    r3, 0x207     @ mask for disabling features
  LDR     r2, =\state
  CBZ     r2, .enable   @ check if state is 0 (enable)
  .disable:
  @ Disable features: Set bits
  ORR     r1, r1, r3
  B       .exit
  .enable:
  @ Enable features: Clear bits
  BIC     r1, r1, r3
  .exit:
  STR     r1, [r0]      @ write back the modified value to ACTLR
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

.macro PENDSV_set_pending:
  SET_PENDING_BIT 0x10000000 @ Set the PENDSV pending bit (bit 28)
.endm

.macro PENDSV_clear_pending:
  SET_PENDING_BIT 0x8000000 @ Clear the PENDSV pending bit (bit 27)
.endm

.macro PENDSV_check_pending:
  CHECK_PENDING_BIT 0x10000000 @ Check the PENDSV pending bit (bit 28)
.endm

.macro SYSTICK_set_pending:
  SET_PENDING_BIT 0x4000000 @ Set the SYSTICK pending bit (bit 26)
.endm

.macro SYSTICK_clear_pending:
  SET_PENDING_BIT 0x2000000 @ Clear the SYSTICK pending bit (bit 25)
.endm

.macro SYSTICK_check_pending:
  CHECK_PENDING_BIT 0x4000000 @ Check the SYSTICK pending bit (bit 26)
.endm

.macro ISR_check_pending:
  CHECK_PENDING_BIT 0x400000 @ Check if any ISR is pending excluding NMI and Faults.
.endm

.macro ISR_highest_pending:
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
.macro NMI_HARDFAULT_en_bus_fault_handling
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


/*--------SHCSR---------*/
/*--------SHCSR---------*/
@ Configurable fault status registers (CFSR; UFSR+BFSR+MMFSR)


@-----------------------------------------------------
@----------------------------------------------------- Tasks 
@-----------------------------------------------------


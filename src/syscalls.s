
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

@ Some of thses functions will be called by the APP thru SVC
@ Others will be called directly by the kernel code
@ Others are available for both
.section .text.syscalls, "ax", %progbits

@-----------------------------------------------------
@-----------------------------------------------------NVIC syscalls
@----------------------------------------------------- 

@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to enable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_enable_irq, %function
_NVIC_enable_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT0_7  r0, NVIC_ISER0  @ Select the appropriate NVIC_ISER register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr
  .align  4
  .size _NVIC_enable_irq, .-_NVIC_enable_irq

  
@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to disable an interrupt
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_disable_irq, %function
_NVIC_disable_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT0_7  r0, NVIC_ICER0   @ Select the appropriate NVIC_ICER register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr
  .align  4
  .size _NVIC_disable_irq, .-_NVIC_disable_irq


@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to set an interrupt as pending
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_set_pend_irq, %function
_NVIC_set_pend_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT0_7  r0, NVIC_ISPR0   @ Select the appropriate NVIC_ISPR register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr
  .align  4
  .size _NVIC_set_pend_irq, .-_NVIC_set_pend_irq


@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to remove an interrupt from pending list
@ arg0: number of the IRQ (0..239)
@-----------------------------------
  .type _NVIC_clear_pend_irq, %function
_NVIC_clear_pend_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT0_7  r0, NVIC_ICPR0   @ Select the appropriate NVIC_ICPR register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  ORR     r1, r1, r3            @ Set the bit of the IRQ
  STR     r1, [r2]              @ store the mask in the NVIC_ISER
  MOV     r0, #0
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr
  .align  4
  .size _NVIC_clear_pend_irq, .-_NVIC_clear_pend_irq


@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to check interrupt if the interrupt is active
@ arg0: number of the IRQ (0..239)
@ return: 1 if active / 0 if idle
@-----------------------------------
  .type _NVIC_check_active_irq, %function
_NVIC_check_active_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  @ Macro sets the address of the register in r2
  @ Normalizes the irq num in r0 to the start of register
  NVIC_REG_SELECT0_7  r0, NVIC_IABR0   @ Select the appropriate NVIC_IABR register
  MOV     r3, #0b1
  LSL     r3, r3, r0            @ shift the mask to the IRQ bit position
  LDR     r1, [r2]              @ load the value of the NVIC_ISER
  TST     r1, r3                @ check if bit is set (IRQ active)
  ITE     NE
  MOVNE   r0, #1                @ bit is set (IRQ active)
  MOVEQ   r0, #0                @ bit is not set (IRQ idle)
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr
  .align  4
  .size _NVIC_check_active_irq, .-_NVIC_check_active_irq
  

@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to set the priority of the interrupt
@ arg0: number of the IRQ (0..239)
@ arg1: priority number
@-----------------------------------
.type _NVIC_set_pri_irq, %function
_NVIC_set_pri_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  NVIC_REG_SELECT0_59 r0, NVIC_IPR0   @ Select the appropriate NVIC_IPR register
  STRB    r1, [r2]                @ Store the priority number in the selected register byte
  MOV     r0, #0
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr                      @ Return from the function
  .align  4
  .size _NVIC_set_pri_irq, .-_NVIC_set_pri_irq


@-----------------------------------SYSCALL
@ syscall used by apps (called by SVC)
@ called by software to get the priority of the interrupt
@ arg0: number of the IRQ (0..239)
@ return: priority number of the IRQ
@-----------------------------------
.type _NVIC_get_pri_irq, %function
_NVIC_get_pri_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  NVIC_REG_SELECT0_59 r0, NVIC_IPR0   @ Select the appropriate NVIC_IPR register
  LDRB    r0, [r2]                    @ Load the priority number from the selected register byte
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr                          @ Return from the function
  .align  4
  .size _NVIC_get_pri_irq, .-_NVIC_get_pri_irq


@-----------------------------------SYSCALL
@ function used directly by apps or thru syscall
@ access to this register can be thru unpriviledged thread mode
@ check SCR reg in page 230 of the stm32-cortex-M4 Referance Manual
@ called by software to trigger an interrupt on the mask specified in arg0
@ arg0: IRQ number (0..239)
@-----------------------------------
  .type _NVIC_soft_trigger_irq, %function
_NVIC_soft_trigger_irq:
  MOV     r1, #1
  MSR     PRIMASK, r1        @ Disable interrupts
  LDR     r1, =NVIC_STIR
  STR     r0, [r1]
  MOV     r0, #0
  MOV     r1, #0
  MSR     PRIMASK, r1        @ Enable interrupts
  BX      lr
  .align  4
  .size _NVIC_soft_trigger_irq, .-_NVIC_soft_trigger_irq

@-----------------------------------------------------
@----------------------------------------------------- system control syscalls
@----------------------------------------------------- 



@-----------------------------------------------------
@----------------------------------------------------- Memory Management syscalls
@-----------------------------------------------------

@-----------------------------------
@ SYSCALL: Used by the app to expand the APP process heap towards the top
@ arg0: amount of SRAM needed
@ returns: pointer (address) of start of the allocated space, or 0 if failed to allocate SRAM
@-----------------------------------
.type _sbrk, %function
_sbrk:
  MRS     r1, PSP             @ Get PSP (Process Stack Pointer)
  
  LDR     r2, =p_brk
  LDR     r2, [r2]            @ Load the address of app system break
  
  ADD     r0, r2, r0          @ Calculate new system break address
  CMP     r0, r1              @ Compare with PSP
  BGE     .exit                @ Return 0 if failed to allocate

  LDR     r2, =p_brk
  STR     r0, [r2]            @ Store the new system break

  .exit:
    BX      lr
  .align  4
  .size _sbrk, .-_sbrk

@-----------------------------------
@ SYSCALL: Used by the app to collapse the APP process heap towards the bottom freeing memory
@ arg0: amount of SRAM to free
@ returns: pointer (address) of new system break, or 0 on error
@-----------------------------------
.type _sbrk_free, %function
_sbrk_free:

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
    BX      lr
  .align  4
  .size _sbrk_free, .-_sbrk_free



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
  .align  4
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

.align 4
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
    BNE     .exit          @ Exit loop if bytes are not equal
    SUBS    r2, r2, #1     @ Decrement length counter
    BNE     .loop          @ Loop if length is not zero
    MOV     r0, #0         @ If all bytes are equal, return 0
    BX      lr
  .exit:
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
    BNE     .exit          @ Exit loop if words are not equal
    SUBS    r2, r2, #4     @ Decrement length counter by 4
    BGE     .loop          @ Loop if length is not zero or negative
    MOV     r0, #0         @ If all words are equal, return 0
    BX      lr             @ Return from function
  .exit:
    MOV     r0, #1         @ If words are not equal, return non-zero
    BX      lr             @ Return from function
  .size memcmp, .-memcmp

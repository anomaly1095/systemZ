
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
  #include "../../../include/define.asm"
  
@ MPU register details provided in STM32F401RE Referance Manual page 60
@----------------------------------------------------------------------
@---------------------------------------------------------------------- SYSTEM INIT section
.section .text.sysinit, "ax", %progbits

  .define DEVELOPMENT_MODE
  @ .define PRODUCTION_MODE

@-----------------------------------
@ main function called by system initialization to configure the FLASH
@-----------------------------------
  .global _FLASH_config
  .type _FLASH_config, %function
_FLASH_config:
  PUSH    {lr}
  LDR     r0, =FLASH_BASE
  BL      _FLASH_access_control
  BL      _FLASH_cr_optcr_config
  POP     {lr}
  MOV     r0, #0      @ RETURN 0
  BX      lr
  .align 4
  .size _FLASH_config, .-_FLASH_config

@-----------------------------------
@ reset & enable caches, enablke frefetch
@ Set latency at 2ws for 84mhz AHB clock
@-----------------------------------
  .type _FLASH_access_control, %function
_FLASH_access_control:
  LDR     r1, [r0]
  MOVW    r2, #0b11       @ reset both instruction and data caches
  ORR     r1, r1, r1
  STR     r1, [r0]        @ save changes
  LDR     r1, [r0]
  MOVW    r2, #0x0702     @ enable caches, prefetch and set latency = 2Ws
  ORR     r1, r1, r1
  STR     r1, [r0]        @ save changes
  BX      lr
  .align 4
  .size _FLASH_access_control, .-_FLASH_access_control


@-----------------------------------
@ set flash options
@-----------------------------------
  .type _FLASH_cr_optcr_config, %function
_FLASH_cr_optcr_config:
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
    PUSH     {lr}
    BL       __FLASH_opt_config      @ configure FLASH option bytes
    POP      {lr}
  .endif

  BX      lr                      @ Return 

  .align 4
  .size _FLASH_cr_optcr_config, .-_FLASH_cr_optcr_config

@-----------------------------------
@ write flash options
@-----------------------------------
  .type __FLASH_opt_config, %function
__FLASH_opt_config:
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
  
  @ If PCROP is not needed, skip the following two lines
  MOV     r2, #1
  ORR     r2, r2, #1              @ r2 = 0x80000000
  ORR     r1, r1, r2              @ Set SPRMOD (optional)
  
  @ set BOR (brownout level)
  MOV     r2, #(0b11 << 2)
  BIC     r1, r1, r2              @ cler BOR_LEV bits
  MOV     r2, #(0b01 << 2)
  ORR     r1, r1, r2              @ set new BOR_LEV bits at BOR level 2

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
  BX      lr
  .align 4
  .size __FLASH_opt_config, .-__FLASH_opt_config

@----------------------------------------------------------------------
@----------------------------------------------------------------------
@ syscalls thru SVC
.section .text.drivers.FLASH, "ax", %progbits





.section .rodata.registers.FLASH, "a", %progbits
  .equ FLASH_BASE, 0x40023C00       @ FLASH base address
  .equ FLASH_KEY1, 0x45670123
  .equ FLASH_KEY2, 0xCDEF89AB
  .equ FLASH_OPTKEY1, 0x08192A3B
  .equ FLASH_OPTKEY2, 0x4C5D6E7F

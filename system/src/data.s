
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


@------------------------------------------------------------
@------------------------------------------------------------
@------------------------------------------------------------.data
@------------------------------------------------------------
@------------------------------------------------------------

@ data section used by syscalls
.section .data.system, "aw", %progbits

@ Application Process system break
p_brk:
  .word _sheap 
@ kernel system break
k_brk:
  .word _skheap

  .align 2

@------------------------------------------------------------
@------------------------------------------------------------
@------------------------------------------------------------.bss
@------------------------------------------------------------
@------------------------------------------------------------

@ uninitialized data section used by syscalls
.section .bss.system, "aw", %nobits

@ Pointer to the first free block in app heap
free_blocks: .word

@ Pointer to the first free block in kernel heap
kfree_blocks: .word

@ array of linker list head pointers for each external IRQ (55*4 = 220bytes)
ll_heads:   .space 220

stk_cntrs:
  .short     @ Milliseconds used by systick
  .word      @ Seconds used by systick

@ Global variable to hold last exception number, initialized to 0
last_IRQ: 
  .byte      @ will hold the last IRQ number and written only by ISR_get_active_num

  .align 2

@------------------------------------------------------------
@------------------------------------------------------------
@------------------------------------------------------------.rodata
@------------------------------------------------------------
@------------------------------------------------------------



.section .rodata.SVC_handlers, "a", %progbits

@ number of SVC handlers
.equ NUM_SVC_HANDLERS, 100
@ Noide size for linked lists: 4 bytes for size and 4 bytes for next pointer
.equ NODE_SIZE, 8

@ generate NUM_SVC_HANDLERS iterations of extern for each SVC_HANDLER
.macro SVC_HANDLERS_TABLE
  SVC_Table:
    .rept NUM_SVC_HANDLERS
    .extern SVC\@_Handler
    .word SVC\@_Handler
    .endr
.endm

SVC_HANDLERS_TABLE
.align 2

.section .rodata.registers.SCB, "a", %progbits
@      Name   Address           Type    Req privilege Reset val
 .equ  ACTLR,  0xE000E008        @ RW    Privileged    0x00000000
 .equ  CPUID,  0xE000ED00        @ RO    Privileged    0x410FC241
 .equ  ICSR,   0xE000ED04        @ RW    Privileged    0x00000000
 .equ  VTOR,   0xE000ED08        @ RW    Privileged    0x00000000
 .equ  AIRCR,  0xE000ED0C        @ RW    Privileged    0xFA050000
 .equ  SCR,    0xE000ED10        @ RW    Privileged    0x00000000
 .equ  CCR,    0xE000ED14        @ RW    Privileged    0x00000200
 .equ  SHPR1,  0xE000ED18        @ RW    Privileged    0x00000000
 .equ  SHPR2,  0xE000ED1C        @ RW    Privileged    0x00000000
 .equ  SHPR3,  0xE000ED20        @ RW    Privileged    0x00000000
 .equ  SHCSR,  0xE000ED24        @ RW    Privileged    0x00000000
 .equ  CFSR,   0xE000ED28        @ RW    Privileged    0x00000000
 .equ  MMSR,   0xE000ED28        @ RW    Privileged    0x00
 .equ  BFSR,   0xE000ED29        @ RW    Privileged    0x00
 .equ  UFSR,   0xE000ED2A        @ RW    Privileged    0x0000
 .equ  HFSR,   0xE000ED2C        @ RW    Privileged    0x00000000
 .equ  MMAR,   0xE000ED34        @ RW    Privileged    Unknown
 .equ  BFAR,   0xE000ED38        @ RW    Privileged    Unknown
 .equ  AFSR,   0xE000ED3C        @ RW    Privileged    0x00000000

@-----------------------------------------------
.section .rodata.registers.NVIC, "a", %progbits
  .equ NVIC_ISER0, 0xE000E100     @ 7 register
  .equ NVIC_ICER0, 0xE000E180     @ 7 register
  .equ NVIC_ISPR0, 0xE000E200     @ 7 register
  .equ NVIC_ICPR0, 0xE000E280     @ 7 register
  .equ NVIC_IABR0, 0xE000E300     @ 7 register
  .equ NVIC_IPR0,  0xE000E400     @ 59 register
  .equ NVIC_STIR,  0xE000EF00     @ 1 register
@-----------------------------------------------
.section .rodata.registers.SYSTICK, "a", %progbits
  .equ SYSTCK_BASE, 0xE000E010
  .equ SYSTICK_COUNTER, 10499   @ value to be laded in STK_LOAD
@-----------------------------------------------
  .section .rodata.registers.FLASH, "a", %progbits
  .equ FLASH_BASE, 0x40023C00      @ FLASH base address
  .equ FLASH_KEY1, 0x45670123
  .equ FLASH_KEY2, 0xCDEF89AB
  .equ FLASH_OPTKEY1, 0x08192A3B
  .equ FLASH_OPTKEY2, 0x4C5D6E7F
@------------------------------------------------
.section .rodata.registers.PWR, "a", %progbits
  .equ PWR_BASE, 0x40007000
  .equ PWR_CR_MASK, 0x8EFD
@-----------------------------------------------
.section .rodata.registers.RCC, "a", %progbits
  .equ RCC_BASE, 0x40023800       @ RCC bit-band base address
  .equ RCC_BASE_BB, 0x42470000    @ RCC base address
  .equ RCC_PLLCFGR_MASK, 0xF437FFF
@-----------------------------------------------
.section .rodata.registers.MPU, "a", %progbits
  .equ MPU_BASE, 0xE000ED90

  @----------------------------- System Peripheral Space
  .equ SECTION7_SIZE, 0x00100000    @ 1MB
  .equ SECTION7_BASE, 0xE0000000
  @                     XN = 1    |   AP =  001   |  TEX =  000   |   S =  1    |   C = 0     |   B = 0     |SRD=00000000| SIZE = 19 | ENABLE 
  .equ SECTION7_MASK, (0b1 << 28) | (0b001 << 24) | (0b000 << 19) | (0b1 << 18) | (0b0 << 17) | (0b0 << 16) | (0x0 << 8) | (19 << 1) | 0b1
  
  @----------------------------- Bit-Band Area of DMA Controller
  .equ SECTION6_SIZE, 0x00020000    @ 128KB
  .equ SECTION6_BASE, 0x424C0000
  @                     XN = 1    |   AP =  010   |  TEX =  000   |   S =  1    |   C = 0     |   B = 1     |SRD=00000000| SIZE = 16 | ENABLE 
  .equ SECTION6_MASK, (0b1 << 28) | (0b010 << 24) | (0b000 << 19) | (0b1 << 18) | (0b0 << 17) | (0b1 << 16) | (0x0 << 8) | (16 << 1) | 0b1
  
  @----------------------------- DMA Controller
  .equ SECTION5_SIZE, 0x00001000    @ 4KB
  .equ SECTION5_BASE, 0x40026000
  @                     XN = 1    |   AP =  010   |  TEX =  000   |   S =  1    |   C = 0     |   B = 1     |SRD=00000000| SIZE = 11 | ENABLE 
  .equ SECTION5_MASK, (0b1 << 28) | (0b010 << 24) | (0b000 << 19) | (0b1 << 18) | (0b0 << 17) | (0b1 << 16) | (0x0 << 8) | (11 << 1) | 0b1

  @----------------------------- Peripheral Registers
  .equ SECTION4_SIZE, 0x20000000    @ 512MB
  .equ SECTION4_BASE, 0x40000000
  @                     XN = 1    |   AP =  011   |  TEX =  000   |   S =  1    |   C = 0     |   B = 1     |SRD=00000000| SIZE = 28 | ENABLE 
  .equ SECTION4_MASK, (0b1 << 28) | (0b011 << 24) | (0b000 << 19) | (0b1 << 18) | (0b0 << 17) | (0b1 << 16) | (0x0 << 8) | (28 << 1) | 0b1

  @----------------------------- Bit-Band Area of KERNEL SRAM
  .equ SECTION3_SIZE, 0x00100000    @ 1MB
  .equ SECTION3_BASE, 0x22200000
  @                     XN = 1    |   AP =  001   |  TEX =  001   |   S =  0    |   C = 0     |   B = 0     |SRD=00000000| SIZE = 19 | ENABLE 
  .equ SECTION3_MASK, (0b1 << 28) | (0b001 << 24) | (0b001 << 19) | (0b0 << 18) | (0b0 << 17) | (0b0 << 16) | (0x0 << 8) | (19 << 1) | 0b1

  @----------------------------- KERNEL SRAM
  .equ SECTION2_SIZE, 0x00008000    @ 32KB
  .equ SECTION2_BASE, 0x20010000
  @                     XN = 0    |   AP =  001   |  TEX =  001   |   S =  0    |   C = 1     |   B = 1     |SRD=00000000| SIZE = 14 | ENABLE 
  .equ SECTION2_MASK, (0b0 << 28) | (0b001 << 24) | (0b001 << 19) | (0b0 << 18) | (0b1 << 17) | (0b1 << 16) | (0x0 << 8) | (14 << 1) | 0b1

  @----------------------------- SRAM
  .equ SECTION1_SIZE, 0x20000000    @ 512MB
  .equ SECTION1_BASE, 0x20000000
  @                     XN = 1    |   AP =  011   |  TEX =  000   |   S =  1    |   C = 1     |   B = 0     |SRD=00000000| SIZE = 28 | ENABLE 
  .equ SECTION1_MASK, (0b1 << 28) | (0b011 << 24) | (0b000 << 19) | (0b1 << 18) | (0b1 << 17) | (0b0 << 16) | (0x0 << 8) | (28 << 1) | 0b1

  @----------------------------- FLASH for kernel and apps
  .equ SECTION0_SIZE, 0x20000000    @ 512MB
  .equ SECTION0_BASE, 0x00000000
  @                     XN = 0    |   AP =  010   |  TEX =  000   |   S =  1    |   C = 1     |   B = 0     |SRD=00000000| SIZE = 28 | ENABLE 
  .equ SECTION0_MASK, (0b0 << 28) | (0b010 << 24) | (0b000 << 19) | (0b1 << 18) | (0b1 << 17) | (0b0 << 16) | (0x0 << 8) | (28 << 1) | 0b1


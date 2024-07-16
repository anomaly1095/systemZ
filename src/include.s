
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
.section .data.syscalls, "aw", %progbits
  .equ SVC_MASK, 0xFFFFFF00
@ callback array for SYSTICK
stk_clbk:
  .word 0x0  @ add the functions to be called by systick here
  .word 0x0  @ add the functions to be called by systick here
  .word 0x0  @ add the functions to be called by systick here
  .word 0x0  @ add the functions to be called by systick here

p_brk:
  .word _sheap       @ Application Process system break 
k_brk:
  .word _skheap      @ kernel system break

  .align 4

@------------------------------------------------------------
@------------------------------------------------------------
@------------------------------------------------------------.bss
@------------------------------------------------------------
@------------------------------------------------------------

@ uninitialized data section used by syscalls
.section .bss.syscalls, "aw", %progbits

stk_cntrs:
  .short     @ Milliseconds used by systick
  .word      @ Seconds used by systick

@ Global variable to hold last exception number, initialized to 0
last_IRQ: 
  .byte      @ will hold the last IRQ number and written only by ISR_get_active_num

  .align 4

@------------------------------------------------------------
@------------------------------------------------------------
@------------------------------------------------------------Macros
@------------------------------------------------------------
@------------------------------------------------------------

.define LITTLE_ENDIAN
.define DEVELOPMENT_MODE 
@ .define PRODUCTION_MODE

@ Macro used for configuring the various regions used by the system
.macro MPU_CONFIG_REGION region_base:req, region_number:req, region_mask:req
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
@ example: NVIC_REG_SELECT0_7 5, 0xE000E100 
@-----------------------------------
.macro NVIC_REG_SELECT0_7 irq_num:req, NVIC0_addr:req
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
.macro NVIC_REG_SELECT0_59 irq_num:req, NVIC0_IPR_addr:req
  LDR     r2, =\NVIC0_IPR_addr     @ Load the base address into r2
  LSR     \irq_num, \irq_num, #2   @ Divide irq_num by 4 to get the byte offset
  ADD     r2, r2, \irq_num         @ Add the offset to the base address
.endm


@------------------------------------------------------------
@------------------------------------------------------------
@------------------------------------------------------------.rodata
@------------------------------------------------------------
@------------------------------------------------------------


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


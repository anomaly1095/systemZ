      
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

@ MPU register details provided in ARM cortex-M7 Referance Manual page 200

.section .text.drivers.RCC, "ax", %progbits

@-----------------------------------
@ main function called by system initialization to configure the MPU
@-----------------------------------
  .global _MPU_config
  .type _MPU_config, %function
_MPU_config:
  PUSH    {lr}
  LDR     r0, =MPU_BASE
  BL      _MPU_type  
  BL      _MPU_sections_config
  BL      _MPU_enable
  POP     {lr}
  BX      lr
  .size _MPU_config, .-_MPU_config

  
@-----------------------------------
@ Check for system's support for MPU
@ Check support for separate or unified sections
@-----------------------------------
  .type _MPU_type, %function
_MPU_type:
  LDR     r1, [r0]
  CMP     r2, #0
  ITTE    EQ
  POP     {lr}          @ if not MPU or no separate sections possibility 
  BXEQ    lr            @ we return to system init
  BX      lr
  .size _MPU_type, .-_MPU_type


@-----------------------------------
@ Configure the 8 sections for the system
@ (macro used is defined in src/common/macros.asm)
@-----------------------------------
  .type _MPU_sections_config, %function
_MPU_sections_config:
  @ Configure REGION0: 0x00000000 to 0x1FFFFFFF (FLASH for kernel and apps)
  MPU_CONFIG_REGION SECTION0_BASE, 0, SECTION0_MASK

  @ Configure REGION1: 0x20000000 to 0x3FFFFFFF (SRAM)
  MPU_CONFIG_REGION SECTION1_BASE, 1, SECTION1_MASK
  
  @ Configure REGION2: 0x20010000 to 0x20017FFF (Kernel SRAM)
  MPU_CONFIG_REGION SECTION2_BASE, 2, SECTION2_MASK

  @ Configure REGION3: 0x22200000 to 0x222FFFFF (Bit-Band Area of Kernel SRAM)
  MPU_CONFIG_REGION SECTION3_BASE, 3, SECTION3_MASK
  
  @ Configure REGION4: 0x40000000 to 0x5FFFFFFF (Peripheral Registers)
  MPU_CONFIG_REGION SECTION4_BASE, 4, SECTION4_MASK
  
  @ Configure REGION5: 0x40026000 to 0x40026FFF (DMA Controller)
  MPU_CONFIG_REGION SECTION5_BASE, 5, SECTION5_MASK

  @ Configure REGION6: 0x424C0000 to 0x424DFFFF (Bit-Band Area of DMA Controller)
  MPU_CONFIG_REGION SECTION6_BASE, 6, SECTION6_MASK

  @ Configure REGION7: 0xE0000000 to 0xE00FFFFF (System Peripheral Space)
  MPU_CONFIG_REGION SECTION7_BASE, 7, SECTION7_MASK
  @ return
  BX      lr
  .size _MPU_sections_config, .-_MPU_sections_config
  

@-----------------------------------
@ Check for system's support for MPU
@ Check support for separate or unified sections
@-----------------------------------
_MPU_enable:
  .type _MPU_enable, %function
  @  Enable background map, enable mpu during NMI AND FAULTS, enable MPU
  LDR     r1, [r0, #0x04]       @ MPU_CTRL reg
  MOV     r2, #0b111
  ORR     r1, r2                @ Set ENABLE, PRIVDEFENA, and HFNMIENA bits
  STR     r1, [r0, #0x04]       @ MPU_CTRL reg
  BX      lr
  .size _MPU_enable, .-_MPU_enable

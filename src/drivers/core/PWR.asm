
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


@ PWR register details provided in STM32F401's ref manual page 87
@----------------------------------------------------------------------
@----------------------------------------------------------------------

.section .text.sysinit, "ax", %progbits


@-----------------------------------
@ main function called by system initialization to take care of power moes an config
@-----------------------------------
  .global _PWR_config
  .type _PWR_config, %function
_PWR_config:
  PUSH    {lr}
  LDR     r1, [r0]
  @ Disable access to RTC and it's backup domain
  @ Enter Stop mode when CPU enters DEEPSLEED
  @ Low power voltage regulator on during stop mode
  @ Set Voltage regulator in scale 2 mode (adequate for 84Mhz clock)
  @ Set main regulator in low voltage during deep sleep
  @ Set Low power regulator low voltage during deep sleep
  @ FLASH power down during deep-sleep 
  @ Enable power voltage detector and set the threshhold at 2.7 V
  LDR     r2, =PWR_CR_MASK
  ORR     r1, r1, r2
  STR     r1, [r0]
  POP     {lr}
  MOV     r0, #0
  BX      lr
  .align 4
  .size _PWR_config, .-_PWR_config

@----------------------------------------------------------------------
@---------------------------------------------------------------------- application accessible functions

.section .text.drivers.PWR, "ax", %progbits




.section .rodata.registers.PWR, "a", %progbits
  .equ PWR_BASE, 0x40007000
  .equ PWR_CR_MASK, 0x8EFD
  

      
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


@ SYSTICK register details provided in stm32-cortex-M4 page 246 
@----------------------------------------------------------------------
@---------------------------------------------------------------------- startup section
.section .text.sysinit, "ax", %progbits

@-----------------------------------
@ main function called by system initialization to configure the SYSTICK
@-----------------------------------
  .global _SYSTICK_config
  .type _SYSTICK_config, %function
_SYSTICK_config:
  @ save link register 
  PUSH    {lr}
  LDR     r0, =SYSTICK_BASE
  
  @ load the systick counter value (10499) in ST_LOAD to prooduce a tick each 1ms  
  MOVW    r1, =SYSTICK_COUNTER
  STR     r1, [r0, #0x04]           @ STK_LOAD
  MOVW    r1, #0
  STR     r1, [r0, #0x08]           @ STK_VAL
  @ CLKSOURCE = AHB/8, enable SYSTICK interrupt, enable SYSTICK
  MOVW    r1, #0b011
  STR     r1, [r0, #0x08]           @ STK_VAL

  @ recover link register and return 0
  POP     {lr}
  MOV     r0, #0
  BX      lr
  .align 4
  .size _SYSTICK_config, .-_SYSTICK_config


.section .rodata.registers.SYSTICK, "a", %progbits
  .equ SYSTCK_BASE, 0xE000E010
  .equ SYSTICK_COUNTER, 10499   @ value to be laded in STK_LOAD



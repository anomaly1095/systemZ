/**
  * # SystemZ Kernel <PRODUCTION BRANCH>
  * 
  * Copyright (C) 2024 Connexion Nord, Inc. or its affiliates. All Rights Reserved.
  * 
  * SPDX-License-Identifier: MIT
  * 
  * Permission is hereby granted, free of charge, to any person obtaining a copy of
  * this software and associated documentation files (the "Software"), to deal in
  * the Software without restriction, including without limitation the rights to
  * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  * the Software, and to permit persons to whom the Software is furnished to do so,
  * subject to the following conditions:
  * 
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  * 
  * <https://github.com/anomaly1095/systemZ>
  * Author: Youssef Azaiez
*/

              /*      |----------- |*/
              /* 32Kb |kernel SRAM |*/
              /*      |----------- |*/
              /* 64kB |  App SRAM  |*/
              /*      |----------- |*/
              /*      |            |*/
              /* 256Kb| App flash  |*/
              /*      |            |*/
              /*      |----------- |*/
              /*      |            |*/
              /* 256Kb|kernel flash|*/
              /*      |            |*/
              /*      |----------- |*/
MEMORY
{
  /* 512KB of FLASH memory */
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 512K   /* full FLASH */
  K_FLASH (rx)   : ORIGIN = 0x08000000, LENGTH = 256K   /* sectors 0..5*/
  APP_FLASH (rx) : ORIGIN = 0x08040000, LENGTH = 256K   /* sectors 6,7*/
  
  /* 96KB of volatile memory */
  SRAM (rw)     : ORIGIN = 0x20000000, LENGTH = 96K   /* FULL SRAM */
  APP_SRAM (rw) : ORIGIN = 0x20000000, LENGTH = 64K   /* Application space */
  K_SRAM (rw)   : ORIGIN = 0x20010000, LENGTH = 32K   /* Kernel space */
}

ENTRY(_reset_handler)  /* initial pc address */

/* Minimum stack and heap sizes */
k_min_stack = 0x400
k_min_heap = 0x400
min_stack = 0x400
min_heap = 0x400

SECTIONS
{
  /*-----------------------------------------------*/
  /*-------------KERNEL sections-------------------*/
  /*-----------------------------------------------*/

  /* Interrupt vector table */
  .isr_vector : 
  {
    ALIGN(4);
    KEEP(*(.isr_vector))
    ALIGN(4);
  } > K_FLASH

  /* Kernel code section */
  .ktext :
  {
    ALIGN(4);
    *(.text.reset_handler)
    *(.text.sysinit)
    *(.text.syscalls)
    ALIGN(4);
  } > K_FLASH

  /* Kernel read-only data section */
  .krodata : 
  {
    ALIGN(4);
    *(.rodata.k_sections)
    *(.rodata.k_registers.*)
    ALIGN(4);
  } > K_FLASH

  _sikdata = LOADADDR(.kdata);

  /* Kernel data section */
  .kdata : 
  {
    ALIGN(4);
    _skdata = .;
    /* add data sections */
    _ekdata = .;
    ALIGN(4);
  } > K_SRAM AT> K_FLASH

  /* Kernel BSS section */
  .kbss (NOLOAD): 
  {
    ALIGN(4);
    _skbss = .;
    *(COMMON)
    _ekbss = .;
    ALIGN(4);
  } > K_SRAM

  /* Kernel heap section */
  .kheap (NOLOAD): 
  {
    ALIGN(4);
    PROVIDE (_skheap = .);
    . = . + min_kheap;
    PROVIDE (_ekheap = .);
    ALIGN(4);
  } > K_SRAM

  /* Kernel stack section */
  .kstack (NOLOAD): 
  {
    ALIGN(8);
    PROVIDE (_skstack = .);
    . = . + min_kstack;
    PROVIDE (_ekstack = .);
    ALIGN(8);
  } > K_SRAM

  /*-----------------------------------------------*/
  /*-------------APP sections----------------------*/
  /*-----------------------------------------------*/

  /* Application code section */
  .text : 
  {
    ALIGN(4);
    *(.text)

    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    ALIGN(4);
  } > APP_FLASH

  /* Application read-only data section */
  .rodata : 
  {
    ALIGN(4);
    *(.rodata.sections)
    ALIGN(4);
  } > APP_FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } > APP_FLASH

  _sidata = LOADADDR(.data);

  /* Application data section */
  .data : 
  {
    ALIGN(4);
    _sdata = .;
    /* add data sections */
    _edata = .;
    ALIGN(4);
  } > APP_SRAM AT> APP_FLASH

  /* Application BSS section */
  .bss (NOLOAD): 
  {
    ALIGN(4);
    _sbss = .;
    /* add BSS sections */
    _ebss = .;
    ALIGN(4);
  } > APP_SRAM

  /* Application heap section */
  .heap (NOLOAD): 
  {
    ALIGN(4);
    PROVIDE (_sheap = .);    
    . = . + min_heap;
    PROVIDE (_eheap = .);    
    ALIGN(4);
  } > APP_SRAM

  /* Application stack section */
  .stack (NOLOAD): 
  {
    ALIGN(8);
    PROVIDE (_sstack = .);    
    . = . + min_stack;
    PROVIDE (_estack = .);    
    ALIGN(8);
  } > APP_SRAM

  /* Assertions to ensure memory allocations are within limits */
  ASSERT(_edata  <= ORIGIN(APP_FLASH)+ LENGTH(APP_FLASH),"Not enough application data space")
  ASSERT(_ebss   <= ORIGIN(APP_SRAM) + LENGTH(APP_SRAM), "Not enough application BSS space")
  ASSERT(_eheap  <= ORIGIN(APP_SRAM) + LENGTH(APP_SRAM), "Not enough application heap space")
  ASSERT(_estack <= ORIGIN(APP_SRAM) + LENGTH(APP_SRAM), "Not enough application stack space")

  ASSERT(_k_edata    <= ORIGIN(K_FLASH)  + LENGTH(K_FLASH),  "Not enough kernel data space")
  ASSERT(_k_ebss     <= ORIGIN(K_SRAM)   + LENGTH(K_SRAM),   "Not enough kernel BSS space")
  ASSERT(_k_eheap    <= ORIGIN(K_SRAM)   + LENGTH(K_SRAM),   "Not enough kernel heap space")
  ASSERT(_k_estack   <= ORIGIN(K_SRAM)   + LENGTH(K_SRAM),   "Not enough kernel stack space")

  /* Ensure total allocation within APP_SRAM and K_SRAM limits */
  ASSERT(_edata  <= ORIGIN(APP_SRAM) + LENGTH(APP_SRAM), "Total application SRAM allocation exceeded")
  ASSERT(_k_edata    <= ORIGIN(K_SRAM)   + LENGTH(K_SRAM),   "Total kernel SRAM allocation exceeded")

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

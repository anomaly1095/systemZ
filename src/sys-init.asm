
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

@----------------------------------------------
@ Default Handler Section
.section .text.default_handler, "ax", %progbits
	.type _default_handler, %function
  .weak _default_handler
	.global _default_handler

_default_handler:
  PUSH    {r0-r15}            @ Save all registers
  B       .                   @ Infinite loop for debugging
  .align 4
.size _default_handler, .-_default_handler

@----------------------------------------------
@ Reset Handler Section
.section .text.reset_handler, "ax", %progbits
	.type _reset_handler, %function
	.global _reset_handler
	.extern _start

_reset_handler:
  LDR     sp, _ekstack       @ Load kernel stack pointer

  @ Copy .kdata section from FLASH to SRAM
  LDR     r0, =_sikdata
  LDR     r1, =_skdata
  LDR     r2, =_ekdata
__kdata_copy:
  LDR     r3, [r0], #4        @ Load word from .kdata Flash section
  CMP     r1, r2
  IT      NE
  STRNE   r3, [r1], #4        @ Store word in .kdata SRAM section
  BNE     _kdata_copy

  @ Zero out .kbss section in SRAM
  LDR     r0, =_skbss
  LDR     r1, =_ekbss
  MOV     r2, #0
__kbss_zero:
  CMP     r0, r1
  STRNE   r2, [r0], #4

  @ System initialization
  BL      _sysinit

  @ Copy .data section from FLASH to SRAM
  LDR     r0, =_sidata
  LDR     r1, =_sdata
  LDR     r2, =_edata
__data_copy:
  LDR     r3, [r0], #4        @ Load word from .data Flash section
  CMP     r1, r2
  IT      NE
  STRNE   r3, [r1], #4        @ Store word in .data SRAM section
  BNE     _data_copy
  
	@ Zero out .bss section in SRAM
  LDR     r0, =_sbss
  LDR     r1, =_ebss
  MOV     r2, #0
__bss_zero:
  CMP     r0, r1
  STRNE   r2, [r0], #4

  @ Configure CONTROL register and switch to unprivileged thread mode
  MOV     r1, #0b011
  MRS     r0, CONTROL
  ORR     r0, r0, r1
  MSR     CONTROL, r0
  LDR     sp, =_estack        @ Load app stack pointer in PSP
  BL      _start              @ Branch to application code
  B       _default_handler    @ Debug: if return from application
  .align 4
.size _reset_handler, .-_reset_handler

@----------------------------------------------
@ System Initialization
.section .text.sysinit, "ax", %progbits
	.global _sysinit
	.type _sysinit, %function

_sysinit:
	PUSH    {lr}              @ Preserve the link register
  
  @ CLOCK configuration
	BL      RCC_config
  CBNZ    r0, _default_handler
  
  @ POWER configuration
	BL      PWR_config
  CBNZ    r0, _default_handler  @ Branch to default handler on error

	@ MPU configuration
	BL      MPU_config
  CBNZ    r0, _default_handler

	@ FLASH configuration
	BL      FLASH_config
  CBNZ    r0, _default_handler

	@ NVIC configuration
  BL      NVIC_config
  CBNZ    r0, _default_handler

  @ SYSTICK configuration
  BL      SYSTICK_config
  CBNZ    r0, _default_handler

  @ Peripherals configuration 
  BL      PERIPH_config
  CBNZ    r0, _default_handler

  POP     {lr}               @ Restore the link register
	BX      lr                 @ Return from sysinit
  .align 4
.size _sysinit, .-_sysinit   @ Define the size of the function

@----------------------------------------------
@ kernel sections limits
.section .rodata.k_sections, "a", %progbits
	.word _sikdata
	.word _skdata
	.word _ekdata
	.word _skbss
	.word _ekbss

@ application sections limits
.section .rodata.sections, "a", %progbits
	.word _estack
	.word _sidata
	.word _sdata
	.word _edata
	.word _sbss
	.word _ebss

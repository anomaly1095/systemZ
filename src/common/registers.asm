.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.section .rodata.k_registers.RCC, "a", %progbits
  .equ RCC_BASE, 0x40023800
  .equ RCC_BASE_BB, 0x42470000

  
.section .rodata.k_registers.PWR, "a", %progbits

.section .rodata.k_registers.MPU, "a", %progbits
  
.section .rodata.k_registers.FLASH, "a", %progbits

.section .rodata.k_registers.NVIC, "a", %progbits
  
.section .rodata.k_registers.SYSTICK, "a", %progbits

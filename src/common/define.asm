.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb


.section .rodata.k_registers.RCC, "a", %progbits
  .equ RCC_BASE, 0x40023800       @ RCC bit-band base address
  .equ RCC_BASE_BB, 0x42470000    @ RCC base address

  
.section .rodata.k_registers.PWR, "a", %progbits
  .equ PWR_BASE, 0x40007000
  .equ PWR_CR_MASK, 0x8EFD
  

.section .rodata.k_registers.MPU, "a", %progbits
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


.section .rodata.k_registers.FLASH, "a", %progbits
  .equ FLASH_BASE, 0x40023C00       @ FLASH base address
  .equ FLASH_KEY1, 0x45670123
  .equ FLASH_KEY2, 0xCDEF89AB
  .equ FLASH_OPTKEY1, 0x08192A3B
  .equ FLASH_OPTKEY2, 0x4C5D6E7F

.section .rodata.k_registers.NVIC, "a", %progbits
  .equ NVIC_ISER0, 0xE000E100     @ 7 register
  .equ NVIC_ICER0, 0xE000E180     @ 7 register
  .equ NVIC_ISPR0, 0xE000E200     @ 7 register
  .equ NVIC_ICPR0, 0xE000E280     @ 7 register
  .equ NVIC_IABR0, 0xE000E300     @ 7 register
  .equ NVIC_IPR0, 0xE000E400      @ 59 register
  .equ STIR, 0xE000EF00           @ 1 register



.section .rodata.k_registers.SYSTICK, "a", %progbits
  .equ SYSTCK_BASE, 0xE000E010
  .equ SYSTICK_COUNTER, 10499   @ value to be laded in STK_LOAD
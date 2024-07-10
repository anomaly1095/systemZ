# systemZ

## Overview

"systemZ" is an experimental project focused on developing a miniature operating system. This project does not attempt to be flexible or compatible across multiple architectures. Instead, it serves as an educational endeavor to explore operating system concepts and low-level programming.

## Features

- **Minimalist Approach**: Prioritizes simplicity and educational value over flexibility.
- **Learning Focus**: Intended for educational purposes to delve into operating system development.
- **Single Architecture**: Developed specifically for the ARM Cortex-M4 architecture.
- **Drivers**: Specifically developed for the STM32F401 microcontroller unit.
- **Power Management**: Configurations are flexible but geared towards the STM32F401RE.

## Project Structure

The project directory structure is as follows:

ROOT/
├── bin/
├── include/
├── LICENSE
├── linker.ld
├── Makefile
├── README.md
├── docs/
│   ├── Nucleo-64.pdf
│   ├── stm32-cortex-M4.pdf
│   ├── stm32F401-ref-man.pdf
│   ├── reference.md
│   └── stm32F401RE-data-sheet.pdf
└── src/
    ├── IRQ.asm
    ├── isr-vectors.asm
    ├── sys-init.asm
    ├── app/
    │   └── main.asm
    └── drivers/
        ├── core/
        │   ├── FLASH.asm
        │   ├── FPU.asm
        │   ├── MPU.asm
        │   ├── NVIC.asm
        │   ├── PWR.asm
        │   └── RCC.asm
        └── extern/
            ├── GPIO.asm
            ├── USART.asm
            ├── SPI.asm
            ├── I2C.asm
            ├── ADC.asm
            ├── DMA.asm
            ├── EXTI.asm
            ├── RTC.asm
            ├── timers.asm
            └── USB.asm

## Getting Started

[Include instructions on how to get started with the project, such as installation steps, prerequisites, and how to compile/build/run the operating system.]

## Contributing

[Guidelines for contributing to the project if applicable.]

## License

[Include information about the project's license. The license file is available in the /LICENSE directory.]

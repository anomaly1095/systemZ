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
    ├── include.asm
    ├── app/
    │   └── main.asm
    └── sys-calls.asm

## Getting Started

[Project is still in developement phase no build is available.]

## Contributing

[All help and participations or advice are welcome.]

## License

[The license file is available in the /LICENSE directory.]

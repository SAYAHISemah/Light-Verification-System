# Light Verification System

This project is for the **Light Verification System** running on the STM32F072 microcontroller. Ensure light functionality using STM32 technology and CAN communication for enhanced safety
## Table of Contents

- [Features](#features)
- [Setup](#setup)
- [Usage](#usage)
- [Project Structure](#project-structure)

## Features

- Ensures front and back scooter lights work correctly.
- Activates instant tests for each light.
- Reports issues to the dashboard.

## Setup

### Prerequisites

- **STM32CubeIDE**: Make sure you have the latest version installed.
- **STM32CubeMX**: Used to configure the microcontroller and peripherals.
- **ARM GCC Toolchain**: Required for compiling the project.

### Installation

1. *Clone the repository:*

    ```bash
    git clone <repository_url>
    ```

2. *Open the project in STM32CubeIDE:*

    After cloning, you can open the project using STM32CubeIDE by selecting the `.project` file.

3. *Build the project:*

    Ensure the STM32F0 series HAL driver and CMSIS are installed. Then, build the project from the IDE.

## Usage

### Running on STM32F072

1. **Configure the Board:**
    Use STM32CubeMX to configure the peripherals, such as GPIOs, clocks, and timers.

2. **Build and Flash:**
    You can directly build and flash the code using STM32CubeIDE. Select the "Debug" or "Release" configuration depending on your needs.

3. **Debugging:**
    Use the integrated debugger in STM32CubeIDE. You can use the provided `.launch` files for quick debugging setup.

## Project Structure

The following is a high-level overview of the project folder structure:
```bash
LIGHT-VERIFICATION-SYSTEM/
├── .settings/
│   ├── com.st.stm32cube.ide.mcu. ...
│   ├── language.settings.xml
│   └── stm32cubeide.project.prefs
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f0xx_hal_conf.h
│   │   └── stm32f0xx_it.h
│   └── Src/
│       ├── main.c
│       ├── stm32f0xx_hal_msp.c
│       ├── stm32f0xx_it.c
│       ├── syscalls.c
│       ├── sysmem.c
│       └── system_stm32f0xx.c
├── Debug/
│   └── Core/
│       └── Src/
│           ├── main.d
│           ├── main.o
│           ├── main.su
│           ├── stm32f0xx_hal_msp.d
│           ├── stm32f0xx_hal_msp.o
│           ├── stm32f0xx_hal_msp.su
│           ├── stm32f0xx_it.d
│           ├── stm32f0xx_it.o
│           ├── stm32f0xx_it.su
│           ├── syscalls.d
│           ├── syscalls.o
│           ├── syscalls.su
│           ├── sysmem.d
│           ├── sysmem.o
│           ├── sysmem.su
│           ├── system_stm32f0xx.d
│           ├── system_stm32f0xx.o
│           ├── system_stm32f0xx.su
│           └── subdir.mk
├── Startup/
│   ├── startup_stm32f072rbtbx.s
│   ├── startup_stm32f072rbtbx.d
│   ├── startup_stm32f072rbtbx.o
│   └── subdir.mk
├── Drivers/
│   ├── CMSIS/
│   └── STM32F0xx_HAL_Driver/
├── makefile
├── objects.list
├── objects.mk
├── sources.mk
├── test2.elf
├── test2.list
├── test2.map
├── README.md
├── STM32F072RBTX_FLASH.ld
├── test2 Debug.launch
├── test2.ioc
├── .project
├── .mxproject
```

### Key Directories:

- **Core**: This contains the main application logic, including the system configuration, interrupts, and memory management.
  
- **Startup**: This contains the assembly code to initialize the STM32F072 microcontroller.

- **Drivers**: Contains CMSIS and STM32 HAL drivers, which abstract low-level microcontroller functionalities.

- **Debug**: Compiled object files and debug artifacts.



# Light Verification System

This project is for the **Light Verification System** running on the STM32F072 microcontroller. The project uses STM32CubeIDE and the STM32 HAL library to manage and control the microcontroller peripherals.

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
LIGHT-VERIFICATION-SYSTEM/ ├── .settings/ # Project configuration files for STM32CubeIDE │ ├── com.st.stm32cube.ide.mcu. ... # STM32CubeIDE settings │ ├── language.settings.xml # Language-specific settings │ └── stm32cubeide.project.prefs # STM32CubeIDE project preferences ├── Core/ │ ├── Inc/ # Header files │ │ ├── main.h # Main header file │ │ ├── stm32f0xx_hal_conf.h # HAL configuration │ │ └── stm32f0xx_it.h # Interrupt handlers │ └── Src/ # Source files │ ├── main.c # Main application logic │ ├── stm32f0xx_hal_msp.c # MSP initialization │ ├── stm32f0xx_it.c # Interrupt handlers │ ├── syscalls.c # System calls │ ├── sysmem.c # Memory configuration │ └── system_stm32f0xx.c # System configuration ├── Debug/ # Compiled debug files │ └── Core/ │ └── Src/ │ ├── *.d, *.o, *.su, *.cyclo # Object and dependency files for debugging ├── Startup/ │ ├── startup_stm32f072rbtbx.s # Assembly code for startup configuration ├── Drivers/ │ ├── CMSIS/ # ARM CMSIS drivers │ └── STM32F0xx_HAL_Driver/ # STM32 HAL drivers ├── test2.elf # Executable output ├── README.md # Project overview and instructions ├── STM32F072RBTX_FLASH.ld # Linker script for STM32F072 ├── test2 Debug.launch # Debug launch configuration ├── test2.ioc # STM32CubeMX project file
```

### Key Directories:

- **Core**: This contains the main application logic, including the system configuration, interrupts, and memory management.
  
- **Startup**: This contains the assembly code to initialize the STM32F072 microcontroller.

- **Drivers**: Contains CMSIS and STM32 HAL drivers, which abstract low-level microcontroller functionalities.

- **Debug**: Compiled object files and debug artifacts.



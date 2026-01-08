# STM32F446xx Bare-Metal Firmware Drivers

## Project Overview
This project contains low-level firmware drivers for the **STM32F446RE (ARM Cortex-M4)**, developed entirely from scratch without using high-level libraries (HAL/LL). The focus is on **Register Transfer Level (RTL)** configuration to understand the hardware-software interface.

## Key Technical Assets
### 1. Register-Level Header File (`stm32f446xx.h`)
* [cite_start]Defines peripheral base addresses for AHB and APB buses (RCC, GPIO, SPI, I2C, UART).
* Implements **Register Structure Definitions** using C structures and `volatile` pointers to map directly to hardware memory.
* [cite_start]Includes **Clock Enable/Disable macros** to manage power for individual SoC functional units[cite: 5, 8].

### 2. GPIO Driver Implementation
* Supports Mode configuration (Input, Output, Alternate Function, Analog).
* Implements **Interrupt Handling (EXTI)** logic, including Rising/Falling edge detection.
* Features API for pin-level operations: `GPIO_Read`, `GPIO_Write`, and `GPIO_Toggle`.

### 3. SPI Driver Implementation
* Supports **Master/Slave** modes and Full-Duplex/Half-Duplex/Simplex bus configurations.
* Manages **Baud Rate** and **Clock Phase/Polarity (CPOL/CPHA)** settings.
* Includes both **Polling** and **Interrupt-based (IT)** data transmission APIs.

## Engineering Methodology
* [cite_start]**Research & Definition:** Referenced the **STM32F446xx Reference Manual** to determine bit-offsets and register reset values.
* **Design:** Used a modular "Handle-based" architecture to separate peripheral configuration from driver logic.
* **Verification (Work in Progress):** Functional testing performed on the **Nucleo-F446RE** board. Logic and timing validation is verified via **Saleae Logic Analyzer** to ensure protocol compliance with physical hardware.

## Future Roadmap
* **Cellular & Wi-Fi Firmware:** Expanding research into L1/L2 stack architectures for wireless SoC integration.
* **Interference Suppression:** Integrating AI/ML models to mitigate interference burdens in 6G communication hardware.

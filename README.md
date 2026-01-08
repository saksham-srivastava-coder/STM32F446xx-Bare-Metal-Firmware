# STM32F446xx Bare-Metal Firmware

## Project Overview
This repository contains low-level firmware drivers for the **STM32F446RE (ARM Cortex-M4)**. These were developed from scratch using the **Reference Manual** and **Datasheet** to bypass abstraction layers and gain direct control over SoC functional units.

## Technical Architecture
### 1. Register-Level Mapping (`stm32f446xx.h`)
* Implemented memory-mapped structures for **RCC, GPIO, SPI, and EXTI**.
* Defined peripheral base addresses for AHB1, APB1, and APB2 bus systems.
* Used `volatile` pointers to ensure compiler-safe access to hardware registers.

### 2. GPIO Sub-system Driver
* **Clock Management**: Implemented macros for enabling/disabling peripheral clocks via the RCC register.
* **Pin Logic**: Supports Mode, Speed, Output Type, and Pull-up/Pull-down configuration.
* **Interrupts**: Integrated NVIC and EXTI logic for real-time hardware events.

### 3. SPI Sub-system Driver
* **Configurable Bus**: Supports Full-Duplex, Half-Duplex, and Simplex RX-only modes.
* **Protocol Control**: Manages Baud Rate, Software Slave Management (SSM), and Data Frame Format (8/16-bit).
* **Transmission**: Includes both Polling (Blocking) and Interrupt-driven (Non-blocking) APIs.

## Engineering Methodology
* **Documentation Study**: Analyzed the STM32F446xx Reference Manual for bit-definitions of the Control Registers (CR1/CR2).
* **Hardware Validation**: Code verified on the **Nucleo-F446RE** board.
* **Verification Logic**: Timing and protocol compliance verified using a **Saleae Logic Analyzer** (screenshots pending).

## Research Interests & Roadmap
* **AI/ML for 6G**: Ongoing work on interference suppression using ML/DL algorithms.
* **Wireless Firmware**: Actively learning L1/L2 firmware stacks for Cellular and Wi-Fi SoC integration.

# Firmware for the Karca Engine Control Box

## Project status
This project is under active development. Currently, the firmware is being tested on the [NUCLEO-F207](https://www.st.com/en/evaluation-tools/nucleo-f207zg.html) development board. We will eventually migrate to the [STM32F407VET6](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F407VET6/2793093)
## Overview
This repository contains the STM32Cube IDE project with the firmware running on the STM32F407 controller for Proxima's Engine Control Box.
The code aims to fulfil the same objective as [slonk](https://github.com/rice-eclipse/slonk) and [RESFET](https://github.com/rice-eclipse/resfet). These are as follows:
- Collect Data from the sensors and relay it to the operator over TCP at a minimum rate of 10Hz
- Receive Commands from the operator over TCP and actuate the corresponding valves/ignition relay
- Support a faster sampling rate (up to 5kHz) during ignition

## Motivation
The engine control firmware was written (yet again!) as part of the team's move away from the Raspberry Pi-based system and towards a microcontroller. 
The reasons for the switch are as follows:
- Deterministic sampling and actuation. The STM32 uses a real-time operating system that reduces the timing uncertainty during data collection and valve actuation
- Wider range of peripherals. The STM32 has three SPI peripherals and three 12-bit internal ADCs, which provide more headroom for expansion in case more sensors are needed
- Low power consumption

However, if future members decide to move back to the Raspberry Pi, they are strongly recommended to reuse and adapt the `slonk` engine control software.

## Installation
Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) and [STM32CubeMX](https://www.st.com/content/st_com/en/stm32cubemx.html)

Clone the repository
```git clone https://github.com/rice-eclipse/mc-firmware```

Open STM32CubeIDE and set the root directory of the repository as the workspace

Build `f407-firmware` to build the code for the controller running on the engine control box 

Run the code using the GUI 

## Hardware Stack
* **MCU:** STM32F407 (ARM Cortex-M4 CPU)
* **Sensors:** Thermocouples, Load Cells, Pressure Transducers
* **Sensor Interface** 12-bit, 8 channel MCP3208 ADC (SPI interface)
* **Wireless Communication** Websocket connection over LwIP (Lightweight IP TCP implementation) 

## Software Design
The system is divided into isolated tasks managed by the FreeRTOS scheduler:

1. **Data Reading**
   * Processes raw data from the ADCs and stores it in a shared buffer.
2. **Command Handling**
   * Processes incoming commands from the Ground Station.
3. **Data Sending**
   * Writes flight data to SD Card for onboard storage.
   * Transmits real-time data via WebSocket to the Ground Station Dashboard (Quonkboard) at a decimated interval.

## Development Environment
* **Language:** C
* **OS:** FreeRTOS

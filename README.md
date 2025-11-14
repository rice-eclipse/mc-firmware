# Rice Eclipse - Mission Control Firmware

## Overview
The repository contains the flight software and firmware for the Rice Eclipse rocket. The firmware is designed to handle real-time sensor acquisition, onboard data logging, and ground telemetry transmission.

## System Architecture
The firmware is built on **FreeRTOS** running on an **STM32** microcontroller. 

## Hardware Stack
* **MCU:** STM32 Family
* **Sensors:** External ADCs

## Software Design
The system is divided into isolated tasks managed by the FreeRTOS scheduler:

1. **Data Reading**
   * Processes raw data from the ADCs and stores it in a shared buffer.
3. **Command Handling**
   * Processes incoming commands from the Ground Station.
5. **Data Sending**
   * Writes flight data to SD Card for onboard storage.
   * Transmits real-time data via WebSocket to the Ground Station Dashboard (Quonkboard) at a decimated interval.

## Development Environment
* **Language:** C
* **OS:** FreeRTOS

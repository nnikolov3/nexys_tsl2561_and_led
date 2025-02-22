# ECE 544 Project #2: PID-Controlled LED Brightness

## Overview

This project implements a closed-loop PID controller using FreeRTOS on a MicroBlaze-based FPGA system (Nexys A7 or Boolean Board). It adjusts an LED's brightness to maintain a user-set light level (lux) measured by a TSL2561 sensor. Users control the setpoint and PID constants (Kp, Ki, Kd) via switches and buttons, with feedback displayed on a 7-segment display.

## Learning Goals

- Practice PID control and driver development.
- Gain experience with FreeRTOS.
- Prepare for the final project.

## Requirements

- **Hardware**: Nexys A7 or Boolean Board, TSL2561 sensor, white LED, resistor, breadboard, jumper wires, enclosure, disturbance materials (e.g., parchment paper, flashlight).
- **Software**: Vivado, Vitis with FreeRTOS.

## Setup

1. **Hardware**:
   - Connect LED to PMOD JC (logic + GND).
   - Connect TSL2561 to PMOD JB (logic, VCC, GND).
   - Place in an enclosure to block ambient light.
2. **Software**:
   - Install Vivado and Vitis.
   - Import `.xsa` and `.bit` files from the project release.
   - Configure FreeRTOS in Vitis (adjust heap/stack if needed).
3. **Build**:
   - Add `tsl2561.c` and `tsl2561.h` to the `src` directory for the sensor driver.
   - Compile and load the firmware onto the FPGA.

## Usage

- **Switches**:
  - [7:6]: Select Kp, Ki, or Kd to adjust.
  - [5:4]: Set increment size (±1, ±5, ±10).
  - [3]: Toggle setpoint adjustment.
  - [2:0]: Enable/disable P, I, D control.
- **Buttons**:
  - BtnU: Increase selected value.
  - BtnD: Decrease selected value.
- **Display**: Setpoint on digits [7:5], current lux on [3:1].
- **Testing**: Use parchment paper or a flashlight to disturb the light level and observe PID response.

## Deliverables

- Live/video demo of functionality.
- 3-5 page report with design details, graphs, and team contributions.
- Clean, commented source code (C and HDL) and schematic.

## Team

- Two-person team project.
- Self-enroll in groups via the course system.

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

### What is PID?

<https://www.ece.ucdavis.edu/~spencer/195/datasheets/PID-Without-a-PhD.pdf>

PID stands for **Proportional-Integral-Derivative**. It’s a control mechanism used in systems to make an output (e.g., light level) match a desired target (setpoint) by adjusting an input (e.g., LED PWM duty cycle). It does this by continuously monitoring the difference between the actual value and the target—called the **error**—and tweaking the system to minimize that error.

Think of it like driving a car:

- You want to stay at 60 mph (setpoint).
- If you’re going 50 mph, the error is 10 mph.
- PID figures out how much to press the gas pedal to get you to 60 mph smoothly.

### The Three Parts of PID

PID combines three strategies to control the system:

1. **Proportional (P)**
   - Responds to the **current error**.
   - The bigger the error, the stronger the adjustment.
   - Example: If the lux is 20 below the setpoint, P increases the LED brightness a lot; if it’s 5 below, it increases it a little.
   - Formula: `P = Kp * error` (Kp is a constant you tune).
   - Problem: Alone, it might overshoot or never quite reach the target (steady-state error).

2. **Integral (I)**
   - Fixes **past errors** by adding them up over time.
   - If the system is consistently below the setpoint, I keeps nudging it up until the error is zero.
   - Example: If the lux stays slightly low for a while, I increases the LED brightness gradually to eliminate that gap.
   - Formula: `I = Ki * (sum of errors over time)` (Ki is another tuning constant).
   - Problem: Too much I can cause overshoot or oscillations.

3. **Derivative (D)**
   - Predicts **future errors** by looking at how fast the error is changing.
   - It dampens the response to prevent overshooting or rapid swings.
   - Example: If the lux is rising quickly toward the setpoint, D slows down the LED adjustment to avoid going too far.
   - Formula: `D = Kd * (change in error / change in time)` (Kd is the third tuning constant).
   - Problem: Too much D can make the system jittery if there’s noise.

### How PID Works Together

The PID controller calculates these three terms and adds them up to decide the adjustment (e.g., PWM duty cycle for the LED):

- **Output = P + I + D**

In your project:

- **Error** = Setpoint lux (what you want) - Current lux (from TSL2561 sensor).
- **Output** = PWM duty cycle to control LED brightness.
- You tune **Kp**, **Ki**, and **Kd** using switches/buttons to make the system stable, fast, and accurate.

### Real-World Example in Your Project

- **Setpoint**: 100 lux.
- **Current lux**: 80 lux → Error = 20.
  - **P**: Kp * 20 → Boosts LED brightness based on the gap.
  - **I**: Ki * (sum of past errors) → Keeps increasing brightness if 80 lux persists.
  - **D**: Kd * (rate of lux change) → Slows adjustment if lux is rising fast.
- Goal: Lux stabilizes at 100 without overshooting or oscillating.

### Tuning PID

- **Kp**: Start here. Increase until the system responds quickly but doesn’t overshoot too much.
- **Ki**: Add this to remove lingering error (e.g., if lux settles at 95 instead of 100). Keep it small to avoid wobbling.
- **Kd**: Add this to smooth out overshoot or oscillations. Too much can slow things down.

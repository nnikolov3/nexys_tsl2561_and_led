#ifndef MAIN_H
#define MAIN_H

/*
 * main.h - Header file for TSL2561-based system with PID control on Nexys A7
 *
 * Purpose: Defines constants, global variables, and function prototypes for
 *          initializing and managing hardware components (GPIO, I2C, TSL2561,
 *          Nexys peripherals) and FreeRTOS constructs (tasks, semaphores,
 * queues) in a Microblaze-based system. Supports button-driven LED toggling,
 *          PID control of RGB LED brightness, and sensor reading/display
 * updates.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* Standard C library includes - General-purpose functions */
#include <stdlib.h> // Standard library functions (e.g., configASSERT)

/* Xilinx BSP and platform includes - Hardware interaction */
#include "platform.h" // Platform-specific configurations (e.g., Microblaze setup)
#include "sleep.h"    // Provides usleep() for timing delays
#include "xgpio.h"    // Xilinx GPIO driver API for button and switch handling
#include "xiic.h"     // Xilinx I2C driver API for AXI IIC controller
#include "xil_printf.h" // Xilinx printf implementation for debug output
#include "xintc.h" // Xilinx interrupt controller API for interrupt management
#include "xparameters.h" // Hardware-specific parameters from the BSP (e.g., device IDs)
#include "xtmrctr.h" // Xilinx timer driver API (used for FreeRTOS systick)

/* FreeRTOS kernel includes - Real-time functionality */
#include "FreeRTOS.h" // Main FreeRTOS kernel definitions and types (e.g., TickType_t)
#include "queue.h" // Queue management APIs for inter-task communication
#include "semphr.h" // Semaphore APIs for synchronization (e.g., binary semaphore)
#include "task.h"   // Task management APIs (e.g., xTaskCreate, vTaskDelay)
#include "timers.h" // Software timer APIs (included for completeness, unused here)

/* Project-specific includes - Custom headers for peripheral and sensor drivers
 */
#include "i2c.h" // I2C driver interface for AXI IIC controller
#include "nexys4IO.h" // Nexys A7 peripheral driver (LEDs, 7-segment display, RGB LEDs)
#include "tsl2561.h" // TSL2561 sensor driver interface

/* Peripheral definitions - Hardware-specific IDs and addresses */
#define N4IO_DEVICE_ID \
        XPAR_NEXYS4IO_0_DEVICE_ID // Device ID for Nexys4IO peripheral
#define N4IO_BASEADDR \
        XPAR_NEXYS4IO_0_S00_AXI_BASEADDR // Base address for Nexys4IO AXI
                                         // interface
#define N4IO_HIGHADDR \
        XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR // High address for Nexys4IO AXI
                                         // interface
#define INTC_DEVICE_ID \
        XPAR_INTC_0_DEVICE_ID // Device ID for AXI interrupt controller
#define I2C_BASE_ADDR \
        XPAR_AXI_IIC_0_BASEADDR // Base address for AXI IIC controller
#define I2C_DEV_ID_ADDR \
        XPAR_AXI_IIC_0_DEVICE_ID // Device ID for AXI IIC controller
#define GPIO_INTR_ID \
        XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR // Interrupt ID
                                                                // for GPIO
                                                                // button
                                                                // channel

/* GPIO channels - Channel definitions for GPIO instance */
#define BTN_CHANNEL       1 // GPIO channel 1 for button inputs
#define SW_CHANNEL        2 // GPIO channel 2 for switch inputs

/* RTOS definitions - Constants for FreeRTOS constructs */
#define MAIN_QUEUE_LENGTH ( 1 ) // Length of the queues for task communication
#define MAIN_DONT_BLOCK \
        ( (TickType_t) 0 ) // Non-blocking operation for queue send/receive

/* Duty Cycle and Lux Related Constants */
#define MAX_DUTY 255 // Max possible duty cycle for RGB1 Blue
#define MIN_DUTY 0   // Min possible duty cycle for RGB1 Blue
#define LUX_MASK \
        0xFFFF // Mask for combining setpoint and lux values into one uint32_t

/* PID structure definition - Used for PID control in PID_Task */
typedef struct
{
        float Kp;         // Proportional gain
        float Ki;         // Integral gain
        float Kd;         // Derivative gain
        float setpoint;   // Desired lux value
        float integral;   // Accumulated error
        float prev_error; // Previous error for derivative term
        float delta_t;    // Time between samples
        float max_lim;    // Maximum output limit
        float min_lim;    // Minimum output limit
} PID_t;

/* Global variables - Extern declarations for objects defined in main.c */
extern XGpio xInputGPIOInstance;     // GPIO instance for buttons and switches,
                                     // defined in main.c
extern SemaphoreHandle_t binary_sem; // Binary semaphore for signaling button
                                     // interrupts, defined in main.c
extern QueueHandle_t toPID;   // Queue for sending button/switch states to PID
                              // task, defined in main.c
extern QueueHandle_t fromPID; // Queue for sending setpoint/lux to display task,
                              // defined in main.c
extern XIntc Intc; // Shared interrupt controller instance, defined in main.c
extern XIic
  IicInstance; // I2C instance for TSL2561 communication, defined in i2c.c

/* Function declarations - Prototypes for functions implemented in main.c */

/**
 * Sets up GPIO hardware for button interrupts.
 * Configures GPIO instance, installs interrupt handler, and enables interrupts
 * using FreeRTOS APIs.
 *
 * @return 0 on success, 1 on failure
 */
static int prvSetupHardware( void );

/**
 * GPIO interrupt handler for button presses.
 * Signals the semaphore to wake the Parse Input task and clears the interrupt.
 *
 * @param pvUnused Unused parameter (required by ISR signature)
 */
static void gpio_intr( void* pvUnused );

/**
 * Task to parse button presses and switch states.
 * Waits for the semaphore, reads GPIO inputs, and sends states to the PID task
 * via queue.
 *
 * @param p Unused parameter
 */
void Parse_Input_Task( void* p );

/**
 * Task for PID control of RGB LED based on TSL2561 readings.
 * Adjusts LED brightness using PID algorithm and updates display task.
 *
 * @param p Pointer to PID_t structure
 */
void PID_Task( void* p );

/**
 * Task to update 7-segment display with setpoint and lux values.
 *
 * @param p Unused parameter
 */
void Display_Task( void* p );

/**
 * Task to read TSL2561 sensor data.
 * Periodically reads CH0 and CH1 values and prints them.
 *
 * @param p Unused parameter
 */
void sensor_task( void* p );

/**
 * Initializes the system: GPIO, I2C, TSL2561, and Nexys peripherals.
 * Sets up hardware components without configuring interrupts (done separately
 * in main).
 *
 * @return XST_SUCCESS on success, XST_FAILURE on failure
 */
static int do_init( void );

/**
 * Initializes Nexys A7 peripherals (e.g., LEDs, 7-segment display).
 * Configures the Nexys4IO driver for peripheral control.
 *
 * @return XST_SUCCESS on success, XST_FAILURE on failure
 */
static int nexys_init( void );

/**
 * Initializes PID structure for use in the PID task.
 * Sets initial values for PID parameters.
 *
 * @param pid Pointer to PID_t structure
 * @return 1 on success (for consistency with XST_SUCCESS-like behavior)
 */
int pid_init( PID_t* pid );

/**
 * PID algorithm implementation.
 * Calculates PID output based on lux value and switch settings.
 *
 * @param pid Pointer to PID_t structure
 * @param lux_value Current lux reading from TSL2561
 * @param switches Current switch states
 * @return Float percentage value for PWM adjustment
 */
float pid_funct( PID_t* pid, float lux_value, uint8_t switches );

#endif /* MAIN_H */

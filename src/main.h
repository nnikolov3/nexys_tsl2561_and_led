#ifndef MAIN_H
#define MAIN_H

/*
 * main.h - Header file for TSL2561-based system on Nexys A7
 *
 * Purpose: Defines constants, global variables, and function prototypes for
 *          initializing and managing hardware components (GPIO, I2C, TSL2561,
 *          Nexys peripherals) and FreeRTOS constructs (tasks, semaphores,
 * queues) in a Microblaze-based system. Supports button-driven LED toggling and
 *          sensor reading.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* Kernel includes - Core FreeRTOS headers for real-time functionality */
#include "FreeRTOS.h" // Main FreeRTOS kernel definitions and types (e.g., TickType_t)
#include "queue.h" // Queue management APIs for inter-task communication
#include "semphr.h" // Semaphore APIs for synchronization (e.g., binary semaphore)
#include "task.h"   // Task management APIs (e.g., xTaskCreate, vTaskDelay)
#include "timers.h" // Software timer APIs (not used here but included for completeness)

/* BSP includes - Xilinx BSP headers for hardware interaction */
#include "sleep.h"      // Provides usleep() for timing delays
#include "xgpio.h"      // Xilinx GPIO driver API for button and switch handling
#include "xiic.h"       // Xilinx I2C driver API for AXI IIC controller
#include "xil_printf.h" // Xilinx printf implementation for debug output
#include "xintc.h" // Xilinx interrupt controller API for interrupt management
#include "xparameters.h" // Hardware-specific parameters from the BSP (e.g., device IDs)
#include "xtmrctr.h" // Xilinx timer driver API (used for FreeRTOS systick)

/* Standard includes - General-purpose C library functions */
#include <stdlib.h> // Standard library functions (e.g., configASSERT)

/* Project-specific includes - Custom headers for peripheral and sensor drivers
 */
#include "i2c.h"      // I2C driver interface for AXI IIC controller
#include "nexys4IO.h" // Nexys A7 peripheral driver (LEDs, 7-segment display)
#include "platform.h" // Platform-specific configurations (e.g., Microblaze setup)
#include "tsl2561.h" // TSL2561 sensor driver interface

/* Peripheral definitions - Hardware-specific IDs and addresses */
#define N4IO_DEVICE_ID \
    XPAR_NEXYS4IO_0_DEVICE_ID // Device ID for Nexys4IO peripheral
#define N4IO_BASEADDR \
    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR // Base address for Nexys4IO AXI interface
#define N4IO_HIGHADDR \
    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR // High address for Nexys4IO AXI interface
#define INTC_DEVICE_ID \
    XPAR_INTC_0_DEVICE_ID // Device ID for AXI interrupt controller
#define GPIO_INTR_ID \
    XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR // Interrupt ID for
                                                            // GPIO button
                                                            // channel

/* GPIO channels - Channel definitions for GPIO instance */
#define BTN_CHANNEL       1 // GPIO channel 1 for button inputs
#define SW_CHANNEL        2 // GPIO channel 2 for switch inputs

/* RTOS definitions - Constants for FreeRTOS constructs */
#define MAIN_QUEUE_LENGTH ( 1 ) // Length of the queue for button-to-LED events
#define MAIN_DONT_BLOCK \
    ( (TickType_t) 0 ) // Non-blocking operation for queue send/receive

/* Global variables - Extern declarations for objects defined in main.c */
extern XGpio xInputGPIOInstance;     // GPIO instance for buttons and switches,
                                     // defined in main.c
extern SemaphoreHandle_t button_sem; // Binary semaphore for signaling button
                                     // interrupts, defined in main.c
extern QueueHandle_t button_queue; // Queue for sending button events to the LED
                                   // task, defined in main.c
extern XIntc Intc; // Shared interrupt controller instance, defined in main.c

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
 * Signals the semaphore to wake the button task and clears the interrupt.
 *
 * @param pvUnused Unused parameter (required by ISR signature)
 */
static void gpio_intr( void* pvUnused );

/**
 * Task to handle button presses and send LED patterns to queue.
 * Waits for the semaphore, toggles an LED pattern, and sends it to the LED task
 * via queue.
 *
 * @param p Unused parameter
 */
void button_task( void* p );

/**
 * Task to initialize and manage I2C communication (polling mode).
 * Currently unused but reserved for potential I2C-specific task management.
 *
 * @param p Unused parameter
 */
void i2c_task( void* p );

/**
 * Task to initialize and read data from the TSL2561 sensor.
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
 * Performs a self-test on the 7-segment display to verify functionality.
 * Displays "ECE544" for 2 seconds to confirm operation.
 */
static void seven_seg_selfTest( void );

#endif /* MAIN_H */

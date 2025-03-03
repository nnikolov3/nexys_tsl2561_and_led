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

/* Xilinx BSP and platform includes. */
#include "sleep.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xtmrctr.h"
#include "xiic.h"     // Xilinx I2C driver API for AXI IIC controller
//#include "xintc.h" // Xilinx interrupt controller API for interrupt management


/* Xilinx BSP and platform includes - Hardware interaction */
#include "platform.h" // Platform-specific configurations (e.g., Microblaze setup)
#include "sleep.h"    // Provides usleep() for timing delays
#include "xgpio.h"    // Xilinx GPIO driver API for button and switch handling
#include "xiic.h"     // Xilinx I2C driver API for AXI IIC controller
#include "xil_printf.h" // Xilinx printf implementation for debug output
#include "xintc.h" // Xilinx interrupt controller API for interrupt management
#include "xparameters.h" // Hardware-specific parameters from the BSP (e.g., device IDs)
#include "xtmrctr.h" // Xilinx timer driver API (used for FreeRTOS systick)

/* Project-specific includes. */
#include "nexys4IO.h"
#include "platform.h"
#include "tsl2561.h"
#include "pidtask.h"
#include "i2c.h" // I2C driver interface for AXI IIC controller

/* Project-specific includes - Custom headers for peripheral and sensor drivers
 */
#include "i2c.h" // I2C driver interface for AXI IIC controller
#include "nexys4IO.h" // Nexys A7 peripheral driver (LEDs, 7-segment display, RGB LEDs)
#include "tsl2561.h" // TSL2561 sensor driver interface

/* Peripheral definitions - Hardware-specific IDs and addresses */
#define N4IO_DEVICE_ID                                                         \
    XPAR_NEXYS4IO_0_DEVICE_ID // Device ID for Nexys4IO peripheral
#define N4IO_BASEADDR                                                          \
    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR // Base address for Nexys4IO AXI
                                     // interface
#define N4IO_HIGHADDR                                                          \
    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR // High address for Nexys4IO AXI
                                     // interface
#define INTC_DEVICE_ID                                                         \
    XPAR_INTC_0_DEVICE_ID // Device ID for AXI interrupt controller
#define I2C_BASE_ADDR                                                          \
    XPAR_AXI_IIC_0_BASEADDR // Base address for AXI IIC controller
#define I2C_DEV_ID_ADDR                                                        \
    XPAR_AXI_IIC_0_DEVICE_ID // Device ID for AXI IIC controller
#define GPIO_INTR_ID                                                           \
    XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR // Interrupt ID
                                                            // for GPIO
                                                            // button
                                                            // channel

/* GPIO channels - Channel definitions for GPIO instance */
#define BTN_CHANNEL 1 // GPIO channel 1 for button inputs
#define SW_CHANNEL 2  // GPIO channel 2 for switch inputs

/* RTOS definitions - Constants for FreeRTOS constructs */
#define MAIN_QUEUE_LENGTH ( 1 ) // Length of the queues for task communication
#define MAIN_DONT_BLOCK                                                        \
    ( (TickType_t) 0 ) // Non-blocking operation for queue send/receive

/********** Duty Cycle Related Constants **********/
#define max_duty 255 // max possible duty cycle for RGB1 Blue
#define min_duty 0   // min possible duty cycle for RGB1 Blue

#define lux_mask 0xFFFF // mask used for combining setpoint and lux values into one double

// macro for repeating code that checks for upper/lower saturation of setpoint and gains
#define UPDATE_SATURATING(val, inc, min_val, max_val, increase)	\
    do {														\
        if (increase)											\
		{                                        				\
            if ((val + inc) < max_val)                     		\
                val += inc;                                		\
            else                                               	\
                val = max_val;                             		\
        }														\
		else													\
		{		                                              	\
			if (val > inc)   									\
                val -= inc;                                		\
            else                                               	\
                val = min_val;                             		\
        }                                                      	\
    } while (0)


// Create Instances
static XGpio xInputGPIOInstance;

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

/* Interrupt-related globals (to be defined in tsl2561.c) */
 XIic IicInstance; // I2C instance
//XIntc Intc; // Shared interrupt controller instance, defined in main.c

/**
 * Sets up GPIO hardware for button interrupts.
 * Configures GPIO instance, installs interrupt handler, and enables interrupts
 * using FreeRTOS APIs.
 *
 * @return 0 on success, 1 on failure
 */
static int prvSetupHardware ( void );

/**
 * GPIO interrupt handler for button presses.
 * Signals the semaphore to wake the Parse Input task and clears the interrupt.
 *
 * @param pvUnused Unused parameter (required by ISR signature)
 */
static void gpio_intr ( void* pvUnused );

/**
 * Task to parse button presses and switch states.
 * Waits for the semaphore, reads GPIO inputs, and sends states to the PID task
 * via queue.
 *
 * @param p Unused parameter
 */
void Parse_Input_Task ( void* p );

/**
 * Task for PID control of RGB LED based on TSL2561 readings.
 * Adjusts LED brightness using PID algorithm and updates display task.
 *
 * @param p Pointer to PID_t structure
 */
void PID_Task ( void* p );

#endif /* MAIN_H */

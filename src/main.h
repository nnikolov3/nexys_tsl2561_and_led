#ifndef MAIN_H
#define MAIN_H

/* Standard C library includes. */
#include <stdlib.h>

/* Xilinx BSP and platform includes. */
#include "sleep.h"
#include "xgpio.h"
#include "xiic.h" // I2C driver
#include "xil_printf.h"
#include "xintc.h" // Interrupt controller
#include "xparameters.h"
#include "xtmrctr.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* Project-specific includes. */
#include "nexys4IO.h"
#include "platform.h"
#include "tsl2561.h"

/* Definitions for NEXYS4IO Peripheral */
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

/* Timer device ID for delay */
#define TIMER_DEVICE_ID XPAR_AXI_TIMER_0_DEVICE_ID

/* Interrupt controller device ID */
#define INTC_DEVICE_ID XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID

/* Timer interrupt ID (from xparameters.h) */
#define TIMER_INTR_ID XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR

#define BTN_CHANNEL 1
#define SW_CHANNEL 2

#define mainQUEUE_LENGTH ( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK (TickType_t) 0 // Use TickType_t for FreeRTOS consistency

// Create Instances
extern XGpio xInputGPIOInstance;

/* Static instances for drivers */
extern XIic    I2C_Instance;  // I2C instance for TSL2561
extern XTmrCtr TimerInstance; // Timer instance for delays
extern XIntc   IntcInstance;  // Interrupt controller instance

// Declare semaphores and queue (use FreeRTOS types)
extern SemaphoreHandle_t binary_sem;      // Semaphore for GPIO interrupts
extern SemaphoreHandle_t delay_semaphore; // Semaphore for delay synchronization
extern QueueHandle_t     xQueue;          // Queue for task communication

// Function Declarations
static void prvSetupHardware ( void );
static void gpio_intr ( void* pvUnused );

void sem_taken_que_tx ( void* p );
void que_rx ( void* p );
int  do_init ( void );
void nexys4io_selfTest ( void );
void timer_interrupt_handler ( void* CallbackRef,
                               u8    TmrCtrNumber ); // Updated signature

#endif /* MAIN_H */

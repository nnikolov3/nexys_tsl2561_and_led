

#ifndef MAIN_H
#define MAIN_H

/* Standard C library includes. */
#include <stdlib.h>

/* Xilinx BSP and platform includes. */
#include "sleep.h"
#include "xgpio.h"
#include "xil_printf.h"
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

/*Definitions for NEXYS4IO Peripheral*/
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

#define BTN_CHANNEL 1
#define SW_CHANNEL 2

#define mainQUEUE_LENGTH ( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK (portTickType) 0

// Create Instances
static XGpio xInputGPIOInstance;

static XIic i2c;

// Function Declarations
static void prvSetupHardware ( void );

// Declare a Semaphore
xSemaphoreHandle binary_sem;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;

static void gpio_intr ( void* pvUnused );

void sem_taken_que_tx ( void* p );

void que_rx ( void* p );

int do_init ( void );

void nexys4io_selfTest ( void );

#endif /* MAIN_H */

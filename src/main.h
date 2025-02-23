
/* Kernel includes. */
#include "FreeRTOS.h"
#include "nexys4IO.h"
#include "platform.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "xil_printf.h"
#include "xparameters.h"
#include <stdlib.h>

/* BSP includes. */
#include "sleep.h"
#include "xgpio.h"
#include "xtmrctr.h"

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

// Function Declarations
static void prvSetupHardware ( void );

// Declare a Semaphore
xSemaphoreHandle binary_sem;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;

static void gpio_intr ( void* pvUnused );

void sem_taken_que_tx ( void* p );

void que_rx ( void* p );

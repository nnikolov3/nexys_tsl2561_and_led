#ifndef MAIN_H
#define MAIN_H

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* BSP includes */
#include "sleep.h"
#include "xgpio.h"
#include "xiic.h"
#include "xil_printf.h"
#include "xintc.h"
#include "xparameters.h"
#include "xtmrctr.h"

/* Standard includes */
#include <stdlib.h>

/* Project-specific includes */
#include "nexys4IO.h"
#include "platform.h"
#include "tsl2561.h"

/* Definitions for NEXYS4IO Peripheral */
#define N4IO_DEVICE_ID   XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

/* I2C */
#define XIIC_BASEADDRESS XPAR_XIIC_0_BASEADDR
#define IIC_DEVICE_ID    XPAR_IIC_0_DEVICE_ID
#define INTC_DEVICE_ID   XPAR_INTC_0_DEVICE_ID
#define INTC             Intc
#define INTC_HANDLER     XIntc_InterruptHandler

#define BTN_CHANNEL      1
#define SW_CHANNEL       2

#define mainQUEUE_LENGTH ( 1 )

/* A block time of 0 means "don't block" */
#define mainDONT_BLOCK   (TickType_t) 0

/* Global variables */
extern XGpio xInputGPIOInstance;
extern SemaphoreHandle_t binary_sem;
extern QueueHandle_t xQueue;
extern XIntc Intc;

/* Function declarations */

static int prvSetupHardware( void );
static void gpio_intr( void* pvUnused );
void sem_taken_que_tx( void* p );
void que_rx( void* p );
static int do_init( void );
static int i2c_init( void );
static int nexys_init( void );
static void seven_seg_selfTest( void );

#endif /* MAIN_H */

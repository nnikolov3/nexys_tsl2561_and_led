

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
#include "pidtask.h"

/*Definitions for NEXYS4IO Peripheral*/
#define N4IO_DEVICE_ID XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR
#define I2C_BASE_ADDR XPAR_AXI_IIC_0_BASEADDR
#define I2C_DEV_ID_ADDR XPAR_AXI_IIC_0_DEVICE_ID

#define BTN_CHANNEL 1
#define SW_CHANNEL 2

#define mainQUEUE_LENGTH ( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK (portTickType) 0

// Create Instances
static XGpio xInputGPIOInstance;

// Declare a Semaphore
xSemaphoreHandle binary_sem;

/* Interrupt-related globals (to be defined in tsl2561.c) */
static XIic I2C_Instance; // I2C instance

/* The queue used by the queue send and queue receive tasks. */
//static xQueueHandle xQueue = NULL;

/* The queue used to transfer buttons/switches from the input task to PID task. */
static xQueueHandle toPID = NULL;

/* The queue used to transfer setpoint and lux from PID to display task. */
static xQueueHandle fromPID = NULL;

// Function Declarations
static void prvSetupHardware ( void );
static void gpio_intr ( void* pvUnused );

// A task which takes the Interrupt Semaphore and sends the btn/sw states to PID Task
void Parse_Input_Task ( void* p )
{

    uint8_t btns = 0x00;
    uint8_t sws = 0x00;
    uint16_t ValueToSend = 0x0000;

    while ( 1 )
        if ( xSemaphoreTake ( binary_sem, 500 ) )
        {
            btns = (NX4IO_getBtns() & 0x0C) >> 2; // get btnu/d and right justify
            sws = (uint8_t)(NX4IO_getSwitches() & 0x00FF); // get lower 8 switches
            ValueToSend |= ((btns << 8) | (sws)); // move btnu to bit 9 and bntd to bit 8
            //xil_printf ( "Queue Sent: %d\r\n", ValueToSend );
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );
            ValueToSend &= 0x0000 ; // clear btn/sw values for next time
        }
        else
            xil_printf ( "Semaphore time out\r\n" );
}

int do_init ( void );

void nexys4io_selfTest ( void );

#endif /* MAIN_H */

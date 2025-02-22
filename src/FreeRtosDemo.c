/* Sample FreeRTOS application

The application Demonstrates usage of Queues, Semaphores, Tasking model.

Application - Toggle LEDs on each button interrupt. Could modify to include
switches

Flow diagram
GPIO Interrupt (BTN) --> ( ISR )Send a Semaphore --> Task 1 (Catch the
Semaphore) -->
-->Task 1 - Send a Queue to Task -2 --> Task 2 Receive the queue --> Write to
GPIO (LED)

Assumptions:
o Nexys4IO is connected to LEDs. (Could also use another GPIO instance
 o GPIO_1 is capable of generating an interrupt and is connect to the switches
and buttons (see project #2 write-up for details) o AXI Timer 0 is a dual 32-bit
timer with the Timer 0 interrupt used to generate the FreeRTOS systick.
*/

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

// ISR, to handle interrupt of GPIO btns
// Give a Semaphore
static void gpio_intr ( void* pvUnused )
{
    xSemaphoreGiveFromISR ( binary_sem, NULL );

    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );
}

// A task which takes the Interrupt Semaphore and sends a queue to task 2.
void sem_taken_que_tx ( void* p )
{

    uint16_t ValueToSend = 0x00FF;

    while ( 1 )
        if ( xSemaphoreTake ( binary_sem, 500 ) )
        {
            xil_printf ( "Queue Sent: %d\r\n", ValueToSend );
            xQueueSend ( xQueue, &ValueToSend, mainDONT_BLOCK );
            ValueToSend = ~ValueToSend; // Toggle for next time.
        }
        else
            xil_printf ( "Semaphore time out\r\n" );
}

void que_rx ( void* p )
{

    uint16_t ReceivedValue;
    while ( 1 )
    {
        xQueueReceive ( xQueue, &ReceivedValue, portMAX_DELAY );
        // Write to LED.
        NX4IO_setLEDs ( ReceivedValue );
        xil_printf ( "Queue Received: %d\r\n", ReceivedValue );
    }
}

int main ( void )
{
    // Announcement
    xil_printf ( "Hello from FreeRTOS Example\r\n" );

    // Initialize the HW
    prvSetupHardware ( );

    // Create Semaphore
    vSemaphoreCreateBinary ( binary_sem );

    /* Create the queue */
    xQueue = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );

    /* Sanity check that the queue was created. */
    configASSERT ( xQueue );

    // Create Task1
    xTaskCreate ( sem_taken_que_tx,
                  (const char*) "TX",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  1,
                  NULL );

    // Create Task2
    xTaskCreate ( que_rx, "RX", configMINIMAL_STACK_SIZE, NULL, 2, NULL );

    // Start the Scheduler
    xil_printf ( "Starting the scheduler\r\n" );
    xil_printf ( "Push Button to change the LED pattern\r\n\r\n" );
    vTaskStartScheduler ( );

    return -1;
}

static void prvSetupHardware ( void )
{
    uint32_t xStatus;

    const unsigned char ucSetToInput = 0xFFU;

    xil_printf ( "Initializing GPIO's\r\n" );

    /* Initialize the GPIO for the button inputs. */

    xStatus =
        XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );

    if ( xStatus == XST_SUCCESS )
    {
        /* Install the handler defined in this task for the button input.
        *NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
        must be used for this purpose. */
        xStatus = xPortInstallInterruptHandler (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR,
            gpio_intr,
            NULL );

        if ( xStatus == pdPASS )
        {
            xil_printf ( "Buttons interrupt handler installed\r\n" );

            /* Set switches and buttons to input. */
            XGpio_SetDataDirection (
                &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
            XGpio_SetDataDirection (
                &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );

            /* Enable the button input interrupts in the interrupt controller.
            *NOTE* The vPortEnableInterrupt() API function must be used for this
            purpose. */

            vPortEnableInterrupt (
                XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

            /* Enable GPIO channel interrupts on button channel. Can moodify to
             * include switches */
            XGpio_InterruptEnable ( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
            XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
        }

        // initialize the Nexys4 driver
        uint32_t status = NX4IO_initialize ( N4IO_BASEADDR );
        if ( status != XST_SUCCESS )
        {
            return XST_FAILURE;
        }
    }

    configASSERT ( ( xStatus == pdPASS ) );
}

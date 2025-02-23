/*
 * main.c - Entry point for TSL2561-based PID control system
 *
 * Purpose: This file initializes the FreeRTOS environment, sets up the TSL2561
 *          sensor driver, and creates tasks for reading lux values, running a
 *          PID controller to adjust LED brightness, and handling user input.
 *          It serves as the starting point for the ECE 544 Project #2 system.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* Include necessary headers */
#include "main.h"

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

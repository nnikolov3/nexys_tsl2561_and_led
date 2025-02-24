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

/* Global variables (defined in main.h) */
XIic              I2C_Instance;           // I2C instance for TSL2561
XTmrCtr           TimerInstance;          // Timer instance for delays
XIntc             IntcInstance;           // Interrupt controller instance
SemaphoreHandle_t binary_sem      = NULL; // Semaphore for GPIO interrupts
SemaphoreHandle_t delay_semaphore = NULL; // Semaphore for delay synchronization
QueueHandle_t     xQueue          = NULL; // Queue for task communication
XGpio             xInputGPIOInstance;

int main ( void )
{
    // Announcement
    xil_printf ( "Hello from FreeRTOS Example\r\n" );

    // Initialize the HW
    prvSetupHardware ( );
    if ( do_init ( ) != XST_SUCCESS )
    {
        xil_printf ( "Hardware initialization failed\r\n" );
        return -1;
    }
    nexys4io_selfTest ( );
    if ( tsl2561_init ( &I2C_Instance ) != XST_SUCCESS )
    {
        xil_printf ( "TSL2561 initialization failed\r\n" );
        return -1;
    }

    // Create Semaphore
    binary_sem = xSemaphoreCreateBinary ( );
    if ( binary_sem == NULL )
    {
        xil_printf ( "Failed to create binary semaphore\r\n" );
        return -1;
    }

    // Create the queue
    xQueue = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );
    if ( xQueue == NULL )
    {
        xil_printf ( "Failed to create queue\r\n" );
        return -1;
    }

    // Create Task1
    if ( xTaskCreate ( sem_taken_que_tx,
                       (const char*) "TX",
                       configMINIMAL_STACK_SIZE,
                       NULL,
                       1,
                       NULL ) != pdPASS )
    {
        xil_printf ( "Failed to create TX task\r\n" );
        return -1;
    }

    // Create Task2
    if ( xTaskCreate (
             que_rx, "RX", configMINIMAL_STACK_SIZE, NULL, 2, NULL ) != pdPASS )
    {
        xil_printf ( "Failed to create RX task\r\n" );
        return -1;
    }

    // Start the Scheduler
    xil_printf ( "Starting the scheduler\r\n" );
    xil_printf ( "Push Button to change the LED pattern\r\n\r\n" );
    vTaskStartScheduler ( );

    return -1; // Should never reach here
}

static void prvSetupHardware ( void )
{
    uint32_t            xStatus;
    const unsigned char ucSetToInput = 0xFFU;

    xil_printf ( "Initializing GPIO's\r\n" );

    /* Initialize the GPIO for the button inputs. */
    xStatus =
        XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
    if ( xStatus != XST_SUCCESS )
    {
        xil_printf ( "GPIO initialization failed\r\n" );
        configASSERT ( 0 );
    }

    /* Install the handler defined in this task for the button input.
     * NOTE: The FreeRTOS defined xPortInstallInterruptHandler() API function
     * must be used for this purpose. */
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
         * NOTE: The vPortEnableInterrupt() API function must be used for this
         * purpose. */
        vPortEnableInterrupt (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

        /* Enable GPIO channel interrupts on button channel. Can modify to
         * include switches */
        XGpio_InterruptEnable ( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
        XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
    }
    else
    {
        xil_printf ( "Failed to install GPIO interrupt handler\r\n" );
        configASSERT ( 0 );
    }
}

// ISR, to handle interrupt of GPIO buttons
// Give a Semaphore
static void gpio_intr ( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );

    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );

    // Yield if a higher priority task was unblocked
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
}

// A task which takes the Interrupt Semaphore and sends a queue to task 2.
void sem_taken_que_tx ( void* p )
{
    uint16_t ValueToSend = 0x00FF;

    while ( 1 )
    {
        if ( xSemaphoreTake ( binary_sem, 500 / portTICK_PERIOD_MS ) )
        {
            xil_printf ( "Queue Sent: %d\r\n", ValueToSend );
            if ( xQueueSend ( xQueue, &ValueToSend, mainDONT_BLOCK ) != pdPASS )
            {
                xil_printf ( "Queue send failed\r\n" );
            }
            ValueToSend = ~ValueToSend; // Toggle for next time.
        }
        else
        {
            xil_printf ( "Semaphore timeout\r\n" );
        }
    }
}

void que_rx ( void* p )
{
    uint16_t ReceivedValue;

    while ( 1 )
    {
        if ( xQueueReceive ( xQueue, &ReceivedValue, portMAX_DELAY ) == pdPASS )
        {
            // Write to LED.
            NX4IO_setLEDs ( ReceivedValue );
            xil_printf ( "Queue Received: %d\r\n", ReceivedValue );
        }
        else
        {
            xil_printf ( "Queue receive failed\r\n" );
        }
    }
}

/****************************************************************************
 * Initialize the system
 *
 * This function is executed once at start-up and after resets. It initializes
 * the peripherals and registers the interrupt handler(s)
 *****************************************************************************/
int do_init ( void )
{
    int status; // Status from Xilinx Lib calls

    // Initialize the Nexys4IO and Pmod544IO hardware and drivers
    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        return XST_FAILURE;
    }

    // Get AXI I2C device configuration
    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        return XST_FAILURE;
    }

    // Initialize the I2C driver
    status =
        XIic_CfgInitialize ( &I2C_Instance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    XIic_Start ( &I2C_Instance );

    /*
     * Initialize the timer for delays
     */
    status = XTmrCtr_Initialize ( &TimerInstance, TIMER_DEVICE_ID );
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    // Set timer to count down and generate interrupts
    XTmrCtr_SetOptions (
        &TimerInstance, 0, XTC_DOWN_COUNT_OPTION | XTC_INT_MODE_OPTION );

    // Start the timer
    XTmrCtr_Start ( &TimerInstance, 0 );

    // Initialize the interrupt controller
    status = XIntc_Initialize ( &IntcInstance, INTC_DEVICE_ID );
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    // Connect the timer interrupt handler
    status = XIntc_Connect ( &IntcInstance,
                             TIMER_INTR_ID,
                             (XInterruptHandler) timer_interrupt_handler,
                             NULL ); // Changed CallBackRef to NULL
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    // Start the interrupt controller
    status = XIntc_Start ( &IntcInstance, XIN_REAL_MODE );
    if ( status != XST_SUCCESS )
    {
        return status;
    }

    // Enable the timer interrupt
    XIntc_Enable ( &IntcInstance, TIMER_INTR_ID );

    // Set the interrupt handler for the timer
    XTmrCtr_SetHandler ( &TimerInstance,
                         timer_interrupt_handler,
                         NULL ); // Changed CallBackRef to NULL
    XTmrCtr_SetHandler ( &TimerInstance, timer_interrupt_handler, NULL );

    // Enable MicroBlaze interrupts
    microblaze_enable_interrupts ( );

    return XST_SUCCESS;
}

/**
 * Interrupt-driven delay for a specified number of milliseconds
 * Non-blocking, uses RTOS semaphore for synchronization
 */
void delay_ms ( uint32_t ms )
{
    uint32_t ticks = ( ms * 100000 ) /
                     1000; // Convert ms to ticks (100 MHz = 10 ns per tick)

    // Create semaphore for delay synchronization (if not already created)
    if ( delay_semaphore == NULL )
    {
        delay_semaphore = xSemaphoreCreateBinary ( );
        if ( delay_semaphore == NULL )
        {
            xil_printf ( "Failed to create delay semaphore\r\n" );
            return; // Semaphore creation failed
        }
    }

    // Reset and configure timer for the delay
    XTmrCtr_Reset ( &TimerInstance, 0 );
    XTmrCtr_SetResetValue ( &TimerInstance, 0, ticks );

    // Start the timer (it will generate an interrupt when done)
    XTmrCtr_Start ( &TimerInstance, 0 );

    // Wait for the semaphore (non-blocking in RTOS context, allows other tasks
    // to run)
    if ( xSemaphoreTake ( delay_semaphore, portMAX_DELAY ) != pdTRUE )
    {
        xil_printf ( "Semaphore take failed in delay_ms\r\n" );
    }

    // Stop the timer after delay (optional, to save power)
    XTmrCtr_Stop ( &TimerInstance, 0 );
}

/**
 * Timer interrupt handler
 * Signals the delay completion via the semaphore
 */
void timer_interrupt_handler ( void* CallbackRef, u8 TmrCtrNumber )
{
    // Clear the timer interrupt
    // Note: Use XTmrCtr_InterruptHandler or check xtmrctr_l.h for correct
    // function
    XTmrCtr_InterruptHandler (
        &TimerInstance ); // Use InterruptHandler instead of InterruptClear

    // Give the semaphore to signal delay completion
    if ( delay_semaphore != NULL )
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR ( delay_semaphore, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
    }
}

/****************************************************************************/
/**
 * nexys4io_selfTest() - performs a self test on the NexysA7 peripheral
 *
 * @brief This is mostly a visual test to confirm that the 7-segment display and
 * RGB LEDs hardware and drivers are operating correctly. The test does the
 * following: o sends pattern(s) to the LEDs on the board o Writes a message on
 * the 7-segment display o individually lights the RGB LEDs o sets the RGB2 LED
 * to several values that can be observed o Turns off the LEDs and blanks the
 * 7-segment digits and decimal points
 */
void nexys4io_selfTest ( void )
{
    xil_printf ( "Starting Nexys4IO self test...\r\n" );

    xil_printf ( "\tcheck functionality of 7-segment display\r\n" );
    // Set the display digits to -ECE544- and turn off the decimal points using
    // the "raw" set functions.
    NX4IO_SSEG_setSSEG_DATA ( SSEGHI, 0x0058E30E );
    NX4IO_SSEG_setSSEG_DATA ( SSEGLO, 0x00144116 );
    usleep ( 2000 * 1000 );

    xil_printf ( "...Nexys4IO self test complete\r\n" );
}

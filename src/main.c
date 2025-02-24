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

/* Global variables (defined in main.c) */
XGpio xInputGPIOInstance; // Define the GPIO instance here to resolve linker
                          // errors
XIic              I2C_Instance;      // I2C instance for TSL2561
SemaphoreHandle_t binary_sem = NULL; // Semaphore for GPIO interrupts
QueueHandle_t     xQueue     = NULL; // Queue for task communication
XIntc             IntcInstance;

int main ( void )
{
    // Announcement
    xil_printf ( "Hello from FreeRTOS Example\r\n" );

    // Initialize the HW
    xil_printf ( "Initializing hardware...\r\n" );
    prvSetupHardware ( );
    if ( do_init ( ) != XST_SUCCESS )
    {
        xil_printf ( "Hardware initialization failed\r\n" );
        while ( 1 )
            ; // Halt to debug
    }
    xil_printf ( "Hardware initialized successfully, continuing...\r\n" );

    xil_printf ( "Nexys4IO self-test starting...\r\n" );
    nexys4io_selfTest ( );
    xil_printf ( "Nexys4IO self-test completed\r\n" );

    xil_printf ( "Initializing TSL2561...\r\n" );
    if ( tsl2561_init ( &I2C_Instance ) != XST_SUCCESS )
    {
        xil_printf ( "TSL2561 initialization failed\r\n" );
        while ( 1 )
            ; // Halt to debug
    }
    xil_printf ( "TSL2561 initialized successfully\r\n" );

    // Create Semaphore
    xil_printf ( "Creating binary semaphore...\r\n" );
    vSemaphoreCreateBinary ( binary_sem );
    if ( binary_sem == NULL )
    { // Check if creation failed
        xil_printf ( "Failed to create binary semaphore\r\n" );
        return -1;
    }
    xil_printf ( "Binary semaphore created successfully\r\n" );

    /* Create the queue */
    xil_printf ( "Creating queue...\r\n" );
    xQueue = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );
    if ( xQueue == NULL )
    {
        xil_printf ( "Failed to create queue\r\n" );
        return -1;
    }
    xil_printf ( "Queue created successfully\r\n" );

    // Create Task1
    xil_printf ( "Creating TX task...\r\n" );
    xTaskCreate ( sem_taken_que_tx,
                  (const char*) "TX",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  1,
                  NULL );

    // Create Task2
    xil_printf ( "Creating RX task...\r\n" );
    xTaskCreate ( que_rx, "RX", configMINIMAL_STACK_SIZE, NULL, 2, NULL );

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

    configASSERT ( xStatus == pdPASS );
}

// ISR, to handle interrupt of GPIO buttons
// Give a Semaphore
static void gpio_intr ( void* pvUnused )
{
    xil_printf ( "GPIO interrupt triggered\r\n" ); // Debug print
    xSemaphoreGiveFromISR ( binary_sem, NULL );

    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );
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
            xQueueSend ( xQueue, &ValueToSend, mainDONT_BLOCK );
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
        xQueueReceive ( xQueue, &ReceivedValue, portMAX_DELAY );
        // Write to LED.
        NX4IO_setLEDs ( ReceivedValue );
        xil_printf ( "Queue Received: %d\r\n", ReceivedValue );
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
    int status; // status from Xilinx Lib calls

    xil_printf ( "Initializing system...\r\n" );

    // initialize the Nexys4IO and Pmod544IO hardware and drivers
    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "Nexys4IO initialization failed, status: %d\r\n", status );
        return XST_FAILURE;
    }
    xil_printf ( "Nexys4IO initialized successfully\r\n" );

    // Get AXI I2C device configuration
    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        xil_printf ( "I2C configuration lookup failed\r\n" );
        return XST_FAILURE;
    }
    // Initialize the I2C driver
    status =
        XIic_CfgInitialize ( &I2C_Instance, ConfigPtr, ConfigPtr->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "I2C initialization failed, status: %d\r\n", status );
        return status;
    }
    xil_printf ( "I2C initialized successfully\r\n" );

    XIic_Start ( &I2C_Instance );
    xil_printf ( "I2C started successfully\r\n" );

    // Check for pending interrupts before enabling MicroBlaze interrupts
    xil_printf ( "Checking interrupt status before enabling MicroBlaze "
                 "interrupts...\r\n" );
    if ( XIntc_GetIntrStatus ( &IntcInstance ) &
         XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR )
    {
        xil_printf ( "GPIO interrupt pending, clearing...\r\n" );
        XIntc_AckIntr (
            &IntcInstance,
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );
    }

    // Enable MicroBlaze interrupts with debugging
    xil_printf ( "Enabling MicroBlaze interrupts...\r\n" );
    microblaze_enable_interrupts ( );
    xil_printf ( "MicroBlaze interrupts enabled successfully, continuing to "
                 "next steps...\r\n" );

    return XST_SUCCESS;
}

/****************************************************************************/
/**
 * nexys4io_selfTest() - performs a self test on the NexysA7 peripheral
 *
 * @brief This is mostly a visual test to confirm that the 7-segment display and
 * RGB LEDs hardware and drivers are operating correctly.  The test does the
 * following: o sends pattern(s) to the LEDs on the board o Writes a message on
 * the 7-segment display o individually lights the RGB LEDs o sets the RGB2 LED
 * to several values that can be observed o Turns off the LEDs and blanks the
 * 7-segment digits and decimal points
 */
void nexys4io_selfTest ( void )
{
    xil_printf ( "Starting Nexys4IO self test...\r\n" );

    xil_printf ( "\tcheck functionality of 7-segment display\r\n" );
    // set the display digits to -ECE544- and turn off
    // the decimal points using the "raw" set functions.
    NX4IO_SSEG_setSSEG_DATA ( SSEGHI, 0x0058E30E );
    NX4IO_SSEG_setSSEG_DATA ( SSEGLO, 0x00144116 );
    usleep ( 2000 * 1000 );

    xil_printf ( "...Nexys4IO self test complete\r\n" );
    return;
}

/*
 * main.c - Entry point for TSL2561-based system with button handling and sensor
 * reading
 *
 * Purpose: Initializes FreeRTOS, sets up GPIO with interrupts for buttons,
 *          initializes the TSL2561 sensor via I2C (polling), and creates tasks
 * for button handling and sensor reading. Designed for Nexys A7 with
 * Microblaze, AXI, and TSL2561 sensor. Interrupt controller setup moved outside
 * do_init().
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* Include necessary headers */
#include "main.h"

/* Global variables */
XGpio xInputGPIOInstance;            // GPIO instance for buttons
SemaphoreHandle_t binary_sem = NULL; // Semaphore for GPIO interrupts
QueueHandle_t xQueue         = NULL; // Queue for button-to-LED communication
XIntc Intc;                          // Shared interrupt controller

/* Function prototypes */
static void gpio_intr( void* pvUnused ); // GPIO interrupt handler
void sem_taken_que_tx(
    void* p );          // Task to handle button presses and send LED patterns
void que_rx( void* p ); // Task to receive LED patterns and update LEDs
void sensor_task( void* p );            // Task to read TSL2561 sensor data
static int do_init( void );             // System initialization
static int nexys_init( void );          // Nexys A7 peripheral initialization
static void seven_seg_selfTest( void ); // 7-segment display self-test
static int prvSetupHardware( void );    // GPIO hardware setup with interrupts

/**
 * Main entry point for the TSL2561 system.
 * Initializes hardware, sets up interrupts, creates FreeRTOS constructs, and
 * starts the scheduler.
 */
int main( void )
{
    xil_printf( "\r\n==================================================\r\n" );
    xil_printf( "||          *** Starting TSL2561 System ***     ||\r\n" );
    xil_printf( "==================================================\r\n" );

    /* Perform system initialization (no interrupt setup here) */
    if( do_init( ) != XST_SUCCESS )
    {
        xil_printf( "\r\n[ERROR] *** Hardware initialization failed ***\r\n" );
        while( 1 )
            ; // Halt to debug
    }

    /* Initialize and start interrupt controller after hardware setup */
    xil_printf( "\r\n>>> Initializing Interrupt Controller\r\n" );
    if( XIntc_Initialize( &Intc, INTC_DEVICE_ID ) != XST_SUCCESS )
    {
        xil_printf(
            "    [ERROR] *** Interrupt controller init failed ***\r\n" );
        return -1;
    }
    if( XIntc_Start( &Intc, XIN_REAL_MODE ) != XST_SUCCESS )
    {
        xil_printf(
            "    [ERROR] *** Interrupt controller start failed ***\r\n" );
        return -1;
    }
    xil_printf( "    [SUCCESS] Interrupt controller ready\r\n" );

    /* Enable global interrupts */
    Xil_ExceptionInit( );
    Xil_ExceptionRegisterHandler( XIL_EXCEPTION_ID_INT,
                                  (Xil_ExceptionHandler) XIntc_InterruptHandler,
                                  &Intc );
    Xil_ExceptionEnable( );
    xil_printf( "    [SUCCESS] Global interrupts enabled\r\n" );

    /* Create semaphore for button interrupts */
    xil_printf( "\r\n>>> Creating Binary Semaphore\r\n" );
    vSemaphoreCreateBinary( binary_sem );
    if( binary_sem == NULL )
    {
        xil_printf( "    [ERROR] *** Failed to create semaphore ***\r\n" );
        return -1;
    }
    xil_printf( "    [SUCCESS] Semaphore created\r\n" );

    /* Create queue for button-to-LED communication */
    xil_printf( "\r\n>>> Creating Queue\r\n" );
    xQueue = xQueueCreate( MAIN_QUEUE_LENGTH, sizeof( uint16_t ) );
    if( xQueue == NULL )
    {
        xil_printf( "    [ERROR] *** Failed to create queue ***\r\n" );
        return -1;
    }
    xil_printf( "    [SUCCESS] Queue created\r\n" );

    /* Create tasks */
    xil_printf( "\r\n>>> Creating Tasks\r\n" );
    xil_printf( "    - TX Task... " );
    xTaskCreate( sem_taken_que_tx,
                 "TX",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
    xil_printf( "[DONE]\r\n" );

    xil_printf( "    - RX Task... " );
    xTaskCreate( que_rx,
                 "RX",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 2,
                 NULL );
    xil_printf( "[DONE]\r\n" );

    xil_printf( "    - Sensor Task... " );
    xTaskCreate( sensor_task,
                 "SENSOR",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 3,
                 NULL );
    xil_printf( "[DONE]\r\n" );

    /* Start the FreeRTOS scheduler */
    xil_printf( "\r\n==================================================\r\n" );
    xil_printf( "||          *** Starting FreeRTOS Scheduler *** ||\r\n" );
    xil_printf( "||          Press buttons to toggle LEDs        ||\r\n" );
    xil_printf( "==================================================\r\n" );
    vTaskStartScheduler( );

    xil_printf( "\r\n[ERROR] *** Scheduler failed to start ***\r\n" );
    return -1; // Should never reach here
}

/**
 * GPIO interrupt handler for button presses.
 * Signals the semaphore to wake the TX task and clears the interrupt.
 *
 * @param pvUnused Unused parameter (required by ISR signature)
 */
static void gpio_intr( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xil_printf( ">>> GPIO Interrupt Triggered\r\n" );

    /* Signal the semaphore to notify the TX task */
    xSemaphoreGiveFromISR( binary_sem, &xHigherPriorityTaskWoken );
    XGpio_InterruptClear( &xInputGPIOInstance, XGPIO_IR_MASK );

    /* Yield to higher-priority tasks if necessary */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * Task to handle button presses and send LED patterns.
 * Waits for the semaphore from the interrupt, toggles an LED pattern, and sends
 * it via the queue.
 *
 * @param p Unused parameter
 */
void sem_taken_que_tx( void* p )
{
    uint16_t ValueToSend = 0x00FF; // Initial LED pattern

    xil_printf( "\r\n>>> TX Task Started\r\n" );

    while( 1 )
    {
        /* Wait for button interrupt via semaphore */
        if( xSemaphoreTake( binary_sem, pdMS_TO_TICKS( 500 ) ) == pdTRUE )
        {
            xil_printf( "    [TX] Sent LED pattern: 0x%04X\r\n", ValueToSend );
            xQueueSend( xQueue, &ValueToSend, MAIN_DONT_BLOCK );
            ValueToSend = ~ValueToSend; // Toggle pattern for next press
        }
        else
        {
            xil_printf( "    [TX] Timeout waiting for button\r\n" );
        }
        vTaskDelay( pdMS_TO_TICKS( 100 ) ); // Debounce delay
    }
}

/**
 * Task to receive LED patterns and update LEDs.
 * Continuously receives from the queue and updates the Nexys A7 LEDs.
 *
 * @param p Unused parameter
 */
void que_rx( void* p )
{
    uint16_t ReceivedValue;

    xil_printf( "\r\n>>> RX Task Started\r\n" );

    while( 1 )
    {
        /* Wait indefinitely for a pattern from the queue */
        if( xQueueReceive( xQueue, &ReceivedValue, portMAX_DELAY ) == pdTRUE )
        {
            NX4IO_setLEDs( ReceivedValue );
            xil_printf( "    [RX] Received: 0x%04X -> LEDs Updated\r\n",
                        ReceivedValue );
        }
    }
}

/**
 * Task to read TSL2561 sensor data.
 * Periodically reads CH0 and CH1 values from the TSL2561 sensor.
 *
 * @param p Unused parameter
 */
void sensor_task( void* p )
{
    xil_printf( "\r\n>>> Sensor Task Started\r\n" );

    while( 1 )
    {
        /* Read sensor channels */
        uint16_t ch0 = tsl2561_readChannel( &IicInstance, TSL2561_CHANNEL_0 );
        uint16_t ch1 = tsl2561_readChannel( &IicInstance, TSL2561_CHANNEL_1 );
        xil_printf( "    [SENSOR] CH0: %5u | CH1: %5u\r\n", ch0, ch1 );

        vTaskDelay( pdMS_TO_TICKS( 500 ) ); // Read every 500ms
    }
}

/**
 * System initialization function.
 * Initializes hardware components (GPIO, Nexys, I2C, TSL2561) without interrupt
 * setup.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
static int do_init( void )
{
    int status;

    xil_printf( "\r\n==================================================\r\n" );
    xil_printf( "||           *** Initializing System ***        ||\r\n" );
    xil_printf( "==================================================\r\n" );

    /* Initialize Nexys A7 peripherals */
    xil_printf( "\r\n>>> Initializing Nexys A7\r\n" );
    status = nexys_init( );
    if( status != XST_SUCCESS )
    {
        xil_printf( "    [ERROR] *** Nexys initialization failed ***\r\n" );
        return XST_FAILURE;
    }
    xil_printf( "    [SUCCESS] Nexys initialized\r\n" );

    /* Initialize I2C (polling mode, from i2c.c) */
    xil_printf( "\r\n>>> Initializing I2C\r\n" );
    status = i2c_init( );
    if( status != XST_SUCCESS )
    {
        xil_printf( "    [ERROR] *** I2C initialization failed ***\r\n" );
        return XST_FAILURE;
    }
    xil_printf( "    [SUCCESS] I2C initialized\r\n" );

    /* Perform I2C self-test */
    status = XIic_SelfTest( &IicInstance );
    if( status != XST_SUCCESS )
    {
        xil_printf( "    [ERROR] *** I2C self-test failed ***\r\n" );
        return XST_FAILURE;
    }
    xil_printf( "    [SUCCESS] I2C self-test passed\r\n" );

    /* Scan I2C bus */
    xil_printf( "\r\n>>> Scanning I2C Bus\r\n" );
    i2c_scan( &IicInstance );

    /* Initialize TSL2561 sensor */
    xil_printf( "\r\n>>> Initializing TSL2561 Sensor\r\n" );
    status = tsl2561_init( &IicInstance );
    if( status != XST_SUCCESS )
    {
        xil_printf( "    [ERROR] *** TSL2561 initialization failed ***\r\n" );
        return XST_FAILURE;
    }
    xil_printf( "    [SUCCESS] TSL2561 initialized\r\n" );

    /* Perform 7-segment self-test */
    xil_printf( "\r\n>>> Running 7-Segment Self-Test\r\n" );
    xil_printf( "    Displaying 'ECE544' to verify system readiness\r\n" );
    seven_seg_selfTest( );

    /* Initialize GPIO for buttons with interrupts */
    xil_printf( "\r\n>>> Initializing Button GPIO\r\n" );
    status = prvSetupHardware( );
    if( status != 0 )
    {
        xil_printf( "    [ERROR] *** Button initialization failed ***\r\n" );
        return XST_FAILURE;
    }
    xil_printf( "    [SUCCESS] Buttons initialized\r\n" );

    xil_printf( "\r\n==================================================\r\n" );
    xil_printf( "||          *** System Initialization Complete *** ||\r\n" );
    xil_printf( "==================================================\r\n" );
    return XST_SUCCESS;
}

/**
 * Nexys A7 peripheral initialization.
 * Sets up the Nexys4IO peripheral for LEDs and 7-segment display.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
static int nexys_init( void )
{
    int status;

    xil_printf( "    Initializing Nexys A7 peripherals...\r\n" );
    status = NX4IO_initialize( N4IO_BASEADDR );
    if( status != XST_SUCCESS )
    {
        xil_printf( "    [ERROR] *** Nexys4IO initialization failed, status: "
                    "%d ***\r\n",
                    status );
        return XST_FAILURE;
    }
    xil_printf( "    Nexys4IO initialized successfully\r\n" );
    return XST_SUCCESS;
}

/**
 * 7-segment display self-test.
 * Displays "ECE544" on the 7-segment display for 2 seconds to verify
 * functionality.
 */
static void seven_seg_selfTest( void )
{
    xil_printf( "    Starting 7-segment display test...\r\n" );
    xil_printf( "    Displaying 'ECE544' for 2 seconds\r\n" );
    NX4IO_SSEG_setSSEG_DATA( SSEGHI, 0x0058E30E ); // Display "ECE544"
    NX4IO_SSEG_setSSEG_DATA( SSEGLO, 0x00144116 );
    usleep( 2000 * 1000 );
    xil_printf( "    7-segment test completed\r\n" );
}

/**
 * GPIO hardware setup.
 * Configures the GPIO for button inputs and installs the interrupt handler
 * using FreeRTOS APIs.
 *
 * @return 0 if setup succeeds, 1 otherwise
 */
static int prvSetupHardware( void )
{
    uint32_t xStatus;
    const unsigned char ucSetToInput = 0xFFU;

    xil_printf( "    Initializing GPIO for buttons...\r\n" );

    /* Initialize the GPIO for the button inputs */
    xStatus =
        XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
    if( xStatus != XST_SUCCESS )
    {
        xil_printf( "    [ERROR] *** GPIO initialization failed ***\r\n" );
        configASSERT( 0 );
    }

    /* Install the handler defined in this task for the button input */
    xStatus = xPortInstallInterruptHandler(
        XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR,
        gpio_intr,
        NULL );

    if( xStatus == pdPASS )
    {
        xil_printf( "    [SUCCESS] Buttons interrupt handler installed\r\n" );

        /* Set switches and buttons to input */
        XGpio_SetDataDirection(
            &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
        XGpio_SetDataDirection( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );

        /* Enable the button input interrupts in the interrupt controller */
        vPortEnableInterrupt(
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

        /* Enable GPIO channel interrupts on button channel */
        XGpio_InterruptEnable( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
        XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
    }
    else
    {
        xil_printf( "    [ERROR] *** Failed to install GPIO interrupt handler "
                    "***\r\n" );
        configASSERT( 0 );
    }

    configASSERT( xStatus == pdPASS );
    return ( xStatus == pdPASS ) ? 0 : 1;
}

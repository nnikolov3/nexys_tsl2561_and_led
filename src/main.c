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
XGpio xInputGPIOInstance;            // Define the GPIO
XIic I2C_Instance;                   // I2C instance for TSL2561
SemaphoreHandle_t binary_sem = NULL; // Semaphore for GPIO interrupts
QueueHandle_t xQueue         = NULL; // Queue for task communication
XIntc IntcInstance;

int main( void )
{

    if( do_init( ) != XST_SUCCESS )
    {
        xil_printf( "Hardware initialization failed\r\n" );
        while( 1 )
            ; // Halt to debug
    }

    // Create Semaphore
    xil_printf( "Creating binary semaphore...\r\n" );
    vSemaphoreCreateBinary( binary_sem );

    if( binary_sem == NULL )
    { // Check if creation failed
        xil_printf( "Failed to create binary semaphore\r\n" );
        return -1;
    }

    xil_printf( "Binary semaphore created successfully\r\n" );

    /* Create the queue */
    xil_printf( "Creating queue...\r\n" );
    xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint16_t ) );
    if( xQueue == NULL )
    {
        xil_printf( "Failed to create queue\r\n" );
        return -1;
    }

    xil_printf( "Queue created successfully\r\n" );

    // Create Task1
    xil_printf( "Creating TX task...\r\n" );
    xTaskCreate( sem_taken_que_tx,
                 (const char*) "TX",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 1,
                 NULL );

    // Create Task2
    xil_printf( "Creating RX task...\r\n" );
    xTaskCreate( que_rx, "RX", configMINIMAL_STACK_SIZE, NULL, 2, NULL );

    // Start the Scheduler
    xil_printf( "Starting the scheduler\r\n" );
    xil_printf( "Push Button to change the LED pattern\r\n\r\n" );
    vTaskStartScheduler( );

    return -1; // Should never reach here
}

// ISR, to handle interrupt of GPIO buttons
// Give a Semaphore
static void gpio_intr( void* pvUnused )
{
    xil_printf( "GPIO interrupt triggered\r\n" ); // Debug print
    xSemaphoreGiveFromISR( binary_sem, NULL );

    XGpio_InterruptClear( &xInputGPIOInstance, XGPIO_IR_MASK );
}

// A task which takes the Interrupt Semaphore and sends a queue to task 2.
void sem_taken_que_tx( void* p )
{
    uint16_t ValueToSend = 0x00FF;

    while( 1 )
    {
        if( xSemaphoreTake( binary_sem, 500 / portTICK_PERIOD_MS ) )
        {
            xil_printf( "Queue Sent: %d\r\n", ValueToSend );
            xQueueSend( xQueue, &ValueToSend, mainDONT_BLOCK );
            ValueToSend = ~ValueToSend; // Toggle for next time.
        }
        else
        {
            xil_printf( "Semaphore timeout\r\n" );
        }
    }
}

void que_rx( void* p )
{
    uint16_t ReceivedValue;

    while( 1 )
    {
        xQueueReceive( xQueue, &ReceivedValue, portMAX_DELAY );
        // Write to LED.
        NX4IO_setLEDs( ReceivedValue );
        xil_printf( "Queue Received: %d\r\n", ReceivedValue );
    }
}

/****************************************************************************
 * Initialize the system
 *
 * This function is executed once at start-up and after resets. It initializes
 * the peripherals and registers the interrupt handler(s)
 *****************************************************************************/
int do_init( void )
{
    // Announcement
    xil_printf( "Hello from FreeRTOS Example\r\n" );

    // Initialize the HW
    xil_printf( "Initializing hardware...\r\n" );
    int status = 0;

    xil_printf( "\rButtons initialization\n" );
    status = prvSetupHardware( );
    if( status )
    {
        xil_printf( "\r HW initialization for btns failed\n" );
        return 1;
    }
    xil_printf( "\r HW initialization SUCCESS\n" );

    status = nexys_init( );
    if( status )
    {
        xil_printf( "\r Nexys initialization failed\n" );
        return 1;
    }
    xil_printf( "\r Nexys initialization SUCCESS \n" );
    status = i2c_init( );
    if( status )
    {
        xil_printf( "\r I2C initialization failed\n" );
        return 1;
    }
    xil_printf( "\r I2C initialization SUCCESS \n" );
    status = XIic_SelfTest( &I2C_Instance );
    if( status )
    {
        xil_printf( "\r I2C Self test failed \n" );
        return 1;
    }
    i2c_scan( &I2C_Instance );
    xil_printf( "\r I2C SelfTest SUCCESS \n" );

    xil_printf( "\r TSL2561 initialization\n" );
    status = tsl2561_init( &I2C_Instance );
    if( status )
    {
        xil_printf( "\r\r TSL2561 initialization failed\n" );
        return 1;
    }
    xil_printf( "\r\r TSL2561 initialization SUCCESS \n" );

    xil_printf( "\r\r If 7segment is working then we are all good \n" );
    seven_seg_selfTest( );

    return 0;
}

static int nexys_init( void )
{
    int status; // status from Xilinx Lib calls

    xil_printf( "Initializing system...\r\n" );

    // initialize the Nexys4IO and Pmod544IO hardware and drivers
    status = NX4IO_initialize( N4IO_BASEADDR );
    if( status != XST_SUCCESS )
    {
        xil_printf( "Nexys4IO initialization failed, status: %d\r\n", status );
        return XST_FAILURE;
    }
    xil_printf( "Nexys4IO initialized successfully\r\n" );
    return status;
}

static int i2c_init( void )
{
    int Status;
    u32 control;
    int retries = 10;

    // Initialize the I2C controller
    Status      = XIic_Initialize( &I2C_Instance, IIC_DEVICE_ID );
    if( Status != XST_SUCCESS )
    {
        xil_printf( "I2C Initialization failed: %d\n", Status );
        return 1;
    }

    // Perform a soft reset
    i2c_soft_reset( &I2C_Instance );

    // Manually enable the I2C peripheral (GE)
    XIic_WriteReg(
        I2C_Instance.BaseAddress, 0x300, 0x1 ); // GE = 1 (Global Enable)
    usleep( 10000 );                            // 10ms delay

    // Retry enabling CR until IIC is enabled (relaxed check)
    while( retries > 0 )
    {
        // Set CR: Enable IIC, Master Mode, Enable Transmit/Receive FIFOs
        XIic_WriteReg( I2C_Instance.BaseAddress, 0x100, 0x8D ); // CR = 0x8D
        usleep( 1000 );                                         // 1ms delay

        // Read back CR to verify
        control = XIic_ReadReg( I2C_Instance.BaseAddress, 0x100 );
        if( control & 0x81 )
        { // Check only Bit 7 (Enable IIC) and Bit 0 (Master Mode)
            xil_printf( "I2C peripheral enabled (partial): CR = 0x%08X\n",
                        control );
            break;
        }
        xil_printf( "Retry %d: Failed to enable I2C peripheral: CR = 0x%08X\n",
                    retries,
                    control );
        retries--;
        usleep( 10000 ); // 10ms delay between retries
    }

    if( retries == 0 )
    {
        xil_printf( "Failed to enable I2C peripheral after retries\n" );
        return 1;
    }

    // Start the I2C controller
    Status = XIic_Start( &I2C_Instance );
    if( Status != XST_SUCCESS )
    {
        xil_printf( "I2C Start failed: %d\n", Status );
        return 1;
    }

    // Re-enable I2C peripheral after XIic_Start
    XIic_WriteReg(
        I2C_Instance.BaseAddress, 0x300, 0x1 ); // GE = 1 (Global Enable)
    XIic_WriteReg( I2C_Instance.BaseAddress, 0x100, 0x8D ); // CR = 0x8D
    usleep( 1000 );

    // Verify CR after re-enable
    control = XIic_ReadReg( I2C_Instance.BaseAddress, 0x100 );
    if( !( control & 0x81 ) )
    {
        xil_printf( "Failed to re-enable I2C peripheral after XIic_Start: CR = "
                    "0x%08X\n",
                    control );
        return 1;
    }
    xil_printf( "I2C peripheral re-enabled after XIic_Start: CR = 0x%08X\n",
                control );

    // Attempt to enable Transmit/Receive FIFOs
    control = XIic_ReadReg( I2C_Instance.BaseAddress, 0x100 );
    control |=
        0xC; // Set Bit 3 (Receive FIFO Enable) and Bit 2 (Transmit FIFO Enable)
    XIic_WriteReg( I2C_Instance.BaseAddress, 0x100, control );
    usleep( 1000 );

    // Verify FIFOs (relaxed check - proceed even if FIFOs fail to enable)
    control = XIic_ReadReg( I2C_Instance.BaseAddress, 0x100 );
    if( ( control & 0xC ) != 0xC )
    {
        xil_printf(
            "Warning: Transmit/Receive FIFOs not enabled: CR = 0x%08X\n",
            control );
        // Proceed anyway to test if I2C works
    }
    else
    {
        xil_printf( "Transmit/Receive FIFOs enabled: CR = 0x%08X\n", control );
    }

    // Read and print CR and SR for debugging
    i2c_read_control( &I2C_Instance );
    i2c_read_status( &I2C_Instance );

    return 0;
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
static void seven_seg_selfTest( void )
{
    xil_printf( "Starting Nexys4IO self test...\r\n" );

    xil_printf( "\tcheck functionality of 7-segment display\r\n" );
    // set the display digits to -ECE544- and turn off
    // the decimal points using the "raw" set functions.
    NX4IO_SSEG_setSSEG_DATA( SSEGHI, 0x0058E30E );
    NX4IO_SSEG_setSSEG_DATA( SSEGLO, 0x00144116 );
    usleep( 2000 * 1000 );

    xil_printf( "...Nexys4IO self test complete\r\n" );
    return;
}

static int prvSetupHardware( void )
{
    uint32_t xStatus;
    const unsigned char ucSetToInput = 0xFFU;

    xil_printf( "Initializing GPIO's\r\n" );

    /* Initialize the GPIO for the button inputs. */
    xStatus =
        XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
    if( xStatus != XST_SUCCESS )
    {
        xil_printf( "GPIO initialization failed\r\n" );
        configASSERT( 0 );
    }

    /* Install the handler defined in this task for the button input.
     * NOTE: The FreeRTOS defined xPortInstallInterruptHandler() API function
     * must be used for this purpose. */
    xStatus = xPortInstallInterruptHandler(
        XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR,
        gpio_intr,
        NULL );

    if( xStatus == pdPASS )
    {
        xil_printf( "Buttons interrupt handler installed\r\n" );

        /* Set switches and buttons to input. */
        XGpio_SetDataDirection(
            &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
        XGpio_SetDataDirection( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );

        /* Enable the button input interrupts in the interrupt controller.
         * NOTE: The vPortEnableInterrupt() API function must be used for this
         * purpose. */
        vPortEnableInterrupt(
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

        /* Enable GPIO channel interrupts on button channel. Can modify to
         * include switches */
        XGpio_InterruptEnable( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
        XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
    }
    else
    {
        xil_printf( "Failed to install GPIO interrupt handler\r\n" );
        configASSERT( 0 );
    }

    configASSERT( xStatus == pdPASS );
    // If it is pass return 0
    return ( xStatus == 1 ) ? 0 : 1;
}

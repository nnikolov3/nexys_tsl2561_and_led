/*
 * main.c - Entry point for TSL2561-based system with button handling, PID
 * control, and sensor reading
 *
 * Purpose: Initializes FreeRTOS, sets up GPIO with interrupts for buttons,
 * initializes the TSL2561 sensor via I2C (polling), and creates tasks for
 * button parsing, PID control, and display updates. Designed for Nexys A7 with
 * Microblaze, AXI, and TSL2561 sensor. Combines LED toggling with PID-driven
 * RGB LED control.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* Include necessary headers */
#include "main.h"

/* Global variables */
XGpio             xInputGPIOInstance; // GPIO instance for buttons and switches
SemaphoreHandle_t binary_sem = NULL;  // Semaphore for GPIO interrupts
QueueHandle_t     toPID =
    NULL; // Queue for sending button/switch states to PID task
QueueHandle_t fromPID = NULL; // Queue for sending setpoint/lux to display task
XIntc         Intc;           // Shared interrupt controller

/* Function prototypes */
static void gpio_intr ( void* pvUnused ); // GPIO interrupt handler
void        Parse_Input_Task ( void* p ); // Task to parse button/switch inputs
void        PID_Task ( void* p );         // Task for PID control of RGB LED
void        Display_Task ( void* p );     // Task to update 7-segment display
static int  do_init ( void );             // System initialization
static int  nexys_init ( void );          // Nexys A7 peripheral initialization
static int  prvSetupHardware ( void );    // GPIO hardware setup with interrupts
int         pid_init ( PID_t* pid );      // PID structure initialization
float       pid_funct ( PID_t*  pid,
                        float   lux_value,
                        uint8_t switches ); // PID algorithm

/**
 * Main entry point for the TSL2561 system.
 * Initializes hardware, sets up interrupts, creates FreeRTOS constructs, and
 * starts the scheduler.
 */
int main ( void )
{
    xil_printf ( "Starting TSL2561 System\r\n" );

    /* Perform system initialization (no interrupt setup here) */
    if ( do_init ( ) != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Hardware init failed\r\n" );
        while ( 1 )
            ; // Halt to debug
    }

    /* Enable global interrupts */
    Xil_ExceptionInit ( );
    Xil_ExceptionRegisterHandler (
        XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler) XIntc_InterruptHandler,
        &Intc );
    Xil_ExceptionEnable ( );

    /* Create semaphore for button interrupts */
    vSemaphoreCreateBinary ( binary_sem );
    if ( binary_sem == NULL )
    {
        xil_printf ( "[ERROR] Semaphore creation failed\r\n" );
        return -1;
    }

    /* Create queues for button-to-PID and PID-to-display communication */
    toPID   = xQueueCreate ( MAIN_QUEUE_LENGTH, sizeof ( uint16_t ) );
    fromPID = xQueueCreate ( MAIN_QUEUE_LENGTH, sizeof ( uint32_t ) );
    if ( toPID == NULL || fromPID == NULL )
    {
        xil_printf ( "[ERROR] Queue creation failed\r\n" );
        return -1;
    }

    /* Create PID structure for LED control */
    PID_t ledPID;

    /* Create tasks */
    xTaskCreate ( Parse_Input_Task,
                  "Parse_Input",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  tskIDLE_PRIORITY + 1,
                  NULL );
    xTaskCreate ( PID_Task,
                  "PID",
                  configMINIMAL_STACK_SIZE,
                  &ledPID,
                  tskIDLE_PRIORITY + 2,
                  NULL );
    xTaskCreate ( Display_Task,
                  "Disp",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  tskIDLE_PRIORITY + 3,
                  NULL );

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler ( );

    xil_printf ( "[ERROR] Scheduler failed\r\n" );
    return -1; // Should never reach here
}

/**
 * GPIO interrupt handler for button presses.
 * Signals the semaphore to wake the Parse Input task and clears the interrupt.
 *
 * @param pvUnused Unused parameter (required by ISR signature)
 */
static void gpio_intr ( void* pvUnused )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Signal the semaphore to notify the Parse Input task */
    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );

    /* Yield to higher-priority tasks if necessary */
    portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
}

/**
 * Task to parse button presses and switch states.
 * Waits for the semaphore, reads GPIO inputs, and sends them to the PID task
 * via queue.
 *
 * @param p Unused parameter
 */
void Parse_Input_Task ( void* p )
{
    uint16_t ValueToSend = 0x0000;

    while ( 1 )
    {
        /* Wait for button interrupt via semaphore */
        if ( xSemaphoreTake ( binary_sem, pdMS_TO_TICKS ( 500 ) ) == pdTRUE )
        {
            uint8_t btns = ( NX4IO_getBtns ( ) & 0x0C ) >>
                           2; // Get BTNU/BTND, right justify
            uint8_t sws = (uint8_t) ( NX4IO_getSwitches ( ) &
                                      0x00FF );    // Get lower 8 switches
            ValueToSend = ( ( btns << 8 ) | sws ); // Combine: btns in upper 8
                                                   // bits, sws in lower
            xQueueSend ( toPID, &ValueToSend, MAIN_DONT_BLOCK );
            ValueToSend = 0x0000; // Clear for next iteration
        }
        vTaskDelay ( pdMS_TO_TICKS ( 100 ) ); // Debounce delay
    }
}

/**
 * Task for PID control of RGB LED based on TSL2561 readings.
 * Adjusts LED brightness using PID algorithm and updates display task.
 *
 * @param p Pointer to PID_t structure
 */
void PID_Task ( void* p )
{
    PID_t*  pid     = (PID_t*) p; // Get the PID struct passed from main
    float   pidOUT  = 0; // Float percent value returned from PID algorithm
    float   tsl2561 = 0; // Float value returned from TSL2561 sensor
    uint8_t pwmLED =
        127;           // 8-bit int value for writing to PWM LED (initial 50%)
    uint16_t   btnSws; // Value received from input task queue
    uint32_t   setpntLux;    // Value to send to display task queue
    uint8_t    btns;         // Button values parsed from btnSws
    uint8_t    sws;          // Switch values parsed from btnSws
    float      baseID = 0.1; // Base increment for integral/derivative tuning
    uint8_t    baseSP = 1;   // Base increment for setpoint and Kp
    uint8_t    incScaling;   // Scaling factor based on switch values
    TickType_t lastTick = 0; // Used for accurate delta_t calculation
    static int isInitialized =
        0; // Changed to int for consistency with pid_init return

    /* Initialize PID structure if not already done */
    if ( !isInitialized )
    {
        isInitialized = pid_init ( pid );
    }

    while ( 1 )
    {
        /* Receive button/switch state from input task */
        xQueueReceive ( toPID, &btnSws, portMAX_DELAY );
        btns = ( btnSws & 0x0300 ) >> 8; // Extract buttons (BTNU/D)
        sws  = ( btnSws & 0x00FF );      // Extract switches

        /* Scale increments based on switches[5:4] */
        if ( !( sws & 0x30 ) )
        { // [0:0]
            incScaling = 1;
        }
        else if ( sws & 0x20 )
        { // [1:X]
            incScaling = 10;
        }
        else if ( sws & 0x10 )
        { // [0:1]
            incScaling = 5;
        }

        /* Adjust PID parameters or setpoint based on switch[3] and
         * buttons */
        if ( sws & 0x08 )
        { // Switch[3] on: Adjust setpoint
            if ( btns == 1 )
            { // BTNU
                pid->setpoint += ( incScaling * baseSP );
            }
            else if ( btns == 2 )
            { // BTND
                pid->setpoint -= ( incScaling * baseSP );
            }
        }
        else
        { // Switch[3] off: Adjust gains based on switches[7:6]
            switch ( sws & 0xC0 )
            {
            case 0x40: // Switch[6]: Kp
                if ( btns == 2 )
                    pid->Kp += ( incScaling * baseSP );
                if ( btns == 1 )
                    pid->Kp -= ( incScaling * baseSP );
                break;
            case 0x80: // Switch[7]: Ki
                if ( btns == 2 )
                    pid->Ki += ( incScaling * baseID );
                if ( btns == 1 )
                    pid->Ki -= ( incScaling * baseID );
                break;
            case 0xC0: // Both: Kd
                if ( btns == 2 )
                    pid->Kd += ( incScaling * baseID );
                if ( btns == 1 )
                    pid->Kd -= ( incScaling * baseID );
                break;
            default:
                break;
            }
        }

        /* Get TSL2561 reading */
        uint16_t ch0 = tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_0 );
        uint16_t ch1 =
            tsl2561_readChannel ( &IicInstance, TSL2561_CHANNEL_1 ); // Now used
        tsl2561 =
            (float) ch0 -
            ( (float) ch1 * 0.5 ); // Basic lux approximation using CH0 and CH1

        /* Calculate delta_t for PID */
        TickType_t currentTick = xTaskGetTickCount ( );
        pid->delta_t =
            ( ( currentTick - lastTick ) * portTICK_PERIOD_MS ) / 1000.0f;
        lastTick = currentTick;

        /* Run PID algorithm */
        pidOUT = pid_funct ( pid, tsl2561, sws );

        /* Adjust PWM LED (RGB1 Blue) and regular LEDs */
        if ( pidOUT >= 0 )
        {
            if ( ( pwmLED + (uint8_t) ( pidOUT * MAX_DUTY ) ) >= MAX_DUTY )
            {
                pwmLED = MAX_DUTY;
            }
            else
            {
                pwmLED += (uint8_t) ( pidOUT * MAX_DUTY );
            }
        }
        else
        {
            if ( ( pwmLED - (uint8_t) ( -pidOUT * MAX_DUTY ) ) <= MIN_DUTY )
            {
                pwmLED = MIN_DUTY;
            }
            else
            {
                pwmLED -= (uint8_t) ( -pidOUT * MAX_DUTY );
            }
        }
        NX4IO_RGBLED_setDutyCycle ( RGB1, 0, 0, pwmLED );
        NX4IO_setLEDs (
            btnSws ); // Update regular LEDs with button/switch state

        /* Update setpoint/lux for display */
        setpntLux = ( (uint32_t) ( pid->setpoint ) & LUX_MASK ) << 16 |
                    ( (uint32_t) tsl2561 & LUX_MASK );
        xQueueSend ( fromPID, &setpntLux, MAIN_DONT_BLOCK );
    }
}

/**
 * Task to update 7-segment display with setpoint and lux values.
 *
 * @param p Unused parameter
 */
void Display_Task ( void* p )
{
    uint32_t receivedLux;
    uint16_t setpnt, luxVal;

    while ( 1 )
    {
        /* Receive setpoint and lux from PID task */
        xQueueReceive ( fromPID, &receivedLux, portMAX_DELAY );
        setpnt = ( receivedLux >> 16 ) & LUX_MASK;
        luxVal = receivedLux & LUX_MASK;

        /* Update 7-segment display */
        NX410_SSEG_setAllDigits ( SSEGHI,
                                  (uint8_t) ( setpnt / 100 ),
                                  (uint8_t) ( ( setpnt % 100 ) / 10 ),
                                  (uint8_t) ( ( setpnt % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
        NX410_SSEG_setAllDigits ( SSEGLO,
                                  (uint8_t) ( luxVal / 100 ),
                                  (uint8_t) ( ( luxVal % 100 ) / 10 ),
                                  (uint8_t) ( ( luxVal % 100 ) % 10 ),
                                  CC_BLANK,
                                  DP_NONE );
    }
}

/**
 * System initialization function.
 * Initializes hardware components (GPIO, Nexys, I2C, TSL2561) without interrupt
 * setup.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
static int do_init ( void )
{
    int status;

    /* Initialize Nexys A7 peripherals */
    status = nexys_init ( );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Nexys init failed\r\n" );
        return XST_FAILURE;
    }

    /* Initialize I2C (polling mode, from i2c.c) */
    status = i2c_init ( );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] I2C init failed\r\n" );
        return XST_FAILURE;
    }

    /* Perform I2C self-test */
    status = XIic_SelfTest ( &IicInstance );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] I2C self-test failed\r\n" );
        return XST_FAILURE;
    }

    /* Scan I2C bus */
    i2c_scan ( &IicInstance );

    /* Initialize TSL2561 sensor */
    status = tsl2561_init ( &IicInstance );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] TSL2561 init failed\r\n" );
        return XST_FAILURE;
    }

    /* Initialize GPIO for buttons with interrupts */
    status = prvSetupHardware ( );
    if ( status != 0 )
    {
        xil_printf ( "[ERROR] GPIO init failed\r\n" );
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/**
 * Nexys A7 peripheral initialization.
 * Sets up the Nexys4IO peripheral for LEDs and 7-segment display.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
static int nexys_init ( void )
{
    int status;

    status = NX4IO_initialize ( N4IO_BASEADDR );
    if ( status != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] Nexys4IO init failed\r\n" );
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

/**
 * GPIO hardware setup.
 * Configures the GPIO for button inputs and installs the interrupt handler
 * using FreeRTOS APIs.
 *
 * @return 0 if setup succeeds, 1 otherwise
 */
static int prvSetupHardware ( void )
{
    uint32_t            xStatus;
    const unsigned char ucSetToInput = 0xFFU;

    xStatus =
        XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
    if ( xStatus != XST_SUCCESS )
    {
        xil_printf ( "[ERROR] GPIO init failed\r\n" );
        configASSERT ( 0 );
    }

    xStatus = xPortInstallInterruptHandler (
        XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR,
        gpio_intr,
        NULL );

    if ( xStatus == pdPASS )
    {
        XGpio_SetDataDirection (
            &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
        XGpio_SetDataDirection (
            &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );
        vPortEnableInterrupt (
            XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );
        XGpio_InterruptEnable ( &xInputGPIOInstance, XGPIO_IR_CH1_MASK );
        XGpio_InterruptGlobalEnable ( &xInputGPIOInstance );
    }
    else
    {
        xil_printf ( "[ERROR] Interrupt handler install failed\r\n" );
        configASSERT ( 0 );
    }

    configASSERT ( xStatus == pdPASS );
    return ( xStatus == pdPASS ) ? 0 : 1;
}

/**
 * PID structure initialization.
 * Sets initial values for PID parameters.
 *
 * @param pid Pointer to PID_t structure
 * @return 1 on success (for consistency with XST_SUCCESS-like behavior)
 */
int pid_init ( PID_t* pid )
{
    pid->Kp         = 0;
    pid->Ki         = 0;
    pid->Kd         = 0;
    pid->setpoint   = 250; // Initial setpoint (adjustable via buttons)
    pid->integral   = 0;
    pid->prev_error = 0;
    pid->delta_t = 0.437; // Initial delta_t (437ms, worst-case sampling time)
    pid->max_lim = 65536; // Max lux value (16-bit)
    pid->min_lim = 0;     // Min lux value

    return 1; // Return 1 to match XST_SUCCESS-like behavior
}

/**
 * PID algorithm implementation.
 * Calculates PID output based on lux value and switch settings.
 *
 * @param pid Pointer to PID_t structure
 * @param lux_value Current lux reading from TSL2561
 * @param switches Current switch states
 * @return Float percentage value for PWM adjustment
 */
float pid_funct ( PID_t* pid, float lux_value, uint8_t switches )
{
    float error = pid->setpoint - lux_value;

    /* Proportional term */
    float Pterm = ( switches & 0x01 ) ? pid->Kp * error : 0;

    /* Integral term */
    pid->integral += ( error * pid->delta_t );
    float Iterm = ( switches & 0x02 ) ? pid->Ki * pid->integral : 0;

    /* Derivative term */
    float Dterm = ( switches & 0x04 )
                      ? pid->Kd * ( ( error - pid->prev_error ) / pid->delta_t )
                      : 0;

    pid->prev_error = error;

    return ( Pterm + Iterm + Dterm ) / pid->setpoint;
}

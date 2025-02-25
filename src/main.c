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

/********** Duty Cycle Related Constants **********/
#define max_duty 255 // max possible duty cycle for RGB1 Blue
#define min_duty 0   // min possible duty cycle for RGB1 Blue

int main ( void )
{
    // Announcement
    xil_printf ( "Hello from FreeRTOS Example\r\n" );

    // Initialize the HW
    prvSetupHardware ( );
    do_init ( );
    nexys4io_selfTest ( );
    tsl2561_init ( &I2C_Instance );

    // Create Semaphore
    vSemaphoreCreateBinary ( binary_sem );

    /* Create the queue */
    xQueue = xQueueCreate ( mainQUEUE_LENGTH, sizeof ( uint16_t ) );

    /* Sanity check that the queue was created. */
    configASSERT ( xQueue );

    /*Creat pid structure for keeping track of PID elements*/
    PID_t ledPID;

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

/****************************************************************************
 * initialize the system
 *
 * This function is executed once at start-up and after resets.  It initializes
 * the peripherals and registers the interrupt handler(s)
 *****************************************************************************/
int do_init ( void )
{
    int status; // status from Xilinx Lib calls

    // initialize the Nexys4IO and Pmod544IO hardware and drivers
    // rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE
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
    // Enable the I2C Controller
    return XST_SUCCESS;
}

/****************************************************************************/
/**
 * nexys4io_selfTest() - performs a self test on the NexysA7 peripheral
 *
 * @brief This is mostly a visual test to confirm that the 7-segment display and
 * RGB LEDs hardware and drivers are operating correctly.  The test does the
 *following: o sends pattern(s) to the LEDs on the board o Writes a message on
 *the 7-segment display o individually lights the RGB LEDs o sets the RGB2 LED
 *to several values that can be observed o Turns off the LEDs and blanks the
 *7-segment digits and decimal points
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

/*********************PID Task Prototype*************************************
*   Task Handles the Following:
*   Reads perameter message from MsgQ
*   Update new control/setpoint parameters
*   Get Current lux readig from TSL2561 sensor
*       done using the TSL2561 driver in implemented
*       in the C file of the same name
*   Execute PID algo function 
*   Drive PWM signal for LED, use RGB writ commands
*   write to display thread MsgQ to update
*   setpoint and current lux
*****************************************************************************/
void PID_Task (PID_t* pid, void* p)
{
    float pidOUT = 0;       // float percent value returned from PID algo
    float tsl2561 = 0;      // float value returned from tsl2561 driver 
    uint8_t pwmLED = 127;   // 8-bit int value for writing to PWM LED

    static bool isInitialized = false;	// true if the init function has run at least once
    // initialize the pid struct if it hasn't been
    if(!isInitialized)
    {
        isInitialized = pid_init(&pid);
    }

    // main task loop
    while(1)

    // TODO: add MsgQ deQ functionality and update pid values with it

    // TODO: get(sample) TSL2561 reading

    // running PID algorithm, uses float from TSL2561 driver
    // returns the percentage to increase/decrease the pwmLED by
    pidOUT = pid_funct(&pid, tsl2561);

    // use the PID output to adjust the duty cycle and write it to the PWM LED
    // if the percentage returned is positive, the intensity needs to be increased
    // because the sensor is reading a value lower than the setpoint
    // if it's negative the intensity needs to be decreased becasue the 
    // sensor is reading a value hgiher than the setpoint
    if (pidOUT >= 0)
    {
        // clamp the output to the max value if pidOUT >= 1
        // prevent over driving LED or wrap around
        if ((pwmLED += (uint8_t)((pidOUT) * max_duty)) >= max_duty)
        {
            pwmLED = max_duty;
        }
        else
        {
            pwmLED += (uint8_t)((pidOUT) * max_duty);
        }
    }
    else
    {
        // clamp the output to the min value if pidOUT <= 0
        // prevent over driving LED or wrap around
        if ((pwmLED -= (uint8_t)((pidOUT) * max_duty)) <= min_duty)
        {
            pwmLED = min_duty;
        }
        else
        {
            pwmLED -= (uint8_t)((pidOUT) * max_duty);
        }
    }
    // write duty cycle to RGB1 blue, drives pwmLED
    NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwmLED)

    // TODO: send message to display thread MsgQ to update setpoint and current lux
}

/*********************PID Initialization*************************************
*   Initializing PID structure for use in the PI Task
*****************************************************************************/
bool pid_init (PID_t *pid)
{
    pid -> Kp = 1;
    pid -> Ki = 0;
    pid -> Kd = 0;
    pid -> setpoint = ;
    pid -> integral = 0;
    pid -> prev_error = 0;
    pid -> delta_t = 0.437; // set to the worst case sampling time but will dynamically update in use
    // assumes that the float value returned is a 16 bit value, would need be adjusted 
    pid -> max_lim = 65536; // set max value to 99%, should result in setting LED PWM to 99%
    pid -> min_lim = 0; // set min value to 1%, should result in setting LED PWM to 1%

    // returns true after initializing PID structure 
    return true;
}

/*********************PID Algo*************************************
*   Take PID structure and lux value from TSL2561
*   returns float percentage value, used for writing to LED
*   PID output = P + I + D
*   P = Kp(e(t)), where Kp is proportional gain
*   I = Ki(integral(e(t)dt)) where Ki is the integral gain
*   D = Kd((de(t))/dt) where Kd is the derivative gain
*   e(t) is the error and is equal to the setpoint - measured value
*   the integral can be though of as the accumulation of e(t)'s
*   the derivative can be though of as the (delta e(t))/(delta_t)
*   delta_t is the time between samples
*****************************************************************************/
float pid_funct (PID_t* pid, float lux_value)
{
    // e(t), error at time of sample
    float error = pid->setpoint - lux_value;

    // proportional term
    float Pterm = pid->Kp * error;
    
    // get integral term and update integral
    pid -> integral += (error * pid->delta_t);
    float Iterm = pid->Ki * pid->integral;

    // get derivative out term
    float Dterm = pid->Kd * ((error - pid->prev_error) / pid->delta_t);

    // update previous error value
    pid->prev_error = error;

    // return a percentage value to be used for setting the intensity of PWM LED
    return (Pterm + Iterm + Dterm) / pid->setpoint;
}
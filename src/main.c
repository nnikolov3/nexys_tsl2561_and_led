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

#define lux_mask 0xFFFF // mask used for combining setpoint and lux values into one double

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
    toPID = xQueueCreate (mainQUEUE_LENGTH, sizeof (uint16_t));
    fromPID = xQueueCreate (mainQUEUE_LENGTH, sizeof (uint32_t));

    /* Sanity check that the queue was created. */
    configASSERT (toPID);
    configASSERT (fromPID);

    /*Creat pid structure for keeping track of PID elements*/
    PID_t ledPID;

    // Create Task1
    xTaskCreate ( Parse_Input_Task,
                  (const char*) "Parse_Input",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  1,
                  NULL );

    // Create Task2
    xTaskCreate ( PID_Task, "PID", configMINIMAL_STACK_SIZE, &ledPID, 2, NULL );

    // Create Task3
    xTaskCreate ( Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, 3, NULL );

    // Start the Scheduler
    xil_printf ( "Starting the scheduler\r\n" );
    //xil_printf ( "Push Button to change the LED pattern\r\n\r\n" );
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

/****************************************************************************
 * nexys4io_selfTest() - performs a self test on the NexysA7 peripheral
 *
 * @brief This is mostly a visual test to confirm that the 7-segment display and
 * RGB LEDs hardware and drivers are operating correctly.  The test does the
 *following: o sends pattern(s) to the LEDs on the board o Writes a message on
 *the 7-segment display o individually lights the RGB LEDs o sets the RGB2 LED
 *to several values that can be observed o Turns off the LEDs and blanks the
 *7-segment digits and decimal points
 *****************************************************************************/
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

/**************************PID Task******************************************
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
void PID_Task (void* p)
{
    PID_t* pid = (PID_t*)p; // get the PID struct passed from main
    float pidOUT = 0;       // float percent value returned from PID algo
    float tsl2561 = 0;      // float value returned from tsl2561 driver 
    uint8_t pwmLED = 127;   // 8-bit int value for writing to PWM LED
    uint16_t btnSws;        // value recieved from the input task Q
    uint32_t setpntLux;     // value to send to the display Task Q
    uint8_t btns;           // btn values parsed from btnSws
    uint8_t sws;            // switch values parsed from btnSws
    float baseID = 0.1;     // base increment for ID tuning set to 0.1
    uint8_t baseSP = 1;     // base increment for setpoint and Kp to 1
    uint8_t incScaling;     // scaling factor based on switch values
    TickType_t lastTick = 0 // used for more accurate delat t values

    static bool isInitialized = false;	// true if the init function has run at least once
    // initialize the pid struct if it hasn't been
    if(!isInitialized)
    {
        isInitialized = pid_init(&pid);
    }

    // main task loop
    while(1)

    // recieve message from input task, 16-bit uint that contains switch and button values
    xQueueReceive (toPID, &btnSws, portMAX_DELAY);
    
    // parse values recieved from input task
    btns = (btnSws & 0x0300) >> 8;
    sws = (btnSws & 0x0FF);

    // scale base increments based on values of switches[5:4]
    if (!(sws & 0x30))  //switches [5:4] = [0:0]
    {
        incScaling = 1;
    }
    else if (sws & 0x20)  //switches [5:4] = [1:X]
    {
        incScaling = 10;
    }
    else if (sws & 0x10)  //switches [5:4] = [1:0]
    {
        incScaling = 5;
    }
    
    /* PID modifier 
    *   if switch[3] only increment or decrement setpoint, based on button press
    *   else check switches[7:6] 
    *   Just switch [6] = 0x40, Kp should changed on button press
    *   Just switch [7] = 0x80, Ki should be changed on button press
    *   Both switches [7:6] = 0XC0, Kd should be changed on button press
    *   for buttons, a value of 1 means up button was pressed
    *   a value of 2 means down button was pressed
    *   After parsing, updates the PID struct with the setpoint or PID gain values 
    */
    if((sws & 0x08))
    {
        if ((btns & 0x03) == 1)
        {
            pid->setpoint += (incScaling * baseSP);
        }
        if ((btns & 0x03) == 2)
        {
            pid->setpoint -= (incScaling * baseSP);
        }
    }
    else
    {
        switch (sws & 0xC0)
        {
        case 0x40:
            if ((btns & 0x03) == 2)
            {
                pid->Kp += (incScaling * baseSP);
            }
            if ((btns & 0x03) == 1)
            {
                pid->Kp -= (incScaling * baseSP);
            }
            break;
        
        case 0x80:
            if ((btns & 0x03) == 2)
            {
                pid->Ki += (incScaling * baseID);
            }
            if ((btns & 0x03) == 1)
            {
                pid->Ki -= (incScaling * baseID);
            }
            break;
        
        case 0xC0:
            if ((btns & 0x03) == 2)
            {
                pid->Kd += (incScaling * baseID);
            }
            if ((btns & 0x03) == 1)
            {
                pid->Kd -= (incScaling * baseID);
            }
            break;
        
         default:
            break;
        }
    }

    // TODO: get(sample) TSL2561 reading

    // this may need to be moved into the TSL2561 module to result in a more accurate reading
    // get tick when sample is collected and use to determin dt
    TickType_t currentTick = xTaskGetTickCount();
    pid->delta_t = ((currentTick - lastTick) * portTICK_PERIOD_MS) / 1000.0f;
    // update last tick time for use in next dt calculation
    lastTick = currentTick;


    // running PID algorithm, uses float from TSL2561 driver
    // returns the percentage to increase/decrease the pwmLED by
    pidOUT = pid_funct(&pid, tsl2561, sws);

    /*PWM LED Write
    *   use the PID output to adjust the duty cycle and write it to the PWM LED
    *   if the percentage returned is positive, the intensity needs to be increased
    *   because the sensor is reading a value lower than the setpoint
    *   if it's negative the intensity needs to be decreased becasue the 
    *   sensor is reading a value hgiher than the setpoint
    *   End by writing duty cycle to RGB1 blue, drives pwmLED
    *   using NX4IO_RGBLED_setDutyCycle command
    */
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
    NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwmLED);

    // update setpntLux with value to be displayed by display task
    setpntLux &= 0x00000000; // make sure old data is cleared 
    setpntLux |= (uint32_t)(((tsl2561 & lux_mask) << 0) | // sensor measured value in lower 32 bits
                 ((pid->setpoint & lux_mask) << 16)); // setpoint in upper 32 bits

    //send message to display thread MsgQ to update setpoint and current lux
    xQueueSend (fromPID, &setpntLux ,mainDONT_BLOCK);
}

/************************Display Task****************************************
*   Gets lux and setpoint values from Q and updates 7-seg display
*****************************************************************************/
void Display_Task (void* p)
{
    uint32_t recievedLux;
    uint16_t setpnt, luxVal;
    while (1)
    {
        // recieve new sensor lux reading and setpoint values
        xQueueReceive (fromPID, &recievedLux, portMAX_DELAY);

        // make sure old values are cleared
        setpnt &= 0x0000;
        luxVal &= 0x0000;

        // parse values
        luxVal |= (recievedLux & lux_mask);
        setpnt |= ((recievedLux >> 16) & lux_mask);

        // write values to 7-seg display
        NX410_SSEG_setAllDigits(SSEGHI, (uint8_t)(setpnt/100),
        (uint8_t)((setpnt%100)/10), (uint8_t)((setpnt%100)%10), 
        CC_BLANK, DP_NONE);
        NX410_SSEG_setAllDigits(SSEGLO, (uint8_t)(luxVal/100),
        (uint8_t)((luxVal%100)/10), (uint8_t)((luxVal%100)%10),
        CC_BLANK, DP_NONE);
    }
}

/*********************PID Initialization*************************************
*   Initializing PID structure for use in the PI Task
*****************************************************************************/
bool pid_init (PID_t *pid)
{
    pid -> Kp = 0;
    pid -> Ki = 0;
    pid -> Kd = 0;
    pid -> setpoint = 250;
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
float pid_funct (PID_t* pid, float lux_value, uint8_t switches)
{
    // e(t), error at time of sample
    float error = pid->setpoint - lux_value;

    // proportional
    float Pterm;
    // set to zero if switch[0] is O, affectively disabling proportional control
    if (switches & 0x01)
    {
        Pterm = pid->Kp * error;
    }
    else
    {
        Pterm = 0;
    } 
    
    // update integral, and get integral term
    pid -> integral += (error * pid->delta_t);
    float Iterm;
    // set to zero if switch[1] is 0
    if (switches & 0x02)
    {
        Iterm = pid->Ki * pid->integral;
    }
    else
    {
        Iterm = 0;
    }    

    // get derivative term
    float Dterm;
    // set to zero if switch[2] is 0
    if (switches & 0x04)
    {
        Dterm = pid->Kd * ((error - pid->prev_error) / pid->delta_t);
    }
    else 
    {
        Dterm = 0;
    }

    // update previous error value
    pid->prev_error = error;

    // return a percentage value to be used for setting the intensity of PWM LED
    return (Pterm + Iterm + Dterm) / pid->setpoint;
}
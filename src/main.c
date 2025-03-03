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

int main ( void )
{
    // Announcement
    xil_printf ( "Hello from FreeRTOS LUX PID Controller\r\n" );

    // Initialize the HW
    prvSetupHardware ( );
    do_init ( );
    tsl2561_init ( &IicInstance );

    // Create Semaphore
    vSemaphoreCreateBinary ( binary_sem );

    /* Create the queue */
    toPID = xQueueCreate (mainQUEUE_LENGTH, sizeof (uint16_t));
    fromPID = xQueueCreate (mainQUEUE_LENGTH, sizeof (uint32_t));

    /* Sanity check that the queue was created. */
    configASSERT (toPID);
    configASSERT (fromPID);

    /*Creat pid structure for keeping track of PID elements*/
    static PID_t ledPID;

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

    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );

    /* Signal the semaphore to notify the Parse Input task */
    xSemaphoreGiveFromISR ( binary_sem, &xHigherPriorityTaskWoken );
    XGpio_InterruptClear ( &xInputGPIOInstance, XGPIO_IR_MASK );

    /* Yield to higher-priority tasks if necessary */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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
            btns = (NX4IO_getBtns() & 0x1C); // get btns and mask for u/d/c
            sws = (uint8_t)(NX4IO_getSwitches() & 0x00FF); // get lower 8 switches
            ValueToSend |= ((btns << 8) | (sws)); // move btnc to bit 10, btnu to bit 9, and bntd to bit 8
            NX4IO_setLEDs(sws);
            xQueueSend ( toPID, &ValueToSend, mainDONT_BLOCK );
            ValueToSend &= 0x0000 ; // clear btn/sw values for next time
        }
        else; // do nothing if semaphore isn't given
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

    // Get AXI I2C device configuration
    XIic_Config* ConfigPtr = XIic_LookupConfig ( I2C_DEV_ID_ADDR );
    if ( ConfigPtr == NULL )
    {
        return XST_FAILURE;
    }
    // Initialize the I2C driver
    status =
        XIic_CfgInitialize ( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
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

    XIic_Start ( &IicInstance );
    // Enable the I2C Controller

    NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);
    return XST_SUCCESS;
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
    uint16_t tsl2561 = 0;      // float value returned from tsl2561 driver
    uint8_t pwmLED = 127;   // 8-bit int value for writing to PWM LED
    uint16_t btnSws;        // value recieved from the input task Q
    uint32_t setpntLux;     // value to send to the display Task Q
    uint8_t btns;           // btn values parsed from btnSws
    uint8_t sws;            // switch values parsed from btnSws
    float baseID = 0.1;     // base increment for ID tuning set to 0.1
    uint8_t baseSP = 1;     // base increment for setpoint and Kp to 1
    uint8_t incScaling;     // scaling factor based on switch values
    TickType_t lastTick = 0; // used for more accurate delat t values

    xStatus =
        XGpio_Initialize ( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
    if ( xStatus != XST_SUCCESS )
    {
        isInitialized = pid_init(pid);
    }

    // main task loop
    while(1)
    {
    	// recieve message from input task, 16-bit uint that contains switch and button values
    	    if (xQueueReceive (toPID, &btnSws, 0) == pdPASS)
    	    {
    	    	// parse values recieved from input task
    	    	btns = (btnSws & 0x1C00) >> 10;
    	    	sws = (btnSws & 0x0FF);
    	    }
    	    else
    	    {
    	    	btns = 0x00;
    	    	sws = sws;
    	    	vTaskDelay(2);
    	    }



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
    	    *   has controls to prevent setpoint from being set higher or lower than the upper and lower limits
    	    *   Also prevents Kp, Ki, and Kd gains from going negative when decrementing
    	    *   uses macro(UPDATE_SATURATING) for saturation checks, help with readability and code redundancy
    	    */
    	    if((sws & 0x08))
    	    {
    	        if (btns & 0x02)
    	        {
    	        	UPDATE_SATURATING(pid->setpoint, (incScaling * baseSP), pid->min_lim, pid->max_lim, true);
    	        }
    	        else if (btns & 0x01)
    	        {
    	        	UPDATE_SATURATING(pid->setpoint, (incScaling * baseSP), pid->min_lim, pid->max_lim, false);
    	        }
    	        else
    	        {
    	        	pid->setpoint = pid->setpoint;
    	        }

    	    }
    	    else
    	    {
    	        switch (sws & 0xC0)
    	        {
    	        case 0x40:
    	            if ((btns & 0x02))
    	            {
    	            	UPDATE_SATURATING(pid->Kp, (incScaling * baseSP), 0, pid->max_lim, true);
    	            }
    	            else if ((btns & 0x01))
    	            {
    	            	UPDATE_SATURATING(pid->Kp, (incScaling * baseSP), 0, pid->max_lim, false);
    	            }
    	            else
    	            {
    	            	pid->Kp = pid->Kp;
    	            }
    	            break;

    	        case 0x80:
    	            if ((btns & 0x02))
    	            {
    	            	UPDATE_SATURATING(pid->Ki, (incScaling * baseID), 0, pid->max_lim, true);
    	            }
    	            else if ((btns & 0x01))
    	            {
    	            	UPDATE_SATURATING(pid->Ki, (incScaling * baseID), 0, pid->max_lim, false);
    	            }
    	            else
    	            {
    	            	pid->Ki = pid->Ki;
    	            }
    	            break;

    	        case 0xC0:
    	            if ((btns & 0x02))
    	            {
    	            	UPDATE_SATURATING(pid->Kd, (incScaling * baseID), 0, pid->max_lim, true);
    	            }
    	            else if ((btns & 0x01))
    	            {
    	            	UPDATE_SATURATING(pid->Kd, (incScaling * baseID), 0, pid->max_lim, false);
    	            }
    	            else
    	            {
    	            	pid->Kd = pid->Kd;
    	            }
    	            break;

    	         default:
    	            break;
    	        }
    	    }

    	    // Get TSL2561 reading
    	    float ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0); // visible and infrared
    	    float ch1 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1 ); // just infrared
    	    tsl2561 = (uint16_t)ch0 - ((uint16_t)ch1 * 0.5); // basic lux approx.

    	    // this may need to be moved into the TSL2561 module to result in a more accurate reading
    	    // get tick when sample is collected and use to determin dt
    	    TickType_t currentTick = xTaskGetTickCount();
    	    pid->delta_t = ((currentTick - lastTick) * portTICK_PERIOD_MS) / 1000.0f;
    	    // update last tick time for use in next dt calculation
    	    lastTick = currentTick;


    	    // running PID algorithm, uses float from TSL2561 driver
    	    // returns the percentage to increase/decrease the pwmLED by
    	    pidOUT = pid_funct(pid, tsl2561, sws);

    	    /*PWM LED Write
    	    *   use the PID output to adjust the duty cycle and write it to the PWM LED
    	    *   if the percentage returned is positive, the intensity needs to be increased
    	    *   because the sensor is reading a value lower than the setpoint
    	    *   if it's negative the intensity needs to be decreased becasue the
    	    *   sensor is reading a value hgiher than the setpoint
    	    *   End by writing duty cycle to RGB1 blue, drives pwmLED
    	    *   using NX4IO_RGBLED_setDutyCycle command
    	    */

    	    // clamp the output to the max value if pidOUT >= 1
    	    // clamp the output to the min value if the correction pulls output <= min value
    	    // otherwise adjusts pwmLED by the required percentage
    	    if ((pwmLED + (pidOUT * max_duty)) >= max_duty)
    	    {
    	    	pwmLED = max_duty;
    	    }
    	    else if ((pwmLED + (pidOUT * max_duty)) <= min_duty)
    	    {
    	    	pwmLED = min_duty;
    	    }
    	    else
    	    {
    	    	pwmLED = (uint8_t)(pwmLED + (pidOUT * max_duty));
    	    }
    	    // send PWM value to LED
    	    NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwmLED);

    	    // Print setpoint, lux value, and duty cycle over serial port. Used for plotting
    	    xil_printf ( "Setpoint Value: %d\r\n", pid->setpoint );
    	    xil_printf ( "Lux Value: %d\r\n", tsl2561 );
    	    xil_printf ( "PWM LED Duty Cycle: %d\r\n", pwmLED );

    	    // update setpntLux with value to be displayed by display task
    	    setpntLux = 0x00000000; // make sure old data is cleared
    	    setpntLux |= (tsl2561 << 0) | (pid->setpoint << 16);//(uint32_t)((((uint32_t)(tsl2561) & lux_mask) << 0) | // sensor measured value in lower 32 bits
    	                 //(((uint32_t)(pid->setpoint) & lux_mask) << 16)); // setpoint in upper 32 bits

    	    //send message to display thread MsgQ to update setpoint and current lux
    	    xQueueSend (fromPID, &setpntLux ,mainDONT_BLOCK);

    	    // if center button is pressed print out the current PID configuration
    	    if ((btns & 0x04))
    	    {
    	    	print_pid(pid);
    	    }
    }
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
    uint32_t recievedLux;
    uint16_t setpnt = 0x0000, luxVal;
    while (1)
    {
        // recieve new sensor lux reading and setpoint values
        xQueueReceive (fromPID, &recievedLux, portMAX_DELAY);

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
    pid -> Kp = 0;
    pid -> Ki = 0;
    pid -> Kd = 0;
    pid -> setpoint = 250;
    pid -> integral = 0;
    pid -> prev_error = 0;
    pid -> delta_t = 0.437; // set to the worst case sampling time but will dynamically update in use
    // assumes that the float value returned is a 16 bit value, would need be adjusted 
    pid -> max_lim = 1000; // set max value to 99%, should result in setting LED PWM to 99%
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
float pid_funct (PID_t* pid, uint16_t lux_value, uint8_t switches)
{
    // e(t), error at time of sample
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

    // return a percentage value to be used for setting the intensity of PWM LED
    return (Pterm + Iterm + Dterm) / pid->setpoint;
}

// Prints out the PID gains when the center is pressed
void print_pid(PID_t *pid)
{
    xil_printf("PID gains:\r\n");
    xil_printf("Kp       = %u\r\n", pid->Kp);				// uint32_t
    xil_printf("Ki       = %u.%02u\r\n", (uint16_t)pid->Ki, (uint16_t)((pid->Ki - (uint16_t)pid->Ki) * 100));			// float, manipulated because xil_printf doesn't handle floating point
    xil_printf("Kd       = %u.%02u\r\n", (uint16_t)pid->Kd, (uint16_t)((pid->Kd - (uint16_t)pid->Kd) * 100));        	// float, manipulated because xil_printf doesn't handle floating point
}

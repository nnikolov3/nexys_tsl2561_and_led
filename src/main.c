/*
 * main.c - Entry point for TSL2561-based system with button handling, PID control, and sensor reading
 *
 * Purpose: Initializes FreeRTOS, sets up GPIO with interrupts for buttons, initializes the TSL2561 sensor 
 *          via I2C (polling), and creates tasks for button parsing, PID control, sensor reading, and 
 *          display updates. Designed for Nexys A7 with Microblaze, AXI, and TSL2561 sensor. Combines 
 *          LED toggling with PID-driven RGB LED control.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

/* Include necessary headers */
#include "main.h"

/* Duty Cycle Related Constants */
#define MAX_DUTY 255      // Max possible duty cycle for RGB1 Blue
#define MIN_DUTY 0        // Min possible duty cycle for RGB1 Blue
#define LUX_MASK 0xFFFF   // Mask for combining setpoint and lux values into one uint32_t

/* Global variables */
XGpio xInputGPIOInstance;            // GPIO instance for buttons and switches
SemaphoreHandle_t binary_sem = NULL; // Semaphore for GPIO interrupts
QueueHandle_t toPID = NULL;          // Queue for sending button/switch states to PID task
QueueHandle_t fromPID = NULL;        // Queue for sending setpoint/lux to display task
XIntc Intc;                          // Shared interrupt controller

/* Function prototypes */
static void gpio_intr(void* pvUnused);      // GPIO interrupt handler
void Parse_Input_Task(void* p);             // Task to parse button/switch inputs
void PID_Task(void* p);                     // Task for PID control of RGB LED
void Display_Task(void* p);                 // Task to update 7-segment display
void sensor_task(void* p);                  // Task to read TSL2561 sensor data
static int do_init(void);                   // System initialization
static int nexys_init(void);                // Nexys A7 peripheral initialization
static void seven_seg_selfTest(void);       // 7-segment display self-test
static int prvSetupHardware(void);          // GPIO hardware setup with interrupts
bool pid_init(PID_t *pid);                  // PID structure initialization
float pid_funct(PID_t* pid, float lux_value, uint8_t switches); // PID algorithm

/* PID structure definition */
typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain
    float setpoint;    // Desired lux value
    float integral;    // Accumulated error
    float prev_error;  // Previous error for derivative term
    float delta_t;     // Time between samples
    float max_lim;     // Maximum output limit
    float min_lim;     // Minimum output limit
} PID_t;

/**
 * Main entry point for the TSL2561 system.
 * Initializes hardware, sets up interrupts, creates FreeRTOS constructs, and starts the scheduler.
 */
int main(void) {
    xil_printf("\r\n==================================================\r\n");
    xil_printf("||          *** Starting TSL2561 System ***     ||\r\n");
    xil_printf("==================================================\r\n");

    /* Perform system initialization (no interrupt setup here) */
    if (do_init() != XST_SUCCESS) {
        xil_printf("\r\n[ERROR] *** Hardware initialization failed ***\r\n");
        while (1); // Halt to debug
    }

    /* Initialize and start interrupt controller after hardware setup */
    xil_printf("\r\n>>> Initializing Interrupt Controller\r\n");
    if (XIntc_Initialize(&Intc, INTC_DEVICE_ID) != XST_SUCCESS) {
        xil_printf("    [ERROR] *** Interrupt controller init failed ***\r\n");
        return -1;
    }
    if (XIntc_Start(&Intc, XIN_REAL_MODE) != XST_SUCCESS) {
        xil_printf("    [ERROR] *** Interrupt controller start failed ***\r\n");
        return -1;
    }
    xil_printf("    [SUCCESS] Interrupt controller ready\r\n");

    /* Enable global interrupts */
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XIntc_InterruptHandler, &Intc);
    Xil_ExceptionEnable();
    xil_printf("    [SUCCESS] Global interrupts enabled\r\n");

    /* Create semaphore for button interrupts */
    xil_printf("\r\n>>> Creating Binary Semaphore\r\n");
    vSemaphoreCreateBinary(binary_sem);
    if (binary_sem == NULL) {
        xil_printf("    [ERROR] *** Failed to create semaphore ***\r\n");
        return -1;
    }
    xil_printf("    [SUCCESS] Semaphore created\r\n");

    /* Create queues for button-to-PID and PID-to-display communication */
    xil_printf("\r\n>>> Creating Queues\r\n");
    toPID = xQueueCreate(MAIN_QUEUE_LENGTH, sizeof(uint16_t));
    fromPID = xQueueCreate(MAIN_QUEUE_LENGTH, sizeof(uint32_t));
    if (toPID == NULL || fromPID == NULL) {
        xil_printf("    [ERROR] *** Failed to create queues ***\r\n");
        return -1;
    }
    xil_printf("    [SUCCESS] Queues created\r\n");

    /* Create PID structure for LED control */
    PID_t ledPID;

    /* Create tasks */
    xil_printf("\r\n>>> Creating Tasks\r\n");
    xil_printf("    - Parse Input Task... ");
    xTaskCreate(Parse_Input_Task, "Parse_Input", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xil_printf("[DONE]\r\n");

    xil_printf("    - PID Task... ");
    xTaskCreate(PID_Task, "PID", configMINIMAL_STACK_SIZE, &ledPID, tskIDLE_PRIORITY + 2, NULL);
    xil_printf("[DONE]\r\n");

    xil_printf("    - Display Task... ");
    xTaskCreate(Display_Task, "Disp", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
    xil_printf("[DONE]\r\n");

    xil_printf("    - Sensor Task... ");
    xTaskCreate(sensor_task, "SENSOR", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xil_printf("[DONE]\r\n");

    /* Start the FreeRTOS scheduler */
    xil_printf("\r\n==================================================\r\n");
    xil_printf("||          *** Starting FreeRTOS Scheduler *** ||\r\n");
    xil_printf("||          Press buttons to adjust PID/LEDs    ||\r\n");
    xil_printf("==================================================\r\n");
    vTaskStartScheduler();

    xil_printf("\r\n[ERROR] *** Scheduler failed to start ***\r\n");
    return -1; // Should never reach here
}

/**
 * GPIO interrupt handler for button presses.
 * Signals the semaphore to wake the Parse Input task and clears the interrupt.
 *
 * @param pvUnused Unused parameter (required by ISR signature)
 */
static void gpio_intr(void* pvUnused) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xil_printf(">>> GPIO Interrupt Triggered\r\n");

    /* Signal the semaphore to notify the Parse Input task */
    xSemaphoreGiveFromISR(binary_sem, &xHigherPriorityTaskWoken);
    XGpio_InterruptClear(&xInputGPIOInstance, XGPIO_IR_MASK);

    /* Yield to higher-priority tasks if necessary */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * Task to parse button presses and switch states.
 * Waits for the semaphore, reads button/switch states, and sends them to the PID task via queue.
 *
 * @param p Unused parameter
 */
void Parse_Input_Task(void* p) {
    uint16_t ValueToSend = 0x0000;

    xil_printf("\r\n>>> Parse Input Task Started\r\n");

    while (1) {
        /* Wait for button interrupt via semaphore */
        if (xSemaphoreTake(binary_sem, pdMS_TO_TICKS(500)) == pdTRUE) {
            uint8_t btns = (NX4IO_getBtns() & 0x0C) >> 2; // Get BTNU/BTND, right justify
            uint8_t sws = (uint8_t)(NX4IO_getSwitches() & 0x00FF); // Get lower 8 switches
            ValueToSend = ((btns << 8) | sws); // Combine: btns in upper 8 bits, sws in lower
            xil_printf("    [PARSE] Sent btn/sw state: 0x%04X\r\n", ValueToSend);
            xQueueSend(toPID, &ValueToSend, MAIN_DONT_BLOCK);
            ValueToSend = 0x0000; // Clear for next iteration
        } else {
            xil_printf("    [PARSE] Timeout waiting for button\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Debounce delay
    }
}

/**
 * Task for PID control of RGB LED based on TSL2561 readings.
 * Adjusts LED brightness using PID algorithm and updates display task.
 *
 * @param p Pointer to PID_t structure
 */
void PID_Task(void* p) {
    PID_t* pid = (PID_t*)p; // Get the PID struct passed from main
    float pidOUT = 0;       // Float percent value returned from PID algorithm
    float tsl2561 = 0;      // Float value returned from TSL2561 sensor
    uint8_t pwmLED = 127;   // 8-bit int value for writing to PWM LED (initial 50%)
    uint16_t btnSws;        // Value received from input task queue
    uint32_t setpntLux;     // Value to send to display task queue
    uint8_t btns;           // Button values parsed from btnSws
    uint8_t sws;            // Switch values parsed from btnSws
    float baseID = 0.1;     // Base increment for integral/derivative tuning
    uint8_t baseSP = 1;     // Base increment for setpoint and Kp
    uint8_t incScaling;     // Scaling factor based on switch values
    TickType_t lastTick = 0;// Used for accurate delta_t calculation
    static bool isInitialized = false; // True if PID has been initialized

    xil_printf("\r\n>>> PID Task Started\r\n");

    /* Initialize PID structure if not already done */
    if (!isInitialized) {
        isInitialized = pid_init(pid);
        xil_printf("    [PID] Initialized with setpoint: %d\r\n", (int)pid->setpoint);
    }

    while (1) {
        /* Receive button/switch state from input task */
        xQueueReceive(toPID, &btnSws, portMAX_DELAY);
        btns = (btnSws & 0x0300) >> 8; // Extract buttons (BTNU/D)
        sws = (btnSws & 0x00FF);       // Extract switches

        /* Scale increments based on switches[5:4] */
        if (!(sws & 0x30)) { // [0:0]
            incScaling = 1;
        } else if (sws & 0x20) { // [1:X]
            incScaling = 10;
        } else if (sws & 0x10) { // [0:1]
            incScaling = 5;
        }

        /* Adjust PID parameters or setpoint based on switch[3] and buttons */
        if (sws & 0x08) { // Switch[3] on: Adjust setpoint
            if (btns == 1) { // BTNU
                pid->setpoint += (incScaling * baseSP);
            } else if (btns == 2) { // BTND
                pid->setpoint -= (incScaling * baseSP);
            }
        } else { // Switch[3] off: Adjust gains based on switches[7:6]
            switch (sws & 0xC0) {
                case 0x40: // Switch[6]: Kp
                    if (btns == 2) pid->Kp += (incScaling * baseSP);
                    if (btns == 1) pid->Kp -= (incScaling * baseSP);
                    break;
                case 0x80: // Switch[7]: Ki
                    if (btns == 2) pid->Ki += (incScaling * baseID);
                    if (btns == 1) pid->Ki -= (incScaling * baseID);
                    break;
                case 0xC0: // Both: Kd
                    if (btns == 2) pid->Kd += (incScaling * baseID);
                    if (btns == 1) pid->Kd -= (incScaling * baseID);
                    break;
                default:
                    break;
            }
        }

        /* Get TSL2561 reading (assume sensor_task updates a shared variable or queue) */
        uint16_t ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
        uint16_t ch1 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
        tsl2561 = (float)ch0; // Simplified: Use CH0 directly (add lux calc if needed)

        /* Calculate delta_t for PID */
        TickType_t currentTick = xTaskGetTickCount();
        pid->delta_t = ((currentTick - lastTick) * portTICK_PERIOD_MS) / 1000.0f;
        lastTick = currentTick;

        /* Run PID algorithm */
        pidOUT = pid_funct(pid, tsl2561, sws);
        xil_printf("    [PID] Output: %.2f, Setpoint: %d, Lux: %.0f\r\n", pidOUT, (int)pid->setpoint, tsl2561);

        /* Adjust PWM LED (RGB1 Blue) */
        if (pidOUT >= 0) {
            if ((pwmLED + (uint8_t)(pidOUT * MAX_DUTY)) >= MAX_DUTY) {
                pwmLED = MAX_DUTY;
            } else {
                pwmLED += (uint8_t)(pidOUT * MAX_DUTY);
            }
        } else {
            if ((pwmLED - (uint8_t)(-pidOUT * MAX_DUTY)) <= MIN_DUTY) {
                pwmLED = MIN_DUTY;
            } else {
                pwmLED -= (uint8_t)(-pidOUT * MAX_DUTY);
            }
        }
        NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, pwmLED);

        /* Update setpoint/lux for display */
        setpntLux = ((uint32_t)(pid->setpoint) & LUX_MASK) << 16 | ((uint32_t)tsl2561 & LUX_MASK);
        xQueueSend(fromPID, &setpntLux, MAIN_DONT_BLOCK);
    }
}

/**
 * Task to update 7-segment display with setpoint and lux values.
 *
 * @param p Unused parameter
 */
void Display_Task(void* p) {
    uint32_t receivedLux;
    uint16_t setpnt, luxVal;

    xil_printf("\r\n>>> Display Task Started\r\n");

    while (1) {
        /* Receive setpoint and lux from PID task */
        xQueueReceive(fromPID, &receivedLux, portMAX_DELAY);
        setpnt = (receivedLux >> 16) & LUX_MASK;
        luxVal = receivedLux & LUX_MASK;

        /* Update 7-segment display */
        NX410_SSEG_setAllDigits(SSEGHI, (uint8_t)(setpnt / 100), (uint8_t)((setpnt % 100) / 10), (uint8_t)((setpnt % 100) % 10), CC_BLANK, DP_NONE);
        NX410_SSEG_setAllDigits(SSEGLO, (uint8_t)(luxVal / 100), (uint8_t)((luxVal % 100) / 10), (uint8_t)((luxVal % 100) % 10), CC_BLANK, DP_NONE);
        xil_printf("    [DISPLAY] Setpoint: %u, Lux: %u\r\n", setpnt, luxVal);
    }
}

/**
 * Task to read TSL2561 sensor data.
 * Periodically reads CH0 and CH1 values from the TSL2561 sensor.
 *
 * @param p Unused parameter
 */
void sensor_task(void* p) {
    xil_printf("\r\n>>> Sensor Task Started\r\n");

    while (1) {
        /* Read sensor channels */
        uint16_t ch0 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_0);
        uint16_t ch1 = tsl2561_readChannel(&IicInstance, TSL2561_CHANNEL_1);
        xil_printf("    [SENSOR] CH0: %5u | CH1: %5u\r\n", ch0, ch1);

        vTaskDelay(pdMS_TO_TICKS(500)); // Read every 500ms
    }
}

/**
 * System initialization function.
 * Initializes hardware components (GPIO, Nexys, I2C, TSL2561) without interrupt setup.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
static int do_init(void) {
    int status;

    xil_printf("\r\n==================================================\r\n");
    xil_printf("||           *** Initializing System ***        ||\r\n");
    xil_printf("==================================================\r\n");

    /* Initialize Nexys A7 peripherals */
    xil_printf("\r\n>>> Initializing Nexys A7\r\n");
    status = nexys_init();
    if (status != XST_SUCCESS) {
        xil_printf("    [ERROR] *** Nexys initialization failed ***\r\n");
        return XST_FAILURE;
    }
    xil_printf("    [SUCCESS] Nexys initialized\r\n");

    /* Initialize I2C (polling mode, from i2c.c) */
    xil_printf("\r\n>>> Initializing I2C\r\n");
    status = i2c_init();
    if (status != XST_SUCCESS) {
        xil_printf("    [ERROR] *** I2C initialization failed ***\r\n");
        return XST_FAILURE;
    }
    xil_printf("    [SUCCESS] I2C initialized\r\n");

    /* Perform I2C self-test */
    status = XIic_SelfTest(&IicInstance);
    if (status != XST_SUCCESS) {
        xil_printf("    [ERROR] *** I2C self-test failed ***\r\n");
        return XST_FAILURE;
    }
    xil_printf("    [SUCCESS] I2C self-test passed\r\n");

    /* Scan I2C bus */
    xil_printf("\r\n>>> Scanning I2C Bus\r\n");
    i2c_scan(&IicInstance);

    /* Initialize TSL2561 sensor */
    xil_printf("\r\n>>> Initializing TSL2561 Sensor\r\n");
    status = tsl2561_init(&IicInstance);
    if (status != XST_SUCCESS) {
        xil_printf("    [ERROR] *** TSL2561 initialization failed ***\r\n");
        return XST_FAILURE;
    }
    xil_printf("    [SUCCESS] TSL2561 initialized\r\n");

    /* Perform 7-segment self-test */
    xil_printf("\r\n>>> Running 7-Segment Self-Test\r\n");
    xil_printf("    Displaying 'ECE544' to verify system readiness\r\n");
    seven_seg_selfTest();

    /* Initialize GPIO for buttons with interrupts */
    xil_printf("\r\n>>> Initializing Button GPIO\r\n");
    status = prvSetupHardware();
    if (status != 0) {
        xil_printf("    [ERROR] *** Button initialization failed ***\r\n");
        return XST_FAILURE;
    }
    xil_printf("    [SUCCESS] Buttons initialized\r\n");

    xil_printf("\r\n==================================================\r\n");
    xil_printf("||          *** System Initialization Complete *** ||\r\n");
    xil_printf("==================================================\r\n");
    return XST_SUCCESS;
}

/**
 * Nexys A7 peripheral initialization.
 * Sets up the Nexys4IO peripheral for LEDs and 7-segment display.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
static int nexys_init(void) {
    int status;

    xil_printf("    Initializing Nexys A7 peripherals...\r\n");
    status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("    [ERROR] *** Nexys4IO initialization failed, status: %d ***\r\n", status);
        return XST_FAILURE;
    }
    xil_printf("    Nexys4IO initialized successfully\r\n");
    return XST_SUCCESS;
}

/**
 * 7-segment display self-test.
 * Displays "ECE544" on the 7-segment display for 2 seconds to verify functionality.
 */
static void seven_seg_selfTest(void) {
    xil_printf("    Starting 7-segment display test...\r\n");
    xil_printf("    Displaying 'ECE544' for 2 seconds\r\n");
    NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E); // Display "ECE544"
    NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);
    usleep(2000 * 1000);
    xil_printf("    7-segment test completed\r\n");
}

/**
 * GPIO hardware setup.
 * Configures the GPIO for button inputs and installs the interrupt handler using FreeRTOS APIs.
 *
 * @return 0 if setup succeeds, 1 otherwise
 */
static int prvSetupHardware(void) {
    uint32_t xStatus;
    const unsigned char ucSetToInput = 0xFFU;

    xil_printf("    Initializing GPIO for buttons...\r\n");

    /* Initialize the GPIO for the button inputs */
    xStatus = XGpio_Initialize(&xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID);
    if (xStatus != XST_SUCCESS) {
        xil_printf("    [ERROR] *** GPIO initialization failed ***\r\n");
        configASSERT(0);
    }

    /* Install the handler defined in this task for the button input */
    xStatus = xPortInstallInterruptHandler(
        XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR,
        gpio_intr,
        NULL);

    if (xStatus == pdPASS) {
        xil_printf("    [SUCCESS] Buttons interrupt handler installed\r\n");

        /* Set switches and buttons to input */
        XGpio_SetDataDirection(&xInputGPIOInstance, BTN_CHANNEL, ucSetToInput);
        XGpio_SetDataDirection(&xInputGPIOInstance, SW_CHANNEL, ucSetToInput);

        /* Enable the button input interrupts in the interrupt controller */
        vPortEnableInterrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR);

        /* Enable GPIO channel interrupts on button channel */
        XGpio_InterruptEnable(&xInputGPIOInstance, XGPIO_IR_CH1_MASK);
        XGpio_InterruptGlobalEnable(&xInputGPIOInstance);
    } else {
        xil_printf("    [ERROR] *** Failed to install GPIO interrupt handler ***\r\n");
        configASSERT(0);
    }

    configASSERT(xStatus == pdPASS);
    return (xStatus == pdPASS) ? 0 : 1;
}

/**
 * PID structure initialization.
 * Sets initial values for PID parameters.
 *
 * @param pid Pointer to PID_t structure
 * @return true if initialized successfully
 */
bool pid_init(PID_t *pid) {
    pid->Kp = 0;
    pid->Ki = 0;
    pid->Kd = 0;
    pid->setpoint = 250; // Initial setpoint (adjustable via buttons)
    pid->integral = 0;
    pid->prev_error = 0;
    pid->delta_t = 0.437; // Initial delta_t (437ms, worst-case sampling time)
    pid->max_lim = 65536; // Max lux value (16-bit)
    pid->min_lim = 0;     // Min lux value

    return true;
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
float pid_funct(PID_t* pid, float lux_value, uint8_t switches) {
    float error = pid->setpoint - lux_value;

    /* Proportional term */
    float Pterm = (switches & 0x01) ? pid->Kp * error : 0;

    /* Integral term */
    pid->integral += (error * pid->delta_t);
    float Iterm = (switches & 0x02) ? pid->Ki * pid->integral : 0;

    /* Derivative term */
    float Dterm = (switches & 0x04) ? pid->Kd * ((error - pid->prev_error) / pid->delta_t) : 0;

    pid->prev_error = error;

    return (Pterm + Iterm + Dterm) / pid->setpoint;
}
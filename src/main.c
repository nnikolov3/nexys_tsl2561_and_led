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
#include "FreeRTOS.h"
#include "task.h"
#include "tsl2561.h"
#include "xiic.h"

/* Task function prototypes */
void sensorTask(void *pvParameters);
void pidTask(void *pvParameters);

/* I2C instance (assumed configured elsewhere) */
XIic i2cInstance;

int main() {
  /* Initialize I2C and TSL2561 sensor */
  XIic_Initialize(&i2cInstance, XPAR_IIC_0_DEVICE_ID);
  tsl2561_init(&i2cInstance);

  /* Create FreeRTOS tasks */
  xTaskCreate(sensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(pidTask, "PID", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler();

  /* Should never reach here unless scheduler fails */
  return 0;
}

void sensorTask(void *pvParameters) {
  for (;;) {
    // TODO: Read TSL2561 and send lux to PID task
    vTaskDelay(pdMS_TO_TICKS(50)); // Run every 50ms
  }
}

void pidTask(void *pvParameters) {
  for (;;) {
    // TODO: Calculate PID output and adjust PWM
    vTaskDelay(pdMS_TO_TICKS(50)); // Run every 50ms
  }
}
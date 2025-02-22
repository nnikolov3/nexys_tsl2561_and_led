#include "FreeRTOS.h"
#include "task.h"
#include "tsl2561.h"
#include "xiic.h"

/* Task function prototypes */
void sensorTask(void *pvParameters);
void pidTask(void *pvParameters);

/* I2C instance (assumed configured elsewhere) */
XIic i2cInstance;

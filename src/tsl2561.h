/*
 * tsl2561.h - Header file for TSL2561 Luminosity Sensor driver
 *
 * Purpose: This driver provides an interface to the TSL2561 sensor for
 * measuring light intensity (lux) via I2C. It supports initialization, reading
 *          raw channel data, and calculating lux values for use in a PID
 *          controller to adjust LED brightness in a closed-loop system.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */
#ifndef TSL2561_H
#define TSL2561_H

/* Include necessary headers */
#include "xiic.h"
#include <stdint.h>

/* Enum to define TSL2561 channels */
typedef enum
{
    TSL2561_CHANNEL_0 = 0, // Visible + IR channel
    TSL2561_CHANNEL_1 = 1  // IR-only channel
} tsl2561_channel_t;

/* Struct to hold TSL2561 channel data */
typedef struct
{
    uint16_t ch0; // Visible + IR channel value (16-bit)
    uint16_t ch1; // IR-only channel value (16-bit)
} tsl2561_data_t;

/* Function Prototypes */

void     tsl2561_init ( XIic* i2c );
uint16_t tsl2561_readChannel ( XIic* i2c, tsl2561_channel_t channel );
float    tsl2561_calculateLux ( uint16_t ch0, uint16_t ch1 );

#endif /* TSL2561_H */

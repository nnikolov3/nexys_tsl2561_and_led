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

#define TSL2561_ADDR 0x39

// Register addresses
#define TSL2561_REG_CONTROL 0x00   // 0x00 | 0x80
#define TSL2561_REG_TIMING 0x01    // 0x01 | 0x80
#define TSL2561_REG_INTERRUPT 0x06 // 0x06 | 0x80

/*
| 7 | 6 | 5 | 4    | 3 | 2 | 1 | 0 |
|CMD|CLR|WORD|BLOCK|--ADDRESS------|

CMD (bit 7): Must always be set to 1 (0x80 in hexadecimal) to indicate this is a
command.
CLR, WORD, BLOCK (bits 6-4): Additional control bits (set to 0 here for
simplicity). ADDRESS (bits 3-0): The 4-bit address of the register you want to
access (e.g., 0x00 for CONTROL, 0x01 for TIMING, 0x06 for INTERRUPT).

The TSL2561 requires this command byte as the first byte in an I2C transaction
to select the register.
*/

// Command format: CMD bit (0x80) | register address
#define TSL2561_CMD_CONTROL ( 0x80 | TSL2561_REG_CONTROL )     // 0x80
#define TSL2561_CMD_TIMING ( 0x80 | TSL2561_REG_TIMING )       // 0x81
#define TSL2561_CMD_INTERRUPT ( 0x80 | TSL2561_REG_INTERRUPT ) // 0x86

// Register values
#define TSL2561_POWER_ON 0x03 // Power up
// High gain (16x), 402ms integration (bits: 0b00010010)
#define TSL2561_TIMING_402MS_16X 0x12
#define TSL2561_INT_DISABLE 0x00 // Disable interrupts

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
/* Interrupt handler prototype */
void TSL2561_I2C_InterruptHandler ( void* CallBackRef );

#endif /* TSL2561_H */

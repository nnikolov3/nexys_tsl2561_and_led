/*
 * tsl2561.h - Header file for TSL2561 Luminosity Sensor driver
 *
 * Purpose: Provides an interface to the TSL2561 sensor for measuring
 *          light intensity (lux) via I2C. Focuses on initialization
 *          and testing sensor connectivity on the board.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#ifndef TSL2561_H
#define TSL2561_H

/* Standard Includes */
#include <stdint.h>

/* FreeRTOS and Xilinx Includes */
#include "FreeRTOS.h"
#include "sleep.h"
#include "task.h"
#include "xiic.h"
#include "xil_printf.h"

/* Device Configuration */
#define TSL2561_ADDR             0x39 // Device I2C address (may need adjustment)

/* Register Addresses */
#define TSL2561_REG_CONTROL      0x00 // Control register
#define TSL2561_REG_TIMING       0x01 // Timing register
#define TSL2561_REG_INTERRUPT    0x06 // Interrupt control register
#define TSL2561_REG_ID           0x0A // Device ID register
#define TSL2561_REG_DATA0LOW     0x0C // Channel 0 low byte
#define TSL2561_REG_DATA0HIGH    0x0D // Channel 0 high byte
#define TSL2561_REG_DATA1LOW     0x0E // Channel 1 low byte
#define TSL2561_REG_DATA1HIGH    0x0F // Channel 1 high byte

/* Command Bytes (Register Address | 0x80) */
#define TSL2561_CMD_CONTROL      0x80 // Control register command
#define TSL2561_CMD_TIMING       0x81 // Timing register command
#define TSL2561_CMD_INTERRUPT    0x86 // Interrupt register command
#define TSL2561_CMD_DATA0        0x8C // Channel 0 data command
#define TSL2561_CMD_DATA1        0x8E // Channel 1 data command

/* Configuration Values */
#define TSL2561_POWER_ON         0x03 // Power on value
#define TSL2561_POWER_OFF        0x00 // Power off value
#define TSL2561_TIMING_402MS_16X 0x12 // 402ms integration, 16x gain
#define TSL2561_INT_DISABLE      0x00 // Disable interrupts

/* External Variables */
extern XIic I2C_Instance; // I2C instance (defined in main.c)

/* Enumerations */
typedef enum
{
    TSL2561_CHANNEL_0 = 0, // Visible + IR channel
    TSL2561_CHANNEL_1 = 1  // IR-only channel
} tsl2561_channel_t;

/* Function Prototypes */
int tsl2561_init( XIic* i2c );
uint16_t tsl2561_readChannel( XIic* i2c, tsl2561_channel_t channel );
float tsl2561_calculateLux( uint16_t ch0, uint16_t ch1 );
void i2c_scan( XIic* i2c );
void i2c_read_status( XIic* i2c );
void i2c_read_control( XIic* i2c );
void i2c_soft_reset( XIic* i2c );

#endif /* TSL2561_H */

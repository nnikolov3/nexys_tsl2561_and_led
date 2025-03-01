/*
 * tsl2561.c - Driver implementation for TSL2561 luminosity sensor
 *
 * Purpose: Provides functions to initialize the TSL2561 sensor and read its
 * channels (CH0 and CH1) over I2C in polling mode using XIic_Send for writes.
 * Enhanced with pre-initialization power-off, reset, and I2C restarts before
 * each operation. Designed for Nexys A7 FPGA, Microblaze, and FreeRTOS.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "tsl2561.h"

/* Forward declarations from i2c.c for state checks and reset */
extern void i2c_read_control( XIic* InstancePtr );
extern void i2c_read_status( XIic* InstancePtr );
extern int i2c_soft_reset( XIic* InstancePtr );

/**
 * Initializes the TSL2561 sensor over I2C with pre-reset and debug checks.
 * Powers off the sensor, resets I2C, then powers on and configures timing with
 * restarts.
 *
 * @param i2c Pointer to the initialized IIC instance
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
int tsl2561_init( XIic* i2c )
{
    uint8_t send[ 2 ];     // Buffer for I2C write commands (command + data)
    uint8_t read_buf[ 1 ]; // Buffer for reading back register values
    int status;            // Status return value for I2C operations
    int write_status  = 1; // Track success of critical writes
    const int TIMEOUT = TIMEOUT_COUNTER; // Timeout limit for bus idle check
    const int RESET_TIMEOUT = 20000;     // 20ms timeout for reset

    xil_printf( "\r\n----------------------------------------\r\n" );
    xil_printf( "|    TSL2561 Initialization Started    |\r\n" );
    xil_printf( "----------------------------------------\r\n" );

    /* Pre-initialization: Power off TSL2561 and reset I2C */
    xil_printf( "       Powering off TSL2561 and resetting I2C...\r\n" );
    send[ 0 ] = TSL2561_CMD_CONTROL; // Command byte (0x80)
    send[ 1 ] = TSL2561_POWER_OFF;   // Power-off value (0x00)
    XIic_Stop( i2c );
    usleep( 5000 );
    status = XIic_Start( i2c );
    if( status == XST_SUCCESS )
    {
        status =
            XIic_Send( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
        if( status == 2 )
        {
            xil_printf( "       TSL2561 powered off\r\n" );
        }
        else
        {
            xil_printf( "       [WARN] ---- Failed to power off TSL2561 (bytes "
                        "sent: %d) ----\r\n",
                        status );
        }
    }
    else
    {
        xil_printf( "       [WARN] ---- I2C restart failed before power-off "
                    "(status: %d) ----\r\n",
                    status );
    }
    i2c_soft_reset( i2c );
    usleep( RESET_TIMEOUT );
    xil_printf( "       I2C reset complete, waiting %dµs for stabilization\r\n",
                RESET_TIMEOUT );

    /* Wait for the I2C bus to become idle */
    int timeout = TIMEOUT;
    while( XIic_IsIicBusy( i2c ) && --timeout > 0 )
        ;
    if( timeout == 0 )
    {
        xil_printf(
            "       [ERROR] ---- I2C bus busy timeout after %d cycles ----\r\n",
            TIMEOUT );
        return XST_FAILURE;
    }
    xil_printf( "       I2C bus available\r\n" );

    /* Delay 100ms to ensure bus stability */
    usleep( 100000 );
    xil_printf( "       100ms stabilization delay complete\r\n" );

    /* Set the TSL2561 I2C slave address (0x39) */
    status = XIic_SetAddress( i2c, XII_ADDR_TO_SEND_TYPE, TSL2561_ADDR );
    if( status != XST_SUCCESS )
    {
        xil_printf(
            "       [ERROR] ---- Failed to set TSL2561 address 0x%02X ----\r\n",
            TSL2561_ADDR );
        return XST_FAILURE;
    }
    xil_printf( "       TSL2561 address set to 0x%02X\r\n", TSL2561_ADDR );

    /* Debug: Read initial Control Register state */
    send[ 0 ] = TSL2561_CMD_CONTROL;
    XIic_Stop( i2c );
    usleep( 5000 );
    status = XIic_Start( i2c );
    if( status == XST_SUCCESS )
    {
        status = XIic_Send(
            i2c->BaseAddress, TSL2561_ADDR, send, 1, XIIC_REPEATED_START );
        if( status != 1 )
        {
            xil_printf( "       [DEBUG] ---- Failed to send Control Reg "
                        "address (bytes sent: %d) ----\r\n",
                        status );
        }
        else
        {
            status = XIic_Recv(
                i2c->BaseAddress, TSL2561_ADDR, read_buf, 1, XIIC_STOP );
            if( status == 1 )
            {
                xil_printf( "       [DEBUG] Initial Control Reg: 0x%02X\r\n",
                            read_buf[ 0 ] );
            }
            else
            {
                xil_printf( "       [DEBUG] ---- Failed to read initial "
                            "Control Reg (bytes read: %d) ----\r\n",
                            status );
            }
        }
    }
    else
    {
        xil_printf( "       [WARN] ---- I2C restart failed before initial read "
                    "(status: %d) ----\r\n",
                    status );
    }

    /* Power on the TSL2561 by writing to the Control Register */
    send[ 0 ] = TSL2561_CMD_CONTROL;
    send[ 1 ] = TSL2561_POWER_ON;
    XIic_Stop( i2c );
    usleep( 5000 );
    status = XIic_Start( i2c );
    if( status == XST_SUCCESS )
    {
        status =
            XIic_Send( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
        if( status != 2 )
        {
            xil_printf(
                "       [ERROR] ---- Power-on failed (bytes sent: %d) ----\r\n",
                status );
            write_status = 0;
        }
        else
        {
            xil_printf( "       TSL2561 powered on\r\n" );

            /* Verify power-on by reading back Control Register */
            send[ 0 ] = TSL2561_CMD_CONTROL;
            XIic_Stop( i2c );
            usleep( 5000 );
            status = XIic_Start( i2c );
            if( status == XST_SUCCESS )
            {
                status = XIic_Send( i2c->BaseAddress,
                                    TSL2561_ADDR,
                                    send,
                                    1,
                                    XIIC_REPEATED_START );
                if( status == 1 )
                {
                    status = XIic_Recv( i2c->BaseAddress,
                                        TSL2561_ADDR,
                                        read_buf,
                                        1,
                                        XIIC_STOP );
                    if( status == 1 )
                    {
                        xil_printf( "       [DEBUG] Control Reg after "
                                    "power-on: 0x%02X (expected 0x%02X)\r\n",
                                    read_buf[ 0 ],
                                    TSL2561_POWER_ON );
                        if( read_buf[ 0 ] != TSL2561_POWER_ON )
                        {
                            xil_printf( "       [WARN] ---- Control Reg "
                                        "mismatch ----\r\n" );
                        }
                    }
                    else
                    {
                        xil_printf( "       [DEBUG] ---- Failed to verify "
                                    "power-on (bytes read: %d) ----\r\n",
                                    status );
                    }
                }
            }
        }
    }
    else
    {
        xil_printf( "       [WARN] ---- I2C restart failed before power-on "
                    "(status: %d) ----\r\n",
                    status );
        write_status = 0;
    }

    /* Delay 100ms to allow power-on to stabilize */
    usleep( 100000 );
    xil_printf( "       100ms power-on delay complete\r\n" );

    /* Configure timing: 402ms integration time, 16x gain */
    send[ 0 ] = TSL2561_CMD_TIMING;
    send[ 1 ] = TSL2561_TIMING_402MS_16X;
    XIic_Stop( i2c );
    usleep( 5000 );
    status = XIic_Start( i2c );
    if( status == XST_SUCCESS )
    {
        status =
            XIic_Send( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
        if( status != 2 )
        {
            xil_printf( "       [ERROR] ---- Timing config failed (bytes sent: "
                        "%d) ----\r\n",
                        status );
            write_status = 0;
        }
        else
        {
            xil_printf( "       Timing set: 402ms integration, 16x gain\r\n" );

            /* Verify timing by reading back Timing Register */
            send[ 0 ] = TSL2561_CMD_TIMING;
            XIic_Stop( i2c );
            usleep( 5000 );
            status = XIic_Start( i2c );
            if( status == XST_SUCCESS )
            {
                status = XIic_Send( i2c->BaseAddress,
                                    TSL2561_ADDR,
                                    send,
                                    1,
                                    XIIC_REPEATED_START );
                if( status == 1 )
                {
                    status = XIic_Recv( i2c->BaseAddress,
                                        TSL2561_ADDR,
                                        read_buf,
                                        1,
                                        XIIC_STOP );
                    if( status == 1 )
                    {
                        xil_printf( "       [DEBUG] Timing Reg after config: "
                                    "0x%02X (expected 0x%02X)\r\n",
                                    read_buf[ 0 ],
                                    TSL2561_TIMING_402MS_16X );
                        if( read_buf[ 0 ] != TSL2561_TIMING_402MS_16X )
                        {
                            xil_printf( "       [WARN] ---- Timing Reg "
                                        "mismatch ----\r\n" );
                        }
                    }
                    else
                    {
                        xil_printf( "       [DEBUG] ---- Failed to verify "
                                    "timing (bytes read: %d) ----\r\n",
                                    status );
                    }
                }
            }
        }
    }
    else
    {
        xil_printf( "       [WARN] ---- I2C restart failed before timing "
                    "config (status: %d) ----\r\n",
                    status );
        write_status = 0;
    }

    /* Delay 410ms for integration time to complete */
    usleep( 410000 );
    xil_printf( "       410ms integration delay complete\r\n" );

    /* Debug: Read Device ID (0x0A) to confirm communication */
    send[ 0 ] = ID_REG | 0x80;
    XIic_Stop( i2c );
    usleep( 5000 );
    status = XIic_Start( i2c );
    if( status == XST_SUCCESS )
    {
        status = XIic_Send(
            i2c->BaseAddress, TSL2561_ADDR, send, 1, XIIC_REPEATED_START );
        if( status == 1 )
        {
            status = XIic_Recv(
                i2c->BaseAddress, TSL2561_ADDR, read_buf, 1, XIIC_STOP );
            if( status == 1 )
            {
                xil_printf( "       [DEBUG] Device ID: 0x%02X (expected ~0x50 "
                            "for TSL2561)\r\n",
                            read_buf[ 0 ] );
            }
            else
            {
                xil_printf( "       [DEBUG] ---- Failed to read Device ID "
                            "(bytes read: %d) ----\r\n",
                            status );
            }
        }
        else
        {
            xil_printf( "       [DEBUG] ---- Failed to send ID Reg address "
                        "(bytes sent: %d) ----\r\n",
                        status );
        }
    }

    /* Test reading channels regardless of prior failures */
    xil_printf( "       [DEBUG] Testing channel reads...\r\n" );
    uint16_t ch0 = tsl2561_readChannel( i2c, TSL2561_CHANNEL_0 );
    uint16_t ch1 = tsl2561_readChannel( i2c, TSL2561_CHANNEL_1 );
    xil_printf( "       [DEBUG] CH0: %5u | CH1: %5u\r\n", ch0, ch1 );

    /* Debug: Final I2C state check */
    xil_printf( "       [DEBUG] Post-init I2C state:\r\n" );
    i2c_read_control( i2c );
    i2c_read_status( i2c );

    /* Return based on write success */
    if( write_status )
    {
        xil_printf( "----------------------------------------\r\n" );
        xil_printf( "|    TSL2561 Initialization Complete   |\r\n" );
        xil_printf( "----------------------------------------\r\n" );
        return XST_SUCCESS;
    }
    else
    {
        xil_printf(
            "       [ERROR] ---- TSL2561 initialization failed ----\r\n" );
        return XST_FAILURE;
    }
}

/**
 * Reads a channel (CH0 or CH1) from the TSL2561 sensor with debug output and
 * I2C restart. Sends the register address and receives 2 bytes (low and high)
 * to form a 16-bit value.
 *
 * @param i2c Pointer to the initialized IIC instance
 * @param channel Channel to read (TSL2561_CHANNEL_0 or TSL2561_CHANNEL_1)
 * @return 16-bit channel value on success, 0 on failure
 */
uint16_t tsl2561_readChannel( XIic* i2c, tsl2561_channel_t channel )
{
    uint8_t buf[ 2 ]; // Buffer for register address (1 byte) and data (2 bytes)
    int status;       // Status return value for I2C operations

    /* Select the low byte register based on the channel */
    buf[ 0 ] = ( channel == TSL2561_CHANNEL_0 ? DATA0LOW_REG : DATA1LOW_REG ) |
               0x80; // Set command bit

    /* Restart I2C before read */
    XIic_Stop( i2c );
    usleep( 5000 );
    status = XIic_Start( i2c );
    if( status != XST_SUCCESS )
    {
        xil_printf( "       [ERROR] ---- I2C restart failed before read "
                    "(status: %d) ----\r\n",
                    status );
        return 0;
    }

    /* Send the register address with repeated start */
    status = XIic_Send(
        i2c->BaseAddress, TSL2561_ADDR, buf, 1, XIIC_REPEATED_START );
    if( status != 1 )
    {
        xil_printf( "       [ERROR] ---- Send address 0x%02X failed (bytes "
                    "sent: %d) ----\r\n",
                    buf[ 0 ],
                    status );
        return 0;
    }

    /* Receive 2 bytes (low and high) from the selected channel */
    status = XIic_Recv( i2c->BaseAddress, TSL2561_ADDR, buf, 2, XIIC_STOP );
    if( status != 2 )
    {
        xil_printf( "       [ERROR] ---- Receive failed for channel %d (bytes "
                    "read: %d) ----\r\n",
                    channel,
                    status );
        return 0;
    }

    /* Combine high and low bytes into a 16-bit value */
    uint16_t value = ( buf[ 1 ] << 8 ) | buf[ 0 ];
    xil_printf( "       [DEBUG] Channel %d read: %5u (Raw: 0x%04X)\r\n",
                channel,
                value,
                value );
    return value;
}

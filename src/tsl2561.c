#include "tsl2561.h"

#include "xil_printf.h"

int tsl2561_init( XIic* i2c )
{
    uint8_t send[ 2 ];
    int status;

    usleep( 100000 ); // 100ms delay after power-up

    xil_printf( "TSL2561 init: Address 0x%02X\n", TSL2561_ADDR );
    xil_printf( "Sending power-on command...\n" );

    send[ 0 ] = TSL2561_CMD_CONTROL;
    send[ 1 ] = TSL2561_POWER_ON;

    status    = XIic_Send( i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP );
    if( status != 2 )
    {
        xil_printf( "Failed to power on: %d bytes sent\n", status );
        return 1;
    }

    xil_printf( "Power-on command sent\n" );
    usleep( 100000 ); // 100ms delay

    return 0;
}

void i2c_scan( XIic* i2c )
{
    uint8_t send[ 1 ] = { 0x00 };
    for( uint8_t addr = 0x00; addr <= 0x77; addr++ )
    {
        int status = XIic_Send( i2c->BaseAddress, addr, send, 0, XIIC_STOP );
        usleep( 10000 ); // 10ms delay
        if( status >= 0 )
        {
            xil_printf( "Found device at address 0x%02X\n", addr );
        }
        else
        {
            xil_printf(
                "No device at address 0x%02X: %d bytes sent\n", addr, status );
        }
        usleep( 5000 ); // Small delay between scans
    }
}

void i2c_read_control( XIic* i2c )
{
    u32 control = XIic_ReadReg( i2c->BaseAddress, 0x100 );
    xil_printf( "I2C Control Register (CR): 0x%08X\n", control );
    if( control & 0x01 )
        xil_printf( " - Master Mode\n" );
    if( control & 0x02 )
        xil_printf( " - Transmit Mode\n" );
    if( control & 0x04 )
        xil_printf( " - Transmit FIFO Enabled\n" );
    if( control & 0x08 )
        xil_printf( " - Receive FIFO Enabled\n" );
    if( control & 0x10 )
        xil_printf( " - General Call Enabled\n" );
    if( control & 0x20 )
        xil_printf( " - Repeated Start Enabled\n" );
    if( control & 0x40 )
        xil_printf( " - Clear Transmit FIFO\n" );
    if( control & 0x80 )
        xil_printf( " - IIC Enabled\n" );
}

void i2c_read_status( XIic* i2c )
{
    u32 status = XIic_ReadReg( i2c->BaseAddress, 0x104 );
    xil_printf( "I2C Status Register (SR): 0x%08X\n", status );
    if( status & 0x01 )
        xil_printf( " - Bus Busy\n" );
    if( status & 0x02 )
        xil_printf( " - Addressed As Slave\n" );
    if( status & 0x04 )
        xil_printf( " - Not Addressed As Slave\n" );
    if( status & 0x08 )
        xil_printf( " - Transmit FIFO Empty\n" );
    if( status & 0x10 )
        xil_printf( " - Receive FIFO Empty\n" );
    if( status & 0x20 )
        xil_printf( " - Transmit FIFO Full\n" );
    if( status & 0x40 )
        xil_printf( " - Receive FIFO Full\n" );
    if( status & 0x80 )
        xil_printf( " - Slave Read Direction\n" );
}

void i2c_soft_reset( XIic* i2c )
{
    xil_printf( "Performing I2C soft reset...\n" );
    // Write 0xA to SOFTR to reset the peripheral
    XIic_WriteReg( i2c->BaseAddress, 0x40, 0xA );
    usleep( 10000 ); // 10ms delay

    // Manually clear FIFOs by toggling CR bits
    XIic_WriteReg(
        i2c->BaseAddress, 0x100, 0x40 ); // CR = 0x40 (Clear Transmit FIFO)
    usleep( 1000 );
    XIic_WriteReg( i2c->BaseAddress, 0x100, 0x0 ); // CR = 0x0 (reset state)
    usleep( 1000 );
}

/*
uint16_t tsl2561_readChannel( XIic* i2c, tsl2561_channel_t channel )
{
    u8 low_reg = ( channel == CHANNEL0 ) ? DATA0LOW_REG : DATA1LOW_REG;
    u8 buf[ 2 ];
    u16 result;

    // Read two bytes (low and high)
    buf[ 0 ] = low_reg | 0x80; // Command byte
    XIic_Send( i2c->BaseAddress, TSL2561_ADDR, buf, 1, XIIC_REPEATED_START );
    XIic_Recv( i2c->BaseAddress, TSL2561_ADDR, buf, 2, XIIC_STOP );

    result = ( buf[ 1 ] << 8 ) | buf[ 0 ];
    return result;
}
*/
/*
float tsl2561_calculateLux( uint16_t ch0, uint16_t ch1 )
{
    // Simplified lux calculation (default settings: 402ms, gain = 1x)
    // Refer to TSL2561 datasheet for exact coefficients
    float ratio = ( ch1 == 0 ) ? 0 : ( (float) ch1 / (float) ch0 );
    float lux;

    if( ratio < 0.5 )
    {
        lux = 0.0304 * ch0 - 0.062 * ch0 * pow( ratio, 1.4 );
    }
    else if( ratio < 0.61 )
    {
        lux = 0.0224 * ch0 - 0.031 * ch1;
    }
    else if( ratio < 0.85 )
    {
        lux = 0.0128 * ch0 - 0.0153 * ch1;
    }
    else
    {
        lux = 0; // Above 0.85, assume saturation or use additional logic
    }
    return ( lux < 0 ) ? 0 : lux;
}
*/

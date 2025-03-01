/*
 * i2c.c - I2C driver implementation for AXI IIC controller on Nexys A7
 *
 * Purpose: Provides polling-based I2C functionality with robust
 * pre-initialization reset, scanning, and state management for TSL2561
 * integration on Microblaze with FreeRTOS.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "i2c.h"

/* Global variables */
XIic IicInstance; // IIC driver instance for accessing the AXI IIC hardware
XIic_Config* ConfigPtr; // Pointer to I2C configuration data
extern XIntc Intc; // Shared interrupt controller instance (defined in main.c)

/**
 * Initializes the AXI IIC controller in polling mode with pre-reset.
 * Ensures a clean state by stopping, resetting, and re-enabling I2C with
 * retries.
 *
 * @return XST_SUCCESS if initialization succeeds, XST_FAILURE otherwise
 */
int i2c_init( void )
{
    int status;
    const int MAX_RETRIES   = 5; // Increased retries for robustness
    int retry_count         = 0;
    const int RESET_TIMEOUT = 20000; // 20ms timeout for reset

    xil_printf( "\r\n----------------------------------------\r\n" );
    xil_printf( "|    I2C Initialization Started        |\r\n" );
    xil_printf( "----------------------------------------\r\n" );

    /* Lookup I2C configuration from hardware parameters */
    ConfigPtr = XIic_LookupConfig( IIC_DEVICE_ID );
    if( !ConfigPtr )
    {
        xil_printf( "       [ERROR] ---- I2C config lookup failed ----\r\n" );
        return XST_FAILURE;
    }
    xil_printf( "       Config found - Base Address: 0x%08X\r\n",
                ConfigPtr->BaseAddress );

    /* Initialize the I2C instance */
    status =
        XIic_CfgInitialize( &IicInstance, ConfigPtr, ConfigPtr->BaseAddress );
    if( status != XST_SUCCESS )
    {
        xil_printf( "       [ERROR] ---- I2C init failed (status: %d) ----\r\n",
                    status );
        return XST_FAILURE;
    }
    xil_printf( "       I2C instance initialized\r\n" );

    /* Turn off I2C and reset to clear any prior state */
    xil_printf( "       Turning off I2C and resetting...\r\n" );
    XIic_WriteReg( IicInstance.BaseAddress, 0x100, 0x00 ); // Disable IIC
    usleep( 5000 ); // 5ms delay to ensure disable
    XIic_Stop( &IicInstance );
    usleep( 5000 );
    i2c_soft_reset( &IicInstance );
    usleep( RESET_TIMEOUT ); // Extended timeout for reset stability
    xil_printf( "       I2C reset complete, waiting %dµs for stabilization\r\n",
                RESET_TIMEOUT );

    /* Enable I2C in master mode with retries */
    do
    {
        XIic_WriteReg(
            IicInstance.BaseAddress, 0x100, 0x81 ); // Master + IIC Enable
        u32 control = XIic_ReadReg( IicInstance.BaseAddress, 0x100 );
        if( control & 0x80 )
        { // Check IIC Enabled bit
            xil_printf( "       Control Register set: 0x%08X\r\n", control );
            break;
        }
        xil_printf( "       [WARN] ---- IIC not enabled on attempt %d (CR: "
                    "0x%08X) ----\r\n",
                    retry_count + 1,
                    control );
        i2c_soft_reset( &IicInstance );
        usleep( RESET_TIMEOUT );
        retry_count++;
    } while( retry_count < MAX_RETRIES );

    if( retry_count >= MAX_RETRIES )
    {
        xil_printf( "       [ERROR] ---- Failed to enable IIC after %d retries "
                    "----\r\n",
                    MAX_RETRIES );
        return XST_FAILURE;
    }

    /* Start the I2C controller */
    status = XIic_Start( &IicInstance );
    if( status != XST_SUCCESS )
    {
        xil_printf(
            "       [ERROR] ---- I2C start failed (status: %d) ----\r\n",
            status );
        return XST_FAILURE;
    }
    xil_printf( "       I2C controller started\r\n" );

    /* Set the default slave address (e.g., TSL2561 at 0x39) */
    status =
        XIic_SetAddress( &IicInstance, XII_ADDR_TO_SEND_TYPE, I2C_SLAVE_ADDR );
    if( status != XST_SUCCESS )
    {
        xil_printf(
            "       [ERROR] ---- Setting slave address 0x%02X failed ----\r\n",
            I2C_SLAVE_ADDR );
        return XST_FAILURE;
    }
    xil_printf( "       Slave address set to: 0x%02X\r\n", I2C_SLAVE_ADDR );

    /* Debug: Verify initial I2C state */
    xil_printf( "       [DEBUG] Initial I2C state:\r\n" );
    i2c_read_control( &IicInstance );
    i2c_read_status( &IicInstance );

    xil_printf( "----------------------------------------\r\n" );
    xil_printf( "|    I2C Initialization Complete       |\r\n" );
    xil_printf( "----------------------------------------\r\n" );
    return XST_SUCCESS;
}

/**
 * Scans the I2C bus for devices from address 0x00 to 0x77 using polling.
 * Probes each address with a 0-byte write, validates ACKs with strict checks,
 * and resets the bus if necessary to ensure accurate device detection.
 *
 * @param InstancePtr Pointer to the initialized IIC instance
 */
void i2c_scan( XIic* InstancePtr )
{
    int devices_found       = 0;
    uint8_t probe_data[ 1 ] = { 0x00 }; // Buffer for 0-byte write probe
    const int TIMEOUT       = 100000;   // Timeout for bus idle check
    int status;

    xil_printf( "\r\n----------------------------------------\r\n" );
    xil_printf( "|    I2C Bus Scan Started              |\r\n" );
    xil_printf( "----------------------------------------\r\n" );

    /* Wait for the I2C bus to become idle */
    int timeout = TIMEOUT;
    while( XIic_IsIicBusy( InstancePtr ) && --timeout > 0 )
        ;
    if( timeout == 0 )
    {
        xil_printf(
            "       [ERROR] ---- Bus busy timeout after %d cycles ----\r\n",
            TIMEOUT );
        i2c_soft_reset( InstancePtr );
        return;
    }
    xil_printf( "       Bus idle - Ready to scan\r\n" );

    /* Debug: Pre-scan I2C state */
    xil_printf( "       [DEBUG] Pre-scan I2C state:\r\n" );
    i2c_read_control( InstancePtr );
    i2c_read_status( InstancePtr );

    /* Ensure IIC is enabled before scanning */
    u32 control = XIic_ReadReg( InstancePtr->BaseAddress, 0x100 );
    if( !( control & 0x80 ) )
    {
        xil_printf( "       [WARN] ---- IIC not enabled, attempting re-enable "
                    "----\r\n" );
        XIic_WriteReg( InstancePtr->BaseAddress, 0x100, 0x81 );
        usleep( 1000 );
        control = XIic_ReadReg( InstancePtr->BaseAddress, 0x100 );
        if( !( control & 0x80 ) )
        {
            xil_printf( "       [ERROR] ---- Failed to re-enable IIC (CR: "
                        "0x%08X) ----\r\n",
                        control );
            return;
        }
        xil_printf( "       IIC re-enabled: 0x%08X\r\n", control );
    }

    xil_printf( "       Scanning addresses 0x00 to 0x77...\r\n" );
    for( uint8_t addr = 0x00; addr <= 0x77; addr++ )
    {
        /* Skip reserved address ranges (0x00-0x07 and 0x78-0x7F) */
        if( ( addr & 0xF8 ) == 0 || ( addr & 0xF8 ) == 0x78 )
            continue;

        /* Set the address to probe */
        status = XIic_SetAddress( InstancePtr, XII_ADDR_TO_SEND_TYPE, addr );
        if( status != XST_SUCCESS )
        {
            xil_printf(
                "       [WARN] ---- Failed to set address 0x%02X ----\r\n",
                addr );
            continue;
        }

        /* Clear FIFOs and restart I2C before probing */
        XIic_WriteReg( InstancePtr->BaseAddress, 0x100, 0x40 ); // Clear Tx FIFO
        usleep( 1000 );
        XIic_Stop( InstancePtr );
        usleep( 1000 );
        status = XIic_Start( InstancePtr );
        if( status != XST_SUCCESS )
        {
            xil_printf(
                "       [ERROR] ---- I2C restart failed (status: %d) ----\r\n",
                status );
            continue;
        }

        /* Probe the address with a 0-byte write */
        status = XIic_MasterSend( InstancePtr, probe_data, 0 );
        if( status == XST_SUCCESS )
        {
            /* Validate ACK with stricter Status Register check */
            u32 sr = XIic_ReadReg( InstancePtr->BaseAddress, 0x104 );
            if( !( sr & 0x01 ) && !( sr & 0x02 ) && ( sr & 0x08 ) &&
                !( sr & 0x40 ) && !( sr & 0x80 ) )
            {
                // Bus not busy, Not Addressed As Slave, Tx FIFO Empty, No FIFO
                // Full
                xil_printf( "       Found device at 0x%02X (SR: 0x%08X)\r\n",
                            addr,
                            sr );
                devices_found++;
            }
            else
            {
                xil_printf(
                    "       [DEBUG] False positive at 0x%02X (SR: 0x%08X)\r\n",
                    addr,
                    sr );
            }
        }
        else
        {
            xil_printf( "       [DEBUG] No ACK at 0x%02X (status: %d)\r\n",
                        addr,
                        status );
        }
        usleep( 10000 ); // Increased delay for bus stability
    }

    xil_printf( "       Scan complete - Detected %d device(s)\r\n",
                devices_found );
    xil_printf( "       [DEBUG] Post-scan I2C state:\r\n" );
    i2c_read_control( InstancePtr );
    i2c_read_status( InstancePtr );
    xil_printf( "----------------------------------------\r\n" );
}

/**
 * Reads and displays the contents of the I2C Control Register (CR).
 * Provides a detailed breakdown of active control bits for debugging.
 *
 * @param InstancePtr Pointer to the initialized IIC instance
 */
void i2c_read_control( XIic* InstancePtr )
{
    u32 control = XIic_ReadReg( InstancePtr->BaseAddress, 0x100 );
    xil_printf( "       [DEBUG] Control Register (0x100): 0x%08X\r\n",
                control );

    static const char* states[] = { "Master Mode",
                                    "Transmit Mode",
                                    "Tx FIFO Enabled",
                                    "Rx FIFO Enabled",
                                    "General Call Enabled",
                                    "Repeated Start",
                                    "Clear Tx FIFO",
                                    "IIC Enabled" };

    int any_set                 = 0;
    for( int i = 0; i < 8; i++ )
    {
        if( control & ( 1 << i ) )
        {
            xil_printf( "         - Bit %d: %s\r\n", i, states[ i ] );
            any_set = 1;
        }
    }
    if( !any_set )
    {
        xil_printf( "         - No features enabled\r\n" );
    }

    if( !( control & 0x80 ) )
    {
        xil_printf( "       [WARN] ---- IIC not enabled ----\r\n" );
    }
}

/**
 * Reads and displays the contents of the I2C Status Register (SR).
 * Provides a detailed breakdown of status bits with diagnostic notes.
 *
 * @param InstancePtr Pointer to the initialized IIC instance
 */
void i2c_read_status( XIic* InstancePtr )
{
    u32 status = XIic_ReadReg( InstancePtr->BaseAddress, 0x104 );
    xil_printf( "       [DEBUG] Status Register (0x104): 0x%08X\r\n", status );

    static const char* states[] = { "Bus Busy",
                                    "Addressed As Slave",
                                    "Not Addressed As Slave",
                                    "Tx FIFO Empty",
                                    "Rx FIFO Empty",
                                    "Tx FIFO Full",
                                    "Rx FIFO Full",
                                    "Slave Read Direction" };

    int any_set                 = 0;
    for( int i = 0; i < 8; i++ )
    {
        if( status & ( 1 << i ) )
        {
            xil_printf( "         - Bit %d: %s\r\n", i, states[ i ] );
            any_set = 1;
        }
    }
    if( !any_set )
    {
        xil_printf( "         - No status flags set\r\n" );
    }

    if( status & 0x01 )
    {
        xil_printf(
            "       [NOTE] ---- Bus busy: Operations may block ----\r\n" );
    }
}

/**
 * Performs a soft reset of the I2C peripheral.
 * Resets the controller, clears FIFOs, and re-enables I2C operation with state
 * verification.
 *
 * @param InstancePtr Pointer to the initialized IIC instance
 * @return XST_SUCCESS if reset succeeds, XST_FAILURE if bus remains busy
 */
int i2c_soft_reset( XIic* InstancePtr )
{
    xil_printf( "\r\n----------------------------------------\r\n" );
    xil_printf( "|    I2C Soft Reset Started            |\r\n" );
    xil_printf( "----------------------------------------\r\n" );

    xil_printf( "       [DEBUG] Pre-reset I2C state:\r\n" );
    i2c_read_control( InstancePtr );
    i2c_read_status( InstancePtr );

    XIic_WriteReg( InstancePtr->BaseAddress, 0x40, 0xA );
    xil_printf( "       Soft reset triggered\r\n" );
    usleep( 20000 ); // Increased to 20ms for thorough reset

    XIic_WriteReg( InstancePtr->BaseAddress, 0x100, 0x40 );
    usleep( 5000 );
    XIic_WriteReg( InstancePtr->BaseAddress, 0x100, 0x00 );
    usleep( 5000 );
    xil_printf( "       FIFOs cleared, CR reset\r\n" );

    XIic_WriteReg( InstancePtr->BaseAddress, 0x100, 0x81 );
    usleep( 5000 );
    xil_printf( "       I2C re-enabled\r\n" );

    u32 status = XIic_ReadReg( InstancePtr->BaseAddress, 0x104 );
    if( status & 0x01 )
    {
        xil_printf( "       [ERROR] ---- Reset failed: Bus still busy (SR: "
                    "0x%08X) ----\r\n",
                    status );
        return XST_FAILURE;
    }

    xil_printf( "       [DEBUG] Post-reset I2C state:\r\n" );
    i2c_read_control( InstancePtr );
    i2c_read_status( InstancePtr );

    xil_printf( "       Soft reset completed successfully\r\n" );
    xil_printf( "----------------------------------------\r\n" );
    return XST_SUCCESS;
}

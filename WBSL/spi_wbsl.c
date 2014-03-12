// *************************************************************************************************
// Functions handling SPI communications with TI's CC1101 during rFBSL.
// *************************************************************************************************



// *************************************************************************************************
// Include section

#include "spi_wbsl.h"

// *************************************************************************************************
// Defines section

/* ------------------------------------------------------------------------------------------------
 *      Local Functions Define
 * ------------------------------------------------------------------------------------------------
 */

static void spiBurstFifoAccess(u8 addrByte, u8 * pData, u8 len);
static u8 spiRegAccess(u8 addrByte, u8 writeValue);


/**************************************************************************************************
 * @fn          wbsl_SpiInit
 *
 * @brief       Initialize SPI.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */

void wbsl_SpiInit(void)
{
  /* configure all SPI related pins */
    WBSL_SPI_CONFIG_CSN_PIN_AS_OUTPUT();
    WBSL_SPI_CONFIG_SCLK_PIN_AS_OUTPUT();
    WBSL_SPI_CONFIG_SI_PIN_AS_OUTPUT();
    WBSL_SPI_CONFIG_SO_PIN_AS_INPUT();

    /* set CSn to default high level */
    WBSL_SPI_DRIVE_CSN_HIGH();

    /* initialize the SPI registers */
    WBSL_SPI_INIT();
}



/**************************************************************************************************
 * @fn          wbsl_SpiCmdStrobe
 *
 * @brief       Send command strobe to the radio.  Returns status byte read during transfer
 *              of strobe command.
 *
 * @param       addr - address of register to strobe
 *
 * @return      status byte of radio
 **************************************************************************************************
 */

u8 wbsl_SpiCmdStrobe(u8 addr)
{
    u8 statusByte;
    wbslSpiIState_t s;

    WBSL_SPI_ASSERT( WBSL_SPI_IS_INITIALIZED() );       /* SPI is not initialized */
    WBSL_SPI_ASSERT((addr >= 0x30) && (addr <= 0x3D));  /* invalid address */

    /* disable interrupts that use SPI */
    WBSL_SPI_ENTER_CRITICAL_SECTION(s);

    /* turn chip select "off" and then "on" to clear any current SPI access */
    WBSL_SPI_TURN_CHIP_SELECT_OFF();
    WBSL_SPI_TURN_CHIP_SELECT_ON();

    /* send the command strobe, wait for SPI access to complete */
    WBSL_SPI_WRITE_BYTE(addr);
    WBSL_SPI_WAIT_DONE();

    /* read the readio status byte returned by the command strobe */
    statusByte = WBSL_SPI_READ_BYTE();

    /* turn off chip select; enable interrupts that call SPI functions */
    WBSL_SPI_TURN_CHIP_SELECT_OFF();
    WBSL_SPI_EXIT_CRITICAL_SECTION(s);

    /* return the status byte */
    return (statusByte);
}



/**************************************************************************************************
 * @fn          wbsl_SpiReadReg
 *
 * @brief       Read value from radio register.
 *
 * @param       addr - address of register
 *
 * @return      register value
 **************************************************************************************************
 */

u8 wbsl_SpiReadReg(u8 addr)
{
    WBSL_SPI_ASSERT(addr <= 0x3B);    /* invalid address */

    /*
     *  The burst bit is set to allow access to read-only status registers.
     *  This does not affect normal register reads.
     */
    return ( spiRegAccess(addr | BURST_BIT | READ_BIT, DUMMY_BYTE) );
}


/**************************************************************************************************
 * @fn          wbsl_SpiWriteReg
 *
 * @brief       Write value to radio register.
 *
 * @param       addr  - address of register
 * @param       value - register value to write
 *
 * @return      none
 **************************************************************************************************
 */

void wbsl_SpiWriteReg(u8 addr, u8 value)
{
     WBSL_SPI_ASSERT((addr <= 0x2E) || (addr == 0x3E));    /* invalid address */

     spiRegAccess(addr, value);
}



/**************************************************************************************************
 * @fn          wbsl_SpiReadRxFifo
 *
 * @brief       Read data from radio receive FIFO.
 *
 * @param       pData - pointer for storing read data
 * @param       len   - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */

void wbsl_SpiReadRxFifo(u8 * pData, u8 len)
{
    spiBurstFifoAccess(RXFIFO | BURST_BIT | READ_BIT, pData, len);
}



/**************************************************************************************************
 * @fn          wbsl_SpiWriteTxFifo
 *
 * @brief       Write data to radio transmit FIFO.
 *
 * @param       pData - pointer for storing write data
 * @param       len   - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */

void wbsl_SpiWriteTxFifo(u8 * pData, u8 len)
{
    spiBurstFifoAccess(TXFIFO | BURST_BIT, pData, len);
}



/*=================================================================================================
 * @fn          spiRegAccess
 *
 * @brief       This function performs a read or write.  The
 *              calling code must configure the read/write bit of the register's address byte.
 *              This bit is set or cleared based on the type of access.
 *
 * @param       regAddrByte - address byte of register; the read/write bit already configured
 *
 * @return      register value
 *=================================================================================================
 */

static u8 spiRegAccess(u8 addrByte, u8 writeValue)
{
    u8 readValue;
    wbslSpiIState_t s;

    WBSL_SPI_ASSERT( WBSL_SPI_IS_INITIALIZED() );   /* SPI is not initialized */

     /* disable interrupts that use SPI */
    WBSL_SPI_ENTER_CRITICAL_SECTION(s);

     /* turn chip select "off" and then "on" to clear any current SPI access */
    WBSL_SPI_TURN_CHIP_SELECT_OFF();
    WBSL_SPI_TURN_CHIP_SELECT_ON();

     /* send register address byte, the read/write bit is already configured */
    WBSL_SPI_WRITE_BYTE(addrByte);
    WBSL_SPI_WAIT_DONE();

     /*
      *  Send the byte value to write.  If this operation is a read, this value
      *  is not used and is just dummy data.  Wait for SPI access to complete.
      */
    WBSL_SPI_WRITE_BYTE(writeValue);
    WBSL_SPI_WAIT_DONE();

    /*
     *  If this is a read operation, SPI data register now contains the register
     *  value which will be returned.  For a read operation, it contains junk info
     *  that is not used.
     */
    readValue = WBSL_SPI_READ_BYTE();

    /* turn off chip select; enable interrupts that call SPI functions */
    WBSL_SPI_TURN_CHIP_SELECT_OFF();
    WBSL_SPI_EXIT_CRITICAL_SECTION(s);

    /* return the register value */
    return (readValue);
}



/*=================================================================================================
 * @fn          spiBurstFifoAccess
 *
 * @brief       Burst mode access used for reading or writing to radio FIFOs.
 *
 *              For more efficient interrupt latency, this function does not keep interrupts
 *              disabled for its entire execution.  It is designed to recover if an interrupt
 *              occurs that accesses SPI.  See comments in code for further details.
 *
 * @param       addrByte - first byte written to SPI, contains address and mode bits
 * @param       pData    - pointer to data to read or write
 * @param       len      - length of data in bytes
 *
 * @return      none
 *=================================================================================================
 */

static void spiBurstFifoAccess(u8 addrByte, u8 * pData, u8 len)
{
    wbslSpiIState_t s;


    WBSL_SPI_ASSERT( WBSL_SPI_IS_INITIALIZED() );   /* SPI is not initialized */
    WBSL_SPI_ASSERT(len != 0);                      /* zero length is not allowed */
    WBSL_SPI_ASSERT(addrByte & BURST_BIT);          /* only burst mode supported */

    /* disable interrupts that use SPI */
    WBSL_SPI_ENTER_CRITICAL_SECTION(s);

    /* turn chip select "off" and then "on" to clear any current SPI access */
    WBSL_SPI_TURN_CHIP_SELECT_OFF();
    WBSL_SPI_TURN_CHIP_SELECT_ON();

    /*-------------------------------------------------------------------------------
    *  Main loop.  If the SPI access is interrupted, execution comes back to
    *  the start of this loop.  Loop exits when nothing left to transfer.
    */
    do
    {
        /* send FIFO access command byte, wait for SPI access to complete */
       // UCB0IFG
        WBSL_SPI_WRITE_BYTE(addrByte);
        WBSL_SPI_WAIT_DONE();

        /*-------------------------------------------------------------------------------
         *  Inner loop.  This loop executes as long as the SPI access is not interrupted.
         *  Loop completes when nothing left to transfer.
         */
       do
       {
           WBSL_SPI_WRITE_BYTE(*pData);

           /*-------------------------------------------------------------------------------
            *  Use idle time.  Perform increment/decrement operations before pending on
            *  completion of SPI access.
            *
            *  Decrement the length counter.  Wait for SPI access to complete.
            */
           len--;
           WBSL_SPI_WAIT_DONE();

           /*-------------------------------------------------------------------------------
            *  SPI data register holds data just read.  If this is a read operation,
            *  store the value into memory.
            */
           if (addrByte & READ_BIT)
           {
               *pData = WBSL_SPI_READ_BYTE();
           }

           /*-------------------------------------------------------------------------------
            *  At least one byte of data has transferred.  Briefly enable (and then disable)
            *  interrupts that can call SPI functions.  This provides a window for any timing
            *  critical interrupts that might be pending.
            *
            *  To improve latency, take care of pointer increment within the interrupt
            *  enabled window.
            */
           WBSL_SPI_EXIT_CRITICAL_SECTION(s);
           pData++;
           WBSL_SPI_ENTER_CRITICAL_SECTION(s);

           /*-------------------------------------------------------------------------------
            *  If chip select is "off" the SPI access was interrupted (all SPI access
            *  functions leave chip select in the "off" state).  In this case, turn
            *  back on chip select and break to the main loop.  The main loop will
            *  pick up where the access was interrupted.
            */
           if (WBSL_SPI_CSN_IS_HIGH())
           {
               WBSL_SPI_TURN_CHIP_SELECT_ON();
               break;
           }

     /*-------------------------------------------------------------------------------
      */
       } while (len); /* inner loop */
    } while (len);   /* main loop */

     /* turn off chip select; enable interrupts that call SPI functions */
    WBSL_SPI_TURN_CHIP_SELECT_OFF();
    WBSL_SPI_EXIT_CRITICAL_SECTION(s);
}

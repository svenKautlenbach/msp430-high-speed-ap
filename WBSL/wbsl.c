// *************************************************************************************************
//
//	Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
//
//
//	  Redistribution and use in source and binary forms, with or without
//	  modification, are permitted provided that the following conditions
//	  are met:
//
//	    Redistributions of source code must retain the above copyright
//	    notice, this list of conditions and the following disclaimer.
//
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the
//	    documentation and/or other materials provided with the
//	    distribution.
//
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
// Wireless Update function
// *************************************************************************************************

// *************************************************************************************************
// Include section
#include "wbsl.h"
#include "project.h"
#include <string.h>
#include "bsp.h"
#include "bsp_macros.h"
#include "spi_wbsl.h"


/* ------------------------------------------------------------------------------------------------
 *                                    Local Constants
 * ------------------------------------------------------------------------------------------------
 */
static const u8 wbslRadioCfg[][2] =
{
    /* internal radio configuration */
    {  MCSM2,     WBSL_SETTING_MCSM2     },
    {  MCSM1,     WBSL_SETTING_MCSM1     },
    {  MCSM0,     WBSL_SETTING_MCSM0     },
    {  SYNC1,     WBSL_SETTING_SYNC1     },
    {  SYNC0,     WBSL_SETTING_SYNC0     },
    {  PKTLEN,    WBSL_SETTING_PKTLEN    },
    {  PKTCTRL1,  WBSL_SETTING_PKTCTRL1  },
    {  PKTCTRL0,  WBSL_SETTING_PKTCTRL0  },
    {  ADDR,      WBSL_SETTING_ADDR      },
    {  FIFOTHR,   WBSL_SETTING_FIFOTHR   },
    {  WOREVT1,   WBSL_SETTING_WOREVT1   },
    {  WOREVT0,   WBSL_SETTING_WOREVT0   },
    {  WORCTRL,   WBSL_SETTING_WORCTRL   },
    /* imported SmartRF radio configuration */
    {  CHANNR,    WBSL_SETTING_CHANNR    },
    {  FSCTRL1,   WBSL_SETTING_FSCTRL1   },
    {  FSCTRL0,   WBSL_SETTING_FSCTRL0   },
    {  FREQ2,     WBSL_SETTING_FREQ2     },
    {  FREQ1,     WBSL_SETTING_FREQ1     },
    {  FREQ0,     WBSL_SETTING_FREQ0     },
    {  MDMCFG4,   WBSL_SETTING_MDMCFG4   },
    {  MDMCFG3,   WBSL_SETTING_MDMCFG3   },
    {  MDMCFG2,   WBSL_SETTING_MDMCFG2   },
    {  MDMCFG1,   WBSL_SETTING_MDMCFG1   },
    {  MDMCFG0,   WBSL_SETTING_MDMCFG0   },
    {  DEVIATN,   WBSL_SETTING_DEVIATN   },

    {  FOCCFG,    WBSL_SETTING_FOCCFG    },
    {  BSCFG,     WBSL_SETTING_BSCFG     },
    {  AGCCTRL2,  WBSL_SETTING_AGCCTRL2  },
    {  AGCCTRL1,  WBSL_SETTING_AGCCTRL1  },
    {  AGCCTRL0,  WBSL_SETTING_AGCCTRL0  },
    {  FREND1,    WBSL_SETTING_FREND1    },
    {  FREND0,    WBSL_SETTING_FREND0    },
    {  FSCAL3,    WBSL_SETTING_FSCAL3    },
    {  FSCAL2,    WBSL_SETTING_FSCAL2    },
    {  FSCAL1,    WBSL_SETTING_FSCAL1    },
    {  FSCAL0,    WBSL_SETTING_FSCAL0    },
    {  AGCTEST,   WBSL_SETTING_AGCTEST   },
    {  PTEST,     WBSL_SETTING_PTEST     },
    {  FSTEST,    WBSL_SETTING_FSTEST    },
    {  TEST2,     WBSL_SETTING_TEST2     },
    {  TEST1,     WBSL_SETTING_TEST1     },
    {  TEST0,     WBSL_SETTING_TEST0     },
};

static const u8 ResetRadioCfg[][2] =
{
    {  MCSM2,      RESET_VALUE_MCSM2      },
    {  PKTCTRL1,   RESET_VALUE_PKTCTRL1   },
    {  ADDR,       RESET_VALUE_ADDR       },
    {  CHANNR,     RESET_VALUE_CHANNR     },

    {  FSCTRL1,    RESET_VALUE_FSCTRL1    },
    {  FSCTRL0,    RESET_VALUE_FSCTRL0    },
    {  MDMCFG4,    RESET_VALUE_MDMCFG4    },
    {  MDMCFG3,    RESET_VALUE_MDMCFG3    },
    {  MDMCFG2,    RESET_VALUE_MDMCFG2    },
    {  MDMCFG1,    RESET_VALUE_MDMCFG1    },
    {  MDMCFG0,    RESET_VALUE_MDMCFG0    },
    {  DEVIATN,    RESET_VALUE_DEVIATN    },
    {  MCSM1,      RESET_VALUE_MCSM1      },
    {  MCSM0,      RESET_VALUE_MCSM0      },
    {  FOCCFG,     RESET_VALUE_FOCCFG     },
    {  BSCFG,      RESET_VALUE_BSCFG      },
    {  AGCCTRL2,   RESET_VALUE_AGCCTRL2   },
    {  AGCCTRL1,   RESET_VALUE_AGCCTRL1   },
    {  AGCCTRL0,   RESET_VALUE_AGCCTRL0   },
    {  FREND1,     RESET_VALUE_FREND1     },
    {  FSCAL3,     RESET_VALUE_FSCAL3     },
    {  FSCAL2,     RESET_VALUE_FSCAL2     },
    {  FSCAL1,     RESET_VALUE_FSCAL1     },
    {  FSCAL0,     RESET_VALUE_FSCAL0     },
    {  TEST0,      RESET_VALUE_TEST0      },
};

/* ------------------------------------------------------------------------------------------------
 *      Local Functions Define
 * ------------------------------------------------------------------------------------------------
 */

static void rxModeOff(void);
static void rxModeOn(void);
static void sendDataPacket(void);
static void sendInitPacket(void);
static void wbsl_DelayUsec(u16 howLong);
static void reset_RF_registers(void);

/* ------------------------------------------------------------------------------------------------
 *       Global Variable section
 * ------------------------------------------------------------------------------------------------
 */

/* reserve space for the maximum possible peer Link IDs */
static u8  wNumCurrentPeers;
static u8 update_complete; /* flag the completion of WBSL update */
static u8 wbsl_number_of_retries; /* Handle number of ACKs sent for an indivudual packet */
static u8 RxBuffer[RX_BUFFER_SIZE];
static u8 discoveryPayload[8] = {7,0,WBSL_AP_ADDRESS,0xBA,0x5E,0xBA,0x11,9};
static u8 discoveryAck[4] = {3, 0, WBSL_AP_ADDRESS, 0};
static u8 wbsl_rxtxMode = 0; /* indicate the current mode RX or TX */
static u16 currentPacket; /* Keep track of which packet needs to be sent to the Watch */
static u8  initOk = 0;   /* Variable to see if the Init Packet has been successfully sent */
static u8  sInitWbslDone=0; /* indicate the Radio Module configuration for WBSL is done */

volatile u8 wbsl_flag;
volatile u8 wbsl_packet_flag;
volatile u8 packet_ready_flag;
volatile u8 rxtx_flag = 0;
unsigned int total_packets; /* Store the total number of packets to be sent to the Watch */
u8 wbsl_data[WBSL_MAX_PAYLOAD_LENGTH];
u8 wbsl_data_length = 0;
u8 TxBuffer[TX_SIZE];
u8 wbsl_status;
u8 initPacket[] = {5,0,WBSL_AP_ADDRESS,0,0,0};
u8 ed_address;

/* ------------------------------------------------------------------------------------------------
 *    Macros
 * ------------------------------------------------------------------------------------------------
 */

/* There is no bit in h/w to tell if RSSI in the register is valid or not.
 * The hardware needs to be in RX state for a certain amount of time before
 * a valid RSSI value is calculated and placed in the register. This min
 * wait time is defined by WBSL_RSSI_VALID_DELAY_US. We don't need to
 * add such delay every time RSSI value is needed. If the Carrier Sense signal
 * is high or CCA signal is high, we know that the RSSI value must be valid.
 * We use that knowledge to reduce our wait time. We break down the delay loop
 * in multiple chunks and during each iteration, check for the CS and CCA
 * signal. If either of these signals is high, we return immediately. Else,
 * we wait for the max delay specified.
 *
 */

#define WBSL_RSSI_VALID_WAIT()                                                \
{                                                                             \
    int16_t delay = WBSL_RSSI_VALID_DELAY_US;                                   \
    do                                                                          \
    {                                                                           \
        if (wbsl_SpiReadReg(PKTSTATUS) & (WBSL_PKTSTATUS_CCA | WBSL_PKTSTATUS_CS))  \
        {                                                                         \
            break;                                                                  \
        }                                                                         \
        wbsl_DelayUsec(64);                                                       \
        delay -= 64;                                                              \
    }while(delay > 0);                                                          \
}


#define WBSL_STROBE_IDLE_AND_WAIT()                   \
{                                                     \
    wbsl_SpiCmdStrobe( SIDLE );          \
    while (wbsl_SpiCmdStrobe( SNOP ) & 0xF0) ;           \
}


/****************************************************************************************************
 * @fn          wbsl_DelayUsec
 *
 * @brief       Execute a delay loop using HW timer. The macro actually used to do the delay
 *              is not thread-safe. This routine makes the delay execution thread-safe by breaking
 *              up the requested delay up into small chunks and executing each chunk as a critical
 *              section. The chunk size is chosen to be the smallest value used by the radio. The delay
 *              is only approximate because of the overhead computations. It errs on the side of
 *              being too long.
 *
 * input parameters
 * @param   howLong - number of microseconds to delay
 *
 * @return      none
 ****************************************************************************************************
 */
static void wbsl_DelayUsec(u16 howLong)
{
    bspIState_t s;
    u16 count = howLong/WBSL_MAX_DELAY_US;

    if (howLong)
    {
        do
        {
            BSP_ENTER_CRITICAL_SECTION(s);
            BSP_DELAY_USECS(WBSL_MAX_DELAY_US);
            BSP_EXIT_CRITICAL_SECTION(s);
         }while (count--);
     }
}


/*************************************************************************************************
 * @fn          reset_RF_registers
 * @brief       Restore default values for RF registers. The function ensures to leave the CC1101
 *              in a workable state after the Wireless Update has completed. SRES command strobe is not used
 *              as it may affect the CSn pin and XOSC (clock source for the USB module).
 * @param       none
 * @return      none
 *************************************************************************************************
 */
static void reset_RF_registers(void)
{
	u8 i;
    for (i=0; i<(sizeof(ResetRadioCfg)/sizeof(ResetRadioCfg[0])); i++)
    {
	    wbsl_SpiWriteReg(ResetRadioCfg[i][0], ResetRadioCfg[i][1]);
	}
}



// *************************************************************************************************
// @fn          wbsl_config
// @brief       Configures the Radio Settings for WBSL
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_config(void)
{

    if (!sInitWbslDone)
	{
	   /* ------------------------------------------------------------------
		*    Initialization
		*   -----------------
		*/

		/* initialize GPIO pins */
	    WBSL_CONFIG_GDO0_PIN_AS_INPUT();

		/* initialize SPI */
		wbsl_SpiInit();

		/* ------------------------------------------------------------------
		 *    Radio power-up reset
		 * ----------------------
		 */
	    WBSL_SPI_ASSERT(WBSL_SPI_CSN_IS_HIGH());

		/* pulse CSn low then high */
		WBSL_SPI_DRIVE_CSN_LOW();
		wbsl_DelayUsec(10);
		WBSL_SPI_DRIVE_CSN_HIGH();

		/* hold CSn high for at least 40 microseconds */
		wbsl_DelayUsec(40);

	    /* pull CSn low and wait for SO to go low */
		WBSL_SPI_DRIVE_CSN_LOW();
		while (WBSL_SPI_SO_IS_HIGH());

		/* return CSn pin to its default high level */
		WBSL_SPI_DRIVE_CSN_HIGH();

	    /* ------------------------------------------------------------------
		 *    Run-time integrity checks
		 * ---------------------------
		 */

	    /* verify that SPI is working, PKTLEN is an arbitrary read/write register used for testing */
		#ifdef WBSL_ASSERTS_ARE_ON
		#define TEST_VALUE 0xA5
		    wbsl_SpiWriteReg( PKTLEN, TEST_VALUE );
		    WBSL_SPI_ASSERT( mrfiSpiReadReg( PKTLEN ) == TEST_VALUE ); /* SPI is not responding */
		#endif

		  /* verify the correct radio is installed */
	    WBSL_SPI_ASSERT( wbsl_SpiReadReg( PARTNUM ) == WBSL_RADIO_PARTNUM );      /* incorrect radio specified */
		WBSL_SPI_ASSERT( wbsl_SpiReadReg( VERSION ) >= WBSL_RADIO_MIN_VERSION );  /* obsolete radio specified  */

		  /* ------------------------------------------------------------------
		   *    Configure radio
		   *   -----------------
		   */

		  /* initialize radio registers */
		{
		    u8 i;

		  	for (i=0; i<(sizeof(wbslRadioCfg)/sizeof(wbslRadioCfg[0])); i++)
		  	{
		       wbsl_SpiWriteReg(wbslRadioCfg[i][0], wbslRadioCfg[i][1]);
		  	}
		}


		  /* ------------------------------------------------------------------
		   *    Configure interrupts
		   *   ----------------------
		   */

		  /*
		   *  Configure and enable the SYNC signal interrupt.
		   *
		   *  This interrupt is used to indicate receive.  The SYNC signal goes
		   *  high when a receive OR a transmit begins.  It goes high once the
		   *  sync word is received or transmitted and then goes low again once
		   *  the packet completes.
		   */

		 WBSL_CONFIG_GDO0_AS_SYNC_SIGNAL();
		 WBSL_CONFIG_SYNC_PIN_FALLING_EDGE_INT();
		 WBSL_CLEAR_SYNC_PIN_INT_FLAG();

		  /* enable global interrupts */
		  BSP_ENABLE_INTERRUPTS();
	}

    sInitWbslDone=1;

}



/**************************************************************************************************
 * @fn          wbsl_GpioIsr
 *
 * @brief       Interrupt Service Routine for handling GPIO interrupts.  The sync pin interrupt
 *              comes in through GPIO.  This function is designed to be compatible with "ganged"
 *              interrupts.  If the GPIO interrupt services more than just a single pin (very
 *              common), this function just needs to be called from the higher level interrupt
 *              service routine.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void wbsl_GpioIsr(void)
{
    /* see if sync pin interrupt is enabled and has fired */
    if (WBSL_SYNC_PIN_INT_IS_ENABLED() && WBSL_SYNC_PIN_INT_FLAG_IS_SET())
    {
        /*  clear the sync pin interrupt, run sync pin ISR */

        /*
         *  NOTE!  The following macro clears the interrupt flag but it also *must*
         *  reset the interrupt capture.  In other words, if a second interrupt
         *  occurs after the flag is cleared it must be processed, i.e. this interrupt
         *  exits then immediately starts again.  Most microcontrollers handle this
         *  naturally but it must be verified for every target.
         */
        WBSL_CLEAR_SYNC_PIN_INT_FLAG();

        wbsl_RfIsr();

     }
}



// *************************************************************************************************
// @fn          wbsl_RfIsr
// @brief       called when a packet has been received. In this function we check for RX Overflow and set the corresponding
//              flags to inform the application a packet is ready on the RxBuffer
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_RfIsr(void)
{
    u8 rxBytes;
    u8 IncomingPacket[RX_BUFFER_SIZE]={0};


   /* We should only be here in RX mode, not in TX mode, nor if RX mode was turned on during CCA */
    if (wbsl_rxtxMode != WBSL_RX_MODE)
    {
        return;
    }

   /* ------------------------------------------------------------------
    *    Get RXBYTES
    *   -------------
    */

   /*
    *  Read the RXBYTES register from the radio.
    *  Bit description of RXBYTES register:
    *    bit 7     - RXFIFO_OVERFLOW, set if receive overflow occurred
    *    bits 6:0  - NUM_BYTES, number of bytes in receive FIFO
    *
    *  Due a chip bug, the RXBYTES register must read the same value twice
    *  in a row to guarantee an accurate value.
    */
    {
       uint8_t rxBytesVerify;

       rxBytes = wbsl_SpiReadReg( RXBYTES );

       do
       {
           rxBytes = rxBytesVerify;
           rxBytesVerify = wbsl_SpiReadReg( RXBYTES ); //NUM_RXBYTES
       }while (rxBytes != rxBytesVerify);

     }

     /* ------------------------------------------------------------------
      *    FIFO empty?
      *   -------------
      */

     /*
      *  See if the receive FIFIO is empty before attempting to read from it.
      *  It is possible nothing the FIFO is empty even though the interrupt fired.
      *  This can happen if address check is enabled and a non-matching packet is
      *  received.  In that case, the radio automatically removes the packet from
      *  the FIFO.
      */
      if (rxBytes == 0)
      {
          /* receive FIFO is empty - do nothing, skip to end */
      }
      else
      {
	      /* receive FIFO is not empty, continue processing */

	      /* ------------------------------------------------------------------
	       *    Process frame length
	       *   ----------------------
	       */
	      /* get packet from FIFO */
	      wbsl_SpiReadRxFifo(IncomingPacket, rxBytes);


	      /*
	       *  Make sure that the frame length just read corresponds to number of bytes in the buffer.
	       *  If these do not match up something is wrong.
	       *
	       *  This can happen for several reasons:
	       *   1) Incoming packet has an incorrect format or is corrupted.
	       *   2) The receive FIFO overflowed.  Overflow is indicated by the high
	       *      bit of rxBytes.  This guarantees the value of rxBytes value will not
	       *      match the number of bytes in the FIFO for overflow condition.
	       *   3) Interrupts were blocked for an abnormally long time which
	       *      allowed a following packet to at least start filling the
	       *      receive FIFO.  In this case, all received and partially received
	       *      packets will be lost - the packet in the FIFO and the packet coming in.
	       *      This is the price the user pays if they implement a giant
	       *      critical section.
	       *   4) A failed transmit forced radio to IDLE state to flush the transmit FIFO.
	       *      This could cause an active receive to be cut short.
	       *
	       *  Also check the sanity of the length to guard against rogue frames.
	       */
	      if ((rxBytes>WBSL_TOTAL_LENGTH + WBSL_RX_METRICS_SIZE) ||
	    		 (rxBytes != (IncomingPacket[WBSL_LENGTH_FIELD_OFS] + WBSL_LENGTH_FIELD_SIZE + WBSL_RX_METRICS_SIZE)))
	      {
	          bspIState_t s;

	          /* mismatch between bytes-in-FIFO and frame length */

	          /*
	           *  Flush receive FIFO to reset receive.  Must go to IDLE state to do this.
	           *  The critical section guarantees a transmit does not occur while cleaning up.
	           */
	           BSP_ENTER_CRITICAL_SECTION(s);
	           WBSL_STROBE_IDLE_AND_WAIT();
	           wbsl_SpiCmdStrobe( SFRX );
	           wbsl_SpiCmdStrobe( SRX );
	           BSP_EXIT_CRITICAL_SECTION(s);

	           /* flush complete, skip to end */
	      }
	      else
	      {
	          /* bytes-in-FIFO and frame length match up - continue processing */

	          /* ------------------------------------------------------------------
	           *    CRC check
	           *   ------------
	           */

	          /*
	           *  Note!  Automatic CRC check is not, and must not, be enabled.  This feature
	           *  flushes the *entire* receive FIFO when CRC fails.  If this feature is
	           *  enabled it is possible to be reading from the FIFO and have a second
	           *  receive occur that fails CRC and automatically flushes the receive FIFO.
	           *  This could cause reads from an empty receive FIFO which puts the radio
	           *  into an undefined state.
	           */

	           /* determine if CRC failed */
	    	   /* CRC Status is appended as the MSB of the 2nd byte of the status bytes */
	          if (!(IncomingPacket[IncomingPacket[WBSL_LENGTH_FIELD_OFS] + WBSL_CRC_STATUS_OFFSET] & CRC_STATUS))
	          {
	              /* CRC failed - do nothing, skip to end */

	          }
	          else
	          {
	              /* CRC passed - continue processing */
	              /* ------------------------------------------------------------------
	               *    Receive successful
	               *   --------------------
	               */

	    	       memcpy(RxBuffer, IncomingPacket, rxBytes);
	    	       rxtx_flag = WBSL_RXTX_RECEIVED;

	           }

	        }
	  }
   }



// *************************************************************************************************
// @fn          wbsl_resetTimer
// @brief       Resets the timer TA0 and stops it
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_resetTimer(void)
{
	/* Reset IRQ flag */
    TA0CCTL1 &= ~CCIFG;

	/* Stop Timer0 */
	TA0CTL &= ~(MC1 + MC0);

	/* Set Timer0 count register to 0x0000 */
    TA0R = 0;
}



// *************************************************************************************************
// @fn          wbsl_setTimer
// @brief       Set the timer for the Packet timeouts. the timeout param is in ticks 1 tick = 1/32768 sec

// @param       u16  timeout. this value must be less than 32768 if greater than 32768 it is set to 32767

// @return      none
// *************************************************************************************************
void wbsl_setTimer(u16 timeout)
{
    if (timeout > 32767)
	    timeout = 32767;

	/* Update CCR */
	TA0CCR1 = timeout;

	/* Reset IRQ flag */
	TA0CCTL1 &= ~CCIFG;

	  /* Clear and start timer now
	   * continuous mode: Count to 0xFFFF and restart from 0 again - 1sec timing will be generated by
	   *  ISR
	   */
	TA0CTL  |= TASSEL0 + MC1 + TACLR;
}



// *************************************************************************************************
// @fn          rxModeOn
// @brief       Puts the radio in RX Mode and enables the needed interrupts
// @param       none
// @return      none
// *************************************************************************************************
void rxModeOn(void)
{
    /* clear any residual receive interrupt */
    WBSL_CLEAR_SYNC_PIN_INT_FLAG();

   /* send strobe to enter receive mode */
    wbsl_SpiCmdStrobe( SRX );

   /* Set Mode to RX */
    wbsl_rxtxMode = WBSL_RX_MODE;

    /* enable receive interrupts */
    WBSL_ENABLE_SYNC_PIN_INT();
}


// *************************************************************************************************
// @fn          rxModeOff
// @brief       Puts the radio in Idle Mode and disables RF Interrupts
// @param       none
// @return      none
// *************************************************************************************************
void rxModeOff(void)
{
    /*disable receive interrupts */
    WBSL_DISABLE_SYNC_PIN_INT();

	/* turn off radio */
	WBSL_STROBE_IDLE_AND_WAIT();

     /* Set Mode to IDLE */
	wbsl_rxtxMode = WBSL_IDLE_MODE;

	/* flush the receive FIFO of any residual data */
	wbsl_SpiCmdStrobe( SFRX );

	/* clear receive interrupt */
	WBSL_CLEAR_SYNC_PIN_INT_FLAG();
}


// *************************************************************************************************
// @fn          wbsl_transmit
// @brief       Sets all the needed values and puts the Radio in Tx Mode
//              to transmit the Buffer received, either in CCA Mode or in Force Mode.
// @param       u8* src        The Buffer which is going to be sent through RF
//              u8 size        The size of the buffer received
// @return      none
// *************************************************************************************************
u8 wbsl_transmit(u8 *src, u8 size)
{
    u8 retValue = WBSL_TX_RES_SUCCESS;

   /* Turn off receiver. We can ignore/drop incoming packets during transmit. */
    rxModeOff();

    /*------------------------------------------------------------------
     *    Write packet to transmit FIFO
     * --------------------------------
     */
    wbsl_SpiWriteTxFifo(src, size);


    #ifdef CCA_MODE
        u8 ccaRetries;
    #endif

    #ifdef CCA_MODE
        ccaRetries = 4;

        /* ===============================================================================
         *  CCA Loop
         *  =============
         */

        /* For CCA algorithm, we need to know the transition from the RX state to
         * the TX state. There is no need for SYNC signal in this logic. So we
         * can re-configure the GDO_0 output from the radio to be PA_PD signal
         * instead of the SYNC signal.
         * Since both SYNC and PA_PD are used as falling edge interrupts, we
         * don't need to reconfigure the MCU input.
         */

    WBSL_CONFIG_GDO0_AS_PAPD_SIGNAL();

    while(1)
    {
	      /* Radio must be in RX mode for CCA to happen.
	       * Otherwise it will transmit without CCA happening.
	       */

	     wbsl_SpiCmdStrobe( SRX );

	     /* wait for the rssi to be valid. */
	     WBSL_RSSI_VALID_WAIT();

	      /*
	       *  Clear the PA_PD pin interrupt flag.  This flag, not the interrupt itself,
	       *  is used to capture the transition that indicates a transmit was started.
	       *  The pin level cannot be used to indicate transmit success as timing may
	       *  prevent the transition from being detected.  The interrupt latch captures
	       *  the event regardless of timing.
	       */
	     WBSL_CLEAR_PAPD_PIN_INT_FLAG();

	      /* send strobe to initiate transmit */
	     wbsl_SpiCmdStrobe( STX );

	     /* Delay long enough for the PA_PD signal to indicate a successful transmit.
	      * Found out that we need a delay of at least 100 us on CC1101 to see
	      * the PA_PD signal change with respect to the radio configuration for WBSL.
	      */
	     wbsl_DelayUsec(110);


	      /* PA_PD signal goes from HIGH to LOW when going from RX state.
	       * This transition is trapped as a falling edge interrupt flag
	       * to indicate that CCA passed and the transmit has started.
	       */
	     if (WBSL_PAPD_INT_FLAG_IS_SET())
	     {
	         /* ------------------------------------------------------------------
	          *    Clear Channel Assessment passed.
	          *   ----------------------------------
	          */

	         /* Clear the PA_PD int flag */
	         WBSL_CLEAR_PAPD_PIN_INT_FLAG();

	         /* PA_PD signal stays LOW while in TX state and goes back to HIGH when
	          * the radio transitions to RX state.
	          */
	          /* wait for transmit to complete */
	         while (!WBSL_PAPD_PIN_IS_HIGH());

	          /* transmit done, break */
	         break;
	      }
	      else
	      {
	          /* ------------------------------------------------------------------
	           *    Clear Channel Assessment failed.
	           *   ----------------------------------
	           */

	          /* Turn off radio and save some power during backoff */

	          WBSL_STROBE_IDLE_AND_WAIT();

	          /* flush the receive FIFO of any residual data */
	          wbsl_SpiCmdStrobe( SFRX );

	          /* Retry ? */
	          if (ccaRetries != 0)
	          {
	              /* Back off delay */
	              wbsl_setTimer(CONV_MS_TO_TICKS(3));
	        	  /* Wait for the Timeout */
	              while(!(TA0CCTL1 & CCIFG));
	        	  /* Reset timer so that next time it starts fresh */
	        	  wbsl_resetTimer();
	              /* decrement CCA retries before loop continues */
	              ccaRetries--;
	          }
	          else /* No CCA retries are left, abort */
	          {
	              /* set return value for failed transmit and break */
	              retValue = WBSL_TX_RES_FAILED;
	              break;
	           }
	        } /* CCA Failed */
	     /* CCA loop */
       }

   /* Restore GDO_0 to be SYNC signal */
    WBSL_CONFIG_GDO0_AS_SYNC_SIGNAL();
    WBSL_SPI_ASSERT( wbsl_SpiReadReg( IOCFG0 ) == WBSL_GDO_SYNC );

#else


    /* ------------------------------------------------------------------
     *    Immediate transmit
     * ---------------------
     */

   /* Issue the TX strobe. */


    wbsl_SpiCmdStrobe( STX );

    /* Wait for transmit to complete */
    while(!WBSL_SYNC_PIN_INT_FLAG_IS_SET());

    /* Clear the interrupt flag */
    WBSL_CLEAR_SYNC_PIN_INT_FLAG();


#endif

   /* Done with TX. Clean up time... */

    /* Radio is already in IDLE state */

     /*
      * Flush the transmit FIFO.  It must be flushed so that
      * the next transmit can start with a clean slate.
      */
   wbsl_SpiCmdStrobe( SFTX );

   return (retValue);
}


// *************************************************************************************************
// @fn          wbsl_link
// @brief       Checks if a discovery packet has been received, checks the integrity of it
//              and sends the discovery ACK
// @param       none
// @return      u8 status
//              WBSL_LINK_FAIL    No discovery packet was received
//              WBSL_LINK_SUCC    Discovery packet received and ACK sent
// *************************************************************************************************
u8 wbsl_link(void)
{
    u8 status = WBSL_LINK_FAIL;
	u8 i = 0;
	u8 validPacket = 0;

	/* Check if packet was received */
	if (rxtx_flag == WBSL_RXTX_RECEIVED)
	{
	    /* Clear RX Flag */
	    rxtx_flag = 0;

        /* Check that packet was a broadcast packet and that the CRC Status is Ok */
	    if ((RxBuffer[AP_ADDRESS_OFFSET_RX] == 0))
	    {
	        /* Check if packet has the discovery Payload */
	        validPacket = 1;
	        for (i=0; i<DISCOVERY_PAYLOAD_LENGTH; i++)
	        {
	            if (RxBuffer[i+4] != discoveryPayload[i+3]) validPacket = 0;
	        }

	        /* If packet O.K., save device address and send ACK */
	       if (validPacket)
	       {
	        /* Save Watch Address to later direct all packets to this watch */
	           ed_address = RxBuffer[ED_ADDRESS_OFFSET_RX];

	          /* Address the discovery packet to the Watch */
	           discoveryAck[ED_ADDRESS_OFFSET_TX] = ed_address;

	          /* Positive ACK of the discovery */
	           discoveryAck[DISCOVERY_ACK_OFFSET] = WBSL_LINK_SUCC;

	           /* Give time to watch to be on RX Mode */
	           wbsl_setTimer(TIMEOUT_FOR_RXWATCH);
	           /* Wait for the Timeout */
	           while(!(TA0CCTL1 & CCIFG));
	           /* Reset timer so that next time it starts fresh */
	           wbsl_resetTimer();


	           TX_ACTIVITY_ON;
	           wbsl_transmit(discoveryAck, sizeof(discoveryAck)); /* Send discovery ACK to Watch */
	           TX_ACTIVITY_OFF;

	           status = WBSL_LINK_SUCC;
	        }
	     }
	 }

	   return status;
}


// *************************************************************************************************
// @fn          sendInitPacket
// @brief       Send the Init packet to the Watch, which contains the total packets to be received
//              during the communication, it also waits for the ACK of the Init packet, if it is not
//              received this function will be called again by the main process
// @param       none
// @return      none
// *************************************************************************************************
void sendInitPacket(void)
{
    /* Initialize the INIT packet to be sent to the newly paired device */
    initPacket[ED_ADDRESS_OFFSET_TX] = ed_address;

    initPacket[5] = wbsl_number_of_retries;

    wbsl_setTimer(TIMEOUT_FOR_ACK);

    /*Wait until packet is ready to be sent or timeout */

    while(packet_ready_flag != WBSL_PACKET_FULL && !(TA0CCTL1 & CCIFG));


    /* Check if the timeout happened before the packet was full,
     * if so, trigger the update to stop, there is an error in communication between
     * GUI and Dongle has occurred or a manual stop has been triggered
     */
    if (packet_ready_flag != WBSL_PACKET_FULL)
    {
        setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
        return;
    }

    /* Reset timer so that next time it starts fresh */
    wbsl_resetTimer();


    TX_ACTIVITY_ON;          /* Mark Initiate Transmit */
    /* Send packet to ED */
    wbsl_transmit(initPacket, sizeof( initPacket));
    TX_ACTIVITY_OFF;   /* Turn off led after packet sent */
    rxModeOn();    /* Turn on RX Mode to wait for the Discovery ACK package */

     /* Set timeout to receive */
    wbsl_setTimer(TIMEOUT_FOR_ACK);
    /* Wait for either the Timeout or packet received flag */
    while(!(TA0CCTL1 & CCIFG) && rxtx_flag != WBSL_RXTX_RECEIVED);

    /* Reset timer so that next time it starts fresh */
    wbsl_resetTimer();

   /* Radio off during decoding */
    rxModeOff();

    /* Check if packet was received */
    if (rxtx_flag == WBSL_RXTX_RECEIVED)
    {
        /* Clear RX Flag */
        rxtx_flag = 0;
        if (RxBuffer[AP_ADDRESS_OFFSET_RX] == WBSL_AP_ADDRESS &&
            RxBuffer[ED_ADDRESS_OFFSET_RX] == ed_address &&
            RxBuffer[ED_ADDRESS_OFFSET_RX + 1] == WBSL_ACK_SUCC)
         {
             /* Reset the retry counter for ACKs */
             wbsl_number_of_retries = 0;
             initOk = 1;

             /* Flag that the buffer is ready to be filled again */
             packet_ready_flag = WBSL_PACKET_EMPTY;
             /* Trigger GUI to send first data packet */
             wbsl_packet_flag = WBSL_SEND_NEW_DATA_PACKET;
         }
   }
}


// *************************************************************************************************
// @fn          sendDataPacket
// @brief       Send the next data packet to the watch and waits for the ACK, if received it triggers
//              the GUI to send the next packet to be sent to the watch, if not, the same packet will
//              be sent again the next time this function is called
// @param       none
// @return      none
// *************************************************************************************************
void sendDataPacket(void)
{
    /* Change the needed fields for the TX Buffer */
    TxBuffer[CURRENT_PACKET_NR_OFFSET] = (currentPacket >> 8) & 0xFF;
    TxBuffer[CURRENT_PACKET_NR_OFFSET + 1] = currentPacket & 0xFF;

   /* Address the packet to the Watch we're synched to */
    TxBuffer[ED_ADDRESS_OFFSET_TX] = ed_address;

    wbsl_setTimer(TIMEOUT_FOR_ACK);

   /* Wait until packet is ready to be sent or timeout */

    while(packet_ready_flag != WBSL_PACKET_FULL && !(TA0CCTL1 & CCIFG));


   /* Check if the timeout happened before the packet was full,
    * if so, trigger the update to stop, there is an error in communication between
    * GUI and Dongle has occurred or a manual stop has been triggered
    */
    if (packet_ready_flag != WBSL_PACKET_FULL)
    {
        setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
        return;
     }

    /* Reset timer so that next time it starts fresh */
    wbsl_resetTimer();

    /* Send packet to ED */
    TX_ACTIVITY_ON;
    wbsl_transmit(TxBuffer,sizeof(TxBuffer));
    TX_ACTIVITY_OFF;     /* Transmit Complete */

    rxModeOn();   /* Turn on RX Mode to wait for the ACK package */

    /* Set timeout to receive ACK */
    wbsl_setTimer(TIMEOUT_FOR_ACK);
    /* Wait for either the Timeout or packet received flag */
    while(!(TA0CCTL1 & CCIFG) && rxtx_flag != WBSL_RXTX_RECEIVED);

    /* Radio off during decoding */
    rxModeOff();

    /* Reset timer so that next time it starts fresh */
    wbsl_resetTimer();

    /* Check if packet was received and check for status fields */
    if (rxtx_flag == WBSL_RXTX_RECEIVED)
    {
        /* Clear RX Flag */
        rxtx_flag = 0;
        if (RxBuffer[AP_ADDRESS_OFFSET_RX] == WBSL_AP_ADDRESS &&
            RxBuffer[ED_ADDRESS_OFFSET_RX] == ed_address &&
            RxBuffer[ED_ADDRESS_OFFSET_RX + 1] == WBSL_ACK_SUCC &&
            RxBuffer[ED_ADDRESS_OFFSET_RX + 2] == ((currentPacket >> 8) &  0x7F) &&
            RxBuffer[ED_ADDRESS_OFFSET_RX + 3] ==  (currentPacket & 0xFF))
        {
            /* Reset the retry counter for ACKs */
            wbsl_number_of_retries = 0;
            currentPacket++;
            wbsl_status = (currentPacket*100)/total_packets;
            if(wbsl_status==100) update_complete=1;
            /* Trigger GUI to send next data packet */
            wbsl_packet_flag = WBSL_SEND_NEW_DATA_PACKET;

            /* Flag that the buffer is ready to be filled again */
            packet_ready_flag = WBSL_PACKET_EMPTY;
        }
     }
}


// *************************************************************************************************
// @fn          wbsl_sendPacket
// @brief       This function handles the sending of the different packets that conform the whole
//              firmware download, it checks if a packet is ready to be sent and sends it, it also waits
//              for the ACK of the packet or a timeout, in the latter case when called again, it will send
//              the same packet.
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_sendPacket(void)
{
    /* Increment the number of retries to send a packet */
    wbsl_number_of_retries++;

    /* Check if too many retries for one packet have been already made */
    if (wbsl_number_of_retries > WBSL_MAXIMUM_RETRIES)
    {
        /* Trigger the stop of the WBSL Update procedure */
        setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
        setFlag(wbsl_flag, WBSL_STATUS_ERROR);
        return;
    }

    if (!initOk)
    {
      /* Send the init packet */
        sendInitPacket();
    }
    else if(currentPacket < total_packets)
    {
      /* Send a regular data packet */
        sendDataPacket();
    }
}


// *************************************************************************************************
// @fn          wbsl_reset
// @brief       Reset variables needed for the WBSL
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_reset(void)
{
    wbsl_number_of_retries = 0;
    wbsl_status = 0;
    wbsl_flag = WBSL_STATUS_LINKING;
   /* Initialize the packet status flag so that GUI knows when it needs to send next packet */
    wbsl_packet_flag = WBSL_DISABLED;
    currentPacket = 0;
    update_complete = 0;
    total_packets = 0;
    packet_ready_flag = WBSL_PACKET_EMPTY;
    initOk = 0;
    //clear the RX buffer
    memset(&RxBuffer, 0x00, sizeof(RxBuffer));
    /* Make sure TimerA0 Interrupts Request are disabled*/
    TA0CTL &= ~TAIE;
    TA0CCTL1 &= ~CCIE;
    /* Initialize the RX Flag */
    rxtx_flag = 0;
}


// *************************************************************************************************
// @fn          wbsl_main
// @brief       This is the main routine, which calls the Link function, if successful, it keeps calling
//              the sendPacket function until all packages has been sent, it also checks for the wbsl_flag
//              in case the procedure is stopped before completing the download, either manually or due to an
//              error
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_main(void)
{
    /* Initialize used variables */
    wbsl_reset();

    /* Init TX Buffer */
    TxBuffer[0] = sizeof(TxBuffer) - 1;  /* Substract the length field (1 byte) */
    TxBuffer[AP_ADDRESS_OFFSET_TX] = WBSL_AP_ADDRESS; /* Include my address in all packets */

    /* LED off */
    LED_OFF;

    wNumCurrentPeers = 0;

    wbsl_status = 0;

    /* Make sure radio is in IDLE Mode before starting */
    rxModeOff();

    while (!getFlag(wbsl_flag,WBSL_TRIGGER_STOP))
    {

	    if (!wNumCurrentPeers)
        {
    	    /* Turn on RX Mode to wait for the Discovery package */
    	    if (wbsl_rxtxMode != WBSL_RX_MODE)
    	        rxModeOn();

             /* First try pairing with an End Device */
            if (wbsl_link() == WBSL_LINK_SUCC)
            {
                wbsl_flag = WBSL_STATUS_LINKED;
                /* Keep track of how many peers are connected */
                wNumCurrentPeers++;
                /* Trigger GUI to send first info packet (total bytes of file) */
                wbsl_packet_flag = WBSL_SEND_INFO_PACKET;
                rxModeOff();    /* Turn off RX Mode after successfully linking with ED */
            }
         }

        /* Keep downloading update image */
        if ((currentPacket < total_packets || !initOk) && (wNumCurrentPeers > 0))
        {
            wbsl_sendPacket();    /* Send the packet */

            /* Delay to give time to watch to be in RX after sending ACK */
            wbsl_setTimer(TIMEOUT_FOR_RXWATCH);
            /* Wait for the Timeout */
            while(!(TA0CCTL1 & CCIFG));
            /* Reset timer so that next time it starts fresh */
            wbsl_resetTimer();
         }

         /* If the update has complete, finish the Transmission and turn off WBSL */
       if (update_complete)
       {
           setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
       }

     }

     /* If GUI or Watch has triggered to stop WBSL, do so, but leave everything in a workable state. */
    reset_RF_registers();
     /* Put radio in Idle mode */
    rxModeOff();

    wNumCurrentPeers = 0;
    sInitWbslDone=0; /* Restart RF configuration for WBSL next time */
   /* Set timeout to receive ACK
    *This is to allow the GUI to read the last progress status
    *This  in case it reached 100
    */
    wbsl_setTimer(TIMEOUT_FOR_ACK);
    while (!(TA0CCTL1 & CCIFG));
    /* Reset timer before exiting */
    wbsl_resetTimer();
}

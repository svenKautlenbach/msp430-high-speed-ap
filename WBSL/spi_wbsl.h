// *************************************************************************************************
//
//	Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
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

// *************************************************************************************************
// Header for Functions handling SPI communications with TI's CC1101 during rFBSL.
// *************************************************************************************************

#ifndef SPI_WBSL_H_
#define SPI_WBSL_H_

#ifdef __cplusplus
extern "C" {
#endif

// *************************************************************************************************
// Include section

#include "project.h"
#include "bsp.h"

// *************************************************************************************************
// Defines section

/* ------------------------------------------------------------------------------------------------
 *          SPI ADDRESS SPACE
 * ------------------------------------------------------------------------------------------------
 */


/* status registers */
#define PARTNUM     0x30      /*  PARTNUM    - Chip ID */
#define VERSION     0x31      /*  VERSION    - Chip ID */
#define FREQEST     0x32      /*  FREQEST    – Frequency Offset Estimate from demodulator */
#define LQI         0x33      /*  LQI        – Demodulator estimate for Link Quality */
#define RSSI        0x34      /*  RSSI       – Received signal strength indication */
#define MARCSTATE   0x35      /*  MARCSTATE  – Main Radio Control State Machine state */
#define WORTIME1    0x36      /*  WORTIME1   – High byte of WOR time */
#define WORTIME0    0x37      /*  WORTIME0   – Low byte of WOR time */
#define PKTSTATUS   0x38      /*  PKTSTATUS  – Current GDOx status and packet status */
#define VCO_VC_DAC  0x39      /*  VCO_VC_DAC – Current setting from PLL calibration module */
#define TXBYTES     0x3A      /*  TXBYTES    – Underflow and number of bytes */
#define RXBYTES     0x3B      /*  RXBYTES    – Overflow and number of bytes */

/* burst write registers */
#define PA_TABLE0   0x3E      /*  PA_TABLE0 - PA control settings table */
#define TXFIFO      0x3F      /*  TXFIFO  - Transmit FIFO */
#define RXFIFO      0x3F      /*  RXFIFO  - Receive FIFO */

/* command strobe registers */
#define SRES        0x30      /*  SRES    - Reset chip. */
#define SFSTXON     0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define SXOFF       0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define SCAL        0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define SRX         0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define STX         0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define SIDLE       0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define SRSVD       0x37      /*  SRVSD   - Reserved.  Do not use. */
#define SWOR        0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define SPWD        0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define SFRX        0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define SFTX        0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define SWORRST     0x3C      /*  SWORRST - Reset real time clock. */
#define SNOP        0x3D      /*  SNOP    - No operation. Returns status byte. */


/* RF registers */
#define IOCFG2      0x00      /*  IOCFG2   - GDO2 output pin configuration  */
#define IOCFG1      0x01      /*  IOCFG1   - GDO1 output pin configuration  */
#define IOCFG0      0x02      /*  IOCFG1   - GDO0 output pin configuration  */
#define FIFOTHR     0x03      /*  FIFOTHR  - RX FIFO and TX FIFO thresholds */
#define SYNC1       0x04      /*  SYNC1    - Sync word, high byte */
#define SYNC0       0x05      /*  SYNC0    - Sync word, low byte */
#define PKTLEN      0x06      /*  PKTLEN   - Packet length */
#define PKTCTRL1    0x07      /*  PKTCTRL1 - Packet automation control */
#define PKTCTRL0    0x08      /*  PKTCTRL0 - Packet automation control */
#define ADDR        0x09      /*  ADDR     - Device address */
#define CHANNR      0x0A      /*  CHANNR   - Channel number */
#define FSCTRL1     0x0B      /*  FSCTRL1  - Frequency synthesizer control */
#define FSCTRL0     0x0C      /*  FSCTRL0  - Frequency synthesizer control */
#define FREQ2       0x0D      /*  FREQ2    - Frequency control word, high byte */
#define FREQ1       0x0E      /*  FREQ1    - Frequency control word, middle byte */
#define FREQ0       0x0F      /*  FREQ0    - Frequency control word, low byte */
#define MDMCFG4     0x10      /*  MDMCFG4  - Modem configuration */
#define MDMCFG3     0x11      /*  MDMCFG3  - Modem configuration */
#define MDMCFG2     0x12      /*  MDMCFG2  - Modem configuration */
#define MDMCFG1     0x13      /*  MDMCFG1  - Modem configuration */
#define MDMCFG0     0x14      /*  MDMCFG0  - Modem configuration */
#define DEVIATN     0x15      /*  DEVIATN  - Modem deviation setting */
#define MCSM2       0x16      /*  MCSM2    - Main Radio Control State Machine configuration */
#define MCSM1       0x17      /*  MCSM1    - Main Radio Control State Machine configuration */
#define MCSM0       0x18      /*  MCSM0    - Main Radio Control State Machine configuration */
#define FOCCFG      0x19      /*  FOCCFG   - Frequency Offset Compensation configuration */
#define BSCFG       0x1A      /*  BSCFG    - Bit Synchronization configuration */
#define AGCCTRL2    0x1B      /*  AGCCTRL2 - AGC control */
#define AGCCTRL1    0x1C      /*  AGCCTRL1 - AGC control */
#define AGCCTRL0    0x1D      /*  AGCCTRL0 - AGC control */
#define WOREVT1     0x1E      /*  WOREVT1  - High byte Event0 timeout */
#define WOREVT0     0x1F      /*  WOREVT0  - Low byte Event0 timeout */
#define WORCTRL     0x20      /*  WORCTRL  - Wake On Radio control */
#define FREND1      0x21      /*  FREND1   - Front end RX configuration */
#define FREND0      0x22      /*  FREDN0   - Front end TX configuration */
#define FSCAL3      0x23      /*  FSCAL3   - Frequency synthesizer calibration */
#define FSCAL2      0x24      /*  FSCAL2   - Frequency synthesizer calibration */
#define FSCAL1      0x25      /*  FSCAL1   - Frequency synthesizer calibration */
#define FSCAL0      0x26      /*  FSCAL0   - Frequency synthesizer calibration */
#define RCCTRL1     0x27      /*  RCCTRL1  - RC oscillator configuration */
#define RCCTRL0     0x28      /*  RCCTRL0  - RC oscillator configuration */
#define FSTEST      0x29      /*  FSTEST   - Frequency synthesizer calibration control */
#define PTEST       0x2A      /*  PTEST    - Production test */
#define AGCTEST     0x2B      /*  AGCTEST  - AGC test */
#define TEST2       0x2C      /*  TEST2    - Various test settings */
#define TEST1       0x2D      /*  TEST1    - Various test settings */
#define TEST0       0x2E      /*  TEST0    - Various test settings */

/* ------------------------------------------------------------------------------------------------
 *               MACROS
 * ------------------------------------------------------------------------------------------------
 */


/* GDO functionality */

#define WBSL_GDO_SYNC     6
#define WBSL_GDO_PA_PD    27  /* low when transmit is active, low during sleep */


#define WBSL_SPI_DEBUG
#ifdef WBSL_SPI_DEBUG
#define WBSL_SPI_ASSERT(x)      BSP_ASSERT(x)
#else
#define WBSL_SPI_ASSERT(x)
#endif

#define __wbsl_SPI_CSN_GPIO_BIT__             0

#define WBSL_SPI_TURN_CHIP_SELECT_ON()       st( P4OUT &= ~BV(__wbsl_SPI_CSN_GPIO_BIT__); )
#define WBSL_SPI_TURN_CHIP_SELECT_OFF()      st( P4OUT |=  BV(__wbsl_SPI_CSN_GPIO_BIT__); )

 //Here we configure the pins that control the CC1101
#define __wbsl_GDO0_BIT__                     0
#define WBSL_CONFIG_GDO0_PIN_AS_INPUT()       st( P2SEL &= ~BV(__wbsl_GDO0_BIT__); ) /* clear pin special function default */
#define WBSL_GDO0_PIN_IS_HIGH()               (P2IN & BV(__wbsl_GDO0_BIT__))

/* CSn Pin Configuration */
#define __wbsl_SPI_CSN_GPIO_BIT__             0
#define WBSL_SPI_CONFIG_CSN_PIN_AS_OUTPUT()   st( P4DIR |=  BV(__wbsl_SPI_CSN_GPIO_BIT__); )
#define WBSL_SPI_DRIVE_CSN_HIGH()             st( P4OUT |=  BV(__wbsl_SPI_CSN_GPIO_BIT__); ) /* atomic operation */
#define WBSL_SPI_DRIVE_CSN_LOW()              st( P4OUT &= ~BV(__wbsl_SPI_CSN_GPIO_BIT__); ) /* atomic operation */
#define WBSL_SPI_CSN_IS_HIGH()                (  P4OUT &   BV(__wbsl_SPI_CSN_GPIO_BIT__) )

/* SCLK Pin Configuration */
#define __wbsl_SPI_SCLK_GPIO_BIT__            3
#define WBSL_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()  st( P4DIR |=  BV(__wbsl_SPI_SCLK_GPIO_BIT__); )
#define WBSL_SPI_DRIVE_SCLK_HIGH()            st( P4OUT |=  BV(__wbsl_SPI_SCLK_GPIO_BIT__); )
#define WBSL_SPI_DRIVE_SCLK_LOW()             st( P4OUT &= ~BV(__wbsl_SPI_SCLK_GPIO_BIT__); )

/* SI Pin Configuration */
#define __wbsl_SPI_SI_GPIO_BIT__              1
#define WBSL_SPI_CONFIG_SI_PIN_AS_OUTPUT()    st( P4DIR |=  BV(__wbsl_SPI_SI_GPIO_BIT__); )
#define WBSL_SPI_DRIVE_SI_HIGH()              st( P4OUT |=  BV(__wbsl_SPI_SI_GPIO_BIT__); )
#define WBSL_SPI_DRIVE_SI_LOW()               st( P4OUT &= ~BV(__wbsl_SPI_SI_GPIO_BIT__); )

/* SO Pin Configuration */
#define __wbsl_SPI_SO_GPIO_BIT__              2
#define WBSL_SPI_CONFIG_SO_PIN_AS_INPUT()     /* nothing to required */
#define WBSL_SPI_SO_IS_HIGH()                 ( P4IN & BV(__wbsl_SPI_SO_GPIO_BIT__) )

/* SPI Port Configuration */
#define WBSL_SPI_CONFIG_PORT()                st( P4SEL |= BV(__wbsl_SPI_SCLK_GPIO_BIT__) |  \
                                                           BV(__wbsl_SPI_SI_GPIO_BIT__)   |  \
                                                           BV(__wbsl_SPI_SO_GPIO_BIT__); )

#define WBSL_SYNC_PIN_IS_HIGH()                     (P2IN & BV(__wbsl_GDO0_BIT__))
#define WBSL_ENABLE_SYNC_PIN_INT()                  st( P2IE  |=  BV(__wbsl_GDO0_BIT__); )
#define WBSL_DISABLE_SYNC_PIN_INT()                 st( P2IE  &= ~BV(__wbsl_GDO0_BIT__); )
#define WBSL_SYNC_PIN_INT_IS_ENABLED()              (  P2IE  &   BV(__wbsl_GDO0_BIT__) )
#define WBSL_CLEAR_SYNC_PIN_INT_FLAG()              st( P2IFG &= ~BV(__wbsl_GDO0_BIT__); )
#define WBSL_SYNC_PIN_INT_FLAG_IS_SET()             (  P2IFG &   BV(__wbsl_GDO0_BIT__) )
#define WBSL_CONFIG_SYNC_PIN_FALLING_EDGE_INT()     st( P2IES |=  BV(__wbsl_GDO0_BIT__); )

#define WBSL_PAPD_PIN_IS_HIGH()                     WBSL_SYNC_PIN_IS_HIGH()
#define WBSL_CLEAR_PAPD_PIN_INT_FLAG()              WBSL_CLEAR_SYNC_PIN_INT_FLAG()
#define WBSL_PAPD_INT_FLAG_IS_SET()                 WBSL_SYNC_PIN_INT_FLAG_IS_SET()
#define WBSL_CONFIG_PAPD_FALLING_EDGE_INT()         WBSL_CONFIG_SYNC_PIN_FALLING_EDGE_INT()

#define WBSL_CONFIG_GDO0_AS_PAPD_SIGNAL()           wbsl_SpiWriteReg(IOCFG0, WBSL_GDO_PA_PD)
#define WBSL_CONFIG_GDO0_AS_SYNC_SIGNAL()           wbsl_SpiWriteReg(IOCFG0, WBSL_GDO_SYNC)

#define WBSL_SPI_IS_INITIALIZED()                   (UCB1CTL0 & UCMST)
#define WBSL_SPI_WRITE_BYTE(x)                      st( UCB1ICTL_H &= ~UCRXIFG;  UCB1TXBUF = x; )
#define WBSL_SPI_READ_BYTE()                        UCB1RXBUF
#define WBSL_SPI_WAIT_DONE()                        while(!(UCB1ICTL_H & UCRXIFG));


/* SPI critical section macros */
typedef bspIState_t wbslSpiIState_t;
#define WBSL_SPI_ENTER_CRITICAL_SECTION(x)          BSP_ENTER_CRITICAL_SECTION(x)
#define WBSL_SPI_EXIT_CRITICAL_SECTION(x)           BSP_EXIT_CRITICAL_SECTION(x)

#define DUMMY_BYTE                  0xDB
#define READ_BIT                    0x80
#define BURST_BIT                   0x40

/*
   *  Radio SPI Specifications
   * -----------------------------------------------
   *    Max SPI Clock   :  10 MHz
   *    Data Order      :  MSB transmitted first
   *    Clock Polarity  :  low when idle
   *    Clock Phase     :  sample leading edge
   */

  /* initialization macro */
#define WBSL_SPI_INIT() \
st ( \
  UCB1CTL1 = UCSWRST;                           \
  UCB1CTL1 = UCSWRST | UCSSEL1;                 \
  UCB1CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;   \
  UCB1BR0  = 2;                                 \
  UCB1BR1  = 0;                                 \
  WBSL_SPI_CONFIG_PORT();                       \
  UCB1CTL1 &= ~UCSWRST;                         \
)



// *************************************************************************************************
// Function prototype section

void wbsl_SpiInit(void);
void wbsl_SpiWriteReg(u8 addr, u8 value);
void wbsl_SpiWriteTxFifo(u8 * pWriteData, u8 len);
void wbsl_SpiReadRxFifo(u8 * pReadData, u8 len);
u8 wbsl_SpiCmdStrobe(u8 addr);
u8 wbsl_SpiReadReg(u8 addr);

#ifdef __cplusplus
}
#endif

#endif /* SPI_WBSL_H_ */

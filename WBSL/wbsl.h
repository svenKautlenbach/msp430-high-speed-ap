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

#ifndef __WBSL_H
#define __WBSL_H

#ifdef __cplusplus
extern "C" {
#endif

// *************************************************************************************************
// include section

#include "project.h"

// *************************************************************************************************
// Defines section

#define BIT_0                   (0x0001)
#define BIT_1                   (0x0002)
#define BIT_2                   (0x0004)
#define BIT_3                   (0x0008)
#define BIT_4                   (0x0010)
#define BIT_5                   (0x0020)
#define BIT_6                   (0x0040)
#define BIT_7                   (0x0080)
#define BIT_8                   (0x0100)
#define BIT_9                   (0x0200)
#define BIT_A                   (0x0400)
#define BIT_B                   (0x0800)
#define BIT_C                   (0x1000)
#define BIT_D                   (0x2000)
#define BIT_E                   (0x4000)
#define BIT_F                   (0x8000)


/* ------------------------------------------------------------------
*        CONFIGURATION RADIO REGISTERS
*   -----------------------------------------------------------
*/

/*Setup radio (SmartRF Studio) */

#define WBSL_SETTING_FIFOTHR     0x07    /* FIFOTHR  - RX FIFO and TX FIFO thresholds */
/* Set the SYNC words to be used */
#define WBSL_SETTING_SYNC1       0xD3   /* Modem configuration. */
#define WBSL_SETTING_SYNC0       0x91   /* Modem configuration. */
#define WBSL_SETTING_PKTLEN      0xFE   /* Packet length. */
#define WBSL_SETTING_PKTCTRL1    0x06   /* Packet automation control. */
#define WBSL_SETTING_PKTCTRL0    0x45   /* Packet automation control. */
#define WBSL_SETTING_ADDR        WBSL_AP_ADDRESS  /* Device address. */

#ifdef ISM_EU
/* 869.525MHz */
#    define WBSL_SETTING_FREQ2    0x21      /*  Frequency control word, high byte */
#    define WBSL_SETTING_FREQ1    0x71      /*  Frequency control word, middle byte */
#    define WBSL_SETTING_FREQ0    0x7A      /*  Frequency control word, low byte */
#    define WBSL_SETTING_CHANNR     0       /* Channel number. */
#    define WBSL_SETTING_PA_TABLE0  0x67    /* PA output power setting. Due to RF regulations (+1.1dBm) */
#else
#    ifdef ISM_US
/* 902MHz (CHANNR=20 --> 906MHz) */
#    define WBSL_SETTING_FREQ2    0x22     /*  Frequency control word, high byte */
#    define WBSL_SETTING_FREQ1    0xB1     /*  Frequency control word, middle byte */
#    define WBSL_SETTING_FREQ0    0x3B    /*  Frequency control word, low byte */
#    define WBSL_SETTING_CHANNR     20      /* Channel number. */
#    define WBSL_SETTING_PA_TABLE0  0x51  /* PA output power setting. Due to RF regulations (+1.3dBm) */
#    else
#        ifdef ISM_LF
/* 433.30MHz */
#    define WBSL_SETTING_FREQ2    0x10      /*  Frequency control word, high byte */
#    define WBSL_SETTING_FREQ1    0xB0      /*  Frequency control word, middle byte */
#    define WBSL_SETTING_FREQ0    0x71      /*  Frequency control word, low byte */
#    define WBSL_SETTING_CHANNR     0       /* Channel number. */
#    define WBSL_SETTING_PA_TABLE0  0x61    /* PA output power setting. Due to RF regulations (+1.4dBm) */
#        else
#            error "Wrong ISM band specified (valid are ISM_LF, ISM_EU and ISM_US)"
#        endif /* ISM_LF */
#    endif     /* ISM_US */
#endif         /* ISM_EU */


#define WBSL_SETTING_FSCTRL1    0x0C   /* (IF) Frequency synthesizer control. */
#define WBSL_SETTING_FSCTRL0    0x00   /* Frequency synthesizer control. */

#define WBSL_SETTING_MDMCFG4    0x2D    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG3    0x3B    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG2    0x13    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG1    0x22    /* Modem configuration. */
#define WBSL_SETTING_MDMCFG0    0xF8    /* Modem configuration. */

#define WBSL_SETTING_DEVIATN     0x62    /* Modem deviation setting (when GSK modulation is enabled). */
#define WBSL_SETTING_MCSM2       0x07
#define WBSL_SETTING_MCSM1       0x3C
#define WBSL_SETTING_MCSM0       0x18   /* Main Radio Control State Machine configuration. */

#define WBSL_SETTING_FOCCFG      0x1D   /* Frequency Offset Compensation Configuration. */
#define WBSL_SETTING_BSCFG       0x1C   /*  Bit synchronization Configuration. */
#define WBSL_SETTING_AGCCTRL2    0xC7   /* AGC control. */
#define WBSL_SETTING_AGCCTRL1    0x00   /*  AGC control. */
#define WBSL_SETTING_AGCCTRL0    0xB0   /* AGC control. */

#define WBSL_SETTING_WOREVT1     0x87
#define WBSL_SETTING_WOREVT0     0x6B
#define WBSL_SETTING_WORCTRL     0xF8

#define WBSL_SETTING_FREND1      0xB6   /* Front end RX configuration. */
#define WBSL_SETTING_FREND0      0x10   /* Front end TX configuration. */

#define WBSL_SETTING_FSCAL3      0xEA   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSCAL2      0x2A   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSCAL1      0x00   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSCAL0      0x1F   /* Frequency synthesizer calibration. */
#define WBSL_SETTING_FSTEST      0x59
#define WBSL_SETTING_PTEST       0x7F
#define WBSL_SETTING_AGCTEST     0x3F
#define WBSL_SETTING_TEST2       0x88   /* Various test settings. */
#define WBSL_SETTING_TEST1       0x31   /* Various test settings. */
#define WBSL_SETTING_TEST0       0x09   /* Various test settings. */

/* To leave the CC1101 in a workable state after the Wireless Update has completed,
 * Some RF registers configured for WBSL need to be reset to their default values.
 */

#define RESET_VALUE_MCSM2        0x07
#define RESET_VALUE_PKTCTRL1     0x04
#define RESET_VALUE_ADDR         0x00
#define RESET_VALUE_CHANNR       0x00

#define RESET_VALUE_FSCTRL1      0x0F
#define RESET_VALUE_FSCTRL0      0x00
#define RESET_VALUE_MDMCFG4      0x8C
#define RESET_VALUE_MDMCFG3      0x22
#define RESET_VALUE_MDMCFG2      0x02
#define RESET_VALUE_MDMCFG1      0x22
#define RESET_VALUE_MDMCFG0      0xF8
#define RESET_VALUE_DEVIATN      0x47
#define RESET_VALUE_MCSM1        0x30
#define RESET_VALUE_MCSM0        0x04
#define RESET_VALUE_FOCCFG       0x36
#define RESET_VALUE_BSCFG        0x6C
#define RESET_VALUE_AGCCTRL2     0x03
#define RESET_VALUE_AGCCTRL1     0x40
#define RESET_VALUE_AGCCTRL0     0x91
#define RESET_VALUE_FREND1       0x56
#define RESET_VALUE_FSCAL3       0xA9
#define RESET_VALUE_FSCAL2       0x0A
#define RESET_VALUE_FSCAL1       0x20
#define RESET_VALUE_FSCAL0       0x0D
#define RESET_VALUE_TEST0        0x0B


/* Max time we can be in a critical section within the delay function.
 * This could be fine-tuned by observing the overhead is calling the bsp delay
 * function. The overhead should be very small compared to this value.
 * Note that the max value for this must be less than 19 usec with the
 * default CLKCON.TICKSPD and CLKCON.CLOCKSPD settings and external 26 MHz
 * crystal as a clock source (which we use).
 *
 * Be careful of direct calls to wbsl_DelayUsec().
 */

#define WBSL_MAX_DELAY_US 16 /* usec */
#define WBSL_PKTSTATUS_CCA BV(4)
#define WBSL_PKTSTATUS_CS  BV(6)

#if (defined MRFI_CC2500)

 #define WBSL_RSSI_OFFSET    72   /* no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
  #define WBSL_RSSI_VALID_DELAY_US    1000

#elif (defined MRFI_CC1100)

  #define WBSL_RSSI_OFFSET    79   /* no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
  #define WBSL_RSSI_VALID_DELAY_US    1300

#elif (defined MRFI_CC1101) || (defined MRFI_CC1100E_470) || (defined MRFI_CC1100E_950)

  #define WBSL_RSSI_OFFSET    74   /* no units */

/* Worst case wait period in RX state before RSSI becomes valid (safety margin added). */

  #define WBSL_RSSI_VALID_DELAY_US    2200

#else
  #error "ERROR: RSSI offset value not defined for this radio"
#endif

/* ---------- Radio Abstraction ---------- */

#if (defined MRFI_CC1100)
#define WBSL_RADIO_PARTNUM          0x00
#define WBSL_RADIO_MIN_VERSION      3

#elif (defined MRFI_CC1101)
#define WBSL_RADIO_PARTNUM          0x00
#define WBSL_RADIO_MIN_VERSION      4

#elif (defined MRFI_CC1100E_470)
#define WBSL_RADIO_PARTNUM          0x00
#define WBSL_RADIO_MIN_VERSION      5

#elif (defined MRFI_CC1100E_950)
#define WBSL_RADIO_PARTNUM          0x00
#define WBSL_RADIO_MIN_VERSION      5

#elif (defined MRFI_CC2500)
#define WBSL_RADIO_PARTNUM          0x80
#define WBSL_RADIO_MIN_VERSION      3
#else
#error "ERROR: Missing or unrecognized radio."
#endif



#define getFlag(val, flag)		            ((val&flag)==flag)
#define setFlag(val, flag)		            (val|=flag)
#define clearFlag(val, flag)		            (val&=(~flag))
#define toggleFlag(val, flag)		            (val^=flag)

/* Conversion from msec to ACLK timer ticks */
#define CONV_MS_TO_TICKS(msec)               (((msec) * 32768) / 1000)

/* bit value */
#ifndef BV
#define BV(n)      (1 << (n))
#endif

/* WBSL Macros*/
#define WBSL_AP_ADDRESS                             (0xCA)
#define WBSL_MAX_PAYLOAD_LENGTH                     (55u)

#if WBSL_MAX_PAYLOAD_LENGTH < 50
#error "MAXIMUM PAYLOAD CANNOT BE LOWER TO 50 BYTES, DUE TO PROTOCOL TIMING CONSTRAINTS"
#elif WBSL_MAX_PAYLOAD_LENGTH > 58
#error "MAXIMUM PAYLOAD MUST NOT BE HIGHER THAN 58 BYTES, TO AVOID TXFIFO OVERFLOW".
#endif

#define WBSL_LENGTH_FIELD_OFS                        0
#define WBSL_LENGTH_FIELD_SIZE                      (1u)
#define WBSL_RX_METRICS_SIZE                        (2u)
#define WBSL_OVERHEAD_LENGTH                        (6u)
#define WBSL_TOTAL_LENGTH                           WBSL_MAX_PAYLOAD_LENGTH + WBSL_OVERHEAD_LENGTH
#define DISCOVERY_PAYLOAD_LENGTH                    (4u)
#define DISCOVERY_OVERHEAD_LENGTH                   (3u)
#define AP_ADDRESS_OFFSET_RX                        (1u)
#define ED_ADDRESS_OFFSET_RX                        (2u)
#define BATTERY_VOLTAGE_OFFSET                      (3u)

#define WBSL_CRC_STATUS_OFFSET                      (2)
#define CRC_STATUS                                  (0x80)

#define WBSL_OPCODE_OFFSET                          (5u)

#define AP_ADDRESS_OFFSET_TX                        (2u)
#define ED_ADDRESS_OFFSET_TX                        (1u)

#define DISCOVERY_ACK_OFFSET                        (3u)
#define INIT_TOTAL_PACKETS_OFFSET                   (3u)
#define CURRENT_PACKET_NR_OFFSET                    (3u)

#define WBSL_TX_RES_FAILED                          (0)
#define WBSL_TX_RES_SUCCESS                         (1u)


/* Battery end of life voltage threshold -> do not accept connection */
#define BATTERY_LOW_THRESHOLD			(240u)


#define WBSL_STATUS_LINKING		            (BIT_0)
#define WBSL_STATUS_LINKED		            (BIT_1)
#define WBSL_STATUS_ERROR		            (BIT_2)
#define WBSL_TRIGGER_SEND_DATA 	                    (BIT_3)
#define WBSL_TRIGGER_RECEIVED_DATA 	            (BIT_4)
#define WBSL_TRIGGER_STOP		            (BIT_5)
#define WBSL_TRIGGER_SEND_CMD                       (BIT_6)
#define WBSL_LOW_BATT                               (BIT_7)


#define WBSL_DISABLED                               (BIT_0)
#define WBSL_PROCESSING_PACKET                      (BIT_1)
#define WBSL_SEND_INFO_PACKET                       (BIT_2)
#define WBSL_SEND_NEW_DATA_PACKET                   (BIT_3)
#define WBSL_ERROR                                  (BIT_7)


#define WBSL_IDLE_MODE                              (BIT_0)
#define WBSL_RX_MODE                                (BIT_1)
#define WBSL_TX_MODE                                (BIT_2)


#define WBSL_RXTX_RECEIVED                          (BIT_0)
#define WBSL_RXTX_SEND                              (BIT_1)

/* Values for linking failed or successful */
#define WBSL_LINK_FAIL                              0
#define WBSL_LINK_SUCC                              1

/* Values for Packet ACK */
#define WBSL_ACK_FAIL                               (0)
#define WBSL_ACK_SUCC                               (1u)

#define TIMEOUT_FOR_ACK               (CONV_MS_TO_TICKS(300))
#define TIMEOUT_FOR_RXWATCH           (CONV_MS_TO_TICKS(10)) /* 10 milliseconds in ticks */

#define WBSL_MAXIMUM_RETRIES                        (5u)


#define WBSL_PACKET_EMPTY                           (BIT_0)
#define WBSL_PACKET_FILLING                         (BIT_1)
#define WBSL_PACKET_FULL                            (BIT_2)
#define WBSL_PACKET_ADDRESS                         (BIT_3)

/*WBSL OP Code Type */
#define WBSL_INIT_PACKET                            0x00
#define WBSL_ADDRESS_PACKET                         0x01
#define WBSL_NORMAL_PACKET                          0x02

#define TX_SIZE                                       WBSL_TOTAL_LENGTH
#define RX_BUFFER_SIZE                                (255u)


// *************************************************************************************************
// Extern section

extern unsigned char TxBuffer[TX_SIZE];
extern unsigned char initPacket[6];
extern unsigned char ed_address;
extern unsigned char wbsl_data[WBSL_MAX_PAYLOAD_LENGTH];
extern unsigned char wbsl_status;
extern unsigned int total_packets;
extern volatile unsigned char packet_ready_flag;
/* Flag for status information and external control through USB driver */
extern volatile unsigned char wbsl_flag;
/* Flag for status information, to see if WBSL is in RX/TX/IDLE mode  */
extern volatile unsigned char wbslMode_flag;
/* Flag to check if a new packet is needed from the GUI */
extern volatile unsigned char wbsl_packet_flag;
/* Flag to indicate a packet has been received */
extern volatile unsigned char rxtx_flag;

// *************************************************************************************************
// API section


/* ------------------------------------------------------------------------------------------------
 *      FUNCTION PROTOTYPES
 * ------------------------------------------------------------------------------------------------
 */

// WBSL functions
extern void wbsl_main(void);
extern void wbsl_config(void);
extern void wbsl_RfIsr(void);
extern void wbsl_GpioIsr(void);
void wbsl_sendPacket(void);
unsigned char wbsl_link(void);
void wbsl_reset(void);
u8 wbsl_transmit(u8 *src, u8 size);

// Timer
void wbsl_resetTimer(void);
void wbsl_setTimer(u16 timeout);

#ifdef __cplusplus
}
#endif

#endif

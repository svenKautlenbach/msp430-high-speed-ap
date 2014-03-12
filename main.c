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
#include <intrinsics.h>
#include <string.h>
#include "project.h"
#include "ccSPI.h"
#include "ccxx00.h"
#include "cmdHandler.h"

#include "USB_config/descriptors.h"

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"               //Basic Type declarations
#include "USB_API/USB_Common/usb.h"                 //USB-specific functions

#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"

#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbConstructs.h"


#include "BM_API.h"
#include "BlueRobin_TX_API.h"
#include "simpliciti.h"
#include "mrfi.h"
#include "WBSL/wbsl.h"

//Function declarations
void InitPorts_v(void);
void InitClock_v(void);

//Global flags set by events
volatile u8 bCDCDataReceived_event = FALSE;   //Indicates data has been received without an open rcv operation

// *************************************************************************************************
// Global Variable section

// SimpliciTI Sync variables
u8 simpliciti_sync_buffer[BM_SYNC_DATA_LENGTH];
u8 simpliciti_sync_buffer_status;

// *************************************************************************************************
// Extern variable section
extern u8 rf_tx_over;     // Flag to indicate last byte has been sent

// *************************************************************************************************
// Extern functions section

// BlueRobin init function
void InitProject_v(void);

// *************************************************************************************************
// Function prototype section

// *************************************************************************************************
// The one and only main function
// *************************************************************************************************
void main (void)
{
  // Stop WDT
  WDTCTL = WDTPW + WDTHOLD;

  // As fast as possible 26MHz to GDO2 pin of CC1101 as it is required to clock USB  
  CC_SPI_Init_v();
  //CC_SPI_SelectChipWaitUntilReady_v();
  CC_SPI_WriteRead_u8(CCXX00_REG_IOCFG0);
  CC_SPI_WriteRead_u8(0x2E);
  CC_SPI_WriteRead_u8(CCXX00_REG_IOCFG2);
  CC_SPI_WriteRead_u8(0x30);
  CC_SPI_DeselectChip_v();
  UCSCTL6 |= XT2BYPASS;

  // Initialize unused port pins
  InitPorts_v();

  // Set Vcore to the level required for USB operation
  SetVCore(3);

  // Initialize clock system
  InitClock_v();

  // Initialize debug output pins
  INIT_TX_ACTIVITY;
  INIT_RX_ACTIVITY;

  // Initialize BR receiver library
  BR_Init_v();
  BRTX_SetID_v(TX_SERIAL_NO);
  BRTX_WriteData_v(0, 40);
  // Reset simpliciti_data
  simpliciti_data[0] = 0xFF;

  // Initialize command handler
  usb_handler_init();

  // Enable interrupts
  __enable_interrupt();

  // Initialize USB port
  USB_init();

  // Enable various USB event handling routines
  USB_setEnabledEvents(  kUSB_VbusOnEvent + kUSB_VbusOffEvent + kUSB_receiveCompletedEvent
                       + kUSB_dataReceivedEvent + kUSB_UsbSuspendEvent + kUSB_UsbResumeEvent + kUSB_UsbResetEvent);
    
  // See if we are already attached physically to USB, and if so, connect to it
  // Normally applications don't invoke the event handlers, but this is an exception.  
  if (USB_connectionInfo() & kUSB_vbusPresent)
  {
    USB_handleVbusOnEvent();
  }

  while (1)
  {

    // For BlueRobin
    if (bluerobin_start_now && !simpliciti_on && !wbsl_on)
    {
      // Start BlueRobin stack
      bluerobin_start();
      system_status = HW_BLUEROBIN_TRANSMITTING;
      // Reset start flag
      bluerobin_start_now = 0;
    }
    // For SimpliciTI AP
    else if (simpliciti_start_now && !wbsl_on)
    {
      RX_ACTIVITY_ON;
      // Clear start trigger
      simpliciti_start_now = 0;
      // Assign new system status
      system_status = HW_SIMPLICITI_LINKED;
      simpliciti_on = 1;
      // Start access point and stay there until exit flag is set
      simpliciti_main();
      // Clear SimpliciTI flags
      system_status = HW_SIMPLICITI_STOPPED;
      simpliciti_on = 0;
      // Clean up after SimpliciTI
      simpliciti_data[0] = 0xFF;
      RX_ACTIVITY_OFF;
    }
    else if (wbsl_start_now && !simpliciti_on)
    {

      RX_ACTIVITY_ON;
      // Clear start Trigger
      wbsl_start_now = 0;
      //Config the RF module for WBSL
      wbsl_config();
      // Assign new System Status
      system_status = HW_WBSL_LINKED;
      wbsl_on = 1;
      // Start access point and try to pair  with an End Device,
      // once paired, download the software update then return.
      wbsl_main();

      // Check if there was an error during the Update procedure to alert the GUI
      if(getFlag(wbsl_flag,WBSL_STATUS_ERROR))
      {
    	  system_status = HW_WBSL_ERROR;
      }
      else
      {
    	  system_status = HW_WBSL_STOPPED;
      }
       //Clear WBSL Flags
       wbsl_on = 0;
       wbsl_data[0] = 0xFF;

       RX_ACTIVITY_OFF;
   }

  }

}

void USB_Handler_v(void)
{
	WORD bytesSent;
	WORD bytesReceived;


        // From PC to MSP430
        if (bCDCDataReceived_event)
        {
          bCDCDataReceived_event = FALSE;                    // Clear flag early -- just in case execution breaks below because of an error
          bytesReceived = USBCDC_bytesInUSBBuffer(0);

          if (usb_bufferIndex + bytesReceived > USB_MAX_MESSAGE_LENGTH)
          {
            usb_bufferIndex = 0;
          }

          USBCDC_receiveData(&usb_buffer[usb_bufferIndex], bytesReceived, 0);

          // Increase buffer index
          usb_bufferIndex += bytesReceived;

          // get packet length from byte #2 of received packet
          if ((usb_bufferIndex >= 2) && (usb_buffer[2] >= USB_MIN_MESSAGE_LENGTH) && (usb_bufferIndex > (usb_buffer[2] - 1)))
          {
            //extract data from packet
            usb_decode();
            usb_bufferIndex = 0;
          }
        }

        // From MSP430 to PC
        if (usb_sendack)
        {
          if ((USBCDC_intfStatus(0, &bytesSent, &bytesReceived) & kUSBCDC_waitingForSend) == 0)				
          { //we can send
            // Disable interrupts
            __disable_interrupt();

            switch (USBCDC_sendData(&usb_buffer[0], usb_buffer[2], 0))
            {
              case kUSBCDC_sendStarted:
                break;
              case kUSBCDC_busNotAvailable:
                break;
              default:
                break;;
            }
            usb_sendack = 0;
            // Enable interrupts
            __enable_interrupt();
          }
          else
          {
            //send operation from previous sendData() is not completed yet.
//            ......

          }
        }


}

// *************************************************************************************************
// Init clock system
// *************************************************************************************************
void InitClock_v(void)
{
  // Enable 32kHz ACLK on XT1 via external crystal and 26MHz on XT2 via external clock signal
  P5SEL |=  (BIT2 | BIT3 | BIT4 | BIT5); // Select XINs and XOUTs
//  UCSCTL6 &= ~XT1OFF;        // Switch on XT1, keep highest drive strength - default
//  UCSCTL6 |=  XCAP_3;        // Set internal load caps to 12pF - default
//  UCSCTL4 |=  SELA__XT1CLK;  // Select XT1 as ACLK - default

  // Configure clock system
  _BIS_SR(SCG0);    // Disable FLL control loop
  UCSCTL0 = 0x0000; // Set lowest DCOx, MODx to avoid temporary overclocking
  // Select suitable DCO frequency range and keep modulation enabled
  UCSCTL1 = DCORSEL_5; // DCO frequency above 8MHz but not bigger than 16MHz
  UCSCTL2 = FLLD__2 | (((MCLK_FREQUENCY + 0x4000) / 0x8000) - 1); // Set FLL loop divider to 2 and
                                                                  // required DCO multiplier
//  UCSCTL3 |= SELREF__XT1CLK;                    // Select XT1 as FLL reference - default
//  UCSCTL4 |= SELS__DCOCLKDIV | SELM__DCOCLKDIV; // Select XT1 as ACLK and
                                                  // divided DCO for SMCLK and MCLK - default
  _BIC_SR(SCG0);                                  // Enable FLL control loop again

  // Loop until XT1 and DCO fault flags are reset
  do
  {
    // Clear fault flags
    UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG;                      
  } while ((SFRIFG1 & OFIFG));	

  // Worst-case settling time for the DCO when changing the DCO range bits is:
  // 32 x 32 x MCLK / ACLK
  __delay_cycles(((32 * MCLK_FREQUENCY) / 0x8000) * 32);

  _BIS_SR(SCG0);    // Disable FLL control loop
}

// *************************************************************************************************
// Init Ports
// *************************************************************************************************
void InitPorts_v(void)
{
  // Initialize all unused pins as low level output
  P4OUT  &= ~(                            BIT4 | BIT5 | BIT6 | BIT7);
  P5OUT  &= ~(BIT0 | BIT1                                          );
  P6OUT  &= ~(BIT0 | BIT1 | BIT2 | BIT3                            );
  P4DIR  |=  (                            BIT4 | BIT5 | BIT6 | BIT7);
  P5DIR  |=  (BIT0 | BIT1                                          );
  P6DIR  |=  (BIT0 | BIT1 | BIT2 | BIT3                            );
}

// *************************************************************************************************
// Start BlueRobin transmission
// *************************************************************************************************
void bluerobin_start(void)
{
  u32 ID_u32;
  // Init the BlueRobin Timer to run continuous mode clocked with ACK
  TA0CTL = MC1 | TASSEL0;
  // Init s/w
  ID_u32 = BRTX_GetID_u32();
  BR_Init_v();
  BRTX_SetID_v(ID_u32);
  BRTX_Start_v();

  // Set on flag
  bluerobin_on = 1;
}

// *************************************************************************************************
// Stop BlueRobin transmission
// *************************************************************************************************
void bluerobin_stop(void)
{
  BRTX_Stop_v();
  //Stop BlueRobin Timer
  TA0CTL &= ~MC1;
  /* Set Timer0 count register to 0x0000 */
  TA0R = 0;
  // Clear on flag
  bluerobin_on = 0;
}

/*  
 * ======== UNMI_ISR ========
 */
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
  switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
  {
    case SYSUNIV_NONE:
      __no_operation();
      break;
    case SYSUNIV_NMIIFG:
      __no_operation();
      break;
    case SYSUNIV_OFIFG:
      UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG); //Clear OSC fault flags
      SFRIFG1 &= ~OFIFG;                          //Clear OFIFG fault flag
      break;
    case SYSUNIV_ACCVIFG:
      __no_operation();
      break;
    case SYSUNIV_BUSIFG:
                                                          //If bus error occured - the cleaning of flag and re-initializing of
                                                          //USB is required.
      SYSBERRIV = 0;                                      //clear bus error flag
      USB_disable();                                      //Disable
      break;
  }
}






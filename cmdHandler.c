// *************************************************************************************************
// Command handler via a UART (USB) interface
// *************************************************************************************************


// *************************************************************************************************
// Include section

#include <string.h>
#include "project.h"
#include "cmdHandler.h"
#include "simpliciti.h"
#include "BlueRobin_TX_API.h"
#include "BM_API.h"
#include "bsp_leds.h"

/*  Headers related to WBSL  */

#include "WBSL/wbsl.h"


// *************************************************************************************************
// Define section


// *************************************************************************************************
// Local Typedef section


// *************************************************************************************************
// Function prototype section

BYTE USB_disconnect ();


// *************************************************************************************************
// Static Variable section


// *************************************************************************************************
// Global Variable section

u8 system_status        = HW_IDLE;
u8 simpliciti_on        = 0;
u8 bluerobin_on         = 0;
u8 bluerobin_start_now  = 0;
u8 simpliciti_start_now = 0;

// WBSL
u8 wbsl_on              = 0;
u8 wbsl_start_now       = 0;
u8 ptrTX;
u8 wbsl_size;
u8 wbsl_inc;
u8 current_packet_size;
u8 wbsl_OPCode;
u8 wbsl_packetLength;

u8 usb_buffer[USB_MAX_MESSAGE_LENGTH + 2];
u8 usb_bufferIndex;
u8 usb_newdata;
u8 usb_sendack;

u8 HeartRate_u8;   // BlueRobin heartrate to transmit
u8 Speed_u8;       // BlueRobin speed to transmit
u16 Distance_u16;  // BlueRobin distance to transmit

// *************************************************************************************************
// Initialize command handler task
// *************************************************************************************************
void usb_handler_init(void)
{
  usb_bufferIndex = 0;
  usb_newdata     = 0;
  usb_sendack     = 0;
  memset(&usb_buffer, 0x00, sizeof(usb_buffer));
}

// *************************************************************************************************
// Decode received command, extract data and trigger actions.
// *************************************************************************************************
void usb_decode(void)
{
  u8  Counter_u8;
  u32 ID_u32;
  u8 i;
  // Check if start marker is set
  if (usb_buffer[PACKET_BYTE_START] != 0xFF) return;

  // Check command code
  switch (usb_buffer[PACKET_BYTE_CMD])
  {
    // Generic commands
    case BM_GET_PRODUCT_ID:   usb_buffer[PACKET_BYTE_FIRST_DATA + 3] = (u8)(PRODUCT_ID >> 24);
                              usb_buffer[PACKET_BYTE_FIRST_DATA + 2] = (u8)(PRODUCT_ID >> 16);
                              usb_buffer[PACKET_BYTE_FIRST_DATA + 1] = (u8)(PRODUCT_ID >> 8);
                              usb_buffer[PACKET_BYTE_FIRST_DATA]     = (u8)(PRODUCT_ID);
                              break;
    case BM_GET_STATUS:       usb_buffer[PACKET_BYTE_FIRST_DATA] = system_status; // + 1;
                              break;
    case BM_START_BSL:        // Call flash updater in BSL memory
                              USB_disconnect();
                              // No further interrupts
                              INTERRUPTS_DISABLE();
                              // Erase reset vector
                              FCTL3 = FWKEY;
                              FCTL1 = FWKEY + ERASE;
                              // Dummy write to erase segment
                              *(u8*)0x0FFFE = 0;
                              // Wait until not busy
                              while ((FCTL3 & BUSY) != 0);
                              // Lock flash memory again
                              FCTL1 = FWKEY + LOCK;
                              // Force BOR
                              PMMCTL0 |= PMMSWBOR;
                              break;
    // BlueRobin TX commands
    case BM_RESET:            bluerobin_stop();
                              bluerobin_on = 0;
                              simpliciti_on = 0;
                              system_status = HW_IDLE;
                              break;
    case BM_START_BLUEROBIN:
    	                      /* Before starting BlueRobin, wait until an ongoing
    	    	               * wireless update has completed
    	    	               */
    	    	              if (wbsl_on)
    	                      {
    	    	                 break;
    	                      }

    	                      if (simpliciti_on)
    	                      {
    	                        setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP);
    	                      }

                              if (!bluerobin_on)
                              {
                                // Can only start once instance
                            	bluerobin_start_now = 1;
                              }
                              break;
    case BM_STOP_BLUEROBIN:   bluerobin_stop();
                              system_status = HW_BLUEROBIN_STOPPED;
                              break;
    case BM_SET_BLUEROBIN_ID: BRTX_SetID_v(  ((u32)usb_buffer[PACKET_BYTE_FIRST_DATA + 3] << 24)
                                           + ((u32)usb_buffer[PACKET_BYTE_FIRST_DATA + 2] << 16)
                                           + ((u16)usb_buffer[PACKET_BYTE_FIRST_DATA + 1] << 8)
                                           +       usb_buffer[PACKET_BYTE_FIRST_DATA]
                                          );
                              break;
    case BM_GET_BLUEROBIN_ID: ID_u32 = BRTX_GetID_u32();
    	                      usb_buffer[PACKET_BYTE_FIRST_DATA+3] = ID_u32 >> 24;
                              usb_buffer[PACKET_BYTE_FIRST_DATA+2] = ID_u32 >> 16;
                              usb_buffer[PACKET_BYTE_FIRST_DATA+1] = ID_u32 >> 8;
                              usb_buffer[PACKET_BYTE_FIRST_DATA]   = ID_u32;
                              break;
    case BM_SET_HEARTRATE:    HeartRate_u8 = usb_buffer[PACKET_BYTE_FIRST_DATA];
                              break;
    case BM_SET_SPEED:        Speed_u8     = usb_buffer[PACKET_BYTE_FIRST_DATA];
                              Distance_u16 =   ((u16)usb_buffer[PACKET_BYTE_FIRST_DATA + 2] << 8)
                                             +       usb_buffer[PACKET_BYTE_FIRST_DATA + 1];
                              break;

    // SimpliciTI RX commands
    case BM_START_SIMPLICITI:
    	                      /* Before starting SimpliciTi, wait until an ongoing
    	                       * wireless update has completed
    	                       */
    	                      if (wbsl_on)
                              {
    	                         break;
                              }

    	                      if (bluerobin_on)
    	                      {
    	                        bluerobin_stop();
    	                      }

                              // Can only start one stack
                              if (!simpliciti_on)
                              {
                            	system_status = HW_SIMPLICITI_TRYING_TO_LINK;
                                // simpliciti_start_rx_only_now = 1;
                            	simpliciti_start_now = 1;
                              }
                              break;
    case BM_GET_SIMPLICITIDATA:
                              if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_RECEIVED_DATA))
                              {
                                // Assemble IN packet (ID/BTN, DataX, DataY, DataZ)
                            	usb_buffer[PACKET_BYTE_FIRST_DATA + 3] = simpliciti_data[3];
                            	usb_buffer[PACKET_BYTE_FIRST_DATA + 2] = simpliciti_data[2];
                            	usb_buffer[PACKET_BYTE_FIRST_DATA + 1] = simpliciti_data[1];
                            	usb_buffer[PACKET_BYTE_FIRST_DATA]     = simpliciti_data[0];
                                // Mark buffer as already read
                                simpliciti_data[0] = 0xFF;
                                clearFlag(simpliciti_flag, SIMPLICITI_TRIGGER_RECEIVED_DATA);
                              }
                              else
                              {
                                // Return packet with "already read" marker
                            	usb_buffer[PACKET_BYTE_FIRST_DATA] = 0xFF;
                              }
                              break;

    // SimpliciTI Sync commands
    case BM_SYNC_START:
                              break;
    case BM_SYNC_SEND_COMMAND:
                              // Copy command data to SimpliciTI buffer
                              if (simpliciti_on)
                              {
                                for (Counter_u8 = 0; Counter_u8 < BM_SYNC_DATA_LENGTH; ++Counter_u8)
                                {
                                  simpliciti_data[Counter_u8] = usb_buffer[PACKET_BYTE_FIRST_DATA + Counter_u8];
                                }
                                // Set flag to send out command when receiving next ready-to-receive packet
                                setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_CMD);
                              }
                              break;
    case BM_SYNC_GET_BUFFER_STATUS:
                              // Set response
    	                      usb_buffer[3] = simpliciti_sync_buffer_status;
                              // Set reply packet length
    	                      usb_buffer[PACKET_BYTE_SIZE] = 1 + PACKET_OVERHEAD_BYTES;
                              break;
    case BM_SYNC_READ_BUFFER:
                              // Copy bytes from sync buffer to USB buffer
                              if (simpliciti_sync_buffer_status)
                              {
                                for (Counter_u8 = 0; Counter_u8 < BM_SYNC_DATA_LENGTH; ++Counter_u8)
                                {
                                  usb_buffer[PACKET_BYTE_FIRST_DATA + Counter_u8] = simpliciti_data[Counter_u8];
                                }
                                // Free buffer
                                simpliciti_sync_buffer_status = 0;
                              }
                              // Set reply packet length
                              usb_buffer[PACKET_BYTE_SIZE] = BM_SYNC_DATA_LENGTH + PACKET_OVERHEAD_BYTES;
                              break;


    // SimpliciTI shared commands
    case BM_STOP_SIMPLICITI:  // Stop through remote control
                              setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP);
                              break;


     // WBSL commands
     case BM_START_WBSL:       //Start WBSL procedure

							// Can only start one stack
							if (bluerobin_on) bluerobin_stop();
							// Can only start one stack
							if (simpliciti_on) setFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP);

							if(!wbsl_on)
							{
							   system_status = HW_WBSL_TRYING_TO_LINK;
							   // Trigger the BSL Start
							   wbsl_start_now = 1;
							}
						    break;

     case BM_STOP_WBSL:        //Stop running the BSL

                           setFlag(wbsl_flag, WBSL_TRIGGER_STOP);
                           // LED off
                           LED_OFF;
                           break;

     case BM_GET_WBSL_STATUS:     //Get the status of how far along is the transmission of the Update

                           usb_buffer[PACKET_BYTE_FIRST_DATA]   = wbsl_status;
                           break;

     case BM_GET_PACKET_STATUS_WBSL:

                            if(wbsl_on)
                            {
                             //Get the status of whether or not a new packet needs to be sent from the GUI
                             usb_buffer[PACKET_BYTE_FIRST_DATA]   = wbsl_packet_flag;
                             }
                             else
                             {
                            // This Command shouldn't have come when wbsl is off, tell the GUI an error has ocurred
                             usb_buffer[PACKET_BYTE_FIRST_DATA] = WBSL_ERROR;
                             }
                             break;

     case BM_GET_MAX_PAYLOAD_WBSL:  // Send the max number of bytes allowed in the payload

                             usb_buffer[PACKET_BYTE_FIRST_DATA]  = WBSL_MAX_PAYLOAD_LENGTH;

                             break;

    case BM_SEND_DATA_WBSL:
                           // Set the flag as processing so that the GUI knows that it needs to wait before sending next packet
                            wbsl_packet_flag = WBSL_PROCESSING_PACKET;

                           // Check if this is the first message received for the current packet
                           if (packet_ready_flag == WBSL_PACKET_EMPTY)
                           {
                             current_packet_size = 0;
                             wbsl_inc = usb_buffer[PACKET_BYTE_FIRST_DATA - 1] - PACKET_OVERHEAD_BYTES - 2; // Get total of bytes received and substract overhead (3 from USB and 2 from Packet Length and Address Byte

                             // Save the OP Code Byte
                             wbsl_OPCode = usb_buffer[PACKET_BYTE_FIRST_DATA];

                             if(wbsl_OPCode == WBSL_INIT_PACKET) // Init Packet
                             {
                                 TxBuffer[WBSL_OVERHEAD_LENGTH] = usb_buffer[PACKET_BYTE_FIRST_DATA + 1];
                                 TxBuffer[WBSL_OVERHEAD_LENGTH + 1] = usb_buffer[PACKET_BYTE_FIRST_DATA + 2];

                                 // Save the total number of packets to be sent
                                 total_packets = usb_buffer[PACKET_BYTE_FIRST_DATA + 1] +((usb_buffer[PACKET_BYTE_FIRST_DATA + 2] << 8) & 0xFF00);

                                 initPacket[INIT_TOTAL_PACKETS_OFFSET] = total_packets & 0xFF;
                                 initPacket[INIT_TOTAL_PACKETS_OFFSET+1] = (total_packets >> 8) & 0xFF;
                                 packet_ready_flag = WBSL_PACKET_FULL;
                                 ptrTX = 0;
                               }
                               else   // Regular data packet
                               {

                            	   wbsl_packetLength = usb_buffer[PACKET_BYTE_FIRST_DATA + 1];

                                  if(wbsl_OPCode == WBSL_ADDRESS_PACKET)
                                	  TxBuffer[WBSL_OPCODE_OFFSET] = WBSL_ADDRESS_PACKET; // Set the address OPCODE
                                  else
                                      TxBuffer[WBSL_OPCODE_OFFSET] = WBSL_NORMAL_PACKET; // Set the packet as a normal packet

                                      //Update size of packet to be sent and substract Overhead and Length field
                                    	TxBuffer[0] = wbsl_packetLength + WBSL_OVERHEAD_LENGTH - 1;

                                        for(i=0;i<wbsl_inc;i++)  // Copy all Received Bytes to the TxBuffer
                                        {
                                            TxBuffer[i+WBSL_OVERHEAD_LENGTH] = usb_buffer[PACKET_BYTE_FIRST_DATA + i + 2];
                                        }

                                        packet_ready_flag = WBSL_PACKET_FILLING;
                                        ptrTX = wbsl_inc;  // Update pointer

                                        // Update the current size to know when the complete packet has been received
                                        current_packet_size += wbsl_inc;

                                        // Check if complete packet has been received
                                        if(current_packet_size >= wbsl_packetLength)
                                        {
                                            packet_ready_flag = WBSL_PACKET_FULL;
                                            ptrTX = 0;
                                         }
                              }

                           }
                           else if(packet_ready_flag == WBSL_PACKET_FILLING)
                           {
                                   wbsl_inc = usb_buffer[PACKET_BYTE_FIRST_DATA - 1] - PACKET_OVERHEAD_BYTES; // Get total of bytes received and substract overhead

                                   for(i=0;i<wbsl_inc;i++)// Copy all Received Bytes to the TxBuffer
                                   {
                                       TxBuffer[ptrTX + i + WBSL_OVERHEAD_LENGTH] = usb_buffer[PACKET_BYTE_FIRST_DATA + i];
                                   }
                                   ptrTX += wbsl_inc;  // Update pointer

                                   // Update the current size to know when the complete packet has been received
                                   current_packet_size += wbsl_inc;

                                   // Check if total bytes have been received or the maximum payload length has been reached
                                   // reaching max payload length and still having bytes to be read is an error since bytes would be lost
                                   if(ptrTX >= WBSL_MAX_PAYLOAD_LENGTH || current_packet_size >= wbsl_packetLength)
                                   {
                                      packet_ready_flag = WBSL_PACKET_FULL;
                                      ptrTX = 0;
                                   }
                            }
                            break;


  }
  
  // Return packet with original data, but modified command byte (acknowledge)
  usb_sendack = 1;
  usb_buffer[PACKET_BYTE_CMD] = HW_NO_ERROR;
}

// Function called just before a new packet is build
void BRTX_PrepareData_v(void)
{
}

// Function called after data have been sent
void BRTX_DataTransfered_v(void)
{
  static u16 TimeAccumulator_u16 = 0;
  u32 DataToSend_u32;
  u8 DataToSend_u8;

  TimeAccumulator_u16 += 0x7000;
  // Another second passed?
  if (TimeAccumulator_u16 >= 0x8000)
  {
    TimeAccumulator_u16 -= 0x8000;
  }

  DataToSend_u8 = HeartRate_u8;
  DataToSend_u32 = (((unsigned long)Speed_u8 << 8) << 8) + Distance_u16;

  BRTX_WriteData_v(DataToSend_u32, DataToSend_u8);
}


// *************************************************************************************************
// End of file

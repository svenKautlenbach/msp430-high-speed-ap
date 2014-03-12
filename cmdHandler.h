// *************************************************************************************************
// Copyright 2010 BM innovations GmbH, all rights reserved.
// The information contained herein is confidential property of BM innovations GmbH.
// The use, copying, transfer or disclosure of such information is prohibited
// except by written agreement with BM innovations GmbH.
// *************************************************************************************************
// Actual revision: 1.0
// Revision date:   10-01-01
// *************************************************************************************************
// Public header for a command handler via a UART interface
// *************************************************************************************************

#ifndef __CMD_HANDLER_UART_H
#define __CMD_HANDLER_UART_H

#ifdef __cplusplus
extern "C" {
#endif


// *************************************************************************************************
// Include section


// *************************************************************************************************
// Defines section

#define USB_MIN_MESSAGE_LENGTH        (3u)
#define USB_MAX_MESSAGE_LENGTH        (32u)


// *************************************************************************************************
// Typedef section

#include "project.h"

// *************************************************************************************************
// API section

void usb_handler_init(void);
void usb_decode(void);

extern u8 system_status;
extern u8 simpliciti_on;
extern u8 bluerobin_on;
extern u8 wbsl_on;
extern u8 bluerobin_start_now;
extern u8 simpliciti_start_now;
extern u8 wbsl_start_now;

extern u8 usb_buffer[USB_MAX_MESSAGE_LENGTH + 2];
extern u8 usb_bufferIndex;
extern u8 usb_newdata;
extern u8 usb_sendack;

extern u8 HeartRate_u8;   // BlueRobin heartrate to transmit
extern u8 Speed_u8;       // BlueRobin speed to transmit
extern u16 Distance_u16;  // BlueRobin distance to transmit

#ifdef __cplusplus
}
#endif

// *************************************************************************************************
#endif // __CMD_HANDLER_UART_H
// End of file

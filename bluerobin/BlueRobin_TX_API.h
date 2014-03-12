// *************************************************************************************************
//
// Copyright 2009 BM innovations GmbH (www.bm-innovations.com), all rights reserved.
//
// This trial version of the "BlueRobin(TM) transmitter library for the Texas Instruments
// MSP430 MCU" may be used for non-profit non-commercial purposes only. If you want to use
// BlueRobin(TM) in a commercial project, please contact the copyright holder for a
// separate license agreement.
//
// By using this trial version of the "BlueRobin(TM) transmitter library for the Texas Instruments
// MSP430 MCU", you implicitly agree that you will not modify, adapt, disassemble, decompile,
// reverse engineer, translate or otherwise attempt to discover the source code of the
// "BlueRobin(TM) transmitter library for the Texas Instruments MSP430 MCU".
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
//
// Public header for eZ430-Chronos specific BlueRobin(TM) transceiver library.
//
// The following BlueRobin(TM) profiles are supported by this build
//   - heart rate (HR) transmitter
//
// *************************************************************************************************
//
// BlueRobin(TM) packet size
// -------------------------
//
//              * average packet rate   1 packet/875 msec = ~1.14 packets/second
//              * payload per packet    5 bytes
//
// BlueRobin(TM) frequency overview
// (Please note: Settings apply for the transmitter side, i.e. the USB dongle)
// ----------------------------------------------------------------------
//
// Bluerobin_RX_433MHz.lib (433MHz ISM band)
//
//              * frequency                             433.30 MHz - 434.00 MHz
//              * deviation                             95 kHz
//              * channels                              3
//              * data rate                             250 kBaud
//
// Bluerobin_RX_868MHz.lib (868MHz ISM band)
//
//              * frequency                             868.25 MHz - 868.95 MHz
//              * deviation                             95 kHz
//              * channels                              3
//              * data rate                             250 kBaud
//
//
// Bluerobin_RX_915MHz.lib (915MHz ISM band)
//
//              * frequency                             914.35 MHz - 917.75 MHz
//              * deviation                             95 kHz
//              * channels                              34
//              * data rate                             250 kBaud
//
// *************************************************************************************************

#ifndef BRTX_API_H_
#define BRTX_API_H_

// *************************************************************************************************
// Include section

// *************************************************************************************************
// Defines section

// List of all possible channel states
typedef enum
{
    TX_OFF = 0,                 // Powerdown mode
    TX_ACTIVE                   // Active mode
} brtx_state_t;

// *************************************************************************************************
// API section

// ----------------------------------------------------------
// Functions for initializing and controlling the library

// Initialize several global variables.
void BR_Init_v(void);

// Start transmission.
void BRTX_Start_v(void);

// Stop transmission.
void BRTX_Stop_v(void);

// Set the transmitter ID.
// Param1: ID
void BRTX_SetID_v(u32 ID_u32);

// Get the transmitter ID.
// Return: ID
u32  BRTX_GetID_u32(void);

// Pass data to be transmitted in the next packet
// Param1: First 4 bytes
// Param2: Another byte
void BRTX_WriteData_v(u32 FirstData_u32, u8 SecondData_u8);

// Callback routine called just before a new packet is build
void BRTX_PrepareData_v(void);
// Callback routine called after data have been sent
void BRTX_DataTransfered_v(void);


// ----------------------------------------------------------
// Radio-related functions
void BlueRobin_RadioISR_v(void);


#endif                          /*BRTX_API_H_ */

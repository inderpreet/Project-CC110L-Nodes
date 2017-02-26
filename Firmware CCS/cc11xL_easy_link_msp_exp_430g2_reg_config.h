/******************************************************************************
    Filename: cc11xl_easy_link_reg_config.h  
    
    Description: Template for CC11xL register export from SmartRF Studio 
                 
*******************************************************************************/
#ifndef CC11XL_EASY_LINK_REG_CONFIG_H
#define CC11XL_EASY_LINK_REG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * INCLUDES
 */
#include "cc11xL_spi.h"
#include "hal_msp_exp430g2_spi.h"

  
/******************************************************************************
 * FUNCTIONS
 */  

#define PA_TABLE {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}

// Deviation = 4.943848 
// Base frequency = 867.999573 
// Carrier frequency = 867.999573 
// Modulated = true 
// Modulation format = GFSK 
// Manchester enable = false 
// Sync word qualifier mode = 30/32 sync word bits detected 
// Preamble count = 4 
// Channel spacing = 199.813843 
// Carrier frequency = 867.999573 
// Data rate = 1.20056 
// RX filter BW = 60.267857 
// Data format = Normal mode 
// CRC enable = true 
// Device address = 0 
// Address config = No address check 
// CRC autoflush = false 
// PA ramping = false 
// TX power = 12 
static const registerSetting_t preferredSettings[] = {
  {CC110L_IOCFG0,       0x06},
  {CC110L_FIFOTHR,      0x47},
  {CC110L_PKTCTRL0,     0x05},
  {CC110L_FSCTRL1,      0x06},
  {CC110L_FREQ2,        0x20},
  {CC110L_FREQ1,        0x25},
  {CC110L_MDMCFG4,      0xF5},
  {CC110L_MDMCFG3,      0x75},
  {CC110L_MDMCFG2,      0x13},
  {CC110L_MDMCFG0,      0xE5},
  {CC110L_DEVIATN,      0x14},
  {CC110L_MCSM0,        0x18},
  {CC110L_FOCCFG,       0x16},
  {CC110L_RESERVED_0X20,0xFB},
  {CC110L_FSCAL3,       0xE9},
  {CC110L_FSCAL2,       0x2A},
  {CC110L_FSCAL1,       0x00},
  {CC110L_FSCAL0,       0x1F},
  {CC110L_TEST2,        0x81},
  {CC110L_TEST1,        0x35},
  {CC110L_TEST0,        0x09},
  {CC11xL_PA_TABLE0,    0xC0},
};
#ifdef  __cplusplus
}
#endif
/******************************************************************************
  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/
#endif
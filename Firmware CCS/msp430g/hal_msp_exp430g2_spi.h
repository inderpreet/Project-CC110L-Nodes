/**************************************************************************//**
    @file       file name

    @brief      describtion

******************************************************************************/
#ifndef HAL_MSP_EXP430G2_SPI_H
#define HAL_MSP_EXP430G2_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "hal_defs.h"  

#ifndef NOP
#define NOP()  __no_operation()
#endif

/******************************************************************************
 * CONSTANTS
 */
#define     SPI_PORT_SEL          P1SEL
#define     SPI_PORT_DIR          P1DIR
#define     SPI_PORT_OUT          P1OUT
#define     SPI_PORT_IN           P1IN
  
#define     SPI_MOSI_PIN          BIT7
#define     SPI_MISO_PIN          BIT6
#define     SPI_SCLK              BIT5
  
#define     CS_N_PORT_SEL         P2SEL
#define     CS_N_PORT_DIR         P2DIR
#define     CS_N_PORT_OUT         P2OUT
  
#define     CS_N_PIN              BIT7
  
#define     RADIO_BURST_ACCESS    0x40
#define     RADIO_SINGLE_ACCESS   0x00
#define     RADIO_READ_ACCESS     0x80
#define     RADIO_WRITE_ACCESS    0x00  
  
/******************************************************************************
 * MACROS
 */  
#define     SPI_BEGIN()           st( CS_N_PORT_OUT &= ~CS_N_PIN; NOP(); )
#define     SPI_TX(x)             st( IFG2 &= ~UCB0RXIFG; UCB0TXBUF= (x); )
#define     SPI_WAIT_DONE()       st( while(!(IFG2 & UCB0RXIFG)); )
#define     SPI_RX()              UCB0RXBUF
#define     SPI_END()             st( NOP(); CS_N_PORT_OUT |= CS_N_PIN; )  

/******************************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint16  addr;
  uint8   data;
}registerSetting_t;

typedef uint8 rfStatus_t;
/******************************************************************************
 * PROTOTYPES
 */
void exp430RfSpiInit(void);

uint8 trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len);
rfStatus_t trxSpiCmdStrobe(uint8 cmd);

#ifdef  __cplusplus
}
#endif

/******************************************************************************
  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

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

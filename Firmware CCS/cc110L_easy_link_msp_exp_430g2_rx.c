/******************************************************************************
  Filename:        cc110L_easy_link.c
  
  Description: 
  
  Notes: 
  
******************************************************************************/


/*****************************************************************************
* INCLUDES
*/
#include "msp430.h"
#include "hal_board.h"
#include "cc11xL_spi.h"
#include "hal_int_rf_msp_exp430g2.h"
#include "cc11xL_easy_link_msp_exp_430g2_reg_config.h"
#include "stdlib.h"
/******************************************************************************
 * CONSTANTS
 */ 

/******************************************************************************
* DEFINES
*/
#define ISR_ACTION_REQUIRED 1
#define ISR_IDLE            0

#define PKTLEN              30
/******************************************************************************
* LOCAL VARIABLES
*/
static uint8  packetSemaphore;
static uint32 packetCounter;

/******************************************************************************
* STATIC FUNCTIONS
*/
static void registerConfig(void);
static void runRX(void);
static void radioRxTxISR(void);
/******************************************************************************
 * @fn          main
 *
 * @brief       Runs the main routine
 *                
 * @param       none
 *
 * @return      none
 */
void main(void)
{
  //init MCU
  halInitMCU();
  //init LEDs
  halLedInit();
  //init button
  halButtonInit();
  halButtonInterruptEnable();
  // init spi
  exp430RfSpiInit();
  // write radio registers
  registerConfig();

  // run either TX or RX dependent of build define  
  runRX();
 
}
/******************************************************************************
 * @fn          runRX
 *
 * @brief       puts radio in RX and waits for packets. Update packet counter
 *              and display for each packet received.
 *                
 * @param       none
 *
 * @return      none
 */
static void runRX(void)
{
  uint8 rxBuffer[64] = {0};
  uint8 rxBytes;
  uint8 rxBytesVerify;
  
   P2SEL &= ~0x40; // P2SEL bit 6 (GDO0) set to one as default. Set to zero (I/O)
  // connect ISR function to GPIO0, interrupt on falling edge
  trxIsrConnect(GPIO_0, FALLING_EDGE, &radioRxTxISR);
  
  // enable interrupt from GPIO_0
  trxEnableInt(GPIO_0);
     
    
  // set radio in RX
  trxSpiCmdStrobe(CC110L_SRX);

  // reset packet counter
  packetCounter = 0;
  
  // infinite loop
  while(1)
  {
    // wait for packet received interrupt 
    if(packetSemaphore == ISR_ACTION_REQUIRED)
    {
        cc11xLSpiReadReg(CC110L_RXBYTES,&rxBytesVerify,1);
        
        do
        {
          rxBytes = rxBytesVerify;
          cc11xLSpiReadReg(CC110L_RXBYTES,&rxBytesVerify,1);
        }
        while(rxBytes != rxBytesVerify);
        
        cc11xLSpiReadRxFifo(rxBuffer,(rxBytes));
        
        // check CRC ok (CRC_OK: bit7 in second status byte)
        if(rxBuffer[rxBytes-1] & 0x80)
        {
          // toggle led
           halLedToggle(LED1);
          // update packet counter
          packetCounter++;
        }
      
      // reset packet semaphore
      packetSemaphore = ISR_IDLE;
      
      // set radio back in RX
      trxSpiCmdStrobe(CC110L_SRX);
      
    }
  } 
}
/*******************************************************************************
* @fn          radioRxTxISR
*
* @brief       ISR for packet handling in RX. Sets packet semaphore, puts radio
*              in idle state and clears isr flag.
*
* @param       none
*
* @return      none
*/
static void radioRxTxISR(void) {

  // set packet semaphore
  packetSemaphore = ISR_ACTION_REQUIRED;
  // clear isr flag
  trxClearIntFlag(GPIO_0);
}

/*******************************************************************************
* @fn          registerConfig
*
* @brief       Write register settings as given by SmartRF Studio
*
* @param       none
*
* @return      none
*/
static void registerConfig(void) {
  uint8 writeByte;
  uint16 i;
  // reset radio
  trxSpiCmdStrobe(CC110L_SRES);
  // write registers to radio
  for(i = 0; i < (sizeof  preferredSettings/sizeof(registerSetting_t)); i++) {
    writeByte =  preferredSettings[i].data;
    cc11xLSpiWriteReg( preferredSettings[i].addr, &writeByte, 1);
  }
}
/***********************************************************************************
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
***********************************************************************************/

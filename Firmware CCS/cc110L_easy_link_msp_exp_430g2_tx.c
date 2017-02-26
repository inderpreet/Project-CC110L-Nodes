/******************************************************************************
  Filename:        cc110L_easy_link_tx.c
  
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
static void runTX(void);
static void createPacket(uint8 txBuffer[]);
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
  runTX();
 
}
/******************************************************************************
 * @fn          runTX
 *
 * @brief       sends one packet on button push. Updates packet counter and
 *              display for each packet sent.
 *                
 * @param       none
 *
 * @return      none
 */
static void runTX(void)
{
  // Initialize packet buffer of size PKTLEN + 1
  uint8 txBuffer[PKTLEN+1] = {0};

   P2SEL &= ~0x40; // P2SEL bit 6 (GDO0) set to one as default. Set to zero (I/O)
  // connect ISR function to GPIO0, interrupt on falling edge
  trxIsrConnect(GPIO_0, FALLING_EDGE, &radioRxTxISR);
  
  // enable interrupt from GPIO_0
  trxEnableInt(GPIO_0);
  
  // infinite loop
  while(1)
  {
    // wait for button push
    if(halButtonPushed())
    { 
      //continiously sent packets until button is pressed
      do
      {
        // update packet counter
        packetCounter++;
        
        // create a random packet with PKTLEN + 2 byte packet counter + n x random bytes
        createPacket(txBuffer);
      
      // write packet to tx fifo
      cc11xLSpiWriteTxFifo(txBuffer,sizeof(txBuffer));
      
      // strobe TX to send packet
      trxSpiCmdStrobe(CC110L_STX);
      
        // wait for interrupt that packet has been sent. 
        // (Assumes the GPIO connected to the radioRxTxISR function is set 
        // to GPIOx_CFG = 0x06)
        while(!packetSemaphore);
        
        // clear semaphore flag
        packetSemaphore = ISR_IDLE;
        
      halLedToggle(LED1);
       __delay_cycles(250000); 
       halLedToggle(LED1);
        
        
      }while(!halButtonPushed());
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
#ifdef PA_TABLE
  uint8 paTable[] = PA_TABLE;
#endif
  
  uint16 i;
  // reset radio
  trxSpiCmdStrobe(CC110L_SRES);
  // write registers to radio
  for(i = 0; i < (sizeof  preferredSettings/sizeof(registerSetting_t)); i++) {
    writeByte =  preferredSettings[i].data;
    cc11xLSpiWriteReg( preferredSettings[i].addr, &writeByte, 1);
  }
#ifdef PA_TABLE
  // write PA_TABLE
  cc11xLSpiWriteReg(CC11xL_PA_TABLE0,paTable, sizeof(paTable));
#endif
}
/******************************************************************************
 * @fn          createPacket
 *
 * @brief       This function is called before a packet is transmitted. It fills
 *              the txBuffer with a packet consisting of a length byte, two
 *              bytes packet counter and n random bytes.
 *
 *              The packet format is as follows:
 *              |--------------------------------------------------------------|
 *              |           |           |           |         |       |        |
 *              | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
 *              |           |           |           |         |       |        |
 *              |--------------------------------------------------------------|
 *               txBuffer[0] txBuffer[1] txBuffer[2]  ......... txBuffer[PKTLEN]
 *                
 * @param       pointer to start of txBuffer
 *
 * @return      none
 */
static void createPacket(uint8 txBuffer[])
{
    uint8 i;
  txBuffer[0] = PKTLEN;                     // Length byte
  txBuffer[1] = (uint8) packetCounter >> 8; // MSB of packetCounter
  txBuffer[2] = (uint8) packetCounter;      // LSB of packetCounter
  
  // fill rest of buffer with random bytes
  for(i =3; i< (PKTLEN+1); i++)
  {
    txBuffer[i] = (uint8)rand();
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

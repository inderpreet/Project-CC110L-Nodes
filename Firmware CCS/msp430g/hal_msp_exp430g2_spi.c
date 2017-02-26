/******************************************************************************
  Filename:   
  
  Description: 
  
  Notes: 
  
******************************************************************************/


/*****************************************************************************
* INCLUDES
*/
#include "msp430.h"
#include "hal_msp_exp430g2_spi.h"
//#include "hal_board.h"
#include "hal_types.h"
//#include "hal_digio2.h"

/******************************************************************************
* LOCAL VARIABLES
*/


/******************************************************************************
* STATIC FUNCTIONS
*/
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len) ;

/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *                
 * @param       input, output parameters
 *
 * @return      describe return value, if any
*/
void exp430RfSpiInit(void)
{
  /* Configuration
   * -  8-bit
   * -  Master Mode
   * -  3-pin
   * -  synchronous mode
   * -  MSB first
   * -  Clock phase select = captured on first edge
   * -  Inactive state is low
   * -  SMCLK as clock source
   * -  Spi clk is adjusted corresponding to systemClock as the highest rate
   *    supported by the supported radios: this could be optimized and done
   *    after chip detect.
   */

  // 1) Set USCI in reset state
  UCB0CTL1 |= UCSWRST;
    
  // 2) Init USCI registers with UCSWRST = 1
  UCB0CTL0  =  0x00+UCMST + UCSYNC + UCMODE_0 + UCMSB + UCCKPH;
  UCB0CTL1 |=  UCSSEL_3;
  //data rate:
  UCB0BR1   =  0x00;
  UCB0BR0   =  0x01; // division factor of clock source
  // 3) Configure ports
  // MISO -> P1.6
  // MOSI -> P1.7
  // SCLK -> P1.5
  // CS_N -> manualy set. (XOUT P2.7)
  
  // select bit 7, 6 and 5 as peripheral
  P1SEL2 |= BIT7 + BIT6 + BIT5; //Setting function select to second peripheral module
  SPI_PORT_SEL |= SPI_MISO_PIN + SPI_MOSI_PIN + SPI_SCLK;
  // manually set CS_N P2.7
  //P2SEL &= ~BIT7;
  CS_N_PORT_SEL &= ~CS_N_PIN;
  
  // ???????
  //P1OUT |= BIT6; //MISO
  //P2OUT |= BIT7; //CS_N
  SPI_PORT_OUT |= SPI_MISO_PIN;
  CS_N_PORT_OUT |= CS_N_PIN;
  
  
  // Set direction
  //P2DIR |= BIT7; //CS_N
  //P1DIR |= BIT7 + BIT5; // MOSI + SCLCK
  //P1DIR &= ~BIT6; //MISO
  
  CS_N_PORT_DIR |= CS_N_PIN;
  SPI_PORT_DIR |= SPI_MOSI_PIN + SPI_SCLK;
  SPI_PORT_DIR &= ~SPI_MISO_PIN;
  
  // 4) Clear UCSWRST. Release for operation
  UCB0CTL1 &= ~UCSWRST;
  // 5) Enable interrupts UCxRXIE and UCxTXIE
  
}
/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *                
 * @param       input, output parameters
 *
 * @return      describe return value, if any
*/
uint8 trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len)
{
  uint8 readValue;

  //Pull CS_N low and wait for SO to go low before communication starts
  SPI_BEGIN();
  while(SPI_PORT_IN & SPI_MISO_PIN);
  // send register address byte
  SPI_TX(accessType|addrByte);
  SPI_WAIT_DONE();
  // Storing chip status
  readValue = SPI_RX();
  
  trxReadWriteBurstSingle(accessType|addrByte,pData,len);
  SPI_END();
  // return the status byte value */
  return(readValue);
}
/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
rfStatus_t trxSpiCmdStrobe(uint8 cmd)
{
    uint8 rc;
    SPI_BEGIN();
    while(SPI_PORT_IN & SPI_MISO_PIN);
    SPI_TX(cmd);
    SPI_WAIT_DONE();
    rc = SPI_RX();
    SPI_END();
    return(rc);
}
/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              SPI_BEGIN();
 *              while(TRXEM_PORT_IN & SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
 *              SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len)
{
	uint16 i;
	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
  if(addr&RADIO_READ_ACCESS)
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      for (i = 0; i < len; i++)
      {
          SPI_TX(0);            /* Possible to combining read and write as one access type */
          SPI_WAIT_DONE();
          *pData = SPI_RX();     /* Store pData from last pData RX */
          pData++;
      }
    }
    else
    {
      SPI_TX(0);
      SPI_WAIT_DONE();
      *pData = SPI_RX();
    }
  }
  else
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
      for (i = 0; i < len; i++)
      {
        SPI_TX(*pData);
        SPI_WAIT_DONE();
        pData++;
      }
    }
    else
    {
      SPI_TX(*pData);
      SPI_WAIT_DONE();
    }
  }
  return;
}
/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *                
 * @param       input, output parameters
 *
 * @return      describe return value, if any
 ******************************************************************************/

/***********************************************************************************
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
***********************************************************************************/
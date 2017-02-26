/******************************************************************************
    Filename: hal_int_rf_trxeb.c  
    
    Description: Implementation file for radio interrupt interface 
                 functions on Port 1, pin 7. The ISR is defined elsewhere
                 and connected to the interrupt vector real time. 
                 
*******************************************************************************/


/******************************************************************************
 * INCLUDES
 */
#include <msp430.h>
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_int_rf_msp_exp430g2.h"
#include "hal_msp_exp430g2_spi.h"
#include "hal_digio2.h"

/******************************************************************************
* CONSTANTS
*/

/* Interrupt port and pin */
#define TRXEM_INT_PORT_IN P1IN

digio gpio3 = {1,0};
digio gpio2 = {1,0};
digio gpio0 = {2,6};

/******************************************************************************
 * FUNCTIONS
 */
 

/*******************************************************************************
 * @fn          trxIsrConnect
 *
 * @brief       Connects an ISR function to PORT1 interrupt vector and 
 *              configures the interrupt to be a high-low transition. 
 * 
 * input parameters
 *
 * @param       pF  - function pointer to ISR
 *
 * output parameters
 *
 * @return      void
 */ 
void trxIsrConnect(uint8 gpio, uint8 edge, ISR_FUNC_PTR pF)
{
  digio io;
  
  switch(gpio)
  {
   case GPIO_3:
    io = gpio3;
    break;
   case GPIO_2:
    io = gpio2;
    break;
   case GPIO_0:
    io = gpio0;
    break;
   default:
    io = gpio0;
    break;
  }
  
  // Assigning ISR function
  halDigio2IntConnect(io, pF);
  // Setting rising or falling edge trigger
  halDigio2IntSetEdge(io, edge);
  return;
}
/*******************************************************************************
 * @fn          trxClearIntFlag
 *
 * @brief       Clears sync interrupt flag
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
void trxClearIntFlag(uint8 gpio)
{
  digio io;
  
  switch(gpio)
  {
   case GPIO_3:
    io = gpio3;
    break;
   case GPIO_2:
    io = gpio2;
    break;
   case GPIO_0:
    io = gpio0;
    break;
   default:
    io = gpio0;
    break;
  }
  
  halDigio2IntClear(io);
  return;
}

/*******************************************************************************
 * @fn          trxEnableInt
 *
 * @brief       Enables sync interrupt 
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */ 
void trxEnableInt(uint8 gpio)
{
  digio io;
  
  switch(gpio)
  {
   case GPIO_3:
    io = gpio3;
    break;
   case GPIO_2:
    io = gpio2;
    break;
   case GPIO_0:
    io = gpio0;
    break;
   default:
    io = gpio0;
    break;
  }
  
  halDigio2IntEnable(io);
  return;
}

/*******************************************************************************
 * @fn          trxDisableInt
 *
 * @brief       Disables sync interrupt
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */ 
void trxDisableInt(uint8 gpio)
{
  digio io;
  
  switch(gpio)
  {
   case GPIO_3:
    io = gpio3;
    break;
   case GPIO_2:
    io = gpio2;
    break;
   case GPIO_0:
    io = gpio0;
    break;
   default:
    io = gpio0;
    break;
  }
  
  halDigio2IntDisable(io);
  return;
}

/******************************************************************************
 * @fn          trxSampleSyncPin
 *
 * @brief       Reads the value of the sync pin. 
 *                 
 * input parameters
 *   
 * @param       none
 *
 * output parameters
 *
 * @return      uint8
 */
uint8 trxSampleSyncPin(uint8 gpio)
{
  digio io;
  
  switch(gpio)
  {
   case GPIO_3:
    io = gpio3;
    break;
   case GPIO_2:
    io = gpio2;
    break;
   case GPIO_0:
    io = gpio0;
    break;
   default:
    io = gpio0;
    break;
  }
  
  return ((TRXEM_INT_PORT_IN & (0x01<<io.pin))>>io.pin);
}


/******************************************************************************
  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
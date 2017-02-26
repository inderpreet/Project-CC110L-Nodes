/******************************************************************************
  Filename:        hal_board.c
  
  Description: 
  
  Notes: 
  
******************************************************************************/


/*****************************************************************************
* INCLUDES
*/
#include "msp430.h"
#include "hal_board.h"
#include "hal_types.h"
#include "hal_digio2.h"

/******************************************************************************
* LOCAL VARIABLES
*/
static uint8 buttonPressed;

/******************************************************************************
* STATIC FUNCTIONS
*/
static void buttonPressedISR(void);

/******************************************************************************
 * @fn          halInitMCU
 *
 * @brief       Description of the function
 *                
 * @param       input, output parameters
 *
 * @return      describe return value, if any
*/
void halInitMCU(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  
  BCSCTL1 = CALBC1_1MHZ;                    // Set range
  DCOCTL = CALDCO_1MHZ;
  BCSCTL2 &= ~(DIVS_3);                     // SMCLK = DCO = 1MHz  
  
  // Enable global interrupt
  _BIS_SR(GIE);
}

/******************************************************************************
 * @fn          halLedInit
 *
 * @brief       inti both leds as off
 *                
 * @param       input, output parameters
 *
 * @return      describe return value, if any
*/
void halLedInit(void)
{
  LED_DIR |= LED1 + LED2;                          
  LED_OUT &= ~(LED1 + LED2);    
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
void halLedSet(uint8 led_id)
{
  LED_OUT |= led_id;
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
void halLedClear(uint8 led_id)
{
  LED_OUT &= ~(led_id);
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
void halLedToggle(uint8 led_id)
{
  LED_OUT ^= led_id;
}
/******************************************************************************
 * @fn          halButtonInit
 *
 * @brief       Description of the function
 *                
 * @param       input, output parameters
 *
 * @return      describe return value, if any
*/
void halButtonInit(void)
{
  BUTTON_DIR &= ~BUTTON;
  BUTTON_OUT |= BUTTON;
  BUTTON_REN |= BUTTON;
  BUTTON_IES |= BUTTON;
  BUTTON_IFG &= ~BUTTON;
  BUTTON_IE |= BUTTON;  
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
void halButtonInterruptEnable(void)
{
  digio io;
  io.port = 1;
  io.pin  = 3;
  
  halDigio2IntConnect(io, &buttonPressedISR);
}
/*****************************************************************************
 * @fn          halButtonsPushed
 *
 * @brief       Reads the value of buttonsPressed and then RESETS buttonsPressed
 *
 * input parameters
 *
 * output parameters
 *
 * @return      value - Contains the bitmask code for the buttons pushed
 *                      since last call
 */
uint8 halButtonPushed(void)
{
  uint8 value = buttonPressed;
  buttonPressed = BUTTON_IDLE; // Clear button bitmask variable
  return value;
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
static void buttonPressedISR(void)
{
  BUTTON_IFG = 0;  
  BUTTON_IE &= ~BUTTON;            /* Debounce */
  WDTCTL = WDT_MDLY_32;
  IFG1 &= ~WDTIFG;                 /* clear interrupt flag */
  IE1 |= WDTIE;
  
  buttonPressed = BUTTON_PRESSED;
  
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
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    IE1 &= ~WDTIE;                   /* disable interrupt */
    IFG1 &= ~WDTIFG;                 /* clear interrupt flag */
    WDTCTL = WDTPW + WDTHOLD;        /* put WDT back in hold state */
    BUTTON_IE |= BUTTON;             /* Debouncing complete */
}




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
/**************************************************************************//**
    @file       acc_bma250.h

    @brief      Header file for accelerometer BMA250. @Note This header file
                does not include defines for all BMA250 register addresses.

******************************************************************************/
#ifndef ACC_BMA250_H
#define ACC_BMA250_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
 * INCLUDES
 */
#include "bsp.h"


/******************************************************************************
 * DEFINES
 */

// BMA250 addressing space
#define ACC_CHIPID                  0x00    // Always 0x03
#define ACC_X_LSB                   0x02    // ACC_X_LSB[7:6] = 2 LSb of X data
#define ACC_X_MSB                   0x03    // ACC_X_MSB[7:0] = 8 MSb of X data
#define ACC_Y_LSB                   0x04
#define ACC_Y_MSB                   0x05
#define ACC_Z_LSB                   0x06
#define ACC_Z_MSB                   0x07
#define ACC_TEMP                    0x08    // Temperature data
#define ACC_INT_STATUS0             0x09    // Interrupt status register
#define ACC_INT_STATUS1             0x0A
#define ACC_TAP_SLOPE_INT_STATUS    0x0B    // Interrupt detailed status
#define ACC_FLAT_ORIENT_STATUS      0x0C    // Interrupt detailed status
#define ACC_RANGE                   0x0F    // 2/4/8/16 G range
#define ACC_BW                      0x10    // Filtered bandwidth
#define ACC_PWR_MODE                0x11    // Power configuration
#define ACC_DATA_CFG                0x12
#define ACC_CONF_FILT_SHADOW        0x13
#define ACC_SOFTRESET               0x14    // Write 0xB6 for soft reset
#define ACC_INT_EN0                 0x16    // Interrupt enable register
#define ACC_INT_EN1                 0x17
#define ACC_INT1_MAP                0x19    // Interrupt - IO mapping
#define ACC_INT_MAP                 0x1A
#define ACC_INT2_MAP                0x1B
#define ACC_INT_SRC                 0x1E
#define ACC_INT_PIN_CFG             0x20
#define ACC_TAP_INT_TIMING          0x2A
#define ACC_TAP_INT_SAMP_TH         0x2B

// Defines for bit fields in the ACC_INT_STATUS0 register
#define ACC_INT_STATUS0_FLAT_INT    0x80
#define ACC_INT_STATUS0_ORIENT_INT  0x40
#define ACC_INT_STATUS0_S_TAP_INT   0x20
#define ACC_INT_STATUS0_D_TAP_INT   0x10
#define ACC_INT_STATUS0_SLOPE_INT   0x04
#define ACC_INT_STATUS0_HIGH_INT    0x02
#define ACC_INT_STATUS0_LOW_INT     0x01

// Defines for bit fields in the ACC_INT_STATUS1 register
#define ACC_INT_STATUS1_DATA_INT    0x80

// Defines for bit fields in the ACC_xx register

// Defines for bit fields in the ACC_RANGE register
#define ACC_RANGE_M                 0x0F
#define ACC_RANGE_2G                0x03    //  3.91 mg/LSB
#define ACC_RANGE_4G                0x05    //  7.81 mg/LSB
#define ACC_RANGE_8G                0x08    // 15.62 mg/LSB
#define ACC_RANGE_16G               0x0C    // 31.25 mg/LSB

// Defines for bit fields in the ACC_POWER_MODE register
#define ACC_SUSPEND                 0x80    // Set in suspend mode (default 0)
#define ACC_LOWPOWER_EN             0x40    // Enable low-power mode (default 0)
#define ACC_SLEEP_DUR_M             0x1E    // sleep_dur bitmask
#define ACC_SLEEP_DUR_0MS           0x00    // sleep_dur default value
#define ACC_SLEEP_DUR_0_5MS         0x14
#define ACC_SLEEP_DUR_1MS           0x18
#define ACC_SLEEP_DUR_2MS           0x1C
#define ACC_SLEEP_DUR_4MS           0x20
#define ACC_SLEEP_DUR_6MS           0x24
#define ACC_SLEEP_DUR_10MS          0x28
#define ACC_SLEEP_DUR_25MS          0x2C
#define ACC_SLEEP_DUR_50MS          0x30
#define ACC_SLEEP_DUR_100MS         0x34
#define ACC_SLEEP_DUR_500MS         0x38
#define ACC_SLEEP_DUR_1000MS        0x3C

// Defines for bit fields in the ACC_DATA_CFG register
#define ACC_DATA_UNFILTERED         0x80    // Select unfiltered acceleration
                                            // to be written into data regs.
                                            // Default is 0 (filtered data)
#define ACC_DATA_SHADOW_DIS         0x80    // Disable shadownig procedure.
                                            // Default is 0 (shadowing enabled)

// Defines for bit fields in the ACC_TAP_INT_TIMING register
#define ACC_TAP_QUIET_M             0x80
#define ACC_TAP_QUIET_30MS          0x00
#define ACC_TAP_QUIET_20MS          0x80
#define ACC_TAP_SHOCK_M             0x40
#define ACC_TAP_SHOCK_50MS          0x00
#define ACC_TAP_SHOCK_75MS          0x40
#define ACC_TAP_DUR_M               0x07
#define ACC_TAP_DUR_50MS            0x00
#define ACC_TAP_DUR_100MS           0x01
#define ACC_TAP_DUR_150MS           0x02
#define ACC_TAP_DUR_200MS           0x03
#define ACC_TAP_DUR_250MS           0x04
#define ACC_TAP_DUR_375MS           0x05
#define ACC_TAP_DUR_500MS           0x06
#define ACC_TAP_DUR_700MS           0x07

// Defines for bit fields in the ACC_TAP_INT_SAMP_TH register
#define ACC_TAMP_SAMP_M             0xC0
#define ACC_TAP_SAMP_2_SAMPLES      0x00    // The number of samples to be
#define ACC_TAP_SAMP_4_SAMPLES      0x40    // processed after wakeup in
#define ACC_TAP_SAMP_8_SAMPLES      0x80    // low-power mode.
#define ACC_TAP_SAMP_16_SAMPLES     0xC0
#define ACC_TAP_TH_M                0x1F    // Bitmask for tap_th subset of
                                            // register.
                                            // The LSb of tap_th
                                            // corresponds to the following
                                            // acceleration difference:
                                            // 2g  range: 62.5mg
                                            // 4g  range: 125mg
                                            // 8g  range: 250mg
                                            // 16g range: 500mg

// Defines for bit fields in the ACC_BW register
// delta_t = time between successive acc samples
#define ACC_BW_7_81HZ               0x08    // delta_t = 64   ms
#define ACC_BW_15_63HZ              0x09    // delta_t = 32   ms
#define ACC_BW_31_25HZ              0x0A    // delta_t = 16   ms
#define ACC_BW_62_5HZ               0x0B    // delta_t =  8   ms
#define ACC_BW_125HZ                0x0C    // delta_t =  4   ms
#define ACC_BW_250HZ                0x0D    // delta_t =  2   ms
#define ACC_BW_500HZ                0x0E    // delta_t =  1   ms
#define ACC_BW_1000HZ               0x0F    // delta_t =  0.5 ms

// Defines for bit fields in the ACC_INT_EN0 register
#define ACC_INT_EN0_FLAT_EN         0x80
#define ACC_INT_EN0_ORIENT_EN       0x40
#define ACC_INT_EN0_S_TAP_EN        0x20
#define ACC_INT_EN0_D_TAP_EN        0x10
#define ACC_INT_EN0_SLOPE_Z_EN      0x04
#define ACC_INT_EN0_SLOPE_Y_EN      0x02
#define ACC_INT_EN0_SLOPE_X_EN      0x01

// Defines for bit fields in the ACC_INT_EN1 register
#define ACC_INT_EN1_DATA_EN         0x10
#define ACC_INT_EN1_LOW_EN          0x08
#define ACC_INT_EN1_HIGH_Z_EN       0x04
#define ACC_INT_EN1_HIGH_Y_EN       0x02
#define ACC_INT_EN1_HIGH_X_EN       0x01

// Defines for bit fields in the ACC_INT1_MAP and ACC_INT2_MAP registers
#define ACC_INT_MAP_FLAT            0x80
#define ACC_INT_MAP_ORIENT          0x40
#define ACC_INT_MAP_S_TAP           0x20
#define ACC_INT_MAP_D_TAP           0x10
#define ACC_INT_MAP_SLOPE           0x04
#define ACC_INT_MAP_HIGH            0x02
#define ACC_INT_MAP_LOW             0x01

// Defines for bit fields in the ACC_INT_MAP register
#define ACC_INT_MAP_DATA_INT1       0x01    // New data IRQ to pin INT1
#define ACC_INT_MAP_DATA_INT2       0x80    // New data IRQ to pin INT2

// Defines for bit fields in the ACC_INT_SRC register
#define ACC_INT_SRC_DATA_FILT       0x20
#define ACC_INT_SRC_TAP_FILT        0x01
#define ACC_INT_SRC_SLOPE_FILT      0x04
#define ACC_INT_SRC_HIGH_FILT       0x02
#define ACC_INT_SRC_LOW_FILT        0x01

// Defines for bit fields in the ACC_INT_PIN_CFG register
#define ACC_INT_CFG_INT2_OD         0x08    // Select open drive for INT2 pin
#define ACC_INT_CFG_INT2_PUSH_PULL  0x00    // Select open drive for INT1 pin
#define ACC_INT_CFG_INT2_ACTIVE_HI  0x04    // Select active high for INT2 pin
#define ACC_INT_CFG_INT2_ACTIVE_LO  0x00    // Select active low for INT2 pin
#define ACC_INT_CFG_INT1_OD         0x02    // Select open drive for INT1 pin
#define ACC_INT_CFG_INT1_PUSH_PULL  0x00    // Select open drive for INT1 pin
#define ACC_INT_CFG_INT1_ACTIVE_HI  0x01    // Select active high for INT1 pin
#define ACC_INT_CFG_INT1_ACTIVE_LO  0x00    // Select active high for INT1 pin

// Perform soft reset
#define ACC_SOFTRESET_EN            0xB6    // Soft reset by writing 0xB6 to
                                            // softreset register

// R/W Bitmask
#define ACC_READ_M                  0x80


/******************************************************************************
* FUNCTION PROTOTYPES
*/

void accInit(void);
void accReadReg(unsigned char reg, unsigned char *pucVal);
void accWriteReg(unsigned char ucReg, unsigned char ucVal);
void accReadAcc(signed short *pssXVal, signed short *pssYVal, signed short *pssZVal);
void accIntRegister(void (*pFnHandler)(void));
void accIntUnregister(void);
void accIntEnable(void);
void accIntDisable(void);
void accIntClear(void);
void accIntTypeSet(unsigned long ulIntType);



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
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
******************************************************************************/

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // ACC_BMA250_H

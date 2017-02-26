/***********************************************************************************
    Filename: hal_digio2.c

    Copyright 2007-2009 Texas Instruments, Inc.
***********************************************************************************/

#include "msp430.h"
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_board.h"
#include "hal_int.h"
#include "hal_digio.h"
#include "hal_digio2.h"

static ISR_FUNC_PTR port1_isr_tbl[8] = {0};
static ISR_FUNC_PTR port2_isr_tbl[8] = {0};



//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint8 halDigio2IntConnect(digio io, ISR_FUNC_PTR func)
{
    istate_t key;
    HAL_INT_LOCK(key);
    switch (io.port)
    {
        case 1: port1_isr_tbl[io.pin] = func; break;
        case 2: port2_isr_tbl[io.pin] = func; break;
        default: HAL_INT_UNLOCK(key); return(HAL_DIGIO_ERROR);
    }
    halDigio2IntClear(io);
    HAL_INT_UNLOCK(key);
    return(HAL_DIGIO_OK);
}


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint8 halDigio2IntEnable(digio io)
{
    uint8 pin_bitmask = (1 << io.pin);
    switch (io.port)
    {
        case 1: P1IE |= pin_bitmask; break;
        case 2: P2IE |= pin_bitmask; break;
        default: return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint8 halDigio2IntDisable(digio io)
{
    uint8 pin_bitmask = (1 << io.pin);
    switch (io.port)
    {
        case 1: P1IE &= ~pin_bitmask; break;
        case 2: P2IE &= ~pin_bitmask; break;
        default: return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint8 halDigio2IntClear(digio io)
{
    uint8 pin_bitmask = (1 << io.pin);
    switch (io.port)
    {
        case 1: P1IFG &= ~pin_bitmask; break;
        case 2: P2IFG &= ~pin_bitmask; break;
        default: return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint8 halDigio2IntSetEdge(digio io, uint8 edge)
{
    uint8 pin_bitmask = (1 << io.pin);
    switch(edge)
    {
        case HAL_DIGIO_INT_FALLING_EDGE:
            switch(io.port)
            {
                case 1: P1IES |= pin_bitmask; break;
                case 2: P2IES |= pin_bitmask; break;
                default: return(HAL_DIGIO_ERROR);
            }
            break;

         case HAL_DIGIO_INT_RISING_EDGE:
            switch(io.port)
            {
                case 1: P1IES &= ~pin_bitmask; break;
                case 2: P2IES &= ~pin_bitmask; break;
                default: return(HAL_DIGIO_ERROR);
            }
            break;

         default:
            return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR(void)
{
    register uint8 i;
    if (P1IFG)
    {
        for (i = 0; i < 8; i++)
        {
            register const uint8 bitmask = 1 << i;
            if ((P1IFG & bitmask) && (P1IE & bitmask) && (port1_isr_tbl[i] != 0))
            {
                (*port1_isr_tbl[i])();
                P1IFG &= ~bitmask;
            }
        }
        __low_power_mode_off_on_exit();
    }
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR(void)
{
    register uint8 i;
    if (P2IFG)
    {
        for (i = 0; i < 8; i++)
        {
            register const uint8 bitmask = 1 << i;
            if ((P2IFG & bitmask) && (P2IE & bitmask) && (port2_isr_tbl[i] != 0))
            {
                (*port2_isr_tbl[i])();
                P2IFG &= ~bitmask;
            }
        }
        __low_power_mode_off_on_exit();
    }
}


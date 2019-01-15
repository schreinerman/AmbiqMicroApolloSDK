/******************************************************************************

Copyright (C) 2013-2017, Fujitsu Electronics Europe GmbH or a               
subsidiary of Fujitsu Electronics Europe GmbH.  All rights reserved.        
                                                                            
This software, including source code, documentation and related             
materials ("Software"), is owned by Fujitsu Electronics Europe GmbH or      
one of its subsidiaries ("Fujitsu").
                                                                            
If no EULA applies, Fujitsu hereby grants you a personal, non-exclusive,    
non-transferable license to copy, modify, and compile the                   
Software source code solely for use in connection with Fujitsu's            
integrated circuit products.  Any reproduction, modification, translation,  
compilation, or representation of this Software except as specified         
above is prohibited without the express written permission of Fujitsu.      
                                                                            
Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                        
WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                        
BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                             
PARTICULAR PURPOSE. Fujitsu reserves the right to make                      
changes to the Software without notice. Fujitsu does not assume any         
liability arising out of the application or use of the Software or any      
product or circuit described in the Software. Fujitsu does not              
authorize its products for use in any products where a malfunction or       
failure of the Fujitsu product may reasonably be expected to result in      
significant property damage, injury or death ("High Risk Product"). By      
including Fujitsu's product in a High Risk Product, the manufacturer        
of such system or application assumes all risk of such use and in doing     
so agrees to indemnify Fujitsu against all liability.                       

 ******************************************************************************/
/******************************************************************************/
/** \file apollosysctrl.c
 **
 ** A detailed description is available at 
 ** @link apollosysctrlGroup Apollo 1 / 2 System Control description @endlink
 **
 ** History:
 **   - 2017-09-01  V1.0  Manuel Schreiner   First Version
 **   - 2017-10-17  V1.1  Manuel Schreiner   Fixed ApolloSysCtrl48MHz setting 
 **   - 2018-07-06  V1.2  Manuel Schreiner   Updated documentation, 
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **   - 2019-01-11  V1.3  Manuel Schreiner   Added cache control for Apollo2
 **
 *****************************************************************************/
#define __APOLLOSYSCTRL_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "apollosysctrl.h"
#include "mcu.h"
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/


/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/



/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/



/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/


/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/


/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
 ******************************************************************************
 ** \brief  Set the main frequency of the core.
 **
 ** \param enFreq  Can be     Apollo 1:
 **                           ApolloSysCtrl24MHz, ApolloSysCtrl12MHz, ApolloSysCtrl8MHz,
 **                           ApolloSysCtrl6MHz,  ApolloSysCtrl4_8MHzA, ApolloSysCtrl4MHz,  
 **                           ApolloSysCtrl3_4MHz, ApolloSysCtrl3MHz 
 **
 **                           Apollo2:
 **                           ApolloSysCtrl28MHz, ApolloSysCtrl24MHz, 
 **
 ******************************************************************************/
void ApolloSysCtrl_SetClk(en_apollosysctrl_freq_t enFreq)
{
    CLKGEN->CLKKEY = 0x47;
    switch(enFreq)
    {
        #if defined(APOLLO2_H)
        case     ApolloSysCtrl48MHz:
            CLKGEN->CCTRL_b.CORESEL = APOLLOSYSCTRL_48MHZ;
            break;
        #endif
        case     ApolloSysCtrl24MHz:
            CLKGEN->CCTRL_b.CORESEL = APOLLOSYSCTRL_24MHZ;
            break;
        #if defined(APOLLO_H) || defined(APOLLO1_H)
            case     ApolloSysCtrl12MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_12MHZ;
                break;
            case     ApolloSysCtrl8MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_8MHZ;
                break;
            case     ApolloSysCtrl6MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_6MHZ;
                break;
            case     ApolloSysCtrl4_8MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_4_8MHZ;
                break;
            case     ApolloSysCtrl4MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_4MHZ;
                break;
            case     ApolloSysCtrl3_4MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_3_4MHZ;
                break;
            case     ApolloSysCtrl3MHz:
                CLKGEN->CCTRL_b.CORESEL  = APOLLOSYSCTRL_3MHZ;
                break;
      #endif
      default:
      break;
    }
    CLKGEN->CLKKEY = 0;
}

/**
 ******************************************************************************
 ** \brief  Enable / Disable core bucks
 **
 ** \param bOnOff  Turn on / off.
 **
 ** \return Ok on success, ErrorTimeout on error
 ******************************************************************************/
en_result_t ApolloSysCtrl_EnableBucks(boolean_t bOnOff)
{
    volatile uint32_t u32Timeout;
    if (TRUE == bOnOff)
    {
        #if defined(APOLLO_H) || defined(APOLLO1_H)
            MCUCTRL->SUPPLYSRC_b.COREBUCKEN = 1;
            MCUCTRL->SUPPLYSRC_b.MEMBUCKEN = 1;
            
            u32Timeout = 100000;
            while((MCUCTRL->SUPPLYSTATUS_b.COREBUCKON == 0) && (u32Timeout > 0)) u32Timeout--;
            
            if (u32Timeout == 0) 
                return ErrorTimeout;
            
            u32Timeout = 100000;
            while((MCUCTRL->SUPPLYSTATUS_b.MEMBUCKON == 0) && (u32Timeout > 0)) u32Timeout--;
            
            if (u32Timeout == 0) 
                return ErrorTimeout;
        #elif defined(APOLLO2_H)
            PWRCTRL->SUPPLYSRC_b.COREBUCKEN = 1;
            PWRCTRL->SUPPLYSRC_b.MEMBUCKEN = 1;
            
            u32Timeout = 100000;
            while((PWRCTRL->POWERSTATUS_b.COREBUCKON != 1) && (u32Timeout > 0)) u32Timeout--;
            
            if (u32Timeout == 0) 
                return ErrorTimeout;
            
            u32Timeout = 100000;
            while((PWRCTRL->POWERSTATUS_b.MEMBUCKON != 1) && (u32Timeout > 0)) u32Timeout--;
            
            if (u32Timeout == 0) 
                return ErrorTimeout;
            
            PWRCTRL->SUPPLYSRC_b.SWITCH_LDO_IN_SLEEP = 0; // For lowest deep sleep power, make sure we stay in BUCK mode.
        #endif
    }
    return Ok;
}

/**
 ******************************************************************************
 ** \brief  Check if bucks are enabled
 **
 ** \return TRUE if enabled, FALSE if disabled
 ******************************************************************************/
boolean_t ApolloSysCtrl_BucksAreEnabled(void)
{
    #if defined(APOLLO_H) || defined(APOLLO1_H)
        if (MCUCTRL->SUPPLYSTATUS_b.COREBUCKON == 0) return FALSE;
        if (MCUCTRL->SUPPLYSTATUS_b.MEMBUCKON == 0)  return FALSE;
    #elif defined(APOLLO2_H)
        if (PWRCTRL->POWERSTATUS_b.MEMBUCKON == 0) return FALSE;
        if (PWRCTRL->POWERSTATUS_b.COREBUCKON == 0)  return FALSE;
    #endif
    return TRUE;
}


/**
 ******************************************************************************
 ** \brief  Get clock frequency
 **         Uses the CMSIS API to get the current clock frequency
 **
 ** \return clock frequency in Hz
 ******************************************************************************/
uint32_t ApolloSysCtrl_GetClk(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
 ******************************************************************************
 ** \brief  Check if XT clock fails
 **
 ** \return TRUE on failture
 ******************************************************************************/
boolean_t ApolloSysCtrl_XtFaiture(void)
{
 return (CLKGEN->STATUS_b.OSCF == 1);
}

/**
 ******************************************************************************
 ** \brief  Turn on/off the autocalbration failture interrupt
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_AutoCalFailIrqEnable(boolean_t bOnOff)
{
#if defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)
    CLKGEN->INTEN_b.ACF = bOnOff;
#else
    CLKGEN->INTRPTEN_b.ACF = bOnOff;
#endif
}

/**
 ******************************************************************************
 ** \brief  Turn on/off the autocalbration complete interrupt
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_AutoCalCompleteIrqEnable(boolean_t bOnOff)
{
#if defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)
    CLKGEN->INTEN_b.ACC = bOnOff;
#else
    CLKGEN->INTRPTEN_b.ACC = bOnOff;
#endif
}

/**
 ******************************************************************************
 ** \brief  Turn on/off the XT failture interrupt
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_OscXtFailIrqEnable(boolean_t bOnOff)
{
#if defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)
    CLKGEN->INTEN_b.OF = bOnOff;
#else
    CLKGEN->INTRPTEN_b.OF = bOnOff;
#endif
}

#if defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)
/**
 ******************************************************************************
 ** \brief  Turn on/off the RTC alarm interrupt
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_RtcAlarmIrqEnable(boolean_t bOnOff)
{
    CLKGEN->INTEN_b.ALM = bOnOff;
}
#endif

/**
 ******************************************************************************
 ** \brief  Turn on/off the XT clock
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_XtEnable(boolean_t bOnOff)
{
    if (TRUE == bOnOff)
    {
        CLKGEN->OCTRL_b.STOPXT = 0;
    } else
    {
        CLKGEN->OCTRL_b.STOPXT = 1;
    }
}

/**
 ******************************************************************************
 ** \brief  Turn on/off the LFRC clock
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_LfrcEnable(boolean_t bOnOff)
{
    if (TRUE == bOnOff)
    {
        CLKGEN->OCTRL_b.STOPRC = 0;
    } else
    {
        CLKGEN->OCTRL_b.STOPRC = 1;
    }
}

/**
 ******************************************************************************
 ** \brief  Turn on/off the switch to LFRC on faiture feature
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_XtSwitchToLfrcOnFaitureEnable(boolean_t bOnOff)
{
    CLKGEN->OCTRL_b.FOS = bOnOff;
}

/**
 ******************************************************************************
 ** \brief  Select the RTC clock source
 **
 ** \param enClockSrc  Can be ApolloSysCtrlRtcXt or ApolloSysCtrlRtcLfrc
 **
 ******************************************************************************/
void ApolloSysCtrl_RtcClockInput(en_apollosysctrl_rtcclk_t enClockSrc)
{
    if (enClockSrc == ApolloSysCtrlRtcXt)
    {
        CLKGEN->OCTRL_b.OSEL = 0;
    } else
    {
        CLKGEN->OCTRL_b.OSEL = 1;
    }
}

#if defined(APOLLO2_H)
/**
 ******************************************************************************
 ** \brief  Enable cache for caching flash data and instructions
 **
 ******************************************************************************/
en_result_t ApolloSysctrl_EnableCache(void)
{
    
    uint32_t u32Conf;
    volatile uint32_t u32Timeout;
    u32Conf = _VAL2FLD(CACHECTRL_CACHECFG_ENABLE,1) | 
              _VAL2FLD(CACHECTRL_CACHECFG_LRU,1) |
              _VAL2FLD(CACHECTRL_CACHECFG_ENABLE_NC0,0) |
              _VAL2FLD(CACHECTRL_CACHECFG_ENABLE_NC1,0) |
              _VAL2FLD(CACHECTRL_CACHECFG_CONFIG,5) |
              _VAL2FLD(CACHECTRL_CACHECFG_SERIAL,0) |
              _VAL2FLD(CACHECTRL_CACHECFG_CACHE_CLKGATE,1) |
              _VAL2FLD(CACHECTRL_CACHECFG_CACHE_LS,0) |
              _VAL2FLD(CACHECTRL_CACHECFG_DLY,1) |
              _VAL2FLD(CACHECTRL_CACHECFG_SMDLY,1) |
              _VAL2FLD(CACHECTRL_CACHECFG_DATA_CLKGATE,1) |
              _VAL2FLD(CACHECTRL_CACHECFG_ENABLE_MONITOR,0);
    PWRCTRL->MEMEN |= _VAL2FLD(PWRCTRL_MEMEN_CACHEB0,1) | _VAL2FLD(PWRCTRL_MEMEN_CACHEB2,1);
    CACHECTRL->CACHECFG = u32Conf;
    for (u32Timeout = 0; u32Timeout < 50; u32Timeout++)
    {
        if (CACHECTRL->CACHECTRL_b.CACHE_READY)
        {
            break;
        }
    }

    CACHECTRL->CACHECTRL_b.INVALIDATE = 1;

    for (u32Timeout = 0; u32Timeout < 50; u32Timeout++)
    {
        if (CACHECTRL->CACHECTRL_b.CACHE_READY)
        {
            break;
        }
    }

    u32Conf |= _VAL2FLD(CACHECTRL_CACHECFG_ICACHE_ENABLE,1) | _VAL2FLD(CACHECTRL_CACHECFG_DCACHE_ENABLE,1);

    CACHECTRL->CACHECFG = u32Conf;

    if (CACHECTRL->CACHECTRL_b.CACHE_READY == 0)
    {
        return Error;
    }
    return Ok;
}
#endif

#if defined(APOLLO_H) || defined(APOLLO1_H)
/**
 ******************************************************************************
 ** \brief  Turn on/off the bandgap for Apollo 1
 **
 ** \param bOnOff  Turn on / off.
 **
 ******************************************************************************/
void ApolloSysCtrl_BandgapEnable(boolean_t bOnOff)
{
    MCUCTRL->BANDGAPEN_b.BGPEN = bOnOff;
}
#endif

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


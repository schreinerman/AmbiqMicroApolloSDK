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
/** \file apollosysctrl.h
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
 **   - 2019-03-25  V1.4  Manuel Schreiner   Added __APOLLOSYSCTRL_VERSION__ and __APOLLOSYSCTRL_DATE__ defines
 *****************************************************************************/
#ifndef __APOLLOSYSCTRL_H__
#define __APOLLOSYSCTRL_H__
#define __APOLLOSYSCTRL_VERSION__  14
#define __APOLLOSYSCTRL_DATE__     "2019-03-25"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup apollosysctrlGroup Low-Level-Driver for Apollo 1/2 System Control
 **
 ** Provided functions of apollosysctrl:
 ** - ApolloSysCtrl_SetClk()                        - set sytem clock
 ** - ApolloSysCtrl_EnableBucks()                   - enable buck converters
 ** - ApolloSysCtrl_BucksAreEnabled()               - check buck converters are enabled
 ** - ApolloSysCtrl_GetClk()                        - get system clock (using CMSIS)
 ** - ApolloSysCtrl_XtFaiture()                     - check if XT clock has a failture
 ** - ApolloSysCtrl_AutoCalFailIrqEnable()          - enable/disable auto calculation failture IRQ
 ** - ApolloSysCtrl_AutoCalCompleteIrqEnable()      - enable/disable auto calculation completion IRQ
 ** - ApolloSysCtrl_OscXtFailIrqEnable()            - enable/disable XT oscillator failture IRQ
 ** - ApolloSysCtrl_RtcAlarmIrqEnable()             - enable/disable RTC alarm IRQ
 ** - ApolloSysCtrl_XtEnable()                      - enable/disable XT oscillator
 ** - ApolloSysCtrl_LfrcEnable()                    - enable/disable LFRC oscillator
 ** - ApolloSysCtrl_XtSwitchToLfrcOnFaitureEnable() - enable/disable swicth to LFRC on XT oscillator failture
 ** - ApolloSysCtrl_RtcClockInput()                 - select RTC input clock
 ** 
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page apollosysctrl_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "apollosysctrl.h"   
 ** @endcode
 **
 ******************************************************************************/
    
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcu.h"
#include "base_types.h"
#if (APOLLOGPIO_ENABLED == 1)
#include "apollogpio.h"
#endif
    
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#if defined(APOLLO_H) || defined(APOLLO1_H)
    #define APOLLOSYSCTRL_24MHZ  0x00000000
    #define APOLLOSYSCTRL_12MHZ  0x00000001
    #define APOLLOSYSCTRL_8MHZ   0x00000002
    #define APOLLOSYSCTRL_6MHZ   0x00000003
    #define APOLLOSYSCTRL_4_8MHZ 0x00000004
    #define APOLLOSYSCTRL_4MHZ   0x00000005
    #define APOLLOSYSCTRL_3_4MHZ 0x00000006
    #define APOLLOSYSCTRL_3MHZ   0x00000007
#elif defined(APOLLO2_H)
    #define APOLLOSYSCTRL_48MHZ  0x00000000
    #define APOLLOSYSCTRL_24MHZ  0x00000001 
#elif defined(APOLLO3_H)
    #define APOLLOSYSCTRL_48MHZ  0x00000000
    #define APOLLOSYSCTRL_24MHZ  0x00000001 
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                        */
/*****************************************************************************/

typedef enum en_apollosysctrl_freq
{
#if defined(APOLLO2_H)
    ApolloSysCtrl48MHz = APOLLOSYSCTRL_48MHZ,
#endif
    ApolloSysCtrl24MHz = APOLLOSYSCTRL_24MHZ,
#if defined(APOLLO_H) || defined(APOLLO1_H)
    ApolloSysCtrl12MHz = APOLLOSYSCTRL_12MHZ,
    ApolloSysCtrl8MHz = APOLLOSYSCTRL_8MHZ,
    ApolloSysCtrl6MHz = APOLLOSYSCTRL_6MHZ,
    ApolloSysCtrl4_8MHz = APOLLOSYSCTRL_4_8MHZ,
    ApolloSysCtrl4MHz = APOLLOSYSCTRL_4MHZ,
    ApolloSysCtrl3_4MHz = APOLLOSYSCTRL_3_4MHZ,
    ApolloSysCtrl3MHz = APOLLOSYSCTRL_3MHZ
#endif
} en_apollosysctrl_freq_t;

typedef enum en_apollosysctrl_rtcclk
{
    ApolloSysCtrlRtcXt,
    ApolloSysCtrlRtcLfrc
} en_apollosysctrl_rtcclk_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/



/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void ApolloSysCtrl_SetClk(en_apollosysctrl_freq_t enFreq);
en_result_t ApolloSysCtrl_EnableBucks(boolean_t bOnOff);
boolean_t ApolloSysCtrl_BucksAreEnabled(void);
uint32_t ApolloSysCtrl_GetClk(void);
boolean_t ApolloSysCtrl_XtFaiture(void);
void ApolloSysCtrl_AutoCalFailIrqEnable(boolean_t bOnOff);
void ApolloSysCtrl_AutoCalCompleteIrqEnable(boolean_t bOnOff);
void ApolloSysCtrl_OscXtFailIrqEnable(boolean_t bOnOff);
void ApolloSysCtrl_RtcAlarmIrqEnable(boolean_t bOnOff);
void ApolloSysCtrl_XtEnable(boolean_t bOnOff);
void ApolloSysCtrl_LfrcEnable(boolean_t bOnOff);
void ApolloSysCtrl_XtSwitchToLfrcOnFaitureEnable(boolean_t bOnOff);
void ApolloSysCtrl_RtcClockInput(en_apollosysctrl_rtcclk_t enClockSrc);
#if defined(APOLLO2_H)
en_result_t ApolloSysctrl_EnableCache(void);
#endif

#ifdef __cplusplus
}
#endif

//@} // apollosysctrlGroup

#endif /*__APOLLOSYSCTRL_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


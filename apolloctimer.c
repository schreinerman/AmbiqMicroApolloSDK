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
/** \file ApolloCTimer.c
 **
 ** A detailed description is available at 
 ** @link ApolloCTimerGroup Apollo CTIMER Implementation description @endlink
 **
 ** History:
 **   - 2017-05-16  V1.0  Manuel Schreiner   First Version
 **   - 2017-06-20  V1.1  Manuel Schreiner   Updated general initialization routine
 **   - 2018-07-06  V1.2  Manuel Schreiner   Updated documentation, 
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **                                          added high/low state using level and no PWM
 **   - 2019-03-25  V1.3  Manuel Schreiner   Added __APOLLOCTIMER_VERSION__ and __APOLLOCTIMER_DATE__ defines
 **
 *****************************************************************************/
#define __APOLLOCTIMER_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "apolloctimer.h"
#include "mcu.h"

#if (CTIMER0_ENABLED == 1) || (CTIMER1_ENABLED == 1) || (CTIMER2_ENABLED == 1) || (CTIMER3_ENABLED == 1) || (APOLLOCTIMER_ENABLED == 1)
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/


/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

const stc_apolloctimer_timer_ab_t stcCTimerA0 = {CTIMER0,CTimerA};
const stc_apolloctimer_timer_ab_t stcCTimerB0 = {CTIMER0,CTimerB};
const stc_apolloctimer_timer_ab_t stcCTimerA1 = {CTIMER1,CTimerA};
const stc_apolloctimer_timer_ab_t stcCTimerB1 = {CTIMER1,CTimerB};
const stc_apolloctimer_timer_ab_t stcCTimerA2 = {CTIMER2,CTimerA};
const stc_apolloctimer_timer_ab_t stcCTimerB2 = {CTIMER2,CTimerB};
const stc_apolloctimer_timer_ab_t stcCTimerA3 = {CTIMER3,CTimerA};
const stc_apolloctimer_timer_ab_t stcCTimerB3 = {CTIMER3,CTimerB};

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/



/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/



/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

#if (APOLLOGPIO_ENABLED == 1)
static stc_apolloctimer_timer_ab_t* GetTimerFromPin(int pin);
#endif

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

#if (APOLLOGPIO_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  Get Timer from pin (use timer on GPIO Func 2 only)
 **
 ** \param pin  Can be every pin name defined PIN_GPIO<n> <n=0..49>, 
 **             but with PWM functionallity, for example PIN_GPIO1, PIN_GPIO2, ... PIN_GPIO49
 **
 ******************************************************************************/
static stc_apolloctimer_timer_ab_t* GetTimerFromPin(int pin)
{
    #if defined(APOLLO_H) ||  defined(APOLLO1_H) 
        switch(pin)
        {
            case PIN_GPIO12:
                return CTIMERA0;
            case PIN_GPIO13:
                return CTIMERB0;
            case PIN_GPIO18:
                return CTIMERA1;  
            case PIN_GPIO19:
                return CTIMERB1;
            case PIN_GPIO20:
                return CTIMERA2;
            case PIN_GPIO21:
                return CTIMERB2;
            case PIN_GPIO22:
                return CTIMERA3;
            case PIN_GPIO23:
                return CTIMERB3;
            case PIN_GPIO25:
                return CTIMERA0;
            case PIN_GPIO26:
                return CTIMERB0;
            case PIN_GPIO27:
                return CTIMERA1;  
            case PIN_GPIO28:
                return CTIMERB1;
            case PIN_GPIO29:
                return CTIMERA2;
            case PIN_GPIO30:
                return CTIMERB2;
            case PIN_GPIO31:
                return CTIMERA3;
            case PIN_GPIO32:
                return CTIMERB3;
            case PIN_GPIO42:
                return CTIMERA0;
            case PIN_GPIO43:
                return CTIMERB0;
            case PIN_GPIO44:
                return CTIMERA1;  
            case PIN_GPIO45:
                return CTIMERB1;
            case PIN_GPIO46:
                return CTIMERA2;
            case PIN_GPIO47:
                return CTIMERB2;
            case PIN_GPIO48:
                return CTIMERA3;
            case PIN_GPIO49:
                return CTIMERB3;
            default:
                return NULL;
        }
    #elif defined(APOLLO2_H) 
        switch(pin)
        {
            case PIN_GPIO12:
                return CTIMERA0;
            case PIN_GPIO13:
                return CTIMERB0;
            case PIN_GPIO18:
                return CTIMERA1;  
            case PIN_GPIO19:
                return CTIMERB1;
            case PIN_GPIO20:
                return CTIMERA2;
            case PIN_GPIO21:
                return CTIMERB2;
            case PIN_GPIO22:
                return CTIMERA3;
            case PIN_GPIO23:
                return CTIMERB3;
            case PIN_GPIO25:
                return CTIMERA0;
            case PIN_GPIO26:
                return CTIMERB0;
            case PIN_GPIO27:
                return CTIMERA1;  
            case PIN_GPIO28:
                return CTIMERB1;
            case PIN_GPIO29:
                return CTIMERA2;
            case PIN_GPIO30:
                return CTIMERB2;
            case PIN_GPIO31:
                return CTIMERA3;
            case PIN_GPIO32:
                return CTIMERB3;
            case PIN_GPIO42:
                return CTIMERA0;
            case PIN_GPIO43:
                return CTIMERB0;
            case PIN_GPIO44:
                return CTIMERA1;  
            case PIN_GPIO45:
                return CTIMERB1;
            case PIN_GPIO46:
                return CTIMERA2;
            case PIN_GPIO47:
                return CTIMERB2;
            case PIN_GPIO48:
                return CTIMERA3;
            case PIN_GPIO49:
                return CTIMERB3;
            default:
            	return NULL;
        }
    #else
        return NULL;
    #endif
}
#endif

#if (APOLLOGPIO_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  Init PWM by pin (use timer on GPIO Func 2 only)
 **
 ** \param pin  Can be every pin name defined PIN_GPIO<n> <n=0..49>, 
 **             but with PWM functionallity, for example PIN_GPIO1, PIN_GPIO2, ... PIN_GPIO49
 **
 ******************************************************************************/
void ApolloCTimer_PwmInitByPin(int pin)
{
    stc_apolloctimer_timer_ab_t* pstcHandle;
    pstcHandle = GetTimerFromPin(pin);
    if (pstcHandle == NULL) return;
    ApolloGpio_GpioOutputConfiguration(pin,GpioPushPull);
    ApolloGpio_GpioSet(pin,TRUE);
    ApolloGpio_GpioSelectFunction(pin,2);
    ApolloCTimer_PwmInit(pstcHandle);
}
#endif

/**
 ******************************************************************************
 ** \brief  Init CTimer
 **
 ** \param pstcHandle  Can be CTIMERA0, CTIMERB0, CTIMERA1, CTIMERB1
 **                           CTIMERA2, CTIMERB2, CTIMERA3, CTIMERB3
 **
 ** \param pstcConfig  Timer configuration
 **
 ******************************************************************************/
void ApolloCTimer_Init(stc_apolloctimer_timer_ab_t* pstcHandle, stc_apolloctimer_config_t* pstcConfig)
{
    if (pstcHandle == NULL) return;
    if (pstcConfig->bLinkABTimers32bit)
    {
        pstcHandle->HANDLE->CTRL_b.CTLINK = 1;
    }
    if (pstcHandle->enTimerAB == CTimerA)
    {
        pstcHandle->HANDLE->CTRL_b.TMRACLR = 1;         //clear timer
    
        pstcHandle->HANDLE->CTRL_b.TMRAPE = pstcConfig->bOutputEnable;          //enable output timer
        pstcHandle->HANDLE->CTRL_b.TMRAPOL = pstcConfig->bInvertPolarity;
        #if defined(APOLLO_H) || defined(APOLLO1_H)
            pstcHandle->HANDLE->CTRL_b.TMRAIE = pstcConfig->bInterruptEnableCompare;          //enable IRQ timer
        #elif defined(APOLLO2_H)
            pstcHandle->HANDLE->CTRL_b.TMRAIE0 = pstcConfig->bInterruptEnableCompare0;          //enable IRQ timer 
            pstcHandle->HANDLE->CTRL_b.TMRAIE1 = pstcConfig->bInterruptEnableCompare1;          //enable IRQ timer        
        #endif
        pstcHandle->HANDLE->CTRL_b.TMRAFN = (uint8_t)pstcConfig->enFunction;          //repeated pulse count timer
        pstcHandle->HANDLE->CTRL_b.TMRACLK = (uint8_t)pstcConfig->enClockInput;      //HFRC div 128 (512 for Apollo 2)
        
        pstcHandle->HANDLE->CMPRA_b.CMPR0A = 128;        //periode - on-time timer
        pstcHandle->HANDLE->CMPRA_b.CMPR1A = 1024;       //on-time timer 
        
        pstcHandle->HANDLE->CTRL_b.TMRACLR = 0;         //release clear timer 
        pstcHandle->HANDLE->CTRL_b.TMRAEN = 1;          //start timer 
    } else if (pstcHandle->enTimerAB == CTimerB)
    {
        
        pstcHandle->HANDLE->CTRL_b.TMRBCLR = 1;         //clear timer
    
        pstcHandle->HANDLE->CTRL_b.TMRBPE = pstcConfig->bOutputEnable;          //enable output timer
        pstcHandle->HANDLE->CTRL_b.TMRBPOL = pstcConfig->bInvertPolarity;
        #if defined(APOLLO_H) || defined(APOLLO1_H)
            pstcHandle->HANDLE->CTRL_b.TMRBIE = pstcConfig->bInterruptEnableCompare;          //enable IRQ timer
        #elif defined(APOLLO2_H)
            pstcHandle->HANDLE->CTRL_b.TMRBIE0 = pstcConfig->bInterruptEnableCompare0;          //enable IRQ timer 
            pstcHandle->HANDLE->CTRL_b.TMRBIE1 = pstcConfig->bInterruptEnableCompare1;          //enable IRQ timer        
        #endif
        pstcHandle->HANDLE->CTRL_b.TMRBFN = (uint8_t)pstcConfig->enFunction;          //repeated pulse count timer
        pstcHandle->HANDLE->CTRL_b.TMRBCLK = (uint8_t)pstcConfig->enClockInput;      //HFRC div 128 (512 for Apollo 2)
        
        pstcHandle->HANDLE->CMPRB_b.CMPR0B = 128;         //periode - on-time timer
        pstcHandle->HANDLE->CMPRB_b.CMPR1B = 1024;       //on-time timer 
        
        pstcHandle->HANDLE->CTRL_b.TMRBCLR = 0;         //release clear timer 
        pstcHandle->HANDLE->CTRL_b.TMRBEN = 1;          //start timer 
    }
}

/**
 ******************************************************************************
 ** \brief  Start CTimer
 **
 ** \param pstcHandle  Can be CTIMERA0, CTIMERB0, CTIMERA1, CTIMERB1
 **                           CTIMERA2, CTIMERB2, CTIMERA3, CTIMERB3
 **
 ******************************************************************************/
void ApolloCTimer_Start(stc_apolloctimer_timer_ab_t* pstcHandle)
{
    if (pstcHandle == NULL) return;
    if (pstcHandle->enTimerAB == CTimerA)
    {
        pstcHandle->HANDLE->CTRL_b.TMRACLR = 0;         //release clear timer 
        pstcHandle->HANDLE->CTRL_b.TMRAEN = 1;          //start timer 
    } else if (pstcHandle->enTimerAB == CTimerB)
    {
        pstcHandle->HANDLE->CTRL_b.TMRBCLR = 0;         //release clear timer 
        pstcHandle->HANDLE->CTRL_b.TMRBEN = 1;          //start timer 
    }
}

/**
 ******************************************************************************
 ** \brief  Disable CTimer
 **
 ** \param pstcHandle  Can be CTIMERA0, CTIMERB0, CTIMERA1, CTIMERB1
 **                           CTIMERA2, CTIMERB2, CTIMERA3, CTIMERB3
 **
 ******************************************************************************/
void ApolloCTimer_Disable(stc_apolloctimer_timer_ab_t* pstcHandle)
{
    if (pstcHandle == NULL) return;
    if (pstcHandle->enTimerAB == CTimerA)
    {
        pstcHandle->HANDLE->CTRL_b.TMRAEN = 0;          //stop timer 
    } else if (pstcHandle->enTimerAB == CTimerB)
    {
        pstcHandle->HANDLE->CTRL_b.TMRBEN = 0;          //stop timer 
    }
}

/**
 ******************************************************************************
 ** \brief  Init PWM
 **
 ** \param pstcHandle  Can be CTIMERA0, CTIMERB0, CTIMERA1, CTIMERB1
 **                           CTIMERA2, CTIMERB2, CTIMERA3, CTIMERB3
 **
 ******************************************************************************/
void ApolloCTimer_PwmInit(stc_apolloctimer_timer_ab_t* pstcHandle)
{
    if (pstcHandle == NULL) return;
    if (pstcHandle->enTimerAB == CTimerA)
    {
        pstcHandle->HANDLE->CTRL_b.TMRACLR = 1;         //clear timer
    
        pstcHandle->HANDLE->CTRL_b.TMRAPE = 1;          //enable output timer
        #if defined(APOLLO_H) || defined(APOLLO1_H)
            pstcHandle->HANDLE->CTRL_b.TMRAIE = 1;          //enable IRQ timer
        #elif defined(APOLLO2_H)
            pstcHandle->HANDLE->CTRL_b.TMRAIE1 = 1;          //enable IRQ timer        
        #endif
        pstcHandle->HANDLE->CTRL_b.TMRAFN = 3;          //repeated pulse count timer
        pstcHandle->HANDLE->CTRL_b.TMRACLK = 0x03;      //HFRC div 128 (512 for Apollo 2)
        
        pstcHandle->HANDLE->CMPRA_b.CMPR0A = 1;         //periode - on-time timer
        pstcHandle->HANDLE->CMPRA_b.CMPR1A = 1024;       //on-time timer 
        
        pstcHandle->HANDLE->CTRL_b.TMRACLR = 0;         //release clear timer 
        pstcHandle->HANDLE->CTRL_b.TMRAEN = 1;          //start timer 
    } else if (pstcHandle->enTimerAB == CTimerB)
    {
        
        pstcHandle->HANDLE->CTRL_b.TMRBCLR = 1;         //clear timer
    
        pstcHandle->HANDLE->CTRL_b.TMRBPE = 1;          //enable output timer
        #if defined(APOLLO_H) || defined(APOLLO1_H)
            pstcHandle->HANDLE->CTRL_b.TMRBIE = 1;          //enable IRQ timer
        #elif defined(APOLLO2_H)
            pstcHandle->HANDLE->CTRL_b.TMRBIE1 = 1;          //enable IRQ timer        
        #endif
        pstcHandle->HANDLE->CTRL_b.TMRBFN = 3;          //repeated pulse count timer
        pstcHandle->HANDLE->CTRL_b.TMRBCLK = 0x03;      //HFRC div 128 (512 for Apollo 2)
        
        pstcHandle->HANDLE->CMPRB_b.CMPR0B = 1;          //periode - on-time timer
        pstcHandle->HANDLE->CMPRB_b.CMPR1B = 1024;       //on-time timer 
        
        pstcHandle->HANDLE->CTRL_b.TMRBCLR = 0;         //release clear timer 
        pstcHandle->HANDLE->CTRL_b.TMRBEN = 1;          //start timer 
    }
}

#if (APOLLOGPIO_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  Disable CTimer by pin
 **
 ** \param pin  Can be every pin name defined PIN_GPIO<n> <n=0..49>, 
 **             but with PWM functionallity, for example PIN_GPIO1, PIN_GPIO2, ... PIN_GPIO49
 **
 ******************************************************************************/
void ApolloCTimer_DisableByPin(int pin)
{
    stc_apolloctimer_timer_ab_t* pstcHandle;
    pstcHandle = GetTimerFromPin(pin);
    ApolloCTimer_Disable(pstcHandle);
}
/**
 ******************************************************************************
 ** \brief  Set the duty cycle of a PWM by pin (use timer on GPIO Func 2 only)
 **
 ** \param pin  Can be every pin name defined PIN_GPIO<n> <n=0..49>, 
 **             but with PWM functionallity, for example PIN_GPIO1, PIN_GPIO2, ... PIN_GPIO49
 **
 ** \param f32Duty duty cycle (between 0.0 and 1.0)
 **
 ******************************************************************************/
void ApolloCTimer_PwmSetDutyByPin(int pin, float32_t f32Duty)
{
    stc_apolloctimer_timer_ab_t* pstcHandle;
    boolean_t bPolarity;
    pstcHandle = GetTimerFromPin(pin);
      
    if (pstcHandle == NULL) return;
    
    if (pstcHandle->enTimerAB == CTimerA)
    {
        bPolarity = pstcHandle->HANDLE->CTRL_b.TMRAPOL;
    } else{
        bPolarity = pstcHandle->HANDLE->CTRL_b.TMRAPOL;
    }
    
    if (f32Duty <= 0.0001f)
    {
        ApolloGpio_GpioSet(pin,((~bPolarity)&0x1));
        ApolloGpio_GpioSelectFunction(pin,3);
    } else  if (f32Duty >= 0.9999f)
    {
        ApolloGpio_GpioSet(pin,bPolarity);
        ApolloGpio_GpioSelectFunction(pin,3);
    } else
    {
        ApolloGpio_GpioSelectFunction(pin,2);
        ApolloCTimer_PwmSetDuty(pstcHandle,f32Duty);
    }
}
#endif

/**
 ******************************************************************************
 ** \brief  Set the duty cycle of a PWM
 **
 ** \param pstcHandle  Can be CTIMERA0, CTIMERB0, CTIMERA1, CTIMERB1
 **                           CTIMERA2, CTIMERB2, CTIMERA3, CTIMERB3
 **
 ** \param f32Duty duty cycle (between 0.0 and 1.0)
 **
 ******************************************************************************/
void ApolloCTimer_PwmSetDuty(stc_apolloctimer_timer_ab_t* pstcHandle, float32_t f32Duty)
{
    if (pstcHandle == NULL) return;
    if (pstcHandle->enTimerAB == CTimerA)
    {
        if (pstcHandle->HANDLE->CTRL_b.TMRAEN == 0) return;
    } else{
        if (pstcHandle->HANDLE->CTRL_b.TMRBEN == 0) return;
    }
    #if defined(APOLLO_H) || defined(APOLLO1_H)
        if (pstcHandle == CTIMERA0)
        {
            CTIMER->INTCLR_b.CTMRA0INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA0INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB0)
        {
            CTIMER->INTCLR_b.CTMRB0INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB0INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERA1)
        {
            CTIMER->INTCLR_b.CTMRA1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB1)
        {
            CTIMER->INTCLR_b.CTMRB1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERA2)
        {
            CTIMER->INTCLR_b.CTMRA2INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA2INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB2)
        {
            CTIMER->INTCLR_b.CTMRB2INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB2INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERA3)
        {
            CTIMER->INTCLR_b.CTMRA3INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA3INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB3)
        {
            CTIMER->INTCLR_b.CTMRB3INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB3INT == 0) __NOP();
        }
        if (pstcHandle->enTimerAB == CTimerA)
        {
            pstcHandle->HANDLE->CMPRA_b.CMPR0A = (uint32_t)(f32Duty * (pstcHandle->HANDLE->CMPRA_b.CMPR1A - 2) + 1);
        } else{
            pstcHandle->HANDLE->CMPRB_b.CMPR0B = (uint32_t)(f32Duty * (pstcHandle->HANDLE->CMPRB_b.CMPR1B - 2) + 1);
        }
    #elif defined(APOLLO2_H)
        if (pstcHandle == CTIMERA0)
        {
            CTIMER->INTCLR_b.CTMRA0C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA0C1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB0)
        {
            CTIMER->INTCLR_b.CTMRA0C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA0C1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERA1)
        {
            CTIMER->INTCLR_b.CTMRA1C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA1C1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB1)
        {
            CTIMER->INTCLR_b.CTMRB1C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB1C1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERA2)
        {
            CTIMER->INTCLR_b.CTMRA2C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA2C1INT == 0) __NOP();

        }
        if (pstcHandle == CTIMERB2)
        {
            CTIMER->INTCLR_b.CTMRB2C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB2C1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERA3)
        {
            CTIMER->INTCLR_b.CTMRA3C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRA3C1INT == 0) __NOP();
        }
        if (pstcHandle == CTIMERB3)
        {
            CTIMER->INTCLR_b.CTMRB3C1INT = 1;
            while (CTIMER->INTSTAT_b.CTMRB3C1INT == 0) __NOP();
        }
        
        if (pstcHandle->enTimerAB == CTimerA)
        {
            pstcHandle->HANDLE->CMPRA_b.CMPR0A = (uint32_t)(f32Duty * (pstcHandle->HANDLE->CMPRA_b.CMPR1A - 2) + 1);
        } else{
            pstcHandle->HANDLE->CMPRB_b.CMPR0B = (uint32_t)(f32Duty * (pstcHandle->HANDLE->CMPRB_b.CMPR1B - 2) + 1);
        }
    #endif
}
#else
#warning Low-Level-Driver for Apollo 1/2 CTIMER is disabled and could be removed from the project
#endif //(CTIMER0_ENABLED == 1) || (CTIMER1_ENABLED == 1) || (CTIMER2_ENABLED == 1) || (CTIMER3_ENABLED == 1) || (APOLLOCTIMER_ENABLED == 1)
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


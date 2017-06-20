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
/** \file ApolloCTimer.h
 **
 ** A detailed description is available at 
 ** @link ApolloCTimerGroup Apollo CTIMER Implementation description @endlink
 **
 ** History:
 **   - 2017-05-16  V1.0  MSc  First Version

 *****************************************************************************/
#ifndef __APOLLOCTIMER_H__
#define __APOLLOCTIMER_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup ApolloCTimerGroup Apollo CTIMER Implementation
 **
 ** Provided functions of ApolloCTimer:
 ** 
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page apolloctimer_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "apolloctimer.h"   
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

#define CTIMER0  ((stc_apolloctimer_timer_t*)(&CTIMER->TMR0))
#define CTIMER1  ((stc_apolloctimer_timer_t*)(&CTIMER->TMR1))
#define CTIMER2  ((stc_apolloctimer_timer_t*)(&CTIMER->TMR2))
#define CTIMER3  ((stc_apolloctimer_timer_t*)(&CTIMER->TMR3))
#define CTIMERA0 ((stc_apolloctimer_timer_ab_t*)&stcCTimerA0)
#define CTIMERB0 ((stc_apolloctimer_timer_ab_t*)&stcCTimerB0)
#define CTIMERA1 ((stc_apolloctimer_timer_ab_t*)&stcCTimerA1)
#define CTIMERB1 ((stc_apolloctimer_timer_ab_t*)&stcCTimerB1)
#define CTIMERA2 ((stc_apolloctimer_timer_ab_t*)&stcCTimerA2)
#define CTIMERB2 ((stc_apolloctimer_timer_ab_t*)&stcCTimerB2)
#define CTIMERA3 ((stc_apolloctimer_timer_ab_t*)&stcCTimerA3)
#define CTIMERB3 ((stc_apolloctimer_timer_ab_t*)&stcCTimerB3)
    
/*****************************************************************************/
/* Global type definitions ('typedef')                                        */
/*****************************************************************************/

     #if defined(APOLLO2_H)
typedef struct stc_apolloctimer_timer
{
  union {
    __IO uint32_t  TMR;                            /*!< Counter/Timer Register                                                */
    
    struct {
      __IO uint32_t  CTTMRA    : 16;               /*!< Counter/Timer A0.                                                     */
      __IO uint32_t  CTTMRB    : 16;               /*!< Counter/Timer B0.                                                     */
    } TMR_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CMPRA;                          /*!< Counter/Timer A0 Compare Registers                                    */
    
    struct {
      __IO uint32_t  CMPR0A    : 16;               /*!< Counter/Timer A0 Compare Register 0. Holds the lower limit for
                                                         timer half A.                                                         */
      __IO uint32_t  CMPR1A    : 16;               /*!< Counter/Timer A0 Compare Register 1. Holds the upper limit for
                                                         timer half A.                                                         */
    } CMPRA_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CMPRB;                          /*!< Counter/Timer B0 Compare Registers                                    */
    
    struct {
      __IO uint32_t  CMPR0B    : 16;               /*!< Counter/Timer B0 Compare Register 0. Holds the lower limit for
                                                         timer half B.                                                         */
      __IO uint32_t  CMPR1B    : 16;               /*!< Counter/Timer B0 Compare Register 1. Holds the upper limit for
                                                         timer half B.                                                         */
    } CMPRB_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CTRL;                           /*!< Counter/Timer Control                                                 */
    
    struct {
      __IO uint32_t  TMRAEN    :  1;               /*!< Counter/Timer A0 Enable bit.                                          */
      __IO uint32_t  TMRACLK   :  5;               /*!< Counter/Timer A0 Clock Select.                                        */
      __IO uint32_t  TMRAFN    :  3;               /*!< Counter/Timer A0 Function Select.                                     */
      __IO uint32_t  TMRAIE0   :  1;               /*!< Counter/Timer A0 Interrupt Enable bit based on COMPR0.                */
      __IO uint32_t  TMRAIE1   :  1;               /*!< Counter/Timer A0 Interrupt Enable bit based on COMPR1.                */
      __IO uint32_t  TMRACLR   :  1;               /*!< Counter/Timer A0 Clear bit.                                           */
      __IO uint32_t  TMRAPOL   :  1;               /*!< Counter/Timer A0 output polarity.                                     */
      __IO uint32_t  TMRAPE    :  1;               /*!< Counter/Timer A0 Output Enable bit.                                   */
           uint32_t             :  2;
      __IO uint32_t  TMRBEN    :  1;               /*!< Counter/Timer B0 Enable bit.                                          */
      __IO uint32_t  TMRBCLK   :  5;               /*!< Counter/Timer B0 Clock Select.                                        */
      __IO uint32_t  TMRBFN    :  3;               /*!< Counter/Timer B0 Function Select.                                     */
      __IO uint32_t  TMRBIE0   :  1;               /*!< Counter/Timer B0 Interrupt Enable bit for COMPR0.                     */
      __IO uint32_t  TMRBIE1   :  1;               /*!< Counter/Timer B0 Interrupt Enable bit for COMPR1.                     */
      __IO uint32_t  TMRBCLR   :  1;               /*!< Counter/Timer B0 Clear bit.                                           */
      __IO uint32_t  TMRBPOL   :  1;               /*!< Counter/Timer B0 output polarity.                                     */
      __IO uint32_t  TMRBPE    :  1;               /*!< Counter/Timer B0 Output Enable bit.                                   */
           uint32_t             :  1;
      __IO uint32_t  CTLINK    :  1;               /*!< Counter/Timer A0/B0 Link bit.                                         */
    } CTRL_b;                                      /*!< BitSize                                                               */
  };
} stc_apolloctimer_timer_t;
#endif

typedef enum en_apolloctimer_ab
{
    CTimerA,
    CTimerB
} en_apolloctimer_ab_t;

typedef enum en_apolloctimer_function
{
    CTimerSingleCount = 0,
    CTimerRepeatedCount = 1,
    CTimerSinglePulse = 2,
    CTimerRepeatedPulse = 3,
    CTimerContinuous = 4,
} en_apolloctimer_function_t;

typedef struct stc_ctimer_timer_ab
{
    stc_apolloctimer_timer_t* HANDLE;
    en_apolloctimer_ab_t enTimerAB;
} stc_apolloctimer_timer_ab_t;



/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern const stc_apolloctimer_timer_ab_t stcCTimerA0;
extern const stc_apolloctimer_timer_ab_t stcCTimerB0;
extern const stc_apolloctimer_timer_ab_t stcCTimerA1;
extern const stc_apolloctimer_timer_ab_t stcCTimerB1;
extern const stc_apolloctimer_timer_ab_t stcCTimerA2;
extern const stc_apolloctimer_timer_ab_t stcCTimerB2;
extern const stc_apolloctimer_timer_ab_t stcCTimerA3;
extern const stc_apolloctimer_timer_ab_t stcCTimerB3;

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

#if (APOLLOGPIO_ENABLED == 1)
void ApolloCTimer_PwmInitByPin(int pin);
void ApolloCTimer_PwmSetDutyByPin(int pin, float32_t f32Duty);
#endif
void ApolloCTimer_PwmInit(stc_apolloctimer_timer_ab_t* pstcHandle);
void ApolloCTimer_PwmSetDuty(stc_apolloctimer_timer_ab_t* pstcHandle, float32_t f32Duty);


#ifdef __cplusplus
}
#endif

//@} // ApolloCTimerGroup

#endif /*__APOLLOCTIMER_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


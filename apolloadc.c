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
/** \file ApolloAdc.c
 **
 ** A detailed description is available at 
 ** @link ApolloAdcGroup Apollo ADC Module description @endlink
 **
 ** History:
 **   - 2017-09-20  V1.0  Manuel Schreiner   First Version
 **   - 2018-06-07  V1.1  Manuel Schreiner   Added missing ApolloAdc_SimpleRead prototype  
 **   - 2018-07-06  V1.2  Manuel Schreiner   Updated documentation, 
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **   - 2018-08-09  V1.3  Manuel Schreiner   Added support for Apollo3
 **
 *****************************************************************************/
#define __APOLLOADC_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "base_types.h"
#include "apolloadc.h"
#include "mcu.h"
#if APOLLOGPIO_ENABLED == 1
#include "apollogpio.h"
#endif

#if (APOLLOADC_ENABLED == 1)
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/


/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/



/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

#if defined(__CC_ARM)
  //#pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif

#if defined(APOLLO_H) ||  defined(APOLLO1_H)
typedef struct stc_adc_fifo_data
{
   union {
    __IO uint32_t  FIFO;                            /*!< FIFO Data and Valid Count Register                                    */
    
    struct {
      __IO uint32_t  DATA       : 16;               /*!< Oldest data in the FIFO.                                              */
      __IO uint32_t  COUNT      :  4;               /*!< Number of valid entries in the ADC FIFO.                              */
      __IO uint32_t  RSVD_20    :  4;               /*!< RESERVED.                                                             */
      __IO uint32_t  SLOTNUM    :  3;               /*!< Slot number associated with this FIFO data.                           */
      __IO uint32_t  RSVD_27    :  5;               /*!< RESERVED.                                                             */
    } FIFO_b;                                       /*!< BitSize                                                               */
  };
} stc_adc_fifo_data_t;
#elif defined(APOLLO2_H) || defined(APOLLO3_H)
typedef struct stc_adc_fifo_data
{
    union {
    __IO uint32_t  FIFO;                            /*!< FIFO Data and Valid Count Register                                    */
    
    struct {
      __IO uint32_t  DATA       : 20;               /*!< Oldest data in the FIFO.                                              */
      __IO uint32_t  COUNT      :  8;               /*!< Number of valid entries in the ADC FIFO.                              */
      __IO uint32_t  SLOTNUM    :  3;               /*!< Slot number associated with this FIFO data.                           */
      __IO uint32_t  RSVD       :  1;               /*!< RESERVED.                                                             */
    } FIFO_b;                                       /*!< BitSize                                                               */
  };
} stc_adc_fifo_data_t;
#endif


#if defined(__CC_ARM)
  //#pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif



/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static uint32_t u32AdcData;
static volatile boolean_t bTriggerAdc = FALSE;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static void checkadc(void);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
/**
 ******************************************************************************
 ** \brief  Check for ADC data
 **
 ******************************************************************************/

static void checkadc(void)
{
    uint8_t u8Status;
    stc_adc_fifo_data_t stcFifoValue;
    if (ADC->INTSTAT != 0)
    {
      u8Status = ADC->INTSTAT;       //read IRQ status
      ADC->INTCLR = u8Status;        //clear IRQ flags
      do
      {
        stcFifoValue.FIFO = ADC->FIFO; //read FIFO
        if (stcFifoValue.FIFO_b.COUNT > 0)
        {
            ADC->FIFO = 0;                 //pop FIFO
      
            if (stcFifoValue.FIFO_b.SLOTNUM == 1) //if it is the right slot
            {
                u32AdcData = stcFifoValue.FIFO_b.DATA; //store data
                bTriggerAdc = TRUE;
            }
        }
      } while(stcFifoValue.FIFO_b.COUNT > 0); 
    }
}

/**
 ******************************************************************************
 ** \brief  Read an ADC value
 **
 ** \param pin  Can be every pin name defined in enum PinName, 
 **             for example GPIO1, GPIO2, ... GPIO49
 **
 ** \param bLowPower TRUE, to execute in low-power mode
 **
 ** \return returns a float value from 0..1.0f
 ** 
 ******************************************************************************/
float32_t ApolloAdc_SimpleRead(uint32_t pin, boolean_t bLowPower)
{
    uint32_t u32Cfg = 0;
    boolean_t bBandgapStatus = FALSE;
    bBandgapStatus = bBandgapStatus;
    #if defined(APOLLO2_H) 
        PWRCTRL->DEVICEEN |= (1 << PWRCTRL_DEVICEEN_ADC_Pos);
    #endif
    #if defined(APOLLO3_H)
        PWRCTRL->DEVPWREN_b.PWRADC = 1;
        while(PWRCTRL->ADCSTATUS_b.ADCPWD == 0) __NOP();
    #endif
	
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        bBandgapStatus = MCUCTRL->BANDGAPEN_b.BGPEN;
        MCUCTRL->BANDGAPEN_b.BGPEN = 1;       //enable bandgap
    #endif
    
    ADC->CFG = 0;
    
    #if defined(APOLLO2_H) || defined(APOLLO3_H) 
        u32Cfg |= (1 << ADC_CFG_CKMODE_Pos); //Low Latency Clock Mode
        ADC->CFG_b.CKMODE = 1;               
    #endif

    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (4 << ADC_CFG_CLKSEL_Pos); //1.5MHz clock              
    #elif defined(APOLLO2_H) || defined(APOLLO3_H)
        u32Cfg |= (1 << ADC_CFG_CLKSEL_Pos); //HCLK 
    #endif
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (8 << ADC_CFG_TRIGSEL_Pos); //Software Trigger                 
    #elif defined(APOLLO2_H) || defined(APOLLO3_H)
        u32Cfg |= (7 << ADC_CFG_TRIGSEL_Pos); //Software Trigger   
    #endif

    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (1 << ADC_CFG_REFSEL_Pos); //select VDD as voltage reference                 
    #elif defined(APOLLO2_H) || defined(APOLLO3_H)
        u32Cfg |= (0 << ADC_CFG_REFSEL_Pos); //select internal 2V as voltage reference
    #endif
    #if defined(APOLLO_H)
        u32Cfg |= (2 << ADC_CFG_OPMODE_Pos); //SAMPLE_RATE_125K_800KSPS
    #endif

    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (2 << ADC_CFG_LPMODE_Pos); //Low Power Mode 2
    #elif defined(APOLLO2_H)  || defined(APOLLO3_H)
        u32Cfg |= (1 << ADC_CFG_LPMODE_Pos); //Low Power Mode 1
    #endif
    u32Cfg |= (0 << ADC_CFG_RPTEN_Pos); //Repeated Mode
    
    ADC->SL1CFG = 0;

    ApolloGpio_GpioInputEnable(pin,TRUE);
    ApolloGpio_GpioSelectFunction(pin,0);
    
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        switch(pin)
        {
            case 12:
                    ADC->SL1CFG_b.CHSEL1 = 0;             //use ch 0 for slot 1
                    break;
            case 13:
                    ADC->SL1CFG_b.CHSEL1 = 1;             //use ch 1 for slot 1
                    break;
            case 14:
                    ADC->SL1CFG_b.CHSEL1 = 2;             //use ch 2 for slot 1
                    break;
            case 15:
                    ADC->SL1CFG_b.CHSEL1 = 3;             //use ch 3 for slot 1
                    break;
            case 29:
                    ADC->SL1CFG_b.CHSEL1 = 4;             //use ch 4 for slot 1
                    break;
            case 30:
                    ADC->SL1CFG_b.CHSEL1 = 5;             //use ch 5 for slot 1
                    break;
            case 31:
                    ADC->SL1CFG_b.CHSEL1 = 6;             //use ch 6 for slot 1
                    break;
            case 32:
                    ADC->SL1CFG_b.CHSEL1 = 7;             //use ch 7 for slot 1
                    break;
            default:
                    return -1;
        }
    #elif defined(APOLLO2_H)
        switch(pin)
        {
            case 16:
                    ADC->SL1CFG_b.CHSEL1 = 0;             //use ch 0 for slot 1
                    break;
            case 19:
                    ADC->SL1CFG_b.CHSEL1 = 1;             //use ch 1 for slot 1
                    break;
            case 11:
                    ADC->SL1CFG_b.CHSEL1 = 2;             //use ch 2 for slot 1
                    break;
            case 31:
                    ADC->SL1CFG_b.CHSEL1 = 3;             //use ch 3 for slot 1
                    break;
            case 32:
                    ADC->SL1CFG_b.CHSEL1 = 4;             //use ch 4 for slot 1
                    break;
            case 33:
                    ADC->SL1CFG_b.CHSEL1 = 5;             //use ch 5 for slot 1
                    break;
            case 34:
                    ADC->SL1CFG_b.CHSEL1 = 6;             //use ch 6 for slot 1
                    break;
            case 35:
                    ADC->SL1CFG_b.CHSEL1 = 7;             //use ch 7 for slot 1
                    break;
            default:
                                return -1;
        }
    #elif defined(APOLLO3_H)
        switch(pin)
        {
            case 16:
                    ADC->SL1CFG_b.CHSEL1 = 0;             //use ch 0 for slot 1
                    break;
            case 29:
                    ADC->SL1CFG_b.CHSEL1 = 1;             //use ch 1 for slot 1
                    break;
            case 11:
                    ADC->SL1CFG_b.CHSEL1 = 2;             //use ch 2 for slot 1
                    break;
            case 31:
                    ADC->SL1CFG_b.CHSEL1 = 3;             //use ch 3 for slot 1
                    break;
            case 32:
                    ADC->SL1CFG_b.CHSEL1 = 4;             //use ch 4 for slot 1
                    break;
            case 33:
                    ADC->SL1CFG_b.CHSEL1 = 5;             //use ch 5 for slot 1
                    break;
            case 34:
                    ADC->SL1CFG_b.CHSEL1 = 6;             //use ch 6 for slot 1
                    break;
            case 35:
                    ADC->SL1CFG_b.CHSEL1 = 7;             //use ch 7 for slot 1
                    break;
            case 13:
                    ADC->SL1CFG_b.CHSEL1 = 8;             //use ch 8 for slot 1
                    break;
            case 12:
                    ADC->SL1CFG_b.CHSEL1 = 9;             //use ch 9 for slot 1
                    break;
            default:
                                return -1;
        }
    #else
        #error No supported MCU found
    #endif
        
    GPIO->PADKEY = 0x0000000;
    ADC->SL1CFG_b.ADSEL1 = 0;
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        ADC->SL1CFG_b.THSEL1 = 0;             //use 1 ADC clock cycles
    #endif
    #if defined(APOLLO2_H) || defined(APOLLO3_H)
        ADC->SL1CFG_b.PRMODE1 = 0;             //use 14-bit
	#endif
    ADC->SL1CFG_b.WCEN1 = 0;              //disable window compare
    ADC->SL1CFG_b.SLEN1 = 1;              //enable channel
    
    ADC->INTEN_b.WCINC = 1;               //enable window comparator voltage incursion interrupt
    ADC->INTEN_b.WCEXC = 1;               //enable window comparator voltage excursion interrupt
    ADC->INTEN_b.FIFOOVR1 = 1;            //enable FIFO 100% full interrupt
    ADC->INTEN_b.FIFOOVR2 = 1;            //enable FIFO 75% full interrupt
    ADC->INTEN_b.SCNCMP = 1;              //enable ADC scan complete interrupt
    ADC->INTEN_b.CNVCMP = 1;              //enable ADC conversion complete interrupt
    
    //ADC->WLIM_b.LLIM = 0x100;
    //ADC->WLIM_b.ULIM = 0x300;
    
    bTriggerAdc = FALSE;
    u32Cfg |= (1 << ADC_CFG_ADCEN_Pos); //Repeated Mode
    ADC->CFG = u32Cfg;                 //enable the ADC
    
    if (bLowPower == TRUE)
    {
        NVIC_ClearPendingIRQ(ADC_IRQn);       //clear pending flag for ADC
        NVIC_EnableIRQ(ADC_IRQn);             //enable IRQ
        NVIC_SetPriority(ADC_IRQn,1);         //set priority of ADC IRQ, smaller value means higher priority
    }
    
    ADC->SWT = 0x37;                      //trigger ADC
    
    if (bLowPower) 
    {
        SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos); //set goto deepsleep (only possible if TPIU or SWO is not enabled)
        __WFI();                                  //wait for interrupt
    }
    
    while(bTriggerAdc == FALSE) 
    {
        if (!bLowPower) checkadc();
    }
    ADC->CFG_b.ADCEN = 0;                 //disable the ADC
    
    #if defined(APOLLO2_H)
        PWRCTRL->DEVICEEN &= ~(1 << PWRCTRL_DEVICEEN_ADC_Pos);
    #endif
    #if defined(APOLLO3_H)
        PWRCTRL->DEVPWREN_b.PWRADC = 0;
    #endif
        
    #if defined(APOLLO_H)
        MCUCTRL->BANDGAPEN_b.BGPEN = bBandgapStatus;  
    #endif
    
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        return u32AdcData / 65535.0f;
    #elif defined(APOLLO2_H) || defined(APOLLO3_H)
        return u32AdcData / 1048512.0f;
    #endif
}

/**
 ******************************************************************************
 ** \brief  Read an ADC value from VCC
 **
 ** \param bLowPower TRUE, to execute in low-power mode
 **
 ** \return returns a float value from 0..1.0f
 ** 
 ******************************************************************************/
float32_t ApolloAdc_CheckBattery(boolean_t bLowPower)
{
    uint32_t u32Cfg = 0;
    float32_t f32Tmp;
    
    boolean_t bBandgapStatus = FALSE;
    #if defined(APOLLO2_H)
        PWRCTRL->DEVICEEN |= (1 << PWRCTRL_DEVICEEN_ADC_Pos);
    #endif
	
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        bBandgapStatus = MCUCTRL->BANDGAPEN_b.BGPEN;
        MCUCTRL->BANDGAPEN_b.BGPEN = 1;       //enable bandgap
    #endif
    
    ADC->CFG = 0;
    
    //ADC->CFG_b.BATTLOAD = 1;
#if defined(APOLLO3_H)
    MCUCTRL->ADCBATTLOAD_b.BATTLOAD = 1;
#else
    u32Cfg |= (1 << ADC_CFG_BATTLOAD_Pos);
#endif
    
    #if defined(APOLLO2_H) 
        u32Cfg |= (1 << ADC_CFG_CKMODE_Pos); //Low Latency Clock Mode
        ADC->CFG_b.CKMODE = 1;               
    #endif

    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (4 << ADC_CFG_CLKSEL_Pos); //1.5MHz clock              
    #elif defined(APOLLO2_H)
        u32Cfg |= (1 << ADC_CFG_CLKSEL_Pos); //HCLK 
    #endif
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (8 << ADC_CFG_TRIGSEL_Pos); //Software Trigger                 
    #elif defined(APOLLO2_H)
        u32Cfg |= (7 << ADC_CFG_TRIGSEL_Pos); //Software Trigger   
    #endif

    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (0 << ADC_CFG_REFSEL_Pos); //select VDD as voltage reference                 
    #elif defined(APOLLO2_H)
        u32Cfg |= (0 << ADC_CFG_REFSEL_Pos); //select internal 2V as voltage reference
    #endif
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (2 << ADC_CFG_OPMODE_Pos); //SAMPLE_RATE_125K_800KSPS
    #endif

    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        u32Cfg |= (2 << ADC_CFG_LPMODE_Pos); //Low Power Mode 2
    #elif defined(APOLLO2_H) 
        u32Cfg |= (1 << ADC_CFG_LPMODE_Pos); //Low Power Mode 1
    #endif
    u32Cfg |= (0 << ADC_CFG_RPTEN_Pos); //Repeated Mode
    
    ADC->SL1CFG = 0;
    
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        ADC->SL1CFG_b.CHSEL1 = 12;             //use ch 0 for slot 1
    #elif defined(APOLLO2_H) || defined(APOLLO3_H)
        ADC->SL1CFG_b.CHSEL1 = 12;             //use ch 0 for slot 1
    #else
        #error No supported MCU found
    #endif
        
    GPIO->PADKEY = 0x0000000;
    ADC->SL1CFG_b.ADSEL1 = 0;
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        ADC->SL1CFG_b.THSEL1 = 0;             //use 1 ADC clock cycles
    #endif
    #if defined(APOLLO2_H)
        ADC->SL1CFG_b.PRMODE1 = 0;             //use 14-bit
	#endif
    ADC->SL1CFG_b.WCEN1 = 0;              //disable window compare
    ADC->SL1CFG_b.SLEN1 = 1;              //enable channel
    
    ADC->INTEN_b.WCINC = 1;               //enable window comparator voltage incursion interrupt
    ADC->INTEN_b.WCEXC = 1;               //enable window comparator voltage excursion interrupt
    ADC->INTEN_b.FIFOOVR1 = 1;            //enable FIFO 100% full interrupt
    ADC->INTEN_b.FIFOOVR2 = 1;            //enable FIFO 75% full interrupt
    ADC->INTEN_b.SCNCMP = 1;              //enable ADC scan complete interrupt
    ADC->INTEN_b.CNVCMP = 1;              //enable ADC conversion complete interrupt
    
    //ADC->WLIM_b.LLIM = 0x100;
    //ADC->WLIM_b.ULIM = 0x300;
    
    bTriggerAdc = FALSE;
    u32Cfg |= (1 << ADC_CFG_ADCEN_Pos); //Repeated Mode
    ADC->CFG = u32Cfg;                 //enable the ADC
    
    if (bLowPower == TRUE)
    {
        NVIC_ClearPendingIRQ(ADC_IRQn);       //clear pending flag for ADC
        NVIC_EnableIRQ(ADC_IRQn);             //enable IRQ
        NVIC_SetPriority(ADC_IRQn,1);         //set priority of ADC IRQ, smaller value means higher priority
    }
    
    ADC->SWT = 0x37;                      //trigger ADC
    
    if (bLowPower) 
    {
        SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos); //set goto deepsleep (only possible if TPIU or SWO is not enabled)
        __WFI();                                  //wait for interrupt
    }
    
    while(bTriggerAdc == FALSE) 
    {
        if (!bLowPower) checkadc();
    }
#if defined(APOLLO3_H)
    MCUCTRL->ADCBATTLOAD_b.BATTLOAD = 0;
#else
    ADC->CFG_b.BATTLOAD = 0;
#endif
    
    ADC->CFG_b.ADCEN = 0;                 //disable the ADC
    
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        ADC->SL1CFG_b.CHSEL1 = 0;             //use ch 0 for slot 1
    #elif defined(APOLLO2_H) || defined(APOLLO3_H)
        ADC->SL1CFG_b.CHSEL1 = 0;             //use ch 0 for slot 1
    #else
        #error No supported MCU found
    #endif
        
    #if defined(APOLLO2_H)
        PWRCTRL->DEVICEEN &= ~(1 << PWRCTRL_DEVICEEN_ADC_Pos);
    #endif
        
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        MCUCTRL->BANDGAPEN_b.BGPEN = bBandgapStatus;
    #endif
    
    //ADC->CFG_b.BATTLOAD = 0;
        
    #if defined(APOLLO_H) ||  defined(APOLLO1_H)
        f32Tmp = (u32AdcData / 65535.0f);
    #elif defined(APOLLO2_H)
        f32Tmp = (u32AdcData / 1048512.0f);
    #endif
    f32Tmp = f32Tmp * 1.5f * 3;
    return f32Tmp;
}
#else
#warning Low-Level-Driver for Apollo 1/2 ADC is disabled and could be removed from the project
#endif

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


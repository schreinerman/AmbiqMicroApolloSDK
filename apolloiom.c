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
/** \file ApolloIom.c
**
** A detailed description is available at 
** @link ApolloIomGroup Apollo IOM Driver description @endlink
**
** History:
**   - 2017-01-31  V1.0  MSc  First Version
**   - 2017-04-13  V1.1  MSc  Added one-byte transfer
**   - 2017-10-17  V1.2  MSc  Added register based transfers
**   - 2018-04-04  V1.3  MSc  Added interrupt handling and SW SPI
**
*****************************************************************************/
#define __APOLLOIOM_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "apolloiom.h"
#include "string.h"
#if APOLLOSWSPI_ENABLED == 1
    #if APOLLOGPIO_ENABLED == 0
        #error Please enable Apollo GPIO module
    #endif
    #include "apollogpio.h"
#endif 

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#if (APOLLOSWSPI_ENABLED == 1) || (APOLLOIOM_ENABLED == 1) || (IOMSTR0_ENABLED == 1) || (IOMSTR1_ENABLED == 1) || (IOMSTR2_ENABLED == 1) || (IOMSTR3_ENABLED == 1) || (IOMSTR4_ENABLED == 1) || (IOMSTR5_ENABLED == 1)

#define INSTANCE_COUNT (uint32_t)(sizeof(m_astcInstanceDataLut) / sizeof(m_astcInstanceDataLut[0]))
#define MAX_FIFO_SIZE  64

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/



/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/


/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
    
/// Look-up table for all enabled IOM instances and their internal data
static stc_apolloiom_instance_data_t m_astcInstanceDataLut[] =
{
#if defined(IOMSTR0) && ((IOMSTR0_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
    { (IOMSTR0),  // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if defined(IOMSTR1) && ((IOMSTR1_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
    { (IOMSTR1),  // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if defined(IOMSTR2) && ((IOMSTR2_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
    { IOMSTR2,  // pstcInstance
    0          // stcInternData (not initialized yet)
    },
#endif
#if defined(IOMSTR3) && ((IOMSTR3_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
    { (IOMSTR3),  // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if defined(IOMSTR4) && ((IOMSTR4_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
    { (IOMSTR4),  // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if defined(IOMSTR5) && ((IOMSTR5_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
    { (IOMSTR5),  // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if (APOLLOSWSPI_ENABLED == 1)
    { (IOMSTR_SWSPI),  // pstcInstance
    (0)                // stcInternData (not initialized yet)
    },
#endif
};

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static stc_apolloiom_intern_data_t* GetInternDataPtr(IOMSTR0_Type* pstcIf);
static boolean_t onebit(uint32_t u32Value);
static uint32_t compute_freq(uint32_t u32Fsel, uint32_t u32Div3, uint32_t u32DivEn, uint32_t u32TotPer);
static en_result_t ApolloIOM_GetClockCfg(uint32_t u32Frequency, uint32_t* pu32RealFrequency, uint32_t* pu32ClkCfg, boolean_t bPhase);
static uint32_t DataToFifo(IOMSTR0_Type* pstcHandle, uint8_t* pu8Data, uint32_t u32NumBytes);
static uint32_t FifoToData(IOMSTR0_Type* pstcHandle, uint8_t* pu8Data, uint32_t u32NumBytes);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
******************************************************************************
** \brief Return the internal data for a certain instance.
**
** \param pstcIf Pointer to instance
**
** \return Pointer to internal data or NULL if instance is not enabled (or not known)
**
******************************************************************************/
static stc_apolloiom_intern_data_t* GetInternDataPtr(IOMSTR0_Type* pstcIf) 
{
    volatile uint32_t u32Instance;
    
    for (u32Instance = 0; u32Instance < INSTANCE_COUNT; u32Instance++)
    {
        if ((uint32_t)pstcIf == (uint32_t)(m_astcInstanceDataLut[u32Instance].pstcInstance))
        {
            return &m_astcInstanceDataLut[u32Instance].stcInternData;
        }
    }
    
    return NULL;
}

/**
******************************************************************************
** \brief Move buffer into FIFO
**
** \param pstcIf  Pointer to instance
**
** \param pu8Data Data buffer
**
** \param u32NumBytes Number of bytes to transfer
**
** \return number of bytes transferred
**
******************************************************************************/
static uint32_t DataToFifo(IOMSTR0_Type* pstcHandle, uint8_t* pu8Data, uint32_t u32NumBytes)
{
    uint32_t i;
    uint32_t u32Tmp;
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
    
    if ((((uint32_t)pu8Data) % 4) == 0)
    {
        //
        // Loop over the words in the array until we have the correct number of
        // bytes.
        //
        for(i = 0;(4*i) < u32NumBytes;i++)
        {
            //
            // Write the word to the FIFO.
            //
            pstcHandle->FIFO = ((uint32_t*)pu8Data)[i];
        }
    }
    else
    {
        for(i = 0;(4*i) < u32NumBytes;i++)
        {
            //
            // Write the word to the FIFO.
            //
            u32Tmp = 0;
            /*for(j = 0; j < 4;j++)
            {
                ((uint8_t*)&u32Tmp)[j] = pu8Data[i*4 + j];
            }*/
            memcpy(&u32Tmp,&pu8Data[i*4],((u32NumBytes - i*4) <= 4 ? (u32NumBytes - i*4) : 4)); 
            pstcHandle->FIFO = u32Tmp;
        }
    }
    return u32NumBytes;
}

/**
******************************************************************************
** \brief Move FIFO into buffer
**
** \param pstcIf  Pointer to instance
**
** \param pu8Data Data buffer
**
** \param u32NumBytes Number of bytes to transfer
**
** \return number of bytes transferred
**
******************************************************************************/
static uint32_t FifoToData(IOMSTR0_Type* pstcHandle, uint8_t* pu8Data, uint32_t u32NumBytes)
{
    uint32_t i;
    uint32_t u32Tmp;
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    //if (u32NumBytes == 0) return Error;
    
    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
    
    if ((((uint32_t)pu8Data) % 4) == 0)
    {
        //
        // Loop over the words in the array until we have the correct number of
        // bytes.
        //
        for(i = 0;(4*i) < u32NumBytes;i++)
        {
            //
            // Write the word to the FIFO.
            //
            if ((u32NumBytes - (4*i)) < 4)
            {
                u32Tmp = pstcHandle->FIFO;
                memcpy(&pu8Data[i*4],&u32Tmp,((u32NumBytes - i*4) <= 4 ? (u32NumBytes - i*4) : 4));
            }
            else
            {
                ((uint32_t*)pu8Data)[i] = pstcHandle->FIFO;
            }
        }
    }
    else
    {
        for(i = 0;(4*i) < u32NumBytes;i++)
        {
            //
            // Write the word to the FIFO.
            //
            u32Tmp = pstcHandle->FIFO;
            memcpy(&pu8Data[i*4],&u32Tmp,((u32NumBytes - i*4) <= 4 ? (u32NumBytes - i*4) : 4)); 
            
        }
    }
    return u32NumBytes;
}

/**
******************************************************************************
** \brief A power of 2?
**
** \return Return true if ui32Value has exactly 1 bit set, otherwise false.
**
******************************************************************************/
static boolean_t onebit(uint32_t u32Value)
{
    //
    // A power of 2?
    // Return true if ui32Value has exactly 1 bit set, otherwise false.
    //
    return u32Value  &&  !(u32Value & (u32Value - 1));
}

/**
******************************************************************************
** \brief Compute the real frequency
**
** \param u32Fsel   FSEL bit value of CLKCFG
**
** \param u32Div3   DIV3 bit value of CLKCFG
**
** \param u32DivEn  DIVEN bit value of CLKCFG
**
** \param u32TotPer TOTPER bit value of CLKCFG
**
** \return Return the real frequency
**
******************************************************************************/
static uint32_t compute_freq(uint32_t u32Fsel, uint32_t u32Div3,
                             uint32_t u32DivEn, uint32_t u32TotPer)
{
    uint32_t ui32Denomfinal, ui32ClkFreq;
    SystemCoreClockUpdate();
    ui32Denomfinal = ((1 << (u32Fsel - 1)) * (1 + u32Div3 * 2) * (1 + u32DivEn * (u32TotPer - 1)));
    ui32ClkFreq = (SystemCoreClock) / ui32Denomfinal;                           // Compute the set frequency value
    ui32ClkFreq +=  (((SystemCoreClock) % ui32Denomfinal) > (ui32Denomfinal / 2)) ? 1 : 0;
    
    return ui32ClkFreq;
}

/**
******************************************************************************
** \brief  Enable an instance
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \return Ok on success, Error on error
**
******************************************************************************/
en_result_t ApolloIOM_Enable(IOMSTR0_Type* pstcInstance)
{
    en_result_t enRes = Error;
    stc_apolloiom_intern_data_t* pstcData = NULL;
    if (pstcInstance == NULL) return ErrorInvalidParameter;
    pstcData = GetInternDataPtr(pstcInstance);
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcInstance == IOMSTR_SWSPI)
       {
            ApolloGpio_GpioInputEnable(pstcData->u32MisoPin,TRUE);
            ApolloGpio_GpioOutputEnable(pstcData->u32MosiPin,TRUE);
            ApolloGpio_GpioOutputEnable(pstcData->u32SckPin,TRUE);

            ApolloGpio_GpioPullupEnable(pstcData->u32MisoPin,TRUE);
            return Ok;
       }
    #endif
    
    pstcInstance->CFG_b.IFCEN = 1;
    #if defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)
        #if defined(IOMSTR0)
            if (pstcInstance == IOMSTR0)
            {
                if (pstcData->enInterfaceMode == IomInterfaceModeSpi)
                {
                    GPIO->PADKEY = 0x00000073;            //unlock pin selection
                    GPIO->PADREGB_b.PAD5INPEN = 1;
                    GPIO->PADREGB_b.PAD6INPEN = 1;
                    
                    GPIO->CFGA_b.GPIO5OUTCFG = 1;
                    GPIO->CFGA_b.GPIO7OUTCFG = 1;
                    
                    GPIO->PADREGB_b.PAD5FNCSEL = 1;
                    GPIO->PADREGB_b.PAD6FNCSEL = 1; 
                    GPIO->PADREGB_b.PAD7FNCSEL = 1;
                    GPIO->PADKEY = 0x0000000;             //lock pin selection
                    enRes = Ok;
                }
            }
        #endif /* defined(IOMSTR0) */
        #if defined(IOMSTR1)
            if (pstcInstance == IOMSTR1)
            {
                if (pstcData->enInterfaceMode == IomInterfaceModeSpi)
                {
                    GPIO->PADKEY = 0x00000073;            //unlock pin selection
                    GPIO->PADREGC_b.PAD9INPEN = 1;
                    GPIO->PADREGC_b.PAD8INPEN = 1;
                    
                    GPIO->CFGB_b.GPIO8OUTCFG = 1;
                    GPIO->CFGB_b.GPIO10OUTCFG = 1;
                    
                    GPIO->PADREGC_b.PAD8FNCSEL = 1;
                    GPIO->PADREGC_b.PAD9FNCSEL = 1;
                    GPIO->PADREGC_b.PAD10FNCSEL = 1;
                    GPIO->PADKEY = 0x0000000;             //lock pin selection
                    enRes = Ok;
                }
            }
        #endif /* defined(IOMSTR1) */
    #endif /* defined(APOLLO_H) */
    return enRes;
}

/**
******************************************************************************
** \brief  Disable an instance
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \return Ok on success, Error on error
**
******************************************************************************/
en_result_t ApolloIOM_Disable(IOMSTR0_Type* pstcInstance)
{
    volatile uint32_t u32Timeout;
    en_result_t enRes = Error;
    stc_apolloiom_intern_data_t* pstcData;
    if (pstcInstance == NULL) return ErrorInvalidParameter;
    pstcData = GetInternDataPtr(pstcInstance);
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcInstance == IOMSTR_SWSPI)
       {
            ApolloGpio_GpioInputEnable(pstcData->u32MisoPin,FALSE);
            ApolloGpio_GpioOutputEnable(pstcData->u32MosiPin,FALSE);
            ApolloGpio_GpioOutputEnable(pstcData->u32SckPin,FALSE);

            ApolloGpio_GpioPullupEnable(pstcData->u32MisoPin,FALSE);
            return Ok;
       }
       
    #endif
       
    //Poll for IOM had completed the operation
    SystemCoreClockUpdate();
    u32Timeout = SystemCoreClock/5;
    while(u32Timeout > 0)
    {
        u32Timeout--;
        if (pstcInstance->STATUS_b.IDLEST == 1) break;
    }
    if (u32Timeout == 0) return ErrorTimeout;
    
    pstcInstance->CFG_b.IFCEN = 0;
    
    #if defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)
        #if defined(IOMSTR0)
            if (pstcInstance == IOMSTR0)
            {
                if (pstcData->enInterfaceMode == IomInterfaceModeSpi)
                {
                    GPIO->PADKEY = 0x00000073;            //unlock pin selection
                    GPIO->PADREGB_b.PAD5INPEN = 0;
                    GPIO->PADREGB_b.PAD6INPEN = 0;
                    GPIO->PADKEY = 0x0000000;             //lock pin selection
                    enRes = Ok;
                }
            }
        #endif /* defined(IOMSTR0) */
        #if defined(IOMSTR1)
            if (pstcInstance == IOMSTR1)
            {
                if (pstcData->enInterfaceMode == IomInterfaceModeSpi)
                {
                    GPIO->PADKEY = 0x00000073;            //unlock pin selection
                    GPIO->PADREGC_b.PAD8INPEN = 0;
                    GPIO->PADREGC_b.PAD9INPEN = 0;
                    GPIO->PADKEY = 0x0000000;             //lock pin selection
                    enRes = Ok;
                }
            }
        #endif /* defined(IOMSTR1) */
    #endif /* defined(APOLLO_H) || defined(APOLLO2_H) */
    return enRes;
}



/**
******************************************************************************
** \brief  Get frequency configuration
**
** \param u32Frequency    Operation: AM_HAL_IOM_READ or AM_HAL_IOM_WRITE
**
** \param pu32RealFrequency  Pointer to variable to return the real frequency. If pointer is NULL, this feature is ignored
**
** \param pu32ClkCfg     Pointer to the variable holding the clock configuraton
**
** \param bPhase      is equal to the bSPHA setting
**
** \return Ok on success, Error on error
**
******************************************************************************/
static en_result_t ApolloIOM_GetClockCfg(uint32_t u32Frequency, uint32_t* pu32RealFrequency, uint32_t* pu32ClkCfg, boolean_t bPhase)
{
    uint32_t u32Fsel, u32Div3, u32DivEn, u32TotPer, u32LowPer;	
    uint32_t u32Denom, u32v1, u32Denomfinal, u32ClkFreq, u32ClkCfg;
    int32_t i32Div, i32N;
    if ( u32Frequency == 0 )
    {
        return Error;
    }
    SystemCoreClockUpdate();
    
    //
    // Compute various parameters used for computing the optimal CLKCFG setting.
    //
    i32Div = (SystemCoreClock / u32Frequency) + ((SystemCoreClock % u32Frequency) ? 1 : 0);    // Round up (ceiling)
    
    
    //
    // Compute N (count the number of LS zeros of Div) = ctz(Div) = log2(Div & (-Div))
    //
    i32N = 31 - __CLZ((i32Div & (-i32Div)));
    
    if ( i32N > 6 )
    {
        i32N = 6;
    }
    
    u32Div3 = ( (u32Frequency < (SystemCoreClock / 16384))            ||
               ( ((u32Frequency >= (SystemCoreClock / 3))    &&
                  (u32Frequency <= ((SystemCoreClock / 2) - 1)) ) ) ) ? 1 : 0;
                  u32Denom = ( 1 << i32N ) * ( 1 + (u32Div3 * 2) );
                  u32TotPer = i32Div / u32Denom;
                  u32TotPer += (i32Div % u32Denom) ? 1 : 0;
                  u32v1 = 31 - __CLZ(u32TotPer);     // v1 = log2(TotPer)
                  u32Fsel = (u32v1 > 7) ? u32v1 + i32N - 7 : i32N;
                  u32Fsel++;
                  
      if ( u32Fsel > 7 )
      {
          //
          // This is an error, can't go that low.
          //
          return Error;
      }
      
      if ( u32v1 > 7 )
      {
          u32DivEn = u32TotPer;     // Save TotPer for the round up calculation
          u32TotPer = u32TotPer>>(u32v1-7);
          u32TotPer += ((u32DivEn) % (1 << (u32v1 - 7))) ? 1 : 0;
      }
      
      u32DivEn = ( (u32Frequency >= (SystemCoreClock / 4)) ||
                  ((1 << (u32Fsel - 1)) == i32Div) ) ? 0 : 1;
                  
      if (bPhase)
      {
          u32LowPer = (u32TotPer - 2) / 2;          // Longer high phase
      }
      else
      {
          u32LowPer = (u32TotPer - 1) / 2;          // Longer low phase
      }
      
      u32ClkCfg = (u32Fsel << IOMSTR0_CLKCFG_FSEL_Pos)                |
          (u32Div3 << IOMSTR0_CLKCFG_DIV3_Pos)                |
              (u32DivEn << IOMSTR0_CLKCFG_DIVEN_Pos)              |
                  (u32LowPer << IOMSTR0_CLKCFG_LOWPER_Pos)            |
                      ((u32TotPer - 1) << IOMSTR0_CLKCFG_TOTPER_Pos);
      
      
      //
      // Now, compute the actual frequency, which will be returned.
      //
      u32ClkFreq = compute_freq(u32Fsel, u32Div3, u32DivEn, u32TotPer);
      
      //
      // Determine if the actual frequency is a power of 2 (MHz).
      //
      if ( (u32ClkFreq % 250000) == 0 )
      {
          //
          // If the actual clock frequency is a power of 2 ranging from 250KHz up,
          // we can simplify the CLKCFG value using DIV3 (which also results in a
          // better duty cycle).
          //
          u32Denomfinal = u32ClkFreq / (uint32_t)250000;
          
          if ( onebit(u32Denomfinal) )
          {
              //
              // These configurations can be simplified by using DIV3.  Configs
              // using DIV3 have a 50% duty cycle, while those from DIVEN will
              // have a 66/33 duty cycle.
              //
              u32TotPer = u32LowPer = u32DivEn = 0;
              u32Div3 = 1;
              
              //
              // Now, compute the return values.
              //
              u32ClkFreq = compute_freq(u32Fsel, u32Div3, u32DivEn, u32TotPer);
              
              u32ClkCfg = (u32Fsel << IOMSTR0_CLKCFG_FSEL_Pos)                |
                  (1 << IOMSTR0_CLKCFG_DIV3_Pos)                |
                      (0 << IOMSTR0_CLKCFG_DIVEN_Pos)              |
                          (0 << IOMSTR0_CLKCFG_LOWPER_Pos)            |
                              (0 << IOMSTR0_CLKCFG_TOTPER_Pos);
          }
      }
      if (pu32RealFrequency != NULL) *pu32RealFrequency = u32ClkFreq;
      if (pu32ClkCfg != NULL) *pu32ClkCfg = u32ClkCfg;
      return Ok;
                              
}

/**
******************************************************************************
** \brief  Execute command
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32Operation    Operation: AM_HAL_IOM_READ or AM_HAL_IOM_WRITE
**
** \param u32ChipSelect   Chipselect number
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
******************************************************************************/
en_result_t ApolloIOM_Configure(IOMSTR0_Type* pstcInstance, const stc_apolloiom_config_t* pstcConfig)
{
    uint32_t u32Config = 0;
    en_result_t enRes = Error;
    stc_apolloiom_intern_data_t* pstcData;
    if (pstcInstance == NULL) return ErrorInvalidParameter;
    pstcData = GetInternDataPtr(pstcInstance);
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcInstance == IOMSTR_SWSPI)
       {
           return Error;
       }
    #endif
    #if defined(APOLLO2_H)
        switch((int)pstcInstance)
        {
        #if defined(IOMSTR0) && ((IOMSTR0_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
              case (int)IOMSTR0:
                PWRCTRL->DEVICEEN_b.IO_MASTER0 = 1;
        #endif
        #if defined(IOMSTR1) && ((IOMSTR1_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
              case (int)IOMSTR1:
                PWRCTRL->DEVICEEN_b.IO_MASTER1 = 1;
        #endif
        #if defined(IOMSTR2) && ((IOMSTR2_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
              case (int)IOMSTR2:
                PWRCTRL->DEVICEEN_b.IO_MASTER2 = 1;
        #endif
        #if defined(IOMSTR3) && ((IOMSTR3_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
              case (int)IOMSTR3:
                PWRCTRL->DEVICEEN_b.IO_MASTER3 = 1;
        #endif
        #if defined(IOMSTR4) && ((IOMSTR4_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
              case (int)IOMSTR4:
                PWRCTRL->DEVICEEN_b.IO_MASTER4 = 1;
        #endif
        #if defined(IOMSTR5) && ((IOMSTR5_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
              case (int)IOMSTR5:
                PWRCTRL->DEVICEEN_b.IO_MASTER5 = 1;
        #endif
        }
    #endif
    if (pstcConfig->enInterfaceMode == IomInterfaceModeSpi)
    {
        u32Config |= (1 << IOMSTR0_CFG_IFCSEL_Pos);
    }
    pstcData->enInterfaceMode = pstcConfig->enInterfaceMode;
    if (pstcConfig->bSPHA)
    {
        u32Config |= (1 << IOMSTR0_CFG_SPHA_Pos);
    }
    if (pstcConfig->bSPOL)
    {
        u32Config |= (1 << IOMSTR0_CFG_SPOL_Pos);
    }
    #if defined(APOLLO2_H)	
        if ( pstcConfig->u32ClockFrequency >= 16000000UL)
        {
            u32Config |= (2 << IOMSTR0_CFG_STARTRD_Pos);
        }
    #endif	
    pstcInstance->CFG = u32Config;
    
    pstcInstance->FIFOTHR = (pstcConfig->u8WriteThreshold << IOMSTR0_FIFOTHR_FIFOWTHR_Pos) | (pstcConfig->u8ReadThreshold << IOMSTR0_FIFOTHR_FIFORTHR_Pos);
    
    enRes = Error;
    
    
    if (ApolloIOM_GetClockCfg(pstcConfig->u32ClockFrequency,0,&u32Config,pstcConfig->bSPHA) == Ok)
    {
        pstcInstance->CLKCFG = u32Config;
        enRes = Ok;
    }
    
    return enRes;
}

/**
******************************************************************************
** \brief  Execute command
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32Operation    Operation: AM_HAL_IOM_READ or AM_HAL_IOM_WRITE
**
** \param u32ChipSelect   Chipselect number
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
******************************************************************************/
en_result_t ApolloIom_SpiCommand(IOMSTR0_Type* pstcHandle, uint32_t u32Operation, uint32_t u32ChipSelect, uint32_t u32NumBytes, uint32_t u32Options)
{
    uint32_t u32Command;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    //
    // Start building the command from the operation parameter.
    //
    u32Command = u32Operation;
    
    //
    // Set the transfer length (the length field is split, so this requires
    // some swizzling).
    //
    u32Command |= ((u32NumBytes & 0xF00) << 15);
    u32Command |= (u32NumBytes & 0xFF);
    
    //
    // Set the chip select number.
    //
    u32Command |= ((u32ChipSelect << 16) & 0x00070000);
    
    //
    // Finally, OR in the rest of the options. This mask should make sure that
    // erroneous option values won't interfere with the other transfer
    // parameters.
    //
    u32Command |= u32Options & 0x5C00FF00;
    
    //
    // Write the complete command word to the IOM command register.
    //
    pstcHandle->CMD = u32Command;
    
    return Ok;
}

/**
******************************************************************************
** \brief  Execute command
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32Operation    Operation: AM_HAL_IOM_READ or AM_HAL_IOM_WRITE
**
** \param ui32BusAddress  Bus Address
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
******************************************************************************/
en_result_t ApolloIom_I2cCommand(IOMSTR0_Type* pstcHandle, uint32_t u32Operation, uint32_t u32BusAddress, uint32_t u32NumBytes, uint32_t u32Options)
{
    uint32_t u32Command;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    //
    // Start building the command from the operation parameter.
    //
    u32Command = u32Operation;
    
    //
    // Set the transfer length (the length field is split, so this requires
    // some swizzling).
    //
    u32Command |= (u32NumBytes & 0xFF);
    
    //
    // Set the bus number.
    //
    u32Command |= ((u32BusAddress << 16) & 0x03FF0000);
    
    //
    // Finally, OR in the rest of the options. This mask should make sure that
    // erroneous option values won't interfere with the other transfer
    // parameters.
    //
    u32Command |= u32Options & 0x5C00FF00;
    
    //
    // Write the complete command word to the IOM command register.
    //
    pstcHandle->CMD = u32Command;
    
    return Ok;
}


/**
******************************************************************************
** \brief  Write data via SPI polled
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32ChipSelect   Chipselect number
**
** \param pu32Data        Data
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                bytes written (max. 64 bytes of fifo size)
******************************************************************************/
en_result_t ApolloIom_SpiWritePolled(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options)
{
    uint32_t u32Bw = 0;
    uint32_t u32Pos = 0;
    uint32_t u32NullCounter = 0;
    while(u32Pos < u32NumBytes)
    {
        ApolloIom_SpiWrite(pstcHandle,u32ChipSelect,&pu8Data[u32Pos],(u32NumBytes - u32Pos),&u32Bw,u32Options);
        if (u32Bw == 0) u32NullCounter++;
        if (u32NullCounter > 100) return Error;
        u32Pos += u32Bw;
    }
    if (pu32BytesWritten != NULL) *pu32BytesWritten = u32Pos;
    return Ok;
}


/**
******************************************************************************
** \brief  Write data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32ChipSelect   Chipselect number
**
** \param pu32Data        Data
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                bytes written (max. 64 bytes of fifo size)
******************************************************************************/
en_result_t ApolloIom_SpiWrite(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options)
{
    uint32_t tmp;
    volatile uint32_t u32Timeout;
    #if (APOLLOSWSPI_ENABLED == 1)
        stc_apolloiom_intern_data_t* pstcData;
        if (pstcHandle == IOMSTR_SWSPI)
        {
            if (pstcHandle == NULL) return ErrorInvalidParameter;
            pstcData = GetInternDataPtr(pstcHandle);
        }
    #endif
    
    if (pu32BytesWritten == NULL) pu32BytesWritten = &tmp;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcHandle == IOMSTR_SWSPI)
       {
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,FALSE);
           }
           for (tmp = 0;tmp < u32NumBytes;tmp++)
           {
               ApolloIom_SwSpiReadWrite(pstcData->u32MosiPin,pstcData->u32MisoPin,pstcData->u32SckPin,pu8Data[tmp]);
           }
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,TRUE);
           }
           if (pu32BytesWritten != NULL) *pu32BytesWritten = u32NumBytes;
           return Ok;
       }
    #endif
       
    //Poll for IOM had completed the operation
    SystemCoreClockUpdate();
    u32Timeout = SystemCoreClock/5;
    while(u32Timeout > 0)
    {
        u32Timeout--;
        if (pstcHandle->STATUS_b.IDLEST == 1) break;
    }
    if (u32Timeout == 0) 
    {
        return ErrorTimeout;
    }
    
    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
    
    if (u32NumBytes >= pstcHandle->FIFOPTR_b.FIFOREM)
    {
        return Error; //The fifo couldn't fit the requested number of bytes
    }
    
    u32NumBytes = DataToFifo(pstcHandle,pu8Data,u32NumBytes);
    /*
    //
    // Loop over the words in the array until we have the correct number of
    // bytes.
    //
    for(i = 0;(4*i) < u32NumBytes;i++)
    {
        //
        // Write the word to the FIFO.
        //
        pstcHandle->FIFO = pu32Data[i];
    }*/
    ApolloIom_SpiCommand(pstcHandle,AM_HAL_IOM_WRITE,u32ChipSelect,u32NumBytes,u32Options);
    *pu32BytesWritten = u32NumBytes;
    return Ok;
}

/**
******************************************************************************
** \brief  Write data via I2C
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32BusAddress   Bus Address
**
** \param pu32Data        Data
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                bytes written
******************************************************************************/
en_result_t ApolloIom_I2cWrite(IOMSTR0_Type* pstcHandle, uint32_t u32BusAddress, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options)
{
    uint32_t tmp;
    volatile uint32_t u32Timeout;
    
    if (pu32BytesWritten == NULL) pu32BytesWritten = &tmp;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    //Poll for IOM had completed the operation
    SystemCoreClockUpdate();
    u32Timeout = SystemCoreClock/5;
    while(u32Timeout > 0)
    {
        u32Timeout--;
        if (pstcHandle->STATUS_b.IDLEST == 1) break;
    }
    if (u32Timeout == 0) return ErrorTimeout;
    
    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
    
    if (u32NumBytes >= pstcHandle->FIFOPTR_b.FIFOREM)
    {
        return Error; //The fifo couldn't fit the requested number of bytes
    }
    
    u32NumBytes = DataToFifo(pstcHandle,pu8Data,u32NumBytes);

    ApolloIom_I2cCommand(pstcHandle,AM_HAL_IOM_WRITE,u32BusAddress,u32NumBytes,u32Options);
    *pu32BytesWritten = u32NumBytes;
    return Ok;
}

/**
******************************************************************************
** \brief  Read data via I2C
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32BusAddress   Bus Address
**
** \param pu32Data        Data
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                bytes read
******************************************************************************/
en_result_t ApolloIom_I2cRead(IOMSTR0_Type* pstcHandle, uint32_t u32BusAddress, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesRead, uint32_t u32Options)
{
    uint32_t u32ReadLen = 0;
    uint32_t tmp;
    volatile uint32_t u32Timeout;
    
    if (pu32BytesRead == NULL) pu32BytesRead = &tmp;
    
    //Poll for IOM had completed the operation
    SystemCoreClockUpdate();
    u32Timeout = SystemCoreClock/5;
    while(u32Timeout > 0)
    {
        u32Timeout--;
        if (pstcHandle->STATUS_b.IDLEST == 1) break;
    }
    if (u32Timeout == 0) return ErrorTimeout;
    
    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
    
    ApolloIom_I2cCommand(pstcHandle,AM_HAL_IOM_READ,u32BusAddress,u32NumBytes,u32Options);
    
    u32Timeout = 0;
    while(pstcHandle->STATUS_b.IDLEST == 0)
    {
        u32ReadLen = 0;
        if (pstcHandle->FIFOPTR_b.FIFOSIZ == u32NumBytes) //Check to see how much data is in the IOM fifo.
        {
            u32ReadLen = u32NumBytes;
        } else if (pstcHandle->FIFOPTR_b.FIFOSIZ >= 4)
        {
            u32ReadLen = pstcHandle->FIFOPTR_b.FIFOSIZ / 4;
            u32ReadLen = u32ReadLen * 4;
        }
        tmp = FifoToData(pstcHandle,pu8Data,u32ReadLen);
        *pu32BytesRead += tmp;
        pu8Data += tmp;
        if (u32ReadLen == 0) 
        {
            u32Timeout++;
            if (u32Timeout > SystemCoreClock/100) return Error;
        }
    }
    
    return Ok;
}


/**
******************************************************************************
** \brief  Read register data via I2C
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32BusAddress   Bus Address
**
** \param u8Register      Register
**
** \param pu8Data         Data
**
** \param u32Length       Data length
**
** \return                Ok on success
******************************************************************************/
en_result_t ApolloIom_I2cReadRegister(IOMSTR0_Type* pstcHandle, uint32_t u32BusAddress,uint8_t u8Register, uint8_t* pu8Data, uint32_t u32Length)
{
    en_result_t res = Ok;
    res = ApolloIom_I2cRead(pstcHandle,u32BusAddress,pu8Data,u32Length,NULL,AM_HAL_IOM_OFFSET(u8Register)); //| AM_HAL_IOM_RAW);
    return res;
}

/**
******************************************************************************
** \brief  Write register data via I2C
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32BusAddress   Bus Address
**
** \param u8Register      Register
**
** \param pu8Data         Data
**
** \param u32Length       Data length
**
** \return                Ok on success
******************************************************************************/
en_result_t ApolloIom_I2cWriteRegister(IOMSTR0_Type* pstcHandle, uint32_t u32BusAddress,uint8_t u8Register, uint8_t* pu8Data, uint32_t u32Length)
{
    en_result_t res = Ok;
    res = ApolloIom_I2cWrite(pstcHandle,u32BusAddress,pu8Data,u32Length,NULL,AM_HAL_IOM_OFFSET(u8Register));
    return res;
}

/**
******************************************************************************
** \brief  Read register data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32Chipselect   Chipselect
**
** \param u8Register      Register
**
** \param pu8Data         Data
**
** \param u32Length       Data length
**
** \return                Ok on success
******************************************************************************/
en_result_t ApolloIom_SpiReadRegister(IOMSTR0_Type* pstcHandle, uint32_t u32Chipselect,uint8_t u8Register, uint8_t* pu8Data, uint32_t u32Length)
{
    en_result_t res = Ok;
    res = ApolloIom_SpiWrite(pstcHandle,u32Chipselect,&u8Register,1,NULL,AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW);
    if (res != Ok) return res;
    while(pstcHandle->STATUS_b.IDLEST == 0) __NOP();
    res = ApolloIom_SpiRead(pstcHandle,u32Chipselect,pu8Data,u32Length,NULL,AM_HAL_IOM_RAW);
    return res;
}

/**
******************************************************************************
** \brief  Write register data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32Chipselect   Chipselect
**
** \param u8Register      Register
**
** \param pu8Data         Data
**
** \param u32Length       Data length
**
** \return                Ok on success
******************************************************************************/
en_result_t ApolloIom_SpiWriteRegister(IOMSTR0_Type* pstcHandle, uint32_t u32Chipselect,uint8_t u8Register, uint8_t* pu8Data, uint32_t u32Length)
{
    en_result_t res = Ok;
    res = ApolloIom_SpiWrite(pstcHandle,u32Chipselect,&u8Register,1,NULL,AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW);
    if (res != Ok) return res;
    while(pstcHandle->STATUS_b.IDLEST == 0) __NOP();
    res = ApolloIom_SpiWrite(pstcHandle,u32Chipselect,pu8Data,u32Length,NULL,AM_HAL_IOM_RAW);
    return res;
}


/**
******************************************************************************
** \brief  Read data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32ChipSelect   Chipselect number
**
** \param pu32Data        Data
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                bytes read
******************************************************************************/
en_result_t ApolloIom_SpiRead(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesRead, uint32_t u32Options)
{
    uint32_t u32ReadLen = 0;
    uint32_t tmp;
    volatile uint32_t u32Timeout;
    #if (APOLLOSWSPI_ENABLED == 1)
        stc_apolloiom_intern_data_t* pstcData;
        if (pstcHandle == IOMSTR_SWSPI)
        {
            if (pstcHandle == NULL) return ErrorInvalidParameter;
            pstcData = GetInternDataPtr(pstcHandle);
        }
    #endif
    
    if (pu32BytesRead == NULL) pu32BytesRead = &tmp;
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcHandle == IOMSTR_SWSPI)
       {
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,FALSE);
           }
           for (tmp = 0;tmp < u32NumBytes;tmp++)
           {
               pu8Data[tmp] = ApolloIom_SwSpiReadWrite(pstcData->u32MosiPin,pstcData->u32MisoPin,pstcData->u32SckPin,0x00);
           }
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,TRUE);
           }
           if (pu32BytesRead != NULL) *pu32BytesRead = u32NumBytes;
           return Ok;
       }
    #endif
       
    //Poll for IOM had completed the operation
    SystemCoreClockUpdate();
    u32Timeout = SystemCoreClock/5;
    while(u32Timeout > 0)
    {
        u32Timeout--;
        if (pstcHandle->STATUS_b.IDLEST == 1) break;
    }
    if (u32Timeout == 0) return ErrorTimeout;
    
    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
    
    ApolloIom_SpiCommand(pstcHandle,AM_HAL_IOM_READ,u32ChipSelect,u32NumBytes,u32Options);
    
    while(pstcHandle->STATUS_b.IDLEST == 0)
    {
        u32ReadLen = 0;
        if (pstcHandle->FIFOPTR_b.FIFOSIZ == u32NumBytes) //Check to see how much data is in the IOM fifo.
        {
            u32ReadLen = u32NumBytes;
        } else if (pstcHandle->FIFOPTR_b.FIFOSIZ >= 4)
        {
            u32ReadLen = pstcHandle->FIFOPTR_b.FIFOSIZ / 4;
            u32ReadLen = u32ReadLen * 4;
        }
        tmp = FifoToData(pstcHandle,pu8Data,u32ReadLen);
        *pu32BytesRead += tmp;
        pu8Data += tmp;
    }
    
    return Ok;
}

/**
******************************************************************************
** \brief  Write one byte data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32ChipSelect   Chipselect number
**
** \param u8Data          Data
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
******************************************************************************/
void ApolloIom_SpiWriteByte(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t u8Data, uint32_t u32Options)
{
    ApolloIom_SpiWrite(pstcHandle,u32ChipSelect,&u8Data,1,NULL,u32Options);
}

/**
******************************************************************************
** \brief  Read one byte data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32ChipSelect   Chipselect number
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                data
******************************************************************************/
uint8_t ApolloIom_SpiReadByte(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint32_t u32Options)
{
    uint8_t u8Data = 0;
    ApolloIom_SpiRead(pstcHandle,u32ChipSelect,&u8Data,1,NULL,u32Options);
    return u8Data;
}

#if APOLLOSWSPI_ENABLED == 1

/**
******************************************************************************
** \brief  Configure Software SPI
**
** \param u32MosiPin      GPIO for MOSI
**
** \param u32MisoPin      GPIO for MISO
**
** \param u32SckPin       GPIO for SCK
**
** \return Ok at success
**
******************************************************************************/
en_result_t ApolloIOM_ConfigureSwSpi(IOMSTR0_Type* pstcInstance, uint32_t u32MosiPin, uint32_t u32MisoPin, uint32_t u32SckPin)
{
    stc_apolloiom_intern_data_t* pstcData = NULL;
    if (pstcInstance == NULL) return ErrorInvalidParameter;
    pstcData = GetInternDataPtr(pstcInstance);
    
    if (pstcInstance == IOMSTR_SWSPI)
    {
        pstcData->u32MosiPin = u32MosiPin;
        pstcData->u32MisoPin = u32MisoPin;
        pstcData->u32SckPin = u32SckPin;
        return Ok;
    }
    return Error;
}

/**
 ******************************************************************************
 ** \brief  Transfert data via software SPI
 **
 ** \param u32MosiPin      GPIO for MOSI
 **
 ** \param u32MisoPin      GPIO for MISO
 **
 ** \param u32SckPin       GPIO for SCK
 **
 ** \param u8DataOut       Data to send
 **
 ** \return                Data read
 ******************************************************************************/
uint8_t ApolloIom_SwSpiReadWrite(uint32_t u32MosiPin, uint32_t u32MisoPin, uint32_t u32SckPin, uint8_t u8DataOut)
{
  uint8_t i;
  uint8_t u8DataIn = 0;

  for (i = 0; i < 8; i++) 
  {
        ApolloGpio_GpioSet(u32MosiPin,(u8DataOut & 0x01));
        u8DataOut = u8DataOut>>1;

        ApolloGpio_GpioSet(u32SckPin,TRUE);

        if (1 == ApolloGpio_GpioGet(u32MisoPin))
        {
            u8DataIn |= (1<<i);
        }

        ApolloGpio_GpioSet(u32SckPin,FALSE);
  }

  return u8DataIn;
}
#endif

/**
 ******************************************************************************
 ** \brief  Enable interrupts
 **
 ** \param pstcHandle     IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
 **
 ** \param u32Priority    Interrupt priority as defined in CMSIS
 **
 ** \param u32EnableMask  Enable mask:
 **                       APOLLOIOM_IRQ_ARB -> This is the arbitration loss interrupt.
 **                       APOLLOIOM_IRQ_STOP -> This is the STOP command interrupt.
 **                       APOLLOIOM_IRQ_START -> This is the START command interrupt.
 **                       APOLLOIOM_IRQ_ICMD -> This is the illegal command interrupt.
 **                       APOLLOIOM_IRQ_IACC -> This is the illegal FIFO access interrupt.
 **                       APOLLOIOM_IRQ_WTLEN -> This is the write length mismatch interrupt.
 **                       APOLLOIOM_IRQ_NAK -> This is the I2C NAK interrupt.
 **                       APOLLOIOM_IRQ_FOVFL -> This is the Read FIFO Overflow interrupt.
 **                       APOLLOIOM_IRQ_FUNDFL -> This is the Write FIFO Underflow interrupt.
 **                       APOLLOIOM_IRQ_THR -> This is the FIFO Threshold interrupt.
 **                       APOLLOIOM_IRQ_CMDCMP -> This is the Command Complete interrupt.
 **
 ** \return               Ok on success
 ******************************************************************************/
en_result_t ApolloIom_EnableInterrupts(IOMSTR0_Type* pstcInstance, uint32_t u32Priority, uint32_t u32EnableMask)
{
    uint32_t u32Tmp;
    IRQn_Type irqNum;
    switch((uint32_t)pstcInstance)
    {
        #if defined(IOMSTR0) && ((IOMSTR0_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR0:
            irqNum = IOMSTR0_IRQn;
            break;
        #endif
        #if defined(IOMSTR1) && ((IOMSTR1_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR1:
            irqNum = IOMSTR1_IRQn;
            break;
        #endif
        #if defined(IOMSTR2) && ((IOMSTR2_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR2:   
            irqNum = IOMSTR2_IRQn;
            break;
        #endif
        #if defined(IOMSTR3) && ((IOMSTR3_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR3:   
            irqNum = IOMSTR3_IRQn;
            break;
        #endif
        #if defined(IOMSTR4) && ((IOMSTR4_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR4:
            irqNum = IOMSTR4_IRQn;
            break;
        #endif
        #if defined(IOMSTR5) && ((IOMSTR5_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR5:
            irqNum = IOMSTR5_IRQn;
            break;
        #endif
        #if (APOLLOSWSPI_ENABLED == 1)
        case (uint32_t)IOMSTR_SWSPI:
            return Error;
        #endif
      default:
        return Error;
    }
    u32Tmp = pstcInstance->INTEN;
    pstcInstance->INTEN = 0;
    u32Tmp |= u32EnableMask;
    
    if (u32Tmp > 0)
    {
        NVIC_ClearPendingIRQ(irqNum);              //clear pending flag 
        NVIC_EnableIRQ(irqNum);                    //enable IRQ
        NVIC_SetPriority(irqNum,u32Priority);      //set priority of IRQ, smaller value means higher priority
    } else
    {
        NVIC_ClearPendingIRQ(irqNum);              //clear pending flag 
        NVIC_DisableIRQ(irqNum);                    //enable IRQ
    }
    pstcInstance->INTEN = u32Tmp;
    return Ok;
}

/**
 ******************************************************************************
 ** \brief  Disable interrupts
 **
 ** \param pstcHandle     IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
 **
 ** \param u32Priority    Interrupt priority as defined in CMSIS
 **
 ** \param u32DisableMask Disable mask:
 **                       APOLLOIOM_IRQ_ARB -> This is the arbitration loss interrupt.
 **                       APOLLOIOM_IRQ_STOP -> This is the STOP command interrupt.
 **                       APOLLOIOM_IRQ_START -> This is the START command interrupt.
 **                       APOLLOIOM_IRQ_ICMD -> This is the illegal command interrupt.
 **                       APOLLOIOM_IRQ_IACC -> This is the illegal FIFO access interrupt.
 **                       APOLLOIOM_IRQ_WTLEN -> This is the write length mismatch interrupt.
 **                       APOLLOIOM_IRQ_NAK -> This is the I2C NAK interrupt.
 **                       APOLLOIOM_IRQ_FOVFL -> This is the Read FIFO Overflow interrupt.
 **                       APOLLOIOM_IRQ_FUNDFL -> This is the Write FIFO Underflow interrupt.
 **                       APOLLOIOM_IRQ_THR -> This is the FIFO Threshold interrupt.
 **                       APOLLOIOM_IRQ_CMDCMP -> This is the Command Complete interrupt.
 **
 ** \return               Ok on success
 ******************************************************************************/
en_result_t ApolloIom_DisableInterrupts(IOMSTR0_Type* pstcInstance, uint32_t u32DisableMask)
{
    uint32_t u32Tmp;
    IRQn_Type irqNum;
    switch((uint32_t)pstcInstance)
    {
        #if defined(IOMSTR0) && ((IOMSTR0_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR0:
            irqNum = IOMSTR0_IRQn;
            break;
        #endif
        #if defined(IOMSTR1) && ((IOMSTR1_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR1:
            irqNum = IOMSTR1_IRQn;
            break;
        #endif
        #if defined(IOMSTR2) && ((IOMSTR2_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR2:   
            irqNum = IOMSTR2_IRQn;
            break;
        #endif
        #if defined(IOMSTR3) && ((IOMSTR3_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR3:   
            irqNum = IOMSTR3_IRQn;
            break;
        #endif
        #if defined(IOMSTR4) && ((IOMSTR4_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR4:
            irqNum = IOMSTR4_IRQn;
            break;
        #endif
        #if defined(IOMSTR5) && ((IOMSTR5_ENABLED == 1) || (APOLLOIOM_ENABLED == 1))
        case (uint32_t)IOMSTR5:
            irqNum = IOMSTR5_IRQn;
            break;
        #endif
        #if (APOLLOSWSPI_ENABLED == 1)
        case (uint32_t)IOMSTR_SWSPI:
            return Error;
        #endif
      default:
        return Error;
    }
    u32Tmp = pstcInstance->INTEN;
    pstcInstance->INTEN = 0;
    u32Tmp &= ~u32DisableMask;
    if (u32Tmp == 0)
    {
        NVIC_ClearPendingIRQ(irqNum);              //clear pending flag 
        NVIC_DisableIRQ(irqNum);                    //enable IRQ
    }
    pstcInstance->INTEN = u32Tmp;
    return Ok;
}

#endif /* (IOMSTR0_ENABLED == 1) && (IOMSTR1_ENABLED == 1) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


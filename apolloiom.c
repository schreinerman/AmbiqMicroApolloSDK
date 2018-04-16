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
**   - 2018-04-16  V1.4  MSc  Added more intelligent pin setup and prepared for mbed
**                            ,added fullduplex for Apollo2 B2 and SW SPI
**
*****************************************************************************/
#define __APOLLOIOM_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "apolloiom.h"
#include "string.h"

#if (APOLLOGPIOS_ENABLED == 0)
    #include "apollogpio.h"
#endif 

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#if (APOLLOSWSPI_ENABLED == 1) || (APOLLOIOM_ENABLED == 1) || (IOMSTR0_ENABLED == 1) || (IOMSTR1_ENABLED == 1) || (IOMSTR2_ENABLED == 1) || (IOMSTR3_ENABLED == 1) || (IOMSTR4_ENABLED == 1) || (IOMSTR5_ENABLED == 1)

#if (APOLLOGPIO_ENABLED == 0) && !defined(__APOLLOGPIO_H__)
    #error please define APOLLOGPIOS_ENABLED as 1 in RTE_Device.h and add the appollogpio module to your project
#endif

#define IOMGPIOS_COUNT (uint32_t)(sizeof(stcIomGpios) / sizeof(stcIomGpios[0]))
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

static const stc_apolloiom_gpio_func_t stcIomGpios[] = {
    //IOM,  , SCK/SCL, MOSI/SDA, MISO, SPI Func, I2C Func.
    {IOMSTR0, 5      , 6       ,  7  ,  1      , 0        },
    {IOMSTR1, 8      , 9       , 10  ,  1      , 0        },
#if defined(APOLLO2_H) || defined(APOLLO3_H)
    {IOMSTR2, 27     , 25      , 28  ,  5      , 4        },
    {IOMSTR3, 42     , 43      , 38  ,  5      , 4        },
    {IOMSTR4, 39     , 40      , 44  ,  5      , 4        },
    {IOMSTR5, 48     , 49      , 47  ,  5      , 4        },
#endif
#if defined(APOLLO2_H)
    {IOMSTR2, 0      , 1       ,  2  ,  5      , 7        },
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
** \brief Get IOM SPI handle by GPIO pins
**
** \param u8SckPin    SCK Pin
**
** \param u8MisoPin   MISO Pin
**
** \param u8MosiPin   MOSI Pin
**
** \return Return the IOM handle or NULL if no match was found
**
******************************************************************************/
IOMSTR0_Type* ApolloIOM_GetSpiByPin(uint8_t u8SckPin, uint8_t u8MisoPin, uint8_t u8MosiPin)
{
    uint32_t i = 0;
    for (i = 0; i < IOMGPIOS_COUNT;i++)
    {
        if ((stcIomGpios[i].stcGpios.u8SckPin == u8SckPin) &&
            (stcIomGpios[i].stcGpios.u8MisoPin == u8MisoPin) && 
            (stcIomGpios[i].stcGpios.u8MosiPin == u8MosiPin))
        {
            return stcIomGpios[i].pstcHandle;
        }
    }
    #if (APOLLOSWSPI_ENABLED == 1)
        return IOMSTR_SWSPI;
    #else
        return NULL;
    #endif
}

/**
******************************************************************************
** \brief Get IOM I2C handle by GPIO pins
**
** \param u8SclPin   SCL Pin
**
** \param u8SdaPin   SDA Pin
**
** \return Return the IOM handle or NULL if no match was found
**
******************************************************************************/
IOMSTR0_Type* ApolloIOM_GetI2cByPin(uint8_t u8SclPin, uint8_t u8SdaPin)
{
    uint32_t i;
    for (i = 0; i < IOMGPIOS_COUNT;i++)
    {
        if ((stcIomGpios[i].stcGpios.u8SclPin == u8SclPin) &&
            (stcIomGpios[i].stcGpios.u8SdaPin == u8SdaPin))
        {
            return stcIomGpios[i].pstcHandle;
        }
    }
    return NULL;
}

/**
******************************************************************************
** \brief Get GPIO pins by IOM instance
**
** \param pstcInstance   IOM Instance
**
** \return Return the struct of GPIOs or NULL if no match was found
**
******************************************************************************/
stc_apolloiom_gpios_t* ApolloIOM_GetPinsByInstance(IOMSTR0_Type* pstcInstance)
{
    uint32_t i;
    for (i = 0; i < IOMGPIOS_COUNT;i++)
    {
        if (stcIomGpios[i].pstcHandle == pstcInstance)
        {
            return (stc_apolloiom_gpios_t*)&stcIomGpios[i].stcGpios;
        }
    }
    return NULL;
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
            ApolloGpio_GpioInputEnable(pstcData->stcGpios.u8MisoPin,TRUE);
            ApolloGpio_GpioOutputEnable(pstcData->stcGpios.u8MosiPin,TRUE);
            ApolloGpio_GpioOutputEnable(pstcData->stcGpios.u8SckPin,TRUE);

            ApolloGpio_GpioPullupEnable(pstcData->stcGpios.u8MisoPin,TRUE);
            return Ok;
       }
    #endif
    
    pstcInstance->CFG_b.IFCEN = 1;

    if (pstcData->enInterfaceMode == IomInterfaceModeSpi)
    {
        ApolloGpio_GpioInputEnable(pstcData->stcGpios.u8MisoPin,TRUE);
        ApolloGpio_GpioInputEnable(pstcData->stcGpios.u8SckPin,TRUE);
        ApolloGpio_GpioOutputEnable(pstcData->stcGpios.u8MosiPin,TRUE);
        ApolloGpio_GpioOutputEnable(pstcData->stcGpios.u8SckPin,TRUE);
        enRes = Ok;
    }
   
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
            ApolloGpio_GpioInputEnable(pstcData->stcGpios.u8MisoPin,FALSE);
            ApolloGpio_GpioOutputEnable(pstcData->stcGpios.u8MosiPin,FALSE);
            ApolloGpio_GpioOutputEnable(pstcData->stcGpios.u8SckPin,FALSE);

            ApolloGpio_GpioPullupEnable(pstcData->stcGpios.u8MisoPin,FALSE);
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

    if (pstcData->enInterfaceMode == IomInterfaceModeSpi)
    {
        ApolloGpio_GpioInputEnable(pstcData->stcGpios.u8MisoPin,FALSE);
        ApolloGpio_GpioInputEnable(pstcData->stcGpios.u8SckPin,FALSE);
        enRes = Ok;
    }
  
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
    uint32_t i;
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
       
    pstcData->stcGpios.u8MisoPin = pstcConfig->stcGpios.u8MisoPin;
    pstcData->stcGpios.u8SckPin = pstcConfig->stcGpios.u8SckPin;
    pstcData->stcGpios.u8MosiPin = pstcConfig->stcGpios.u8MosiPin;
    
    // 
    // pre-check for valid GPIO configuration
    //
    if (pstcData->stcGpios.u8MisoPin == pstcData->stcGpios.u8SckPin == pstcData->stcGpios.u8MosiPin)
    {
        // 
        // no valid GPIO configuration found
        //
        
        // if SPI mode, set standard SPI settings
        if (pstcConfig->enInterfaceMode == IomInterfaceModeSpi)
        {
            for (i = 0; i < IOMGPIOS_COUNT;i++)
            {
                if (pstcInstance == stcIomGpios[i].pstcHandle)
                {
                     ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8MosiPin,stcIomGpios[i].u8FunctionSPI);
                     ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8MisoPin,stcIomGpios[i].u8FunctionSPI);
                     ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8SckPin,stcIomGpios[i].u8FunctionSPI);
                     break;
                }
            }
        } 
        // if I2C mode, set standard I2C settings
        else
        {
            for (i = 0; i < IOMGPIOS_COUNT;i++)
            {
                if (pstcInstance == stcIomGpios[i].pstcHandle)
                {
                     ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8SdaPin,stcIomGpios[i].u8FunctionI2C);
                     ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8SclPin,stcIomGpios[i].u8FunctionI2C);
                     break;
                }
            }
        }
    } else
    {
        // 
        // valid GPIO configuration found
        //
        
        // if SPI mode, set SPI settings
        if (pstcConfig->enInterfaceMode == IomInterfaceModeSpi)
        {
            for (i = 0; i < IOMGPIOS_COUNT;i++)
            {
                if ((pstcInstance == stcIomGpios[i].pstcHandle) && (pstcData->stcGpios.u8MosiPin == stcIomGpios[i].stcGpios.u8MosiPin))
                {
                    ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8MosiPin,stcIomGpios[i].u8FunctionSPI);
                }
                if ((pstcInstance == stcIomGpios[i].pstcHandle) && (pstcData->stcGpios.u8MisoPin == stcIomGpios[i].stcGpios.u8MisoPin))
                {
                    ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8MisoPin,stcIomGpios[i].u8FunctionSPI);
                }
                if ((pstcInstance == stcIomGpios[i].pstcHandle) && (pstcData->stcGpios.u8SckPin == stcIomGpios[i].stcGpios.u8SckPin))
                {
                    ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8SckPin,stcIomGpios[i].u8FunctionSPI);
                }
            }
        } 
        // if I2C mode, set I2C settings
        else
        {
            for (i = 0; i < IOMGPIOS_COUNT;i++)
            {
                if ((pstcInstance == stcIomGpios[i].pstcHandle) && (pstcData->stcGpios.u8SdaPin == stcIomGpios[i].stcGpios.u8SdaPin))
                {
                    ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8SdaPin,stcIomGpios[i].u8FunctionI2C);
                }
                if ((pstcInstance == stcIomGpios[i].pstcHandle) && (pstcData->stcGpios.u8SdaPin == stcIomGpios[i].stcGpios.u8SdaPin))
                {
                    ApolloGpio_GpioSelectFunction(pstcData->stcGpios.u8SdaPin,stcIomGpios[i].u8FunctionI2C);
                }
            }
        }
    }
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
        if (pstcConfig->bFullDuplex == TRUE)
        {
            u32Config |= (1 << IOMSTR0_CFG_FULLDUP_Pos);
        }
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
        ApolloIom_SpiWrite(pstcHandle,u32ChipSelect,&pu8Data[u32Pos],(u32NumBytes - u32Pos),&u32Bw,u32Options,NULL);
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
en_result_t ApolloIom_SpiWrite(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options,pfn_apollospi_rxtx_t pfnCallback)
{
    uint32_t tmp;
    volatile uint32_t u32Timeout;

    stc_apolloiom_intern_data_t* pstcData;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    pstcData = GetInternDataPtr(pstcHandle);
    
    if (pstcData == NULL) return Error;
    
    pstcData->pfnCallback = pfnCallback;
    pstcData->stcBuffer.pu8Data = pu8Data;
    pstcData->stcBuffer.pu8DataPos = pu8Data;
    pstcData->stcBuffer.u32BytesLeft = u32NumBytes;
    pstcData->stcBuffer.u32Chipselect = u32ChipSelect;
    pstcData->stcBuffer.u32Options = u32Options;
    pstcData->stcBuffer.u32State_b.TX = 1;
    
    if (pu32BytesWritten == NULL) pu32BytesWritten = &tmp;
    
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcHandle == IOMSTR_SWSPI)
       {
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,FALSE);
           }
           for (tmp = 0;tmp < u32NumBytes;tmp++)
           {
               ApolloIom_SwSpiReadWrite(pstcData->stcGpios.u8MosiPin,pstcData->stcGpios.u8MisoPin,pstcData->stcGpios.u8SckPin,pu8Data[tmp]);
           }
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,TRUE);
           }
           if (pu32BytesWritten != NULL) *pu32BytesWritten = u32NumBytes;
           
           if (pstcData->pfnCallback != NULL) 
           {
               pstcData->pfnCallback(pstcHandle,&pstcData->stcBuffer);
           }
           
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
    
    pstcData->stcBuffer.u32BytesLeft -= u32NumBytes;
    pstcData->stcBuffer.pu8DataPos += u32NumBytes;
    
    ApolloIom_SpiCommand(pstcHandle,AM_HAL_IOM_WRITE,u32ChipSelect,u32NumBytes,u32Options);
    
    *pu32BytesWritten = u32NumBytes;
    return Ok;
}


/**
******************************************************************************
** \brief  Read/Write data via SPI
**
** \param pstcHandle      IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
**
** \param u32ChipSelect   Chipselect number
**
** \param pu8DataOut      Data Out
**
** \param pu8DataOut      Data In
**
** \param u32NumBytes     Data size in bytes
**
** \param u32Options      Options: AM_HAL_IOM_CS_LOW, AM_HAL_IOM_LSB_FIRST, AM_HAL_IOM_RAW, AM_HAL_IOM_OFFSET(n)   
**
** \return                Ok, ErrorInvalidMode if IOM is not in fullduplex mode or other error
******************************************************************************/
en_result_t ApolloIom_SpiTransfer(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8DataOut, uint8_t* pu8DataIn, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options,pfn_apollospi_rxtx_t pfnCallback)
{
    uint32_t tmp;
    volatile uint32_t u32Timeout;

    stc_apolloiom_intern_data_t* pstcData;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    #if defined(APOLLO2_H)	
        if (pstcHandle->CFG_b.FULLDUP == 0)
        {
            return ErrorInvalidMode;
        }
    #else
        return ErrorInvalidMode;
    #endif
    
    pstcData = GetInternDataPtr(pstcHandle);
    
    if (pstcData == NULL) return Error;
    
    pstcData->pfnCallback = pfnCallback;
    pstcData->stcBuffer.pu8DataIN = pu8DataIn;
    pstcData->stcBuffer.pu8DataPosIN = pu8DataIn;
    pstcData->stcBuffer.pu8DataOUT = pu8DataOut;
    pstcData->stcBuffer.pu8DataPosOUT = pu8DataOut;
    pstcData->stcBuffer.u32BytesLeft = u32NumBytes;
    pstcData->stcBuffer.u32Chipselect = u32ChipSelect;
    pstcData->stcBuffer.u32Options = u32Options;
    pstcData->stcBuffer.u32State_b.TX = 1;
    pstcData->stcBuffer.u32State_b.RX = 1;
    
    if (pu32BytesWritten == NULL) pu32BytesWritten = &tmp;
    
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcHandle == IOMSTR_SWSPI)
       {
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,FALSE);
           }
           for (tmp = 0;tmp < u32NumBytes;tmp++)
           {
               pu8DataOut[tmp] = ApolloIom_SwSpiReadWrite(pstcData->stcGpios.u8MosiPin,pstcData->stcGpios.u8MisoPin,pstcData->stcGpios.u8SckPin,pu8DataIn[tmp]);
           }
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,TRUE);
           }
           if (pu32BytesWritten != NULL) *pu32BytesWritten = u32NumBytes;
           
           if (pstcData->pfnCallback != NULL) 
           {
               pstcData->pfnCallback(pstcHandle,&pstcData->stcBuffer);
           }
           
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
    
    u32NumBytes = DataToFifo(pstcHandle,pu8DataOut,u32NumBytes);
    
    pstcData->stcBuffer.u32BytesLeft -= u32NumBytes;
    pstcData->stcBuffer.pu8DataPos += u32NumBytes;

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
    res = ApolloIom_SpiWrite(pstcHandle,u32Chipselect,&u8Register,1,NULL,AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW,NULL);
    if (res != Ok) return res;
    while(pstcHandle->STATUS_b.IDLEST == 0) __NOP();
    res = ApolloIom_SpiRead(pstcHandle,u32Chipselect,pu8Data,u32Length,NULL,AM_HAL_IOM_RAW,NULL);
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
    res = ApolloIom_SpiWrite(pstcHandle,u32Chipselect,&u8Register,1,NULL,AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW,NULL);
    if (res != Ok) return res;
    while(pstcHandle->STATUS_b.IDLEST == 0) __NOP();
    res = ApolloIom_SpiWrite(pstcHandle,u32Chipselect,pu8Data,u32Length,NULL,AM_HAL_IOM_RAW,NULL);
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
en_result_t ApolloIom_SpiRead(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesRead, uint32_t u32Options,pfn_apollospi_rxtx_t pfnCallback)
{
    uint32_t u32ReadLen = 0;
    uint32_t tmp;
    volatile uint32_t u32Timeout;
    stc_apolloiom_intern_data_t* pstcData;
    
    if (pstcHandle == NULL) return ErrorInvalidParameter;
    
    pstcData = GetInternDataPtr(pstcHandle);
    
    if (pstcData == NULL) return Error;
    
    if (pu32BytesRead == NULL) pu32BytesRead = &tmp;
    
    *pu32BytesRead = 0;
    
    pstcData->pfnCallback = pfnCallback;
    pstcData->stcBuffer.pu8Data = pu8Data;
    pstcData->stcBuffer.pu8DataPos = pu8Data;
    pstcData->stcBuffer.u32BytesLeft = u32NumBytes;
    pstcData->stcBuffer.u32Chipselect = u32ChipSelect;
    pstcData->stcBuffer.u32Options = u32Options;
    pstcData->stcBuffer.u32State_b.RX = 1;
    
    #if (APOLLOSWSPI_ENABLED == 1)
       if (pstcHandle == IOMSTR_SWSPI)
       {
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,FALSE);
           }
           for (tmp = 0;tmp < u32NumBytes;tmp++)
           {
               pu8Data[tmp] = ApolloIom_SwSpiReadWrite(pstcData->stcGpios.u8MosiPin,pstcData->stcGpios.u8MisoPin,pstcData->stcGpios.u8SckPin,0x00);
           }
           if (u32Options & AM_HAL_IOM_CS_LOW)
           {
               ApolloGpio_GpioSet(u32ChipSelect,TRUE);
           }
           if (pu32BytesRead != NULL) *pu32BytesRead = u32NumBytes;
           
           if (pstcData->pfnCallback != NULL) 
           {
               pstcData->pfnCallback(pstcHandle,&pstcData->stcBuffer);
           }
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
    
    if (pfnCallback == NULL)
    {
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
    ApolloIom_SpiWrite(pstcHandle,u32ChipSelect,&u8Data,1,NULL,u32Options,NULL);
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
    ApolloIom_SpiRead(pstcHandle,u32ChipSelect,&u8Data,1,NULL,u32Options,NULL);
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
        pstcData->stcGpios.u8MosiPin = u32MosiPin;
        pstcData->stcGpios.u8MisoPin = u32MisoPin;
        pstcData->stcGpios.u8SckPin = u32SckPin;
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
 **
 ** Automatically clears pending flags using NVIC_ClearPendingIRQ(), enabled the interrupt using NVIC_EnableIRQ()
 ** and sets the priority using NVIC_SetPriority(). If there is no bit enabled, the function 
 ** automatically clears pending flags using NVIC_ClearPendingIRQ() and disbables the interrupt
 ** for the IOM instance using NVIC_DisableIRQ():
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

/**
 ******************************************************************************
 ** \brief  IRQ Handler, to be called from interrupt or from a RTOS
 **
 ** \param pstcHandle     IOM handle: IOMSTR0 or IOMSTR1 for Apollo, IOMSTR1..5 for Apollo 2
 **
 ** \param u32Priority    Interrupt priority as defined in CMSIS
 **
 ** \return               Ok on success
 **
 **
 ** If used with interrupt, IOMSTR<n>_USE_IRQS must be set for handling one instance
 ** or APOLLOIOM_USE_IRQS for handling all instances of the IOM module.
 **
 ** Example called from interrupt outside of the IOM module (IOMSTR0):
 ** @code
 ** void IOMSTR0_IRQHandler(void)
 ** {
 **     ApolloIom_IRQHandler(IOMSTR0);
 ** }
 ** @endcode
 **
 **
 ** Example called from operation system outside of the IOM module (IOMSTR0):
 ** @code
 ** ApolloIom_IRQHandler(IOMSTR0);
 ** @endcode
 **
 **
 ** Example IRQ handling in IOM module for IOMSTR0 only. (in RTE_Device.h):
 ** @code
 ** #define IOMSTR0_USE_IRQS 1
 ** @endcode
 **
 ******************************************************************************/
void ApolloIom_IRQHandler(IOMSTR0_Type* pstcInstance)
{
    stc_apolloiom_intern_data_t* pstcData;
    uint32_t IntStatus = pstcInstance->INTSTAT;
    uint32_t u32ReadLen,u32NumBytes;
    
    //check pstcInstance is valid
    if (pstcInstance == NULL) return;
    
    //get internal variables linked to the instance
    pstcData = GetInternDataPtr(pstcInstance);
    
    //check pstcData is valid
    if (pstcData == NULL) return;
    
    //retrieve the data in fifo
    u32ReadLen = pstcInstance->FIFOPTR_b.FIFOSIZ;
    
    //if the read length is > 0, read data and check if there is more data to read
    if (u32ReadLen > 0)
    {
        //get data and return the real read bytes in u32ReadLen
        u32ReadLen = FifoToData(pstcInstance,pstcData->stcBuffer.pu8DataPos,u32ReadLen);
        
        //update the buffer
        pstcData->stcBuffer.pu8DataPos += u32ReadLen;
        pstcData->stcBuffer.u32BytesLeft -= u32ReadLen;
        
        //check there is no more data to process
        if (pstcData->stcBuffer.u32BytesLeft == 0)
        {
            //no more data, close this transfer
            
            //check there is a callback function specified and execute
            if (pstcData->pfnCallback != NULL)
            {
                 pstcData->pfnCallback(pstcInstance,&pstcData->stcBuffer);
            }
            pstcData->stcBuffer.u32State_b.RX = 0;
            pstcData->stcBuffer.u32State_b.TX = 0;
            pstcInstance->INTCLR = IntStatus;
        }
        else
        {
            //more data, initiate next transfer
            
            //first clear the current status bits
            pstcInstance->INTCLR = IntStatus;
            
            //update the next data to read
            u32ReadLen = pstcData->stcBuffer.u32BytesLeft;
            u32ReadLen = (u32ReadLen <= MAX_FIFO_SIZE ? u32ReadLen : MAX_FIFO_SIZE);
            
            #if defined(APOLLO2_H)
                if ((pstcData->stcBuffer.u32State_b.TX == 1) && (pstcInstance->CFG_b.FULLDUP == 1))
                {
                    //initiate the in/out transfer
                    u32NumBytes = pstcData->stcBuffer.u32BytesLeft;
                    
                    u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
                    u32NumBytes = DataToFifo(pstcInstance,pstcData->stcBuffer.pu8DataPosOUT,u32NumBytes);
                    
                    pstcData->stcBuffer.u32BytesLeft -= u32NumBytes;
                    pstcData->stcBuffer.pu8DataPos += u32NumBytes;
                    
                    ApolloIom_SpiCommand(pstcInstance,AM_HAL_IOM_WRITE,pstcData->stcBuffer.u32Chipselect,u32NumBytes,pstcData->stcBuffer.u32Options);
                } else
            #endif
                {
                    //initiate the read transfer
                    ApolloIom_SpiCommand(pstcInstance,AM_HAL_IOM_READ,pstcData->stcBuffer.u32Chipselect,u32ReadLen,pstcData->stcBuffer.u32Options);
                }
        }
    }
    else
    {
        if (pstcData->stcBuffer.u32State_b.TX == 1)
        {
             //check there is no more data to process
            if (pstcData->stcBuffer.u32BytesLeft == 0)
            {
                //no more data, close this transfer
                
                //check there is a callback function specified and execute
                if (pstcData->pfnCallback != NULL)
                {
                     pstcData->pfnCallback(pstcInstance,&pstcData->stcBuffer);
                }
                pstcData->stcBuffer.u32State_b.RX = 0;
                pstcData->stcBuffer.u32State_b.TX = 0;
                pstcInstance->INTCLR = IntStatus;
            }
            else
            {
                pstcInstance->INTCLR = IntStatus;
                u32NumBytes = pstcData->stcBuffer.u32BytesLeft;
                
                u32NumBytes = (u32NumBytes <= MAX_FIFO_SIZE ? u32NumBytes : MAX_FIFO_SIZE);
                u32NumBytes = DataToFifo(pstcInstance,pstcData->stcBuffer.pu8DataPos,u32NumBytes);
                
                pstcData->stcBuffer.u32BytesLeft -= u32NumBytes;
                pstcData->stcBuffer.pu8DataPos += u32NumBytes;
                
                ApolloIom_SpiCommand(pstcInstance,AM_HAL_IOM_WRITE,pstcData->stcBuffer.u32Chipselect,u32NumBytes,pstcData->stcBuffer.u32Options);
            }
        } else 
        {
            //check there is a callback function specified and execute
            if (pstcData->pfnCallback != NULL)
            {
                 pstcData->pfnCallback(pstcInstance,&pstcData->stcBuffer);
            }
            pstcData->stcBuffer.u32State_b.RX = 0;
            pstcData->stcBuffer.u32State_b.TX = 0;
            pstcInstance->INTCLR = IntStatus;
        }
    }
}


#if defined(IOMSTR0) && ((IOMSTR0_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR0_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR0);
}
#endif

#if defined(IOMSTR1) && ((IOMSTR1_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR1_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR1);
}
#endif

#if defined(IOMSTR2) && ((IOMSTR2_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR2_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR2);
}
#endif

#if defined(IOMSTR3) && ((IOMSTR3_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR3_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR3);
}
#endif

#if defined(IOMSTR4) && ((IOMSTR4_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR4_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR4);
}
#endif

#if defined(IOMSTR5) && ((IOMSTR5_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR5_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR5);
}
#endif

#if defined(IOMSTR6) && ((IOMSTR6_USE_IRQS == 1) || (APOLLOIOM_USE_IRQS == 1))
void IOMSTR6_IRQHandler(void)
{
    ApolloIom_IRQHandler(IOMSTR6);
}
#endif

#endif /* (IOMSTR0_ENABLED == 1) && (IOMSTR1_ENABLED == 1) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


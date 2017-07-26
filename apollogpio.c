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
/** \file ApolloGpio.c
 **
 ** A detailed description is available at 
 ** @link ApolloGpioGroup Apollo GPIO description @endlink
 **
 ** History:
 **   - 2017-04-04  V1.0  MSc  First Version
 **   - 2017-04-13  V1.1  MSc  IRQs added, Smaller macro bug fixes
 **   - 2017-06-26  V1.2  MSc  Read pin added
 **   - 2017-07-26  V1.3  MSc  Fixed input setting
 **
 *****************************************************************************/
#define __APOLLOGPIO_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "apollogpio.h"

#if (APOLLOGPIO_ENABLED == 1)

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

static pfn_apollogpio_callback_t apfnCallbacks[64];
/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/


/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
#if (defined(APOLLO_H) || defined(APOLLO1_H)) && defined(APOLLOGPIO_USE_IRQS)
/**
 ******************************************************************************
 ** \brief  GPIO IRQ Handler
 **
 ******************************************************************************/
void GPIO_IRQHandler(void)
{
    uint32_t i;
    uint32_t u32Status;
    uint32_t u32Clear;
    
    u32Clear = 0;
    u32Status = GPIO->INT0STAT;
    for(i = 0; i < 32;i++)
    {
        if (u32Status & (1 << i))
        {
            if (apfnCallbacks[i] != NULL)
            {
                apfnCallbacks[i](i);
            }
            u32Clear |= (1 << i);
        }
    }
    GPIO->INT0CLR = u32Clear;
    
    u32Clear = 0;
    u32Status = GPIO->INT1STAT;
    for(i = 0; i < 32;i++)
    {
        if (u32Status & (1 << i))
        {
            if (apfnCallbacks[i + 32] != NULL)
            {
                apfnCallbacks[i + 32](i);
            }
            u32Clear |= (1 << i);
        }
    }
    GPIO->INT1CLR = u32Clear;
}
#endif

/**
 ******************************************************************************
 ** \brief  Check an IRQ is enabled
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ******************************************************************************/
boolean_t ApolloGpio_IrqIsEnabled(uint32_t pin)
{
    if (pin < 32)
    {
        if ((GPIO->INT0EN & (1 << pin)) != 0)
        {
            return TRUE;
        }
    } else
    {
        if ((GPIO->INT1EN & (1 << (pin - 32))) != 0)
        {
            return TRUE;
        }
    }
    return FALSE;
}

/**
 ******************************************************************************
 ** \brief  Check if an IRQ is pending
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ******************************************************************************/
boolean_t ApolloGpio_IrqIsPending(uint32_t pin)
{
    if (pin < 32)
    {
        if ((GPIO->INT0STAT & (1 << pin)) != 0)
        {
            return TRUE;
        }
    } else
    {
        if ((GPIO->INT1STAT & (1 << (pin - 32))) != 0)
        {
            return TRUE;
        }
    }
    return FALSE;
}

/**
 ******************************************************************************
 ** \brief  Execute an IRQ callback virtually
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ******************************************************************************/
boolean_t ApolloGpio_IrqExecute(uint32_t pin)
{
    if (pin < 32)
    {
        if ((GPIO->INT0STAT & (1 << pin)) != 0)
        {
            if (apfnCallbacks[pin] != NULL)
            {
                apfnCallbacks[pin](pin);
            }
            GPIO->INT0CLR |= (1 << pin);
            return TRUE;
        }
    } else
    {
        if ((GPIO->INT1STAT & (1 << (pin - 32))) != 0)
        {
            if (apfnCallbacks[pin-32] != NULL)
            {
                apfnCallbacks[pin-32](pin);
            }
            GPIO->INT1CLR |= (1 << (pin-32));
            return TRUE;
        }
    }
    return FALSE;
}

/**
 ******************************************************************************
 ** \brief  Register a new callback
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param enMode  Can be GpioRisingEdge or GpioFallingEdge
 **
 ** \param pfnCallback Callback function
 **
 ******************************************************************************/
void ApolloGpio_RegisterIrq(uint32_t pin, en_apollogpio_edgedetect_t enMode, pfn_apollogpio_callback_t pfnCallback)
{
    if (pin < 32)
    {
        GPIO->INT0EN &= ~(1 << pin);
    } else
    {
        GPIO->INT1EN &= ~(1 << (pin - 32));
    }
    
    GPIO->PADKEY = 0x00000073;
    switch(enMode)
    {
        case GpioRisingEdge:
          APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0INTD_Msk,(0 << GPIO_CFGA_GPIO0INTD_Pos));
          break;
        case GpioFallingEdge:
          APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0INTD_Msk,(1 << GPIO_CFGA_GPIO0INTD_Pos));
          break;

    }
    GPIO->PADKEY = 0x00000000;
    
    if (pin < 32)
    {
        GPIO->INT0EN |= (1 << pin);
    } else
    {
        GPIO->INT1EN |= (1 << (pin - 32));
    }
}

/**
 ******************************************************************************
 ** \brief  Set a value for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param bOnOff TRUE to set a logical high, FALSE for a logical FALSE
 **
 ******************************************************************************/
void ApolloGpio_GpioSet(uint32_t pin, boolean_t bOnOff)
{
    if (bOnOff)
    {
      if (pin < 32)
      {
          GPIO->WTSA = (1 << (pin));
      } else
      {
          GPIO->WTSB = (1 << (pin - 32));
      }
    } else
    {
      if (pin < 32)
      {
          GPIO->WTCA = (1 << (pin));
      } else
      {
          GPIO->WTCB = (1 << (pin - 32));
      }
    }
}

/**
 ******************************************************************************
 ** \brief  Read a value for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \return bOnOff TRUE to set a logical high, FALSE for a logical FALSE
 **
 ******************************************************************************/
boolean_t ApolloGpio_GpioGet(uint32_t pin)
{
    if (pin < 32)
    {
      if (GPIO->RDA & (1 << (pin)))
      {
          return TRUE;
      }
    } else
    {
      if (GPIO->RDB & (1 << (pin - 32)))
      {
          return TRUE;
      }
    }
    return FALSE;
}

/**
 ******************************************************************************
 ** \brief  Set a pullup for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param bOnOff TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioPullupEnable(uint32_t pin, boolean_t bEnable)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,GPIO_PADREGA_PAD0PULL_Msk,(bEnable << GPIO_PADREGA_PAD0PULL_Pos));
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Enable the input for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param bOnOff TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioInputEnable(uint32_t pin, boolean_t bEnable)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,GPIO_PADREGA_PAD0INPEN_Msk,(bEnable << GPIO_PADREGA_PAD0INPEN_Pos));
    //APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0INCFG_Msk,(bEnable << GPIO_CFGA_GPIO0INCFG_Pos));
    ApolloGpio_GpioSelectFunction(pin,3);
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Enable the strength output for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param bOnOff TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioStrengthEnable(uint32_t pin, boolean_t bEnable)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,GPIO_PADREGA_PAD0STRNG_Msk,(bEnable << GPIO_PADREGA_PAD0STRNG_Pos));
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Set the output configuration for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param enMode Can be GpioOutputDisabled, GpioPushPull, GpioOpenDrain, GpioTriState
 **
 ******************************************************************************/
void ApolloGpio_GpioOutputConfiguration(uint32_t pin, en_apollogpio_mode_t enMode)
{
    GPIO->PADKEY = 0x00000073;
    switch(enMode)
    {
        case GpioOutputDisabled:
          APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0OUTCFG_Msk,(0 << GPIO_CFGA_GPIO0OUTCFG_Pos));
          break;
        case GpioPushPull:
          APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0OUTCFG_Msk,(1 << GPIO_CFGA_GPIO0OUTCFG_Pos));
          break;
        case GpioOpenDrain:
          APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0OUTCFG_Msk,(2 << GPIO_CFGA_GPIO0OUTCFG_Pos));
          break;
        case GpioTriState:
          APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0OUTCFG_Msk,(3 << GPIO_CFGA_GPIO0OUTCFG_Pos));
          break;
    }
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Enable the output for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param bEnable TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioOutputEnable(uint32_t pin, boolean_t bEnable)
{
    if (bEnable)
    {
        ApolloGpio_GpioSelectFunction(pin,3);
        ApolloGpio_GpioOutputConfiguration(pin,GpioPushPull);
    } else
    {
        ApolloGpio_GpioSelectFunction(pin,3);
        ApolloGpio_GpioOutputConfiguration(pin,GpioOutputDisabled); 
    }
}

/**
 ******************************************************************************
 ** \brief  Set the function for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param u8Function Can be 0..7
 **
 ******************************************************************************/
void ApolloGpio_GpioSelectFunction(uint32_t pin, uint8_t u8Function)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,GPIO_PADREGA_PAD0FNCSEL_Msk,((u8Function & 0x7) << GPIO_PADREGA_PAD0FNCSEL_Pos));
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Set the function for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin with high side switch 
 **             for example 1, 2, ... 49
 **
 ** \param bOnOff TRUE for on, FALSE for off
 **
 ******************************************************************************/
void ApolloGpio_GpioSetHighSwitch(uint32_t pin, boolean_t bOnOff)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,(1 << 7),(bOnOff << 7));
    GPIO->PADKEY = 0x00000000;
}

#endif /* (APOLLOGPIO_ENABLED == 1) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


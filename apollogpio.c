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
 **   - 2017-04-04  V1.0  Manuel Schreiner   First Version
 **   - 2017-04-13  V1.1  Manuel Schreiner   IRQs added, Smaller macro bug fixes
 **   - 2017-06-26  V1.2  Manuel Schreiner   Read pin added
 **   - 2018-04-17  V1.3  Manuel Schreiner   Removed ApolloGpio_GpioSelectFunction(pin,3); in 
 **                                          enabled as input / output
 **   - 2018-07-06  V1.4  Manuel Schreiner   Updated documentation, 
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **   - 2018-10-29  V1.5  Manuel Schreiner   Fixed IRQ handling
 **                                          Added Arduino API, enable via RTE_Device.h APOLLOGPIO_USE_ARDUINO
 **
 *****************************************************************************/
#define __APOLLOGPIO_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "apollogpio.h"
#include "mcu.h"

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
#if (defined(APOLLO_H) || defined(APOLLO1_H) || defined(APOLLO2_H)) && defined(APOLLOGPIO_USE_IRQS)
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
                apfnCallbacks[i + 32](i + 32);
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
boolean_t ApolloGpio_IrqIsEnabled(apollogpio_gpio_pin_t pin)
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
boolean_t ApolloGpio_IrqIsPending(apollogpio_gpio_pin_t pin)
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
boolean_t ApolloGpio_IrqExecute(apollogpio_gpio_pin_t pin)
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
            if (apfnCallbacks[pin] != NULL)
            {
                apfnCallbacks[pin](pin);
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
 ** \param u32Priority    Interrupt priority as defined in CMSIS
 **
 ** \param pfnCallback Callback function
 **
 ******************************************************************************/
void ApolloGpio_RegisterIrq(apollogpio_gpio_pin_t pin, en_apollogpio_edgedetect_t enMode, uint32_t u32Priority, pfn_apollogpio_callback_t pfnCallback)
{
    uint32_t u32Status;

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
    if (apfnCallbacks[pin] != pfnCallback)
    {
        apfnCallbacks[pin] = pfnCallback;
        if (pin < 32)
        {
            u32Status = (GPIO->INT0STAT & (1 << pin));
            GPIO->INT0CLR = u32Status;
            GPIO->INT0EN |= (1 << pin);
        } else
        {
            u32Status = (GPIO->INT1STAT & (1 << (pin - 32)));
            GPIO->INT1CLR = u32Status;
            GPIO->INT1EN |= (1 << (pin - 32));
        }
        
        if ((GPIO->INT0EN > 0) || (GPIO->INT1EN > 0))
        {
            NVIC_DisableIRQ(GPIO_IRQn);                    //enable IRQ
            NVIC_ClearPendingIRQ(GPIO_IRQn);              //clear pending flag 
            NVIC_EnableIRQ(GPIO_IRQn);                    //enable IRQ
            NVIC_SetPriority(GPIO_IRQn,u32Priority);      //set priority of IRQ, smaller value means higher priority
        } else
        {
            NVIC_ClearPendingIRQ(GPIO_IRQn);              //clear pending flag 
            NVIC_DisableIRQ(GPIO_IRQn);                    //enable IRQ
        }
    } else if (pin < 32)
    {
        u32Status = (GPIO->INT0STAT & (1 << pin));
        GPIO->INT0CLR = u32Status;
        GPIO->INT0EN |= (1 << pin);
    } else
    {
        u32Status = (GPIO->INT1STAT & (1 << (pin - 32)));
        GPIO->INT1CLR = u32Status;
        GPIO->INT1EN |= (1 << (pin - 32));
    }
    
}

/**
 ******************************************************************************
 ** \brief  Register a new callback
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ******************************************************************************/
void ApolloGpio_UnRegisterIrq(apollogpio_gpio_pin_t pin)
{
    uint32_t u32Status;
    if (pin < 32)
    {
        GPIO->INT0EN &= ~(1 << pin);
    } else
    {
        GPIO->INT1EN &= ~(1 << (pin - 32));
    }
    apfnCallbacks[pin] = NULL;
    
    if (pin < 32)
    {
        u32Status = (GPIO->INT0STAT & (1 << pin));
        GPIO->INT0CLR = u32Status;
    } else
    {
        u32Status = (GPIO->INT1STAT & (1 << (pin - 32)));
        GPIO->INT1CLR = u32Status;
    }
    
    if (GPIO->INT0EN == 0)
    {
        NVIC_ClearPendingIRQ(GPIO_IRQn);              //clear pending flag 
        NVIC_DisableIRQ(GPIO_IRQn);                    //enable IRQ
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
void ApolloGpio_GpioSet(apollogpio_gpio_pin_t pin, boolean_t bOnOff)
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
 ** \brief  Get the register address and mask for a specific pin for the data out register
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param [out] pstcRegisterMaskPair pointer to store register & mask pair
 **
 ** \return Ok 
 **
 ** \details Example:
 ** @code  
 ** stc_apollogpio_register_mask_pair_t stcTogglePin;
 **
 ** ApolloGpio_GpioOutputEnable(42,TRUE);
 ** ApolloGpio_GetRegisterAddressDataOut(42,&stcTogglePin);
 **
 ** while(1)
 ** {
 **     *stcTogglePin.pRegister |= stcTogglePin.u32Mask;  //set   GPIO 42
 **     *stcTogglePin.pRegister &= ~stcTogglePin.u32Mask; //clear GPIO 42
 ** }
 **
 ** @endcode
 **
 ** Alternatively following code is doing the same, but slower:
 ** @code  
 ** ApolloGpio_GpioOutputEnable(42,TRUE);
 ** while(1)
 ** {
 **     ApolloGpio_GpioSet(42,TRUE);
 **     ApolloGpio_GpioSet(42,FALSE);
 ** }
 ** @endcode
 **
 ******************************************************************************/
en_result_t ApolloGpio_GetRegisterAddressDataOut(apollogpio_gpio_pin_t pin,stc_apollogpio_register_mask_pair_t* pstcRegisterMaskPair)
{
    if (pin < 32)
    {
       pstcRegisterMaskPair->u32Pin = pin;
       pstcRegisterMaskPair->u32Mask = (1 << pin);
       pstcRegisterMaskPair->pRegister = &GPIO->WTA; 
       return Ok;
    } else if (pin < 49)
    {
       pstcRegisterMaskPair->u32Pin = pin;
       pstcRegisterMaskPair->u32Mask = (1 << (pin-32));
       pstcRegisterMaskPair->pRegister = &GPIO->WTB; 
       return Ok;
    }
    return Error;
}

/**
 ******************************************************************************
 ** \brief  Get the register address and mask for a specific pin for the data out set bit register
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param [out] pstcRegisterMaskPair pointer to store register & mask pair
 **
 ** \return Ok 
 **
 ** \details Example:
 ** @code  
 ** stc_apollogpio_register_mask_pair_t stcTogglePinSet;
 ** stc_apollogpio_register_mask_pair_t stcTogglePinClear;
 **
 ** ApolloGpio_GpioOutputEnable(42,TRUE);
 ** ApolloGpio_GetRegisterAddressDataOutSet(42,&stcTogglePinSet);
 ** ApolloGpio_GetRegisterAddressDataOutClear(42,&stcTogglePinClear);
 ** while(1)
 ** {
 **     *stcTogglePinSet.pRegister    = stcTogglePinSet.u32Mask;   //set   GPIO 42
 **     *stcTogglePinClear.pRegister  = stcTogglePinClear.u32Mask; //clear GPIO 42
 ** }
 **
 ** @endcode
 **
 ** Alternatively following code is doing the same, but slower:
 ** @code  
 ** ApolloGpio_GpioOutputEnable(42,TRUE);
 ** while(1)
 ** {
 **     ApolloGpio_GpioSet(42,TRUE);
 **     ApolloGpio_GpioSet(42,FALSE);
 ** }
 ** @endcode
 **
 ******************************************************************************/
en_result_t ApolloGpio_GetRegisterAddressDataOutSet(apollogpio_gpio_pin_t pin,stc_apollogpio_register_mask_pair_t* pstcRegisterMaskPair)
{
    if (pin < 32)
    {
       pstcRegisterMaskPair->u32Pin = pin;
       pstcRegisterMaskPair->u32Mask = (1 << pin);
       pstcRegisterMaskPair->pRegister = &GPIO->WTSA; 
       return Ok;
    } else if (pin < 49)
    {
       pstcRegisterMaskPair->u32Pin = pin;
       pstcRegisterMaskPair->u32Mask = (1 << (pin-32));
       pstcRegisterMaskPair->pRegister = &GPIO->WTSB; 
       return Ok;
    }
    return Error;
}

/**
 ******************************************************************************
 ** \brief  Get the register address and mask for a specific pin for the data out clear bit register
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param [out] pstcRegisterMaskPair pointer to store register & mask pair
 **
 ** \return Ok 
 **
 ** \details Example:
 ** @code  
 ** stc_apollogpio_register_mask_pair_t stcTogglePinSet;
 ** stc_apollogpio_register_mask_pair_t stcTogglePinClear;
 **
 ** ApolloGpio_GpioOutputEnable(42,TRUE);
 ** ApolloGpio_GetRegisterAddressDataOutSet(42,&stcTogglePinSet);
 ** ApolloGpio_GetRegisterAddressDataOutClear(42,&stcTogglePinClear);
 ** while(1)
 ** {
 **     *stcTogglePinSet.pRegister    = stcTogglePinSet.u32Mask;   //set   GPIO 42
 **     *stcTogglePinClear.pRegister  = stcTogglePinClear.u32Mask; //clear GPIO 42
 ** }
 **
 ** @endcode
 **
 ** Alternatively following code is doing the same, but slower:
 ** @code  
 ** ApolloGpio_GpioOutputEnable(42,TRUE);
 ** while(1)
 ** {
 **     ApolloGpio_GpioSet(42,TRUE);
 **     ApolloGpio_GpioSet(42,FALSE);
 ** }
 ** @endcode
 **
 ******************************************************************************/
en_result_t ApolloGpio_GetRegisterAddressDataOutClear(apollogpio_gpio_pin_t pin,stc_apollogpio_register_mask_pair_t* pstcRegisterMaskPair)
{
    if (pin < 32)
    {
       pstcRegisterMaskPair->u32Pin = pin;
       pstcRegisterMaskPair->u32Mask = (1 << pin);
       pstcRegisterMaskPair->pRegister = &GPIO->WTCA; 
       return Ok;
    } else if (pin < 49)
    {
       pstcRegisterMaskPair->u32Pin = pin;
       pstcRegisterMaskPair->u32Mask = (1 << (pin-32));
       pstcRegisterMaskPair->pRegister = &GPIO->WTCB; 
       return Ok;
    }
    return Error;
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
boolean_t ApolloGpio_GpioGet(apollogpio_gpio_pin_t pin)
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
 ** \param bEnable TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioPullupEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable)
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
 ** \param bEnable TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioInputEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,GPIO_PADREGA_PAD0INPEN_Msk,(bEnable << GPIO_PADREGA_PAD0INPEN_Pos));
    //APOLLOGPIO_CFG_WRITE(pin,GPIO_CFGA_GPIO0INCFG_Msk,(bEnable << GPIO_CFGA_GPIO0INCFG_Pos));
    //ApolloGpio_GpioSelectFunction(pin,3);
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Enable the strength output for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin  
 **             for example 1, 2, ... 49
 **
 ** \param bEnable TRUE to turn on, FALSE to turn off
 **
 ******************************************************************************/
void ApolloGpio_GpioStrengthEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable)
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
void ApolloGpio_GpioOutputConfiguration(apollogpio_gpio_pin_t pin, en_apollogpio_mode_t enMode)
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
void ApolloGpio_GpioOutputEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable)
{
    if (bEnable)
    {
        //ApolloGpio_GpioSelectFunction(pin,3);
        ApolloGpio_GpioOutputConfiguration(pin,GpioPushPull);
    } else
    {
        //ApolloGpio_GpioSelectFunction(pin,3);
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
void ApolloGpio_GpioSelectFunction(apollogpio_gpio_pin_t pin, uint8_t u8Function)
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
void ApolloGpio_GpioSetHighSwitch(apollogpio_gpio_pin_t pin, boolean_t bOnOff)
{
    GPIO->PADKEY = 0x00000073;
    APOLLOGPIO_PADREG_WRITE(pin,(1 << 7),(bOnOff << 7));
    GPIO->PADKEY = 0x00000000;
}

/**
 ******************************************************************************
 ** \brief  Set the pull-up for a specified GPIO
 **
 ** \param pin  Can be every GPIO pin with selectable pull-ups
 **             for example 0,1, 2, ... 49
 **
 ** \param enPullUp Pullup Type
 **
 ******************************************************************************/
void ApolloGpio_GpioSelectPullup(apollogpio_gpio_pin_t pin, en_apollogpio_pullup_t enPullUp)
{
    GPIO->PADKEY = 0x00000073;
    switch(enPullUp)
    {
      case PullUp1K5:
        APOLLOGPIO_PADREG_WRITE(pin,(0x3 << 6),(0 << 6));
        break;
      case PullUp6K:
        APOLLOGPIO_PADREG_WRITE(pin,(0x3 << 6),(1 << 6));
        break;
      case PullUp12K:
        APOLLOGPIO_PADREG_WRITE(pin,(0x3 << 6),(2 << 6));
        break;
      case PullUp24K:
        APOLLOGPIO_PADREG_WRITE(pin,(0x3 << 6),(3 << 6));
        break;  
    }
    GPIO->PADKEY = 0x00000000;
}

#if APOLLOGPIO_USE_ARDUINO == 1
void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin > 50) return;
    ApolloGpio_GpioSelectFunction(pin,3);
    if (mode == INPUT) { 
        ApolloGpio_GpioOutputEnable(pin,FALSE);
        ApolloGpio_GpioInputEnable(pin,TRUE);
        ApolloGpio_GpioPullupEnable(pin,FALSE);
    } else if (mode == INPUT_PULLUP) {
        ApolloGpio_GpioOutputEnable(pin,FALSE);
	ApolloGpio_GpioInputEnable(pin,TRUE);
        ApolloGpio_GpioPullupEnable(pin,TRUE);
    } else {
	ApolloGpio_GpioOutputEnable(pin,TRUE);
        ApolloGpio_GpioInputEnable(pin,FALSE);
        ApolloGpio_GpioPullupEnable(pin,FALSE);
    }
}
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode) 
{
   if (interruptNum > 50) return;
   switch(mode)
   {
       case LOW:
           ApolloGpio_RegisterIrq(interruptNum,GpioFallingEdge,1,(pfn_apollogpio_callback_t)userFunc);
           if (digitalRead(interruptNum) == LOW)
           {
               userFunc();
           }
           break;
       case HIGH:
           ApolloGpio_RegisterIrq(interruptNum,GpioRisingEdge,1,(pfn_apollogpio_callback_t)userFunc);
           if (digitalRead(interruptNum) == HIGH)
           {
               userFunc();
           }
           break;
       case RISING:
           ApolloGpio_RegisterIrq(interruptNum,GpioRisingEdge,1,(pfn_apollogpio_callback_t)userFunc);
           break;
       case FALLING:
           ApolloGpio_RegisterIrq(interruptNum,GpioRisingEdge,1,(pfn_apollogpio_callback_t)userFunc);
           break;
   }
    
}
#endif

#else
#warning Low-Level-Driver for Apollo 1/2 GPIO is disabled and could be removed from the project
#endif /* (APOLLOGPIO_ENABLED == 1) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


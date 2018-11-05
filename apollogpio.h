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
/** \file ApolloGpio.h
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
 **
 *****************************************************************************/
#ifndef __APOLLOGPIO_H__
#define __APOLLOGPIO_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup ApolloGpioGroup  Low-Level-Driver for Apollo 1/2 GPIO
 **
 ** Provided functions of ApolloGpio:
 ** - ApolloGpio_GpioSet()                 - set/clear GPIO
 ** - ApolloGpio_GpioGet()                 - read GPIO
 ** - ApolloGpio_GpioPullupEnable()        - enable pullups for the GPIO
 ** - ApolloGpio_GpioSelectPullup()        - set the pullup resistor for the GPIO supporting this feature
 ** - ApolloGpio_GpioInputEnable()         - enable input for the GPIO
 ** - ApolloGpio_GpioStrengthEnable()      - set output strength for the GPIO supporting this feature
 ** - ApolloGpio_GpioOutputConfiguration() - set output type
 ** - ApolloGpio_GpioOutputEnable()        - enable output
 ** - ApolloGpio_GpioSelectFunction()      - select function at this GPIO, 3 is GPIO
 ** - ApolloGpio_IrqIsEnabled()            - check IRQ for the GPIO is enabled
 ** - ApolloGpio_IrqIsPending()            - check if a GPIO is pending for this GPIO
 ** - ApolloGpio_IrqExecute()              - execute a callback if IRQ is pending without use of NVIC
 ** - ApolloGpio_RegisterIrq()             - register IRQ and enable NVIC
 ** - ApolloGpio_UnRegisterIrq()           - unregister IRQ and disable NVIC if no other GPIO is registered for interrupt usage
 ** - ApolloGpio_GpioSetHighSwitch()       - enable / disable high current switch at the GPIO supporting this feature
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page apollogpio_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "apollogpio.h"   
 ** @endcode
 **
 ******************************************************************************/
     
/**
******************************************************************************    
** \page apollogpio_module_setup Required settings in RTE_Device.h
** \brief Following defines are required in RTE_Device.h
** @code   
**
** #define APOLLOGPIO_ENABLED  1  //enables usage of GPIO module
**
** #define APOLLOGPIO_USE_IRQS 1  //enables usage of interrupt handling in the driver
**
** @endcode
**
******************************************************************************/
 
     
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/


#include "base_types.h"
#include "mcu.h"

#if defined(APOLLOGPIO_USE_MBED)
#include "board.h" //needed for PinName enumeration
#endif

#if (APOLLOGPIO_ENABLED == 1)
     
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#if !defined(MASK_BITLEN)
    #define MASK_BITLEN
    #define MASK_BITLEN1          0x1
    #define MASK_BITLEN2          0x3
    #define MASK_BITLEN3          0x7
    #define MASK_BITLEN4          0xF
    #define MASK_BITLEN5         0x1F
    #define MASK_BITLEN6         0x3F
    #define MASK_BITLEN7         0x7F
    #define MASK_BITLEN8         0xFF
    #define MASK_BITLEN9        0x1FF
    #define MASK_BITLEN10       0x3FF
    #define MASK_BITLEN11       0x7FF
    #define MASK_BITLEN12       0xFFF
    #define MASK_BITLEN13      0x1FFF
    #define MASK_BITLEN14      0x3FFF
    #define MASK_BITLEN15      0x7FFF
    #define MASK_BITLEN16      0xFFFF
    #define MASK_BITLEN17     0x1FFFF
    #define MASK_BITLEN18     0x3FFFF
    #define MASK_BITLEN19     0x7FFFF
    #define MASK_BITLEN20     0xFFFFF
    #define MASK_BITLEN21    0x1FFFFF
    #define MASK_BITLEN22    0x3FFFFF
    #define MASK_BITLEN23    0x7FFFFF
    #define MASK_BITLEN24    0xFFFFFF
    #define MASK_BITLEN25   0x1FFFFFF
    #define MASK_BITLEN26   0x3FFFFFF
    #define MASK_BITLEN27   0x7FFFFFF
    #define MASK_BITLEN28   0xFFFFFFF
    #define MASK_BITLEN29  0x1FFFFFFF
    #define MASK_BITLEN30  0x3FFFFFFF
    #define MASK_BITLEN31  0x7FFFFFFF
    #define MASK_BITLEN32  0xFFFFFFFF
#endif

//registersize = 32bit = 4 byte, Configuration bitlen = 4; configurations per register = 32/4 = 8, register addresses are in 4 byte steps
//
//                                                                          config per register = 8
//                                                                           |   reg-size in bytes = 4
//                                                                           |    |
#define APOLLOGPIO_CFG(n) *(volatile uint32_t*)(((uint32_t)&GPIO->CFGA) + (n/8) * 4)

#define APOLLOGPIO_CFG_GET(n,mask) (APOLLOGPIO_CFG(n) & (mask << (((n) % 8)*4))) >> (((n) % 8)*4))
#define APOLLOGPIO_CFG_SET(n,val) APOLLOGPIO_CFG(n) |= val << (((n) % 8)*4)
#define APOLLOGPIO_CFG_CLR(n,val) APOLLOGPIO_CFG(n) &= ~(val << (((n) % 8)*4))
#define APOLLOGPIO_CFG_ZERO(n) APOLLOGPIO_CFG(n) &= ~(0xF << (((n) % 8) * 4))
#define APOLLOGPIO_CFG_WRITE(n,mask,val) APOLLOGPIO_CFG_CLR(n,mask); APOLLOGPIO_CFG_SET(n,val)  

#define GPIO_CFG_GPIOINTD_Pos           (1 << 3)
#define GPIO_CFG_GPIOINTD_Msk           (MASK_BITLEN1 << GPIO_CFG_GPIOINTD_Pos)
#define GPIO_CFG_GPIOOUTCFG_Pos         (1 << 1)
#define GPIO_CFG_GPIOOUTCFG_Msk         (MASK_BITLEN2 << GPIO_CFG_GPIOOUTCFG_Pos)
#define GPIO_CFG_GPIOINCFG_Pos          (1 << 0)
#define GPIO_CFG_GPIOINCFG_Msk          (MASK_BITLEN1 << GPIO_CFG_GPIOINCFG_Pos)




//registersize = 32bit = 4 byte, Padregconfig bitlen = 8; padregconfig per register = 32/8 = 4, register addresses are in 4 byte steps
//
//                                                                                padregconfig per register = 4
//                                                                                 |   reg-size in bytes = 4
//                                                                                 |    |
#define APOLLOGPIO_PADREG(n) *(volatile uint32_t*)(((uint32_t)&GPIO->PADREGA) + (n/4) * 4)     

#define APOLLOGPIO_PADREG_GET(n,mask) (APOLLOGPIO_PADREG(n) & (mask << (((n) % 4)*8))) >> (((n) % 4)*8))
#define APOLLOGPIO_PADREG_SET(n,val)  APOLLOGPIO_PADREG(n) |= val << (((n) % 4)*8)
#define APOLLOGPIO_PADREG_CLR(n,val)  APOLLOGPIO_PADREG(n) &= ~(val << (((n) % 4)*8))
#define APOLLOGPIO_PADREG_ZERO(n) APOLLOGPIO_PADREG(n) &= ~(0xFF << (((n) % 4)*8))
#define APOLLOGPIO_PADREG_WRITE(n,mask,val) APOLLOGPIO_PADREG_CLR(n,mask); APOLLOGPIO_PADREG_SET(n,val) 

#define GPIO_PADREG_PADPWRUP_Pos    (1 << 7)      
#define GPIO_PADREG_PADPWRUP_Msk    (MASK_BITLEN1 << GPIO_PADREG_PADPWRUP_Pos) 
#define GPIO_PADREG_PADPWRDN_Pos    (1 << 6)     
#define GPIO_PADREG_PADPWRDN_Msk    (MASK_BITLEN1 << GPIO_PADREG_PADPWRDN_Pos)  
#define GPIO_PADREG_PADRSEL_Pos     (1 << 6)     
#define GPIO_PADREG_PADRSEL_Msk     (MASK_BITLEN2 << GPIO_PADREG_PADRSEL_Pos)  
#define GPIO_PADREG_PADFNCSEL_Pos   (1 << 3)      
#define GPIO_PADREG_PADFNCSEL_Msk   (MASK_BITLEN3 << GPIO_PADREG_PADFNCSEL_Pos)      
#define GPIO_PADREG_PADSTRNG_Pos    (1 << 2)      
#define GPIO_PADREG_PADSTRNG_Msk    (MASK_BITLEN1 << GPIO_PADREG_PADSTRNG_Pos)          
#define GPIO_PADREG_PADINPEN_Pos    (1 << 1)      
#define GPIO_PADREG_PADINPEN_Msk    (MASK_BITLEN1 << GPIO_PADREG_PADINPEN_Pos)      
#define GPIO_PADREG_PADPULL_Pos     (1 << 0)        
#define GPIO_PADREG_PADPULL_Msk     (MASK_BITLEN1 << GPIO_PADREG_PADPULL_Pos)

#define	PIN_GPIO0	0
#define	PIN_GPIO1	1
#define	PIN_GPIO2	2
#define	PIN_GPIO3	3
#define	PIN_GPIO4	4
#define	PIN_GPIO5	5
#define	PIN_GPIO6	6
#define	PIN_GPIO7	7
#define	PIN_GPIO8	8
#define	PIN_GPIO9	9
#define	PIN_GPIO10	10
#define	PIN_GPIO11	11
#define	PIN_GPIO12	12
#define	PIN_GPIO13	13
#define	PIN_GPIO14	14
#define	PIN_GPIO15	15
#define	PIN_GPIO16	16
#define	PIN_GPIO17	17
#define	PIN_GPIO18	18
#define	PIN_GPIO19	19
#define	PIN_GPIO20	20
#define	PIN_GPIO21	21
#define	PIN_GPIO22	22
#define	PIN_GPIO23	23
#define	PIN_GPIO24	24
#define	PIN_GPIO25	25
#define	PIN_GPIO26	26
#define	PIN_GPIO27	27
#define	PIN_GPIO28	28
#define	PIN_GPIO29	29
#define	PIN_GPIO30	30
#define	PIN_GPIO31	31
#define	PIN_GPIO32	32
#define	PIN_GPIO33	33
#define	PIN_GPIO34	34
#define	PIN_GPIO35	35
#define	PIN_GPIO36	36
#define	PIN_GPIO37	37
#define	PIN_GPIO38	38
#define	PIN_GPIO39	39
#define	PIN_GPIO40	40
#define	PIN_GPIO41	41
#define	PIN_GPIO42	42
#define	PIN_GPIO43	43
#define	PIN_GPIO44	44
#define	PIN_GPIO45	45
#define	PIN_GPIO46	46
#define	PIN_GPIO47	47
#define	PIN_GPIO48	48
#define	PIN_GPIO49	49


#define SET_GPIO(n)       (n < 32) ? (GPIO->WTSA = (1 << n) & 0xFFFFFFFF) : (GPIO->WTSB = (1 << (n - 32)) & 0xFFFFFFFF)
#define CLEAR_GPIO(n)     (n < 32) ? (GPIO->WTCA = (1 << n) & 0xFFFFFFFF) : (GPIO->WTCB = (1 << (n - 32)) & 0xFFFFFFFF)

#define SET_GPIOS(mask)   GPIO->WTSB = (uint32_t)((mask) >> 32); GPIO->WTSA = (uint32_t)(mask) ///< set multible GPIOs in a 64-bit mask
#define CLEAR_GPIOS(mask) GPIO->WTCB = (uint32_t)((mask) >> 32); GPIO->WTCA = (uint32_t)(mask) ///< clear multible GPIOs in a 64-bit mask
    
#if APOLLOGPIO_USE_ARDUINO == 1
    #define HIGH 0x1
    #define LOW  0x0
    #define INPUT 0x0
    #define OUTPUT 0x1
    #define INPUT_PULLUP 0x2
    //#define CHANGE 1 not supported
    #define FALLING 2
    #define RISING  3
    #define digitalWrite(pin, val) ApolloGpio_GpioSet(pin,val)
    #define digitalRead(pin) ApolloGpio_GpioGet(pin)
    #define detachInterrupt(interruptNum) ApolloGpio_UnRegisterIrq(interruptNum);
#endif
    
/*****************************************************************************/
/* Global type definitions ('typedef')                                        */
/*****************************************************************************/
typedef enum en_apollogpio_mode
{
    GpioOutputDisabled = 0,
    GpioPushPull = 1,
    GpioOpenDrain = 2,
    GpioTriState = 3
} en_apollogpio_mode_t;

typedef enum en_apollogpio_pullup
{
    PullUp1K5 = 0,
    PullUp6K = 1,
    PullUp12K = 2,
    PullUp24K = 3,
} en_apollogpio_pullup_t;

typedef void (*pfn_apollogpio_callback_t)(uint8_t pin);

typedef enum en_apollogpio_edgedetect
{
    GpioFallingEdge = 1,
    GpioRisingEdge = 0,
} en_apollogpio_edgedetect_t;

#if !defined(_APOLLOGPIO_GPIO_PIN_T_)
#define _APOLLOGPIO_GPIO_PIN_T_
    #if defined(APOLLOGPIO_USE_MBED)
        typedef PinName apollogpio_gpio_pin_t;
    #else
        typedef uint32_t apollogpio_gpio_pin_t;
    #endif
#endif
        
typedef struct stc_apollogpio_register_mask_pair
{
    uint32_t u32Pin;
    volatile uint32_t* pRegister;
    uint32_t u32Mask;
} stc_apollogpio_register_mask_pair_t;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/



/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void ApolloGpio_GpioSet(apollogpio_gpio_pin_t pin, boolean_t bOnOff);
void ApolloGpio_GpioPullupEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable);
void ApolloGpio_GpioInputEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable);
void ApolloGpio_GpioStrengthEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable);
void ApolloGpio_GpioOutputConfiguration(apollogpio_gpio_pin_t pin, en_apollogpio_mode_t enMode);
void ApolloGpio_GpioOutputEnable(apollogpio_gpio_pin_t pin, boolean_t bEnable);
void ApolloGpio_GpioSelectFunction(apollogpio_gpio_pin_t pin, uint8_t u8Function);
boolean_t ApolloGpio_IrqIsEnabled(apollogpio_gpio_pin_t pin);
boolean_t ApolloGpio_IrqIsPending(apollogpio_gpio_pin_t pin);
boolean_t ApolloGpio_IrqExecute(apollogpio_gpio_pin_t pin);
void ApolloGpio_RegisterIrq(apollogpio_gpio_pin_t pin, en_apollogpio_edgedetect_t enMode, uint32_t u32Priority, pfn_apollogpio_callback_t pfnCallback);
void ApolloGpio_UnRegisterIrq(apollogpio_gpio_pin_t pin);
boolean_t ApolloGpio_GpioGet(apollogpio_gpio_pin_t pin);
void ApolloGpio_GpioSetHighSwitch(apollogpio_gpio_pin_t pin, boolean_t bOnOff);
void ApolloGpio_GpioSelectPullup(apollogpio_gpio_pin_t pin, en_apollogpio_pullup_t enPullUp);
#if APOLLOGPIO_USE_ARDUINO == 1
void pinMode(uint8_t pin, uint8_t mode);
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
#endif

#endif /* (APOLLOGPIO_ENABLED == 1) */

#ifdef __cplusplus
}
#endif

//@} // ApolloGpioGroup

#endif /*__APOLLOGPIO_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


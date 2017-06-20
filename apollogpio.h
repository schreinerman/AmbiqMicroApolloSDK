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
 **   - 2017-04-04  V1.0  MSc  First Version
 **   - 2017-04-13  V1.1  MSc  IRQs added, Smaller macro bug fixes
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
 ** \defgroup ApolloGpioGroup Apollo GPIO
 **
 ** Provided functions of ApolloGpio:
 ** 
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
    
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcu.h"
#include "base_types.h"

#if (APOLLOGPIO_ENABLED == 1)
     
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define APOLLOGPIO_PADREG(n) *(volatile uint32_t*)(0x40010000 + (n/4) * 4)
     
#define APOLLOGPIO_CFG(n) *(volatile uint32_t*)(0x40010040 + (n/8) * 4)
#define APOLLOGPIO_CFG_SET(n,val) APOLLOGPIO_CFG(n) |= val << (((n) % 8)*4)
#define APOLLOGPIO_CFG_CLR(n,val) APOLLOGPIO_CFG(n) &= ~(val << (((n) % 8)*4))
#define APOLLOGPIO_CFG_ZERO(n) APOLLOGPIO_CFG(n) &= ~(0xF << (((n) % 8) * 4))
#define APOLLOGPIO_CFG_GET(n,val) APOLLOGPIO_CFG(n) val >> (((n) % 8)*4)
#define APOLLOGPIO_CFG_WRITE(n,mask,val) APOLLOGPIO_CFG_CLR(n,mask); APOLLOGPIO_CFG_SET(n,val)  
     
#define APOLLOGPIO_PADREG_SET(n,val) APOLLOGPIO_PADREG(n) |= val << (((n) % 4)*8)
#define APOLLOGPIO_PADREG_CLR(n,val) APOLLOGPIO_PADREG(n) &= ~(val << (((n) % 4)*8))
#define APOLLOGPIO_PADREG_ZERO(n) APOLLOGPIO_PADREG(n) &= ~(0xFF << (((n) % 4)*8))
#define APOLLOGPIO_PADREG_WRITE(n,mask,val) APOLLOGPIO_PADREG_CLR(n,mask); APOLLOGPIO_PADREG_SET(n,val) 
     
#define SET_OUTPUT_GPIO(n) APOLLOGPIO_PADREG_SET(n,(0x3 << 3)); APOLLOGPIO_CFG_SET(n,(0x01 << 1));

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

#define SET_GPIO(n)       GPIO->WTSA = (1 << n) & 0xFFFFFFFF; GPIO->WTSB = (1 << (n - 32)) & 0xFFFFFFFF
#define CLEAR_GPIO(n)     GPIO->WTCA = (1 << n) & 0xFFFFFFFF; GPIO->WTCB = (1 << (n - 32)) & 0xFFFFFFFF  
    
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

typedef void (*pfn_apollogpio_callback_t)(uint8_t pin);

typedef enum en_apollogpio_edgedetect
{
    GpioFallingEdge = 1,
    GpioRisingEdge = 0,
} en_apollogpio_edgedetect_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/



/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void ApolloGpio_GpioSet(uint32_t pin, boolean_t bOnOff);
void ApolloGpio_GpioPullupEnable(uint32_t pin, boolean_t bEnable);
void ApolloGpio_GpioInputEnable(uint32_t pin, boolean_t bEnable);
void ApolloGpio_GpioStrengthEnable(uint32_t pin, boolean_t bEnable);
void ApolloGpio_GpioOutputConfiguration(uint32_t pin, en_apollogpio_mode_t enMode);
void ApolloGpio_GpioOutputEnable(uint32_t pin, boolean_t bEnable);
void ApolloGpio_GpioSelectFunction(uint32_t pin, uint8_t u8Function);
boolean_t ApolloGpio_IrqIsEnabled(uint32_t pin);
boolean_t ApolloGpio_IrqIsPending(uint32_t pin);
boolean_t ApolloGpio_IrqExecute(uint32_t pin);
void ApolloGpio_RegisterIrq(uint32_t pin, en_apollogpio_edgedetect_t enMode, pfn_apollogpio_callback_t pfnCallback);

#endif /* (APOLLOGPIO_ENABLED == 1) */

#ifdef __cplusplus
}
#endif

//@} // ApolloGpioGroup

#endif /*__APOLLOGPIO_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


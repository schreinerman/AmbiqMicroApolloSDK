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
/** \file ApolloAdc.h
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
#ifndef __APOLLOADC_H__
#define __APOLLOADC_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup ApolloAdcGroup Low-Level-Driver for Apollo 1/2 ADC
 **
 ** Provided functions of ApolloAdc:
 ** 
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page apolloadc_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "apolloadc.h"   
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ******************************************************************************    
 ** \page apolloadc_module_rte_device Required defines in RTE_Device.h to set and enable features
 ** \brief Following includes are required
 ** @code   
 ** #define APOLLOADC_ENABLED 1 //set following define to enable the ADC module
 ** @endcode
 **
 ******************************************************************************/  
     
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcu.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/



/*****************************************************************************/
/* Global type definitions ('typedef')                                        */
/*****************************************************************************/


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/



/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
float32_t ApolloAdc_SimpleRead(uint32_t pin, boolean_t bLowPower);
float32_t ApolloAdc_CheckBattery(boolean_t bLowPower);

#ifdef __cplusplus
}
#endif

//@} // ApolloAdcGroup

#endif /*__APOLLOADC_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


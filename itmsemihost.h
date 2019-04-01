/*******************************************************************************
* Copyright (C) 2013-2016, Fujitsu Electronics Europe GmbH or a                *
* subsidiary of Fujitsu Electronics Europe GmbH.  All rights reserved.         *
*                                                                              *
* This software, including source code, documentation and related              *
* materials ("Software"), is owned by Fujitsu Electronics Europe GmbH or       *
* one of its subsidiaries ("Fujitsu").
*                                                                              *
* If no EULA applies, Fujitsu hereby grants you a personal, non-exclusive,     *
* non-transferable license to copy, modify, and compile the                    *
* Software source code solely for use in connection with Fujitsu's             *
* integrated circuit products.  Any reproduction, modification, translation,   *
* compilation, or representation of this Software except as specified          *
* above is prohibited without the express written permission of Fujitsu.       *
*                                                                              *
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                         *
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                         *
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                 *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                              *
* PARTICULAR PURPOSE. Fujitsu reserves the right to make                       *
* changes to the Software without notice. Fujitsu does not assume any          *
* liability arising out of the application or use of the Software or any       *
* product or circuit described in the Software. Fujitsu does not               *
* authorize its products for use in any products where a malfunction or        *
* failure of the Fujitsu product may reasonably be expected to result in       *
* significant property damage, injury or death ("High Risk Product"). By       *
* including Fujitsu's product in a High Risk Product, the manufacturer         *
* of such system or application assumes all risk of such use and in doing      *
* so agrees to indemnify Fujitsu against all liability.                        *
*******************************************************************************/
/******************************************************************************/
/** \file itmsemihost.h
 **
 ** A detailed description is available at 
 ** @link ItmSemihostGroup ITM Semihosting description @endlink
 **
 ** History:
 **   - 2016-15-09  V1.0  Manuel Schreiner   First Version
 **   - 2017-04-04  V1.1  Manuel Schreiner   ITMSEMIHOST_ENABLED added (to be defined in RTE_Device.h)
 **   - 2018-07-06  V1.2  Manuel Schreiner   Updated documentation, 
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **   - 2019-03-25  V1.3  Manuel Schreiner   Added __ITMSEMIHOST_VERSION__ and __ITMSEMIHOST_DATE__ defines
 **
 *****************************************************************************/

#ifndef __ITMSEMIHOST_H__
#define __ITMSEMIHOST_H__
#define __ITMSEMIHOST_VERSION__  13
#define __ITMSEMIHOST_DATE__     "2019-03-25"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup ItmSemihostGroup Low-Level-Driver for Apollo 1/2 ITM Semihosting
 **
 ** Provided functions of ITM Semihosting:
 ** 
 ** - ItmSemihost_Init()
 ** - ItmSemihost_Deinit()
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page itmsemihost_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "itmsemihost.h"   
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ****************************************************************************** 
 ** \page itmsemihost_module_init Example: Initialization
 ** \brief Following initialization is required 
 **
 ** @code
 ** ItmSemihost_Init();   
 ** @endcode
 **
 ******************************************************************************/
    
    
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "base_types.h"
#include "mcu.h"
#include "string.h"
#include "stdio.h"
   
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void ItmSemihost_Init(void);
void ItmSemihost_Deinit(void);

#ifdef __cplusplus
}
#endif

//@} // ItmSemihostGroup

#endif /* __ITMSEMIHOST_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


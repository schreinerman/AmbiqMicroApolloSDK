/*******************************************************************************
* Copyright (C) 2016, Fujitsu Electronics Europe GmbH or a                     *
* subsidiary of Fujitsu Electronics Europe GmbH.                               *
*                                                                              *
* This software, including source code, documentation and related materials    *
* ("Software") is developed by Fujitsu Electronics Europe GmbH ("Fujitsu")     *
* unless identified differently hereinafter for Open Source Software.          *
* All rights reserved.                                                         *
*                                                                              *
* This software is provided free of charge and not for sale, providing test    *
* and sandbox applications. Fujitsu reserves the right to make changes to      *
* the Software without notice. Before use please check with Fujitsu            *
* for the most recent software.                                                *
*                                                                              *
* If no specific Open Source License Agreement applies (see hereinafter),      *
* Fujitsu hereby grants you a personal, non-exclusive,                         *
* non-transferable license to copy, modify and compile the                     *
* Software source code solely for use in connection with products              *
* supplied by Fujitsu. Any reproduction, modification, translation,            *
* compilation, or representation of this Software requires written             *
* permission of Fujitsu.                                                       *
*                                                                              *
* NO WARRANTY: This software is provided “as-is” with no warranty of any kind  *
* (German “Unter Ausschluss jeglicher Gewähleistung”), express or implied,     *
* including but not limited to non-infringement of third party rights,         *
* merchantability and fitness for use.                                         *
*                                                                              *
* In the event the software deliverable includes the use of                    *
* open source components, the provisions of the respective governing           *
* open source license agreement shall apply with respect to such software      *
* deliverable. Open source components are identified in the read-me files      *
* of each section / subsection.                                                *
*                                                                              *
* German law applies with the exclusion of the rules of conflict of law.       *
*                                                                              *
*                                      ***                                     *
* September 2016                                                               *
* FUJITSU ELECTRONICS Europe GmbH                                              *
*                                                                              *
*******************************************************************************/
/******************************************************************************/
/** \file apollouart.h
 **
 ** A detailed description is available at 
 ** @link ApolloUartGroup UART routines for Apollo @endlink
 **
 ** History:
 **   - 2016-11-26  V1.0  MSc  First Version
 **   - 2016-04-13  V1.1  MSc  Added IRQs and callbacks
 **   - 2016-05-16  V1.2  MSc  Added Apollo 2 support
 **
 *****************************************************************************/

#ifndef __APOLLOUART_H__
#define __APOLLOUART_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup MetrahitUltraGroup Driving a 8x8 matrix with MAX7219 chipset via SPI
 **
 ** Provided functions of Driving a 8x8 matrix with MAX7219 chipset via SPI:
 ** 
 ** - ApolloUart_Init()
 ** - ApolloUart_Deinit()
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page metrahitultra_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "apollouart.h"   
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ****************************************************************************** 
 ** \page apollouart_module_init Example: Initialization
 ** \brief Following initialization is required 
 **
 ** @code
 ** ApolloUart_Init();   
 ** @endcode
 **
 ******************************************************************************/
    
    
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "base_types.h"
#include "mcu.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef int16_t (*pfn_apollouart_txnext_t) (void);
typedef void (*pfn_apollouart_rx_t)(int16_t);

#if defined(APOLLO2_H)
typedef UART0_Type UART_Type;
#endif

/// ApolloUart module internal data
typedef struct stc_apollouart_intern_data
{
    pfn_apollouart_txnext_t cbTxNext;
    pfn_apollouart_rx_t cbRx;   
} stc_apollouart_intern_data_t;

/// ApolloUart module internal data, storing internal information for each UART instance.
typedef struct stc_apollouart_instance_data
{
    UART_Type*  pstcInstance;  ///< pointer to registers of an instance
    stc_apollouart_intern_data_t stcInternData; ///< module internal data of instance
} stc_apollouart_instance_data_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void ApolloUart_Init(UART_Type* pstcUart,uint32_t u32Baudrate);
void ApolloUart_PutChar(UART_Type* pstcUart, uint8_t u8Char);
void ApolloUart_PutString(UART_Type* pstcUart, char_t *pu8Buffer);
uint8_t ApolloUart_GetChar(UART_Type* pstcUart);
boolean_t ApolloUart_HasChar(UART_Type* pstcUart);
void ApolloUart_RegisterCallbacks(UART_Type* pstcUart,pfn_apollouart_txnext_t cbTxNext, pfn_apollouart_rx_t cbRx);
void ApolloUart_NewTxData(UART_Type* pstcUart);
boolean_t ApolloUart_NewTxDataEnabled(UART_Type* pstcUart);
void ApolloUart_Deinit(void);

#ifdef __cplusplus
}
#endif

//@} // ApolloUartGroup

#endif /* __APOLLOUART_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


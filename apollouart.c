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
* NO WARRANTY: This software is provided â€œas-isâ€� with no warranty of any kind  *
* (German â€œUnter Ausschluss jeglicher GewÃ¤hleistungâ€�), express or implied,     *
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
/** \file ApolloUart.c
 **
 ** A detailed description is available at 
 ** @link ApolloUartGroup UART routines for Apollo @endlink
 **
 ** History:
 **   - 2016-11-26  V1.0  MSc  First Version
 **   - 2016-04-13  V1.1  MSc  Added IRQs and callbacks
 **   - 2016-05-16  V1.2  MSc  Added Apollo 2 support
 *****************************************************************************/
#define __APOLLOUART_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "apollouart.h"
#include "stdio.h"
#include "string.h"
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define INSTANCE_COUNT (uint32_t)(sizeof(m_astcInstanceDataLut) / sizeof(m_astcInstanceDataLut[0]))

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/


/// Look-up table for all enabled UART instances and their internal data
static stc_apollouart_instance_data_t m_astcInstanceDataLut[] =
{
#if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
    { (UART),    // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if defined(UART0) && (APOLLOUART0_ENABLED == 1)
    { (UART0),   // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
#if defined(UART1) && (APOLLOUART1_ENABLED == 1)
    { (UART1),   // pstcInstance
    (0)          // stcInternData (not initialized yet)
    },
#endif
};    
/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static stc_apollouart_intern_data_t* GetInternDataPtr(UART_Type* pstcIf);
static void ConfigureBaudrate(UART_Type* pstcUart,uint32_t u32Baudrate, uint32_t u32UartClkFreq);
static void UARTn_IRQHandler(UART_Type* pstcUart);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
******************************************************************************
** \brief Return the internal data for a certain instance.
**
** \param pstcUsb Pointer to instance
**
** \return Pointer to internal data or NULL if instance is not enabled (or not known)
**
******************************************************************************/
static stc_apollouart_intern_data_t* GetInternDataPtr(UART_Type* pstcIf) 
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
 ** \brief  Set baudrate
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  u32Baudrate      Baudrate
 **
 ** \param  u32UartClkFreq   UART clock
 **
 *****************************************************************************/
static void ConfigureBaudrate(UART_Type* pstcUart,uint32_t u32Baudrate, uint32_t u32UartClkFreq)
{
    uint64_t u64FractionDivisorLong;
    uint64_t u64IntermediateLong;
    uint32_t u32IntegerDivisor;
    uint32_t u32FractionDivisor;
    uint32_t u32BaudClk;
    
    //
    // Calculate register values.
    //
    u32BaudClk = 16 * u32Baudrate;
    u32IntegerDivisor = (uint32_t)(u32UartClkFreq / u32BaudClk);
    u64IntermediateLong = (u32UartClkFreq * 64) / u32BaudClk;
    u64FractionDivisorLong = u64IntermediateLong - (u32IntegerDivisor * 64);
    u32FractionDivisor = (uint32_t)u64FractionDivisorLong;
    
    //
    // Integer divisor MUST be greater than or equal to 1.
    //
    if(u32IntegerDivisor == 0)
    {
        //
        // Spin in a while because the selected baudrate is not possible.
        //
        while(1);
    }
    //
    // Write the UART regs.
    //
    pstcUart->IBRD = u32IntegerDivisor;
    pstcUart->IBRD = u32IntegerDivisor;
    pstcUart->FBRD = u32FractionDivisor;
}

/**
 ******************************************************************************
 ** \brief  sends a single character  (no timeout !)
 **
 ** \param  pstcUart  UART pointer
 **
 ** \param  u8Char    Data to send
 **
 *****************************************************************************/
void ApolloUart_PutChar(UART_Type* pstcUart, uint8_t u8Char)
{
    while(pstcUart->FR_b.TXFF) __NOP();
    pstcUart->DR = u8Char;
}

/**
 ******************************************************************************
 ** \brief  sends a complete string (0-terminated) 
 **
 ** \param  pstcUart  UART pointer
 **
 ** \param  Pointer to (constant) file of bytes in mem
 **
 *****************************************************************************/
void ApolloUart_PutString(UART_Type* pstcUart, char_t *pu8Buffer)
{
  while (*pu8Buffer != '\0')
  { 
    ApolloUart_PutChar(pstcUart,*pu8Buffer++);        // send every char of string
  }
}

/**
 ******************************************************************************
 ** \brief  returns incoming character, if received on UART0
 **
 ** \param  pstcUart  UART pointer
 **
 ** \return Character or 0xFF (Error) or 0 (Nothing)
 *****************************************************************************/
uint8_t ApolloUart_GetChar(UART_Type* pstcUart)   
{
  uint8_t u8Char;
  
  //
  // Wait for data, i.e. RX FIFO NOT EMPTY.
  //
  while(pstcUart->FR_b.RXFE) __NOP();

  //
  // Save the char.
  //
  u8Char = pstcUart->DR;
  return u8Char;
}

boolean_t ApolloUart_HasChar(UART_Type* pstcUart)
{
    return (pstcUart->FR_b.RXFE == 0);
}

/**
 ******************************************************************************
 ** \brief  Init UART...
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  u32Baudrate      Baudrate
 **
 *****************************************************************************/
void ApolloUart_Init(UART_Type* pstcUart,uint32_t u32Baudrate)
{
    //
    // Enable UART clock in CLKGEN
    //
#if defined(APOLLO_H) || defined(APOLLO1_H)
    CLKGEN->UARTEN_b.UARTEN = 1;        //enable UART clocking
#endif
    
    //
    // Enable clock / select clock...
    //
    pstcUart->CR = 0;
    pstcUart->CR_b.CLKEN = 1;                 //enable clock
    pstcUart->CR_b.CLKSEL = 1;                //use 24MHz clock
    
    //
    // Disable UART before config...
    //
    pstcUart->CR_b.UARTEN = 0;                //enable UART
    pstcUart->CR_b.RXE = 0;                   //enable receiver
    pstcUart->CR_b.TXE = 0;                   //enable transmitter
    
    
    //
    // Starting UART config...
    //
    
    // initialize baudrate before all other settings, otherwise UART will not be initialized
    SystemCoreClockUpdate();
    ConfigureBaudrate(pstcUart,u32Baudrate,SystemCoreClock);
    
    // initialize line coding...
    pstcUart->LCRH = 0;
    pstcUart->LCRH_b.WLEN = 3;                //3 = 8 data bits (2..0 = 7..5 data bits)
    pstcUart->LCRH_b.STP2 = 0;                //1 stop bit
    pstcUart->LCRH_b.PEN = 0;                 //no parity
    
    
    //
    // Enable UART after config...
    //
    pstcUart->CR_b.UARTEN = 1;                //enable UART
    pstcUart->CR_b.RXE = 1;                   //enable receiver
    pstcUart->CR_b.TXE = 1;                   //enable transmitter
    

}

/**
 ******************************************************************************
 ** \brief  Register callbacks
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  cbTxNext         Callback retreiving next data
 **
 ** \param  cbTxNext         Callback after data was received
 **
 *****************************************************************************/
void ApolloUart_RegisterCallbacks(UART_Type* pstcUart,pfn_apollouart_txnext_t cbTxNext, pfn_apollouart_rx_t cbRx)
{
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) return;
    pstcHandle->cbTxNext = cbTxNext;
    pstcHandle->cbRx = cbRx;
    if (cbTxNext != NULL)
    {
        pstcUart->IER_b.TXIM = 1;
    } else
    {
        pstcUart->IER_b.TXIM = 0;
    }
    if (cbRx != NULL)
    {
        pstcUart->IER_b.RXIM = 1;
    } else
    {
        pstcUart->IER_b.RXIM = 0;
    }
    #if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
    if (pstcUart == UART)
    {
        NVIC_ClearPendingIRQ(UART_IRQn);    //clear pending flag for UART
        NVIC_EnableIRQ(UART_IRQn);          //enable IRQ
        NVIC_SetPriority(UART_IRQn,1);      //set priority of UART IRQ, smaller value means higher priority
    }
    #endif
    #if defined(UART0) && (APOLLOUART0_ENABLED == 1)
    if (pstcUart == UART0)
    {
        NVIC_ClearPendingIRQ(UART0_IRQn);    //clear pending flag for UART
        NVIC_EnableIRQ(UART0_IRQn);          //enable IRQ
        NVIC_SetPriority(UART0_IRQn,1);      //set priority of UART IRQ, smaller value means higher priority
    }
    #endif
    #if defined(UART1) && (APOLLOUART1_ENABLED == 1)
    if (pstcUart == UART1)
    {
        NVIC_ClearPendingIRQ(UART1_IRQn);    //clear pending flag for UART
        NVIC_EnableIRQ(UART1_IRQn);          //enable IRQ
        NVIC_SetPriority(UART1_IRQn,1);      //set priority of UART IRQ, smaller value means higher priority
    }
    #endif
}

/**
 ******************************************************************************
 ** \brief  Initiate sending data via IRQ
 **
 ** \param  pstcUart         UART pointer
 **
 *****************************************************************************/
void ApolloUart_NewTxData(UART_Type* pstcUart)
{
    int16_t i16ret;
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) return;
    if (pstcHandle->cbTxNext != NULL)
    {
        pstcUart->IEC_b.TXIC = 1;
        pstcUart->IER_b.TXIM = 1;
        i16ret = pstcHandle->cbTxNext();
        if (i16ret == -1)
        {
            pstcUart->IER_b.TXIM = 0;
        } else
        {
            while(pstcUart->FR_b.TXFF) __NOP();
            pstcUart->DR = (uint8_t)i16ret;
        }
    } else
    {
        pstcUart->IER_b.TXIM = 0;
    }
}

/**
 ******************************************************************************
 ** \brief  Check sending data via IRQs is enabled
 **
 *****************************************************************************/
boolean_t ApolloUart_NewTxDataEnabled(UART_Type* pstcUart)
{
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) return FALSE;
    if (pstcHandle->cbTxNext != NULL)
    {
        if (pstcUart->IER_b.TXIM == 1)
        {
            return TRUE;
        }
    } 
    return FALSE;
}

/**
 ******************************************************************************
 ** \brief  IRQ handler
 **
 *****************************************************************************/
#if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
void UART_IRQHandler(void)
{
    UARTn_IRQHandler(UART);
}
#endif

#if defined(UART0) && (APOLLOUART0_ENABLED == 1)
void UART0_IRQHandler(void)
{
    UARTn_IRQHandler(UART0);
}
#endif

#if defined(UART1) && (APOLLOUART1_ENABLED == 1)
void UART1_IRQHandler(void)
{
    UARTn_IRQHandler(UART1);
}
#endif

static void UARTn_IRQHandler(UART_Type* pstcUart)
{
    int16_t i16ret;
    uint32_t u32Status;
    stc_apollouart_intern_data_t* pstcHandle;
    pstcHandle = GetInternDataPtr(pstcUart);
    u32Status = pstcUart->IES;
    if (pstcUart->IES_b.RXRIS)
    {
        if (pstcHandle->cbRx != NULL)
        {
            pstcHandle->cbRx(pstcUart->DR);
        }
    }
    if (pstcUart->IES_b.TXRIS)
    {
        if (pstcHandle->cbTxNext != NULL)
        {
            i16ret = pstcHandle->cbTxNext();
            if (i16ret == -1)
            {
                pstcUart->IER_b.TXIM = 0;
            } else
            {
                while(pstcUart->FR_b.TXFF) __NOP();
                pstcUart->DR = (uint8_t)i16ret;
            }
        }
    }
    
    pstcUart->IEC = u32Status;
}

/**
 ******************************************************************************
 ** \brief Deinitialization Routine
 **
 ******************************************************************************/
void Uart_Deinit(void)
{

}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

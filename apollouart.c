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
 **   - 2016-06-26  V1.3  MSc  Added CMSIS Driver API
 **   - 2016-07-26  V1.4  MSc  Fixed error if CMSIS Driver API is disabled
 *****************************************************************************/
#define __APOLLOUART_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "apollouart.h"
#include "stdio.h"
#include "string.h"
#include "apolloctimer.h"
     
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
#if defined(USE_CMSIS_DRIVER)
static const ARM_DRIVER_VERSION drv_vers = {0x0101,0x0205};
#endif

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static stc_apollouart_intern_data_t* GetInternDataPtr(UART_Type* pstcIf);
static void ConfigureBaudrate(UART_Type* pstcUart,uint32_t u32Baudrate, uint32_t u32UartClkFreq);
static void UARTn_IRQHandler(UART_Type* pstcUart);


/*****************************************************************************
 *   CCCCCC   M         M     SSSSSS    I   SSSSSS  
 *  C         M M     M M    S          I  S         
 *  C         M  M   M  M    S          I  S      
 *  C         M    M    M     SSSSSS    I   SSSSSS  
 *  C         M         M           S   I         S  
 *  C         M         M           S   I         S  
 *   CCCCCC   M         M     SSSSSS    I   SSSSSS  
 *
 *  >> CMSIS Driver API Start
 *
 *****************************************************************************/
#if defined(USE_CMSIS_DRIVER)
    static int32_t Control(UART_Type* pstcUart, uint32_t control, uint32_t arg);
    static ARM_DRIVER_VERSION GetVersion(void);
    static ARM_USART_STATUS GetStatus(UART_Type* pstcUart);
    static ARM_USART_MODEM_STATUS GetModemStatus(UART_Type* pstcUart);
    static int32_t Initialize(UART_Type* pstcUart, ARM_USART_SignalEvent_t cb_event);
    static int32_t Uninitialize(UART_Type* pstcUart);
    static int32_t PowerControl(UART_Type* pstcUart, ARM_POWER_STATE state);
    static int32_t SetModemControl(UART_Type* pstcUart, ARM_USART_MODEM_CONTROL control);
    
    #if (defined(UART) || defined(UART0)) && (APOLLOUART0_ENABLED == 1)
        static ARM_USART_CAPABILITIES GetCapabilities_UART0(void);
        static int32_t Initialize_UART0(ARM_USART_SignalEvent_t cb_event);
        static int32_t Uninitialize_UART0(void);
        static int32_t PowerControl_UART0(ARM_POWER_STATE state);
        static int32_t Send_UART0(const void *data, uint32_t num);
        static int32_t Receive_UART0(      void *data, uint32_t num);
        static int32_t Transfer_UART0(const void *data_out,void *data_in,uint32_t num); 
        static uint32_t GetTxCount_UART0(void);
        static uint32_t GetRxCount_UART0(void);
        static int32_t Control_UART0(uint32_t control, uint32_t arg);
        static ARM_USART_STATUS GetStatus_UART0(void);
        static int32_t SetModemControl_UART0(ARM_USART_MODEM_CONTROL control);
        static ARM_USART_MODEM_STATUS GetModemStatus_UART0(void);

        ARM_DRIVER_USART Driver_UART0 = {
        GetVersion,
        GetCapabilities_UART0,
        Initialize_UART0,
        Uninitialize_UART0,
        PowerControl_UART0,
        Send_UART0,
        Receive_UART0,
        Transfer_UART0,
        GetTxCount_UART0,
        GetRxCount_UART0,
        Control_UART0,
        GetStatus_UART0,
        SetModemControl_UART0,
        GetModemStatus_UART0
        };
    #endif /* (defined(UART) || defined(UART0)) && (APOLLOUART0_ENABLED == 1) */
        
    #if (defined(UART1)) && (APOLLOUART1_ENABLED == 1)
        static ARM_USART_CAPABILITIES GetCapabilities_UART1(void);
        static int32_t Initialize_UART1(ARM_USART_SignalEvent_t cb_event);
        static int32_t Uninitialize_UART1(void);
        static int32_t PowerControl_UART1(ARM_POWER_STATE state);
        static int32_t Send_UART1(const void *data, uint32_t num);
        static int32_t Receive_UART1(      void *data, uint32_t num);
        static int32_t Transfer_UART1(const void *data_out,void *data_in,uint32_t num); 
        static uint32_t GetTxCount_UART1(void);
        static uint32_t GetRxCount_UART1(void);
        static int32_t Control_UART1(uint32_t control, uint32_t arg);
        static ARM_USART_STATUS GetStatus_UART1(void);
        static int32_t SetModemControl_UART1(ARM_USART_MODEM_CONTROL control);
        static ARM_USART_MODEM_STATUS GetModemStatus_UART1(void);

        ARM_DRIVER_USART Driver_UART1 = {
        GetVersion,
        GetCapabilities_UART1,
        Initialize_UART1,
        Uninitialize_UART1,
        PowerControl_UART1,
        Send_UART1,
        Receive_UART1,
        Transfer_UART1,
        GetTxCount_UART1,
        GetRxCount_UART1,
        Control_UART1,
        GetStatus_UART1,
        SetModemControl_UART1,
        GetModemStatus_UART1
        };
    #endif /* (defined(UART) || defined(UART0)) && (APOLLOUART0_ENABLED == 1) */    
#endif /* defined(USE_CMSIS_DRIVER) */
/*****************************************************************************
 *
 * << CMSIS Driver API End
 *
 *****************************************************************************/
        
        
        
        
        
/// Look-up table for all enabled UART instances and their internal data
static stc_apollouart_instance_data_t m_astcInstanceDataLut[] =
{
#if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
    {   (UART),    // pstcInstance
        { //stc_apollouart_intern_data_t
            NULL, //pfn_apollouart_txnext_t cbTxNext;
            NULL, //pfn_apollouart_rx_t cbRx;   
            FALSE, //bInitialized
            0, //u32RegCR
            (3 << UART_LCRH_WLEN_Pos), //u32RegLCRH, default: 8 databits, no parity, 1 stopbit
            9600, //u32Baudrate, default 9600 baud
        #if defined(USE_CMSIS_DRIVER)
            &Driver_UART0,
            NULL, //ARM_USART_SignalEvent pfnCmsisSignalEvent
            { //ARM_USART_CAPABILITIES u32Capabilities; 
                    1,//uint32_t asynchronous       : 1;      ///< supports UART (Asynchronous) mode 
                    0,//uint32_t synchronous_master : 1;      ///< supports Synchronous Master mode
                    0,//uint32_t synchronous_slave  : 1;      ///< supports Synchronous Slave mode
                    0,//uint32_t single_wire        : 1;      ///< supports UART Single-wire mode
                    0,//uint32_t irda               : 1;      ///< supports UART IrDA mode
                    0,//uint32_t smart_card         : 1;      ///< supports UART Smart Card mode
                    0,//uint32_t smart_card_clock   : 1;      ///< Smart Card Clock generator available
                    1,//uint32_t flow_control_rts   : 1;      ///< RTS Flow Control available
                    1,//uint32_t flow_control_cts   : 1;      ///< CTS Flow Control available
                    1,//uint32_t event_tx_complete  : 1;      ///< Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
                    1,//uint32_t event_rx_timeout   : 1;      ///< Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
                    1,//uint32_t rts                : 1;      ///< RTS Line: 0=not available, 1=available
                    1,//uint32_t cts                : 1;      ///< CTS Line: 0=not available, 1=available
                    0,//uint32_t dtr                : 1;      ///< DTR Line: 0=not available, 1=available
                    0,//uint32_t dsr                : 1;      ///< DSR Line: 0=not available, 1=available
                    0,//uint32_t dcd                : 1;      ///< DCD Line: 0=not available, 1=available
                    0,//uint32_t ri                 : 1;      ///< RI Line: 0=not available, 1=available
                    0,//uint32_t event_cts          : 1;      ///< Signal CTS change event: \ref ARM_USART_EVENT_CTS
                    0,//uint32_t event_dsr          : 1;      ///< Signal DSR change event: \ref ARM_USART_EVENT_DSR
                    0,//uint32_t event_dcd          : 1;      ///< Signal DCD change event: \ref ARM_USART_EVENT_DCD
                    0,//uint32_t event_ri           : 1;      ///< Signal RI change event: \ref ARM_USART_EVENT_RI
                    0//uint32_t reserved           : 11;     ///< Reserved (must be zero)
            }
        #endif    
        }
    },
#endif
#if (!defined(UART)) && defined(UART0) && (APOLLOUART0_ENABLED == 1)
    { (UART0),   // pstcInstance
    { //stc_apollouart_intern_data_t
            NULL, //pfn_apollouart_txnext_t cbTxNext;
            NULL, //pfn_apollouart_rx_t cbRx;   
            FALSE, //bInitialized   
            0, //u32RegCR
            (3 << UART_LCRH_WLEN_Pos), //u32RegLCRH, default: 8 databits, no parity, 1 stopbit
            9600, //u32Baudrate, default 9600 baud
        #if defined(USE_CMSIS_DRIVER)
            &Driver_UART0,
            NULL, //ARM_USART_SignalEvent pfnCmsisSignalEvent
            { //ARM_USART_CAPABILITIES u32Capabilities; 
                    1,//uint32_t asynchronous       : 1;      ///< supports UART (Asynchronous) mode 
                    0,//uint32_t synchronous_master : 1;      ///< supports Synchronous Master mode
                    0,//uint32_t synchronous_slave  : 1;      ///< supports Synchronous Slave mode
                    0,//uint32_t single_wire        : 1;      ///< supports UART Single-wire mode
                    0,//uint32_t irda               : 1;      ///< supports UART IrDA mode
                    0,//uint32_t smart_card         : 1;      ///< supports UART Smart Card mode
                    0,//uint32_t smart_card_clock   : 1;      ///< Smart Card Clock generator available
                    0,//uint32_t flow_control_rts   : 1;      ///< RTS Flow Control available
                    0,//uint32_t flow_control_cts   : 1;      ///< CTS Flow Control available
                    0,//uint32_t event_tx_complete  : 1;      ///< Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
                    0,//uint32_t event_rx_timeout   : 1;      ///< Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
                    0,//uint32_t rts                : 1;      ///< RTS Line: 0=not available, 1=available
                    0,//uint32_t cts                : 1;      ///< CTS Line: 0=not available, 1=available
                    0,//uint32_t dtr                : 1;      ///< DTR Line: 0=not available, 1=available
                    0,//uint32_t dsr                : 1;      ///< DSR Line: 0=not available, 1=available
                    0,//uint32_t dcd                : 1;      ///< DCD Line: 0=not available, 1=available
                    0,//uint32_t ri                 : 1;      ///< RI Line: 0=not available, 1=available
                    0,//uint32_t event_cts          : 1;      ///< Signal CTS change event: \ref ARM_USART_EVENT_CTS
                    0,//uint32_t event_dsr          : 1;      ///< Signal DSR change event: \ref ARM_USART_EVENT_DSR
                    0,//uint32_t event_dcd          : 1;      ///< Signal DCD change event: \ref ARM_USART_EVENT_DCD
                    0,//uint32_t event_ri           : 1;      ///< Signal RI change event: \ref ARM_USART_EVENT_RI
                    0//uint32_t reserved           : 11;     ///< Reserved (must be zero)
            } 
        #endif    
        }
    },
#endif
#if defined(UART1) && (APOLLOUART1_ENABLED == 1)
    { (UART1),   // pstcInstance
    { //stc_apollouart_intern_data_t
            NULL, //pfn_apollouart_txnext_t cbTxNext;
            NULL, //pfn_apollouart_rx_t cbRx;   
            FALSE, //bInitialized   
            0, //u32RegCR
            (3 << UART_LCRH_WLEN_Pos), //u32RegLCRH, default: 8 databits, no parity, 1 stopbit
            9600, //u32Baudrate, default 9600 baud
        #if defined(USE_CMSIS_DRIVER)
            &Driver_UART1,
            NULL, //ARM_USART_SignalEvent pfnCmsisSignalEvent
            { //ARM_USART_CAPABILITIES Capabilities; 
                    1,//uint32_t asynchronous       : 1;      ///< supports UART (Asynchronous) mode 
                    0,//uint32_t synchronous_master : 1;      ///< supports Synchronous Master mode
                    0,//uint32_t synchronous_slave  : 1;      ///< supports Synchronous Slave mode
                    0,//uint32_t single_wire        : 1;      ///< supports UART Single-wire mode
                    0,//uint32_t irda               : 1;      ///< supports UART IrDA mode
                    0,//uint32_t smart_card         : 1;      ///< supports UART Smart Card mode
                    0,//uint32_t smart_card_clock   : 1;      ///< Smart Card Clock generator available
                    0,//uint32_t flow_control_rts   : 1;      ///< RTS Flow Control available
                    0,//uint32_t flow_control_cts   : 1;      ///< CTS Flow Control available
                    0,//uint32_t event_tx_complete  : 1;      ///< Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
                    0,//uint32_t event_rx_timeout   : 1;      ///< Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
                    0,//uint32_t rts                : 1;      ///< RTS Line: 0=not available, 1=available
                    0,//uint32_t cts                : 1;      ///< CTS Line: 0=not available, 1=available
                    0,//uint32_t dtr                : 1;      ///< DTR Line: 0=not available, 1=available
                    0,//uint32_t dsr                : 1;      ///< DSR Line: 0=not available, 1=available
                    0,//uint32_t dcd                : 1;      ///< DCD Line: 0=not available, 1=available
                    0,//uint32_t ri                 : 1;      ///< RI Line: 0=not available, 1=available
                    0,//uint32_t event_cts          : 1;      ///< Signal CTS change event: \ref ARM_USART_EVENT_CTS
                    0,//uint32_t event_dsr          : 1;      ///< Signal DSR change event: \ref ARM_USART_EVENT_DSR
                    0,//uint32_t event_dcd          : 1;      ///< Signal DCD change event: \ref ARM_USART_EVENT_DCD
                    0,//uint32_t event_ri           : 1;      ///< Signal RI change event: \ref ARM_USART_EVENT_RI
                    0//uint32_t reserved           : 11;     ///< Reserved (must be zero)
            } 
        #endif    
        }
    },
#endif
};    


/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/*****************************************************************************
 *   CCCCCC   M         M     SSSSSS    I   SSSSSS  
 *  C         M M     M M    S          I  S         
 *  C         M  M   M  M    S          I  S      
 *  C         M    M    M     SSSSSS    I   SSSSSS  
 *  C         M         M           S   I         S  
 *  C         M         M           S   I         S  
 *   CCCCCC   M         M     SSSSSS    I   SSSSSS  
 *
 *  >> CMSIS Driver API Start
 *
 *****************************************************************************/
#if defined(USE_CMSIS_DRIVER)

/**
  \fn          ARM_DRIVER_VERSION USARTx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion(void)
{
    return drv_vers;
}


/**
  \fn          ARM_USART_STATUS USART_GetStatus (USART_RESOURCES *usart)
  \brief       Get USART status.
  \param[in]   pstcUart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS GetStatus(UART_Type* pstcUart)
{
    ARM_USART_STATUS stat;
    memset((void*)&stat,0,sizeof(stat));
    stat.tx_busy = pstcUart->FR_b.TXFF;
    stat.rx_busy = pstcUart->FR_b.RXFE;   
    stat.rx_framing_error = pstcUart->RSR_b.FESTAT;
    stat.rx_parity_error =  pstcUart->RSR_b.PESTAT;
    stat.rx_overflow =  pstcUart->RSR_b.OESTAT;
    stat.rx_break = pstcUart->RSR_b.BESTAT;
    return stat;
}

/**
  \fn          ARM_USART_MODEM_STATUS USART_GetModemStatus (USART_RESOURCES *usart)
  \brief       Get USART Modem Status lines state.
  \param[in]   pstcUart     Pointer to USART resources
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS GetModemStatus(UART_Type* pstcUart)
{
    ARM_USART_MODEM_STATUS stat;
    memset((void*)&stat,0,sizeof(stat));
    stat.cts = pstcUart->FR_b.CTS;
    stat.dsr = pstcUart->FR_b.DSR;
    stat.dcd = pstcUart->FR_b.DCD;
    return stat;
}

/**
  \fn          int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL  control,
                                              USART_RESOURCES         *usart)
  \brief       Set USART Modem Control line state.
  \param[in]   pstcUart     Pointer to USART resources
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t SetModemControl(UART_Type* pstcUart, ARM_USART_MODEM_CONTROL control)
{
    stc_apollouart_intern_data_t* pstcIntHandle = GetInternDataPtr(pstcUart);
  
    //not yet implemented

   if (control == ARM_USART_RTS_CLEAR) {
    if (pstcIntHandle->Capabilities.rts) {  }
    else                         { return ARM_DRIVER_ERROR_UNSUPPORTED;       }
  }
  if (control == ARM_USART_RTS_SET) {
    if (pstcIntHandle->Capabilities.rts) {  }
    else                         {return ARM_DRIVER_ERROR_UNSUPPORTED;        }
  }
  if (control == ARM_USART_DTR_CLEAR) {
    if (pstcIntHandle->Capabilities.dtr) {  }
    else                         { return ARM_DRIVER_ERROR_UNSUPPORTED;       }
  }
  if (control == ARM_USART_DTR_SET) {
    if (pstcIntHandle->Capabilities.dtr) {  }
    else                         { return ARM_DRIVER_ERROR_UNSUPPORTED;       }
  }
  return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USART_Control (uint32_t          control,
                                      uint32_t          arg,
                                      USART_RESOURCES  *usart)
  \brief       Control USART Interface.
  \param[in]   pstcUart    Pointer to USART resources
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t Control(UART_Type* pstcUart, uint32_t control, uint32_t arg)
{
      stc_apollouart_intern_data_t* pstcIntHandle = GetInternDataPtr(pstcUart);
      boolean_t bUartEn = pstcUart->CR_b.UARTEN;
      if (control & ARM_USART_MODE_ASYNCHRONOUS)
      {
          pstcUart->CR_b.UARTEN = 0;                //disable UART
          SystemCoreClockUpdate();
          pstcIntHandle->u32Baudrate = arg;
          ConfigureBaudrate(pstcUart,arg,SystemCoreClock);
      }
      if (control & ARM_USART_PARITY_EVEN)
      {
          pstcUart->CR_b.UARTEN = 0;                //disable UART
          pstcUart->LCRH_b.PEN = 1;
          pstcUart->LCRH_b.EPS = 1;
          pstcIntHandle->u32RegLCRH |= (1 << UART_LCRH_PEN_Pos);
          pstcIntHandle->u32RegLCRH |= (1 << UART_LCRH_EPS_Pos);
      }
      if (control & ARM_USART_PARITY_NONE)
      {
          pstcUart->CR_b.UARTEN = 0;                //disable UART
          pstcUart->LCRH_b.PEN = 0;
          pstcIntHandle->u32RegLCRH &= ~(1 << UART_LCRH_PEN_Pos);
      }
      if (control & ARM_USART_PARITY_ODD)
      {
          pstcUart->CR_b.UARTEN = 0;                //disable UART
          pstcUart->LCRH_b.PEN = 1;
          pstcUart->LCRH_b.EPS = 0;
          pstcIntHandle->u32RegLCRH |= (1 << UART_LCRH_PEN_Pos);
          pstcIntHandle->u32RegLCRH &= ~(1 << UART_LCRH_EPS_Pos);
      }
      if (control & ARM_USART_STOP_BITS_1)
      {
          pstcUart->CR_b.UARTEN = 0;                //disable UART
          pstcUart->LCRH_b.STP2 = 0;
          pstcIntHandle->u32RegLCRH &= ~(1 << UART_LCRH_STP2_Pos);
      }
      if (control & ARM_USART_STOP_BITS_2)
      {
          pstcUart->CR_b.UARTEN = 0;                //disable UART
          pstcUart->LCRH_b.STP2 = 1;
          pstcIntHandle->u32RegLCRH |= (1 << UART_LCRH_STP2_Pos);
      }
      if (control & ARM_USART_FLOW_CONTROL_NONE)
      {
      }
      if (control & ARM_USART_FLOW_CONTROL_CTS)
      {
      }
      if (control & ARM_USART_FLOW_CONTROL_RTS)
      {
      }
      if (control & ARM_USART_ABORT_RECEIVE)
      {
      }
      if (control & ARM_USART_ABORT_SEND)
      {
      }
      if (control & ARM_USART_ABORT_TRANSFER)
      {
      }
      if (control & ARM_USART_CONTROL_BREAK)
      {
      }
      if (control & ARM_USART_CONTROL_TX)
      {
          pstcUart->CR_b.TXE = arg;
          pstcIntHandle->u32RegCR &= ~(UART_CR_TXE_Msk);
          pstcIntHandle->u32RegCR |= (arg << UART_CR_TXE_Pos);
      }
      if (control & ARM_USART_CONTROL_RX)
      {
          pstcUart->CR_b.RXE = arg;
          pstcIntHandle->u32RegCR &= ~(UART_CR_RXE_Msk);
          pstcIntHandle->u32RegCR |= (arg << UART_CR_RXE_Pos);
      }
      
      if (pstcUart->CR_b.UARTEN != bUartEn) pstcUart->CR_b.UARTEN = bUartEn;           //restore old UART enable state
      
      return ARM_DRIVER_OK;
}



/**
  \fn          int32_t USART_Initialize (ARM_USART_SignalEvent_t  cb_event
                                         USART_RESOURCES         *usart)
  \brief       Initialize USART Interface.
  \param[in]   pstcUart     Pointer to USART resources
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize(UART_Type* pstcUart, ARM_USART_SignalEvent_t cb_event)
{
    stc_apollouart_intern_data_t* pstcIntHandle = GetInternDataPtr(pstcUart);
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
    pstcIntHandle->u32RegCR &= ~UART_CR_CLKSEL_Msk;
    pstcIntHandle->u32RegCR |= (1 << UART_CR_CLKEN_Pos);
    pstcIntHandle->u32RegCR |= (1 << UART_CR_CLKSEL_Pos);
    
    //
    // Disable UART before config...
    //
    pstcUart->CR_b.UARTEN = 0;                //disable UART
    pstcUart->CR_b.RXE = 0;                   //disable receiver
    pstcUart->CR_b.TXE = 0;                   //disable transmitter
    
    //
    // Starting UART config...
    //
    
    // initialize baudrate before all other settings, otherwise UART will not be initialized
    SystemCoreClockUpdate();
    ConfigureBaudrate(pstcUart,pstcIntHandle->u32Baudrate,SystemCoreClock);
    
    // initialize line coding...
    pstcUart->LCRH = pstcIntHandle->u32RegLCRH;
    
    //
    // Enable UART after config...
    //
    pstcUart->CR_b.UARTEN = 1;                //enable UART
    pstcUart->CR = pstcIntHandle->u32RegCR | (0x01);
    return ARM_DRIVER_OK;
}



/**
  \fn          int32_t USART_Uninitialize (USART_RESOURCES *usart)
  \param[in]   pstcUart     Pointer to USART resources
  \brief       De-initialize USART Interface.
  \return      \ref execution_status
*/
static int32_t Uninitialize(UART_Type* pstcUart)
{
    //
    // Disable UART before config...
    //
    pstcUart->CR_b.UARTEN = 0;                //disable UART
    
    //
    // Disable clock
    //
    pstcUart->CR = 0;
   
    //
    // Disable UART clock in CLKGEN
    //
#if defined(APOLLO_H) || defined(APOLLO1_H)
    CLKGEN->UARTEN_b.UARTEN = 0;        //enable UART clocking
#endif
    return ARM_DRIVER_OK;
}


/**
  \fn          int32_t USART_PowerControl (ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   pstcUart  Pointer to USART resources
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl(UART_Type* pstcUart, ARM_POWER_STATE state)
{
    stc_apollouart_intern_data_t* pstcIntHandle = GetInternDataPtr(pstcUart);
    switch(state)
    {
      case ARM_POWER_OFF:
        break;
      case ARM_POWER_LOW:
        break;
      case ARM_POWER_FULL:
        break;
    }
    return ARM_DRIVER_OK;
}





#if (defined(UART0)) && (APOLLOUART0_ENABLED == 1)
/**
  \fn          ARM_USART_CAPABILITIES GetCapabilities_UART0 (void)
  \brief       Get driver capabilities
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES GetCapabilities_UART0(void)
{
    stc_apollouart_intern_data_t* pstcIntHandle = GetInternDataPtr(UART0);
    return pstcIntHandle->Capabilities;
}

/**
  \fn          ARM_USART_STATUS GetStatus_UART0 (void)
  \brief       Get USART status.
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS GetStatus_UART0(void)
{
    return GetStatus(UART0);
}


/**
  \fn          int32_t Initialize_UART0 (ARM_USART_SignalEvent_t  cb_event)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize_UART0(ARM_USART_SignalEvent_t cb_event)
{
    return Initialize(UART0,cb_event);
}


/**
  \fn          int32_t Uninitialize_UART0 (void)
  \brief       De-initialize USART Interface.
  \return      \ref execution_status
*/
static int32_t Uninitialize_UART0(void)
{
    return Uninitialize(UART0);
}


/**
  \fn          int32_t PowerControl_UART0 (ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl_UART0(ARM_POWER_STATE state)
{
    return PowerControl(UART0, state);
}


/**
  \fn          int32_t Send_UART0 (const void            *data,
                                         uint32_t         num)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t Send_UART0(const void *data, uint32_t num)
{
    ApolloUart_SendPolled(UART0,(uint8_t*)data,num);
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Receive_UART0 (void            *data,
                                      uint32_t         num)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t Receive_UART0(void *data, uint32_t num)
{
    ApolloUart_ReceivePolled(UART0,(uint8_t*)data,num);
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Transfer_UART0 (const void             *data_out,
                                             void             *data_in,
                                             uint32_t          num)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t Transfer_UART0(const void *data_out,void *data_in,uint32_t num)
{
    ApolloUart_TransferPolled(UART0,(uint8_t*)data_out,(uint8_t*)data_in,num);
    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t GetTxCount_UART0 (USART_RESOURCES *usart)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
static uint32_t GetTxCount_UART0(void)
{
    return 0;
}

/**
  \fn          uint32_t GetRxCount_UART0 (void)
  \brief       Get received data count.
  \return      number of data items received
*/
static uint32_t GetRxCount_UART0(void)
{
    return (UART0->FR_b.RXFE == 0);
}

/**
  \fn          int32_t Control_UART0 (uint32_t          control,
                                      uint32_t          arg)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t Control_UART0(uint32_t control, uint32_t arg)
{
    return Control(UART0,control,arg);
}


/**
  \fn          int32_t SetModemControl_UART0 (ARM_USART_MODEM_CONTROL  control)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t SetModemControl_UART0(ARM_USART_MODEM_CONTROL control)
{
    return SetModemControl(UART0,control);
}


/**
  \fn          ARM_USART_STATUS USART_GetStatus (void)
  \brief       Get USART status.
  \param[in]   usart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_MODEM_STATUS GetModemStatus_UART0(void)
{
    return GetModemStatus(UART0);
}
#endif //end UART / UART0





#if (defined(UART1)) && (APOLLOUART1_ENABLED == 1)
/**
  \fn          ARM_USART_CAPABILITIES GetCapabilities_UART1 (void)
  \brief       Get driver capabilities
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES GetCapabilities_UART1(void)
{
    stc_apollouart_intern_data_t* pstcIntHandle = GetInternDataPtr(UART1);
    return pstcIntHandle->Capabilities;
}

/**
  \fn          ARM_USART_STATUS GetStatus_UART1 (void)
  \brief       Get USART status.
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS GetStatus_UART1(void)
{
    return GetStatus(UART1);
}


/**
  \fn          int32_t Initialize_UART1 (ARM_USART_SignalEvent_t  cb_event)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize_UART1(ARM_USART_SignalEvent_t cb_event)
{
    return Initialize(UART1,cb_event);
}


/**
  \fn          int32_t Uninitialize_UART1 (void)
  \brief       De-initialize USART Interface.
  \return      \ref execution_status
*/
static int32_t Uninitialize_UART1(void)
{
    return Uninitialize(UART1);
}


/**
  \fn          int32_t PowerControl_UART1 (ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl_UART1(ARM_POWER_STATE state)
{
    return PowerControl(UART1, state)
}


/**
  \fn          int32_t Send_UART1 (const void            *data,
                                         uint32_t         num)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t Send_UART1(const void *data, uint32_t num)
{
    ApolloUart_SendPolled(UART1,(uint8_t*)data,num);
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Receive_UART1 (void            *data,
                                      uint32_t         num)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t Receive_UART1(void *data, uint32_t num)
{
    ApolloUart_ReceivePolled(UART1,(uint8_t*)data,num);
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Transfer_UART1 (const void             *data_out,
                                             void             *data_in,
                                             uint32_t          num)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t Transfer_UART1(const void *data_out,void *data_in,uint32_t num)
{
    ApolloUart_TransferPolled(UART1,(uint8_t*)data_out,(uint8_t*)data_in,num);
    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t GetTxCount_UART1 (USART_RESOURCES *usart)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
static uint32_t GetTxCount_UART1(void)
{
    return 0;
}

/**
  \fn          uint32_t GetRxCount_UART1 (void)
  \brief       Get received data count.
  \return      number of data items received
*/
static uint32_t GetRxCount_UART1(void)
{
    return (UART1->FR_b.RXFE == 0);
}

/**
  \fn          int32_t Control_UART1 (uint32_t          control,
                                      uint32_t          arg)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t Control_UART1(uint32_t control, uint32_t arg)
{
    return Control(UART1,control,arg);
}


/**
  \fn          int32_t SetModemControl_UART1 (ARM_USART_MODEM_CONTROL  control)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t SetModemControl_UART1(ARM_USART_MODEM_CONTROL control)
{
    return SetModemControl(UART1,control);
}


/**
  \fn          ARM_USART_STATUS USART_GetStatus (void)
  \brief       Get USART status.
  \param[in]   usart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_MODEM_STATUS GetModemStatus_UART1(void)
{
    return GetModemStatus(UART1);
}
#endif //end UART1

#endif
/*****************************************************************************
 *
 * << CMSIS Driver API End
 *
 *****************************************************************************/






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
 ** \brief  returns incoming character, if received on UART
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

/**
 ******************************************************************************
 ** \brief  Check if data is available
 **
 ** \param  pstcUart  UART pointer
 **
 ** \return TRUE, if data is available
 *****************************************************************************/
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
 ** \brief  Send data polled (blocking)
 **
 ** \param  pstcUart         UART pointer
 ** 
 ** \param  pu8Data          Databuffer to send
 **
 ** \param u32Size           Size of Data
 **
 *****************************************************************************/
void ApolloUart_SendPolled(UART_Type* pstcUart, uint8_t* pu8Data,uint32_t u32Size)
{
    uint32_t i;
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) return;
    for(i = 0;i < u32Size;i++)
    {
        while(pstcUart->FR_b.TXFF) __NOP();
        pstcUart->DR = pu8Data[i];
    }
}

/**
 ******************************************************************************
 ** \brief  Receive data polled (blocking, no timeout feature!)
 **
 ** \param  pstcUart         UART pointer
 ** 
 ** \param  pu8Data          Databuffer to receive
 **
 ** \param u32Size           Size of Data
 **
 *****************************************************************************/
void ApolloUart_ReceivePolled(UART_Type* pstcUart, uint8_t* pu8Data,uint32_t u32Size)
{
    uint32_t i;
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) return;
    for(i = 0;i < u32Size;i++)
    {
        while(pstcUart->FR_b.RXFE) __NOP();
        pu8Data[i] = pstcUart->DR;
    }
}

/**
 ******************************************************************************
 ** \brief  Asynchrounous receive and send data (experimental)
 **
 ** \param  pstcUart         UART pointer
 ** 
 ** \param  pu8DataOut       Databuffer to send
 ** 
 ** \param  pu8DataIn        Databuffer to receive
 **
 ** \param u32Size           Size of Data
 **
 *****************************************************************************/
void ApolloUart_TransferPolled(UART_Type* pstcUart, uint8_t* pu8DataOut,uint8_t* pu8DataIn,uint32_t u32Size)
{
    uint32_t i;
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) return;
    for(i = 0;i < u32Size;i++)
    {
        while(pstcUart->FR_b.TXFF) __NOP();
        pstcUart->DR = pu8DataOut[i];
        while(pstcUart->FR_b.RXFE) __NOP();
        pu8DataIn[i] = pstcUart->DR;
        
    }
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
 ** \param pstcUart UART Handle
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
/**
 ******************************************************************************
 ** \brief  IRQ handler UART (Apollo 1)
 **
 *****************************************************************************/
void UART_IRQHandler(void)
{
    UARTn_IRQHandler(UART);
}
#endif

#if defined(UART0) && (APOLLOUART0_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  IRQ handler UART0
 **
 *****************************************************************************/
void UART0_IRQHandler(void)
{
    UARTn_IRQHandler(UART0);
}
#endif

#if defined(UART1) && (APOLLOUART1_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  IRQ handler UART1
 **
 *****************************************************************************/
void UART1_IRQHandler(void)
{
    UARTn_IRQHandler(UART1);
}
#endif

/**
 ******************************************************************************
 ** \brief  IRQ handler depending on UART
 **
 ** \param pstcUart UART Handle
 **
 *****************************************************************************/
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

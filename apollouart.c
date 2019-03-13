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
* NO WARRANTY: This software is provided "as-is" with no warranty of any kind  *
* (German "Unter Ausschluss jeglicher Gewaehleistung", express or implied,     *
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
 **   - 2016-11-26  V1.0  Manuel Schreiner   First Version
 **   - 2017-04-13  V1.1  Manuel Schreiner   Added IRQs and callbacks
 **   - 2017-05-16  V1.2  Manuel Schreiner   Added Apollo 2 support
 **   - 2017-06-26  V1.3  Manuel Schreiner   Added CMSIS Driver API
 **   - 2017-07-26  V1.4  Manuel Schreiner   Fixed error if CMSIS Driver API is disabled
 **   - 2018-03-15  V1.5  Manuel Schreiner   Fixed interrupt handling
 **   - 2018-04-24  V1.6  Manuel Schreiner   Added configuration by pin (for Arduino or MBED based SDKs)
 **                                          Added extended configuration options
 **   - 2018-07-06  V1.7  Manuel Schreiner   Updated documentation,
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **   - 2018-07-24  V1.8  Manuel Schreiner   Updated pin-configuration and status-infomration
 **   - 2019-01-15  V1.9  Manuel Schreiner   Fixed input/output pin configuration setup
 **                                          Added debug tracing
 **   - 2019-03-07  V2.0  Manuel Schreiner   Fixed not initialized FIFO in extended UART initialization
 **   - 2019-03-12  V2.0a Manuel Schreiner   Added advanced IRQ handling
 **
 *****************************************************************************/
#define __APOLLOUART_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "apollouart.h"
#include "stdio.h"
#include "string.h"
#include "mcu.h"
#if (APOLLOGPIO_ENABLED == 1)
#include "apollogpio.h"
#endif

#if (UART_DEBUG == 1)
  #warning UART_DEBUG == 1
  #if APOLLOUART_ENABLED == 1
     #warning APOLLOUART_ENABLED == 1
     #if defined(UART)
     #warning   ==> APOLLOUART_ENABLED == 1
     #endif
     #if defined(UART0)
     #warning   ==> APOLLOUART0_ENABLED == 1
     #endif
     #if defined(UART1)
     #warning   ==> APOLLOUART1_ENABLED == 1
     #endif
  #else
    #if APOLLOUART0_ENABLED == 1
       #warning APOLLOUART0_ENABLED == 1
    #endif
    #if APOLLOUART1_ENABLED == 1
       #warning APOLLOUART1_ENABLED == 1
    #endif
  #endif
#endif

#if (APOLLOUART_ENABLED == 1) || (APOLLOUART0_ENABLED == 1) || (APOLLOUART1_ENABLED == 1)
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define INSTANCE_COUNT (uint32_t)(sizeof(m_astcInstanceDataLut) / sizeof(m_astcInstanceDataLut[0]))
#define UARTGPIOS_COUNT (uint32_t)(sizeof(stcUartGpios) / sizeof(stcUartGpios[0]))

//set DEBUG_OUTPUT 1 and IOM_DEBUG to 1 in RTE_Device.h to have Debug information output
#if (DEBUG_OUTPUT == 1) && (!defined(debugprint))
#define debugprint(...) if ((CoreDebug->DHCSR & (1 << CoreDebug_DHCSR_C_DEBUGEN_Pos)) != 0) printf(__VA_ARGS__)
#define debugprintln(...) debugprint(__VA_ARGS__); debugprint("\r\n")
#endif

//#if !defined(debugprint)
//#define debugprint(...)   
//#endif
//#if !defined(debugprintln)
//#define debugprintln(...) 
//#endif

#if (UART_DEBUG == 1) && (DEBUG_OUTPUT == 0)
#warning UART_DEBUG is 1 but DEBUG_OUT is 0, so no output is produced
#endif
#if UART_DEBUG == 1
#define UART_DEBUG_PRINT(...) debugprint(__VA_ARGS__)
#define UART_DEBUG_PRINTLN(...) debugprintln(__VA_ARGS__)
#define UART_ASSERT(...) debugprint("Error: "); debugprint(__FILE__); debugprint(", "); debugprint("#%d",__LINE__); debugprintln(":");  debugprint(__VA_ARGS__)
#else 
#define UART_DEBUG_PRINT(...)
#define UART_DEBUG_PRINTLN(...)
#define UART_ASSERT(...)
#endif

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

static stc_apollouart_intern_data_t* GetInternDataPtr(UART_Type* pstcHandle);
static void ConfigureBaudrate(UART_Type* pstcUart,uint32_t u32Baudrate, uint32_t u32UartClkFreq);

static const stc_apollouart_gpios_t stcUartGpios[] =
{
#if defined(APOLLO_H) || defined(APOLLO1_H)
    {UART,0,2,ApolloUartGpioTypeTx},
    {UART,1,2,ApolloUartGpioTypeRx},
    {UART,5,2,ApolloUartGpioTypeRts},
    {UART,6,2,ApolloUartGpioTypeCts},
    {UART,14,2,ApolloUartGpioTypeTx},
    {UART,15,2,ApolloUartGpioTypeRx},
    {UART,22,0,ApolloUartGpioTypeTx},
    {UART,23,0,ApolloUartGpioTypeRx},
    {UART,35,2,ApolloUartGpioTypeTx},
    {UART,36,2,ApolloUartGpioTypeRx},
    {UART,37,2,ApolloUartGpioTypeRts},
    {UART,38,2,ApolloUartGpioTypeCts},
    {UART,39,1,ApolloUartGpioTypeTx},
    {UART,40,1,ApolloUartGpioTypeRx},
#endif

#if defined(APOLLO2_H)
    #if (defined(UART) || defined(UART0)) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
        {UART0,1,2,ApolloUartGpioTypeTx},
        {UART0,2,2,ApolloUartGpioTypeRx},
        {UART0,3,0,ApolloUartGpioTypeRts},
        {UART0,4,0,ApolloUartGpioTypeCts},
        {UART0,5,2,ApolloUartGpioTypeRts},
        {UART0,6,2,ApolloUartGpioTypeCts},
        {UART0,7,5,ApolloUartGpioTypeTx},
        {UART0,11,6,ApolloUartGpioTypeRx},
        {UART0,12,6,ApolloUartGpioTypeCts},
        {UART0,13,6,ApolloUartGpioTypeRts},
        {UART0,16,6,ApolloUartGpioTypeTx},
        {UART0,17,6,ApolloUartGpioTypeRx},
        {UART0,20,4,ApolloUartGpioTypeTx},
        {UART0,21,4,ApolloUartGpioTypeRx},
        {UART0,22,0,ApolloUartGpioTypeTx},
        {UART0,23,0,ApolloUartGpioTypeRx},
        {UART0,29,4,ApolloUartGpioTypeCts},
        {UART0,30,4,ApolloUartGpioTypeTx},
        {UART0,31,4,ApolloUartGpioTypeRx},
        {UART0,35,6,ApolloUartGpioTypeRts},
        {UART0,36,6,ApolloUartGpioTypeCts},
        {UART0,37,2,ApolloUartGpioTypeRts},
        {UART0,38,2,ApolloUartGpioTypeCts},
        {UART0,39,0,ApolloUartGpioTypeTx},
        {UART0,40,0,ApolloUartGpioTypeRx},
        {UART0,41,7,ApolloUartGpioTypeRts},
    #endif
    #if (defined(UART1)) && (APOLLOUART1_ENABLED == 1)
        {UART1,8,6,ApolloUartGpioTypeTx},
        {UART1,9,6,ApolloUartGpioTypeRx},
        {UART1,10,5,ApolloUartGpioTypeRts},
        {UART1,11,5,ApolloUartGpioTypeCts},
        {UART1,13,7,ApolloUartGpioTypeTx},
        {UART1,14,7,ApolloUartGpioTypeRx},
        {UART1,16,7,ApolloUartGpioTypeRts},
        {UART1,17,7,ApolloUartGpioTypeCts},
        {UART1,18,6,ApolloUartGpioTypeTx},
        {UART1,19,6,ApolloUartGpioTypeRx},
        {UART1,20,5,ApolloUartGpioTypeTx},
        {UART1,21,5,ApolloUartGpioTypeRx},
        {UART1,29,5,ApolloUartGpioTypeCts},
        {UART1,30,5,ApolloUartGpioTypeRts},
        {UART1,35,2,ApolloUartGpioTypeTx},
        {UART1,36,2,ApolloUartGpioTypeRx},
        {UART1,39,2,ApolloUartGpioTypeTx},
        {UART1,40,2,ApolloUartGpioTypeRx},
        {UART1,44,0,ApolloUartGpioTypeRts},
        {UART1,45,0,ApolloUartGpioTypeCts},
    #endif
#endif
#if defined(APOLLO3_H)
    #if (defined(UART) || defined(UART0)) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
        {UART0,1,2,ApolloUartGpioTypeTx},
        {UART0,2,2,ApolloUartGpioTypeRx},
        {UART0,3,0,ApolloUartGpioTypeRts},
        {UART0,4,0,ApolloUartGpioTypeCts},
        {UART0,5,2,ApolloUartGpioTypeRts},
        {UART0,6,2,ApolloUartGpioTypeCts},
        {UART0,7,5,ApolloUartGpioTypeTx},
        {UART0,11,6,ApolloUartGpioTypeRx},
        {UART0,12,6,ApolloUartGpioTypeCts},
        {UART0,13,6,ApolloUartGpioTypeRts},
        {UART0,16,6,ApolloUartGpioTypeTx},
        {UART0,17,6,ApolloUartGpioTypeRx},
        {UART0,18,6,ApolloUartGpioTypeRts},
        {UART0,20,4,ApolloUartGpioTypeTx},
        {UART0,21,4,ApolloUartGpioTypeRx},
        {UART0,22,0,ApolloUartGpioTypeTx},
        {UART0,23,0,ApolloUartGpioTypeRx},
        {UART0,24,4,ApolloUartGpioTypeCts},
        {UART0,26,6,ApolloUartGpioTypeTx},
        {UART0,27,0,ApolloUartGpioTypeRx},
        {UART0,28,6,ApolloUartGpioTypeTx},
        {UART0,29,6,ApolloUartGpioTypeRx},
        {UART0,29,4,ApolloUartGpioTypeCts},
        {UART0,30,4,ApolloUartGpioTypeTx},
        {UART0,31,4,ApolloUartGpioTypeRx},
        {UART0,33,3,ApolloUartGpioTypeCts},
        {UART0,34,5,ApolloUartGpioTypeRts},
        {UART0,34,6,ApolloUartGpioTypeRx},
        {UART0,35,6,ApolloUartGpioTypeRts},
        {UART0,36,6,ApolloUartGpioTypeCts},
        {UART0,37,2,ApolloUartGpioTypeRts},
        {UART0,38,2,ApolloUartGpioTypeCts},
        {UART0,39,0,ApolloUartGpioTypeTx},
        {UART0,40,0,ApolloUartGpioTypeRx},
        {UART0,41,7,ApolloUartGpioTypeRts},
        {UART0,41,6,ApolloUartGpioTypeRx},
        {UART0,44,6,ApolloUartGpioTypeTx},
        {UART0,45,6,ApolloUartGpioTypeRx},
        {UART0,48,0,ApolloUartGpioTypeTx},
        {UART0,49,0,ApolloUartGpioTypeRx},
    #endif
    #if (defined(UART1)) && (APOLLOUART1_ENABLED == 1)
        {UART1,2,0,ApolloUartGpioTypeRx},
        {UART1,4,5,ApolloUartGpioTypeRx},
        {UART1,8,6,ApolloUartGpioTypeTx},
        {UART1,9,6,ApolloUartGpioTypeRx},
        {UART1,10,5,ApolloUartGpioTypeRts},
        {UART1,10,0,ApolloUartGpioTypeTx},
        {UART1,11,5,ApolloUartGpioTypeCts},
        {UART1,12,7,ApolloUartGpioTypeTx},
        {UART1,13,7,ApolloUartGpioTypeRx},
        {UART1,14,2,ApolloUartGpioTypeTx},
        {UART1,15,2,ApolloUartGpioTypeRx},
        {UART1,16,7,ApolloUartGpioTypeRts},
        {UART1,17,7,ApolloUartGpioTypeCts},
        {UART1,18,6,ApolloUartGpioTypeTx},
        {UART1,19,6,ApolloUartGpioTypeRx},
        {UART1,20,5,ApolloUartGpioTypeTx},
        {UART1,21,5,ApolloUartGpioTypeRx},
        {UART1,24,0,ApolloUartGpioTypeTx},
        {UART1,25,0,ApolloUartGpioTypeRx},
        {UART1,26,7,ApolloUartGpioTypeCts},
        {UART1,29,5,ApolloUartGpioTypeCts},
        {UART1,30,5,ApolloUartGpioTypeRts},
        {UART1,31,7,ApolloUartGpioTypeRts},
        {UART1,32,7,ApolloUartGpioTypeCts},
        {UART1,34,2,ApolloUartGpioTypeRts},
        {UART1,35,2,ApolloUartGpioTypeTx},
        {UART1,36,2,ApolloUartGpioTypeRx},
        {UART1,36,5,ApolloUartGpioTypeCts},
        {UART1,37,5,ApolloUartGpioTypeTx},
        {UART1,38,6,ApolloUartGpioTypeRx},
        {UART1,39,2,ApolloUartGpioTypeTx},
        {UART1,40,2,ApolloUartGpioTypeRx},
        {UART1,41,5,ApolloUartGpioTypeRts},
        {UART1,42,0,ApolloUartGpioTypeRx},
        {UART1,43,0,ApolloUartGpioTypeTx},
        {UART1,44,0,ApolloUartGpioTypeRts},
        {UART1,45,0,ApolloUartGpioTypeCts},
        {UART1,46,6,ApolloUartGpioTypeRx},
        {UART1,47,6,ApolloUartGpioTypeTx},
    #endif
#endif
};

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

    #if (defined(UART) || defined(UART0)) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
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
            FALSE, //bRxEnabled
            FALSE, //bTxEnabled
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
            FALSE, //bRxEnabled
            FALSE, //bTxEnabled
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





#if (defined(UART0)) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
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
** \param pstcHandle Pointer to instance
**
** \return Pointer to internal data or NULL if instance is not enabled (or not known)
**
******************************************************************************/
static stc_apollouart_intern_data_t* GetInternDataPtr(UART_Type* pstcHandle)
{
    volatile uint32_t u32Instance;

    for (u32Instance = 0; u32Instance < INSTANCE_COUNT; u32Instance++)
    {
        if ((uint32_t)pstcHandle == (uint32_t)(m_astcInstanceDataLut[u32Instance].pstcInstance))
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
 ** \param  pu8Buffer Pointer to (constant) file of bytes in mem
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
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);

    pstcHandle->bRxEnabled = TRUE;
    pstcHandle->bTxEnabled = TRUE;

    //
    // Enable UART clock in CLKGEN
    //
#if defined(APOLLO_H) || defined(APOLLO1_H)
    CLKGEN->UARTEN_b.UARTEN = 1;        //enable UART clocking
#endif
#if defined(APOLLO2_H)
    if (pstcUart == UART0)
    {
        CLKGEN->UARTEN_b.UART0EN = 1;        //enable UART clocking
        PWRCTRL->DEVICEEN |= (1 << PWRCTRL_DEVICEEN_UART0_Pos);
    }
    if (pstcUart == UART1)
    {
        CLKGEN->UARTEN_b.UART1EN = 1;        //enable UART clocking
        PWRCTRL->DEVICEEN |= (1 << PWRCTRL_DEVICEEN_UART1_Pos);
    }
#endif
#if defined(APOLLO3_H)
    if (pstcUart == UART0)
    {
        PWRCTRL->DEVPWREN |= (1 << PWRCTRL_DEVPWREN_UART0_Pos);
    }
    if (pstcUart == UART1)
    {
        PWRCTRL->DEVPWREN |= (1 << PWRCTRL_DEVPWREN_UART1_Pos);
    }
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
#if defined(APOLLO_H) || defined(APOLLO1_H)
    ConfigureBaudrate(pstcUart,u32Baudrate,SystemCoreClock);
#else
    ConfigureBaudrate(pstcUart,u32Baudrate,24000000UL);
#endif
    // initialize line coding...
    pstcUart->LCRH = 0;
    pstcUart->LCRH_b.WLEN = 3;                //3 = 8 data bits (2..0 = 7..5 data bits)
    pstcUart->LCRH_b.STP2 = 0;                //1 stop bit
    pstcUart->LCRH_b.PEN = 0;                 //no parity


    //
    // Enable UART after config...
    //
    ApolloUart_Enable(pstcUart);

}

#if (APOLLOGPIO_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  Simple init UART by pin and baudrate
 **
 ** \param  u8RxPin          RX pin
 **
 ** \param  u8TxPin          TX pin
 **
 ** \param  u32Baudrate      Baudrate
 **
 ** \param  ppstcUart        returns pointer of the found UART
 **
 ** \return Ok on success and Error on error
 **
 *****************************************************************************/
en_result_t ApolloUart_InitByPin(uint8_t u8RxPin,uint8_t u8TxPin,uint32_t u32Baudrate, UART_Type** ppstcUart)
{
    uint32_t i;
    UART_Type* pstcUart = NULL;
    stc_apollouart_intern_data_t* pstcHandle;
    boolean_t bRxDone = FALSE;
    boolean_t bTxDone = FALSE;
    for(i = 0; i < UARTGPIOS_COUNT;i++)
    {
        if ((pstcUart == NULL) || (stcUartGpios[i].pstcHandle == pstcUart))
        {
            if ((stcUartGpios[i].u8Gpio == u8RxPin) && (stcUartGpios[i].enUartType == ApolloUartGpioTypeRx))
            {
                ApolloGpio_GpioInputEnable(u8RxPin,TRUE);
                ApolloGpio_GpioSelectFunction(u8RxPin,stcUartGpios[i].u8Function);
                pstcUart = stcUartGpios[i].pstcHandle;
                pstcHandle = GetInternDataPtr(pstcUart);
                pstcHandle->stcGpios.i8RxPin = u8RxPin;
                bRxDone = TRUE;
            } else if ((stcUartGpios[i].u8Gpio == u8TxPin) && (stcUartGpios[i].enUartType == ApolloUartGpioTypeTx))
            {
                ApolloGpio_GpioOutputEnable(u8TxPin,TRUE);
                ApolloGpio_GpioSelectFunction(u8TxPin,stcUartGpios[i].u8Function);
                pstcUart = stcUartGpios[i].pstcHandle;
                pstcHandle = GetInternDataPtr(pstcUart);
                pstcHandle->stcGpios.i8TxPin = u8TxPin;
                bTxDone = TRUE;
            }
        }
        if ((bRxDone) && (bTxDone))
        {
            if (ppstcUart != NULL)
            {
                *ppstcUart = pstcUart;
            }
            ApolloUart_Init(pstcUart,u32Baudrate);
            return Ok;
        }
    }
    UART_ASSERT("ApolloUart_InitByPin, no valid GPIO configuration found.\r\n");
    return Error;
}
#endif

/**
 ******************************************************************************
 ** \brief  Init UART with extended settings
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  pstcConfig       Configuration
 **
 *****************************************************************************/
void ApolloUart_InitExtended(UART_Type* pstcUart,stc_apollouart_config_t* pstcConfig)
{
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);

    pstcHandle->u32Baudrate = pstcConfig->u32Baudrate;
    pstcHandle->bRxEnabled = pstcConfig->bEnableRx;
    pstcHandle->bTxEnabled = pstcConfig->bEnableTx;

    if (pstcConfig->bEnableRts)
    {
        pstcHandle->stcGpios.i8RtsPin = pstcConfig->stcGpios.u8RtsPin;
    } else
    {
         pstcHandle->stcGpios.i8RtsPin = -1;
    }

    if (pstcConfig->bEnableCts)
    {
        pstcHandle->stcGpios.i8CtsPin = pstcConfig->stcGpios.u8CtsPin;
    } else
    {
        pstcHandle->stcGpios.i8CtsPin = -1;
    }

    if (pstcConfig->bEnableRx)
    {
        pstcHandle->stcGpios.i8RxPin = pstcConfig->stcGpios.u8RxPin;
    } else
    {
        pstcHandle->stcGpios.i8RxPin = -1;
    }

    if (pstcConfig->bEnableTx)
    {
        pstcHandle->stcGpios.i8TxPin = pstcConfig->stcGpios.u8TxPin;
    } else
    {
        pstcHandle->stcGpios.i8TxPin = -1;
    }

    //
    // Enable UART clock in CLKGEN
    //
#if defined(APOLLO_H) || defined(APOLLO1_H)
    CLKGEN->UARTEN_b.UARTEN = 1;        //enable UART clocking
#endif
#if defined(APOLLO2_H)
    if (pstcUart == UART0)
    {
        CLKGEN->UARTEN_b.UART0EN = 1;        //enable UART clocking
        PWRCTRL->DEVICEEN |= (1 << PWRCTRL_DEVICEEN_UART0_Pos);
    }
    if (pstcUart == UART1)
    {
        CLKGEN->UARTEN_b.UART1EN = 1;        //enable UART clocking
        PWRCTRL->DEVICEEN |= (1 << PWRCTRL_DEVICEEN_UART1_Pos);
    }
#endif
#if defined(APOLLO3_H)
    if (pstcUart == UART0)
    {
        PWRCTRL->DEVPWREN |= (1 << PWRCTRL_DEVPWREN_UART0_Pos);
        //missing documentation, temp fix
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); 
    }
    if (pstcUart == UART1)
    {
        PWRCTRL->DEVPWREN |= (1 << PWRCTRL_DEVPWREN_UART1_Pos);
        //missing documentation, temp fix
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    }
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
    pstcUart->CR_b.UARTEN = 0;                //disable UART
    pstcUart->CR_b.RXE = 0;                   //disable receiver
    pstcUart->CR_b.TXE = 0;                   //disable transmitter


    //
    // Starting UART config...
    //

    // initialize baudrate before all other settings, otherwise UART will not be initialized
    SystemCoreClockUpdate();
#if defined(APOLLO_H) || defined(APOLLO1_H)
    ConfigureBaudrate(pstcUart,pstcConfig->u32Baudrate,SystemCoreClock);
#else
    ConfigureBaudrate(pstcUart,pstcConfig->u32Baudrate,SystemCoreClock/2);
#endif
    // initialize line coding...
    pstcUart->LCRH = 0;

    switch(pstcConfig->enDataLen)
    {
      case ApolloUartWlen5Bit:
        pstcUart->LCRH_b.WLEN = 0;
        break;
      case ApolloUartWlen6Bit:
        pstcUart->LCRH_b.WLEN = 1;
        break;
      case ApolloUartWlen7Bit:
        pstcUart->LCRH_b.WLEN = 2;
        break;
      case ApolloUartWlen8Bit:
        pstcUart->LCRH_b.WLEN = 3;
        break;
    }

    if (pstcConfig->enStopBit == ApolloUartStop2)
    {
        pstcUart->LCRH_b.STP2 = 1;                //2 stop bit
    }
    else
    {
        pstcUart->LCRH_b.STP2 = 0;                //1 stop bit
    }

    switch(pstcConfig->enParity)
    {
      case ApolloUartParityNone:
        pstcUart->LCRH_b.PEN = 0;                 //no parity
        break;
      case ApolloUartParityOdd:
        pstcUart->LCRH_b.EPS = 0;                 //odd parity
        break;
      case ApolloUartParityEven:
        pstcUart->LCRH_b.EPS = 1;                 //even parity
        break;
    }

    pstcUart->LCRH_b.FEN = pstcConfig->bEnableFifo;

    pstcUart->LCRH_b.BRK = pstcConfig->bEnableBreak;

    pstcUart->CR_b.LBE = pstcConfig->bEnableLoopback;

    pstcUart->CR_b.RTSEN = pstcConfig->bEnableRts;

    pstcUart->CR_b.CTSEN = pstcConfig->bEnableCts;

    

    ApolloUart_InitGpios(pstcUart);

    //
    // Enable UART after config...
    //
    ApolloUart_Enable(pstcUart);
};

/**
 ******************************************************************************
 ** \brief  Get UART Status
 **
 ** \param  pstcUart  UART pointer
 **
 ** \return the status
 **
 *****************************************************************************/
stc_apollouart_status_t ApolloUart_GetStatus(UART_Type* pstcUart)
{
    stc_apollouart_status_t stcStat;
    *((uint32_t*)&stcStat) = 0;
    stcStat.bErrorOverflow =  pstcUart->RSR_b.OESTAT;
    stcStat.bErrorBreak =  pstcUart->RSR_b.BESTAT;
    stcStat.bErrorParity = pstcUart->RSR_b.PESTAT;
    stcStat.bErrorFrameing = pstcUart->RSR_b.FESTAT;
    if (pstcUart->FR_b.BUSY != 0) stcStat.bRxBusy = TRUE;
    if ((pstcUart->FR_b.TXFE == 0) || (pstcUart->FR_b.BUSY != 0)) stcStat.bTxBusy = TRUE;
    if (pstcUart->FR_b.RXFE == 0) stcStat.bRxFull = TRUE;
    if (pstcUart->FR_b.TXFF == 0) stcStat.bTxEmpty = TRUE;
    return stcStat;
}


/**
 ******************************************************************************
 ** \brief  Enable UART
 **
 ** \param  pstcUart         UART pointer
 **
 *****************************************************************************/

void ApolloUart_Enable(UART_Type* pstcUart)
{
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    uint32_t i;

    //
    // Enable UART after config...
    //
    pstcUart->CR_b.UARTEN = 1;                    //enable UART

    pstcUart->CR_b.RXE = pstcHandle->bRxEnabled;
    pstcUart->CR_b.TXE = pstcHandle->bTxEnabled;

    if ((pstcHandle->stcGpios.i8RxPin >= 0) && (pstcHandle->stcGpios.i8RxPin < 50))
    {
        ApolloGpio_GpioInputEnable(pstcHandle->stcGpios.i8RxPin,TRUE);
    }

    if ((pstcHandle->stcGpios.i8TxPin >= 0) && (pstcHandle->stcGpios.i8TxPin < 50))
    {
        ApolloGpio_GpioOutputEnable(pstcHandle->stcGpios.i8TxPin,TRUE);
    }
    
}

/**
 ******************************************************************************
 ** \brief  Disable UART
 **
 ** \param  pstcUart         UART pointer
 **
 *****************************************************************************/

void ApolloUart_Disable(UART_Type* pstcUart)
{
    volatile uint32_t u32Timeout = 1000000;
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);

    //
    // Make sure peripheral is not busy
    //
    if (pstcUart->FR_b.BUSY == 1)
    {
        while(u32Timeout > 0)
        {
             u32Timeout--;
             if (pstcUart->FR_b.BUSY == 0)
             {
                 break;
             }
        }
    }
    //
    // Enable UART after config...
    //
    pstcUart->CR_b.RXE = 0;
    pstcUart->CR_b.TXE = 0;
    pstcUart->CR_b.UARTEN = 0;                    //disable UART
    if ((pstcHandle->stcGpios.i8RxPin >= 0) && (pstcHandle->stcGpios.i8RxPin < 50))
    {
        ApolloGpio_GpioInputEnable(pstcHandle->stcGpios.i8RxPin,FALSE);
    }
}

/**
 ******************************************************************************
 ** \brief  Init UART GPIOs
 **
 ** \param  pstcUart         UART pointer
 **
 *****************************************************************************/
void ApolloUart_InitGpios(UART_Type* pstcUart)
{
    uint32_t i;
    stc_apollouart_intern_data_t* pstcInternHandle = GetInternDataPtr(pstcUart);

    for(i = 0; i < UARTGPIOS_COUNT;i++)
    {
        if (stcUartGpios[i].pstcHandle == pstcUart)
        {
            if ((pstcInternHandle->stcGpios.i8CtsPin != -1) && (stcUartGpios[i].enUartType == ApolloUartGpioTypeCts) && (stcUartGpios[i].u8Gpio == pstcInternHandle->stcGpios.i8CtsPin))
            {
                ApolloGpio_GpioSelectFunction(stcUartGpios[i].u8Gpio,stcUartGpios[i].u8Function);
            }
            if ((pstcInternHandle->stcGpios.i8RtsPin != -1) && (stcUartGpios[i].enUartType == ApolloUartGpioTypeRts) && (stcUartGpios[i].u8Gpio == pstcInternHandle->stcGpios.i8RtsPin))
            {
                ApolloGpio_GpioSelectFunction(stcUartGpios[i].u8Gpio,stcUartGpios[i].u8Function);
            }
            if ((pstcInternHandle->stcGpios.i8RxPin != -1) && (stcUartGpios[i].enUartType == ApolloUartGpioTypeRx) && (stcUartGpios[i].u8Gpio == pstcInternHandle->stcGpios.i8RxPin))
            {
                ApolloGpio_GpioInputEnable(pstcInternHandle->stcGpios.i8RxPin,TRUE);
                ApolloGpio_GpioSelectFunction(stcUartGpios[i].u8Gpio,stcUartGpios[i].u8Function);
            }
            if ((pstcInternHandle->stcGpios.i8TxPin != -1) && (stcUartGpios[i].enUartType == ApolloUartGpioTypeTx) && (stcUartGpios[i].u8Gpio == pstcInternHandle->stcGpios.i8TxPin))
            {
                ApolloGpio_GpioOutputEnable(pstcInternHandle->stcGpios.i8TxPin,TRUE);
                ApolloGpio_GpioSelectFunction(stcUartGpios[i].u8Gpio,stcUartGpios[i].u8Function);
            }
        }
    }
}

/**
 ******************************************************************************
 ** \brief  Set RX FIFO interrupt level
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  enLevel          Level
 **
 **
 *****************************************************************************/
void ApolloUart_SetRxFifoIrqLevel(UART_Type* pstcUart,en_apollouart_fifo_irq_level_t enLevel)
{
    pstcUart->IFLS_b.RXIFLSEL = enLevel;
}

/**
 ******************************************************************************
 ** \brief  Set TX FIFO interrupt level
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  enLevel          Level
 **
 **
 *****************************************************************************/
void ApolloUart_SetTxFifoIrqLevel(UART_Type* pstcUart,en_apollouart_fifo_irq_level_t enLevel)
{
    pstcUart->IFLS_b.TXIFLSEL = enLevel;
}

/**
 ******************************************************************************
 ** \brief  Register callback defined in enIrqType
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  enIrqType        Callbacktype to register
 **
 ** \param  cbCallback       Callback
 **
 ** \param  u8Priority       NVIC priority, note this changes priority for all interrupts for this UART handle
 **
 *****************************************************************************/
void ApolloUart_RegisterCallback(UART_Type* pstcUart,en_apollouart_irqtype_t enIrqType,pfn_apollouart_cb_t cbCallback, uint8_t u8Priority)
{
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) 
    {
        UART_ASSERT("ApolloUart_RegisterCallback, no data handle for UART handle found, did you forgot to enable it?\r\n");
        return;
    }
    switch(enIrqType)
    {
        case ApolloUartIrqTypeOverrun:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.OEIM = 1;
            } else
            {
                pstcUart->IER_b.OEIM = 0;
            }
            pstcHandle->cbOverrun = cbCallback;
            break;
        case ApolloUartIrqTypeBreakError:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.BEIM = 1;
            } else
            {
                pstcUart->IER_b.BEIM = 0;
            }
            pstcHandle->cbBreakError = cbCallback;
            break;
        case ApolloUartIrqTypeParityError:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.PEIM = 1;
            } else
            {
                pstcUart->IER_b.PEIM = 0;
            }
            pstcHandle->cbParityError = cbCallback;
            break;
        case ApolloUartIrqTypeFramingError:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.FEIM = 1;
            } else
            {
                pstcUart->IER_b.FEIM = 0;
            }
            pstcHandle->cbFramingError = cbCallback;
            break;
        case ApolloUartIrqTypeRxTimeout:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.RTIM = 1;
            } else
            {
                pstcUart->IER_b.RTIM = 0;
            }
            pstcHandle->cbRxTimeout = cbCallback;
            break;
        case ApolloUartIrqTypeTx:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.TXIM = 1;
            } else
            {
                pstcUart->IER_b.TXIM = 0;
            }
            pstcHandle->cbTx= cbCallback;
            break;
        case ApolloUartIrqTypeRx:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.RXIM = 1;
            } else
            {
                pstcUart->IER_b.RXIM = 0;
            }
            pstcHandle->cbRx= cbCallback;
            break;
        case ApolloUartIrqTypeDsr:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.DSRMIM = 1;
            } else
            {
                pstcUart->IER_b.DSRMIM = 0;
            }
            pstcHandle->cbDsr = cbCallback;
            break;
        case ApolloUartIrqTypeDcd:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.DCDMIM = 1;
            } else
            {
                pstcUart->IER_b.DCDMIM = 0;
            }
            pstcHandle->cbDcd = cbCallback;
            break;
        case ApolloUartIrqTypeCts:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.CTSMIM = 1;
            } else
            {
                pstcUart->IER_b.CTSMIM = 0;
            }
            pstcHandle->cbCts = cbCallback;
            break;
        #if defined (APOLLO_H) || defined (APOLLO1_H) 
        case ApolloUartIrqTypeRi:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.RIMIM = 1;
            } else
            {
                pstcUart->IER_b.RIMIM = 0;
            }
            pstcHandle->cbRi = cbCallback;
            break;
        #endif
        #if defined (APOLLO2_H) || defined (APOLLO3_H) 
        case ApolloUartIrqTypeTxCmp:
            if (cbCallback != NULL)
            {
                pstcUart->IER_b.TXCMPMIM = 1;
            } else
            {
                pstcUart->IER_b.TXCMPMIM = 0;
            }
            pstcHandle->cbTxCmp = cbCallback;
            break;
        #endif
        default:
            return;
    }

    #if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
    if (pstcUart == UART)
    {
        if (pstcUart->IER == 0)
        {
            NVIC_ClearPendingIRQ(UART_IRQn);    //clear pending flag for UART
            NVIC_DisableIRQ(UART_IRQn);         //disable IRQ
        } else
        {
            NVIC_ClearPendingIRQ(UART_IRQn);    //clear pending flag for UART
            NVIC_EnableIRQ(UART_IRQn);          //enable IRQ
            NVIC_SetPriority(UART_IRQn,u8Priority);      //set priority of UART IRQ, smaller value means higher priority
        }
    }
    #endif
    #if defined(UART0) && (APOLLOUART0_ENABLED == 1)
    if (pstcUart == UART0)
    {
        if (pstcUart->IER == 0)
        {
            NVIC_ClearPendingIRQ(UART0_IRQn);    //clear pending flag for UART
            NVIC_DisableIRQ(UART0_IRQn);         //disable IRQ
        } else
        {
            NVIC_ClearPendingIRQ(UART0_IRQn);    //clear pending flag for UART
            NVIC_EnableIRQ(UART0_IRQn);          //enable IRQ
            NVIC_SetPriority(UART0_IRQn,u8Priority);      //set priority of UART IRQ, smaller value means higher priority
        }
    }
    #endif
    #if defined(UART1) && (APOLLOUART1_ENABLED == 1)
    if (pstcUart == UART1)
    {
        if (pstcUart->IER == 0)
        {
            NVIC_ClearPendingIRQ(UART1_IRQn);    //clear pending flag for UART
            NVIC_DisableIRQ(UART1_IRQn);         //disable IRQ
        } else
        {
            NVIC_ClearPendingIRQ(UART1_IRQn);    //clear pending flag for UART
            NVIC_EnableIRQ(UART1_IRQn);          //enable IRQ
            NVIC_SetPriority(UART1_IRQn,u8Priority);      //set priority of UART IRQ, smaller value means higher priority
        }
    }
    #endif
}

/**
 ******************************************************************************
 ** \brief  Simple register callbacks for RX, TX only
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  cbTxNext         Callback retreiving next data
 **
 ** \param  cbRx             Callback after data was received
 **
 ** \param  u8Priority       NVIC priority, note this changes priority for all interrupts for this UART handle
 **
 *****************************************************************************/
void ApolloUart_RegisterCallbacks(UART_Type* pstcUart,pfn_apollouart_txnext_t cbTxNext, pfn_apollouart_rx_t cbRx, uint8_t u8Priority)
{
    stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    if (pstcHandle == NULL) 
    {
        UART_ASSERT("ApolloUart_RegisterCallbacks, no data handle for UART handle found, did you forgot to enable it?\r\n");
        return;
    }
    pstcHandle->cbTxNext = cbTxNext;
    pstcHandle->cbRxSimple = cbRx;
    if ((cbTxNext != NULL) ||  (pstcHandle->cbTx != NULL))
    {
        pstcUart->IER_b.TXIM = 1;
    } else
    {
        pstcUart->IER_b.TXIM = 0;
    }
    if ((cbRx != NULL) || (pstcHandle->cbRx != NULL))
    {
        pstcUart->IER_b.RXIM = 1;
    } else
    {
        pstcUart->IER_b.RXIM = 0;
    }
    #if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
    if (pstcUart == UART)
    {
        if (pstcUart->IER == 0)
        {
            NVIC_ClearPendingIRQ(UART_IRQn);    //clear pending flag for UART
            NVIC_DisableIRQ(UART_IRQn);         //disable IRQ
        } else
        {
            NVIC_ClearPendingIRQ(UART_IRQn);    //clear pending flag for UART
            NVIC_EnableIRQ(UART_IRQn);          //enable IRQ
            NVIC_SetPriority(UART_IRQn,u8Priority);      //set priority of UART IRQ, smaller value means higher priority
        }
    }
    #endif
    #if defined(UART0) && (APOLLOUART0_ENABLED == 1)
    if (pstcUart == UART0)
    {
        if (pstcUart->IER == 0)
        {
            NVIC_ClearPendingIRQ(UART0_IRQn);    //clear pending flag for UART
            NVIC_DisableIRQ(UART0_IRQn);         //disable IRQ
        } else
        {
            NVIC_ClearPendingIRQ(UART0_IRQn);    //clear pending flag for UART
            NVIC_EnableIRQ(UART0_IRQn);          //enable IRQ
            NVIC_SetPriority(UART0_IRQn,u8Priority);      //set priority of UART IRQ, smaller value means higher priority
        }
    }
    #endif
    #if defined(UART1) && (APOLLOUART1_ENABLED == 1)
    if (pstcUart == UART1)
    {
        if (pstcUart->IER == 0)
        {
            NVIC_ClearPendingIRQ(UART1_IRQn);    //clear pending flag for UART
            NVIC_DisableIRQ(UART1_IRQn);         //disable IRQ
        } else
        {
            NVIC_ClearPendingIRQ(UART1_IRQn);    //clear pending flag for UART
            NVIC_EnableIRQ(UART1_IRQn);          //enable IRQ
            NVIC_SetPriority(UART1_IRQn,u8Priority);      //set priority of UART IRQ, smaller value means higher priority
        }
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
    //stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    //if (pstcHandle == NULL) 
    //{
    //    UART_ASSERT("ApolloUart_SendPolled, no data handle for UART handle found, did you forgot to enable it?\r\n");
    //    return;
    //}
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
    //stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    //if (pstcHandle == NULL) 
    //{
    //    UART_ASSERT("ApolloUart_ReceivePolled, no data handle for UART handle found, did you forgot to enable it?\r\n");
    //    return;
    //}
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
    //stc_apollouart_intern_data_t* pstcHandle = GetInternDataPtr(pstcUart);
    //if (pstcHandle == NULL) 
    //{
    //    UART_ASSERT("ApolloUart_TransferPolled, no data handle for UART handle found, did you forgot to enable it?\r\n");
    //    return;
    //}
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
    if (pstcHandle == NULL) 
    {
        UART_ASSERT("ApolloUart_NewTxData, no data handle for UART handle found, did you forgot to enable it?\r\n");
        return;
    }
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
    if (pstcHandle == NULL) 
    {
        UART_ASSERT("ApolloUart_NewTxDataEnabled, no data handle for UART handle found, did you forgot to enable it?\r\n");
        return FALSE;
    }
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
#if defined(UART) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1)) && ((APOLLOUART_IRQ_ENABLED == 1) || (APOLLOUART0_IRQ_ENABLED == 1))
/**
 ******************************************************************************
 ** \brief  IRQ handler UART (Apollo 1)
 **
 *****************************************************************************/
void UART_IRQHandler(void)
{
    ApolloUart_UARTn_IRQHandler(UART);
}
#endif

#if defined(UART0) && (APOLLOUART0_ENABLED == 1) && (APOLLOUART0_IRQ_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  IRQ handler UART0
 **
 *****************************************************************************/
void UART0_IRQHandler(void)
{
    ApolloUart_UARTn_IRQHandler(UART0);
}
#endif

#if defined(UART1) && (APOLLOUART1_ENABLED == 1) && (APOLLOUART1_IRQ_ENABLED == 1)
/**
 ******************************************************************************
 ** \brief  IRQ handler UART1
 **
 *****************************************************************************/
void UART1_IRQHandler(void)
{
    ApolloUart_UARTn_IRQHandler(UART1);
}
#endif

/**
 ******************************************************************************
 ** \brief  IRQ handler depending on UART
 **
 ** \param pstcUart UART Handle
 **
 *****************************************************************************/
void ApolloUart_UARTn_IRQHandler(UART_Type* pstcUart)
{
    int16_t i16ret;
    uint32_t u32Status;
    stc_apollouart_intern_data_t* pstcHandle;
    pstcHandle = GetInternDataPtr(pstcUart);
    u32Status = pstcUart->IES;
    if (pstcUart->IES_b.OERIS)
    {
        if (pstcHandle->cbOverrun != NULL)
        {
            pstcHandle->cbOverrun(pstcUart);
        }
        pstcUart->IEC_b.OEIC = 1;
        u32Status &= ~(UART_IES_OERIS_Msk);
    }

    if (pstcUart->IES_b.BERIS)
    {
        if (pstcHandle->cbBreakError != NULL)
        {
            pstcHandle->cbBreakError(pstcUart);
        }
        pstcUart->IEC_b.BEIC = 1;
        u32Status &= ~(UART_IES_BERIS_Msk);
    }

    if (pstcUart->IES_b.PERIS)
    {
        if (pstcHandle->cbParityError != NULL)
        {
            pstcHandle->cbParityError(pstcUart);
        }
        pstcUart->IEC_b.PEIC = 1;
        u32Status &= ~(UART_IES_PERIS_Msk);
    }

    if (pstcUart->IES_b.FERIS)
    {
        if (pstcHandle->cbFramingError != NULL)
        {
            pstcHandle->cbFramingError(pstcUart);
        }
        pstcUart->IEC_b.FEIC = 1;
        u32Status &= ~(UART_IES_FERIS_Msk);
    }

    if (pstcUart->IES_b.RTRIS)
    {
        if (pstcHandle->cbRxTimeout != NULL)
        {
            pstcHandle->cbRxTimeout(pstcUart);
        }
        pstcUart->IEC_b.RTIC = 1;
        u32Status &= ~(UART_IES_RTRIS_Msk);
    }

    if (pstcUart->IES_b.TXRIS)
    {
        pstcUart->IEC_b.TXIC = 1;
        u32Status &= ~(UART_IES_TXRIS_Msk);
        if (pstcHandle->cbTx != NULL)
        {
            pstcHandle->cbTx(pstcUart);
        }
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

    if (pstcUart->IES_b.RXRIS)
    {
        if (pstcHandle->cbRxSimple != NULL)
        {
            pstcHandle->cbRxSimple(pstcUart->DR);
        }
        if (pstcHandle->cbRx != NULL)
        {
            pstcHandle->cbRx(pstcUart);
        }
        pstcUart->IEC_b.RXIC = 1;
        u32Status &= ~(UART_IES_RXRIS_Msk);
    }

    if (pstcUart->IES_b.DSRMRIS)
    {
        if (pstcHandle->cbDsr != NULL)
        {
            pstcHandle->cbDsr(pstcUart);
        }
        pstcUart->IEC_b.DSRMIC= 1;
        u32Status &= ~(UART_IES_DSRMRIS_Msk);
    }

    if (pstcUart->IES_b.DCDMRIS)
    {
        if (pstcHandle->cbDcd != NULL)
        {
            pstcHandle->cbDcd(pstcUart);
        }
        pstcUart->IEC_b.DCDMIC= 1;
        u32Status &= ~(UART_IES_DCDMRIS_Msk);
    }

    if (pstcUart->IES_b.CTSMRIS)
    {
        if (pstcHandle->cbCts != NULL)
        {
            pstcHandle->cbCts(pstcUart);
        }
        pstcUart->IEC_b.CTSMIC= 1;
        u32Status &= ~(UART_IES_CTSMRIS_Msk);
    }
    if (pstcUart->IES & 0x1)
    {
        #if defined (APOLLO_H) || defined (APOLLO1_H) 
            if (pstcHandle->cbRi != NULL)
            {
                pstcHandle->cbRi(pstcUart);
            }
        #else
            if (pstcHandle->cbTxCmp != NULL)
            {
                pstcHandle->cbTxCmp(pstcUart);
            }
        #endif
        pstcUart->IEC = 0x1;
        u32Status &= ~(0x1);
    }

    //clear all other interrupts
    pstcUart->IEC = u32Status;
}

#if UART_SEMIHOST_ENABLED == 1

#if !defined(UART_SEMIHOST)
#error please define UART_SEMIHOST in RTE_Device.h
#endif

#if  defined(__CC_ARM)
/**
 ******************************************************************************
 ** \brief Low Level for stdio for ARM / Keil µVision
 **
 ******************************************************************************/

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return 0;
}

void _ttywrch(int c) {
    ApolloUart_PutChar(UART_SEMIHOST,(char_t)c);
}

void _sys_exit(int return_code) {
    while(1);  /* endless loop */
}

int fputc(int c, FILE *f) {
    ApolloUart_PutChar(UART_SEMIHOST,(char_t)c);
    return 0;
}


int fgetc(FILE *f) {
    char_t c;
    c = ApolloUart_GetChar(UART_SEMIHOST);
    return (c);
}

#elif defined(__ICCARM__)
/**
 ******************************************************************************
 ** \brief Low Level for stdio for IAR EWARM
 **
 ******************************************************************************/

int putchar(int ch)
{
   ApolloUart_PutChar(UART_SEMIHOST,(char_t)ch);
   return ch;
}

int __close(int fileno)
{
    return 0;
}
int __write(int fileno, char *buf, unsigned int size)
{
     ApolloUart_SendPolled(UART_SEMIHOST,(uint8_t*)buf,size);
    return 0;
}
int __read(int fileno, char *buf, unsigned int size)
{
    ApolloUart_ReceivePolled(UART_SEMIHOST,(uint8_t*)buf,size);
    return 0;
}

#elif defined(__GNUC__)

/**
 ******************************************************************************
 ** \brief Low Level for stdio for GNU GCC
 **
 ******************************************************************************/

int _read_r (struct _reent *r, int file, char * ptr, int len)
{
  r = r;
  file = file;
  ApolloUart_ReceivePolled(UART_SEMIHOST,(uint8_t*)ptr,len);
  return 0;
}


int _lseek_r (struct _reent *r, int file, int ptr, int dir)
{
  r = r;
  file = file;
  ptr = ptr;
  dir = dir;
  return 0;
}


int _write (int file, char * ptr, int len)
{
  file = file;
  ptr = ptr;
  ApolloUart_SendPolled(UART_SEMIHOST,(uint8_t*)ptr,(uint32_t)len);

  return 0;

}

int _close_r (struct _reent *r, int file)
{
  return 0;
}

/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr asm ("sp");

caddr_t _sbrk_r (struct _reent *r, int incr)
{
  extern char   end asm ("end"); /* Defined by the linker.  */
  static char * heap_end;
  char *        prev_heap_end;

  if (heap_end == NULL)
    heap_end = & end;

  prev_heap_end = heap_end;

  if (heap_end + incr > stack_ptr)
  {
      /* Some of the libstdc++-v3 tests rely upon detecting
        out of memory errors, so do not abort here.  */
#if 0
      extern void abort (void);

      _write (1, "_sbrk: Heap and stack collision\n", 32);

      abort ();
#else
      return (caddr_t) -1;
#endif
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

#endif

#if    HEAP_SIZE
extern  char   *sbrk(int size)
{
   if (brk_siz + size > _heap_size || brk_siz + size < 0)
        return((char*)-1);
   brk_siz += size;
   return( (char *)_heap + brk_siz - size);
}
#endif


#if    HEAP_SIZE
    static   long        brk_siz = 0;
    typedef  int         _heap_t;
    #define ROUNDUP(s)   (((s)+sizeof(_heap_t)-1)&~(sizeof(_heap_t)-1))
    static   _heap_t     _heap[ROUNDUP(HEAP_SIZE)/sizeof(_heap_t)];
    #define              _heap_size       ROUNDUP(HEAP_SIZE)
#else
    extern  char         *_heap;
    extern  long         _heap_size;
#endif

#endif
/**
 ******************************************************************************
 ** \brief Deinitialization Routine
 **
 ******************************************************************************/
void Uart_Deinit(void)
{

}
#else
#warning Low-Level-Driver for Apollo 1/2 UART is disabled and could be removed from the project
#endif //(APOLLOUART_ENABLED == 1) || (APOLLOUART0_ENABLED == 1) || (APOLLOUART1_ENABLED == 1)

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

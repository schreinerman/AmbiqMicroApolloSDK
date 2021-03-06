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
 **   - 2019-03-13  V2.0b Manuel Schreiner   Added proper internal data initialization
 **   - 2019-03-25  V2.1  Manuel Schreiner   Added __APOLLOUART_VERSION__ and __APOLLOUART_DATE__ defines
 **
 *****************************************************************************/

#ifndef __APOLLOUART_H__
#define __APOLLOUART_H__
#define __APOLLOUART_VERSION__  21
#define __APOLLOUART_DATE__     "2019-03-25"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup ApolloUartGroup  Low-Level-Driver for Apollo 1/2 UART
 **
 ** Provided functions of UART routines for Apollo:
 ** 
 ** - ApolloUart_Init()               - simple init UART by baudrate
 ** - ApolloUart_InitExtended()       - init UART with extended settings
 ** - ApolloUart_InitByPin()          - init UART by GPIO pin and returns UART handle
 ** - ApolloUart_InitGpios()          - init UART GPIOs (done by init automatically)
 ** - ApolloUart_Enable()             - enable UART (done by init automatically)
 ** - ApolloUart_Disable()            - disable UART
 ** - ApolloUart_PutChar()            - put char
 ** - ApolloUart_PutString()          - put string
 ** - ApolloUart_GetChar()            - get char
 ** - ApolloUart_HasChar()            - check UART has data
 ** - ApolloUart_RegisterCallbacks()  - register callback
 ** - ApolloUart_NewTxData()          - get data via cbTxNext() callback and initiate sending data
 ** - ApolloUart_NewTxDataEnabled()   - check TX IRQ is enabled
 ** - ApolloUart_Deinit()             - deinit UART
 ** - ApolloUart_SendPolled()         - send buffer polled
 ** - ApolloUart_ReceivePolled()      - receive buffer polled
 ** - ApolloUart_TransferPolled()     - send/received data polled
 ** - ApolloUart_UARTn_IRQHandler()   - execute IRQ handling from OS 
 **
 ** Following functions calling ApolloUart_Enable() before transfer and ApolloUart_Disable() after transfer:
 **
 ** - ApolloUart_LowPowerPutString()          - put string
 ** - ApolloUart_LowPowerSendPolled()         - send buffer polled
 ** - ApolloUart_LowPowerReceivePolled()      - receive buffer polled
 ** - ApolloUart_LowPowerTransferPolled()     - send/received data polled 
 **   
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page apollouart_module_includes Required includes in main application
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
    
#if defined(USE_CMSIS_DRIVER)
#include "Driver_USART.h"
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#if defined(UART0) && (!defined(UART))

/* ==========================================================  DR  =========================================================== */
#define UART_DR_OEDATA_Pos               (11UL)                    /*!< UART DR: OEDATA (Bit 11)                             */
#define UART_DR_OEDATA_Msk               (0x800UL)                 /*!< UART DR: OEDATA (Bitfield-Mask: 0x01)                */
#define UART_DR_BEDATA_Pos               (10UL)                    /*!< UART DR: BEDATA (Bit 10)                             */
#define UART_DR_BEDATA_Msk               (0x400UL)                 /*!< UART DR: BEDATA (Bitfield-Mask: 0x01)                */
#define UART_DR_PEDATA_Pos               (9UL)                     /*!< UART DR: PEDATA (Bit 9)                              */
#define UART_DR_PEDATA_Msk               (0x200UL)                 /*!< UART DR: PEDATA (Bitfield-Mask: 0x01)                */
#define UART_DR_FEDATA_Pos               (8UL)                     /*!< UART DR: FEDATA (Bit 8)                              */
#define UART_DR_FEDATA_Msk               (0x100UL)                 /*!< UART DR: FEDATA (Bitfield-Mask: 0x01)                */
#define UART_DR_DATA_Pos                 (0UL)                     /*!< UART DR: DATA (Bit 0)                                */
#define UART_DR_DATA_Msk                 (0xffUL)                  /*!< UART DR: DATA (Bitfield-Mask: 0xff)                  */
/* ==========================================================  RSR  ========================================================== */
#define UART_RSR_OESTAT_Pos              (3UL)                     /*!< UART RSR: OESTAT (Bit 3)                             */
#define UART_RSR_OESTAT_Msk              (0x8UL)                   /*!< UART RSR: OESTAT (Bitfield-Mask: 0x01)               */
#define UART_RSR_BESTAT_Pos              (2UL)                     /*!< UART RSR: BESTAT (Bit 2)                             */
#define UART_RSR_BESTAT_Msk              (0x4UL)                   /*!< UART RSR: BESTAT (Bitfield-Mask: 0x01)               */
#define UART_RSR_PESTAT_Pos              (1UL)                     /*!< UART RSR: PESTAT (Bit 1)                             */
#define UART_RSR_PESTAT_Msk              (0x2UL)                   /*!< UART RSR: PESTAT (Bitfield-Mask: 0x01)               */
#define UART_RSR_FESTAT_Pos              (0UL)                     /*!< UART RSR: FESTAT (Bit 0)                             */
#define UART_RSR_FESTAT_Msk              (0x1UL)                   /*!< UART RSR: FESTAT (Bitfield-Mask: 0x01)               */
/* ==========================================================  FR  =========================================================== */
#define UART_FR_TXBUSY_Pos               (8UL)                     /*!< UART FR: TXBUSY (Bit 8)                              */
#define UART_FR_TXBUSY_Msk               (0x100UL)                 /*!< UART FR: TXBUSY (Bitfield-Mask: 0x01)                */
#define UART_FR_TXFE_Pos                 (7UL)                     /*!< UART FR: TXFE (Bit 7)                                */
#define UART_FR_TXFE_Msk                 (0x80UL)                  /*!< UART FR: TXFE (Bitfield-Mask: 0x01)                  */
#define UART_FR_RXFF_Pos                 (6UL)                     /*!< UART FR: RXFF (Bit 6)                                */
#define UART_FR_RXFF_Msk                 (0x40UL)                  /*!< UART FR: RXFF (Bitfield-Mask: 0x01)                  */
#define UART_FR_TXFF_Pos                 (5UL)                     /*!< UART FR: TXFF (Bit 5)                                */
#define UART_FR_TXFF_Msk                 (0x20UL)                  /*!< UART FR: TXFF (Bitfield-Mask: 0x01)                  */
#define UART_FR_RXFE_Pos                 (4UL)                     /*!< UART FR: RXFE (Bit 4)                                */
#define UART_FR_RXFE_Msk                 (0x10UL)                  /*!< UART FR: RXFE (Bitfield-Mask: 0x01)                  */
#define UART_FR_BUSY_Pos                 (3UL)                     /*!< UART FR: BUSY (Bit 3)                                */
#define UART_FR_BUSY_Msk                 (0x8UL)                   /*!< UART FR: BUSY (Bitfield-Mask: 0x01)                  */
#define UART_FR_DCD_Pos                  (2UL)                     /*!< UART FR: DCD (Bit 2)                                 */
#define UART_FR_DCD_Msk                  (0x4UL)                   /*!< UART FR: DCD (Bitfield-Mask: 0x01)                   */
#define UART_FR_DSR_Pos                  (1UL)                     /*!< UART FR: DSR (Bit 1)                                 */
#define UART_FR_DSR_Msk                  (0x2UL)                   /*!< UART FR: DSR (Bitfield-Mask: 0x01)                   */
#define UART_FR_CTS_Pos                  (0UL)                     /*!< UART FR: CTS (Bit 0)                                 */
#define UART_FR_CTS_Msk                  (0x1UL)                   /*!< UART FR: CTS (Bitfield-Mask: 0x01)                   */
/* =========================================================  ILPR  ========================================================== */
#define UART_ILPR_ILPDVSR_Pos            (0UL)                     /*!< UART ILPR: ILPDVSR (Bit 0)                           */
#define UART_ILPR_ILPDVSR_Msk            (0xffUL)                  /*!< UART ILPR: ILPDVSR (Bitfield-Mask: 0xff)             */
/* =========================================================  IBRD  ========================================================== */
#define UART_IBRD_DIVINT_Pos             (0UL)                     /*!< UART IBRD: DIVINT (Bit 0)                            */
#define UART_IBRD_DIVINT_Msk             (0xffffUL)                /*!< UART IBRD: DIVINT (Bitfield-Mask: 0xffff)            */
/* =========================================================  FBRD  ========================================================== */
#define UART_FBRD_DIVFRAC_Pos            (0UL)                     /*!< UART FBRD: DIVFRAC (Bit 0)                           */
#define UART_FBRD_DIVFRAC_Msk            (0x3fUL)                  /*!< UART FBRD: DIVFRAC (Bitfield-Mask: 0x3f)             */
/* =========================================================  LCRH  ========================================================== */
#define UART_LCRH_SPS_Pos                (7UL)                     /*!< UART LCRH: SPS (Bit 7)                               */
#define UART_LCRH_SPS_Msk                (0x80UL)                  /*!< UART LCRH: SPS (Bitfield-Mask: 0x01)                 */
#define UART_LCRH_WLEN_Pos               (5UL)                     /*!< UART LCRH: WLEN (Bit 5)                              */
#define UART_LCRH_WLEN_Msk               (0x60UL)                  /*!< UART LCRH: WLEN (Bitfield-Mask: 0x03)                */
#define UART_LCRH_FEN_Pos                (4UL)                     /*!< UART LCRH: FEN (Bit 4)                               */
#define UART_LCRH_FEN_Msk                (0x10UL)                  /*!< UART LCRH: FEN (Bitfield-Mask: 0x01)                 */
#define UART_LCRH_STP2_Pos               (3UL)                     /*!< UART LCRH: STP2 (Bit 3)                              */
#define UART_LCRH_STP2_Msk               (0x8UL)                   /*!< UART LCRH: STP2 (Bitfield-Mask: 0x01)                */
#define UART_LCRH_EPS_Pos                (2UL)                     /*!< UART LCRH: EPS (Bit 2)                               */
#define UART_LCRH_EPS_Msk                (0x4UL)                   /*!< UART LCRH: EPS (Bitfield-Mask: 0x01)                 */
#define UART_LCRH_PEN_Pos                (1UL)                     /*!< UART LCRH: PEN (Bit 1)                               */
#define UART_LCRH_PEN_Msk                (0x2UL)                   /*!< UART LCRH: PEN (Bitfield-Mask: 0x01)                 */
#define UART_LCRH_BRK_Pos                (0UL)                     /*!< UART LCRH: BRK (Bit 0)                               */
#define UART_LCRH_BRK_Msk                (0x1UL)                   /*!< UART LCRH: BRK (Bitfield-Mask: 0x01)                 */
/* ==========================================================  CR  =========================================================== */
#define UART_CR_CTSEN_Pos                (15UL)                    /*!< UART CR: CTSEN (Bit 15)                              */
#define UART_CR_CTSEN_Msk                (0x8000UL)                /*!< UART CR: CTSEN (Bitfield-Mask: 0x01)                 */
#define UART_CR_RTSEN_Pos                (14UL)                    /*!< UART CR: RTSEN (Bit 14)                              */
#define UART_CR_RTSEN_Msk                (0x4000UL)                /*!< UART CR: RTSEN (Bitfield-Mask: 0x01)                 */
#define UART_CR_OUT2_Pos                 (13UL)                    /*!< UART CR: OUT2 (Bit 13)                               */
#define UART_CR_OUT2_Msk                 (0x2000UL)                /*!< UART CR: OUT2 (Bitfield-Mask: 0x01)                  */
#define UART_CR_OUT1_Pos                 (12UL)                    /*!< UART CR: OUT1 (Bit 12)                               */
#define UART_CR_OUT1_Msk                 (0x1000UL)                /*!< UART CR: OUT1 (Bitfield-Mask: 0x01)                  */
#define UART_CR_RTS_Pos                  (11UL)                    /*!< UART CR: RTS (Bit 11)                                */
#define UART_CR_RTS_Msk                  (0x800UL)                 /*!< UART CR: RTS (Bitfield-Mask: 0x01)                   */
#define UART_CR_DTR_Pos                  (10UL)                    /*!< UART CR: DTR (Bit 10)                                */
#define UART_CR_DTR_Msk                  (0x400UL)                 /*!< UART CR: DTR (Bitfield-Mask: 0x01)                   */
#define UART_CR_RXE_Pos                  (9UL)                     /*!< UART CR: RXE (Bit 9)                                 */
#define UART_CR_RXE_Msk                  (0x200UL)                 /*!< UART CR: RXE (Bitfield-Mask: 0x01)                   */
#define UART_CR_TXE_Pos                  (8UL)                     /*!< UART CR: TXE (Bit 8)                                 */
#define UART_CR_TXE_Msk                  (0x100UL)                 /*!< UART CR: TXE (Bitfield-Mask: 0x01)                   */
#define UART_CR_LBE_Pos                  (7UL)                     /*!< UART CR: LBE (Bit 7)                                 */
#define UART_CR_LBE_Msk                  (0x80UL)                  /*!< UART CR: LBE (Bitfield-Mask: 0x01)                   */
#define UART_CR_CLKSEL_Pos               (4UL)                     /*!< UART CR: CLKSEL (Bit 4)                              */
#define UART_CR_CLKSEL_Msk               (0x70UL)                  /*!< UART CR: CLKSEL (Bitfield-Mask: 0x07)                */
#define UART_CR_CLKEN_Pos                (3UL)                     /*!< UART CR: CLKEN (Bit 3)                               */
#define UART_CR_CLKEN_Msk                (0x8UL)                   /*!< UART CR: CLKEN (Bitfield-Mask: 0x01)                 */
#define UART_CR_SIRLP_Pos                (2UL)                     /*!< UART CR: SIRLP (Bit 2)                               */
#define UART_CR_SIRLP_Msk                (0x4UL)                   /*!< UART CR: SIRLP (Bitfield-Mask: 0x01)                 */
#define UART_CR_SIREN_Pos                (1UL)                     /*!< UART CR: SIREN (Bit 1)                               */
#define UART_CR_SIREN_Msk                (0x2UL)                   /*!< UART CR: SIREN (Bitfield-Mask: 0x01)                 */
#define UART_CR_UARTEN_Pos               (0UL)                     /*!< UART CR: UARTEN (Bit 0)                              */
#define UART_CR_UARTEN_Msk               (0x1UL)                   /*!< UART CR: UARTEN (Bitfield-Mask: 0x01)                */
/* =========================================================  IFLS  ========================================================== */
#define UART_IFLS_RXIFLSEL_Pos           (3UL)                     /*!< UART IFLS: RXIFLSEL (Bit 3)                          */
#define UART_IFLS_RXIFLSEL_Msk           (0x38UL)                  /*!< UART IFLS: RXIFLSEL (Bitfield-Mask: 0x07)            */
#define UART_IFLS_TXIFLSEL_Pos           (0UL)                     /*!< UART IFLS: TXIFLSEL (Bit 0)                          */
#define UART_IFLS_TXIFLSEL_Msk           (0x7UL)                   /*!< UART IFLS: TXIFLSEL (Bitfield-Mask: 0x07)            */
/* ==========================================================  IER  ========================================================== */
#define UART_IER_OEIM_Pos                (10UL)                    /*!< UART IER: OEIM (Bit 10)                              */
#define UART_IER_OEIM_Msk                (0x400UL)                 /*!< UART IER: OEIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_BEIM_Pos                (9UL)                     /*!< UART IER: BEIM (Bit 9)                               */
#define UART_IER_BEIM_Msk                (0x200UL)                 /*!< UART IER: BEIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_PEIM_Pos                (8UL)                     /*!< UART IER: PEIM (Bit 8)                               */
#define UART_IER_PEIM_Msk                (0x100UL)                 /*!< UART IER: PEIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_FEIM_Pos                (7UL)                     /*!< UART IER: FEIM (Bit 7)                               */
#define UART_IER_FEIM_Msk                (0x80UL)                  /*!< UART IER: FEIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_RTIM_Pos                (6UL)                     /*!< UART IER: RTIM (Bit 6)                               */
#define UART_IER_RTIM_Msk                (0x40UL)                  /*!< UART IER: RTIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_TXIM_Pos                (5UL)                     /*!< UART IER: TXIM (Bit 5)                               */
#define UART_IER_TXIM_Msk                (0x20UL)                  /*!< UART IER: TXIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_RXIM_Pos                (4UL)                     /*!< UART IER: RXIM (Bit 4)                               */
#define UART_IER_RXIM_Msk                (0x10UL)                  /*!< UART IER: RXIM (Bitfield-Mask: 0x01)                 */
#define UART_IER_DSRMIM_Pos              (3UL)                     /*!< UART IER: DSRMIM (Bit 3)                             */
#define UART_IER_DSRMIM_Msk              (0x8UL)                   /*!< UART IER: DSRMIM (Bitfield-Mask: 0x01)               */
#define UART_IER_DCDMIM_Pos              (2UL)                     /*!< UART IER: DCDMIM (Bit 2)                             */
#define UART_IER_DCDMIM_Msk              (0x4UL)                   /*!< UART IER: DCDMIM (Bitfield-Mask: 0x01)               */
#define UART_IER_CTSMIM_Pos              (1UL)                     /*!< UART IER: CTSMIM (Bit 1)                             */
#define UART_IER_CTSMIM_Msk              (0x2UL)                   /*!< UART IER: CTSMIM (Bitfield-Mask: 0x01)               */
#define UART_IER_TXCMPMIM_Pos            (0UL)                     /*!< UART IER: TXCMPMIM (Bit 0)                           */
#define UART_IER_TXCMPMIM_Msk            (0x1UL)                   /*!< UART IER: TXCMPMIM (Bitfield-Mask: 0x01)             */
/* ==========================================================  IES  ========================================================== */
#define UART_IES_OERIS_Pos               (10UL)                    /*!< UART IES: OERIS (Bit 10)                             */
#define UART_IES_OERIS_Msk               (0x400UL)                 /*!< UART IES: OERIS (Bitfield-Mask: 0x01)                */
#define UART_IES_BERIS_Pos               (9UL)                     /*!< UART IES: BERIS (Bit 9)                              */
#define UART_IES_BERIS_Msk               (0x200UL)                 /*!< UART IES: BERIS (Bitfield-Mask: 0x01)                */
#define UART_IES_PERIS_Pos               (8UL)                     /*!< UART IES: PERIS (Bit 8)                              */
#define UART_IES_PERIS_Msk               (0x100UL)                 /*!< UART IES: PERIS (Bitfield-Mask: 0x01)                */
#define UART_IES_FERIS_Pos               (7UL)                     /*!< UART IES: FERIS (Bit 7)                              */
#define UART_IES_FERIS_Msk               (0x80UL)                  /*!< UART IES: FERIS (Bitfield-Mask: 0x01)                */
#define UART_IES_RTRIS_Pos               (6UL)                     /*!< UART IES: RTRIS (Bit 6)                              */
#define UART_IES_RTRIS_Msk               (0x40UL)                  /*!< UART IES: RTRIS (Bitfield-Mask: 0x01)                */
#define UART_IES_TXRIS_Pos               (5UL)                     /*!< UART IES: TXRIS (Bit 5)                              */
#define UART_IES_TXRIS_Msk               (0x20UL)                  /*!< UART IES: TXRIS (Bitfield-Mask: 0x01)                */
#define UART_IES_RXRIS_Pos               (4UL)                     /*!< UART IES: RXRIS (Bit 4)                              */
#define UART_IES_RXRIS_Msk               (0x10UL)                  /*!< UART IES: RXRIS (Bitfield-Mask: 0x01)                */
#define UART_IES_DSRMRIS_Pos             (3UL)                     /*!< UART IES: DSRMRIS (Bit 3)                            */
#define UART_IES_DSRMRIS_Msk             (0x8UL)                   /*!< UART IES: DSRMRIS (Bitfield-Mask: 0x01)              */
#define UART_IES_DCDMRIS_Pos             (2UL)                     /*!< UART IES: DCDMRIS (Bit 2)                            */
#define UART_IES_DCDMRIS_Msk             (0x4UL)                   /*!< UART IES: DCDMRIS (Bitfield-Mask: 0x01)              */
#define UART_IES_CTSMRIS_Pos             (1UL)                     /*!< UART IES: CTSMRIS (Bit 1)                            */
#define UART_IES_CTSMRIS_Msk             (0x2UL)                   /*!< UART IES: CTSMRIS (Bitfield-Mask: 0x01)              */
#define UART_IES_TXCMPMRIS_Pos           (0UL)                     /*!< UART IES: TXCMPMRIS (Bit 0)                          */
#define UART_IES_TXCMPMRIS_Msk           (0x1UL)                   /*!< UART IES: TXCMPMRIS (Bitfield-Mask: 0x01)            */
/* ==========================================================  MIS  ========================================================== */
#define UART_MIS_OEMIS_Pos               (10UL)                    /*!< UART MIS: OEMIS (Bit 10)                             */
#define UART_MIS_OEMIS_Msk               (0x400UL)                 /*!< UART MIS: OEMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_BEMIS_Pos               (9UL)                     /*!< UART MIS: BEMIS (Bit 9)                              */
#define UART_MIS_BEMIS_Msk               (0x200UL)                 /*!< UART MIS: BEMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_PEMIS_Pos               (8UL)                     /*!< UART MIS: PEMIS (Bit 8)                              */
#define UART_MIS_PEMIS_Msk               (0x100UL)                 /*!< UART MIS: PEMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_FEMIS_Pos               (7UL)                     /*!< UART MIS: FEMIS (Bit 7)                              */
#define UART_MIS_FEMIS_Msk               (0x80UL)                  /*!< UART MIS: FEMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_RTMIS_Pos               (6UL)                     /*!< UART MIS: RTMIS (Bit 6)                              */
#define UART_MIS_RTMIS_Msk               (0x40UL)                  /*!< UART MIS: RTMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_TXMIS_Pos               (5UL)                     /*!< UART MIS: TXMIS (Bit 5)                              */
#define UART_MIS_TXMIS_Msk               (0x20UL)                  /*!< UART MIS: TXMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_RXMIS_Pos               (4UL)                     /*!< UART MIS: RXMIS (Bit 4)                              */
#define UART_MIS_RXMIS_Msk               (0x10UL)                  /*!< UART MIS: RXMIS (Bitfield-Mask: 0x01)                */
#define UART_MIS_DSRMMIS_Pos             (3UL)                     /*!< UART MIS: DSRMMIS (Bit 3)                            */
#define UART_MIS_DSRMMIS_Msk             (0x8UL)                   /*!< UART MIS: DSRMMIS (Bitfield-Mask: 0x01)              */
#define UART_MIS_DCDMMIS_Pos             (2UL)                     /*!< UART MIS: DCDMMIS (Bit 2)                            */
#define UART_MIS_DCDMMIS_Msk             (0x4UL)                   /*!< UART MIS: DCDMMIS (Bitfield-Mask: 0x01)              */
#define UART_MIS_CTSMMIS_Pos             (1UL)                     /*!< UART MIS: CTSMMIS (Bit 1)                            */
#define UART_MIS_CTSMMIS_Msk             (0x2UL)                   /*!< UART MIS: CTSMMIS (Bitfield-Mask: 0x01)              */
#define UART_MIS_TXCMPMMIS_Pos           (0UL)                     /*!< UART MIS: TXCMPMMIS (Bit 0)                          */
#define UART_MIS_TXCMPMMIS_Msk           (0x1UL)                   /*!< UART MIS: TXCMPMMIS (Bitfield-Mask: 0x01)            */
/* ==========================================================  IEC  ========================================================== */
#define UART_IEC_OEIC_Pos                (10UL)                    /*!< UART IEC: OEIC (Bit 10)                              */
#define UART_IEC_OEIC_Msk                (0x400UL)                 /*!< UART IEC: OEIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_BEIC_Pos                (9UL)                     /*!< UART IEC: BEIC (Bit 9)                               */
#define UART_IEC_BEIC_Msk                (0x200UL)                 /*!< UART IEC: BEIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_PEIC_Pos                (8UL)                     /*!< UART IEC: PEIC (Bit 8)                               */
#define UART_IEC_PEIC_Msk                (0x100UL)                 /*!< UART IEC: PEIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_FEIC_Pos                (7UL)                     /*!< UART IEC: FEIC (Bit 7)                               */
#define UART_IEC_FEIC_Msk                (0x80UL)                  /*!< UART IEC: FEIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_RTIC_Pos                (6UL)                     /*!< UART IEC: RTIC (Bit 6)                               */
#define UART_IEC_RTIC_Msk                (0x40UL)                  /*!< UART IEC: RTIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_TXIC_Pos                (5UL)                     /*!< UART IEC: TXIC (Bit 5)                               */
#define UART_IEC_TXIC_Msk                (0x20UL)                  /*!< UART IEC: TXIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_RXIC_Pos                (4UL)                     /*!< UART IEC: RXIC (Bit 4)                               */
#define UART_IEC_RXIC_Msk                (0x10UL)                  /*!< UART IEC: RXIC (Bitfield-Mask: 0x01)                 */
#define UART_IEC_DSRMIC_Pos              (3UL)                     /*!< UART IEC: DSRMIC (Bit 3)                             */
#define UART_IEC_DSRMIC_Msk              (0x8UL)                   /*!< UART IEC: DSRMIC (Bitfield-Mask: 0x01)               */
#define UART_IEC_DCDMIC_Pos              (2UL)                     /*!< UART IEC: DCDMIC (Bit 2)                             */
#define UART_IEC_DCDMIC_Msk              (0x4UL)                   /*!< UART IEC: DCDMIC (Bitfield-Mask: 0x01)               */
#define UART_IEC_CTSMIC_Pos              (1UL)                     /*!< UART IEC: CTSMIC (Bit 1)                             */
#define UART_IEC_CTSMIC_Msk              (0x2UL)                   /*!< UART IEC: CTSMIC (Bitfield-Mask: 0x01)               */
#define UART_IEC_TXCMPMIC_Pos            (0UL)                     /*!< UART IEC: TXCMPMIC (Bit 0)                           */
#define UART_IEC_TXCMPMIC_Msk            (0x1UL)                   /*!< UART IEC: TXCMPMIC (Bitfield-Mask: 0x01)             */
     
#endif

/**
 ******************************************************************************
 ** \brief  Select GPIO function
 **
 ** \param  uart_gpio        name, for example UART0TX_GPIO1, UART0RX_GPIO2, 
 **                          etc. (depending on Apollo1/2/3)
 **
 **
 *****************************************************************************/  
#define APOLLOUART_GPIO_FNCSEL(uart_gpio) ApolloGpio_GpioSelectFunction((uart_gpio),(uart_gpio ## _FNCSEL))

#if defined(APOLLO_H) || defined(APOLLO1_H)

//Table 239 in Appollo1 datasheet
#define UARTTX_GPIO0          0
#define UARTTX_GPIO0_FNCSEL   2
#define UARTTX_GPIO14        14
#define UARTTX_GPIO14_FNCSEL  2
#define UARTTX_GPIO22        22
#define UARTTX_GPIO22_FNCSEL  0
#define UARTTX_GPIO35        35
#define UARTTX_GPIO35_FNCSEL  2
#define UARTTX_GPIO39        39
#define UARTTX_GPIO39_FNCSEL  1

//Table 240 in Appollo1 datasheet
#define UARTRX_GPIO1          1
#define UARTRX_GPIO1_FNCSEL   2
#define UARTRX_GPIO15        15
#define UARTRX_GPIO15_FNCSEL  2
#define UARTRX_GPIO23        23
#define UARTRX_GPIO23_FNCSEL  0
#define UARTRX_GPIO36        36
#define UARTRX_GPIO36_FNCSEL  2
#define UARTRX_GPIO40        40
#define UARTRX_GPIO40_FNCSEL  1

//Table 241 in Appollo1 datasheet
#define UARTRTS_GPIO5          5
#define UARTRTS_GPIO5_FNCSEL   2
#define UARTRTS_GPIO37        37
#define UARTRTS_GPIO37_FNCSEL  2

//Table 242 in Appollo1 datasheet
#define UARTCTS_GPIO6          6
#define UARTCTS_GPIO6_FNCSEL   2
#define UARTCTS_GPIO38        38
#define UARTCTS_GPIO38_FNCSEL  2

#elif defined(APOLLO2_H)

//Table 349 in Appollo2 datasheet
#define UART0TX_GPIO1         1
#define UART0TX_GPIO1_FNCSEL  2
#define UART0TX_GPIO7         7
#define UART0TX_GPIO7_FNCSEL  5
#define UART0TX_GPIO16       16
#define UART0TX_GPIO16_FNCSEL 6
#define UART0TX_GPIO20       20
#define UART0TX_GPIO20_FNCSEL 4
#define UART0TX_GPIO22       22
#define UART0TX_GPIO22_FNCSEL 0
#define UART0TX_GPIO30       30
#define UART0TX_GPIO30_FNCSEL 4
#define UART0TX_GPIO39       39
#define UART0TX_GPIO39_FNCSEL 0

//Table 350 in Appollo2 datasheet
#define UART0RX_GPIO2         2
#define UART0RX_GPIO2_FNCSEL  2
#define UART0RX_GPIO11       11
#define UART0RX_GPIO11_FNCSEL 6
#define UART0RX_GPIO17       17
#define UART0RX_GPIO17_FNCSEL 6
#define UART0RX_GPIO21       21
#define UART0RX_GPIO21_FNCSEL 4
#define UART0RX_GPIO23       23
#define UART0RX_GPIO23_FNCSEL 0
#define UART0RX_GPIO31       31
#define UART0RX_GPIO31_FNCSEL 4
#define UART0RX_GPIO40       40
#define UART0RX_GPIO40_FNCSEL 0

//Table 351 in Appollo2 datasheet
#define UART0RTS_GPIO3         3
#define UART0RTS_GPIO3_FNCSEL  0
#define UART0RTS_GPIO5         5
#define UART0RTS_GPIO5_FNCSEL  2
#define UART0RTS_GPIO13       13
#define UART0RTS_GPIO13_FNCSEL 6
#define UART0RTS_GPIO35       35
#define UART0RTS_GPIO35_FNCSEL 6
#define UART0RTS_GPIO37       37
#define UART0RTS_GPIO37_FNCSEL 2
#define UART0RTS_GPIO41       41
#define UART0RTS_GPIO41_FNCSEL 7

//Table 352 in Appollo2 datasheet
#define UART0CTS_GPIO4         4
#define UART0CTS_GPIO4_FNCSEL  0
#define UART0CTS_GPIO6         6
#define UART0CTS_GPIO6_FNCSEL  2
#define UART0CTS_GPIO12       12
#define UART0CTS_GPIO12_FNCSEL 6
#define UART0CTS_GPIO29       29
#define UART0CTS_GPIO29_FNCSEL 4
#define UART0CTS_GPIO36       36
#define UART0CTS_GPIO36_FNCSEL 6
#define UART0CTS_GPIO38       38
#define UART0CTS_GPIO38_FNCSEL 2

//Table 353 in Appollo2 datasheet
#define UART1TX_GPIO8         8
#define UART1TX_GPIO8_FNCSEL  6
#define UART1TX_GPIO12       12
#define UART1TX_GPIO12_FNCSEL 7
#define UART1TX_GPIO14       14
#define UART1TX_GPIO14_FNCSEL 2
#define UART1TX_GPIO18       18
#define UART1TX_GPIO18_FNCSEL 6
#define UART1TX_GPIO20       20
#define UART1TX_GPIO20_FNCSEL 5
#define UART1TX_GPIO35       35
#define UART1TX_GPIO35_FNCSEL 2
#define UART1TX_GPIO39       39
#define UART1TX_GPIO39_FNCSEL 1
            
//Table 354 in Appollo2 datasheet
#define UART1RX_GPIO9         9
#define UART1RX_GPIO9_FNCSEL  6
#define UART1RX_GPIO13       13
#define UART1RX_GPIO13_FNCSEL 7
#define UART1RX_GPIO15       15
#define UART1RX_GPIO15_FNCSEL 2
#define UART1RX_GPIO19       19
#define UART1RX_GPIO19_FNCSEL 6
#define UART1RX_GPIO21       21
#define UART1RX_GPIO21_FNCSEL 5
#define UART1RX_GPIO36       36
#define UART1RX_GPIO36_FNCSEL 2
#define UART1RX_GPIO40       40
#define UART1RX_GPIO40_FNCSEL 1

//Table 355 in Appollo2 datasheet            
#define UART1RTS_GPIO10       10
#define UART1RTS_GPIO10_FNCSEL 5
#define UART1RTS_GPIO16       16
#define UART1RTS_GPIO16_FNCSEL 7
#define UART1RTS_GPIO30       30
#define UART1RTS_GPIO30_FNCSEL 5
#define UART1RTS_GPIO44       44
#define UART1RTS_GPIO44_FNCSEL 0
            
//Table 356 in Appollo2 datasheet
#define UART1CTS_GPIO11       11
#define UART1CTS_GPIO11_FNCSEL 5
#define UART1CTS_GPIO17       17
#define UART1CTS_GPIO17_FNCSEL 7
#define UART1CTS_GPIO29       29
#define UART1CTS_GPIO29_FNCSEL 5
#define UART1CTS_GPIO45       45
#define UART1CTS_GPIO45_FNCSEL 0

#endif
/**
 ******************************************************************************
 ** \brief  Send data polled (blocking), enables UART before transfer and disables UART after transfer
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  pu8Data          Databuffer to send
 **
 ** \param u32Size           Size of Data
 **
 *****************************************************************************/     
#define ApolloUart_LowPowerSendPolled(pstcUart,pu8Data,u32Size) ApolloUart_Enable(ptscUart); ApolloUart_SendPolled(pstcUart, pu8Data,uint32_t u32Size); ApolloUart_Disable(ptscUart)

/**
 ******************************************************************************
 ** \brief  Receive data polled (blocking, no timeout feature!), enables UART before transfer and disables UART after transfer
 **
 ** \param  pstcUart         UART pointer
 **
 ** \param  pu8Data          Databuffer to receive
 **
 ** \param u32Size           Size of Data
 **
 *****************************************************************************/
#define ApolloUart_LowPowerReceivePolled(pstcUart, pu8Data,u32Size) ApolloUart_Enable(ptscUart); ApolloUart_ReceivePolled(pstcUart, pu8Data,u32Size); ApolloUart_Disable(ptscUart)

/**
 ******************************************************************************
 ** \brief  Asynchrounous receive and send data (experimental), enables UART before transfer and disables UART after transfer
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
#define ApolloUart_LowPowerTransferPolled(pstcUart, pu8DataOut,pu8DataIn,u32Size) ApolloUart_Enable(ptscUart); ApolloUart_TransferPolled(pstcUart, pu8DataOut,pu8DataIn,u32Size); ApolloUart_Disable(ptscUart)

/**
 ******************************************************************************
 ** \brief  sends a complete string (0-terminated), enables UART before transfer and disables UART after transfer
 **
 ** \param  pstcUart  UART pointer
 **
 ** \param  pu8Buffer Pointer to (constant) file of bytes in mem
 **
 *****************************************************************************/
#define ApolloUart_LowPowerPutString(pstcUart, pu8Buffer)  ApolloUart_Enable(ptscUart); ApolloUart_PutString(pstcUart, pu8Buffer); ApolloUart_Disable(ptscUart)

#if (!defined(UART0)) && defined(UART)
     #define UART0 UART
#endif

#if (APOLLOUART_ENABLED == 1) && defined(APOLLO2_H)
    #ifndef APOLLOUART0_ENABLED
    #define APOLLOUART0_ENABLED 1
    #endif
    #ifndef APOLLOUART1_ENABLED
    #define APOLLOUART1_ENABLED 1
    #endif    
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef int16_t (*pfn_apollouart_txnext_t) (void);
typedef void (*pfn_apollouart_rx_t)(int16_t);


#if defined(APOLLO2_H) || defined(APOLLO3_H)
typedef UART0_Type UART_Type;
#endif

typedef void (*pfn_apollouart_cb_t)(UART_Type* pstcUart);

/// ApolloUart module internal data
typedef struct stc_apollouart_intern_data
{
    pfn_apollouart_txnext_t cbTxNext;
    pfn_apollouart_rx_t cbRxSimple;
    pfn_apollouart_cb_t cbOverrun;
    pfn_apollouart_cb_t cbBreakError;
    pfn_apollouart_cb_t cbParityError;
    pfn_apollouart_cb_t cbFramingError;
    pfn_apollouart_cb_t cbRxTimeout;
    pfn_apollouart_cb_t cbTx;
    pfn_apollouart_cb_t cbRx;
    pfn_apollouart_cb_t cbDsr;
    pfn_apollouart_cb_t cbDcd;
    pfn_apollouart_cb_t cbCts;
    #if defined (APOLLO_H) || defined (APOLLO1_H) 
    pfn_apollouart_cb_t cbRi;
    #endif
    #if defined (APOLLO2_H) || defined (APOLLO3_H) 
    pfn_apollouart_cb_t cbTxCmp;
    #endif

    boolean_t bInitialized;
    boolean_t bRxEnabled;
    boolean_t bTxEnabled;
    uint32_t u32RegCR;
    uint32_t u32RegLCRH;
    uint32_t u32Baudrate;
    #if defined(USE_CMSIS_DRIVER)
    ARM_DRIVER_USART* pstcCmsisUartDriver;
    ARM_USART_SignalEvent_t pfnCmsisSignalEvent;
    ARM_USART_CAPABILITIES Capabilities;    
    #endif    
    struct 
    {
        int8_t i8RxPin;
        int8_t i8TxPin;
        int8_t i8CtsPin;
        int8_t i8RtsPin;
    } stcGpios;
} stc_apollouart_intern_data_t;

typedef enum en_apollouart_wlen
{
   ApolloUartWlen5Bit = 0,
   ApolloUartWlen6Bit = 1,
   ApolloUartWlen7Bit = 2,
   ApolloUartWlen8Bit = 3
} en_apollouart_wlen_t;

typedef enum en_apollouart_stop
{
    ApolloUartStop1 = 0,
    ApolloUartStop2 = 1
} en_apollouart_stop_t;

typedef enum en_apollouart_parity
{
    ApolloUartParityNone = 0,
    ApolloUartParityOdd = 1,
    ApolloUartParityEven = 3,
} en_apollouart_parity_t;

typedef struct stc_apollouart_config
{
    uint32_t u32Baudrate;
    en_apollouart_wlen_t enDataLen;
    en_apollouart_stop_t enStopBit;
    en_apollouart_parity_t enParity;
    uint32_t bEnableBreak       : 1;
    uint32_t bEnableLoopback    : 1;
    uint32_t bEnableFifo        : 1;
    uint32_t bEnableRx          : 1;
    uint32_t bEnableTx          : 1;
    uint32_t bEnableCts         : 1;
    uint32_t bEnableRts         : 1;
    uint32_t bEnableSirLowPower : 1;
    uint32_t bEnableSir         : 1;
    struct 
    {
        uint8_t u8RxPin;
        uint8_t u8TxPin;
        uint8_t u8CtsPin;
        uint8_t u8RtsPin;
    } stcGpios;
} stc_apollouart_config_t;
    
typedef enum en_apollouart_gpiotype
{
    ApolloUartGpioTypeTx = 0,
    ApolloUartGpioTypeRx = 1,
    ApolloUartGpioTypeRts = 2,
    ApolloUartGpioTypeCts = 3,
} en_apollouart_gpiotype_t;

typedef struct stc_apollouart_gpios
{
    UART_Type* pstcHandle;
    uint8_t u8Gpio;
    uint8_t u8Function : 4;
    en_apollouart_gpiotype_t enUartType : 4;
} stc_apollouart_gpios_t;

typedef struct stc_apollouart_status
{
    uint32_t bRxBusy        : 1;
    uint32_t bTxBusy        : 1;
    uint32_t bRxFull        : 1;
    uint32_t bTxEmpty       : 1;
    uint32_t bErrorParity   : 1;
    uint32_t bErrorOverflow : 1;
    uint32_t bErrorBreak    : 1;
    uint32_t bErrorFrameing : 1;
    uint32_t bDCD           : 1;
    uint32_t bDSR           : 1;
    uint32_t bCTS           : 1;
    uint32_t Reserved       : 21;
} stc_apollouart_status_t;

typedef enum en_apollouart_fifo_irq_level
{
    ApolloUartFifoIrqLevel12_5 = 0,
    ApolloUartFifoIrqLevel25 = 1,
    ApolloUartFifoIrqLevel50 = 2,
    ApolloUartFifoIrqLevel75 = 3,
    ApolloUartFifoIrqLevel87_5 = 4,

} en_apollouart_fifo_irq_level_t;

typedef enum en_apollouart_irqtype
{
    ApolloUartIrqTypeOverrun,
    ApolloUartIrqTypeBreakError,
    ApolloUartIrqTypeParityError,
    ApolloUartIrqTypeFramingError,
    ApolloUartIrqTypeRxTimeout,
    ApolloUartIrqTypeTx,
    ApolloUartIrqTypeRx,
    ApolloUartIrqTypeDsr,
    ApolloUartIrqTypeDcd,
    ApolloUartIrqTypeCts,
    #if defined (APOLLO_H) || defined (APOLLO1_H) 
    ApolloUartIrqTypeRi,
    #endif
    #if defined (APOLLO2_H) || defined (APOLLO3_H) 
    ApolloUartIrqTypeTxCmp,
    #endif
} en_apollouart_irqtype_t;

/// ApolloUart module internal data, storing internal information for each UART instance.
typedef struct stc_apollouart_instance_data
{
    UART_Type*  pstcInstance;  ///< pointer to registers of an instance
    stc_apollouart_intern_data_t stcInternData; ///< module internal data of instance
} stc_apollouart_instance_data_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
#if defined(USE_CMSIS_DRIVER)
    #if (defined(UART) || defined(UART0)) && ((APOLLOUART0_ENABLED == 1) || (APOLLOUART_ENABLED == 1))
        extern ARM_DRIVER_USART Driver_UART0;
    #endif

    #if (defined(UART1)) && (APOLLOUART1_ENABLED == 1)
        extern ARM_DRIVER_USART Driver_UART1;
    #endif
#endif

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void ApolloUart_Init(UART_Type* pstcUart,uint32_t u32Baudrate);
void ApolloUart_InitExtended(UART_Type* pstcUart,stc_apollouart_config_t* pstcConfig);
#if (APOLLOGPIO_ENABLED == 1)
en_result_t ApolloUart_InitByPin(uint8_t u8RxPin,uint8_t u8TxPin,uint32_t u32Baudrate, UART_Type** ppstcUart);
#endif
void ApolloUart_InitGpios(UART_Type* pstcUart);
void ApolloUart_Enable(UART_Type* pstcUart);
void ApolloUart_Disable(UART_Type* pstcUart);
void ApolloUart_PutChar(UART_Type* pstcUart, uint8_t u8Char);
void ApolloUart_PutString(UART_Type* pstcUart, char_t *pu8Buffer);
uint8_t ApolloUart_GetChar(UART_Type* pstcUart);
boolean_t ApolloUart_HasChar(UART_Type* pstcUart);
stc_apollouart_status_t ApolloUart_GetStatus(UART_Type* pstcUart);
void ApolloUart_SetRxFifoIrqLevel(UART_Type* pstcUart,en_apollouart_fifo_irq_level_t enLevel);
void ApolloUart_SetTxFifoIrqLevel(UART_Type* pstcUart,en_apollouart_fifo_irq_level_t enLevel);
void ApolloUart_RegisterCallback(UART_Type* pstcUart,en_apollouart_irqtype_t enIrqType,pfn_apollouart_cb_t cbCallback, uint8_t u8Priority);
void ApolloUart_RegisterCallbacks(UART_Type* pstcUart,pfn_apollouart_txnext_t cbTxNext, pfn_apollouart_rx_t cbRx, uint8_t u8Priority);
void ApolloUart_NewTxData(UART_Type* pstcUart);
boolean_t ApolloUart_NewTxDataEnabled(UART_Type* pstcUart);
void ApolloUart_Deinit(void);
void ApolloUart_SendPolled(UART_Type* pstcUart, uint8_t* pu8Data,uint32_t u32Size);
void ApolloUart_ReceivePolled(UART_Type* pstcUart, uint8_t* pu8Data,uint32_t u32Size);
void ApolloUart_TransferPolled(UART_Type* pstcUart, uint8_t* pu8DataOut,uint8_t* pu8DataIn,uint32_t u32Size);
void ApolloUart_UARTn_IRQHandler(UART_Type* pstcUart);

#ifdef __cplusplus
}
#endif

//@} // ApolloUartGroup

#endif /* __APOLLOUART_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


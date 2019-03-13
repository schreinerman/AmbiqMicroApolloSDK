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
/** \file main.c
 **
 ** Example how to use UART with interrupts
 **
 ** History:
 **   - 2019-03-13  V1.0  MSc  First Version
 *****************************************************************************/
#define __MAIN_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mcu.h"
#include "base_types.h"
#include "apollouart.h"
#include "ioringbuffer.h"
#include "string.h"
#include <stdio.h> 


/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define UART_PRIO  1
#define MY_UART    UART0
#define MAX_FIFO   32

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/



/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/


/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

static volatile uint32_t u32Counter;  //ms counter


stc_apollouart_config_t stcUartConfig =
{
    115200,               //uint32_t u32Baudrate;
    ApolloUartWlen8Bit,   //en_apollouart_wlen_t enDataLen;
    ApolloUartStop1,      //en_apollouart_stop_t enStopBit;
    ApolloUartParityNone, //en_apollouart_parity_t enParity;
    FALSE,                //uint32_t bEnableBreak       : 1;
    FALSE,                //uint32_t bEnableLoopback    : 1;
    TRUE,                 //uint32_t bEnableFifo        : 1;
    TRUE,                 //uint32_t bEnableRx          : 1;
    TRUE,                 //uint32_t bEnableTx          : 1;
    FALSE,                //uint32_t bEnableCts         : 1;
    FALSE,                //uint32_t bEnableRts         : 1;
    FALSE,                //uint32_t bEnableSirLowPower : 1;
    FALSE,                //uint32_t bEnableSir         : 1;
    {
        23,               //uint8_t u8RxPin;
        22,               //uint8_t u8TxPin;
        0xFF,             //uint8_t u8CtsPin;
        0xFF,             //uint8_t u8RtsPin;
    }
};


uint8_t au8BufferRx[512];
uint8_t au8BufferTx[512];

stc_ioringbuffer_t stcRingBufferRx = {&au8BufferRx[0],sizeof(au8BufferRx)};
stc_ioringbuffer_t stcRingBufferTx = {&au8BufferTx[0],sizeof(au8BufferTx)};

volatile boolean_t bTxEnabled = FALSE;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/


/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
 *********************************************************************************
 ** \brief weak delay function used with SysTick IRQ
 **  
 ** \param [in] delayMs Delay in ms
 **  
 ** \details weak delay function used with SysTick IRQ
 **  
 *********************************************************************************/
void delay(uint32_t delayMs) 
{
    uint32_t u32End = u32Counter;
    u32End += delayMs;
    while(u32End != u32Counter) __NOP();
}

/**
 *****************************************************************************
 ** 
 **\brief Systick interrupt handler defined by CMSIS
 **
 *****************************************************************************/
void SysTick_Handler(void)
{
	u32Counter++;
}


/**
 *****************************************************************************
 ** 
 **\brief RX callback
 **
 **\param pstcUart UART handle
 **
 *****************************************************************************/
void RxCallback(UART_Type* pstcUart)
{
   static uint8_t au8Buffer[MAX_FIFO];
   static int i;
   for(i = 0; i < sizeof(au8Buffer);i++)
   {
       if (ApolloUart_HasChar(pstcUart))
       {
           au8Buffer[i] = ApolloUart_GetChar(pstcUart);
       }
       else
       {
           break;
       }
   }
   IoRingBuffer_Add(&stcRingBufferRx,&au8Buffer[0],i,NULL);
}

/**
 *****************************************************************************
 ** 
 **\brief TX callback
 **
 **\param pstcUart UART handle
 **
 *****************************************************************************/
void TxCallback(UART_Type* pstcUart)
{
   static int i;
   static uint8_t u8Tmp;
   static uint32_t u32Read;
   for(i = 0; i < MAX_FIFO;i++)
   {
       if (ApolloUart_GetStatus(pstcUart).bTxEmpty == TRUE)
       {
           IoRingBuffer_Read(&stcRingBufferTx,&u8Tmp,1,&u32Read,NULL);
           if (u32Read == 0)
           {
               ApolloUart_RegisterCallback(pstcUart,ApolloUartIrqTypeTx,NULL,UART_PRIO);
               bTxEnabled = FALSE;
               return;
           }
           ApolloUart_PutChar(pstcUart,u8Tmp);
       }
       else
       {
           break;
       }
   }
}

/**
 *****************************************************************************
 ** 
 **\brief PutString Implementation
 **
 **\param myString String buffer
 **
 *****************************************************************************/
void MyPutString(char_t* myString)
{
    IoRingBuffer_Add(&stcRingBufferTx,myString,strlen(myString),NULL);
    if (bTxEnabled == FALSE)
    {
        ApolloUart_RegisterCallback(MY_UART,ApolloUartIrqTypeTx,TxCallback,UART_PRIO);
        TxCallback(MY_UART);
        bTxEnabled = TRUE;
    }
}

/**
 *****************************************************************************
 ** 
 **\brief Main function
 **
 *****************************************************************************/
int main(void)
{
    volatile uint32_t Count = 0;
    SystemCoreClockUpdate();                //update clock variable SystemCoreClock (defined by CMSIS)
    SysTick_Config(SystemCoreClock / 1000); //setup 1ms SysTick (defined by CMSIS)

    //application initialization area
	
    IoRingBuffer_Init(&stcRingBufferRx);
    IoRingBuffer_Init(&stcRingBufferTx);
	
    ApolloUart_InitExtended(MY_UART,&stcUartConfig);
    ApolloUart_SetRxFifoIrqLevel(MY_UART,ApolloUartFifoIrqLevel87_5);
    ApolloUart_SetTxFifoIrqLevel(MY_UART,ApolloUartFifoIrqLevel87_5);
    ApolloUart_RegisterCallback(MY_UART,ApolloUartIrqTypeRx,RxCallback,UART_PRIO);
    MyPutString("This is an ultra long string that is handled via IRQ and FIFO in \"background\" and can be much longer...\r\n");
    
    while(1)
    {
        //application code
    }
}



/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


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
 ** A detailed description is available at 
 ** @link mainGroup  description @endlink
 **
 ** History:
 **   - 2019-03-07  V1.0  MSc  Software SPI example using GPIOs
 *****************************************************************************/
#define __MAIN_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mcu.h"
#include "base_types.h"
#include "apollogpio.h"
#include <stdio.h>
#include "string.h"


/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

#define SW_SPI_MOSI_PIN  5
#define SW_SPI_MISO_PIN  6
#define SW_SPI_SCK_PIN   7
#define SW_SPI_CS_PIN    42

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

static uint8_t au8Data[100];


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
 *********************************************************************************
 ** \brief Read / Write Byte via SPI
 **  
 ** \param write_byte byte to write
 **  
 ** \return byte read
 **  
 *********************************************************************************/
static uint8_t SwSpiReadWriteByte(uint8_t write_byte)
{
    uint8_t the_bit;
    uint8_t read_byte = 0;

    for (the_bit = 0; the_bit < 8; the_bit++) 
    {
        /* write MOSI on trailing edge of previous clock */
        ApolloGpio_GpioWrite(SW_SPI_MOSI_PIN, (write_byte & 0x01));

        write_byte = write_byte>>1;

        /* half a clock cycle before leading/rising edge */
        ApolloGpio_GpioWrite(SW_SPI_SCK_PIN,TRUE);

        /* half a clock cycle before trailing/falling edge */

        /* read MISO on trailing edge */
        if (TRUE == ApolloGpio_GpioRead(SW_SPI_MISO_PIN))
        {
          read_byte |= (1<<the_bit);
        }

        ApolloGpio_GpioWrite(SW_SPI_SCK_PIN,FALSE);
    }

    return read_byte;
}

/**
 *********************************************************************************
 ** \brief Read / Write Bytes via SPI
 **  
 ** \param u8Chipselect  GPIO used for chipselect
 **  
 ** \param pu8DataIn  pointer to buffer for received data
 **  
 ** \param pu8DataOut  pointer to buffer for sending data
 **  
 ** \param u32Len  data length
 **  
 *********************************************************************************/
static void SwSpiReadWrite(uint8_t u8Chipselect, uint8_t* pu8DataIn, uint8_t* pu8DataOut, uint32_t u32Len)
{
    uint32_t i;
    ApolloGpio_GpioWrite(u8Chipselect,FALSE);
    for(i = 0; i < u32Len; i++)
    {
        if ((pu8DataIn != NULL) && (pu8DataOut != NULL))
        {
            *pu8DataIn = SwSpiReadWriteByte(*pu8DataOut);
            pu8DataOut++;
            pu8DataIn++;
        }
        else if (pu8DataOut != NULL)
        {
            SwSpiReadWriteByte(*pu8DataOut);
            pu8DataOut++;
        } else if (pu8DataIn != NULL)
        {
            *pu8DataIn = SwSpiReadWriteByte(0xFF);
            pu8DataIn++;
        }

    }
    ApolloGpio_GpioWrite(u8Chipselect,TRUE);
}

static void SwSpiInit(void)
{
    ApolloGpio_GpioWrite(SW_SPI_SCK_PIN,FALSE);
    ApolloGpio_GpioOutputEnable(SW_SPI_SCK_PIN,TRUE);

    ApolloGpio_GpioWrite(SW_SPI_MOSI_PIN,FALSE);
    ApolloGpio_GpioOutputEnable(SW_SPI_MOSI_PIN,TRUE);

    ApolloGpio_GpioWrite(SW_SPI_CS_PIN,TRUE);
    ApolloGpio_GpioOutputEnable(SW_SPI_CS_PIN,TRUE);

    ApolloGpio_GpioPullupEnable(SW_SPI_MISO_PIN,TRUE);
    ApolloGpio_GpioInputEnable(SW_SPI_MISO_PIN,TRUE);
}

/**
 *****************************************************************************
 ** 
 **\brief Main function
 **
 *****************************************************************************/
int main(void)
{
    SystemCoreClockUpdate();                //update clock variable SystemCoreClock (defined by CMSIS)
    SysTick_Config(SystemCoreClock / 1000); //setup 1ms SysTick (defined by CMSIS)

    //application initialization area
    
    memset(&au8Data[0],0,sizeof(au8Data));

    SwSpiInit();
    
    SwSpiReadWrite(SW_SPI_CS_PIN,&au8Data[0],"Hello World\r\n",13);

    printf("%s",(char_t*)&au8Data[0]);

    while(1)
    {
        //application code
    }
}



/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


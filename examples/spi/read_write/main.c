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
 ** Example how to use a SPI
 **
 ** Need enabled in RTE_Device.h:
 ** #define APOLLOGPIO_ENABLED    1
 ** #define IOMSTR0_ENABLED       1
 **
 ** Optionally enabled in RTE_Device.h for debugging:
 ** #define DEBUG_OUTPUT 1
 ** #define IOM_DEBUG    1
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
#include "apollogpio.h"
#include "apolloiom.h"


/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/


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

static const stc_apolloiom_config_t stcSpiConfig = {
      IomInterfaceModeSpi, //use SPI mode
      4000000UL,          //frequency
      FALSE,               //SPHA setting
      FALSE,               //SPOL setting
      8,                   //WriteThreshold
      8,                   //ReadThreshold
      FALSE,               //FullDuplex
      5,     // SCK pin
      6,    // MISO
      7,    // MOSI
      FALSE};              // Only write not need to have common MOSI/MISO

uint8_t au8DataOut[2] = {0xAA,0xBB};
uint8_t au8DataIn[20];
uint8_t u8Channel;

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
 **\brief Main function
 **
 *****************************************************************************/
int main(void)
{
    en_result_t res;
    SystemCoreClockUpdate();                //update clock variable SystemCoreClock (defined by CMSIS)
    SysTick_Config(SystemCoreClock / 1000); //setup 1ms SysTick (defined by CMSIS)

    //application initialization area

    // GPIO5 SCK pin
    // GPIO6 MISO pin
    // GPIO7 MOSI pin
    // GPIO42 CS pin

    // Sending 2 bytes in au8DataOut, 
    // receiving 20 bytes with pullup on MISO, 
    // receiving 20 bytes without pullup on MISO
    // and keeping CS low during all transfers

    ApolloIOM_Configure(IOMSTR0,&stcSpiConfig);
    ApolloIOM_Enable(IOMSTR0);

    ApolloIOM_InitSpiCs(IOMSTR0,42,&u8Channel);
    
    res = ApolloIom_SpiWrite(IOMSTR0, u8Channel, &au8DataOut[0], sizeof(au8DataOut), NULL, AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW, NULL);
    
    ApolloGpio_GpioPullupEnable(6,TRUE);  //Set pullup on MISO
    res = ApolloIom_SpiRead(IOMSTR0, u8Channel, &au8DataIn[0], sizeof(au8DataIn), NULL, AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW, NULL); //will read all 0xFF
    
    ApolloGpio_GpioPullupEnable(6,FALSE); //remove pullup on MISO
    res = ApolloIom_SpiRead(IOMSTR0, u8Channel, &au8DataIn[0], sizeof(au8DataIn), NULL, AM_HAL_IOM_RAW, NULL); //will read all 0x00

    while(1)
    {
        //application code
    }
}



/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


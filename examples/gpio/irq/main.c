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
 ** Example how to use GPIOs with interrupts
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
 **\brief Systick interrupt handler defined by CMSIS
 **
 **\param
 **
 *****************************************************************************/
void GpioCallback(uint8_t u8Gpio)
{
    ApolloGpio_GpioToggle(14);
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

    ApolloGpio_GpioOutputEnable(17,TRUE); //enable output for LED connected at GPIO17
    ApolloGpio_GpioOutputEnable(14,TRUE); //enable output for LED connected at GPIO14

    ApolloGpio_GpioInputEnable(16,TRUE);  //enable button connected to GPIO16
    ApolloGpio_RegisterIrq(16,GpioFallingEdge,1,GpioCallback);  //register callback for GPIO16 interrupt
    while(1)
    {
        if ((u32Counter % 500) == 0)
        {
            ApolloGpio_GpioToggle(17);
            while((u32Counter % 500) == 0) __NOP(); //waits maximum 1ms
        }
        //application code
    }
}



/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


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
/** \file ApolloIom.h
**
** A detailed description is available at 
** @link ApolloIomGroup Apollo IOM Driver description @endlink
**
** History:
**   - 2017-01-31  V1.0  MSc  First Version
**   - 2017-04-13  V1.1  MSc  Added one-byte transfer
**
*****************************************************************************/
#ifndef __APOLLOIOM_H__
#define __APOLLOIOM_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
******************************************************************************
** \defgroup ApolloIomGroup Apollo IOM Driver
**
** Provided functions of ApolloIom:
** 
**   
******************************************************************************/
//@{

/**
******************************************************************************    
** \page apolloiom_module_includes Required includes in main application
** \brief Following includes are required
** @code   
** #include "apolloiom.h"   
** @endcode
**
******************************************************************************/

/**
******************************************************************************    
** \page apolloiom_module_setup Required settings in RTE_Device.h
** \brief Following defines are required in RTE_Device.h
** @code   
** #define IOMSTR0_ENABLED 1    //enable IOMSTR0 
** @endcode
**
******************************************************************************/
 
/**
******************************************************************************    
** \page apolloiom_module_config Required configuration in the user code
** \brief Following configuration are required in the user code
** @code   
** const stc_apolloiom_config_t stcIomConfig = {
**    IomInterfaceModeSpi, //use SPI mode
**    15000000UL,          //frequency
**    FALSE,               //SPHA setting
**    FALSE,               //SPOL setting
**    0,                   //WriteThreshold
**    60                   //ReadThreshold
** };
**
** void MI0283QTTftSpiInit(boolean_t bEnableDisable)
** {
**     if (bEnableDisable)
**     {
**         ApolloIOM_Configure(IOMSTR0,&stcIomConfig);
**         ApolloIOM_Enable(IOMSTR0);
**     }
** }
** @endcode
**
******************************************************************************/

/**
******************************************************************************    
** \page apolloiom_module_example_write Required code to write data in the user code
** \brief Following code is required to write data in the user code
** @code   
** void MI0283QTTftSpiWrite(uint32_t* pu32Data, uint32_t u32Len)
** {
**     uint32_t u32BytesWritten = 0;
**     ApolloIom_SpiWrite(IOMSTR1,6,(uint32_t*)pu32Data,u32Len,&u32BytesWritten,AM_HAL_IOM_RAW);
** }
** @endcode
**
******************************************************************************/
    
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
    
#include "mcu.h"
#include "base_types.h"
    
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
    
#define APOLLO_IOM_IS_SPIMODE(handle) (handle->CFG_b.IFCSEL == 1)
#define APOLLO_IOM_IS_I2CMODE(handle) (handle->CFG_b.IFCSEL == 0)
#define APOLLO_IOM_IS_IDLE(handle) (handle->STATUS_b.IDLEST == 1)
    
#if !defined(AM_HAL_IOM_CS_LOW)   
#define AM_HAL_IOM_CS_LOW                   0x10000000
#endif
    
#if !defined(AM_HAL_IOM_LSB_FIRST)   
#define AM_HAL_IOM_LSB_FIRST                0x08000000
#endif
    
#if !defined(AM_HAL_IOM_RAW)   
#define AM_HAL_IOM_RAW                      0x40000000
#endif
    
#if !defined(AM_HAL_IOM_OFFSET)   
#define AM_HAL_IOM_OFFSET(n)                ((n << 8) & 0x0000FF00)
#endif
    
#if !defined(AM_HAL_IOM_WRITE)      
#define AM_HAL_IOM_WRITE                    0x00000000
#endif
    
#if !defined(AM_HAL_IOM_READ)   
#define AM_HAL_IOM_READ                     0x80000000
#endif
    
/*****************************************************************************/
/* Global type definitions ('typedef')                                        */
/*****************************************************************************/

typedef enum en_apolloiom_interface_mode
{
    IomInterfaceModeI2C = 0,
    IomInterfaceModeSpi = 1,
} en_apolloiom_interface_mode_t;


//*****************************************************************************
//
// \brief Configuration structure for the IO master module.
//
//*****************************************************************************
typedef struct stc_apolloiom_config
{
    //
    // \brief Selects the physical protocol for the IO master module. Choose
    // either IomInterfaceModeSpi or IomInterfaceModeI2C.
    //
    en_apolloiom_interface_mode_t enInterfaceMode;
    
    
    uint32_t u32ClockFrequency;
    
    //
    // Select the SPI clock phase (unused in I2C mode).
    //
    boolean_t bSPHA;
    
    //
    // Select the SPI clock polarity (unused in I2C mode).
    //
    boolean_t bSPOL;
    
    //
    // \brief Select the FIFO write threshold.
    //
    // The IOM controller will generate a processor interrupt when the number
    // of entries in the FIFO goes *below* this number.
    //
    uint8_t u8WriteThreshold;
    
    //
    // \brief Select the FIFO read threshold.
    //
    // The IOM controller will generate a processor interrupt when the number
    // of entries in the FIFO grows *larger* than this number.
    //
    uint8_t u8ReadThreshold;
} stc_apolloiom_config_t;

// Non-blocking buffer and buffer-management variables  
typedef struct stc_apolloiom_nb_buffer
{
    uint32_t u32State;
    uint32_t *pu32Data;
    uint32_t u32BytesLeft;
    void (*pfnCallback)(void);
} stc_apolloiom_nb_buffer_t;

/// ApolloIOM module internal data
typedef struct stc_apolloiom_intern_data
{
    uint32_t u32ModInterface;
    stc_apolloiom_nb_buffer_t pstcBuffer;
    en_apolloiom_interface_mode_t enInterfaceMode;    
} stc_apolloiom_intern_data_t;

/// ApolloIOM module internal data, storing internal information for each IOM instance.
typedef struct stc_apolloiom_instance_data
{
    IOMSTR0_Type*  pstcInstance;  ///< pointer to registers of an instance
    stc_apolloiom_intern_data_t stcInternData; ///< module internal data of instance
} stc_apolloiom_instance_data_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/



/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

en_result_t ApolloIOM_Enable(IOMSTR0_Type* pstcInstance);
en_result_t ApolloIOM_Disable(IOMSTR0_Type* pstcInstance);
en_result_t ApolloIOM_Configure(IOMSTR0_Type* pstcInstance, const stc_apolloiom_config_t* pstcConfig);
en_result_t ApolloIom_SpiCommand(IOMSTR0_Type* pstcHandle, uint32_t u32Operation, uint32_t u32ChipSelect, uint32_t u32NumBytes, uint32_t u32Options);
en_result_t ApolloIom_SpiWrite(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options);
en_result_t ApolloIom_SpiRead(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesRead, uint32_t u32Options);
void ApolloIom_SpiWriteByte(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint8_t u8Data, uint32_t u32Options);
uint8_t ApolloIom_SpiReadByte(IOMSTR0_Type* pstcHandle, uint32_t u32ChipSelect, uint32_t u32Options);
en_result_t ApolloIom_I2cCommand(IOMSTR0_Type* pstcHandle, uint32_t u32Operation, uint32_t u32BusAddress, uint32_t u32NumBytes, uint32_t u32Options);
en_result_t ApolloIom_I2cWrite(IOMSTR0_Type* pstcHandle, uint32_t u32BusAddress, uint8_t* pu8Data, uint32_t u32NumBytes, uint32_t* pu32BytesWritten, uint32_t u32Options);


#ifdef __cplusplus
}
#endif

//@} // ApolloIomGroup

#endif /*__APOLLOIOM_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/


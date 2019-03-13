//
//  IoRingBuffer.c
//  ioRingBuffer
//
//  Created by Manuel Schreiner on 07.02.16.
//  Copyright © 2016 io-expert.com. All rights reserved.
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this condition and the following disclaimer.
//
// This software is provided by the copyright holder and contributors "AS IS"
// and any warranties related to this software are DISCLAIMED.
// The copyright owner or contributors be NOT LIABLE for any damages caused
// by use of this software.

#include "IoRingBuffer.h"
#include "base_types.h"

/**
	Initialize Ring Buffer
	@param pstcHandle Ring Buffer Handle
	@returns Ok or ErrorUninitialized if pstcHandle is NULL
 */
en_result_t IoRingBuffer_Init(stc_ioringbuffer_t* pstcHandle)

{
    if (pstcHandle == NULL) return ErrorUninitialized;
    pstcHandle->u32DataWritePos = 0;
    pstcHandle->u32DataReadPos = 0;
    return Ok;
}

/**
	Add new data to ring buffer
	@param pstcHandle Ring Buffer Handle
	@param pu8Buffer Buffer to add
	@param u32DataLength Size of buffer to add
	@param pfnOverwriteWaitCallback Wait routine, if returns TRUE, overwrite will executed
                                    If this is NULL, overwrite will always work
	@returns Ok or ErrorUninitialized if pstcHandle is NULL
 */
en_result_t IoRingBuffer_Add(stc_ioringbuffer_t* pstcHandle, uint8_t* pu8Buffer, uint32_t u32DataLength, ioringbuffer_datawait_func_ptr_t pfnOverwriteWaitCallback)

{
    uint32_t i;
    if (pstcHandle == NULL) return ErrorUninitialized;
    for(i = 0; i < u32DataLength;i++)
    {
        if (pstcHandle->u32DataWritePos >= pstcHandle->u32Size)
        {
            pstcHandle->u32DataWritePos = 0;
            
            if (pfnOverwriteWaitCallback != NULL)
            {
                while(pstcHandle->u32DataWritePos == pstcHandle->u32DataReadPos)
                {
                    if (pfnOverwriteWaitCallback() == TRUE)
                    {
                        break;
                    }
                }
            }
            
        }
        pstcHandle->pu8Buffer[pstcHandle->u32DataWritePos] = pu8Buffer[i];
        pstcHandle->u32DataWritePos++;
        if (pfnOverwriteWaitCallback != NULL)
        {
            while(pstcHandle->u32DataWritePos == pstcHandle->u32DataReadPos)
            {
                if (pfnOverwriteWaitCallback() == TRUE)
                {
                    break;
                }
            }
        }
        
        
    }
    return Ok;
}

/**
	Read data from ring buffer
	@param pstcHandle Ring Buffer Handle
	@param pu8Buffer Buffer to read in
	@param u32DataLength Size of data to be read
	@param pu32DataRead Returns read data size
	@param pfnDataWaitCallback  Wait routine if data is not ready, if returns TRUE, read will stopped
                                If this is NULL only maximum available data will be read
	@returns Ok or ErrorUninitialized if pstcHandle is NULL
 */
en_result_t IoRingBuffer_Read(stc_ioringbuffer_t* pstcHandle, uint8_t* pu8Buffer, uint32_t u32DataLength,uint32_t* pu32DataRead, ioringbuffer_datawait_func_ptr_t pfnDataWaitCallback)

{
    uint32_t i;
    *pu32DataRead = 0;
    if (pstcHandle == NULL) return ErrorUninitialized;
    for(i = 0;i < u32DataLength;i++)
    {
        if (pstcHandle->u32DataReadPos >= pstcHandle->u32Size)
        {
            pstcHandle->u32DataReadPos = 0;
        }
        if (pfnDataWaitCallback != NULL)
        {
            while(pstcHandle->u32DataWritePos == pstcHandle->u32DataReadPos)
            {
                if (pfnDataWaitCallback() == TRUE)
                {
                    return Ok;
                }
            }
        } else if (pstcHandle->u32DataWritePos == pstcHandle->u32DataReadPos)
        {
            return Ok;
        }
        *pu32DataRead = *pu32DataRead + 1;
        pu8Buffer[i] = pstcHandle->pu8Buffer[pstcHandle->u32DataReadPos];
        pstcHandle->u32DataReadPos++;
    }
    return Ok;
}

/**
	Read data from ring buffer without changing read pointer
	@param pstcHandle Ring Buffer Handle
	@param pu8Buffer Buffer to read in
	@param u32DataLength Size of data to be read
	@param pu32DataRead Returns read data size
	@param pfnDataWaitCallback Wait routine if data is not ready, if returns TRUE, read will stopped
                               If this is NULL only maximum available data will be read
	@returns Ok or ErrorUninitialized if pstcHandle is NULL
 */
en_result_t IoRingBuffer_SniffRead(stc_ioringbuffer_t* pstcHandle, uint8_t* pu8Buffer, uint32_t u32DataLength,uint32_t* pu32DataRead, ioringbuffer_datawait_func_ptr_t pfnDataWaitCallback)

{
    uint32_t i;
    *pu32DataRead = 0;
    if (pstcHandle == NULL) return ErrorUninitialized;
    for(i = 0;i < u32DataLength;i++)
    {
        if (pstcHandle->u32DataReadPos >= pstcHandle->u32Size)
        {
            pstcHandle->u32DataReadPos = 0;
        }
        if (pfnDataWaitCallback != NULL)
        {
            while(pstcHandle->u32DataWritePos == pstcHandle->u32DataReadPos)
            {
                if (pfnDataWaitCallback() == TRUE)
                {
                    return Ok;
                }
            }
        } else if (pstcHandle->u32DataWritePos == pstcHandle->u32DataReadPos)
        {
            return Ok;
        }
        *pu32DataRead = *pu32DataRead + 1;
        pu8Buffer[i] = pstcHandle->pu8Buffer[pstcHandle->u32DataReadPos + i];
    }
    return Ok;
}

/**
	Set Read Position to Write Position
	@param pstcHandle Ring Buffer Handle
	@returns Ok or ErrorUninitialized if pstcHandle is NULL
 */
en_result_t IoRingBuffer_SetReadToWritePosition(stc_ioringbuffer_t* pstcHandle)

{
    if (pstcHandle == NULL) return ErrorUninitialized;
    pstcHandle->u32DataReadPos = pstcHandle->u32DataWritePos;
    return Ok;
}
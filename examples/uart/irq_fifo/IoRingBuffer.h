//
//  IoRingBuffer.h
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

#ifndef IoRingBuffer_h
#define IoRingBuffer_h

#include <stdio.h>
#include "base_types.h"

typedef boolean_t (*ioringbuffer_datawait_func_ptr_t)(void);

typedef struct stc_ioringbuffer
{
    uint8_t* pu8Buffer;         ///< Buffer of the ring buffer
    uint32_t u32Size;           ///< Ring Buffer size =sizeof(pu8Buffer)
    uint32_t u32DataWritePos;   ///< Current write position
    uint32_t u32DataReadPos;    ///< Current read position
} stc_ioringbuffer_t;

en_result_t IoRingBuffer_Init(stc_ioringbuffer_t* pstcHandle);
en_result_t IoRingBuffer_Add(stc_ioringbuffer_t* pstcHandle, uint8_t* pu8Buffer, uint32_t u32DataLength, ioringbuffer_datawait_func_ptr_t pfnOverwriteWaitCallback);
en_result_t IoRingBuffer_Read(stc_ioringbuffer_t* pstcHandle, uint8_t* pu8Buffer, uint32_t u32DataLength,uint32_t* pu32DataRead, ioringbuffer_datawait_func_ptr_t pfnDataWaitCallback);
en_result_t IoRingBuffer_SniffRead(stc_ioringbuffer_t* pstcHandle, uint8_t* pu8Buffer, uint32_t u32DataLength,uint32_t* pu32DataRead, ioringbuffer_datawait_func_ptr_t pfnDataWaitCallback);
en_result_t IoRingBuffer_SetReadToWritePosition(stc_ioringbuffer_t* pstcHandle);

#endif /* IoRingBuffer_h */

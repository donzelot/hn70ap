/****************************************************************************
 * hn70ap/apps/export/memio.h
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef MEMIO_H
#define MEMIO_H

#include <stdint.h>

#define PEEK_U16LE(addr) ( *(uint8_t*)(addr)    | *((uint8_t*)(addr)+1)<<8 )
#define PEEK_U16BE(addr) ( *(uint8_t*)(addr)<<8 | *((uint8_t*)(addr)+1)    )

#define POKE_U16LE(addr,val) do { *((uint8_t*)(addr)) = (uint8_t)((val)   ); *((uint8_t*)(addr)+1) = (uint8_t)((val)>>8); } while(0)
#define POKE_U16BE(addr,val) do { *((uint8_t*)(addr)) = (uint8_t)((val)>>8); *((uint8_t*)(addr)+1) = (uint8_t)((val)   ); } while(0)

#define PEEK_U32LE(addr) ( *(uint8_t*)(addr)     | *((uint8_t*)(addr)+1)<< 8 | *((uint8_t*)(addr)+2)<<16 | *((uint8_t*)(addr)+3)<<24 )
#define PEEK_U32BE(addr) ( *(uint8_t*)(addr)<<24 | *((uint8_t*)(addr)+1)<<16 | *((uint8_t*)(addr)+2)<< 8 | *((uint8_t*)(addr)+3)     )

#define POKE_U32LE(addr,val) do { *((uint8_t*)(addr)) = (uint8_t)((val)    ); *((uint8_t*)(addr)+1) = (uint8_t)((val)>> 8); *((uint8_t*)(addr)+2) = (uint8_t)((val)>>16); *((uint8_t*)(addr)+3) = (uint8_t)((val)>>24); } while(0)
#define POKE_U32BE(addr,val) do { *((uint8_t*)(addr)) = (uint8_t)((val)>>24); *((uint8_t*)(addr)+1) = (uint8_t)((val)>>16); *((uint8_t*)(addr)+2) = (uint8_t)((val)>> 8); *((uint8_t*)(addr)+3) = (uint8_t)((val)    ); } while(0)

#endif


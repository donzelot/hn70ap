/****************************************************************************
 * hn70ap/apps/libhn70ap/crc.c
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

#include <stdint.h>
#include <stddef.h>
#include <hn70ap/crc.h>

static uint32_t crc32_table[256];

/*----------------------------------------------------------------------------*/
uint16_t crc16(uint16_t crc, uint8_t data)
{
  crc ^= data&0xFF;
  crc ^= (crc<<4)&0xFF;
  crc  = (uint16_t)(crc>>8)^
    (uint16_t)((uint16_t)(crc&0xFF)<<8)^
    (uint16_t)((uint16_t)(crc&0xFF)<<3)^
    (uint16_t)((crc&0xFF)>>4);

  return crc;
}

/*----------------------------------------------------------------------------*/
void crc32_init(void)
{
  unsigned long c;
  int n, k;
  for (n = 0; n < 256; n++)
    {
      c = (unsigned long) n;
      for (k = 0; k < 8; k++)
        {
          if (c & 1)
            {
              c = 0xedb88320L ^ (c >> 1);
            }
          else
            {
              c = c >> 1;
            }
        }
      crc32_table[n] = c;
    }
}

/*----------------------------------------------------------------------------*/
uint32_t crc32_do(uint32_t crc, uint8_t *data, uint32_t len)
{
  uint32_t c = crc;
  uint32_t n;
  static uint32_t *table = NULL;

  if(table == NULL)
    {
      crc32_init();
      table = crc32_table;
    }

  for (n = 0; n < len; n++)
    {
      c = crc32_table[(c ^ data[n]) & 0xff] ^ (c >> 8);
    }
  return c;
}


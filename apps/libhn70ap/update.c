/****************************************************************************
 * hn70ap/apps/libhn70ap/update.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <hn70ap/tlv.h>
#include <hn70ap/memio.h>
#include <hn70ap/crc.h>
#include <hn70ap/update.h>

#define TAG_UPSIZE 0xC0
#define TAG_UPCRC  0xC3
#define TAG_UPSHA  0xC4

/*----------------------------------------------------------------------------*/
int update_parseheader(struct update_header_s *hdr, uint8_t *buf, int buflen)
{
  uint8_t *ptr;
  uint32_t len;
  uint32_t crc = CRC32_INIT;

  /* Check CRC of header */
  crc = crc32_do(crc, buf+4, buflen-4);
  crc ^= PEEK_U32LE(buf);

  if(crc)
    {
      fprintf(stderr, "Bad Header CRC\n");
      return ERROR;
    }

  /* Skip CRC */
  buf    += 4;
  buflen -= 4;

  /* Find important tags */
  ptr = tlv_find(buf, buflen, TAG_UPSIZE, &len, 0);
  if(ptr==NULL || len != 4)
    {
      fprintf(stderr, "TAG_UPSIZE not found/correct\n");
      return ERROR;
    }
  hdr->size = PEEK_U32BE(ptr);
  printf("Update size : %u bytes\n", hdr->size);
  hdr->size += 16384; //add size of bootloader, not encoded in field.

  ptr = tlv_find(buf, buflen, TAG_UPCRC, &len, 0);
  if(ptr==NULL || len != 4)
    {
      fprintf(stderr, "TAG_UPCRC not found/correct\n");
    }
  else
    {
    hdr->crc = PEEK_U32BE(ptr);
    printf("Update CRC : %08X\n", hdr->crc);
    }

  ptr = tlv_find(buf, buflen, TAG_UPSHA, &len, 0);
  if(ptr==NULL || len != 32)
    {
      fprintf(stderr, "TAG_UPSHA not found/correct\n");
    }
  else
    {
    int i;
    memcpy(hdr->sha, ptr, 32);
    printf("Update SHA : ");
    for(i=0;i<32;i++) printf("%02X",*ptr++);
    printf("\n");
    }

  return OK;
}



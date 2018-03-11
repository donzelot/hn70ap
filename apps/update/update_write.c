/****************************************************************************
 * hn70ap/apps/update/update_write.c
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
#include <stdlib.h>
#include <string.h>

#include <sys/ioctl.h>
#include <fcntl.h>

#include <hn70ap/memio.h>
#include <hn70ap/crc.h>
#include <hn70ap/mtdchar.h>
#include <hn70ap/update.h>

#include "update_internal.h"

int update_write_start(struct update_context_s *ctx)
{
  ctx->header_received = false;
  ctx->block_received  = 0;
  ctx->total_received  = 0;
  ctx->block_id        = 1; //Start flash write right after header block
  ctx->datacrc         = CRC32_INIT;

  return OK;
}

/*----------------------------------------------------------------------------*/
int update_write(struct update_context_s *ctx, uint8_t *buf, uint32_t len)
{
  int retval = OK; //default will proceed with next packet after this one.
  int room;
  int off = 0;
  int ret; /* ioctl return values */
  struct mtdchar_req_s req;

again:
  //printf("curpagelen=%d rxlen=%d \n", ctx->block_received, len);
  room = ctx->block_len - ctx->block_received; /* compute length needed to fill current page*/
  if(len <= room)
    {
      room = len;
      //printf("all block goes into page (rxoff %d)\n", off);
    }
  else
    {
      /* We have more data than what is roomed to finish a page*/
      //printf("partial block (%d bytes, rxoff %d) goes into page\n", room, off);
    }

  memcpy(ctx->block + ctx->block_received, buf+off, room);
  ctx->block_received += room;
  ctx->total_received += room;
  len -= room;
  off += room;

  if(ctx->block_received == ctx->block_len)
    {
      if(!ctx->header_received)
        {
          int i,j;
          uint8_t check, check2;

          //printf("HEADER\n");
          memcpy(ctx->header, ctx->block, 256);
          if(update_parseheader(&ctx->update, ctx->header, 256) != OK)
            {
              return ERROR;
            }
          ctx->header_received = true;

          printf("BLANK CHECK:"); fflush(stdout);
          check = 0xFF;
          for(i = 0; i < ctx->block_count; i++)
            {
              req.block = i;
              req.count = 1;
              req.buf   = ctx->block;
              ret = ioctl(ctx->mtdfd, MTDCHAR_BREAD, (unsigned long)&req);
              if(ret < 0)
                {
                  printf("FAILED\n");
                  return ERROR;
                }
              check2 = 0xFF;
              for(j = 0; j < 256; j++)
                {
                  check2 &= ctx->block[j];
                }
             if(check2 != 0xFF) {printf("[%d] ",i); fflush(stdout);}
             check &= check2;
            }

          if(check != 0xFF)
            {
              printf("NOT BLANK. ERASE:"); fflush(stdout);
              ret = ioctl(ctx->mtdfd, MTDIOC_BULKERASE, 0);
              if(ret < 0)
                {
                  printf("FAILED\n");
                  return ERROR;
                }
              else
                {
                  printf("OK\n");
                }
            } //was not blank
          else
            {
              printf("OK\n");
            }
        }
      else
        {
          //printf("WRITE [%u]:",ctx->block_id);
          if(ctx->block_id == 1)
            {
              printf("WRITE:"); fflush(stdout);
            }
          printf("#"); fflush(stdout);
          /* Only data past 16k enters the CRC */
          if(ctx->block_id > 63)
            {
              ctx->datacrc = crc32_do(ctx->datacrc, ctx->block, 256);
            }
          req.block = ctx->block_id;
          req.buf   = ctx->block;
          req.count = 1;
          ret = ioctl(ctx->mtdfd, MTDCHAR_BWRITE, (unsigned long)&req);
          if(ret < 0)
            {
              printf("FAILED BLOCK %d\n");
              return ERROR;
            }
            ctx->block_id += 1;
        }
      ctx->block_received = 0;
    }

  if(len > 0)
    {
      goto again;
    }

  if(ctx->header_received) //Means we have parsed the header and know the total size.
    {
      //printf("managed %u of %u\n", ctx->total_received, ctx->update_size);
      if(ctx->total_received >= ctx->update.size)
        {
          //printf("Update complete.\n");
          /* Write last partial page */
          if(ctx->block_received>0)
            {
              printf("WRITE LAST [%u]:",ctx->block_id);
              if(ctx->block_id > 63)
                {
                  ctx->datacrc = crc32_do(ctx->datacrc, ctx->block, ctx->block_received);
                }
              req.block = ctx->block_id;
              req.buf   = ctx->block;
              req.count = 1;
              ret = ioctl(ctx->mtdfd, MTDCHAR_BWRITE, (unsigned long)&req);
              if(ret < 0)
                {
                  printf("FAILED\n");
                  return ERROR;
                }
              else
                {
                  printf("OK\n");
                }
            }
          ctx->datacrc ^= CRC32_MASK;
          /* Check data CRC against info in header */
          if(ctx->datacrc != ctx->update.crc)
            {
              fprintf(stderr, "Data integrity error\n");
              return ERROR;
            }

          /* Copy header into first block of flash */
          printf("WRITE HEADER:");
          req.block = 0;
          req.buf   = ctx->header;
          req.count = 1;
          ret = ioctl(ctx->mtdfd, MTDCHAR_BWRITE, (unsigned long)&req);
          if(ret < 0)
            {
              printf("FAILED\n");
              return ERROR;
            }
          else
            {
              printf("OK\n");
            }
          ctx->done = true;
          return OK;
        }
    }

  /* No more data in rx buffer */
  return ret;
}

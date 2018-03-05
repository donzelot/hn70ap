/****************************************************************************
 * hn70ap/apps/update/update_status.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <hn70ap/mtdchar.h>
#include <hn70ap/update.h>
#include <hn70ap/crc.h>
#include <hn70ap/sha256.h>

#include "update_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int update_status(int mtdfd)
{
  struct mtdchar_req_s req;
  struct update_header_s hdr;
  uint8_t *buf;
  uint8_t acc;
  int ret;
  uint32_t todo;
  uint32_t crc;
  struct sha256_s sha;

  buf = malloc(256);
  if(!buf)
    {
      fprintf(stderr, "Mem error\n");
      return ERROR;
    }

  /* Read the header page */
  req.block = 0;
  req.buf   = buf;
  req.count = 1;
  ret = ioctl(mtdfd, MTDCHAR_BREAD, (unsigned long)&req);
  if(ret < 0)
    {
      fprintf(stderr, "Cannot read external flash\n");
      goto done;
    }

  /* Blank check */
  acc = 0xFF;
  for(ret = 0; ret < 256; ret++)
    {
      acc &= buf[ret];
    }

  if(acc == 0xFF)
    {
      printf("No update in external flash.\n");
      ret = OK;
      goto done;
    }

  ret = update_parseheader(&hdr, buf, 256);
  if(ret < 0)
    {
      fprintf(stderr, "Update header not valid\n");
      goto done;
    }
  printf("Computing image checksums...\n");

  todo = hdr.size - 16384; //Dont include extended header in CRC
  req.block = 64;
  crc = CRC32_INIT;
  sha256_init(&sha);

  while(todo > 0)
    {
      ret = ioctl(mtdfd, MTDCHAR_BREAD, (unsigned long)&req);
      if(ret < 0)
        {
        fprintf(stderr, "failed to read flash page %d (errno %d)\n", req.block, errno);
        goto done;
        }
      crc = crc32_do(crc, buf, (todo > 256) ? 256 : todo );
      sha256_update(&sha, buf, (todo > 256) ? 256 : todo );
      if(todo > 256)
        {
          todo -= 256;
          req.block += 1;
        }
      else
        {
          todo = 0; //done
        }
    }
  crc ^= CRC32_MASK;
  sha256_final(&sha, buf);

  printf("Update size    : %u bytes\n", hdr.size);
  printf("Update CRC     : %08X (%s)\n", hdr.crc, (hdr.crc == crc) ? "OK" : "FAIL" );
  printf("Update SHA-256 : ");
  acc = 0;
  for(ret = 0; ret < 32; ret++)
    {
      printf("%02X", hdr.sha[ret]);
      acc |= (buf[ret] ^ hdr.sha[ret]);
    }
  printf(" (%s)\n", (acc == 0) ? "OK": "FAIL");

  ret = OK;

done:
  free(buf);
  return ret;
}


/****************************************************************************
 * hn70ap/apps/update/update_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <nuttx/mtd/mtd.h>

#include "update_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int update_usage(void)
{
  printf("update serial - Receive update from console\n"
         "update tftp host port - Receive update from TFTP server\n"
         "update status - Show info about current firmware update\n"
         "update cancel - Erase the firmware update partition\n");
  return ERROR;
}

/****************************************************************************
 * status_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int update_main(int argc, char *argv[])
#endif
{
  int fd;
  int ret;
  struct mtd_geometry_s geo;
  struct update_context_s ctx;

  if(argc<2)
    {
      return update_usage();
    }

  fd = open("/dev/firmware", O_RDWR);
  if(fd < 0)
    {
      fprintf(stderr, "Cannot open firmware partition\n");
      return ERROR;
    }

  /* Make sure it's a MTD partition of 2 Mbytes */
  ret = ioctl(fd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if(ret != 0)
    {
      fprintf(stderr, "Cannot get MTD geometry\n");
      ret = ERROR;
      goto error_with_mtd;
    }

  fprintf(stderr, "neraseblocks=%d erasesize=%d blocksize=%d\n", geo.neraseblocks, geo.erasesize, geo.blocksize);

  if(geo.blocksize != 256)
    {
      fprintf(stderr, "Block size !=256 not supported!\n");
      ret = ERROR;
      goto error_with_mtd;
    }

  ctx.mtdfd       = fd;
  ctx.block_len   = geo.blocksize;
  ctx.block_count = geo.neraseblocks * geo.erasesize / geo.blocksize;

  /* Allocate storage for header */
  ctx.header = malloc(256);
  if(ctx.header==0)
    {
      fprintf(stderr, "alloc error!\n");
      goto error_with_mtd;
    }

  /* Allocate storage for flash page buffer */
  ctx.block = malloc(256);
  if(ctx.block == 0)
    {
      fprintf(stderr, "alloc error!\n");
      goto error_with_header;
    }

  if(!strcmp(argv[1], "serial"))
    {
      ret = update_serial(&ctx);
    }
  else if(!strcmp(argv[1], "tftp"))
    {
      if(argc != 4)
        {
          ret = update_usage();
        }
      else
        {
          ret = update_tftp(&ctx, argv[2], argv[3]);
        }
    }
  else if(!strcmp(argv[1], "status"))
    {
      ret = update_status(fd);
    }
  else if(!strcmp(argv[1], "cancel"))
    {
      ret = update_cancel(fd);
    }
  else
    {
      ret = update_usage();
    } 

  free(ctx.block);
error_with_header:
  free(ctx.header);
error_with_mtd:
 close(fd);

 return ret;
}

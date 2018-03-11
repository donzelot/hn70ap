/****************************************************************************
 * hn70ap/apps/update/update_tftp.c
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

#include <errno.h>

#include <netutils/tftp.h>
#include <netutils/netlib.h>

//#include <hn70ap/memio.h>
//#include <hn70ap/crc.h>
#include <hn70ap/update.h>

#include "update_internal.h"

/* Load an update image via TFTP */

/*----------------------------------------------------------------------------*/
ssize_t update_tftpwrite(FAR void *ctx, uint32_t offset, uint8_t *buf, size_t len)
{
  //printf("tftp_write len=%d", len); fflush(stdout);

  update_write((struct update_context_s *)ctx, buf, len);

  return len;
}

/*----------------------------------------------------------------------------*/
int update_tftp(struct update_context_s *ctx, const char *server, const char *remote_filename)
{
  int ret;
  in_addr_t host;

  if (!netlib_ipv4addrconv(server, (FAR unsigned char*)&host))
    {
      fprintf(stderr, "Incorrect IP address!\n");
      return ERROR;
    }

  printf("hn70ap tftp update from %d.%d.%d.%d:%s\n", host&0xFF, (host>>8)&0xFF, (host>>16)&0xFF, (host>>24)&0xFF, remote_filename);

  update_write_start(ctx);

  ret = tftpget_cb(remote_filename, host, true, update_tftpwrite, ctx);
  
  if(ret != OK)
    {
      printf("Transfer failed! (errno %d)\n", errno);
    }
  else
    {
      printf("Transfer complete.\n");
    }

  return ret;
}


/****************************************************************************
 * hn70ap/apps/rxt/rxt_main.c
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <hn70ap/system.h>
#include <hn70ap/radio.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * status_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int rxt_main(int argc, char *argv[])
#endif
{
  int ret = OK;
  int fd;
  int buflen = 1024;
  int i;

  uint8_t *buf;

  printf("RX test using aux radio\n");

  hn70ap_system_init();

  buf = malloc(buflen);
  if(!buf)
    {
      fprintf(stderr, "malloc failed!\n");
      ret = ERROR;
      goto done;
    }

  do
    {
      ret = hn70ap_radio_receive(HN70AP_RADIO_AUX, buf, buflen);
      //printf("read done, ret = %d, errno=%d\n", ret, errno);
      if(ret > 0)
        {
/*          for(i=0; i<ret; i++)
            {
              printf("%02X ", buf[i]);
            }
          printf("\n");*/
          for(i=0; i<ret; i++)
            {
              printf("%c", (buf[i]<0x20 && buf[i]!=0x0a)?'.':buf[i]);
            }
        }
      else if(ret < 0 && errno==ETIMEDOUT)
        {
          printf("RX timeout\n");
//          ret = 1;
//          continue;
        }
    }
  while(ret > 0);

  printf("Read sequence done.\n");

  free(buf);
done:
  return ret;
}


/****************************************************************************
 * hn70ap/apps/beacon/beacon_main.c
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

#include <hn70ap/eeprom.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int beacon_usage(void)
{
  printf(
    "beacon [device] <ascii payload>\n"
  );
  return ERROR;
}

/****************************************************************************
 * status_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int beacon_main(int argc, char *argv[])
#endif
{
  char *devname = "/dev/raux";
  char *data;
  int ret = OK;
  int fd;
  int buflen = 1024;
  uint32_t seqnum = 0;
  char call[9];
  uint8_t ssid;

  uint8_t *buf;

  if(argc == 3)
    {
      devname = argv[1];
      data    = argv[2];
    }
  else if(argc == 2)
    {
      data = argv[1];
    }
  else
    {
      return beacon_usage();
    }

  buf = malloc(buflen);
  if(!buf)
    {
      fprintf(stderr, "malloc failed!\n");
      ret = ERROR;
      goto done;
    }


  fd = open(devname, O_RDWR);
  if(fd<0)
    {
      fprintf(stderr, "open failed!\n");
      ret = ERROR;
      goto retfree;
    }

  printf("\nTX beacon using %s\n", devname);

  /* Define payload */
  hn70ap_eeconfig_getcall("call", call);
  call[8] = 0;
  hn70ap_eeconfig_getbyte("ssid", &ssid);

  if(strlen(call)==0)
    {
      fprintf(stderr, "call sign not defined, see config app\n");
      ret = ERROR;
      goto retclose;
    }

  while(1)
    {
      sprintf(buf, "DE %s/%d HN70AP TEST BEACON SEQ %u: %s\n", call, ssid, seqnum, data);
      printf("%s", buf);
      ret = write(fd, buf, strlen(buf));
      if(ret < 0)
        {
          printf("write failed, errno=%d\n", errno);
          break;
        }
      sleep(1);
      seqnum += 1;
    }

retclose:
  close(fd);
retfree:
  free(buf);
done:
  return ret;
}


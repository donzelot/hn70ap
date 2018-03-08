/****************************************************************************
 * hn70ap/apps/config/config_list.c
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
#include <stdbool.h>

#include <arpa/inet.h>

#include <hn70ap/eeprom.h>

#include "config_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int config_list(void)
{
  int i = 0;
  int ret = OK;
  char name[16];
  uint32_t type;

  while(ret == OK)
    {
      ret = hn70ap_eeconfig_describe(i, name, sizeof(name), &type);
      if(ret == OK)
        {
          printf("name:%s type:%d value:", name, type);
          if(type == EECONFIG_TYPE_BOOL)
            {
              bool val;
              ret = hn70ap_eeconfig_getbool(name, &val);
              if(ret == OK)
                {
                  printf("%s\n", val?"true":"false");
                }
            }
          else if(type == EECONFIG_TYPE_IP)
            {
              struct in_addr val;
              ret = hn70ap_eeconfig_getip(name, &val);
              if(ret == OK)
                {
                  printf("%s\n", inet_ntoa(val));
                }
            }
          if(ret != OK)
            {
              printf("<unreadable>\n");
              ret = OK;
            }
        }
      i++;
    }
  return OK;
}


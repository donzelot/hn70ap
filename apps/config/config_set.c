/****************************************************************************
 * hn70ap/apps/config/config_set.c
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
#include <string.h>

#include <hn70ap/eeprom.h>

#include "config_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int config_set(char *key, char *value)
{
  char name[16];
  uint32_t type;
  int ret;

  /* Get config entry */
  ret = hn70ap_eeconfig_describe(hn70ap_eeconfig_find(name), name, sizeof(name), &type);

  if(ret != OK)
    {
      fprintf(stderr, "Config entry not found\n");
      return ERROR;
    }

  if(type == EECONFIG_TYPE_BOOL)
    {
      bool val;
      printf("Entry type is bool\n");
      if(!strcmp(value, "true") ||
         !strcmp(value, "True") ||
         !strcmp(value, "TRUE") ||
         !strcmp(value, "yes") ||
         !strcmp(value, "Yes") ||
         !strcmp(value, "YES") ||
         !strcmp(value, "y") ||
         !strcmp(value, "Y") ||
         !strcmp(value, "1")
      )
        {
        val = true;
        }
      else if(!strcmp(val, "false") ||
         !strcmp(value, "False") ||
         !strcmp(value, "FALSE") ||
         !strcmp(value, "no") ||
         !strcmp(value, "No") ||
         !strcmp(value, "NO") ||
         !strcmp(value, "n") ||
         !strcmp(value, "N") ||
         !strcmp(value, "0")
      )
        {
        val = false;
        }
      else
        {
          printf("Unrecognized boolean value: %s\n",val);
          return ERROR;
        }
      ret = hn70ap_eeconfig_setbool(name, val);
    }
  else if(type == EECONFIG_TYPE_IP)
    {
      in_addr_t val;
      printf("Entry type is IPv4\n");
      val = inet_aton(value);
      ret = hn70ap_eeconfig_setip(name, val);
    }
  else if(type == EECONFIG_TYPE_CALL)
    {
      printf("Entry type is call\n");
    }
  else if(type == EECONFIG_TYPE_BYTE)
    {
      printf("Entry type is byte\n");
    }
  else
    {
      fprintf(stderr, "Config entry not found\n");
      return ERROR;
    }

  return ret;
}


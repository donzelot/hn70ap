/****************************************************************************
 * hn70ap/apps/libhn70ap/eeprom.c
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

/* EEPROM configuration management */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>

#include <netinet/in.h>

#include <hn70ap/eeprom.h>

struct param_s
{
  const char *name; /* Config name */
  int type;         /* Config type */
  int addr;         /* Storage address of first byte in EEPROM */
  int bit;          /* For boolean configs, mask value of config */
};

/* EEPROM configuration
 * Address length Default Name Type Desciption
 * 0     : Config validation marker. If FF, EEPROM is virgin, and default config is loaded
 * 1-127 : Config entries described below
 */

const struct param_s eeconfig_params[] = 
{
  {"dhcp", TYPE_BOOL, 1, 0x01}, /* 0x01 - 0x01 DHCP Client Enable - board will request an IPv4 address via DHCP */
                                /* 0x02 - 0x03 RFU for more network options */
  {"ip"  , TYPE_IP  , 4,    0}, /* 0x04 - 0x07 Static IP address to use if DHCP is not enabled */
  {"mask", TYPE_IP  , 8,    0}, /* 0x08 - 0x0B Static IP mask to use if DHCP is not enabled */
  {"gw"  , TYPE_IP  , 12,   0}, /* 0x0C - 0x0F IP address of default gateway */
  {"dns" , TYPE_IP  , 16,   0}, /* 0x10 - 0x13 IP address of DNS server (will be used if set even if DHCP is enabled)*/
                                /* 0x14 - 0x7F RFU for more network options */
};

#define EECONFIG_PARAMS_COUNT (sizeof(eeconfig_params) / sizeof(eeconfig_params[0]))

/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_init(void)
{
  return 0;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_describe(int index, char *namebuf, int namebuflen, uint32_t *type)
{
  if(index < 0) return ERROR;
  if(index >= EECONFIG_PARAMS_COUNT) return ERROR;
  if(namebuf == NULL) return ERROR;

  strncpy(namebuf, eeconfig_params[index].name, namebuflen);
  if(type)
    {
      *type = eeconfig_params[index].type;
    }

  return OK;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_getbool(char *name, bool *value)
{
  return ERROR;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_getip(char *name, struct in_addr *value)
{
  return ERROR;
}


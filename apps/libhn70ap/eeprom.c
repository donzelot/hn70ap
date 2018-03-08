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

#include <fcntl.h>
#include <unistd.h>

#include <netinet/in.h>

#include <hn70ap/eeprom.h>

struct param_s
{
  const char *name; /* Config name */
  int type;         /* Config type */
  int addr;         /* Storage address of first byte in EEPROM */
  int mask;         /* For boolean configs, mask value of bit config */
};

/* EEPROM configuration
 * Address length Default Name Type Desciption
 * 0     : Config validation marker. If FF, EEPROM is virgin, and default config is loaded
 * 1-127 : Config entries described below
 */

const struct param_s eeconfig_params[] = 
{
  {"dhcp", EECONFIG_TYPE_BOOL, 1, 0x01}, /* 0x01 - 0x01 DHCP Client Enable */
                                        /* 0x02 - 0x03 RFU for more network options */
  {"ip"  , EECONFIG_TYPE_IP  , 4,    0}, /* 0x04 - 0x07 Static IP address to use if DHCP is not enabled */
  {"mask", EECONFIG_TYPE_IP  , 8,    0}, /* 0x08 - 0x0B Static IP mask to use if DHCP is not enabled */
  {"gw"  , EECONFIG_TYPE_IP  , 12,   0}, /* 0x0C - 0x0F IP address of default gateway */
  {"dns" , EECONFIG_TYPE_IP  , 16,   0}, /* 0x10 - 0x13 IP address of DNS server (independent of DHCP enable)*/
                                         /* 0x14 - 0x7F RFU for more network options */
};

#define EECONFIG_PARAMS_COUNT (sizeof(eeconfig_params) / sizeof(eeconfig_params[0]))

#define HN70AP_CONFIG_INSTALLED 0x42 //Vlue set at address zero of eeprom if
/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_init(bool *default_set)
{
  uint8_t buf;
  int ret;

  *default_set = false;

  /* Read the configuration enabled marker */
  ret = hn70ap_eeprom_read(0, &buf, 1);
  if(ret != OK)
    {
      return ret;
    }

  /* Not set? Set defaults */
  if( buf != HN70AP_CONFIG_INSTALLED)
    {
    *default_set = true;
    buf = HN70AP_CONFIG_INSTALLED;
    hn70ap_eeprom_write(0, &buf, 1);
    }

  /* Done */

  return 0;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeprom_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  int fd;

  fd = open(HN70AP_EECONFIG_DEVICE, O_RDONLY);
  if(fd < 0)
    {
      return ERROR;
    }
  if(lseek(fd, addr, SEEK_SET) != OK)
    {
      close(fd);
      return ERROR;
    }
  if(read(fd, buf, len) != len)
    {
      close(fd);
      return ERROR;
    }
  close(fd);
  return OK;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeprom_write(uint32_t addr, uint8_t *buf, uint32_t len)
{
  int fd;

  fd = open(HN70AP_EECONFIG_DEVICE, O_WRONLY);
  if(fd < 0)
    {
      return ERROR;
    }
  if(lseek(fd, addr, SEEK_SET) != OK)
    {
      close(fd);
      return ERROR;
    }
  if(write(fd, buf, len) != len)
    {
      close(fd);
      return ERROR;
    }
  close(fd);
  return OK;
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
static int hn70ap_eeconfig_find(char *name)
{
  int i;
  for(i = 0; i < EECONFIG_PARAMS_COUNT; i++)
    {
      if(!strcmp(name, eeconfig_params[i].name))
        {
          return i;
        }
    }
  return -1;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_getbool(char *name, bool *value)
{
  int index = hn70ap_eeconfig_find(name);
  int ret;
  uint8_t buf;
  if(index < 0)
    {
      return ERROR;
    }
  if(eeconfig_params[index].type != EECONFIG_TYPE_BOOL)
    {
      return ERROR;
    }
  ret = hn70ap_eeprom_read(eeconfig_params[index].addr, &buf, 1);
  if(ret != OK)
    {
      return ERROR;
    }

  *value = (buf & eeconfig_params[index].mask) == eeconfig_params[index].mask;

  return OK;
}

/*----------------------------------------------------------------------------*/
int hn70ap_eeconfig_getip(char *name, struct in_addr *value)
{
  int index = hn70ap_eeconfig_find(name);
  int ret;
  if(index < 0)
    {
      return ERROR;
    }
  if(eeconfig_params[index].type != EECONFIG_TYPE_IP)
    {
      return ERROR;
    }

  ret = hn70ap_eeprom_read(eeconfig_params[index].addr, (uint8_t*)value, 4);
  if(ret != OK)
    {
      return ERROR;
    }

  return OK;
}


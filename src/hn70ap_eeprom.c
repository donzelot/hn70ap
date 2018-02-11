/****************************************************************************
 * config/hn70ap/src/hn70ap_eeprom.c
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/eeprom/i2c_xx24xx.h>

#include "stm32.h"
#include "hn70ap.h"

int hn70ap_eeprom_initialize(void)
{
  struct i2c_master_s *i2c3;
  int ret;

  i2c3 = stm32_i2cbus_initialize(3);
  if (!i2c3)
    {
      _err("ERROR: Failed to initialize I2C port 3\n");
      return -ENODEV;
    }

#if defined(CONFIG_I2C_DRIVER)
  /* register a test i2c char driver for tests */
  ret = i2c_register(i2c3, 3);
  if(ret != 0)
    {
      _err("Failed to register char driver for i2c3\n");
      return -EIO;
    }
  syslog(LOG_INFO, "Registered i2c bus device /dev/i2c3\n");
#endif

#if defined(CONFIG_I2C_EE_24XX)
  ret = ee24xx_initialize(i2c3, 0x50, "/dev/eeprom", EEPROM_24xx02, false);
  if(ret != 0)
    {
      _err("Failed to register char driver for eeprom\n");
      return -EIO;
    }
  syslog(LOG_INFO, "Registered /dev/eeprom\n");
#endif

  return 0;
}


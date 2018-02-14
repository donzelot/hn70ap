/****************************************************************************
 * config/hn70ap/src/hn70ap_oled.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "hn70ap.h"

#ifdef CONFIG_VIDEO_FB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_LCD_SSD1306
#  error "The OLED driver requires CONFIG_LCD_SSD1306 in the configuration"
#endif

#ifndef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

#define OLED_I2C_PORT         1 /* OLED display connected to I2C1 */

/****************************************************************************
 * Private Data
 ****************************************************************************/
FAR struct lcd_dev_s    *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* Initialize I2C */
  FAR struct i2c_master_s *i2c3 = stm32_i2cbus_initialize(3);
  int devno = 0;

  if (!i2c3)
    {
      lcderr("ERROR: Failed to initialize I2C port 3\n");
      return -EIO;
    }

  /* Bind the I2C port to the OLED */

  g_lcddev = ssd1306_initialize(i2c3, NULL, devno);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind I2C port 3 to OLED %d: %d\n", devno);
      return -EIO;
    }
  lcdinfo("Bound I2C port 3 to OLED %d\n", devno);

  /* And turn the OLED on */

  g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

  return 0;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if(devno == 0) return g_lcddev;
  return NULL; //unknown device
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* TO-FIX */
}

#endif /* CONFIG_VIDEO_FB */


/****************************************************************************
 * hn70ap/apps/libhn70ap/system.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <syslog.h>

#include <net/if.h>

#include <hn70ap/eeprom.h>
#include <hn70ap/timer.h>
#include <hn70ap/leds.h>
#include <hn70ap/lcd.h>
#include <hn70ap/radio.h>
#include <hn70ap/tun.h>

static bool hn70ap_system_initialized = false;

int hn70ap_system_init(void)
{
  int ret;
  int tunid = -1;
  bool defaults;
  char tunname[IFNAMSIZ];

  if(hn70ap_system_initialized)
    {
      syslog(LOG_WARNING, "System already initialized\n");
      return 0;
    }

  /* Initialize timer thread to help us schedule delays */

  ret = hn70ap_timer_init();
  if(ret != 0)
    {
      syslog(LOG_ERR, "FATAL: Failed to initialize timers\n");
      return ERROR;
    }

  /* Initialize the leds */
  /* Requires timer for blinking */

  ret = hn70ap_leds_init();
  if(ret != 0)
    {
      syslog(LOG_ERR, "FATAL: Failed to initialize Leds\n");
      return ERROR;
    }

  /* Initialize the EEPROM */

  ret = hn70ap_eeconfig_init(&defaults);
  if(ret != 0)
    {
      syslog(LOG_ERR, "FATAL: Failed to initialize EEPROM\n");
      return ERROR;
    }
  if(defaults)
    {
      syslog(LOG_ERR, "WARNING: Default config values loaded in EEPROM\n");
    }

  ret = hn70ap_lcd_init();
  if(ret != OK)
    {
      syslog(LOG_ERR, "WARNING: Failed to initialize Screen\n");
    }
#ifdef CONFIG_NET_TUN
  ret = hn70ap_tun_init();
  if(ret != 0)
    {
      syslog(LOG_ERR, "WARNING: Failed to initialize tunnels\n");
    }

  strncpy(tunname, "uhf0", IFNAMSIZ);
#if 1
  ret = hn70ap_tun_devinit(tunname);
  if(ret < 0)
    {
      syslog(LOG_ERR, "WARNING: Failed to initialize TUN interface\n");
    }
  tunid = ret;
#endif
#endif

  ret = hn70ap_radio_init();
  if(ret != 0)
    {
      syslog(LOG_ERR, "WARNING: Failed to initialize Radios\n");
    }


  // bind the tunnel and the aux radio
  hn70ap_system_initialized = true;

  return OK;
}


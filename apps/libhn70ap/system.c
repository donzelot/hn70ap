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

int hn70ap_systeminit(void)
{
  int ret;

  /* Initialize timer thread to help us schedule delays */

  ret = timer_init();
  if(ret != 0)
    {
      printf("FATAL: Failed to initialize timers\n");
      goto lfail;
    }

  /* Initialize the leds */
  /* Requires timer for blinking */

  ret = leds_init();
  if(ret != 0)
    {
      printf("FATAL: Failed to initialize Leds\n");
      goto lfail;
    }

  /* Initialize the EEPROM */

  hn70ap_eeconfig_init(&defaults);
  if(defaults)
    {
      printf("WARNING: Default config values loaded in EEPROM\n");
    }
  else
    {
      hn70ap_eeconfig_getcall("call", call);
      if(call[0] == 0)
        {
          printf("Callsign not defined yet, please use the config tool\n");
        }
      else
        {
          call[8] = 0;
          printf("Hello %s, best 73's\n", call);
        }
    }

  return ret;
}


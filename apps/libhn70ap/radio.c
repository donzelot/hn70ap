/****************************************************************************
 * hn70ap/apps/libhn70ap/radio.c
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

#include <fcntl.h>
#include <syslog.h>

#include <hn70ap/radio.h>
#include <hn70ap/leds.h>

static int fd_main;
static int fd_aux;

int hn70ap_radio_init(void)
{
  int ret = 0;

#ifdef CONFIG_HN70AP_MAINRADIO
  syslog(LOG_INFO, "Checking main radio\n");
  fd_main = open("/dev/rmain", O_RDWR);
  if(fd_main<0)
    {
      syslog(LOG_ERR, "Failed to access main radio!\n");
    }
#endif
#ifdef CONFIG_HN70AP_AUXRADIO
  syslog(LOG_INFO, "Checking aux radio\n");
  fd_aux = open("/dev/raux", O_RDWR);
  if(fd_aux<0)
    {
      syslog(LOG_ERR, "Failed to access aux radio!\n");
    }
#endif

  return ret;
}

int hn70ap_radio_transmit(uint8_t device, uint8_t *buf, size_t len)
{
  int fd;
  int ret;

  if(device == HN70AP_RADIO_MAIN)
    {
      fd = fd_main;
    }
  else if(device == HN70AP_RADIO_AUX)
    {
      fd = fd_aux;
    }
  else
    {
      return ERROR;
    }

  /* Turn on radio LED in transmit mode */
  hn70ap_leds_state(LED_1A, LED_STATE_ON);
  hn70ap_leds_state(LED_1B, LED_STATE_OFF);

  /* Transmit */
  ret = write(fd, buf, len);

  /* Turn off radio LED */
  hn70ap_leds_state(LED_1A, LED_STATE_OFF);
  hn70ap_leds_state(LED_1B, LED_STATE_OFF);

  return ret;
}


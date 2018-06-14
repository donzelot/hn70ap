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
#include <unistd.h>
#include <pthread.h>

#include <hn70ap/radio.h>
#include <hn70ap/leds.h>

/* management variables for one radio */
struct radio_s
{
  int devfd;
  bool alive;
  pthread_t txthread;
  pthread_t rxthread;
};

static struct radio_s g_hn70ap_mainradio;
static struct radio_s g_hn70ap_auxradio;

/****************************************************************************
 * hn70ap_radio_txthread
 * This thread waits for radio transmit requests from other processes,
 * and sends packets.
 ****************************************************************************/

void *hn70ap_radio_txthread(void *arg)
{
  struct radio_s *radio = (struct radio_s*)arg;
  syslog(LOG_INFO, "Started radio TX thread\n");
  while(radio->alive)
    {
      //Wait for messages in transmit queue
      //Transmit these messages
    }
  syslog(LOG_INFO, "Stopped radio TX thread\n");
}

/****************************************************************************
 * hn70ap_radio_rxthread
 * This thread waits for radio packets on the air, and sends them to processes.
 ****************************************************************************/

void *hn70ap_radio_rxthread(void *arg)
{
  struct radio_s *radio = (struct radio_s*)arg;
  syslog(LOG_INFO, "Started radio RX thread\n");
  while(radio->alive)
    {
      //Wait for messages on the air
      //Put these messages on the queue
    }
  syslog(LOG_INFO, "Stopped radio RX thread\n");
}

/****************************************************************************
 * hn70ap_radio_transmit
 ****************************************************************************/

int hn70ap_radio_transmit(uint8_t device, uint8_t *buf, size_t len)
{
  struct radio_s *radio;
  int ret;

  if(device == HN70AP_RADIO_MAIN)
    {
      radio = &g_hn70ap_mainradio;
    }
  else if(device == HN70AP_RADIO_AUX)
    {
      radio = &g_hn70ap_auxradio;
    }
  else
    {
      return ERROR;
    }

  /* Turn on radio LED in transmit mode */
  hn70ap_leds_state(LED_1A, LED_STATE_ON);
  hn70ap_leds_state(LED_1B, LED_STATE_OFF);

  /* Transmit */
  ret = write(radio->devfd, buf, len);

  /* Turn off radio LED */
  hn70ap_leds_state(LED_1A, LED_STATE_OFF);
  hn70ap_leds_state(LED_1B, LED_STATE_OFF);

  return ret;
}

/****************************************************************************
 * hn70ap_radio_receive
 ****************************************************************************/

int hn70ap_radio_receive(uint8_t device, uint8_t *buf, size_t len)
{
  struct radio_s *radio;
  int ret;

  if(device == HN70AP_RADIO_MAIN)
    {
      radio = &g_hn70ap_mainradio;
    }
  else if(device == HN70AP_RADIO_AUX)
    {
      radio = &g_hn70ap_auxradio;
    }
  else
    {
      return ERROR;
    }

  /* Turn on radio LED in transmit mode */
  hn70ap_leds_state(LED_1A, LED_STATE_OFF);
  hn70ap_leds_state(LED_1B, LED_STATE_ON);

  /* Transmit */
  ret = read(radio->devfd, buf, len);

  /* Turn off radio LED */
  hn70ap_leds_state(LED_1A, LED_STATE_OFF);
  hn70ap_leds_state(LED_1B, LED_STATE_OFF);

  return ret;
}

/****************************************************************************
 * hn70ap_radio_devinit
 ****************************************************************************/

int hn70ap_radio_devinit(struct radio_s *radio, const char *dev)
{
  int ret = 0;

  radio->devfd = open("/dev/rmain", O_RDWR);
  if(radio->devfd<0)
    {
      syslog(LOG_ERR, "Failed to access main radio!\n");
      ret = -1;
      goto lret;
    }

  ret = pthread_create(&radio->txthread, NULL, hn70ap_radio_txthread, NULL);
  if(ret < 0)
    {
      syslog(LOG_ERR, "Failed to start the transmit thread\n");
    }

lret:
  return ret;
}

/****************************************************************************
 * hn70ap_radio_init
 ****************************************************************************/

int hn70ap_radio_init(void)
{
  int ret = 0;

#ifdef CONFIG_HN70AP_MAINRADIO
  syslog(LOG_INFO, "Checking main radio\n");
  hn70ap_radio_devinit(&g_hn70ap_mainradio, "/dev/rmain");

#endif
#ifdef CONFIG_HN70AP_AUXRADIO
  syslog(LOG_INFO, "Checking aux radio\n");
  hn70ap_radio_devinit(&g_hn70ap_auxradio, "/dev/raux");
#endif

}


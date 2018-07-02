/****************************************************************************
 * hn70ap/apps/libhn70ap/tun.c
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

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <syslog.h>

#include <sys/ioctl.h>

#include <nuttx/net/tun.h>

#include <hn70ap/tun.h>

struct iptunnel_s
{
  int       fd;
  char      ifname[IFNAMSIZ];
  bool      alive;
  pthread_t rxthread;

  /*data reception and calling back to the user*/
  FAR uint8_t *   userbuf;
  int             userbuflen;
  tunrxfunction_f callback;
  FAR void *      arg;
};

struct iptunnel_s tunnels[2];

/****************************************************************************
 * hn70ap_tun_rxthread
 * This thread waits for packets from the tun interface, and sends them to processes.
 ****************************************************************************/

void *hn70ap_tun_rxthread(void *arg)
{
  struct iptunnel_s *tunnel = (struct iptunnel_s*)arg;
  syslog(LOG_INFO, "Started tun RX thread\n");
  int ret;

  while(tunnel->alive)
    {
      if(tunnel->callback)
        {
          //Wait for messages on the air
          ret = read(tunnel->fd, tunnel->userbuf, tunnel->userbuflen);
          if(ret > 0)
            {
              //Dispatch them to the callback
              tunnel->callback((tunnel==tunnels)?0:1, tunnel->arg, tunnel->userbuf, ret);
            }
          else
            {
              //syslog(LOG_ERR, "tunnel rx failed -> errno=%d\n", errno);
            }
        } //callback defined
      else
       {
         pthread_yield();
       }
    } //tunnel alive
  syslog(LOG_INFO, "Stopped tunnel RX thread\n");
  return NULL;
}

/****************************************************************************
 * hn70ap_tun_transmit
 ****************************************************************************/
int hn70ap_tun_transmit(int tunnel, FAR uint8_t *buf, size_t len)
{
  if(tunnel != 0 && tunnel != 1)
    {
      return -1;
    }

  return write(tunnels[tunnel].fd, buf, len);
}

/****************************************************************************
 * hn70ap_tun_rxfunction
 ****************************************************************************/

int hn70ap_tun_rxfunction(int tunnel, tunrxfunction_f rx, FAR void *arg, FAR uint8_t *userbuf, int userbuflen)
{
  if(tunnel != 0 && tunnel != 1)
    {
      return -1;
    }

  tunnels[tunnel].userbuf    = userbuf;
  tunnels[tunnel].userbuflen = userbuflen;
  tunnels[tunnel].arg        = arg;
  tunnels[tunnel].callback   = rx;

  return OK;

}

/****************************************************************************
 * hn70ap_tun_devinit
 ****************************************************************************/

int hn70ap_tun_devinit(char name[IFNAMSIZ])
{
  struct ifreq       ifr;
  int                errcode;
  int                fd;
  int                ret;
  struct iptunnel_s *tunnel = &tunnels[0];

  if(tunnel->fd != 0)
    {
      tunnel = &tunnels[1];
    }

  if(tunnel->fd != 0)
    {
      syslog(LOG_ERR, "No tunnel available");
      return -1;
    }

  if ((fd = open("/dev/tun", O_RDWR)) < 0)
    {
      return fd;
    }

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TUN;
  if (name[0])
    {
      strncpy(ifr.ifr_name, name, IFNAMSIZ);
    }

  errcode = ioctl(fd, TUNSETIFF, (unsigned long)&ifr);
  if(errcode < 0)
    {
      close(fd);
      return errcode;
    }
  tunnel->fd = fd;
  strncpy(tunnel->ifname, ifr.ifr_name, IFNAMSIZ);

  syslog(LOG_INFO, "Started interface: %s\n", tunnel->ifname);

  //Start RX thread
  tunnel->alive = true;
  tunnel->callback = NULL;
  ret = pthread_create(&tunnel->rxthread, NULL, hn70ap_tun_rxthread, tunnel);
  if(ret < 0)
    {
      syslog(LOG_ERR, "Failed to start the receive thread\n");
    }

  return fd;
}

/****************************************************************************
 * hn70ap_tun_addroute
 ****************************************************************************/

int hn70ap_tun_addroute(int tunnel, in_addr_t destination, int maskbits)
{
  return ERROR;
}

/****************************************************************************
 * hn70ap_tun_init
 ****************************************************************************/

int hn70ap_tun_init(void)
{
  tunnels[0].fd = 0;
  tunnels[1].fd = 0;

  return 0;
}


/****************************************************************************
 * hn70ap/apps/sysdaemon/sysdaemon_netmonitor.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/leds/userled.h>

#include <netutils/netlib.h>
#include <netutils/dhcpc.h>

#include <hn70ap/leds.h>

#define NET_DEVNAME "eth0"
#define NETMONITOR_RETRYMSEC 1000
#define SHORT_TIME_SEC 1
#define LONG_TIME_SEC 10

static sem_t g_notify_sem;

/* DHCP stuff */
struct dhcpc_state g_dhcp_state;

/*----------------------------------------------------------------------------*/
static void dhcp_negociate(void)
{
  void *handle;
  uint8_t mac[IFHWADDRLEN];
  int ret;

  netlib_getmacaddr(NET_DEVNAME, mac);
  handle = dhcpc_open(NET_DEVNAME, &mac, IFHWADDRLEN);

  if(!handle)
    {
      syslog(LOG_ERR, "DHCP client failed\n");
      return;
      }

  syslog(LOG_INFO, "Starting DHCP request\n");
  ret = dhcpc_request(handle, &g_dhcp_state);

  if(ret != 0)
    {
      syslog(LOG_INFO, "DHCP request failed: %d errno %d\n", ret, errno);
      goto close;
    }

  /* Apply the result */
  syslog(LOG_INFO, "IP addr: %d.%d.%d.%d\n",
       g_dhcp_state.ipaddr.s_addr        & 0xff,
      (g_dhcp_state.ipaddr.s_addr >>  8) & 0xff,
      (g_dhcp_state.ipaddr.s_addr >> 16) & 0xff,
       g_dhcp_state.ipaddr.s_addr >> 24);
  netlib_set_ipv4addr(NET_DEVNAME, &g_dhcp_state.ipaddr);

  if (g_dhcp_state.netmask.s_addr != 0)
    {
      syslog(LOG_INFO, "Net mask: %d.%d.%d.%d\n",
           g_dhcp_state.netmask.s_addr        & 0xff,
          (g_dhcp_state.netmask.s_addr >>  8) & 0xff,
          (g_dhcp_state.netmask.s_addr >> 16) & 0xff,
           g_dhcp_state.netmask.s_addr >> 24);
      netlib_set_ipv4netmask(NET_DEVNAME, &g_dhcp_state.netmask);
    }

  if (g_dhcp_state.default_router.s_addr != 0)
    {
      syslog(LOG_INFO, "Default router: %d.%d.%d.%d\n",
           g_dhcp_state.default_router.s_addr        & 0xff,
          (g_dhcp_state.default_router.s_addr >>  8) & 0xff,
          (g_dhcp_state.default_router.s_addr >> 16) & 0xff,
           g_dhcp_state.default_router.s_addr >> 24);
      netlib_set_dripv4addr(NET_DEVNAME, &g_dhcp_state.default_router);
    }

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NETDB_DNSCLIENT)
  if (g_dhcp_state.dnsaddr.s_addr != 0)
    {
      syslog(LOG_INFO, "DNS: %d.%d.%d.%d\n",
           g_dhcp_state.dnsaddr.s_addr        & 0xff,
          (g_dhcp_state.dnsaddr.s_addr >>  8) & 0xff,
          (g_dhcp_state.dnsaddr.s_addr >> 16) & 0xff,
           g_dhcp_state.dnsaddr.s_addr >> 24);
      netlib_set_ipv4dnsaddr(&g_dhcp_state.dnsaddr);
    }
#endif

close:
  dhcpc_close(handle);
}

/*----------------------------------------------------------------------------*/
static void netmonitor_ifup(void* arg)
{
  syslog(LOG_INFO, "Interface is going UP\n");

  /* Enable the link LED */
  hn70ap_leds_state(LED_MACLINK, LED_STATE_ON);

  dhcp_negociate();
}

/*----------------------------------------------------------------------------*/
static void netmonitor_ifdown(void)
{
  syslog(LOG_INFO, "Interface is going DOWN\n");

  /* Disable the link LED */
  hn70ap_leds_state(LED_MACLINK, LED_STATE_OFF);
}

/*----------------------------------------------------------------------------*/
/*
Description: Signal handler to be notified of link status change
*/
static void netmonitor_signal(int signo, FAR siginfo_t *siginfo,
                               FAR void * context)
{
  int semcount;
  int ret;

  /* What is the count on the semaphore?  Don't over-post */

  ret = sem_getvalue(&g_notify_sem, &semcount);

  if (ret == OK && semcount <= 0)
    {
      sem_post(&g_notify_sem);
    }
}

/*----------------------------------------------------------------------------*/
/*
 * Description:
 *   Monitor link status, gracefully taking the link up and down as the
 *   link becomes available or as the link is lost.
*/
static void* netmonitor_thread(void *arg)
{
  struct timespec abstime;
  struct timespec reltime;
  struct ifreq ifr;
  struct sigaction act;
  bool devup;
  int ret;
  int sd;

  syslog(LOG_INFO, "Entry\n");

  /* Initialize the notification semaphore */

  DEBUGVERIFY(sem_init(&g_notify_sem, 0, 0));

  /* Get a socket descriptor that we can use to communicate with the network
   * interface driver.
   */

  sd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sd < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      syslog(LOG_ERR, "Failed to create a socket: %d\n", ret);
      goto errout;
    }

  /* Attach a signal handler so that we do not lose PHY events */

  act.sa_sigaction = netmonitor_signal;
  act.sa_flags     = SA_SIGINFO;

  ret = sigaction(SIGUSR2, &act, NULL);
  if (ret < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      syslog(LOG_ERR, "sigaction() failed: %d\n", ret);
      goto errout_with_socket;
    }

  /* Now loop, waiting for changes in link status */

  for (;;)
    {
      /* Configure to receive a signal on changes in link status */

      strncpy(ifr.ifr_name, NET_DEVNAME, IFNAMSIZ);

      ifr.ifr_mii_notify_pid   = 0; /* PID=0 means this task */
      ifr.ifr_mii_notify_signo = SIGUSR2;
      ifr.ifr_mii_notify_arg   = NULL;

      ret = ioctl(sd, SIOCMIINOTIFY, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          syslog(LOG_ERR, "ioctl(SIOCMIINOTIFY) failed: %d\n", ret);
          goto errout_with_sigaction;
        }

      /* Does the driver think that the link is up or down? */

      ret = ioctl(sd, SIOCGIFFLAGS, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          syslog(LOG_ERR, "ioctl(SIOCGIFFLAGS) failed: %d\n", ret);
          goto errout_with_notification;
        }

      devup = ((ifr.ifr_flags & IFF_UP) != 0);

      /* Get the current PHY address in use.  This probably does not change,
       * but just in case...
       *
       * NOTE: We are assuming that the network device name is preserved in
       * the ifr structure.
       */

      ret = ioctl(sd, SIOCGMIIPHY, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;

          syslog(LOG_ERR, "ioctl(SIOCGMIIPHY) failed: %d\n", ret);
          goto errout_with_notification;
        }

      /* Read the PHY status register */

      ifr.ifr_mii_reg_num = MII_MSR;

      ret = ioctl(sd, SIOCGMIIREG, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          syslog(LOG_ERR, "ioctl(SIOCGMIIREG) failed: %d\n", ret);
          goto errout_with_notification;
        }

      //syslog(LOG_INFO, "%s: devup=%d PHY address=%02x MSR=%04x",
      //      ifr.ifr_name, devup, ifr.ifr_mii_phy_id, ifr.ifr_mii_val_out);

      /* Check for link up or down */

      if ((ifr.ifr_mii_val_out & MII_MSR_LINKSTATUS) != 0)
        {
          /* Link up... does the drive think that the link is up? */

          if (!devup)
            {
              /* No... We just transitioned from link down to link up.
               * Bring the link up.
               */

              syslog(LOG_INFO, "Bringing the link up\n");

              ifr.ifr_flags = IFF_UP;
              ret = ioctl(sd, SIOCSIFFLAGS, (unsigned long)&ifr);
              if (ret < 0)
                {
                  ret = -errno;
                  DEBUGASSERT(ret < 0);

                  syslog(LOG_ERR, "ioctl(SIOCSIFFLAGS) failed: %d\n", ret);
                  goto errout_with_notification;
                }
              netmonitor_ifup(arg);

              /* And wait for a short delay.  We will want to recheck the
               * link status again soon.
               */

              reltime.tv_sec  = SHORT_TIME_SEC;
              reltime.tv_nsec = 0;
            }
          else
            {
              /* The link is still up.  Take a long, well-deserved rest */

              reltime.tv_sec  = LONG_TIME_SEC;
              reltime.tv_nsec = 0;
            }
        }
      else
        {
          /* Link down... Was the driver link state already down? */

          if (devup)
            {
              /* No... we just transitioned from link up to link down.  Take
               * the link down.
               */

              syslog(LOG_INFO, "Taking the link down\n");

              ifr.ifr_flags = IFF_DOWN;
              ret = ioctl(sd, SIOCSIFFLAGS, (unsigned long)&ifr);
              if (ret < 0)
                {
                  ret = -errno;
                  DEBUGASSERT(ret < 0);

                  syslog(LOG_ERR, "ioctl(SIOCSIFFLAGS) failed: %d\n", ret);
                  goto errout_with_notification;
                }
              netmonitor_ifdown();
            }
          /* In either case, wait for the short, configurable delay */
          reltime.tv_sec  = NETMONITOR_RETRYMSEC / 1000;
          reltime.tv_nsec = (NETMONITOR_RETRYMSEC % 1000) * 1000000;
        }

      /* Now wait for either the semaphore to be posted for a timed-out to
       * occur.
       */

      sched_lock();
      DEBUGVERIFY(clock_gettime(CLOCK_REALTIME, &abstime));

      abstime.tv_sec  += reltime.tv_sec;
      abstime.tv_nsec += reltime.tv_nsec;
      if (abstime.tv_nsec >= 1000000000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000000000;
        }

      (void)sem_timedwait(&g_notify_sem, &abstime);
      sched_unlock();
    }

  /* TODO: Stop the PHY notifications and remove the signal handler. */

errout_with_notification:
#  warning Missing logic
errout_with_sigaction:
#  warning Missing logic
errout_with_socket:
  close(sd);
errout:
  syslog(LOG_ERR, "Aborting\n");
  return (void*)ret;
}

/*----------------------------------------------------------------------------*/
/*
Description: Initialize software update operation
    - Set mac address and hostname
    - Launch netmonitor to setup connectioin and start com
*/
int hn70ap_netmonitor_init(void)
{
  uint8_t mac[IFHWADDRLEN];
  int ret;
  int fd;

  //int hostnamelen;
  //char hostname[HOST_NAME_MAX+1];

  //sethostname(hostname, hostnamelen);

  fd = open("/dev/eeprom", O_RDONLY);
  if(fd<0)
    {
      syslog(LOG_ERR, "failed to open EEPROM to read MAC address\n");
      memcpy(mac, "123456", 6);
    }

  ret = lseek(fd, 0xFA, SEEK_SET);
  if(ret<0)
    {
      syslog(LOG_ERR, "failed to seek the EEPROM\n");
      return ERROR;
    }

  ret = read(fd, mac, 6);
  if(ret<0)
    {
      syslog(LOG_ERR, "failed to read the EEPROM\n");
      return ERROR;
    }

  close(fd);
  syslog(LOG_INFO, "Set MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  ret = netlib_setmacaddr(NET_DEVNAME, mac);
  if(ret)
    {
      syslog(LOG_ERR, "Set MAC on ethernet interface FAILED\n");
      return ret;
    }

  ret = pthread_create(NULL, NULL, netmonitor_thread, NULL);

  return ret;
}


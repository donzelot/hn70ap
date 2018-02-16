/****************************************************************************
 * config/hn70ap/src/hn70ap_ethernet.c
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
#include <nuttx/arch.h>
#include <debug.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include <arch/board/board.h>
#include "hn70ap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_phy_handler;
static void  *g_phy_arg;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hn70ap_net_initialize
 ****************************************************************************/

void hn70ap_net_initialize(void)
    {
    _info("Configuring PHY GPIOs\n");
    stm32_configgpio(GPIO_IRQ_ETHERNET);
    stm32_configgpio(GPIO_ETHERNET_RST);
    g_phy_handler = NULL;
    g_phy_arg     = NULL;
    }

/****************************************************************************
 * Name: stm32_phy_boardinitialize
 ****************************************************************************/

int stm32_phy_boardinitialize(int intf)
    {
    _info("called (intf=%d)\n", intf);

    _info("PHY reset...\n");
    stm32_gpiowrite(GPIO_ETHERNET_RST, 0);
    up_mdelay(1);
    _info("PHY reset done.\n");
    stm32_gpiowrite(GPIO_ETHERNET_RST, 1);
    up_mdelay(1);

    return 0;
    }

/****************************************************************************
 * Name: stm32_phy_enable
 ****************************************************************************/

static void stm32_phy_enable(bool enable)
    {
    //_info("PHY IRQ enable :%d\n", enable);
    if (enable)
        {
        stm32_gpiosetevent(GPIO_IRQ_ETHERNET, /*rising*/ FALSE, /*falling*/ TRUE, TRUE,
                           g_phy_handler, /*arg*/ g_phy_arg);
        }
    else
        {
        stm32_gpiosetevent(GPIO_IRQ_ETHERNET, /*rising*/ FALSE, /*falling*/ FALSE, FALSE,
                           NULL, /*arg*/ NULL);
        }
    }

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a PHY interrupt occurs.  This function both attaches
 *   the interrupt handler and enables the interrupt if 'handler' is non-
 *   NULL.  If handler is NULL, then the interrupt is detached and disabled
 *   instead.
 *
 *   The PHY interrupt is always disabled upon return.  The caller must
 *   call back through the enable function point to control the state of
 *   the interrupt.
 *
 *   This interrupt may or may not be available on a given platform depending
 *   on how the network hardware architecture is implemented.  In a typical
 *   case, the PHY interrupt is provided to board-level logic as a GPIO
 *   interrupt (in which case this is a board-specific interface and really
 *   should be called board_phy_irq()); In other cases, the PHY interrupt
 *   may be cause by the chip's MAC logic (in which case arch_phy_irq()) is
 *   an appropriate name.  Other other boards, there may be no PHY interrupts
 *   available at all.  If client attachable PHY interrupts are available
 *   from the board or from the chip, then CONFIG_ARCH_PHY_INTERRUPT should
 *   be defined to indicate that fact.
 *
 *   Typical usage:
 *   a. OS service logic (not application logic*) attaches to the PHY
 *      PHY interrupt and enables the PHY interrupt.
 *   b. When the PHY interrupt occurs:  (1) the interrupt should be
 *      disabled and () work should be scheduled on the worker thread (or
 *      perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *   d. It should then interact the the PHY to clear any pending
 *      interrupts, then re-enable the PHY interrupt.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.  Rather applications should use the SIOCMIISIG
 *      ioctl to receive a signal when a PHY event occurs.
 *   ** This interrupt is really of no use if the Ethernet MAC driver
 *      does not support these ioctl calls.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".  Only
 *             useful on platforms that support multiple Ethernet interfaces
 *             and, hence, multiple PHYs and PHY interrupts.
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  Must reside in OS space, but can
 *             signal tasks in user space.  A value of NULL can be passed
 *             in order to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be unsed to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/
int arch_phy_irq(FAR const char *intf, xcpt_t handler, void *arg, phy_enable_t *enable)
    {
    irqstate_t flags;

    /* Disable interrupts until we are done.  This guarantees that the
     * following operations are atomic.
     */

    flags = enter_critical_section();
  
    /* Get the old interrupt handler and save the new one */

    g_phy_handler = handler;
    g_phy_arg     = arg;
    
    if (handler)
        {
        _info("Attach PHY IRQ\n");
        *enable = stm32_phy_enable;
        }
    else
        {
        _info("Detach PHY IRQ\n");
        *enable = NULL;
        }

    /* Return with the interrupt disabled in either case */

    stm32_phy_enable(FALSE);

    leave_critical_section(flags);
    return 0;
    }


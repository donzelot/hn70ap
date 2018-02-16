/****************************************************************************
 * configs/hn70ap/src/hn70ap_genradio.c
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <nuttx/wireless/generic/si4463.h>
#include <nuttx/wireless/generic/genradio.h>

#include <stm32.h>
#include "hn70ap.h"

#if defined(CONFIG_STM32_SPI4) && defined(CONFIG_GENERICRADIO_SI4463)

struct si4463_priv_s
{
  uint32_t gpio_irq; /* GPIO definition used for IRQ */
  xcpt_t handler;
  FAR void *arg;
};

/*******************************************************************************
 * Name: si4463_attach
 *
 * Description:
 *   Initialize required resources to enable si4463 IRQs.
 *   Called from driver code.
 *
 ******************************************************************************/

int si4463_attach(FAR const struct si4463_lower_s *lower, xcpt_t handler, FAR void *arg)
{
  struct si4463_priv_s *priv = lower->privpointer;
  priv->handler = handler;
  priv->arg     = arg;
  stm32_configgpio(priv->gpio_irq);
  return 0;
}

/*******************************************************************************
 * Name: si4463_enable
 *
 * Description:
 *   Enable or disable the required IRQ. Called from driver code.
 *
 ******************************************************************************/

void si4463_enable(FAR const struct si4463_lower_s *lower, int state)
{
  struct si4463_priv_s *priv = lower->privpointer;
  if(state)
    {
      stm32_gpiosetevent(priv->gpio_irq, /*rising*/ false, /*falling*/ true, /*event*/ false, priv->handler, priv->arg);
    }
  else
    {
      stm32_gpiosetevent(priv->gpio_irq, /*rising*/ false, /*falling*/ false, /*event*/ false, NULL, NULL);
    }
}

struct si4463_priv_s si4463_priv_main;
struct si4463_priv_s si4463_priv_aux;

const struct si4463_lower_s si4463_lower_main =
{
  si4463_attach,
  si4463_enable,
  &si4463_priv_main
};

const struct si4463_lower_s si4463_lower_aux =
{
  si4463_attach,
  si4463_enable,
  &si4463_priv_aux
};

/*******************************************************************************
 * Name: hn70ap_genradio_initialize
 *
 * Description:
 *   Initialize the on board radio transceivers.
 *
 ******************************************************************************/

int hn70ap_genradio_initialize(void)
{
  FAR struct genradio_dev_s *radio;
  int ret;
  FAR struct spi_dev_s *spi4;

  /* Initialize SPI bus */

  spi4 = stm32_spibus_initialize(4);
  if (!spi4)
    {
      spierr("ERROR: FAILED to initialize SPI port 4\n");
    }

  /* Initialize radio devices */

  si4463_priv_main.gpio_irq = GPIO_IRQ_RADIOMAIN;
  radio = RFM26_init(spi4, 1, SI4463_IO0, SI4463_IO1, &si4463_lower_main);
  if(radio==NULL)
    {
      _err("Unable to initialize si4463\n");
    }
  else
    {
      ret = genradio_register(radio, "/dev/rmain");
      if(ret)
        {
          _err("Failed to register si4463 /dev/rmain\n");
        }
    }

  si4463_priv_aux.gpio_irq = GPIO_IRQ_RADIOAUX;
  radio = RFM26_init(spi4, 2, SI4463_IO0, SI4463_IO1, &si4463_lower_aux);
  if(radio==NULL)
    {
      _err("Unable to initialize si4463\n");
    }
  else
    {
      ret = genradio_register(radio, "/dev/raux");
      if(ret)
        {
          _err("Failed to register si4463 /dev/raux\n");
        }
    }

  return 0;
}
#endif


/*******************************************************************************
 * configs/hn70ap/src/stm32_leds.c
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Authors: Sebastien Lorquet <sebastien@lorquet.fr>
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
 ******************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "hn70ap.h"

static const uint32_t g_leds[BOARD_NLEDS] = 
{
  GPIO_LED_1A       ,
  GPIO_LED_1B       ,
  GPIO_LED_RED      ,
  GPIO_LED_ORANGE   ,
  GPIO_LED_GREEN    ,
  GPIO_LED_HEARTBEAT,
  GPIO_LED_MACLINK
};

/*----------------------------------------------------------------------------*/
void hn70ap_leds_initialize(void)
{
  stm32_configgpio(GPIO_LED_CPUACT);

  stm32_gpiowrite(GPIO_LED_CPUACT, 1);
}

/* USEFUL definitions for the LEDs lower half */

/*----------------------------------------------------------------------------*/
void board_userled_initialize(void)
{
  int i;

  for(i=0; i<sizeof(g_leds)/sizeof(g_leds[0]); i++)
    {
      stm32_configgpio(g_leds[i]);
    }

  stm32_gpiowrite(GPIO_LED_HEARTBEAT, 1);
  stm32_gpiowrite(GPIO_LED_GREEN, 0);
}

/*----------------------------------------------------------------------------*/
void board_userled(int led, bool ledon)
{
  if(led < 0) return;

  if(led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_leds[led], !ledon); //ledon requires pulling the line to gnd
    }
}

/*----------------------------------------------------------------------------*/
void board_userled_all(uint8_t ledset)
{
}

/* Fake definitions for the unused autoleds features of nuttx */
void board_autoled_on(int led) {}
void board_autoled_off(int led) {}


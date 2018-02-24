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

void hn70ap_leds_initialize(void)
  {
    stm32_configgpio(GPIO_LED_1A       );
    stm32_configgpio(GPIO_LED_1B       );
    stm32_configgpio(GPIO_LED_RED      );
    stm32_configgpio(GPIO_LED_ORANGE   );
    stm32_configgpio(GPIO_LED_GREEN    );
    stm32_configgpio(GPIO_LED_HEARTBEAT);
    stm32_configgpio(GPIO_LED_CPUACT   );
    stm32_configgpio(GPIO_LED_MACLINK  );

    stm32_gpiowrite(GPIO_LED_HEARTBEAT, 1);
    stm32_gpiowrite(GPIO_LED_CPUACT, 1);
    stm32_gpiowrite(GPIO_LED_GREEN, 0);

  }



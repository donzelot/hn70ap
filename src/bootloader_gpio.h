/*******************************************************************************
 * configs/hn70ap/src/bootloader_gpio.h
 *
 *   Copyright (C) 2016-2018 Sebastien Lorquet. All rights reserved.
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
 ******************************************************************************/

#ifndef BOOTLOADER_GPIO
#define BOOTLOADER_GPIO

/* format for flags
	offset size name
	0      4    LINE (0-15)
	4      4    PORT (0-10)
	8      4    ALT (0-15)
    12     2    MODER (0-3)
	14     2	PUPDR (0-2)
	16     2    OSPEEDR (0-3)
	18     1    OTYPER (0-1)
    19     1    INITSTATE (0-1)
*/

#define	GPIO_PIN_SHIFT   0
#define GPIO_PIN_MASK    (15 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_0       ( 0 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_1       ( 1 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_2       ( 2 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_3       ( 3 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_4       ( 4 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_5       ( 5 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_6       ( 6 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_7       ( 7 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_8       ( 8 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_9       ( 9 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_10      (10 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_11      (11 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_12      (12 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_13      (13 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_14      (14 << GPIO_PIN_SHIFT)
#define	GPIO_PIN_15      (15 << GPIO_PIN_SHIFT)

#define GPIO_PORT_SHIFT  4
#define GPIO_PORT_MASK   (15 << GPIO_PORT_SHIFT)
#define GPIO_PORT_A      ( 0 << GPIO_PORT_SHIFT)
#define GPIO_PORT_B      ( 1 << GPIO_PORT_SHIFT)
#define GPIO_PORT_C      ( 2 << GPIO_PORT_SHIFT)
#define GPIO_PORT_D      ( 3 << GPIO_PORT_SHIFT)
#define GPIO_PORT_E      ( 4 << GPIO_PORT_SHIFT)
#define GPIO_PORT_F      ( 5 << GPIO_PORT_SHIFT)
#define GPIO_PORT_G      ( 6 << GPIO_PORT_SHIFT)
#define GPIO_PORT_H      ( 7 << GPIO_PORT_SHIFT)
#define GPIO_PORT_I      ( 8 << GPIO_PORT_SHIFT)
#define GPIO_PORT_J      ( 9 << GPIO_PORT_SHIFT)
#define GPIO_PORT_K      (10 << GPIO_PORT_SHIFT)

#define	GPIO_ALT_SHIFT   8
#define GPIO_ALT_MASK    (15 << GPIO_ALT_SHIFT)
#define GPIO_ALT_0       ( 0 << GPIO_ALT_SHIFT)
#define GPIO_ALT_1       ( 1 << GPIO_ALT_SHIFT)
#define GPIO_ALT_2       ( 2 << GPIO_ALT_SHIFT)
#define GPIO_ALT_3       ( 3 << GPIO_ALT_SHIFT)
#define GPIO_ALT_4       ( 4 << GPIO_ALT_SHIFT)
#define GPIO_ALT_5       ( 5 << GPIO_ALT_SHIFT)
#define GPIO_ALT_6       ( 6 << GPIO_ALT_SHIFT)
#define GPIO_ALT_7       ( 7 << GPIO_ALT_SHIFT)
#define GPIO_ALT_8       ( 8 << GPIO_ALT_SHIFT)
#define GPIO_ALT_9       ( 9 << GPIO_ALT_SHIFT)
#define GPIO_ALT_10      (10 << GPIO_ALT_SHIFT)
#define GPIO_ALT_11      (11 << GPIO_ALT_SHIFT)
#define GPIO_ALT_12      (12 << GPIO_ALT_SHIFT)
#define GPIO_ALT_13      (13 << GPIO_ALT_SHIFT)
#define GPIO_ALT_14      (14 << GPIO_ALT_SHIFT)
#define GPIO_ALT_15      (15 << GPIO_ALT_SHIFT)

#define GPIO_MODE_SHIFT  12
#define GPIO_MODE_MASK   ( 3 << GPIO_MODE_SHIFT)
#define GPIO_MODE_IN     ( 0 << GPIO_MODE_SHIFT)
#define GPIO_MODE_OUT    ( 1 << GPIO_MODE_SHIFT)
#define GPIO_MODE_ALT    ( 2 << GPIO_MODE_SHIFT)
#define GPIO_MODE_ANALOG ( 3 << GPIO_MODE_SHIFT)

#define	GPIO_PULL_SHIFT  14
#define GPIO_PULL_MASK   ( 3 << GPIO_PULL_SHIFT)
#define GPIO_PULL_NONE   ( 0 << GPIO_PULL_SHIFT)
#define GPIO_PULL_UP     ( 1 << GPIO_PULL_SHIFT)
#define GPIO_PULL_DOWN   ( 2 << GPIO_PULL_SHIFT)

#define	GPIO_SPD_SHIFT   16
#define GPIO_SPD_MASK    ( 3 << GPIO_SPD_SHIFT)
#define GPIO_SPD_LOW     ( 0 << GPIO_SPD_SHIFT)
#define GPIO_SPD_MED     ( 1 << GPIO_SPD_SHIFT)
#define GPIO_SPD_FAST    ( 2 << GPIO_SPD_SHIFT)
#define GPIO_SPD_HIGH    ( 3 << GPIO_SPD_SHIFT)

#define	GPIO_TYPE_SHIFT  18
#define GPIO_TYPE_MASK   ( 1 << GPIO_TYPE_SHIFT)
#define GPIO_TYPE_PP     ( 0 << GPIO_TYPE_SHIFT)
#define GPIO_TYPE_OD     ( 1 << GPIO_TYPE_SHIFT)

#define	GPIO_INIT_SHIFT  19
#define GPIO_INIT_MASK   ( 1 << GPIO_INIT_SHIFT)
#define GPIO_INIT_CLR    ( 0 << GPIO_INIT_SHIFT)
#define GPIO_INIT_SET    ( 1 << GPIO_INIT_SHIFT)

void bootloader_gpio_init(uint32_t gpio);
void bootloader_gpio_write(uint32_t gpio, int state);
int  bootloader_gpio_read(uint32_t gpio);

#endif /* BOOTLOADER_GPIO */

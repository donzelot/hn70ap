/****************************************************************************
 * configs/hn70ap/src/bootloader_impl.c
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

/* Notes:
 *   - The clock is not changed, so we use the default, which is the 16 MHz HSI
 *     oscillator.
 */
#include "bootloader.h"
#include "bootloader_gpio.h"
#include "bootloader_uart.h"

#define UART4_TX      GPIO_PORT_C | GPIO_PIN_10 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_INIT_SET | GPIO_ALT_8
#define UART4_RX      GPIO_PORT_C | GPIO_PIN_11 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_INIT_SET | GPIO_ALT_8
#define SPI2_MOSI     GPIO_PORT_B | GPIO_PIN_15 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_ALT_5
#define SPI2_MISO     GPIO_PORT_B | GPIO_PIN_14 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_ALT_5
#define SPI2_SCLK     GPIO_PORT_B | GPIO_PIN_10 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_ALT_5
#define FLASH_CS      GPIO_PORT_A | GPIO_PIN_9  | GPIO_MODE_OUT | GPIO_TYPE_PP | GPIO_INIT_SET
#define LED_HEARTBEAT GPIO_PORT_D | GPIO_PIN_15 | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_INIT_SET
#define LED_CPUACT    GPIO_PORT_D | GPIO_PIN_11 | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_INIT_SET
#define BUTTON        GPIO_PORT_E | GPIO_PIN_11 | GPIO_MODE_IN  | GPIO_PULL_UP

static const char BOOTRODATA STR_WELCOME[] = "\r\n\r\n***** hn70ap bootloader *****\r\n";

/* -------------------------------------------------------------------------- */
/* Initialize all hardware needed by the bootloader */
BOOTCODE void bootloader_inithardware(void)
{
  /* Initialize UART4 */
  bootloader_gpio_init(UART4_TX);
  bootloader_gpio_init(UART4_RX);
  bootloader_uart_init(4);
  bootloader_uart_setbaud(4, 230400);

  /* Initialize SPI2 */
  bootloader_gpio_init(SPI2_MISO);
  bootloader_gpio_init(SPI2_MOSI);
  bootloader_gpio_init(SPI2_SCLK);

  /* Initialize external flash */
  bootloader_gpio_init(FLASH_CS);

  /* Initialize LEDs */
  bootloader_gpio_init(LED_HEARTBEAT);
  bootloader_gpio_init(LED_CPUACT);

  /* Initialize Button */
  bootloader_gpio_init(BUTTON);

  bootloader_uart_write_string(4, STR_WELCOME);
}

/* -------------------------------------------------------------------------- */
/* Deinitialize all hardware that was initialized.
 * Specially important for SPI
 */
BOOTCODE void bootloader_stophardware(void)
{
  /* Disable SPI2 */

  /* Disable UART4 */
}

/* -------------------------------------------------------------------------- */
/* Return the state of the on-board button */
BOOTCODE bool bootloader_buttonpressed(void)
{
  return bootloader_gpio_read(BUTTON) == 0;
}

/* -------------------------------------------------------------------------- */
/* Read the contents of the external flash and determine if a valid
 * software image is present.
 */
BOOTCODE bool bootloader_checkupdate(void)
{
  return false;
}

/* -------------------------------------------------------------------------- */
/* Copy the firmware (supposed valid) from the external flash to the
 * internal stm32 flash. Return true on success, false on failure.
 */
BOOTCODE bool bootloader_apply(void)
{
  return false;
}

/* -------------------------------------------------------------------------- */
/* Handle a download protocol to fill the external flash from data received
 * through the UART
 */
BOOTCODE void bootloader_download(void)
{
}


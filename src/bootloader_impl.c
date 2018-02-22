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
#include "bootloader_spi.h"
#include "bootloader_crc.h"

/* All text messages should be defined here instead of directly as parameters to
 * functions, because there is absolutely NO WAY to control which rodata section
 * is used to store litteral strings passed as parameters.
 * String messages must be declared as char arrays, NOT pointers.
 */
static const char STR_WELCOME[] BOOTRODATA = "\r\n\r\n***** hn70ap bootloader *****\r\n";
static const char STR_NOFLASH[] BOOTRODATA = "External flash device not detected.\r\n";
static const char STR_BOOT[]    BOOTRODATA = "Starting OS.\r\n";

static uint8_t hdrbuf[256] BOOTBSS;  /* Buffer for the header page containing update parameters */
static uint8_t pagebuf[256]; /* Additional Buffer for flash data handling */

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
  bootloader_spi_init(2, SPI_MODE_0 | SPI_BAUDDIV_64);

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
  /* Disable SPI2, else NuttX wont properly initialize the SPI block */
  bootloader_spi_fini(2);

  /* Last string displayed before starting the OS */
  bootloader_uart_write_string(4, STR_BOOT);
}

/* -------------------------------------------------------------------------- */
/* Return the state of the on-board button */
BOOTCODE bool bootloader_buttonpressed(void)
{
  return bootloader_gpio_read(BUTTON) == 0;
}

/* -------------------------------------------------------------------------- */
/* Read the contents of the external flash and determine if a valid
 * software image is present. If a check error happens, the update header page
 * is erased to avoid an infinite update failure loop, which will allow the
 * current OS to start.
 */
BOOTCODE bool bootloader_checkupdate(void)
{
  uint8_t  flashid[3];
  bool     success    = false;
  uint32_t sectorsize = 0; //size of the external flash erase block size

  /* Preparations. */
  bootloader_crc_init();

  /* Attempt to detect the flash */
  bootloader_spiflash_readjedec(flashid);

  if(flashid[0] == 0xBF && flashid[1] == 0x25 && flashid[2] == 0x43)
  {
    success    = true;
    sectorsize = 4096;
  }

  /* If the hardware is upgraded to support more flash devices, detect these
   * here. */

  if(!success)
    {
      bootloader_uart_write_string(4, STR_NOFLASH);
      return false;
    }

  /* Flash is there. Read the first page. */

  /* Check CRC of header page */

  /* CRC is correct, means we're very likely to have an update */

  /* Get update parameters */

  /* Compute the CRC/SHA-256 of the image */

  return false;
}

/* -------------------------------------------------------------------------- */
/* Copy the firmware (supposed valid) from the external flash to the
 * internal stm32 flash. Return true on success, false on failure.
 * If we are interrupted anywhere in this proces, the update (previously
 * declared valid) can still be applied at next boot.
 */
BOOTCODE bool bootloader_apply(void)
{
  /* Erase the internal flash */

  /* Copy the update from spi flash to internal flash */

  /* Verify the contents of the internal flash */

  /* We can now erase the update header in the external flash. */

  return false;
}

/* -------------------------------------------------------------------------- */
/* Handle a download protocol to fill the external flash from data received
 * through the UART. Protocol undefined yet.
 */
BOOTCODE void bootloader_download(void)
{
}


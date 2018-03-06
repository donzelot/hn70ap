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
#include "bootloader_spiflash.h"

/* All text messages should be defined here instead of directly as parameters to
 * functions, because there is absolutely NO WAY to control which rodata section
 * is used to store litteral strings passed as parameters.
 * String messages must be declared as char arrays, NOT pointers.
 */
static const char hex[]          BOOTRODATA = "0123456789ABCDEF";
static const char STR_WELCOME[]  BOOTRODATA = "\r\n\r\n***** hn70ap bootloader *****\r\n";
static const char STR_NOFLASH[]  BOOTRODATA = "Flash not detected.\r\n";
static const char STR_BOOT[]     BOOTRODATA = "Starting OS.\r\n";
static const char STR_DOWNLOAD[] BOOTRODATA = "Download mode.\r\n";
static const char STR_NOUPDATE[] BOOTRODATA = "No update.\r\n";
static const char STR_BADHCRC[]  BOOTRODATA = "Bad Header CRC!\r\n";
static const char STR_OK[]       BOOTRODATA = "Update OK.\r\n";

static uint8_t pgbuf[256] BOOTBSS;

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
  bootloader_spi_init(2, SPI_MSBFIRST | SPI_MODE_0 | SPI_BAUDDIV_64);

  /* Initialize external flash */
  bootloader_gpio_init(FLASH_CS);
  bootloader_gpio_write(FLASH_CS, 1);

  /* Initialize LEDs */
  bootloader_gpio_init(LED_HEARTBEAT);
  bootloader_gpio_init(LED_CPUACT);
  bootloader_gpio_init(LED_RED);
  bootloader_gpio_init(LED_ORANGE);

  bootloader_gpio_write(LED_RED      , 0);

  /* Initialize Button */
  bootloader_gpio_init(BUTTON);

  bootloader_uart_write_string(4, STR_WELCOME);
}

/* -------------------------------------------------------------------------- */
BOOTCODE void puthb(uint32_t uartid, uint8_t b)
{
  bootloader_uart_send(uartid, hex[b>> 4]);
  bootloader_uart_send(uartid, hex[b&0xf]);
}

/* -------------------------------------------------------------------------- */
/* Deinitialize all hardware that was initialized.
 * Specially important for SPI
 */
BOOTCODE void bootloader_stophardware(void)
{
  /* Stop internal LEDs, signalling starting of OS */
  bootloader_gpio_write(LED_RED      , 1);

  /* Disable SPI2, else NuttX wont properly initialize the SPI block */
  bootloader_spi_fini(2);

  /* Last string displayed before starting the OS */
  bootloader_uart_write_string(4, STR_BOOT);
}

/* -------------------------------------------------------------------------- */
/* Return the state of the on-board button. We need a small delay to ensure
 * that the debounce capacitor had time to discharge through the pin pullup. */
BOOTCODE bool bootloader_buttonpressed(void)
{
  volatile uint32_t delay;
  for(delay=0;delay<100000LU;delay++) {}
  return bootloader_gpio_read(BUTTON) == 0;
}

/* -------------------------------------------------------------------------- */
/* Read the contents of the external flash and determine if a valid
 * software image is present. If a check error happens, the update header page
 * is erased to avoid an infinite update failure loop, which will allow the
 * current OS to start.
 */
BOOTCODE bool bootloader_check(void)
{
  bool     success    = false;
  uint32_t sectorsize = 0; //external flash erase block size
  uint32_t crc;
  uint8_t  check;
  int i;
  uint8_t *ptr;
  uint32_t len;
  uint32_t size;

  /* Preparations. */
  bootloader_crc_init();

  /* Attempt to detect the flash */
  bootloader_spiflash_readjedec(2, pgbuf);
  
  if(pgbuf[0] == 0xBF && pgbuf[1] == 0x26 && pgbuf[2] == 0x43)
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
  bootloader_spiflash_readpage(2, 0, pgbuf);

  /* Check for blank */
  check = 0xFF;
  for(i=0;i<256;i++)
    {
      puthb(4, pgbuf[i]);
      check &= pgbuf[i];
    }
  if(check == 0xFF)
    {
      bootloader_uart_write_string(4, STR_NOUPDATE);
      return false;
    }

  /* Check CRC of header page */
  crc  = CRC32_INIT;
  crc  = bootloader_crc_do(crc, pgbuf+4, 252);
  crc ^= CRC32_MASK;
  crc ^= PEEK_U32BE(pgbuf);

  if(crc)
    {
      bootloader_uart_write_string(4, STR_BADHCRC);
      return false;
    }

  /* CRC is correct, means we're very likely to have an update */
  bootloader_uart_write_string(4, STR_OK);

  /* Get update parameters */
  ptr = bootloader_tlv_find(pgbuf+4, 252, 0xC0, &len, 0);
  if(!ptr || len != 4)
    {
    }

  /* Compute the CRC/SHA-256 of the image */

  return false;
}

/* -------------------------------------------------------------------------- */
/* Copy the firmware (supposed valid) from the external flash to the
 * internal stm32 flash. If we are interrupted anywhere in this proces, the
 * update (previously declared valid) can still be applied at next boot.
 */
BOOTCODE void bootloader_apply(void)
{
  /* Erase the internal flash */

  /* Copy the update from spi flash to internal flash */

  /* Verify the contents of the internal flash */

  /* We can now erase the update header in the external flash. */

}

BOOTCODE void bootloader_cleanup(void)
{
}

/* -------------------------------------------------------------------------- */
/* Handle a download protocol to fill the external flash from data received
 * through the UART. Protocol undefined yet.
 */
BOOTCODE void bootloader_download(void)
{
  bootloader_gpio_write(LED_RED      , 1);
  bootloader_gpio_write(LED_ORANGE   , 0);
  bootloader_uart_write_string(4, STR_DOWNLOAD);
}


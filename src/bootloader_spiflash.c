/*******************************************************************************
 * configs/hn70ap/src/bootloader_spiflash.c
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
 ******************************************************************************/

#include <stdint.h>

#include "bootloader.h"
#include "bootloader_gpio.h"
#include "bootloader_spi.h"

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_spiflash_reset(uint32_t spidev)
{
  uint8_t cmd[1];
  volatile int i;

  cmd[0] = 0x66; //RESET ENABLE

  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 1, cmd, NULL);
  bootloader_gpio_write(FLASH_CS, 1);

  for(i=0;i<1000;i++) {}  // Small delay

  cmd[0] = 0x99; //RESET

  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 1, cmd, NULL);
  bootloader_gpio_write(FLASH_CS, 1);
}

/* -------------------------------------------------------------------------- */
/* Read the JEDEC ID from the SPI flash */
BOOTCODE void bootloader_spiflash_readjedec(uint32_t spidev, uint8_t *id)
{
  uint8_t buf;
  buf = 0x9F; //RDID
  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 1, &buf, NULL);
  bootloader_spi_transac8(spidev, 3, NULL, id);
  bootloader_gpio_write(FLASH_CS, 1);
}

/* -------------------------------------------------------------------------- */
/* Read a 256 byte page from the SPI flash */
BOOTCODE void bootloader_spiflash_readpage(int spidev, uint32_t pageid, uint8_t *pagebuf)
{
  uint8_t buf[5];
  buf[0] = 0x0B; //RDFAST
  buf[1] = pageid>>8;
  buf[2] = pageid;
  buf[3] = 0x00;
  buf[4] = 0xFF; //dummy, fast read latency
  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 5  , buf , NULL   );
  bootloader_spi_transac8(spidev, 256, NULL, pagebuf);
  bootloader_gpio_write(FLASH_CS, 1);
}

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_spiflash_globalunlock(uint32_t spidev)
{
  uint8_t cmd[1];
  cmd[0] = 0x98; //GLOBAL UNLOCK

  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 1, cmd, NULL);
  bootloader_gpio_write(FLASH_CS, 1);
}

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_spiflash_writeenable(uint32_t spidev, bool state)
{
  uint8_t cmd[1];
  if(state)
    {
    cmd[0] = 0x06; //WRITE ENABLE
    }
  else
    {
    cmd[0] = 0x04; //WRITE DISABLE
    }

  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 1, cmd, NULL);
  bootloader_gpio_write(FLASH_CS, 1);
}

/* -------------------------------------------------------------------------- */
BOOTCODE static void bootloader_spiflash_waitcomplete(uint32_t spidev)
{
  uint8_t cmd[1];
  cmd[0] = 0x05; //READ STATUS REGISTER

  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 1, cmd, NULL);
  do
    {
      bootloader_spi_transac8(spidev, 1, NULL, cmd);
    }
  while(cmd[0] & 0x01);
  bootloader_gpio_write(FLASH_CS, 1);
}

/* -------------------------------------------------------------------------- */
/* Erase memory at given address */
BOOTCODE void bootloader_spiflash_eraseat(int spidev, uint32_t address, uint8_t type)
{
  uint8_t buf[4];

  bootloader_spiflash_writeenable(spidev, true);

  buf[0] = type;
  buf[1] = address >> 16;
  buf[2] = address >> 8;
  buf[3] = address;

  bootloader_gpio_write(FLASH_CS, 0);
  bootloader_spi_transac8(spidev, 4, buf, NULL);
  bootloader_gpio_write(FLASH_CS, 1);

  /* Now wait for completion */
  bootloader_spiflash_waitcomplete(spidev);
}

/* -------------------------------------------------------------------------- */
/* Erase a 4k sector */
BOOTCODE void bootloader_spiflash_erase4ksector(int spidev, uint32_t sectorid)
{
  //sector s -> byte ssssssss_ssss0000_00000000
  bootloader_spiflash_eraseat(spidev, sectorid << 12, 0x20);//4K SECTOR ERASE
}


/****************************************************************************
 * configs/hn70ap/src/bootloader_intflash.c
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
#include <stdint.h>

#include "bootloader.h"
#include "bootloader_intflash.h"

/* Even for 2MB parts we dont use the dual bank feature */

const uint8_t bootloader_sectors_kb[] BOOTRODATA = {
  /* 512 K */

  16,  /*  0 */
  16,  /*  1 */
  16,  /*  2 */
  16,  /*  3 */
  64,  /*  4 */
  128, /*  5 */
  128, /*  6 */
  128, /*  7 */

  /* 1024 K */

  128, /*  8 */
  128, /*  9 */
  128, /* 10 */
  128, /* 11 */

  /* 2048 K */

  16,  /* 12 */
  16,  /* 13 */
  16,  /* 14 */
  16,  /* 15 */
  64,  /* 16 */
  128, /* 17 */
  128, /* 18 */
  128, /* 19 */
  128, /* 20 */
  128, /* 21 */
  128, /* 23 */
  128  /* 23 */
};

static uint32_t bootloader_sectcount BOOTBSS;

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_intflash_init(uint8_t blocksize)
{
  /*check that the dual bank feature is disabled*/
  if(getreg32(FLASH_OPTCR) & FLASH_OPTCR_DB1M)
    {
      putreg32(FLASH_OPTKEYR, 0x08192A3B);
      putreg32(FLASH_OPTKEYR, 0x4C5D6E7F); /* unlock */
      /*putreg32(FLASH_OPTCR, 0x0FFFAAED); value used to cancel write protection */
      modreg32(FLASH_OPTCR, 0, FLASH_OPTCR_DB1M);
      modreg32(FLASH_OPTCR, FLASH_OPTCR_OPTSTRT, 0);

      /* wait for completion */
      while(getreg32(FLASH_SR) & FLASH_SR_BSY);

      modreg32(FLASH_OPTCR, FLASH_OPTCR_OPTLOCK, 0); /* relock */
    }

  /* unlock */
  putreg32(FLASH_KEYR, 0x45670123);
  putreg32(FLASH_KEYR, 0xCDEF89AB);

  /* set block size */
  blocksize &= 3; /* only keep 2 bits */
  modreg32(FLASH_CR, blocksize<<8, 0x00000300);

  /* Relock */
  modreg32(FLASH_CR, FLASH_CR_LOCK, 0);

  /* read flash info and define sector count */
  bootloader_sectcount = getreg16(FLASH_SIZE_ADDR);

  switch(bootloader_sectcount)
    {
      case 0x200: bootloader_sectcount =  8; break;
      case 0x400: bootloader_sectcount = 12; break;
      case 0x800: bootloader_sectcount = 24; break;
      default:    bootloader_sectcount =  0;
    }
}

/* -------------------------------------------------------------------------- */
BOOTCODE uint32_t bootloader_intflash_erase(uint32_t destaddr)
{
  int i;
  uint32_t addr = FLASH_BASE;

  /* check for busy */
  if( getreg32(FLASH_SR) & FLASH_SR_BSY)
    {
      return 0;
    }

  /*find sector*/
  for(i = 0; i < bootloader_sectcount; i++)
    {
      if(addr == destaddr)
        {
          break;
        } else if(addr > destaddr) {
          return 0; //not a sector begin addr
        }
      addr += ((uint32_t)bootloader_sectors_kb[i]<<10);
    }
  if(i == bootloader_sectcount)
    {
      /* searched all without finding */
      return 0;
    }

  /* okay, erase */

  /* unlock */
  putreg32(FLASH_KEYR, 0x45670123);
  putreg32(FLASH_KEYR, 0xCDEF89AB);

  /*set SER*/
  modreg32(FLASH_CR, FLASH_CR_SER, 0);

  /* set sector index (SNB) */
  modreg32(FLASH_CR, (i << 3), (0x1F << 3));
    
  /*set START*/
  modreg32(FLASH_CR, FLASH_CR_START, 0);

  /* wait for completion */
  while(getreg32(FLASH_SR) & FLASH_SR_BSY);

  /* Relock */
  modreg32(FLASH_CR, FLASH_CR_LOCK, 0);

  /* Return KB in this sector */
  return (uint32_t)bootloader_sectors_kb[i] << 10;
}

/* -------------------------------------------------------------------------- */
BOOTCODE int bootloader_intflash_write(uint32_t destaddr, uint8_t *sourcedata, uint32_t len)
{
  /* retrieve current write element size from CR */
  uint32_t blocksize = (getreg32(FLASH_CR) >> 8) & 0x03;
  uint32_t offset;

  blocksize = 1 << blocksize;

  /* check that memory to write has the correct size */
  if( (len & (blocksize - 1)) != 0)
    {
    return 1;
    }

  /* unlock */
  putreg32(FLASH_KEYR, 0x45670123);
  putreg32(FLASH_KEYR, 0xCDEF89AB);

  /* program */

  for(offset = 0; offset < len; offset += blocksize)
    {
      modreg32(FLASH_CR, FLASH_CR_PG, FLASH_CR_SER);

      switch(blocksize)
        {
        case 1: /* 8-bit writes */
          *((volatile uint8_t*)destaddr) = *sourcedata;
          break;

        case 2: /* 16-bit writes */
          *((volatile uint16_t*)destaddr) = *(uint16_t*)sourcedata;
          break;

        case 4: /* 32-bit writes */
          *((volatile uint32_t*)destaddr) = *(uint32_t*)sourcedata;
          break;

        case 8: /* 64-bit writes */
          *((volatile uint64_t*)destaddr) = *(uint64_t*)sourcedata;
          break;

        default:
          return 1; /*error*/
        }

      /* wait for completion */
      while(getreg32(FLASH_SR) & FLASH_SR_BSY);

      /* next block */
      sourcedata += blocksize;
      destaddr   += blocksize;
    }

  /* Relock */
  modreg32(FLASH_CR, FLASH_CR_LOCK, 0);

  return 0; /*success*/
}

/* -------------------------------------------------------------------------- */
BOOTCODE int bootloader_intflash_writeprotect(uint32_t sector, uint32_t prot)
{
  uint32_t reg;
  uint32_t addr;
  uint32_t mask;

  if(sector >= bootloader_sectcount)
    {
      return 1;
    }

  if(sector < 12)
    {
      addr = FLASH_OPTCR;
    }
  else
    {
      addr = FLASH_OPTCR1;
      sector -= 12;
    }

  reg  = getreg32(addr);
  mask = 1 << (16 + sector);

  /* Test if already done and exit */
  if (reg & mask)
    {
      return 0;
    }

  /* wait half a second for the power to sette */
  {int _i; for(_i=0;_i<1000000;_i++) asm("nop");}
  

  if(prot)
    {
      reg &= ~mask;
    }
  else
    {
      reg |=  mask;
    }

  /* unlock */
  putreg32(FLASH_OPTKEYR, 0x08192A3B);
  putreg32(FLASH_OPTKEYR, 0x4C5D6E7F);

  /* write */
  putreg32(addr, reg);
  modreg32 (FLASH_OPTCR, FLASH_OPTCR_OPTSTRT, 0);

  /* wait for completion */
  while(getreg32(FLASH_SR) & FLASH_SR_BSY);

  /* relock */
  modreg32 (FLASH_OPTCR, FLASH_OPTCR_OPTLOCK, 0);

  return 0;
}

/* -------------------------------------------------------------------------- */
BOOTCODE int bootloader_intflash_enablebor(uint32_t level)
{
  uint32_t reg;

  reg = getreg32(FLASH_OPTCR);

  if( (reg & FLASH_OPTCR_BORLEV_MASK) == level)
    {
      /* Bit already in the correct state... */
      return 0;
    }

  /* Wait a half second to let the power supply settle */
  {int _i; for(_i=0;_i<1000000;_i++) asm("nop");}

  /* unlock */
  putreg32(FLASH_OPTKEYR, 0x08192A3B);
  putreg32(FLASH_OPTKEYR, 0x4C5D6E7F);

  /* write */
  modreg32(FLASH_OPTCR, level, FLASH_OPTCR_BORLEV_MASK);
  modreg32(FLASH_OPTCR, FLASH_OPTCR_OPTSTRT, 0);

  /* wait for completion */
  while(getreg32(FLASH_SR) & FLASH_SR_BSY);

  /* relock */
  modreg32(FLASH_OPTCR, FLASH_OPTCR_OPTLOCK, 0);

  return 0;
}


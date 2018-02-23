/*******************************************************************************
 * configs/hn70ap/src/bootloader_uart.h
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

#include "bootloader.h"
#include "bootloader_rcc.h"
#include "bootloader_spi.h"

struct spi_params {
  uint32_t base;
  uint32_t ckenreg;
  uint32_t ckenbit;
};

static const struct spi_params g_spi[] BOOTRODATA = {
  {0x40013000, RCC_APB2ENR, 12},
  {0x40003800, RCC_APB1ENR, 14},
  {0x40003C00, RCC_APB1ENR, 15},
  {0x40013400, RCC_APB2ENR, 13},
  {0x40015000, RCC_APB2ENR, 20},
};

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_spi_init(uint32_t spiid, uint32_t flags)
{
  uint32_t base;
  uint32_t val;

  if( (spiid < 1) || (spiid > 5) )
    {
      return;
    }
  spiid -= 1;
  base = g_spi[spiid].base;

  //Enable clock to SPI peripheral
  modreg32(g_spi[spiid].ckenreg, (1<<g_spi[spiid].ckenbit), 0);

  //configure registers for a simple spi, 8 bits values

  //CR1: BIDI=0, no bidioe, CRCEN=0, CRCNEXT=0, DFF=0, RXONLY=0, SSM=1, SSI=1 (software SS), SPE=1, MSTR=1
  val  = SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

  //lsbfirst,BR,cpol,cpha as given
  if((flags & SPI_BITORDER_MASK) >> SPI_BITORDER_SHIFT)
    {
      val |=  SPI_CR1_LSBFIRST;
    }
  if((flags & SPI_CPOL_MASK) >> SPI_CPOL_SHIFT)
    {
      val |=  SPI_CR1_CPOL;
    }
  if((flags & SPI_CPHA_MASK) >> SPI_CPHA_SHIFT)
    {
      val |=  SPI_CR1_CPHA;
    }

  val |= ((flags & SPI_BAUDDIV_MASK) >> SPI_BAUDDIV_SHIFT) << SPI_CR1_BR_SHIFT;
  putreg32(base + SPI_OFF_CR1, val);

  //CR2: no ints, FRF=0, SSOE=0, no DMA
  putreg32(base + SPI_OFF_CR2, 0x00000000);

  //I2SCFGR: No I2S, all zero.
  putreg32(base + SPI_OFF_I2SCFGR, 0x00000000);
}

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_spi_fini(uint32_t spiid)
{
  if( (spiid < 1) || (spiid > 5) )
    {
      return;
    }
  spiid -= 1;

  putreg32(g_spi[spiid].base + SPI_OFF_CR1, 0);
}

/* -------------------------------------------------------------------------- */
BOOTCODE void bootloader_spi_transac8(uint32_t spiid, uint32_t len, uint8_t *mosi, uint8_t *miso)
{
  uint32_t dreg,sreg;
  uint32_t txid=0,rxid=0;
  uint32_t stat,data;

  if( (spiid < 1) || (spiid > 5) )
    {
      return;
    }
  spiid -= 1;

  sreg = g_spi[spiid].base + SPI_OFF_SR;
  dreg = g_spi[spiid].base + SPI_OFF_DR;

  //see RM0090 (docid 018909) page 871, figure 253
  //tx write and rx read must be managed concurrently,
  //because the tx buffer is empty before the char is sent, so we can write in advance.
  while(rxid<len)
    {
      stat = getreg32(sreg);
      if( txid<len && (stat & SPI_SR_TXE) )
        {
          if(mosi)
            {
              data=mosi[txid];
            }
          else
            {
              data=0xFF;
            }
          putreg32(dreg, data); //this clears TXE
          txid++;
        }
      //as soon as RXNE is set, read and store
      stat = getreg32(sreg);
      if( rxid<len && (stat & SPI_SR_RXNE))
        {
          data = getreg32(dreg); //this clears TXNE
          if(miso)
            {
              miso[rxid] = data;
            }
          rxid++;
        }
    }

  //All data was now sent, wait until the spi transaction is not busy anymore (tx complete)
  while( (getreg32(sreg) & SPI_SR_BSY) )
    {
    }

}


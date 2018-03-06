/****************************************************************************
 * configs/hn70ap/src/bootloader_spi.h
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
#ifndef BOOTLOADER_SPI
#define BOOTLOADER_SPI

#include <stdint.h>

//reg offsets
#define SPI_OFF_CR1              0x00
#define SPI_OFF_CR2              0x04
#define SPI_OFF_SR               0x08
#define SPI_OFF_DR               0x0C
#define SPI_OFF_CRCPR            0x10
#define SPI_OFF_RXCRCR           0x14
#define SPI_OFF_TXCRCR           0x18
#define SPI_OFF_I2SCFGR          0x1C
#define SPI_OFF_I2SPR            0x20

//bits values
#define SPI_CR1_BIDIMODE         ( 1 << 15)
#define SPI_CR1_BIDIOE           ( 1 << 14)
#define SPI_CR1_CRCEN            ( 1 << 13)
#define SPI_CR1_CRCNEXT          ( 1 << 12)
#define SPI_CR1_DFF              ( 1 << 11)
#define SPI_CR1_RXONLY           ( 1 << 10)
#define SPI_CR1_SSM              ( 1 <<  9)
#define SPI_CR1_SSI              ( 1 <<  8)
#define SPI_CR1_LSBFIRST         ( 1 <<  7)
#define SPI_CR1_SPE              ( 1 <<  6)
#define SPI_CR1_BR_SHIFT         3
#define SPI_CR1_BR_MASK          (15 << SPI_CR1_BR_SHIFT)
#define SPI_CR1_MSTR             ( 1 <<  2)
#define SPI_CR1_CPOL             ( 1 <<  1)
#define SPI_CR1_CPHA             ( 1 <<  0)

#define SPI_CR2_TXEIE            ( 1 <<  7)
#define SPI_CR2_RXNEIE           ( 1 <<  6)
#define SPI_CR2_ERRIE            ( 1 <<  5)
#define SPI_CR2_FRF              ( 1 <<  4)
#define SPI_CR2_SSOE             ( 1 <<  2)
#define SPI_CR2_TXDMAEN          ( 1 <<  1)
#define SPI_CR2_RXDMAEN          ( 1 <<  0)

#define SPI_SR_FRE               ( 1 << 8)
#define SPI_SR_BSY               ( 1 << 7)
#define SPI_SR_OVR               ( 1 << 6)
#define SPI_SR_MODF              ( 1 << 5)
#define SPI_SR_CRCERR            ( 1 << 4)
#define SPI_SR_UDR               ( 1 << 3)
#define SPI_SR_CHSIDE            ( 1 << 2)
#define SPI_SR_TXE               ( 1 << 1)
#define SPI_SR_RXNE              ( 1 << 0)

#define SPI_I2SCFGR_I2SMOD       ( 1 << 11)
#define SPI_I2SCFGR_I2SE         ( 1 << 10)
#define SPI_I2SCFGR_I2SMAST      ( 1 <<  9)
#define SPI_I2SCFGR_I2SRX        ( 1 <<  8)
#define SPI_I2SCFGR_I2SCFG_SHIFT 8
#define SPI_I2SCFGR_I2SCFG_MASK  ( 1 << SPI_I2SCFGR_I2SCFG_SHIFT)
#define SPI_I2SCFGR_PCMSYNC      ( 1 <<  7)
#define SPI_I2SCFGR_I2SSTD_SHIFT 4
#define SPI_I2SCFGR_I2SSTD_MASK  ( 7 << SPI_I2SCFGR_I2SSTD_SHIFT)
#define SPI_I2SCFGR_CKPOL        ( 1 <<  5)
#define SPI_I2SCFGR_DATLEN_SHIFT 1
#define SPI_I2SCFGR_DATLEN_MASK  (15 << SPI_I2SCFGR_DATLEN_SHIFT)
#define SPI_I2SCFGR_CHLEN        ( 1 <<  0)

#define SPI_I2SPR_MCKOE          ( 1 << 9 )
#define SPI_I2SPR_ODD            ( 1 << 8 )
#define SPI_I2SPR_I2SDIV_SHIFT   0
#define SPI_I2SPR_I2SDIV_MASK    (0xFF << SPI_I2SPR_I2SDIV_SHIFT)

/* spi flags
   pos len  name
   0   1    CPHA
   1   1    CPOL
   2   1    LSB_FIRST
   3   3    BAUD_DIV
*/


#define SPI_CPHA_SHIFT     0
#define SPI_CPHA_MASK      (1<<SPI_CPHA_SHIFT)
#define SPI_CPHA_0         (0<<SPI_CPHA_SHIFT)
#define SPI_CPHA_1         (1<<SPI_CPHA_SHIFT)

#define SPI_CPOL_SHIFT     1
#define SPI_CPOL_MASK      (1<<SPI_CPOL_SHIFT)
#define SPI_CPOL_0         (0<<SPI_CPOL_SHIFT)
#define SPI_CPOL_1         (1<<SPI_CPOL_SHIFT)

#define SPI_MODE_0         (SPI_CPOL_0 | SPI_CPHA_0)
#define SPI_MODE_1         (SPI_CPOL_0 | SPI_CPHA_1)
#define SPI_MODE_2         (SPI_CPOL_1 | SPI_CPHA_0)
#define SPI_MODE_3         (SPI_CPOL_1 | SPI_CPHA_1)

#define SPI_BITORDER_SHIFT 2
#define SPI_BITORDER_MASK  (1<<SPI_BITORDER_SHIFT)
#define SPI_MSBFIRST       (0<<SPI_BITORDER_SHIFT)
#define SPI_LSBFIRST       (1<<SPI_BITORDER_SHIFT)

#define SPI_BAUDDIV_SHIFT  3
#define SPI_BAUDDIV_MASK   (7<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_2      (0<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_4      (1<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_8      (2<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_16     (3<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_32     (4<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_64     (5<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_128    (6<<SPI_BAUDDIV_SHIFT)
#define SPI_BAUDDIV_256    (7<<SPI_BAUDDIV_SHIFT)

void bootloader_spi_init(uint32_t spiid, uint32_t flags);
void bootloader_spi_fini(uint32_t spiid);
void bootloader_spi_transac8(uint32_t spiid, uint32_t len, uint8_t *mosi, uint8_t *miso);

#endif /* BOOTLOADER_SPI */


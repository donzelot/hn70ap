/****************************************************************************
 * configs/hn70ap/src/bootloader_intflash.h
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
#ifndef BOOTLOADER_INTFLASH_H
#define BOOTLOADER_INTFLASH_H

#include <stdint.h>

enum {
    FLASH_BLOCK_8,
    FLASH_BLOCK_16,
    FLASH_BLOCK_32,
    FLASH_BLOCK_64
};

#define FLASH_BASE          0x08000000

#define FLASH_SIZE_ADDR     0x1FFF7A22

#define FLASH_KEYR          0x40023C04
#define FLASH_OPTKEYR       0x40023C08
#define FLASH_SR            0x40023C0C
#define FLASH_CR            0x40023C10
#define FLASH_OPTCR         0x40023C14
#define FLASH_OPTCR1        0x40023C18

#define FLASH_CR_PG         (1<<  0)
#define FLASH_CR_SER        (1<<  1)
#define FLASH_CR_START      (1<< 16)
#define FLASH_CR_LOCK       (1<< 31)

#define FLASH_SR_BSY        (1<< 16)

#define FLASH_OPTCR_OPTSTRT (1<<  1)
#define FLASH_OPTCR_OPTLOCK (1<<  0)
#define FLASH_OPTCR_DB1M    (1<< 30)

#define FLASH_OPTCR_BORLEV_SHIFT 2
#define FLASH_OPTCR_BORLEV_MASK (3 << FLASH_OPTCR_BORLEV_SHIFT)
#define FLASH_OPTCR_BORLEV_LVL3 (0 << FLASH_OPTCR_BORLEV_SHIFT)
#define FLASH_OPTCR_BORLEV_LVL2 (1 << FLASH_OPTCR_BORLEV_SHIFT)
#define FLASH_OPTCR_BORLEV_LVL1 (2 << FLASH_OPTCR_BORLEV_SHIFT)
#define FLASH_OPTCR_BORLEV_OFF  (3 << FLASH_OPTCR_BORLEV_SHIFT)

/* init registers before operations*/

void bootloader_intflash_init(uint8_t blocksize);

/* erase flash at any address, return erased size */

uint32_t bootloader_intflash_erase(uint32_t destaddr);

/* write some data to the flash */

int bootloader_intflash_write(uint32_t destaddr, uint8_t *sourcedata, uint32_t len);

int bootloader_intflash_writeprotect(uint32_t sector, uint32_t prot);
int bootloader_intflash_enablebor(uint32_t level);

#endif


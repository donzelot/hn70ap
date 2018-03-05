/****************************************************************************
 * configs/hn70ap/src/bootloader.h
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

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdbool.h>
#include <stdint.h>

#include "bootloader_gpio.h"

#define BOOTCODE   __attribute__(( section(".boot.text") ))
#define BOOTRODATA __attribute__(( section(".boot.rodata") ))
#define BOOTBSS    __attribute__(( section(".boot.bss") ))

#define NULL ((void*)0)
#define CLOCK_SPEED 16000000LU

#define UART4_TX      GPIO_PORT_C | GPIO_PIN_10 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_SPD_FAST | GPIO_INIT_SET | GPIO_ALT_8
#define UART4_RX      GPIO_PORT_C | GPIO_PIN_11 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_SPD_FAST | GPIO_INIT_SET | GPIO_ALT_8
#define SPI2_MOSI     GPIO_PORT_B | GPIO_PIN_15 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_SPD_FAST | GPIO_ALT_5
#define SPI2_MISO     GPIO_PORT_B | GPIO_PIN_14 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_SPD_FAST | GPIO_ALT_5
#define SPI2_SCLK     GPIO_PORT_B | GPIO_PIN_10 | GPIO_MODE_ALT | GPIO_TYPE_PP | GPIO_SPD_FAST | GPIO_ALT_5
#define FLASH_CS      GPIO_PORT_A | GPIO_PIN_9  | GPIO_MODE_OUT | GPIO_TYPE_PP | GPIO_SPD_FAST | GPIO_INIT_SET
#define LED_HEARTBEAT GPIO_PORT_D | GPIO_PIN_15 | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_SPD_FAST | GPIO_INIT_SET
#define LED_CPUACT    GPIO_PORT_D | GPIO_PIN_11 | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_SPD_FAST | GPIO_INIT_SET
#define LED_RED       GPIO_PORT_B | GPIO_PIN_8  | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_SPD_FAST | GPIO_INIT_SET
#define LED_ORANGE    GPIO_PORT_B | GPIO_PIN_7  | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_SPD_FAST | GPIO_INIT_SET
#define LED_GREEN     GPIO_PORT_B | GPIO_PIN_6  | GPIO_MODE_OUT | GPIO_TYPE_OD | GPIO_SPD_FAST | GPIO_INIT_SET
#define BUTTON        GPIO_PORT_E | GPIO_PIN_11 | GPIO_MODE_IN  | GPIO_PULL_UP | GPIO_SPD_FAST  

/* Register access */

#define putreg32(addr, val) *((volatile uint32_t*)(addr)) = (val)
#define getreg32(addr)     (*((volatile uint32_t*)(addr)))

#define modreg32(addr, set, clr) putreg32(addr, (getreg32(addr) & ~(clr)) | (set))

/* MEMIO */

#define PEEK_U32BE(addr) ( *(uint8_t*)(addr)<<24 | *((uint8_t*)(addr)+1)<<16 | *((uint8_t*)(addr)+2)<< 8 | *((uint8_t*)(addr)+3)     )

/* Functions required for implementation of the above high level routines */

BOOTCODE void bootloader_inithardware(void);
BOOTCODE void bootloader_stophardware(void);
BOOTCODE bool bootloader_buttonpressed(void);
BOOTCODE bool bootloader_check(void);
BOOTCODE void bootloader_apply(void);
BOOTCODE void bootloader_cleanup(void);
BOOTCODE void bootloader_download(void);

#endif /* BOOTLOADER_H */


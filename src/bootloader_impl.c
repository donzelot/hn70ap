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


#include "bootloader.h"

/* ------------------------------------------------------------------------- */
/* Initialize all hardware needed by the bootloader */
BOOTCODE void bootloader_inithardware(void)
{
  /* Initialize UART4 */
  /* Initialize SPI2 */
  /* Initialize external flash */
  /* Initialize LEDs */
  /* Initialize Button */
}

/* ------------------------------------------------------------------------- */
/* Deinitialize all hardware that was initialized.
 * Specially important for SPI
 */
BOOTCODE void bootloader_stophardware(void)
{
  /* Disable SPI2 */
  /* Disable UART4 */
}

/* ------------------------------------------------------------------------- */
/* Return the state of the on-board button */
BOOTCODE bool bootloader_buttonpressed(void)
{
  return false;
}

/* ------------------------------------------------------------------------- */
/* Read the contents of the external flash and determine if a valid
 * software image is present.
 */
BOOTCODE bool bootloader_checkupdate(void)
{
  return false;
}

/* ------------------------------------------------------------------------- */
/* Copy the firmware (supposed valid) from the external flash to the
 * internal stm32 flash. Return true on success, false on failure.
 */
BOOTCODE bool bootloader_apply(void)
{
  return false;
}

/* ------------------------------------------------------------------------- */
/* Handle a download protocol to fill the external flash from data received
 * through the UART
 */
BOOTCODE void bootloader_download(void)
{
}


############################################################################
# configs/hn70ap/src/Makefile
#
#   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#   Copyright (C) 2017 Sebastien Lorquet. All rights reserved.
#   Author: Sebastien Lorquet <sebastien@lorquet.fr>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS  = bootloader.c bootloader_impl.c
CSRCS += bootloader_gpio.c bootloader_uart.c bootloader_spi.c bootloader_intflash.c
CSRCS += bootloader_crc.c bootloader_tlv.c bootloader_spiflash.c
CSRCS += hn70ap_boot.c hn70ap_spi.c hn70ap_leds.c

#mandatory for firmware update
CSRCS += driver_mtdchar.c
CSRCS += hn70ap_flash.c

#mandatory for apps
CSRCS += hn70ap_eeprom.c

ifeq ($(CONFIG_HN70AP_SCREEN),y)
CSRCS += hn70ap_oled.c
endif

ifeq ($(CONFIG_HN70AP_ETHERNET),y)
CSRCS += hn70ap_ethernet.c
endif

ifeq ($(CONFIG_HN70AP_RADIO),y)
CSRCS += hn70ap_genradio.c
endif

ifeq ($(CONFIG_ARCH_IDLE_CUSTOM),y)
CSRCS += stm32_idle.c
endif

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS += stm32_ostest.c
endif


hn70ap_boot.c: grxversion.h
bootloader_impl.c: grxversion.h

.PHONY: hn70ap_boot.c bootloader_impl.c grxversion.h
grxversion.h:
	$(shell echo "#define GRXVERSION \"$(shell git describe --abbrev=40 --always --dirty)\"" > $@)

include $(TOPDIR)/configs/Board.mk


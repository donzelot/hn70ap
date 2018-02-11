############################################################################
# configs/hn70ap/src/Makefile
#
#   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#   Copyright (C) 2017 Sebastien F4GRX. All rights reserved.
#   Author: Sebastien F4GRX <f4grx@f4grx.net>
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
CSRCS  = bootloader.c hn70ap_boot.c hn70ap_spi.c hn70ap_leds.c
CSRCS += hn70ap_flash.c hn70ap_eeprom.c

ifeq ($(CONFIG_ARCH_IDLE_CUSTOM),y)
CSRCS += stm32_idle.c
endif

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS += stm32_ostest.c
endif

stm32_boot.c: grxversion.h

.PHONY: stm32_boot.c grxversion.h
grxversion.h:
	$(shell echo "#define GRXVERSION \"$(shell git describe --always --dirty)\"" > $@)

include $(TOPDIR)/configs/Board.mk

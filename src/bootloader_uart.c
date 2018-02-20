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
#include "bootloader_uart.h"

struct uart_params {
  uint32_t base;
  uint32_t ckenreg;
  uint32_t ckenbit;
};

static const struct uart_params g_uart[] BOOTRODATA = {
  [0] = {0x40011000, RCC_APB2ENR,  4},
  [1] = {0x40004400, RCC_APB1ENR, 17},
  [2] = {0x40004800, RCC_APB1ENR, 18},
  [3] = {0x40004C00, RCC_APB1ENR, 19},
  [4] = {0x40005000, RCC_APB1ENR, 20},
  [5] = {0x40011400, RCC_APB2ENR,  5},
  [6] = {0x40007800, RCC_APB1ENR, 30},
  [7] = {0x40007C00, RCC_APB1ENR, 31}
};

BOOTCODE void bootloader_uart_init(uint32_t uartid)
{
  uint32_t base;

  if( (uartid < 1) || (uartid > 8) )
    {
      return;
    }
  uartid -= 1;

  base = g_uart[uartid].base;

  //Enable clock to UART peripheral
  modreg32(g_uart[uartid].ckenreg, 1<<g_uart[uartid].ckenbit, 0);

  //configure registers for a simple uart, 8 bits, 1 stop bit, no parity
  //CR1: OVER8=0, UE=1, M=0, WAKE=0, PCE=0, no parity, no interrupts, RE=1, RWU=0
  modreg32(
    base + REGOFF_USART_CR1,
    USART_CR1_UE | USART_CR1_RE | USART_CR1_TE,
    0
    );

  //CR2: LINEN=0, STOP=00, CLKEN=0, no cpol, no cpha, no lbcl, no lin
  modreg32(base + REGOFF_USART_CR2, 0, 0);

  //CR3: ONEBIT=0, no interrupts, CTSE=0, RTSE=0, no dma, SCEN=0, NACK=0, HDSEL=0, no irda
  modreg32(base + REGOFF_USART_CR3, 0, 0);
}

/*
 * Fclk      baud      float     int(16times) hex   int part   frac part      approximated
 * 16 MHz    115200    8,68055   138          0x8A  8          10/16 = 0.625  8.625
 */
BOOTCODE void bootloader_uart_setbaud(uint32_t uartid, uint32_t baud)
{
  uint32_t base;
  if( (uartid < 1) || (uartid > 8) )
    {
      return;
    }
  uartid -= 1;
  base = g_uart[uartid].base;

  //compute using current value for clock, assuming OVER8=0
  //BAUD = fclk / (16 x DIV)
  //16 x DIV x BAUD = fclk
  // DIVfloat = fclk / (16 x BAUD) (DIV is a FLOAT)
  // DIVint   = (16 x fclk) / (16 x BAUD) (DIV is a INT with 4-bit frac part)
  // DIVint   = fclk / BAUD (DIV is a INT with 4-bit frac part)

  //divide
  baud = CLOCK_SPEED / baud;
  //mask high bits
  baud &= 0xFFFF;
  putreg32(base + REGOFF_USART_BRR, baud);
}

BOOTCODE void bootloader_uart_send(uint32_t uartid, int data)
{
  uint32_t reg;
  if( (uartid < 1) || (uartid > 8) )
    {
      return;
    }
  uartid -= 1;

  reg = g_uart[uartid].base;
  reg += REGOFF_USART_SR;

  //wait until the tx buffer is not empty
  while(!(getreg32(reg) & USART_SR_TXE));

  //send a single byte
  putreg32(g_uart[uartid].base + REGOFF_USART_DR, data&0xFF);
}

BOOTCODE void bootloader_uart_write_string(uint32_t uartid, const char *s)
{
  while(*s)
    {
      bootloader_uart_send(uartid, *s++);
    }
}


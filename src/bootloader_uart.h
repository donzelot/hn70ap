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

#ifndef BOOTLOADER_UART
#define BOOTLOADER_UART

#include <stdint.h>

//reg offsets
#define USART_OFF_SR     0x00
#define USART_OFF_DR     0x04
#define USART_OFF_BRR    0x08
#define USART_OFF_CR1    0x0C
#define USART_OFF_CR2    0x10
#define USART_OFF_CR3    0x14

//bits values
#define USART_SR_CTS         (1<<9)
#define USART_SR_LBD         (1<<8)
#define USART_SR_TXE         (1<<7)
#define USART_SR_TC          (1<<6)
#define USART_SR_RXNE        (1<<5)
#define USART_SR_IDLE        (1<<4)
#define USART_SR_ORE         (1<<3)
#define USART_SR_NF          (1<<2)
#define USART_SR_FE          (1<<1)
#define USART_SR_PE          (1<<0)

#define USART_CR1_OVER8      (1<<15)
#define USART_CR1_UE         (1<<13)
#define USART_CR1_M          (1<<12)
#define USART_CR1_WAKE       (1<<11)
#define USART_CR1_PCE        (1<<10)
#define USART_CR1_PS         (1<<9)
#define USART_CR1_PEIE       (1<<8)
#define USART_CR1_TXEIE      (1<<7)
#define USART_CR1_TCIE       (1<<6)
#define USART_CR1_RXNEIE     (1<<5)
#define USART_CR1_IDLEIE     (1<<4)
#define USART_CR1_TE         (1<<3)
#define USART_CR1_RE         (1<<2)
#define USART_CR1_RWU        (1<<1)
#define USART_CR1_SBK        (1<<0)

#define USART_CR2_LINEN      (1<<14)
#define USART_CR2_STOP_SHIFT 12
#define USART_CR2_STOP_MASK  (3<< USART_CR2_STOP_SHIFT)
#define USART_CR2_CLKEN      (1<<11)
#define USART_CR2_CPOL       (1<<10)
#define USART_CR2_CPHA       (1<<9)
#define USART_CR2_LBCL       (1<<8)
#define USART_CR2_LBDIE      (1<<7)
#define USART_CR2_LBDL       (1<<6)
#define USART_CR2_ADD_SHIFT  0
#define USART_CR2_ADD_MASK   (31 << USART_CR2_ADD_SHIFT)

#define USART_CR3_ONEBIT     (1<<11)
#define USART_CR3_CTSIE      (1<<10)
#define USART_CR3_CTSE       (1<<9)
#define USART_CR3_RTSE       (1<<8)
#define USART_CR3_DMAT       (1<<7)
#define USART_CR3_DMAR       (1<<6)
#define USART_CR3_SCEN       (1<<5)
#define USART_CR3_NACK       (1<<4)
#define USART_CR3_HDSEL      (1<<3)
#define USART_CR3_IRLP       (1<<2)
#define USART_CR3_IREN       (1<<1)
#define USART_CR3_EIE        (1<<0)

void bootloader_uart_init(uint32_t uartid);
void bootloader_uart_setbaud(uint32_t uartid, uint32_t baud);
void bootloader_uart_send(uint32_t uartid, int data);
void bootloader_uart_write_string(uint32_t uartid, const char *s);

#endif /* BOOTLOADER_UART */


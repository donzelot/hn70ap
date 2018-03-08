/*******************************************************************************
 * configs/hn70ap/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ******************************************************************************/

#ifndef __CONFIG_HN70AP_INCLUDE_BOARD_H
#define __CONFIG_HN70AP_INCLUDE_BOARD_H

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Clocking *******************************************************************/
/* The hn70ap board features a 20MHz crystal for HSI, and a 32kHz RTC backup
 * crystal.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL config
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 20000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 20           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 360          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : TODO         (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Disabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 20MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        20000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (20,000,000 / 20) * 360
 *         = 360,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 360,000,000 / 2 = 180,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(20)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(360)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  180000000ul

/* AHB clock (HCLK) is SYSCLK (180MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (45MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (90MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* Alternate function pin selections ************************************************/
/* The only pins declared here are the one required for the drivers in arch.
 * All board specific pins (LEDs, CS, IRQs) are declared in the local header only.
 * These resources are either available to the board directory only, or via
 * specific drivers.
 */

#define GPIO_UART4_RX GPIO_UART4_RX_2 /* PC10 */
#define GPIO_UART4_TX GPIO_UART4_TX_2 /* PC11 */

/* I2C - For 24AA02E48 EEPROM */

#define GPIO_I2C3_SCL GPIO_I2C3_SCL_1 /* PA8 */
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_1 /* PC9 */

/* SPI - For SST26F064 Flash */

#define GPIO_SPI2_MISO GPIO_SPI2_MISO_1 /* PB14 */
#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI_1 /* PB15 */
#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_1 /*  PB10 */
#define DMACHAN_SPI2_RX DMAMAP_SPI2_RX
#define DMACHAN_SPI2_TX DMAMAP_SPI2_TX

/* SPI - For both si4463 transceivers */

#define GPIO_SPI4_MISO GPIO_SPI4_MISO_1 /* PE5 */
#define GPIO_SPI4_MOSI GPIO_SPI4_MOSI_1 /* PE6 */
#define GPIO_SPI4_SCK  GPIO_SPI4_SCK_1  /* PE2 */
#define DMACHAN_SPI4_RX DMAMAP_SPI4_RX_1
#define DMACHAN_SPI4_TX DMAMAP_SPI4_TX_1

/* Ethernet - Not all pins can be configured, some of them are dedicated. */
#define GPIO_ETH_RMII_TXD0  GPIO_ETH_RMII_TXD0_1 /* PB12 */
#define GPIO_ETH_RMII_TXD1  GPIO_ETH_RMII_TXD1_1 /* PB13 */
#define GPIO_ETH_RMII_TX_EN GPIO_ETH_RMII_TX_EN_1 /* PB11 */

/* LEDs stuff */

#define BOARD_NLEDS 7

/* Unused definitions required to build the led driver */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 0
#define LED_INIRQ        0
#define LED_SIGNAL       0
#define LED_ASSERTION    0
#define LED_PANIC        0

/************************************************************************************
 * Public Data
 ************************************************************************************/
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HN70AP_INCLUDE_BOARD_H */

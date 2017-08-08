/****************************************************************************
 * configs/grxbetastamp/src/bootloader.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
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

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define BOOT_STACKSIZE  (16 * 1024)
#define BOOT_STACK      ((unsigned)&_ebss+ BOOT_STACKSIZE-4)
#define BOOT_PERIPHERAL_INTERRUPTS   16

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Chip-specific entrypoint */

extern void __boot_start(void);

/* Common exception entrypoint */

extern void __boot_exception_common(void);

/************************************************************************************
 * Public data
 ************************************************************************************/

/* Provided by the linker script to indicate the end of the BSS and start of text */

extern char _ebss;
extern char _stext;

/* The v7m vector table consists of an array of function pointers, with the first
 * slot (vector zero) used to hold the initial stack pointer.
 *
 * As all exceptions (interrupts) are routed via exception_common, we just need to
 * fill this array with pointers to it.
 *
 * Note that the [ ... ] designated initialiser is a GCC extension.
 */

unsigned __boot_vectors[] __attribute__((section(".boot.vectors"))) =
{
  BOOT_STACK,              /* Initial stack */
  (unsigned)&__boot_start, /* Reset exception handler */

  /* Vectors 2 - n point directly at the generic handler */

  [2 ... (15 + BOOT_PERIPHERAL_INTERRUPTS)] = (unsigned)&__boot_exception_common
};

#define BOOTCODE __attribute__(( section(".boot.text") ))

BOOTCODE __attribute__((naked)) void __boot_app(void)
  {
    __asm__ __volatile__ ("\t"
        /* load SP (from 08004000) */

        "movw r0, #0x4000              \n\t"
        "movt r0, #0x0800              \n\t"
        "ldr  sp, [r0]                 \n\t"

        /* load LR (from 08004004) */

        "movw r0, #0x4004              \n\t"
        "movt r0, #0x0800              \n\t"
        "ldr  r0, [r0]                 \n\t"
        "mov  lr, r0                   \n\t"

        /* setup VTOR (at E000ED08) to remap vectors*/

        "movw r0, #0xED08        /*adr lo*/ \n\t"
        "movt r0, #0xE000        /*adr hi*/ \n\t"
        "movw r1, #0x4000        /*val lo*/ \n\t"
        "movt r1, #0x0800        /*val hi*/ \n\t"
        "str  r1, [r0]           /*store value at address*/ \n\t"

        "bx   lr                 /*take the jump*/ \n"
    );
  }

BOOTCODE void __boot_start(void)
  {
    __boot_app();
    while(1);
  }

BOOTCODE void __boot_exception_common(void)
  {
    while(1);
  }


/****************************************************************************
 * config/hn70ap/src/stm32_appinit.c
 *
 *   Copyright (C) 2012, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <nuttx/mtd/mtd.h>

#include "stm32.h"
#include "hn70ap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct mtd_partition_info_s
{
  const char *name;
  size_t      blocks;  //number of blocks (for sst26 this is 256 bytes)
  bool        smartfs; //Enable SmartFS on this partition
};

static const struct mtd_partition_info_s parts[] =
{
  {"firmware",  64, false},
  {"storage" , 128, false}
};

#define PARTCOUNT (sizeof(parts)/sizeof(parts[0]))

FAR struct mtd_dev_s *mtdparts[PARTCOUNT];

static int flash_initialize(void)
{
  FAR struct spi_dev_s *spi2;
  FAR struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
  int ret;

  int partno;
  int partoffset;

  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 2\n");

  spi2 = stm32_spibus_initialize(2);
  if (!spi2)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port 2\n");

  /* Now bind the SPI interface to the SST25F064 SPI FLASH driver. */

  syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");

  mtd = sst26_initialize_spi(spi2);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 2 to the SPI FLASH driver\n");
      return -ENODEV;
    }
  syslog(LOG_INFO, "Successfully bound SPI port 2 to the SPI FLASH driver\n");

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      ferr("ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  /* Now create partitions on the FLASH device */

  partoffset = 0;
  for(partno = 0; partno<PARTCOUNT; partno++)
    {
      mtdparts[partno] = mtd_partition(mtd, partoffset, parts[partno].blocks);
      partoffset  += parts[partno].blocks;

      if (mtdparts[partno] == NULL)
        {
          ferr("ERROR: failed to create partition %s\n", partname);
          continue;
        }

      /* Now initialize a SMART Flash block device and bind it
       * to the MTD device.
       */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
      if(parts[partno].is_smart)
        {
          sprintf(partref, "p%d", partno);
          smart_initialize(0, mtdpart[partno], partref);
        }
#endif

#if defined(CONFIG_MTD_PARTITION_NAMES)
      mtd_setpartitionname(mtdparts[partno], partname);
#endif
    }
  return OK;
}


/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 *   CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_LIB_BOARDCTL=n :
 *     Called from board_initialize().
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  flash_initialize();

  return OK;
}

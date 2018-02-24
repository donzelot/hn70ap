/****************************************************************************
 * config/hn70ap/src/hn70ap_flash.c
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
#include "driver_mtdchar.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
#ifndef CONFIG_MTD_PARTITION
#error MTD partition support is required
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct mtd_partition_info_s
{
  const char *name;
  size_t      blocks;  //number of blocks (for sst26 this is 256 bytes)
  bool        smartfs; //Enable SmartFS on this partition
};

/* Total space is 64 Mbit, 8 MB, 2048 sectors, 32768 blocks */
static const struct mtd_partition_info_s parts[] =
{
  {"firmware",   8192, false}, // 2MB
  {"storage" , 24576, false}   // 6MB
};

#define PARTCOUNT (sizeof(parts)/sizeof(parts[0]))

FAR struct mtd_dev_s *mtdparts[PARTCOUNT];

int hn70ap_flash_initialize(void)
{
  FAR struct spi_dev_s *spi2;
  FAR struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
  int ret;

  int partno;
  int partoffset;

  /* Get the SPI port */

  spi2 = stm32_spibus_initialize(2);
  if (!spi2)
    {
      _err("ERROR: Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the SST25F064 SPI FLASH driver. */

  mtd = sst26_initialize_spi(spi2);
  if (!mtd)
    {
      _err("ERROR: Failed to bind SPI port 2 to the SPI FLASH driver\n");
      return -ENODEV;
    }

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      _err("ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }
  syslog(
    LOG_INFO,
    "SST26: %d erase blocks each %d bytes (total %d kB), blocksize %d\n",
    geo.neraseblocks,
    geo.erasesize, 
    (geo.neraseblocks * geo.erasesize) >> 10,
    geo.blocksize
    );

  /* Now create partitions on the FLASH device */

  partoffset = 0;
  for(partno = 0; partno<PARTCOUNT; partno++)
    {
      mtdparts[partno] = mtd_partition(mtd, partoffset, parts[partno].blocks);
      partoffset  += parts[partno].blocks;

      if (mtdparts[partno] == NULL)
        {
          _err("ERROR: failed to create partition %s\n", parts[partno].name);
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
      mtd_setpartitionname(mtdparts[partno], part[parno].name);
#endif
      syslog(LOG_INFO, "SST26: Created partition %s (%d blocks)\n", parts[partno].name, parts[partno].blocks);

    }

  _info("before mtdchar VTOR=%08X\n", getreg32(0xe000ed08));

  /* Create char device for firmware update partition */
  mtdchar_register(mtdparts[0], "/dev/firmware");
  _info("Registered /dev/firmware\n");

  return OK;
}


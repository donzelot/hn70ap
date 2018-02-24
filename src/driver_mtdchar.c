/****************************************************************************
 * configs/hn70ap/src/driver_mtdchar.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <sys/types.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>

#include "driver_mtdchar.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Types
 ****************************************************************************/

/* Private data attached to the inode */

struct mtdchar_dev_s
{
  struct mtd_dev_s *mtd;  //Underlying flash device
  uint32_t          size; //Total number of bytes in device (for seek)
  uint8_t           refs; //Simultaneous openings
  sem_t             sem;  //Access serialization
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mtdchar_open(FAR struct file *filep);
static int     mtdchar_close(FAR struct file *filep);
static off_t   mtdchar_seek(FAR struct file *filep, off_t offset, int whence);
static ssize_t mtdchar_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t mtdchar_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     mtdchar_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver operations */

static const struct file_operations mtdchar_fops =
{
  mtdchar_open,  /* open */
  mtdchar_close, /* close */
  mtdchar_read,  /* read */
  mtdchar_write, /* write */
  mtdchar_seek,  /* seek */
  mtdchar_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0           /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdchar_semtake
 *
 * Acquire a resource to access the device.
 * The purpose of the semaphore is to block tasks that try to access the
 * EEPROM while another task is actively using it.
 *
 ****************************************************************************/

static void mtdchar_semtake(FAR struct mtdchar_dev_s *mtdchardev)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&mtdchardev->sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(errno == EINTR || errno == ECANCELED);
    }
}

/****************************************************************************
 * Name: mtdchar_semgive
 *
 * Release a resource to access the device.
 *
 ****************************************************************************/

static inline void mtdchar_semgive(FAR struct mtdchar_dev_s *mtdchardev)
{
  sem_post(&mtdchardev->sem);
}

/****************************************************************************
 * Driver Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdchar_open
 *
 * Description: Open the device
 *
 ****************************************************************************/

static int mtdchar_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdchar_dev_s *mtdchardev;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  mtdchardev = (FAR struct mtdchar_dev_s *)inode->i_private;
  mtdchar_semtake(mtdchardev);

  /* Increment the reference count */

  if ((mtdchardev->refs + 1) == 0)
    {
      ret = -EMFILE;
    }
  else
    {
      mtdchardev->refs += 1;
    }

  mtdchar_semgive(mtdchardev);
  return ret;
}

/****************************************************************************
 * Name: mtdchar_close
 *
 * Description: Close the block device
 *
 ****************************************************************************/

static int mtdchar_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdchar_dev_s *mtdchardev;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  mtdchardev = (FAR struct mtdchar_dev_s *)inode->i_private;
  mtdchar_semtake(mtdchardev);

  /* Decrement the reference count. I want the entire close operation
   * to be atomic wrt other driver operations.
   */

  if (mtdchardev->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      mtdchardev->refs -= 1;
    }

  mtdchar_semgive(mtdchardev);
  return ret;
}

/****************************************************************************
 * Name: mtdchar_seek
 *
 * Remark: Copied from bchlib
 *
 ****************************************************************************/

static off_t mtdchar_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct mtdchar_dev_s *mtdchardev;
  off_t                   newpos;
  int                     ret;
  FAR struct inode        *inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  mtdchardev = (FAR struct mtdchar_dev_s *)inode->i_private;
  mtdchar_semtake(mtdchardev);

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = mtdchardev->size + offset;
      break;

    default:
      /* Return EINVAL if the whence argument is invalid */

      mtdchar_semgive(mtdchardev);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the end
   *   of the existing data in the file. If data is later written at this point,
   *   subsequent reads of data in the gap shall return bytes with the value 0
   *   until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second.  But return EINVAL if
   *
   *  "...the resulting file offset would be negative for a regular file, block
   *   special file, or directory."
   */

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
      finfo("SEEK newpos %d\n",newpos);
    }
  else
    {
      ret = -EINVAL;
    }

  mtdchar_semgive(mtdchardev);
  return ret;
}

/****************************************************************************
 * Name: mtdchar_read
 ****************************************************************************/

static ssize_t mtdchar_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct mtdchar_dev_s *mtdchardev;
  FAR struct inode        *inode = filep->f_inode;
  int                      ret;

  DEBUGASSERT(inode && inode->i_private);
  mtdchardev = (FAR struct mtdchar_dev_s *)inode->i_private;

  mtdchar_semtake(mtdchardev);

  /* Clamp len to avoid crossing the end of the memory */

  if ((filep->f_pos + len) > mtdchardev->size)
    {
      len = mtdchardev->size - filep->f_pos;
    }

  if (len == 0)
    {
      /* We are at end of file */

      ret = 0;
      goto done;
    }

  ret = len;

  /* Do the read */

  /* Update the file position */

  filep->f_pos += len;

done:
  mtdchar_semgive(mtdchardev);
  return ret;
}

/****************************************************************************
 * Name: mtdchar_write
 ****************************************************************************/

static ssize_t mtdchar_write(FAR struct file *filep, FAR const char *buffer,
                            size_t len)
{
  FAR struct mtdchar_dev_s *mtdchardev;
  FAR struct inode        *inode = filep->f_inode;
  int                      ret   = -EACCES;

  DEBUGASSERT(inode && inode->i_private);
  mtdchardev = (FAR struct mtdchar_dev_s *)inode->i_private;

  /* Forbid writes past the end of the device */

  if (filep->f_pos >= mtdchardev->size)
    {
      return -EFBIG;
    }

  /* Clamp len to avoid crossing the end of the memory */

  if ((len + filep->f_pos) > mtdchardev->size)
    {
      len = mtdchardev->size - filep->f_pos;
    }

  ret = len; /* save number of bytes written */

  mtdchar_semtake(mtdchardev);

  /* Do the write */

  /* Update the file position */

  filep->f_pos += len;

done:
  mtdchar_semgive(mtdchardev);
  return ret;
}

/****************************************************************************
 * Name: mtdchar_ioctl
 *
 * Description: 
 *
 ****************************************************************************/

static int mtdchar_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct mtdchar_dev_s *mtdchardev;
  FAR struct inode         *inode = filep->f_inode;
  int                       ret   = 0;
  FAR struct mtdchar_req_s *req = (FAR struct mtdchar_req_s *)arg;

  DEBUGASSERT(inode && inode->i_private);
  mtdchardev = (FAR struct mtdchar_dev_s *)inode->i_private;

  mtdchar_semtake(mtdchardev);
  switch (cmd)
    {
      case MTDCHAR_ERASE:  ret = MTD_ERASE (mtdchardev->mtd, req->block, req->count); break;
      case MTDCHAR_BREAD:  ret = MTD_BREAD (mtdchardev->mtd, req->block, req->count, req->buf); break;
      case MTDCHAR_BWRITE: ret = MTD_BWRITE(mtdchardev->mtd, req->block, req->count, req->buf); break;
      default:             ret = MTD_IOCTL(mtdchardev->mtd, cmd, arg);
    }

  mtdchar_semgive(mtdchardev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdchar_initialize
 *
 * Description: Bind a EEPROM driver to an I2C bus. The user MUST provide
 * a description of the device geometry, since it is not possible to read
 * this information from the device (contrary to the SPI flash devices).
 *
 ****************************************************************************/

int mtdchar_register(FAR struct mtd_dev_s *mtd, FAR char *devname)
{
  FAR struct mtdchar_dev_s *mtdchardev;
  struct mtd_geometry_s geo;
  int ret;

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if(ret != 0)
    {
      return ret;
    }

  mtdchardev = kmm_zalloc(sizeof(struct mtdchar_dev_s));

  if (!mtdchardev)
    {
      return -ENOMEM;
    }

  sem_init(&mtdchardev->sem, 0, 1);

  mtdchardev->mtd  = mtd;
  mtdchardev->size = geo.neraseblocks * geo.erasesize;

  return register_driver(devname, &mtdchar_fops, 0666, mtdchardev);
}


/****************************************************************************
 * hn70ap/apps/update/update_internal.h
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

#ifndef UPDATE_INTERNAL_H
#define UPDATE_INTERNAL_H

#include <stdint.h>
#include <hn70ap/update.h>

struct update_context_s
{
  /* flash storage info */
  int                    mtdfd; /* FD for MTD device */
  uint32_t               block_len; /*flash block size*/
  uint32_t               block_count; /* Number of blocks in the partition */

  uint32_t               block_id; /*flash page sequence number */

  /* Buffers */
  uint8_t               *header; /*storage for header block (written last)*/
  uint8_t               *block; /*storage for flash block*/

  /* Pending data */
  uint32_t               block_received; /* number of bytes of current block received so far */
  uint32_t               total_received; /* total number of UPDATE bytes received so far */

  /* Update info */
  bool                   header_received;
  struct update_header_s update;
  uint32_t               datacrc; /* Computed image CRC */
  bool                   done; //completion marker
};

int update_serial(struct update_context_s *ctx);
int update_tftp(struct update_context_s *ctx, const char *server, const char *remote_filename);
int update_status(int mtdfd);
int update_cancel(int mtdfd);

int update_write_start(struct update_context_s *ctx);
int update_write(struct update_context_s *ctx, uint8_t *buf, uint32_t len);

#endif

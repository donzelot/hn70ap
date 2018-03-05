/****************************************************************************
 * hn70ap/apps/update/update_serial.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/ioctl.h>
#include <fcntl.h>

#include <hn70ap/crc.h>
#include <hn70ap/hdlc.h>
#include <hn70ap/mtdchar.h>
#include <hn70ap/update.h>

/* Load an update image via a serial port. Protocol:
 * update is sent as a sequence of HDLC framed packets (RFC1662) containing the
 * following fields:
 * 1 byte 0x7E (packet separator)
 * 1 byte instruction (only one for now: 0x00 = write)
 * 2 bytes packet index big endian (increments starting at zero)
 * 2 bytes ISO14443-B CRC16
 * 1 byte 0x7E (packet separator)
 * Data bytes between separators are escaped as follows:
 * 0x7E becomes 0x7D 0x5E
 * 0x7D becomes 0x7D 0x5D
 * Valid packets are replied to by an ACK packet with a similar format:
 * 1 byte 0x7E (packet separator)
 * 1 byte write ACK = 0x01
 * 2 bytes packet index big endian of ACKed packet
 * 1 byte status (0 is OK 1 is repeat, any other is an error)
 * 2 bytes ISO14443-B/ISO3309/HDLC/RFC1662 CRC16
 * 1 byte 0x7E (packet separator)
 * An ack with an error status interrupts the transmission, which must be
 * restarted with the same packet (status 1) or from scratch, after a delay.
 * The total length of the transfer is known after the first 256 bytes (update
 * header) have been received.
 */

#define INST_WRITE 0 /*Write buffer to external flash*/
#define RESP_WRITE 1 /*Write confirmation*/

struct updateapp_context_s
{
  int      mtdfd; /* FD for MTD device */
  uint8_t *header; /*storage for header block (written last)*/
  uint8_t *block; /*storage for flash block*/
  uint32_t header_len; /*number of bytes of header received so far */
  uint32_t block_len; /*flash block size*/
  uint32_t block_received; /* number of bytes of current block received so far */
  uint32_t total_received; /* total number of UPDATE bytes received so far */
  uint32_t datacrc; /* Computed image CRC */
  uint16_t seq; /* packet sequence number */
  uint32_t block_id; /*flash page sequence number */

  struct update_header_s update;

  bool done; //completion marker
};

#define RESP_WRITE_OK 0
#define RESP_WRITE_ERRPARAMS 1
#define RESP_WRITE_ERRIO 2
#define RESP_WRITE_COMPLETE 3

/*----------------------------------------------------------------------------*/
int update_write(struct updateapp_context_s *ctx, uint8_t *buf, int len)
{
  uint8_t status = RESP_WRITE_OK;
  int retval = OK; //default will proceed with next packet after this one.
  int need;
  int off;
  int ret; /* ioctl return values */
  uint16_t seq; //received packet sequence number
  struct mtdchar_req_s req;

  len -= 1; //subtract instruction
  if(len < 2)
    {
      fprintf(stderr, "no sequence\n");
      status = RESP_WRITE_ERRPARAMS;
      goto done;
    }

  seq = buf[1];
  seq <<= 8;
  seq |= buf[2];
  len -= 2;

  buf += 3; //now we are looking at data

  if(len < 1)
    {
      fprintf(stderr, "no data\n");
      status = RESP_WRITE_ERRPARAMS;
      goto done;
    }

  /* Manage data block */
  if(seq == 0)
    {
      /* First block: reset context */
      ctx->seq = 0;
      ctx->header_len = 0;
      ctx->block_received = 0;
      ctx->total_received = 0;
      ctx->block_id = 1; //Start flash write right after header block
      ctx->datacrc = CRC32_INIT;
      //printf("header reset\n");
    }
  else
    {
      /* Next blocks: sequence number must be incremented */
      if(seq != (ctx->seq + 1) )
        {
          fprintf(stderr, "bad seq!\n");
          status = RESP_WRITE_ERRPARAMS;
          goto done;
        }
      ctx->seq = seq;
    }

  /* Process received data fragments */

  off = 0;
again:
  //printf("curpagelen=%d rxlen=%d \n", ctx->block_received, len);
  need = ctx->block_len - ctx->block_received; /* compute length needed to fill current page*/
  if(len <= need)
    {
      need = len;
      //printf("all block goes into page (rxoff %d)\n", off);
    }
  else
    {
      /* We have more data than what is needed to finish a page*/
      //printf("partial block (%d bytes, rxoff %d) goes into page\n", need, off);
    }

  memcpy(ctx->block + ctx->block_received, buf+off, need);
  ctx->block_received += need;
  ctx->total_received += need;
  len -= need;
  off += need;

  if(ctx->block_received == ctx->block_len)
    {
      if(ctx->header_len == 0)
        {
          //printf("HEADER\n");
          memcpy(ctx->header, ctx->block, 256);
          if(update_parseheader(&ctx->update, ctx->header, 256) != OK)
            {
              status = RESP_WRITE_ERRPARAMS; //failed
              goto done;
            }
          ctx->header_len = ctx->block_len;
          printf("ERASE:"); fflush(stdout);
          ret = ioctl(ctx->mtdfd, MTDIOC_BULKERASE, 0);
          if(ret < 0)
            {
              printf("FAILED\n");
              status = RESP_WRITE_ERRIO;
              retval = ERROR;
              goto done;
            }
          else
            {
              printf("OK\n");
            }
        }
      else
        {
          printf("WRITE [%u]:",ctx->block_id);
          /* Only data past 16k enters the CRC */
          if(ctx->block_id > 63)
            {
              ctx->datacrc = crc32_do(ctx->datacrc, ctx->block, 256);
            }
          req.block = ctx->block_id;
          req.buf   = ctx->block;
          req.count = 1;
          ret = ioctl(ctx->mtdfd, MTDCHAR_BWRITE, (unsigned long)&req);
          if(ret < 0)
            {
              printf("FAILED\n");
              status = RESP_WRITE_ERRIO;
              retval = ERROR;
              goto done;
            }
          else
            {
              printf("OK\n");
            }
            ctx->block_id += 1;
        }
      ctx->block_received = 0;
    }

  if(len > 0)
    {
      goto again;
    }

  if(ctx->header_len > 0) //Means we have parsed the header and know the total size.
    {
      //printf("managed %u of %u\n", ctx->total_received, ctx->update_size);
      if(ctx->total_received >= ctx->update.size)
        {
          //printf("Update complete.\n");
          /* Write last partial page */
          if(ctx->block_received>0)
            {
              printf("WRITE LAST [%u]:",ctx->block_id);
              if(ctx->block_id > 63)
                {
                  ctx->datacrc = crc32_do(ctx->datacrc, ctx->block, ctx->block_received);
                }
              req.block = ctx->block_id;
              req.buf   = ctx->block;
              req.count = 1;
              ret = ioctl(ctx->mtdfd, MTDCHAR_BWRITE, (unsigned long)&req);
              if(ret < 0)
                {
                  printf("FAILED\n");
                  status = RESP_WRITE_ERRIO;
                  retval = ERROR;
                  goto done;
                }
              else
                {
                  printf("OK\n");
                }
            }
          ctx->datacrc ^= CRC32_MASK;
          /* Check data CRC against info in header */
          if(ctx->datacrc != ctx->update.crc)
            {
              fprintf(stderr, "Data integrity error\n");
              status = RESP_WRITE_ERRPARAMS;
              retval = ERROR;
              goto done;
            }

          /* Copy header into first block of flash */
          printf("WRITE HEADER:");
          req.block = 0;
          req.buf   = ctx->header;
          req.count = 1;
          ret = ioctl(ctx->mtdfd, MTDCHAR_BWRITE, (unsigned long)&req);
          if(ret < 0)
            {
              printf("FAILED\n");
              status = RESP_WRITE_ERRIO;
              retval = ERROR;
              goto done;
            }
          else
            {
              printf("OK\n");
            }
          ctx->done = true;
          status = RESP_WRITE_COMPLETE;
        }
    }

  /* No more data in rx buffer */
done:
  buf[0] = RESP_WRITE;
  buf[3] = status;
  frame_send(stdout, buf, 4);
  return retval;
}

/*----------------------------------------------------------------------------*/
int update_doframe(struct updateapp_context_s *ctx, uint8_t *buf, int len)
{
  //printf("got frame, %d bytes\n", len);
  if(len < 1)
    {
      fprintf(stderr, "no instruction\n");
      return ERROR;
    }
  if(buf[0] == INST_WRITE)
    {
      return update_write(ctx, buf, len);
    }
  /* Other frames are discarded */
  return OK;
}

/*----------------------------------------------------------------------------*/
void update_serial(int mtdfd, int blocksize, int erasesize)
{
  int ret;
  uint8_t *pktbuf;

  struct updateapp_context_s ctx;

  /* Allocate storage for header */
  ctx.header = malloc(256);
  if(ctx.header==0)
    {
      fprintf(stderr, "alloc error!\n");
      return;
    }

  /* Allocate storage for flash page buffer */
  ctx.mtdfd = mtdfd;
  ctx.block_len = blocksize;
  ctx.block = malloc(256);
  if(ctx.block == 0)
    {
      fprintf(stderr, "alloc error!\n");
      goto retfree;
    }

  /* Allocate storage for protocol */
  pktbuf = malloc(1+2+256+2); //with room for inst seqnum and CRC
  if(pktbuf == 0)
    {
      fprintf(stderr, "alloc error!\n");
      goto retfree;
    }

  printf("hn70ap serial update waiting...\n");
  ctx.done = false;
  while(!ctx.done)
    {
      ret = frame_receive(stdin, pktbuf, 1+2+256+2);
      if(ret == 0)
        {
          printf("Timeout/RX problem!\n");
          goto retfree;
        }
      if(update_doframe(&ctx, pktbuf, ret) != OK)
        {
          printf("Transfer aborted!\n");
          goto retfree;
        }
    }
  printf("Transfer complete.\n");

retfree:
  free(pktbuf);
  free(ctx.block);
  free(ctx.header);
}


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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <termios.h>

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

#define CRC16_INIT 0xFFFF
#define CRC16_GOOD 0xF0B8

enum
{
  STATE_NOSYNC,
  STATE_SYNC,
  STATE_DATA,
  STATE_ESC
};

#define FDELIM 0x7E
#define FESC   0x7D

#define INST_WRITE 0 /*Write buffer to external flash*/
#define RESP_WRITE 1 /*Write confirmation*/

/*----------------------------------------------------------------------------*/
uint16_t crc16(uint16_t crc, uint8_t data)
{
  crc ^= data&0xFF;
  crc ^= (crc<<4)&0xFF;
  crc  = (uint16_t)(crc>>8)^
    (uint16_t)((uint16_t)(crc&0xFF)<<8)^
    (uint16_t)((uint16_t)(crc&0xFF)<<3)^
    (uint16_t)((crc&0xFF)>>4);

  return crc;
}

/*----------------------------------------------------------------------------*/
static inline void send_esc(FILE *out, uint8_t buf)
{
  if((buf == FESC) || (buf == FDELIM))
    {
      fputc(FESC, out);
      buf ^= 0x20;
    }
  fputc(buf, out);
}

/*----------------------------------------------------------------------------*/
static void frame_send(FILE *out, uint8_t *buf, int len)
{
  uint16_t crc = CRC16_INIT;
  struct termios term;  
  tcflag_t ofl;

  /* Turn the console into a raw uart */
  tcgetattr(fileno(stdout), &term);
  ofl = term.c_oflag;
  term.c_oflag &= ~ONLCR;
  tcsetattr(fileno(stdout), TCSANOW, &term);


  fputc(FDELIM, out);

  while(len--)
    {
      crc = crc16(crc, *buf);
      send_esc(out, *buf);
      buf++;
    }

  crc ^= 0xFFFF;
  send_esc(out,  crc    &0xFF);
  send_esc(out, (crc>>8)&0xFF);

  fputc(FDELIM, out);
  fflush(out);

  /* Restore serial console settings */
  term.c_oflag = ofl;
  tcsetattr(fileno(stdout), TCSANOW, &term);
}

/*----------------------------------------------------------------------------*/
static int frame_receive(FILE *in, uint8_t *buf, int maxlen)
{
  int ret;
  int frame_state = STATE_NOSYNC;
  int frame_len;
  uint16_t crc = CRC16_INIT;

  while(1)
    {
      ret = fgetc(in);
      //printf("%02X in state %d\n", ret, frame_state); fflush(stdout);
      switch(frame_state)
        {
          case STATE_NOSYNC: //Only wait for a SYNC byte
            if(ret == FDELIM)
              {
                frame_state = STATE_SYNC;
                continue;
              }
            break;

          case STATE_SYNC: //Do nothing until the first data byte
            if(ret != FDELIM)
              {
                frame_state = STATE_DATA;
                frame_len = 0;
                crc = crc16(crc, ret);
                if(frame_len == maxlen)
                  {
                    printf("frame overflow\n");
                    goto done;
                  }
                buf[frame_len++] = ret;
                continue;
              }
            break;

          case STATE_DATA: //Save data if no overflow. manage esc
            if(ret == FESC)
              {
                frame_state = STATE_ESC;
                continue;
              }
            else if(ret == FDELIM)
              {
                goto done;
              }
            else
              {
                crc = crc16(crc, ret);
                if(frame_len == maxlen)
                  {
                    printf("frame overflow\n");
                    goto done;
                  }
                buf[frame_len++] = ret;
                continue;
              }

          case STATE_ESC: //unescape char and back to data unless overflow
            ret ^= 0x20;
            crc  = crc16(crc, ret);
            if(frame_len == maxlen)
              {
                printf("frame overflow\n");
                goto done;
              }
            buf[frame_len++] = ret;
            frame_state = STATE_DATA;
            continue;
        }
    }

done:
  /* Frame complete. Check CRC */
  if(frame_len < 2)
    {
      return 0;
    }
  if(crc != CRC16_GOOD)
    {
      printf("bad frame\n");
      return 0;
    }

  //printf("crc=%04X last=%02X %02X\n",crc, buf[frame_len-2], buf[frame_len-1]);
  return frame_len - 2;
}

struct update_context_s
{
  uint8_t *header;
  uint8_t *block;
  int header_len; /*number of bytes of header received so far */
  int block_len; /*flash block size*/
  int block_received; /*number of bytes of current block received so far */
  int seq;
};

/*----------------------------------------------------------------------------*/
int update_write(struct update_context_s *ctx, uint8_t *buf, int len)
{
  uint8_t *block;
  uint8_t status = 0;
  int need;
  int off;

  len -= 1; //subtract instruction
  if(len < 2)
    {
      fprintf(stderr, "no sequence\n");
      status = 1;
      goto done;
    }

  ctx->seq = buf[1];
  ctx->seq <<= 8;
  ctx->seq |= buf[2];
  len -= 2;

  buf += 3; //now we are looking at data

  if(len < 1)
    {
      fprintf(stderr, "no data\n");
      status = 1;
      goto done;
    }

  /* Manage data block */
  if(ctx->seq == 0)
    {
      /* First block: reset context */
      ctx->header_len = 0;
      ctx->block_received = 0;
      printf("header reset\n");
    }
  else
    {
      /* Next blocks: sequence number must be incremented */
    }

  /* Process received data fragments */

  off = 0;
again:
  printf("curpagelen=%d rxlen=%d \n", ctx->block_received, len);
  need = ctx->block_len - ctx->block_received; /* compute length needed to fill current page*/
  if(len <= need)
    {
      need = len;
      printf("all block goes into page (rxoff %d)\n", off);
    }
  else
    {
      /* We have more data than what is needed to finish a page*/
      printf("partial block (%d bytes, rxoff %d) goes into page\n", need, off);
    }

  memcpy(ctx->block + ctx->block_received, buf+off, need);
  ctx->block_received += need;
  len -= need;
  off += need;

  if(ctx->block_received == ctx->block_len)
    {
      if(ctx->header_len == 0)
        {
          int i;
          printf("HEADER\n");
          memcpy(ctx->header, ctx->block, 256);
          for(i=0;i<256;i++) printf("%02X",ctx->header[i]); printf("\n");
          ctx->header_len = ctx->block_len;
        }
      else
        {
          printf("WRITE\n");
        }
      ctx->block_received = 0;
    }

  if(len > 0)
    {
      goto again;
    }

  /* No more data in rx buffer */
done:
  buf[0] = RESP_WRITE;
  buf[3] = status;
  frame_send(stdout, buf, 4);
  return OK;
}

/*----------------------------------------------------------------------------*/
int update_doframe(struct update_context_s *ctx, uint8_t *buf, int len)
{
  printf("got frame, %d bytes\n", len);
  if(len<1)
    {
      fprintf(stderr, "no instruction\n");
      return ERROR;
    }
  if(buf[0] == INST_WRITE)
    {
      update_write(ctx, buf, len);
    }
  return OK;
}

/*----------------------------------------------------------------------------*/
void update_serial(int mtdfd, int blocksize, int erasesize)
{
  int ret;
  uint8_t *pktbuf;

  struct update_context_s ctx;

  ctx.header = malloc(256);
  if(ctx.header==0)
    {
      fprintf(stderr, "alloc error!\n");
      return;
    }

  ctx.block_len = blocksize;
  ctx.block = malloc(256);
  if(ctx.block == 0)
    {
      fprintf(stderr, "alloc error!\n");
      return;
    }

  pktbuf = malloc(1+2+256+2); //with room for inst seqnum and CRC
  if(pktbuf == 0)
    {
      fprintf(stderr, "alloc error!\n");
      return;
    }

  printf("hn70ap serial update waiting...\n");
  while(true)
    {
      ret = frame_receive(stdin, pktbuf, 1+2+256+2);
      if(ret == 0)
        {
          return;
        }
      update_doframe(&ctx,pktbuf,ret);
    }

}


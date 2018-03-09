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

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <hn70ap/memio.h>
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
#define INST_SETSPEED 2 /*Set UART speed to 921600 bauds */
#define RESP_SETSPEED 3 /*Confirmation of high speed mode at next command */

#define RESP_STATUS_OK 0
#define RESP_STATUS_ERRPARAMS 1
#define RESP_STATUS_ERRIO 2
#define RESP_STATUS_COMPLETE 3

struct update_serialcontext_s
{
  struct update_context_s *update;
  uint16_t seq; /* packet sequence number */
  uint8_t *pktbuf;
};

/*----------------------------------------------------------------------------*/
int update_serial_setspeed(struct update_serialcontext_s *ctx, uint8_t *buf, int len)
{
  uint8_t status = RESP_STATUS_OK;
  int     retval = OK;
  uint32_t speed;
  struct termios term;  

  len -= 1; //subtract instruction
  if(len < 4)
    {
      fprintf(stderr, "no speed\n");
      status = RESP_STATUS_ERRPARAMS;
      goto done;
    }

  speed = PEEK_U32BE(buf+1);
  printf("Set speed to %u bauds\n", speed); fflush(stdout);

done:
  buf[0] = RESP_SETSPEED;
  buf[1] = status;
  frame_send(stdout, buf, 2);

  /* Now apply change */
  if(status == RESP_STATUS_OK)
    {
      tcgetattr(fileno(stdout), &term);
      cfsetspeed(&term, speed);
      tcsetattr(fileno(stdout), TCSADRAIN, &term);
    }

  return retval;
}

/*----------------------------------------------------------------------------*/
int update_serial_write(struct update_serialcontext_s *sctx, uint8_t *buf, int len)
{
  uint8_t status = RESP_STATUS_OK;
  int ret = OK;
  uint16_t seq; //received packet sequence number
  struct mtdchar_req_s req;

  len -= 1; //subtract instruction
  if(len < 2)
    {
      fprintf(stderr, "no sequence\n");
      status = RESP_STATUS_ERRPARAMS;
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
      status = RESP_STATUS_ERRPARAMS;
      goto done;
    }

  /* Manage data block */
  if(seq == 0)
    {
      sctx->seq = 0;
      ret = update_write_start(ctx);
      if(ret != OK)
        {
          fprintf(stderr, "bad write init!\n");
          status = RESP_STATUS_ERRPARAMS;
          goto done;
        }
    }
  else
    {
      /* Next blocks: sequence number must be incremented */
      if(seq != (sctx->seq + 1) )
        {
          fprintf(stderr, "bad seq!\n");
          status = RESP_STATUS_ERRPARAMS;
          goto done;
        }
      sctx->seq = seq;
    }

  /* Process received data fragments */
  ret = update_write(ctx->update, buf, len);

  if(ret == OK)
    {
      status = RESP_STATUS_OK;
    }
  else
    {
      sttaus = RESP_STATUS_ERRIO;
    }

  buf[0] = RESP_WRITE;
  buf[3] = status;
  frame_send(stdout, buf, 4);
  return ret;
}

/*----------------------------------------------------------------------------*/
int update_serial_doframe(struct update_serialcontext_s *sctx, uint8_t *buf, int len)
{
  //printf("got frame, %d bytes\n", len);
  if(len < 1)
    {
      fprintf(stderr, "no instruction\n");
      return ERROR;
    }
  if(buf[0] == INST_WRITE)
    {
      return update_serial_write(sctx, buf, len);
    }
  else if(buf[0] == INST_SETSPEED)
    {
      return update_serial_setspeed(sctx, buf, len);
    }
  /* Other frames are discarded */
  return OK;
}

/*----------------------------------------------------------------------------*/
void update_serial(struct update_context_s *ctx)
{
  int ret;
  struct termios term;  
  struct update_serialcontext_s sctx;

  sctx.update = ctx;

  /* Save the original UART config */
  tcgetattr(fileno(stdout), &term);

  /* Allocate storage for protocol */
  sctx.pktbuf = malloc(1+2+256+2); //with room for inst seqnum and CRC
  if(sctx.pktbuf == 0)
    {
      fprintf(stderr, "alloc error!\n");
      goto retfree;
    }

  printf("hn70ap serial update waiting...\n");
  ctx->done = false;
  while(!ctx->done)
    {
      ret = frame_receive(stdin, sctx.pktbuf, 1+2+256+2);
      if(ret == 0)
        {
          printf("Timeout/RX problem!\n");
          goto retfree;
        }
      ret = update_serial_doframe(sctx, sctx.pktbuf, ret);
      if( ret != OK)
        {
          printf("Transfer aborted!\n");
          goto retfree;
        }
    }
  printf("Transfer complete.\n");

retfree:
  /* Restore the original UART config */
  tcsetattr(fileno(stdout), TCSADRAIN, &term);

  free(pktbuf);
}


/****************************************************************************
 * hn70ap/apps/libhn70ap/hdlc.c
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

#include <termios.h>

#include <hn70ap/crc.h>

enum
{
  STATE_NOSYNC,
  STATE_SYNC,
  STATE_DATA,
  STATE_ESC
};

#define FDELIM 0x7E
#define FESC   0x7D

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
void frame_send(FILE *out, uint8_t *buf, int len)
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
int frame_receive(FILE *in, uint8_t *buf, int maxlen)
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


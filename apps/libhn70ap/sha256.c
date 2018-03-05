/****************************************************************************
 * hn70ap/apps/libhn70ap/sha256.c
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
#include <string.h>
#include <hn70ap/sha256.h>

//bitwise rotation
#define RSHI(w,bits) ( (w) >> (uint32_t)(bits) )
#define RROT(w,bits) ( RSHI(w, bits) | ( (w) << (32- (uint32_t)(bits) ) ) )

#define SS0(w)      ( RROT(w, 7) ^ RROT(w, 18) ^ RSHI(w,  3) )
#define SS1(w)      ( RROT(w,17) ^ RROT(w, 19) ^ RSHI(w, 10) )
#define LS0(w)      ( RROT(w, 2) ^ RROT(w, 13) ^ RROT(w, 22) )
#define LS1(w)      ( RROT(w, 6) ^ RROT(w, 11) ^ RROT(w, 25) )
#define CH(x,y,z)   ( (x & y) ^ ((~x) & z) )
#define MAJ(x,y,z)  ( (x & (y ^ z)) ^ (y & z) )

//One SHA-256 rounds. There are 256 of them.
#define ROUND(i) \
        temp = h + LS1(e) + CH(e,f,g) + sha256_k[i] + w[i]; \
        h = g; \
        g = f; \
        f = e; \
        e = d + temp; \
        d = c; \
        c = b; \
        b = a; \
        a = temp + LS0(b) + MAJ(b,c,d); \
        //printf("t=%2d %08X %08X %08X %08X %08X %08X %08X %08X\n",i,a,b,c,d,e,f,g,h);

#define ITERATIONS(CODE) \
        { CODE( 0) } { CODE( 1) } { CODE( 2) } { CODE( 3) } \
        { CODE( 4) } { CODE( 5) } { CODE( 6) } { CODE( 7) } \
        { CODE( 8) } { CODE( 9) } { CODE(10) } { CODE(11) } \
        { CODE(12) } { CODE(13) } { CODE(14) } { CODE(15) } \
        { CODE(16) } { CODE(17) } { CODE(18) } { CODE(19) } \
        { CODE(20) } { CODE(21) } { CODE(22) } { CODE(23) } \
        { CODE(24) } { CODE(25) } { CODE(26) } { CODE(27) } \
        { CODE(28) } { CODE(29) } { CODE(30) } { CODE(31) } \
        { CODE(32) } { CODE(33) } { CODE(34) } { CODE(35) } \
        { CODE(36) } { CODE(37) } { CODE(38) } { CODE(39) } \
        { CODE(40) } { CODE(41) } { CODE(42) } { CODE(43) } \
        { CODE(44) } { CODE(45) } { CODE(46) } { CODE(47) } \
        { CODE(48) } { CODE(49) } { CODE(50) } { CODE(51) } \
        { CODE(52) } { CODE(53) } { CODE(54) } { CODE(55) } \
        { CODE(56) } { CODE(57) } { CODE(58) } { CODE(59) } \
        { CODE(60) } { CODE(61) } { CODE(62) } { CODE(63) }

static const uint32_t sha256_k[] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

/*----------------------------------------------------------------------------*/
void sha256_init(struct sha256_s *state)
{
  state->total   = 0;
  state->len     = 0;
  state->hash[0] = 0x6a09e667;
  state->hash[1] = 0xbb67ae85;
  state->hash[2] = 0x3c6ef372;
  state->hash[3] = 0xa54ff53a;
  state->hash[4] = 0x510e527f;
  state->hash[5] = 0x9b05688c;
  state->hash[6] = 0x1f83d9ab;
  state->hash[7] = 0x5be0cd19;
}

/*----------------------------------------------------------------------------*/
static void _sha256_round(struct sha256_s *state)
{
  uint32_t a,b,c,d,e,f,g,h;
  uint32_t w[64],temp;
  register int i;

    //extend
    //break chunk into sixteen 32-bit big-endian words w[0..15]
    //printf("in: ");
    //for(i=0;i<64;i++) printf("%02X",state->data[i]);
    //printf("\n");

  for(i=0;i<16;i++)
    {
      w[i]= ((uint32_t)state->data[(i<<2)  ]<<24)
          | ((uint32_t)state->data[(i<<2)+1]<<16)
          | ((uint32_t)state->data[(i<<2)+2]<< 8)
          | ((uint32_t)state->data[(i<<2)+3]    );
    }

  for(i=16;i<64;i++)
    {
      w[i] = w[i-16] + SS0(w[i-15]) + w[i-7] + SS1(w[i- 2]);
    }
/*
    printf("init ");
    for(i=0;i<8;i++) printf("%08X ",state->hash[i]);
    printf("\n");
*/
  //import
  a=state->hash[0]; b=state->hash[1]; c=state->hash[2]; d=state->hash[3];
  e=state->hash[4]; f=state->hash[5]; g=state->hash[6]; h=state->hash[7];

  //main loop
#ifdef UNROLLED
  ITERATIONS(ROUND)
#else
  for(i=0;i<64;i++)
    {
      ROUND(i)
    }
#endif

  //export
  state->hash[0]+=a; state->hash[1]+=b; state->hash[2]+=c; state->hash[3]+=d;
  state->hash[4]+=e; state->hash[5]+=f; state->hash[6]+=g; state->hash[7]+=h;
}

/*----------------------------------------------------------------------------*/
void sha256_update(struct sha256_s *state, uint8_t *indata, uint32_t inlen)
{
  //try to fit data in buffer
  uint32_t room = 64 - state->len;

again:
  if(room > inlen)
    {
      //there is space to store every incoming data, no round required
      memcpy(state->data + state->len, indata, inlen);
      state->len += inlen;
      state->total += inlen;
      return; //we're done
    }

  //else, incoming data will not fit in the buffer
  //fit as much as possible
  memcpy(state->data + state->len, indata, room);

  //we have a full block now, so we process it
  _sha256_round(state);
  state->len    = 0;
  state->total += 64;

  //This data has been integrated, look at next data
  inlen  -= 64;
  indata += 64;
  room    = 64;
  goto again;
}

/*----------------------------------------------------------------------------*/
void sha256_final(struct sha256_s *state, uint8_t *dest)
{
  //pad
  int i,j;
  int tot = state->len + 1 + 9;

  //printf("bytes in final block: %d\n",state->len);
  //printf("total bytes with padding: %d\n",tot);
  if(tot>63)
    {
      //padding will not fit in this block
      //finish it and hash it
      int thispad = 64-state->len;
      //printf("one more block with %d zeros\n", thispad);
      memset(state->data + state->len, 0, thispad);
      state->data[state->len] = 0x80; //one bit padding just after data
      _sha256_round(state);
      //now we're on a block boundary.
      //fill last buffer with zeros, then length
      memset(state->data, 0, 56);
    }
  else
    {
      //printf("that fits (len=%d)\n",state->len);
      //final block AND padding will fit in the last block
      //only one final round, pad to 56 bytes
      memset(state->data + state->len, 0, 56-state->len);
      state->data[state->len] = 0x80; //one bit padding just after data
    }

  //append length in bits
  state->total <<=3;
  state->data[63]= (state->total&0xFF); state->total >>=8;
  state->data[62]= (state->total&0xFF); state->total >>=8;
  state->data[61]= (state->total&0xFF); state->total >>=8;
  state->data[60]= (state->total&0xFF); state->total >>=8;
  state->data[59]= (state->total&0xFF); state->total >>=8;
  state->data[58]= (state->total&0xFF); state->total >>=8;
  state->data[57]= (state->total&0xFF); state->total >>=8;
  state->data[56]= (state->total&0xFF);
  _sha256_round(state);

  //done, map hash values into bytes
  for(i=0;i<8;i++)
    {
      for(j=0;j<4;j++)
        {
          dest[(i<<2)+(3-j)] = state->hash[i] & 0xff;
          state->hash[i] >>= 8;
        }
    }
}


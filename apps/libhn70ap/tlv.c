/****************************************************************************
 * hn70ap/apps/libhn70ap/tlv.c
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
#include <stddef.h>

#include <hn70ap/tlv.h>

static uint32_t tlv_parse(uint8_t *buf, uint32_t buflen, uint32_t *foundtag, uint32_t *foundlen)
    {
    uint32_t off=0;
    uint32_t tag,len;

    /*skip zeros while checking overflow*/
    while(buf[off]==0 && off < buflen)
        {
        off++;
        }

    if(off==buflen)
        {
        return 0;
        }

    /*read tag*/
    tag = buf[off];
    off++;
    if(off == buflen)
        {
        return 0;
        }
    if(tag == 0xFF)
        {
        //FF tag is normally valid but we reject it (erased mem)
        return 0;
        }

    if( (tag&0x1F)==0x1F)
        {
        tag <<= 8;
        tag |= buf[off];
        off++;
        if(off == buflen)
            {
            return 0;
            }
        }

  /* read len */
    len = buf[off];
    off++;
    if(off == buflen)
        {
        return 0;
        }
    if(len>0x84)
        {
        //Invalid length
        return 0;
        }
    if(len>0x7F)
        {
        uint32_t lenlen = len & 0x7F;
        len = 0;
        while(lenlen>0 && off!=buflen)
            {
            len <<= 8;
            len |= buf[off];
            off++;
            lenlen--;
            }
        if(off == buflen)
            {
            return 0;
            }
        }

    /* check overflow */
    if(len+off > buflen)
        {
        return 0;
        }

    /* return data */
    *foundtag = tag;
    *foundlen = len;
    return off;
}


/*----------------------------------------------------------------------------*/
/*
Description: - return length and pointer to required tag, or null if not found
*/
uint8_t *tlv_find(uint8_t *buf, uint32_t buflen, uint32_t searchtag, uint32_t *foundlen, int recurse)
    {
    uint32_t off;
    uint32_t tmptag, tmplen;
    uint8_t *content;

    uint8_t *ptr = buf;
    uint32_t len = buflen;

    while(len>0)
        {
        off = tlv_parse(ptr, len, &tmptag, &tmplen);
        if(off)
            {
            /* a tag was found */
            if (tmptag==searchtag)
                {
                /* correct tag, return */
                *foundlen = tmplen;
                return ptr+off;
                }
            else
                {
                /* not the correct tag. Recurse if composite and requested */
                if( ( (tmptag & 0x20) == 0x20) && (recurse > 0) )
                    {
                    /* the tag is composite and recursion is allowed */
                    content = tlv_find(ptr + off, tmplen, searchtag, foundlen, recurse - 1);
                    if (content)
                        {
                        return content; /* recursion has found the tag */
                        }
                    }
                }
            }
        else
            {
            //parser returned zero, so not valid.
            return NULL;
            }
        /* look at next tag */
        ptr += off + tmplen;
        len -= off + tmplen;
        }
    return NULL;
    }

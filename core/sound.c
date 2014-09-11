/*
 * Copyright (c) 2014, Peter Rutenbar <pruten@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include "../core/shoebill.h"


static uint8_t sound_dma_read_byte(uint16_t addr)
{
    if (addr == 0x804)
        return 0xff;
    return 0;
}

static void sound_dma_write_byte(uint16_t addr, uint8_t data)
{
    if (addr >= 0x800) {
        // registers
    }
    else if (addr >= 0x400) {
        // Buf B
    }
    else {
        // Buf A
        /*FILE *f = fopen("buf_a.dmp", "ab");
        if (f) {
            fwrite(&data, 1, 1, f);
            fclose(f);
        }*/
    }
}


void sound_dma_write_raw(uint16_t addr, uint8_t sz, uint32_t data)
{
    int32_t i;
    slog("sound_dma_write: addr=0x%04x sz=%u dat=0x%x\n", addr, sz, data);
    
    for (i = (sz-1) * 8; i >= 0; i -= 8)
        sound_dma_write_byte(addr, (data >> i) & 0xff);
}

uint32_t sound_dma_read_raw(uint16_t addr, uint8_t sz)
{
    uint32_t i, result = 0;
    slog("sound_dma_read: addr=0x%04x sz=%u\n", addr, sz);
    
    for (i=0; i<sz; i++) {
        result <<= 8;
        result |= sound_dma_read_byte(addr + i);
    }
    
    return result;
}
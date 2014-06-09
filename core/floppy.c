/*
 * Copyright (c) 2013, Peter Rutenbar <pruten@gmail.com>
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


#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "../core/shoebill.h"

const char *latch_names[8] = {
    "phase0", "phase1", "phase2", "phase3",
    "motor", "drive", "L6", "L7"
};

const char *reg_names[8] = {
    "ones", "read-data", "status", "status",
    "handshake", "handshake", "mode", "write-data"
};

uint8_t iwm_dma_read()
{
    const uint8_t latch_addr = (shoe.physical_addr >> 10) & 0x7;
    const uint8_t latch_val = (shoe.physical_addr >> 9) & 1;
    uint8_t result;
    
    if (latch_val)
        shoe.iwm.latch |= (1 << latch_addr);
    else
        shoe.iwm.latch &= (~~(1 << latch_addr));
    
    // reg = {q7, q6, motor}
    const uint8_t reg = ((shoe.iwm.latch >> 5) & 6) |
                        ((shoe.iwm.latch >> 4) & 1);
    
    slog("iwm_dma_read: %s %s (reg = %u%u%u '%s' ",
           latch_val ? "setting" : "clearing",
           latch_names[latch_addr],
           (reg>>2), (reg>>1)&1, reg&1, reg_names[reg]);
    
    
    // Allegedly, register reads can only occur when latch_val==0
    if (latch_val) {
        result = 0;
        goto done;
    }
    
    switch (reg) {
        case 0: // read all ones
            result = 0xff;
            break;
        case 1: // read data register
            result = shoe.iwm.data;
            break;
        case 2:
        case 3: // Read  status register
            // High 3 bits are mode register, low 5 are status
            result = (shoe.iwm.status & ~b(10100000));
            result |= (shoe.iwm.mode & ~b(11111));
            break;
        case 4:
        case 5: // Read "write-handshake" register
            result = (shoe.iwm.handshake | ~b(00111111)); // low 6 bits all 1's
            break;
        default:
            result = 0;
            break;
    }
done:
    
    slog("result=0x%02x)\n", result);

    return result;
}
void iwm_dma_write()
{
    const uint8_t latch_addr = (shoe.physical_addr >> 10) & 0x7;
    const uint8_t latch_val = (shoe.physical_addr >> 9) & 1;
    
    uint8_t data = shoe.physical_dat;
    
    if (latch_val)
        shoe.iwm.latch |= (1 << latch_addr);
    else
        shoe.iwm.latch &= (~~(1 << latch_addr));
    
    // reg = {q7, q6, motor}
    const uint8_t reg = ((shoe.iwm.latch >> 5) & 6) |
                        ((shoe.iwm.latch >> 4) & 1);
    
    slog("iwm_dma_write: %s %s (reg = %u%u%u '%s' val 0x%02x)\n",
         latch_val ? "setting" : "clearing",
         latch_names[latch_addr],
         (reg>>2), (reg>>1)&1, reg&1, reg_names[reg],
         data);
    
    // Allegedly, register writes can only occur when latch_val==1
    if (!latch_val) {
        return ;
    }
    
    switch (reg) {
        case 6: // Write mode
            shoe.iwm.mode = data & ~b(01111111);
            break;
        case 7: // Write data
            shoe.iwm.data = data;
            break;
    }
}

void init_iwm_state ()
{
    memset(&shoe.iwm, 0, sizeof(iwm_state_t));
}

void reset_iwm_state ()
{
    memset(&shoe.iwm, 0, sizeof(iwm_state_t));
}
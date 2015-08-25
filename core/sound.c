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
#include <string.h>
#include "../core/shoebill.h"

void init_asc_state(void)
{
    memset(&shoe.asc, 0, sizeof(shoe.asc));
    
    shoe.asc.version = 0x00; // My Mac II's ASC reports 0x00 as its version.
                             // From the ROM, it looks like other Mac II ASC masks have different version numbers
}

static uint8_t sound_dma_read_byte(uint16_t addr)
{
    apple_sound_chip_registers_t *asc = &shoe.asc;
    
    if (addr < 0x800) {
        
    }
    
    switch (addr) {
        case 0x800: // Version
            return asc->version;
            break;
            
        case 0x801: // ASC-mode
            return asc->asc_mode;
            break;
            
        case 0x802: // Channel control
            return asc->channel_ctrl;
            break;
            
        case 0x803: // FIFO control
            return asc->fifo_ctrl;
            break;
        
        case 0x804: // FIFO interrupt
            //return asc->fifo_intr;
            return 0xff;
            break;
            
        case 0x805: // Unknown (??)
            return asc->unknown1;
            break;
            
        case 0x806: // Volume control
            return asc->volume_ctrl;
            break;
        
        case 0x807: // Clock control
            return asc->clock_ctrl;
            break;
    }
    
    return 0;
}

static void sound_dma_write_byte(uint16_t addr, uint8_t data)
{
    apple_sound_chip_registers_t *asc = &shoe.asc;
    
    if (addr < 0x800) { // buf_a/b
        if (asc->asc_mode == 1) {
            // PCM mode (FIFO is append-only)
            if (asc->channel_ctrl & 2) {
                // stereo mode - each channel has its own buffer pointer
            }
            else {
                // mono mode
            }
        }
        else {
            // Off or wavetable mode (FIFO is random access)
            asc->buf[asc->left_ptr] = data;
            asc->left_ptr = (asc->left_ptr + 1) & 0x7ff;
        }
    }
    else {
        switch (addr) {
            case 0x800: // Version
                // Version is read-only
                break;
                
            case 0x801: // ASC-mode
                // ASC version 0x00 preserves the low 2 bits
                asc->asc_mode = data & 0x03;
                assert(asc->asc_mode != 3); // mode-3 produces noise, but isn't deterministic
                break;
                
            case 0x802: // Channel control
                // ASC version 0x00 preserves the high bit and low 2 bits
                asc->channel_ctrl = data & 0x83;
                break;
                
            case 0x803: // FIFO control
                // ASC version 0x00 preserves the low 2 bits
                asc->fifo_ctrl = data & 0x83;
                break;
                
            case 0x804: // FIFO interrupt
                // FIFO interrupt is read-only
                break;
                
            case 0x805: // unknown (???)
                // ASC version 0x00 preserves the high bit and low 4 bits
                // (But what does it do?)
                asc->unknown1 = data & 0x8f;
                break;
                
            case 0x806: // Volume control
                // ASC version 0x00 preserves the high 6 bits
                asc->volume_ctrl = data & 0xfc;
                break;
                
            case 0x807: // Clock control
                // ASC version 0x00 preserves the low 2 bits
                asc->clock_ctrl = data & 0x03;
                break;
        
        }
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




/*
ASC notes:

 buf_a (0x000-0x3ff) -> left
 buf_b (0x400-0x7ff) -> right
 writing to any address in buf_a or buf_b appends to the end pointer in the ring buffer (ignoring the address)
 reading from any address returns... the currently-being-read byte? Or the about-to-be-overwritten byte?
 (def one or the other)
 - Update: MODE=0: You can do regular random read/write access to buf_a/b when MODE=0.
           MODE=1: Writing anywhere within the buffer automatically appends to the end of the ring buffer
                   For stereo mode, writing between 0-0x3ff appends to buf_a, 0x400-0x7ff to buf_b
                   Reading anywhere in the buffer is strange - it seems non-deterministic.
                   Update 2: no wait, it's returning the byte at buf_b's ring pointer, does it always do that?
                   Update 3: yes, it looks like reading anywhere returns the byte at buf_b's pointer in stereo mode,
                             and buf_a's pointer in mono mode
           MODE=2: Same as MODE=0 (?)
 
 - ASC maintains two distinct ring buffer pointers for stereo mode
 
 
 
 
 0x800 - Version (?) MacII only ever seems to TST.B it
 (READ ONLY)
 
 $00 -> My Mac II's ASC's version. According to the Mac II rom, there seem to be other masks with non-0x00 versions.
 
 $b0 -> something more advanced than regular ASC (something with registers in the $fxx range)
 
 ————
 
 0x801 - Mode (?) 0 == no output, 1 == PCM, 2 == wavetable
 - Preserves low 2 bits, ignores high 6 bits
 
 0x802 - Channel selector
 Preserves mask 0x83 (high bit, low 2 bits)
 - High bit is set if “engine overflow” (according to MESS)
 - Low bits, 0x?2 -> stereo output (buf_a -> left, buf_b -> right)
 0x?0 -> Mono output (from buf_a)
 - Switching to stereo from mono produces a blip of sound in the right ear, unless the fifo has been cleared (buf_b), (or unless there was nothing in buf_b to start with)
 
 0x803 - Fifo control (cycle 0x80 to reset the internal FIFO pointer (pointers?))
 Preserves mask 0x83 (high bit, low 2 bits)
 
 0x804 - Fifo interrupt status
 (READ ONLY, doesn’t preserve any written bits (doesn’t acknowledge writes at all?))
 
 0x805 - ???
 - Preserves bits at 0x8F, (high bit, low 4 bits), doesn't seem to do anything
 
 0x806 - Volume control (high 3 bits)
 Preserves top 6 bits, ignores bottom 2 bits. (Only top 3 bits control volume)
 
 0x807 - Clock rate select (0, 2, or 3, I think)
 0x00 -> 11127hz
 0x01 -> Illegal???
 0x02 -> 11025hz
 0x03 -> 22050hz
 Writes to fifo_a/b(?) seem to block for clock rates 0, 2, and 3, but not 1. The speaker clicks like the sound chip is turning off when you set clock to 0x01.
 
 
 
 ASC has more registers past 0x807 for wave table control
 
 
 
 
 
 */
 
 

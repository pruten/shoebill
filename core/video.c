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

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <GLUT/glut.h>
#include "shoebill.h"
#include "core_api.h"

#include "video_rom/rom.c"

typedef struct __attribute__ ((__packed__)) {
    uint32_t structure_size;
    uint32_t offset_to_ram;
    
    uint16_t line_width; // the number of byten is a scan line
    uint16_t top;
    uint16_t left;
    uint16_t bottom;
    uint16_t right;
    uint16_t version;
    uint16_t pack_type;
    
    uint32_t pack_size;
    uint32_t hres;
    uint32_t vres;
    
    uint16_t pixel_type;
    uint16_t pixel_size;
    uint16_t num_comp;
    uint16_t comp_size;
    
    uint32_t plane_bytes;
} video_params_t;

/*void nubus_video_display_func (void)
{
    uint32_t myw = glutGet(GLUT_WINDOW_WIDTH);
    uint32_t myh = glutGet(GLUT_WINDOW_HEIGHT);
    uint32_t slotnum, i;
    int32_t my_window_id = glutGetWindow();
    
    for (slotnum = 0; slotnum < 16; slotnum++)
        if (shoe.slots[slotnum].glut_window_id == my_window_id)
            break;
    
    video_ctx_t *ctx = (video_ctx_t*)shoe.slots[slotnum].ctx;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, myw, 0, myh, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glColor3f(0.1, 0.1, 0.8);
    
    uint32_t gli = 0;
    
    switch (ctx->depth) {
        case 1: {
            for (i=0; i < ctx->pixels/8; i++) {
                const uint8_t byte = ctx->indexed_buf[i];
                ctx->direct_buf[i * 8 + 0] = ctx->clut[(byte >> 7) & 1];
                ctx->direct_buf[i * 8 + 1] = ctx->clut[(byte >> 6) & 1];
                ctx->direct_buf[i * 8 + 2] = ctx->clut[(byte >> 5) & 1];
                ctx->direct_buf[i * 8 + 3] = ctx->clut[(byte >> 4) & 1];
                ctx->direct_buf[i * 8 + 4] = ctx->clut[(byte >> 3) & 1];
                ctx->direct_buf[i * 8 + 5] = ctx->clut[(byte >> 2) & 1];
                ctx->direct_buf[i * 8 + 6] = ctx->clut[(byte >> 1) & 1];
                ctx->direct_buf[i * 8 + 7] = ctx->clut[(byte >> 0) & 1];
            }
            break;
        }
        case 2: {
            for (i=0; i < ctx->pixels/4; i++) {
                const uint8_t byte = ctx->indexed_buf[i];
                ctx->direct_buf[i * 4 + 0] = ctx->clut[(byte >> 6) & 3];
                ctx->direct_buf[i * 4 + 1] = ctx->clut[(byte >> 4) & 3];
                ctx->direct_buf[i * 4 + 2] = ctx->clut[(byte >> 2) & 3];
                ctx->direct_buf[i * 4 + 3] = ctx->clut[(byte >> 0) & 3];
            }
            break;
        }
        case 4: {
            for (i=0; i < ctx->pixels/2; i++) {
                const uint8_t byte = ctx->indexed_buf[i];
                ctx->direct_buf[i * 2 + 0] = ctx->clut[(byte >> 4) & 0xf];
                ctx->direct_buf[i * 2 + 1] = ctx->clut[(byte >> 0) & 0xf];
            }
            break;
        }
        case 8:
            for (i=0; i < ctx->pixels; i++)
                ctx->direct_buf[i] = ctx->clut[ctx->indexed_buf[i]];
            break;
        
        default:
            assert(!"unknown depth");
    }
    
    glViewport(0, 0, myw, myh);
    glRasterPos2i(0, myh);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    glPixelZoom(1.0, -1.0);
    
    glDrawPixels(myw, myh, GL_RGBA, GL_UNSIGNED_BYTE, ctx->direct_buf);
    
    glFlush();
}*/

uint32_t compute_nubus_crc(uint8_t *rom, uint32_t len)
{
    uint32_t i, sum = 0;
    
    for (i=0; i<len; i++) {
        uint8_t byte = rom[i];
        
        if (i==(len-9) || i==(len-10) || i==(len-11) || i==(len-12))
            byte = 0;
        
        sum = (sum << 1) + (sum >> 31) + byte;
    }
    
    rom[len-9] = sum & 0xff;
    rom[len-10] = (sum >> 8) & 0xff;
    rom[len-11] = (sum >> 16) & 0xff;
    rom[len-12] = (sum >> 24) & 0xff;
    
    return sum;
}

void nubus_video_init(void *_ctx, uint8_t slotnum,
                      uint16_t width, uint16_t height, uint16_t scanline_width,
                      double refresh_rate)
{
    shoebill_card_video_t *ctx = (shoebill_card_video_t*)_ctx;
    ctx->width = width;
    ctx->height = height;
    ctx->scanline_width = scanline_width;
    ctx->pixels = scanline_width * height;
    
    ctx->direct_buf = valloc(ctx->pixels * sizeof(video_ctx_color_t));
    ctx->indexed_buf = valloc(ctx->pixels);
    ctx->clut = malloc(256 * sizeof(video_ctx_color_t));
    ctx->rom = malloc(4096);
    
    // Set the depth and clut for B&W
    ctx->depth = 1;
    bzero(ctx->clut, 256 * 4);
    ctx->clut[0].r = 0xff;
    ctx->clut[0].g = 0xff;
    ctx->clut[0].b = 0xff;
    
    memcpy(ctx->rom, _video_rom, 4096);
    
    video_params_t *params = (video_params_t*)ctx->rom;
    
    // Load our chosen width/height into all the paramater blocks in the video rom
    params[0].line_width = htons(scanline_width / 8);
    params[0].right = htons(width);
    params[0].bottom = htons(height);
    
    params[1].line_width = htons(scanline_width / 4);
    params[1].right = htons(width);
    params[1].bottom = htons(height);
    
    params[2].line_width = htons(scanline_width / 2);
    params[2].right = htons(width);
    params[2].bottom = htons(height);
    
    params[3].line_width = htons(scanline_width);
    params[3].right = htons(width);
    params[3].bottom = htons(height);
    
    // Recompute the rom crc
    compute_nubus_crc(ctx->rom, 4096);
    
    shoe.slots[slotnum].ctx = ctx;
}

uint32_t nubus_video_read_func(const uint32_t rawaddr, const uint32_t size,
                               const uint8_t slotnum)
{
    shoebill_card_video_t *ctx = (shoebill_card_video_t*)shoe.slots[slotnum].ctx;
    const uint32_t addr = rawaddr & 0x00ffffff;
    
    // ROM and control registers
    if ((addr >> 20) == 0xf) {
        
        printf("nubus_video_read_func: got a read to 0x%08x sz=%u\n", rawaddr, size);
        
        // ROM (0xFsFFxxxx)
        if ((addr >> 16) == 0xff) {
            if (addr & 3) // Respond to lane 1 only
                return 0;
            return ctx->rom[(addr >> 2) % 4096];
        }
        
        // CLUT table (0xFsFE0xxx)
        
        // Control registers (0xFsF0xxxx)
        if ((addr >> 16) == 0xf0) {
            switch ((addr & 0x0000ffff) >> 2) {
                // case 0: // clear interrupt flag (write-only)
                    
            }
        }
        
        return 0;
    }
    
    // Else, this is video ram
    
    uint32_t i, myaddr = addr % ctx->pixels, result = 0;
    for (i=0; i<size; i++) {
        result <<= 8;
        result |= ctx->indexed_buf[(myaddr + i) % ctx->pixels];
    }
    
    return result;
}



void nubus_video_write_func(const uint32_t rawaddr, const uint32_t size,
                            const uint32_t data, const uint8_t slotnum)
{
    shoebill_card_video_t *ctx = (shoebill_card_video_t*)shoe.slots[slotnum].ctx;
    const uint32_t addr = rawaddr & 0x00ffffff;
    
    // ROM and control registers
    if ((addr >> 20) == 0xf) {
        
        printf("nubus_video_write_func: got a write to 0x%08x sz=%u data=0x%x\n", rawaddr, size, data);
        
        // ROM (0xFsFFxxxx)
        if ((addr >> 16) == 0xff)
            return ; // can't write to rom
        
        // CLUT table (0xFsFE0xxx)
        
        // Control registers (0xFsF0xxxx)
        if ((addr >> 16) == 0xf0) {
            switch ((addr & 0x0000ffff) >> 2) {
                case 0: {// Clear interrupt flag
                    shoe.via[1].rega |= (1 << (slotnum - 9));
                    break;
                }
                case 1: { // Set depth
                    const uint32_t depth = 1 << (data - 128); // FIXME: this won't work for direct mode
                    ctx->depth = depth;
                    printf("nubus_magic: set depth = %u\n", ctx->depth);
                    break;
                }
                case 2: { // Gray out screen
                    // FIXME: implement me
                    printf("nubus_magic: grey screen\n");
                    break;
                }
                case 3: { // Set clut index
                    ctx->clut_idx = data & 0xff;
                    // assert(ctx->clut_idx < 256);
                    printf("nubus_magic: set clut_idx = %u\n", ctx->clut_idx);
                    break;
                }
                case 4: { // Set red component of clut
                    ctx->clut[ctx->clut_idx].r = (data >> 8) & 0xff;
                    printf("nubus_magic: set %u.red = 0x%04x\n", ctx->clut_idx, data);
                    break;
                }
                case 5: { // Set green component of clut
                    ctx->clut[ctx->clut_idx].g = (data >> 8) & 0xff;
                    printf("nubus_magic: set %u.green = 0x%04x\n", ctx->clut_idx, data);
                    break;
                }
                case 6: { // Set blue component of clut
                    ctx->clut[ctx->clut_idx].b = (data >> 8) & 0xff;
                    printf("nubus_magic: set %u.blue = 0x%04x\n", ctx->clut_idx, data);
                    break;
                }
            }
        }
        
        return ;
    }
    
    // Else, this is video ram
   // uint32_t i, myaddr = addr % ctx->indexed_buf_len, mydata = data;
    uint32_t myaddr, mydata;
    for (myaddr = addr + size, mydata = data; addr < myaddr; ) {
        // assert(myaddr < ctx->pixels)
        ctx->indexed_buf[(--myaddr) % ctx->pixels] = mydata & 0xff;
        mydata >>= 8;
    }
}










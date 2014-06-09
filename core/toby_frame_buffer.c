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

#define ROM_SIZE 4096

uint8_t macii_video_rom[ROM_SIZE] = {
    /* 
     * Redacted!
     * But if you own a toby frame buffer card, you
     * can dump its 4kb ROM and stick it here.
     * A/UX 1, 2 and 3 seem happy with this hacky
     * implementation.
     */
};

typedef struct {
    uint8_t *buf_base;
    uint32_t buf_size;
    uint32_t clut_idx;
    uint32_t h_offset; // offset in bytes for each horizontal line
    uint8_t clut[256 * 3];
    uint8_t depth;
    uint8_t vsync;
    
    uint8_t *gldat;
} tfb_ctx_t;


void nubus_tfb_display_func (void)
{
    uint32_t myw = glutGet(GLUT_WINDOW_WIDTH);
    uint32_t myh = glutGet(GLUT_WINDOW_HEIGHT);
    uint32_t slotnum, i;
    int32_t my_window_id = glutGetWindow();
    
    for (slotnum = 0; slotnum < 16; slotnum++)
        if (shoe.slots[slotnum].glut_window_id == my_window_id)
            break;
    
    tfb_ctx_t *ctx = (tfb_ctx_t*)shoe.slots[slotnum].ctx;
    
    const uint8_t depth = ctx->depth;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, myw, 0, myh, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glColor3f(0.1, 0.1, 0.8);
    
    uint32_t gli = 0;
    for (i=0; i < (1024 * 480);) {
        uint8_t code;
        
        if (i % 1024 == 640) {
            i += (1024 - 640);
            continue;
        }
        
        assert(gli < (640 * 480));
        
        if (depth <= 1) {
            code = (ctx->buf_base[ctx->h_offset + i/8] & (0x80 >> (i % 8))) != 0;
            code = (code << 7) | 0x7f;
        }
        else if (depth == 2) {
            const uint8_t byte = ctx->buf_base[ctx->h_offset + i/4];
            const uint8_t mod = i % 4;
            code = (byte << (mod * 2)) & 0xc0;
            code |= 0x3f;
        }
        else if (depth == 4) {
            const uint8_t byte = ctx->buf_base[ctx->h_offset + i/2];
            const uint8_t mod = i % 2;
            code = (byte << (mod * 4)) & 0xf0;
            code |= 0x0f;
        }
        else if (depth == 8) {
            code = ctx->buf_base[ctx->h_offset + i];
        }
        
        ctx->gldat[gli * 3 + 0] = ctx->clut[code * 3 + 2];
        ctx->gldat[gli * 3 + 1] = ctx->clut[code * 3 + 1];
        ctx->gldat[gli * 3 + 2] = ctx->clut[code * 3 + 0];
        
        gli++;
        i++;
    }
    
    glViewport(0, 0, myw, myh);
    glRasterPos2i(0, myh);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    glPixelZoom(1.0, -1.0);
    
    glDrawPixels(myw, myh, GL_RGB, GL_UNSIGNED_BYTE, ctx->gldat);
    
    glFlush();
}

void global_mouse_func (int button, int state, int x, int y);
void global_motion_func (int x, int y);
void global_passive_motion_func (int x, int y);
void global_keyboard_up_func (unsigned char c, int x, int y);
void global_keyboard_down_func (unsigned char c, int x, int y);
void global_special_up_func (int special, int x, int y);
void global_special_down_func (int special, int x, int y);

void nubus_tfb_init(uint8_t slotnum)
{
    tfb_ctx_t *ctx = (tfb_ctx_t*)calloc(sizeof(tfb_ctx_t), 1);
    
    ctx->buf_size = 512 * 1024;
    ctx->buf_base = (uint8_t*)calloc(ctx->buf_size, 1);
    ctx->depth = 1;
    ctx->clut_idx = 786;
    ctx->gldat = valloc(480 * 640 * 3);
    
    memset(ctx->clut, 0x0, 384);
    memset(ctx->clut+384, 0xff, 384);
    memset(ctx->buf_base, 0xff, ctx->buf_size);
    
    shoe.slots[slotnum].ctx = ctx;
    
    glutInitWindowSize(640, 480);
    shoe.slots[slotnum].glut_window_id = glutCreateWindow("");
    glutDisplayFunc(nubus_tfb_display_func);
    
    glutIgnoreKeyRepeat(1);
    glutKeyboardFunc(global_keyboard_down_func);
    glutKeyboardUpFunc(global_keyboard_up_func);
    
    glutSpecialFunc(global_special_down_func);
    glutSpecialUpFunc(global_special_up_func);
    
    glutMouseFunc(global_mouse_func);
    glutMotionFunc(global_motion_func);
    glutPassiveMotionFunc(global_passive_motion_func);
    
    glShadeModel(GL_FLAT);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearColor(0.1, 1.0, 0.1, 1.0);
}

uint32_t nubus_tfb_read_func(const uint32_t rawaddr, const uint32_t size, const uint8_t slotnum)
{
    // 1111 nnnn xxxx rrrr SSSSSSSS SSSSSSxx
    // n -> slot number
    // S -> significant bits
    // x -> ignored
    // region ->
    //   0000-0111(?) - framebuffer
    //   1000
    //   1001
    //   1010
    //   1011
    //   1100
    //   1101
    //   1110
    //   1111 = ROM (repeating 4kb images in S bits)
    tfb_ctx_t *ctx = (tfb_ctx_t*)shoe.slots[slotnum].ctx;
    
    const uint32_t addr = rawaddr & 0x000fffff;
    uint32_t result = 0;
    
    switch (addr >> 16) {
        case 0x0 ... 0x7: { // frame buffer
            uint32_t i;
            for (i=0; i<size; i++)
                result = (result << 8) | ctx->buf_base[addr + i];
            break;
        }
            
        case 0xf: { // rom
            if ((addr & 3) == 0) {
                const uint32_t rom_addr = (addr >> 2) % 4096;
                result = macii_video_rom[rom_addr];
                slog("nubus_tfb_read_func: returning %x for addr %x (rom_addr=%x)\n", (uint32_t)result, shoe.physical_addr, rom_addr);
            }
            else
                result = 0;
            break;
        }
            
        case 0xd: { // vsync registers
            // 0xd0000 = ReadVSync
            // 0xd0004, d0008 = ??
            if (addr == 0xd0000) {
                ctx->vsync++;
                result = (ctx->vsync >> 7)&1;
                slog("nubus_tfb_read_func: reading vsync bit shoe.pc=0x%08x\n", shoe.orig_pc);
                // We just need to oscillate the vsync bit, to pretend we're going in and out of the blanking interval.
                // The card driver waits until the bit goes low, then waits until it goes high
                // FIXME: clean this up
                break;
            }
            else {
                slog("nubus_tfb_read_func: unknown read of *0x%08x\n", shoe.physical_addr);
                result = 0;
                break;
            }
        }
            
        default: {
            slog("nubus_tfb_read_func: unknown read of *0x%08x\n", shoe.physical_addr);
            result = 0;
            break;
        }
    }
    
    return result;
}

void nubus_tfb_write_func(const uint32_t rawaddr, const uint32_t size,
                          const uint32_t data, const uint8_t slotnum)
{
    tfb_ctx_t *ctx = (tfb_ctx_t*)shoe.slots[slotnum].ctx;
    const uint32_t addr = rawaddr & 0x000fffff;
    
    switch (addr >> 16) {
            
        case 0x0 ... 0x7: { // frame buffer
            uint32_t i;
            for (i=0; i<size; i++) {
                ctx->buf_base[addr + size - (i+1)] = (data >> (8*i)) & 0xFF;
            }
            return ;
        }
            
        case 0x8: { // other registers
            
            if (addr == 0x80000) { // some register that seems to indicate the color depth
                const uint8_t val = data & 0xff;
                if (val == 0xdf)
                    ctx->depth = 1;
                else if (val == 0xbf)
                    ctx->depth = 2;
                else if (val == 0x7f)
                    ctx->depth = 4;
                else if (val == 0xff)
                    ctx->depth = 8;
                else
                    assert(!"Can't figure out the color depth!");
                return ;
            }
            
            if (addr == 0x8000c) { // horizontal offset
                ctx->h_offset = 4 * ((~data) & 0xff);
                return ;
            }
            else {
                slog("video_nubus_set: unknown write to *0x%08x (0x%x)\n", shoe.physical_addr, data);
                return ;
            }
        }
            
        case 0x9: { // CLUT registers?
            if (addr == 0x90018) { // CLUT
                slog("clut[0x%03x (0x%02x+%u)] = 0x%02x\n", ctx->clut_idx, ctx->clut_idx/3, ctx->clut_idx%3, (uint8_t)(data & 0xff));
                ctx->clut[ctx->clut_idx] = 255 - (data & 0xff);
            
                ctx->clut_idx = (ctx->clut_idx == 0) ? 767 : ctx->clut_idx-1;
                
                return ;
            }
            
            if (addr == 0x9001c) { // CLUT register?
                const uint32_t clut_dat = data & 0xff;
                
                ctx->clut_idx = clut_dat * 3 + 2;

            }
            slog("video_nubus_set: unknown write to *0x%08x (0x%x)\n", shoe.physical_addr, data);
            return ;
        }
            
        case 0xa: { // clear interrupt for slot 0xa(?) in via2.rega (by setting the appropriate bit)
            assert((data & 0xff) == 0);
            // shoe.via[1].rega |= 0b00000010;
            shoe.via[1].rega |= (1 << (slotnum - 9));
            return ;
        }
         
        default: {
            slog("video_nubus_set: unknown write to *0x%08x (0x%x)\n", shoe.physical_addr, data);
            return ;
        }
    }
}

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
#include "shoebill.h"


#define SSW_IS_READ (1<<6)
#define SSW_DF (1<<8)

/*
 * Throw the long-format (format 0xb) bus error. Mac II ROM expects to see
 * this format for nubus bus errors.
 *
 * FIXME: nearly all the fields here are bogus
 */
void throw_long_bus_error(uint32_t addr, uint8_t is_write)
{
    if (shoe.suppress_exceptions) {
        shoe.abort = 1;
        return ;
    }
    // Save this value now, because lset() calls will reset it
    const uint8_t fc = shoe.logical_fc;
    
    //slog("throw_long_bus_error(): I'm throwing a LONG bus error (at pc = 0x%08x)!\n", shoe.orig_pc);
    
    // set supervisor bit
    set_sr_s(1);
    
    // inhibit tracing
    set_sr_t0(0);
    set_sr_t1(0);
    
    // compute vector offset
    const uint32_t vector_num = 2;
    const uint32_t vector_offset = vector_num * 4;
    
    // fetch vector handler address
    const uint32_t vector_addr = lget(shoe.vbr + vector_offset, 4);
    //slog("throw_long_bus_error(): shoe.vbr=0x%08x, vector_addr=0x%08x, offending addr=0x%08x, shoe.op=0x%04x\n", shoe.vbr, vector_addr, addr, shoe.op);
    assert(!shoe.abort); // FIXME: I can't handle another exception here
    
    const uint16_t ssw = SSW_DF | (is_write ? 0 : SSW_IS_READ) | fc;
    
    // Note: We're pushing frame format 0xB

    push_a7(0, 4); // internal registers, 18 words
    push_a7(0, 4);
    push_a7(0, 4);
    push_a7(0, 4);
    push_a7(0, 4);
    push_a7(0, 4);
    push_a7(0, 4);
    push_a7(0, 4);
    push_a7(0, 4);
    
    push_a7(0x0000, 2); // version + internal information
    
    push_a7(0, 2); // internal registers, 3 words
    push_a7(0, 4);
    
    push_a7(0, 4); // data input buffer
    
    push_a7(0, 4); // Internal register, 2 words
    
    push_a7(0, 4); // Stage B addr
    
    push_a7(0, 4); // Internal register, 4 words
    push_a7(0, 4); //
    
    push_a7(0, 4); // data output buffer
    push_a7(0, 4); // internal registers 3 and 2
    push_a7(addr, 4); // data cycle fault address
    push_a7(0, 4); // instruction pipe stage B and C
    push_a7(ssw, 2); // special status word
    push_a7(0, 2); // internal register 1
    push_a7(0xB000 | vector_offset, 2); // format word (frame format B)
    push_a7(shoe.orig_pc, 4); // PC for the current instruction
    push_a7(shoe.orig_sr, 2); // original status register
 
    shoe.pc = vector_addr;
    
    assert(!shoe.abort); // FIXME: again, can't handle an exception here
    
    shoe.abort = 1;
}

void throw_bus_error(uint32_t addr, uint8_t is_write)
{
    if (shoe.suppress_exceptions) {
        shoe.abort = 1;
        return ;
    }
    
    // Save this value now, because lset() calls will reset it
    const uint8_t fc = shoe.logical_fc;
    
    //dbg_state.running = 0;
    
    //slog("throw_bus_error(): I'm throwing a bus error (at pc = 0x%08x)!\n", shoe.orig_pc);
    
    // set supervisor bit
    set_sr_s(1);
    
    // inhibit tracing
    set_sr_t0(0);
    set_sr_t1(0);
    
    // compute vector offset
    const uint32_t vector_num = 2;
    const uint32_t vector_offset = vector_num * 4;
    
    // fetch vector handler address
    const uint32_t vector_addr = lget(shoe.vbr + vector_offset, 4);
    //slog("throw_bus_error(): shoe.vbr=0x%08x, vector_addr=0x%08x, offending addr=0x%08x, shoe.op=0x%04x, a7=0x%08x", shoe.vbr, vector_addr, addr, shoe.op, shoe.a[7]);
    assert(!shoe.abort); // FIXME: I can't handle another exception here
    
    const uint16_t ssw =
        SSW_DF | // the cpu will rerun the memory access
        (is_write ? 0 : SSW_IS_READ) | // read or write
        fc; // a/ux 3.0.1 cares about this - the address space
    
    //slog(" fc=%u\n", shoe.logical_fc);
    
    // Note: We're pushing frame format 0xA
    push_a7(0, 4); // internal registers 5 and 4
    push_a7(0, 4); // data output buffer
    push_a7(0, 4); // internal registers 3 and 2
    push_a7(addr, 4); // data cycle fault address
    push_a7(0, 4); // instruction pipe stage B and C
    push_a7(ssw, 2); // special status word
    push_a7(0, 2); // internal register 1
    push_a7(0xA000 | vector_offset, 2); // format word
    push_a7(shoe.orig_pc, 4); // PC for the current instruction
    push_a7(shoe.orig_sr, 2); // original status register
    
    shoe.pc = vector_addr;
    
    assert(!shoe.abort); // FIXME: again, can't handle an exception here
    
    shoe.abort = 1;
}


void throw_address_error()
{
    slog("throw_address_error(): I'm throwing an address error!\n");
    assert(!"address error");
    shoe.abort = 1;
}

void throw_frame_zero(uint16_t sr, uint32_t pc, uint16_t vector_num)
{
    // set supervisor bit
    set_sr_s(1);
    
    // inhibit tracing
    set_sr_t0(0);
    set_sr_t1(0);
    
    const uint32_t vector_addr = lget(shoe.vbr + vector_num*4, 4);
    assert(!shoe.abort); // FIXME: I can't handle another exception here
    
    push_a7(vector_num*4, 2);
    push_a7(pc, 4);
    push_a7(sr, 2);
    
    shoe.pc = vector_addr;
    
    shoe.abort = 1;
}

void throw_frame_two (uint16_t sr, uint32_t next_pc, uint32_t vector_num, uint32_t orig_pc)
{
    set_sr_s(1);
    
    // inhibit tracing
    set_sr_t0(0);
    set_sr_t1(0);
    
    const uint32_t vector_addr = lget(shoe.vbr + vector_num*4, 4);
    assert(!shoe.abort); // FIXME: can't handle exception here
    
    const uint16_t frame_word = 0x2000 | (vector_num*4);
    
    push_a7(orig_pc, 4);
    push_a7(frame_word, 2);
    push_a7(next_pc, 4);
    push_a7(sr, 2);
    
    shoe.pc = vector_addr;
    shoe.abort = 1;
}

void throw_illegal_instruction()
{
    //slog("throw_illegal_instruction(): I'm throwing an illegal instruction exception! (shoe.pc = 0x%08x, op=0x%04x, a7=0x%08x)\n", shoe.orig_pc, shoe.op, shoe.a[7]);
    
    /*if ((shoe.op != 0xf010) && ((shoe.op >> 12) != 0xa))
        //assert(!"illegal");
        dbg_state.running = 0; */
    
    // fetch vector number
    const uint32_t vector_num =
        ((shoe.op>>12) == 0xa) ? 10 :
        (((shoe.op>>12) == 0xf) ? 11 : 4);
    
    throw_frame_zero(shoe.orig_sr, shoe.orig_pc, vector_num);
    
    /*if ((shoe.op >> 12) == 0xa) {
        slog("Atrap: %s\n", atrap_names[shoe.op & 0xfff]?atrap_names[shoe.op & 0xfff]:"???");
    }
    else
        dbg_state.running = 0;*/
    shoe.abort = 1;
}

void throw_privilege_violation()
{
    //slog("throw_privilege_violation(): I'm throwing a privilege violation exception! (shoe.orig_pc = 0x%08x op=0x%04x\n", shoe.orig_pc, shoe.op);
    
    throw_frame_zero(shoe.orig_sr, shoe.orig_pc, 8);
    // shoe.abort = 1;
    
}

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
#include <fenv.h>
#include <float.h>
#include <math.h>
#include "../core/shoebill.h"

extern struct dis_t dis;
extern uint16_t dis_op;

#define FPU_JUMP_EMU 0
#define FPU_JUMP_DIS 1
typedef void (fpu_func_t)(uint16_t, uint16_t);

typedef struct {
    fpu_func_t *emu, *dis;
    const char *name;
} fpu_inst_t;

~newmacro(create_fpu_jump_table, 0, {
    my $names = [
        'unknown', 'fabs', 'facos', 'fadd', 'fasin', 'fatan', 'fatanh', 'fbcc', 'fcmp', 'fcos', 'fcosh',
        'fdbcc', 'fdiv', 'fetox', 'fetoxm1', 'fgetexp', 'fgetman', 'fint', 'fintrz', 'flog10', 'flog2',
        'flogn', 'flognp1', 'fmod', 'fmove', 'fmovecr', 'fmovem', 'fmovem_control', 'fmul', 'fneg', 'fnop',
        'frem', 'frestore', 'fsave', 'fscale', 'fscc', 'fsgldiv', 'fsglmul', 'fsin', 'fsincos', 'fsinh',
        'fsqrt', 'fsub', 'ftan', 'ftanh', 'ftentox', 'ftrapcc', 'ftst', 'ftwotox'
    ];
    my $fpu_enum = "typedef enum {\n";
    foreach my $n (@$names) {
        $fpu_enum .= "\tfpu_inst_$n,\n";
    }
    $fpu_enum .= "\nfpu_inst_max} fpu_inst_name_t;";
    
    my $fpu_table = "fpu_inst_t fpu_inst_table[fpu_inst_max] = {\n";
    foreach my $n (@$names) {
        $fpu_table .= "\t{NULL, NULL, \"" . $n . "\"},\n";
    }
    $fpu_table = substr($fpu_table, 0, -2);
    $fpu_table .= "\n};";
    
    my $out = "$fpu_enum \n $fpu_table \n";
    return $out;
})

~create_fpu_jump_table()

static fpu_inst_name_t fpu_decode_op(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 ttt MMMMMM);
    
    if (t) {
        switch (t) {
            case 1:
                if ((M>>3) == 1)
                    return fpu_inst_fdbcc;
                else if ((M>>3) == 7)
                    return fpu_inst_ftrapcc;
                return fpu_inst_fscc;
            case 2:
                if (M==0 && ext == 0)
                    return fpu_inst_fnop; // same as fbf.w
                // fall through
            case 3:
                return fpu_inst_fbcc;
            case 4:
                return fpu_inst_fsave;
            case 5:
                return fpu_inst_frestore;
        }
        return fpu_inst_unknown;
    }
    
    ~decompose(ext, ccc xxx yyy eeeeeee)
    
    switch (c) {
        case 0: // Reg to reg
            break;
        case 1: // unused
            return fpu_inst_unknown;
        case 2: // Memory->reg & movec
            break;
            
        case 3: // reg->mem
            return fpu_inst_fmove;
            
        case 4: // mem -> sys ctl registers
        case 5: // sys ctl registers -> mem
            return fpu_inst_fmovem_control;
            
        case 6: // movem to fp registers
        case 7: // movem to memory
            return fpu_inst_fmovem;
    }
    
    // Here c == 0b000 or 010
    
    if (M == 0 && ~bmatch(ext, 010 111 xxx xxxxxxx))
        return fpu_inst_fmovecr;
    
    if ((e>>3) == ~b(0110))
        return fpu_inst_fsincos;
    
    switch (e) {
            
        case ~b(0000000):
        case ~b(1000000):
        case ~b(1000100):
            return fpu_inst_fmove;
            
        case ~b(0000001): return fpu_inst_fint;
        case ~b(0000010): return fpu_inst_fsinh;
        case ~b(0000011): return fpu_inst_fintrz;
        case ~b(0000110): return fpu_inst_flognp1;
        case ~b(0001000): return fpu_inst_fetoxm1;
        case ~b(0001001): return fpu_inst_ftanh;
        case ~b(0001010): return fpu_inst_fatan;
        case ~b(0001100): return fpu_inst_fasin;
        case ~b(0001101): return fpu_inst_fatanh;
        case ~b(0001110): return fpu_inst_fsin;
        case ~b(0001111): return fpu_inst_ftan;
        case ~b(0010000): return fpu_inst_fetox;
        case ~b(0010001): return fpu_inst_ftwotox;
        case ~b(0010010): return fpu_inst_ftentox;
        case ~b(0010100): return fpu_inst_flogn;
        case ~b(0010101): return fpu_inst_flog10;
        case ~b(0010110): return fpu_inst_flog2;
        case ~b(0011001): return fpu_inst_fcosh;
        case ~b(0011100): return fpu_inst_facos;
        case ~b(0011101): return fpu_inst_fcos;
        case ~b(0011110): return fpu_inst_fgetexp;
        case ~b(0011111): return fpu_inst_fgetman;
        case ~b(0100001): return fpu_inst_fmod;
        case ~b(0100100): return fpu_inst_fsgldiv;
        case ~b(0100111): return fpu_inst_fsglmul;
        case ~b(0100101): return fpu_inst_frem;
        case ~b(0100110): return fpu_inst_fscale;
        case ~b(0111000): return fpu_inst_fcmp;
        case ~b(0111010): return fpu_inst_ftst;
            
        case ~b(0011000):
        case ~b(1011000):
        case ~b(1011100):
            return fpu_inst_fabs;
            
        case ~b(0100010):
        case ~b(1100010):
        case ~b(1100110):
            return fpu_inst_fadd;
            
        case ~b(0100000):
        case ~b(1100000):
        case ~b(1100100):
            return fpu_inst_fdiv;
            
            
        case ~b(0100011):
        case ~b(1100011):
        case ~b(1100111):
            return fpu_inst_fmul;
            
        case ~b(0011010):
        case ~b(1011010):
        case ~b(1011110):
            return fpu_inst_fneg;
            
        case ~b(0000100):
        case ~b(1000001):
        case ~b(1000101):
            return fpu_inst_fsqrt;
            
        case ~b(0101000):
        case ~b(1101000):
        case ~b(1101100):
            return fpu_inst_fsub;
    }
    
    return fpu_inst_unknown;
    
}

#define nextword() ({const uint16_t w=lget(shoe.pc,2); if (shoe.abort) {return;}; shoe.pc+=2; w;})
#define verify_supervisor() {if (!sr_s()) {throw_privilege_violation(); return;}}

void dis_fpu_decode ()
{
    ~decompose(dis_op, 1111 001 xxx 000000);
    
    fpu_inst_name_t name;
    uint16_t ext = 0;
    
    if (x == 4)
        name = fpu_inst_fsave;
    else if (x == 5)
        name = fpu_inst_frestore;
    else {
        ext = dis_next_word();
        name = fpu_decode_op(dis_op, ext);
    }
    
    if (fpu_inst_table[name].dis) {
        (*fpu_inst_table[name].dis)(dis_op, ext);
        return ;
    }
    
    sprintf(dis.str, "%s ???", fpu_inst_table[name].name);
}

void inst_fpu_decode ()
{
    ~decompose(shoe.op, 1111 001 xxx 000000);
    
    fpu_inst_name_t name;
    uint16_t ext = 0;
    
    if (x == 4)
        name = fpu_inst_fsave;
    else if (x == 5)
        name = fpu_inst_frestore;
    else {
        ext = nextword();
        name = fpu_decode_op(shoe.op, ext);
        // "For FPCP instructions that generate FPU exceptions,
        //  FPIAR is loaded with the address of an instruction before it's executed,
        //  unless all arithmetic exceptions are disabled."
        // My take: set fpiar for all instructions except fsave, frestore, and fmovem_control
        if (name != fpu_inst_fmovem_control)
            shoe.fpiar = shoe.orig_pc;
    }
    
    if (fpu_inst_table[name].emu) {
        (*fpu_inst_table[name].emu)(shoe.op, ext);
        return ;
    }
    
    slog("inst_fpu_decode: unhandled instruction: %s op=0x%04x ext = 0x%04x pc=0x%08x\n", fpu_inst_table[name].name, shoe.op, ext, shoe.orig_pc);
    assert(!"unknown fpu inst");
    //dbg_state.running = 0;
    
}


void dis_fsave(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 100 MMMMMM);
    ~decompose(op, 1111 001 100 mmmrrr);
    
    if (m == 4)
        sprintf(dis.str, "fsave -(a%u)", r);
    else
        sprintf(dis.str, "fsave %s", decode_ea_addr(M));
}

void inst_fsave(uint16_t op, uint16_t ext)
{
    verify_supervisor();
    
    ~decompose(op, 1111 001 100 MMMMMM);
    ~decompose(op, 1111 001 100 mmmrrr);
    
    const uint32_t size = 0x1c; // IDLE frame
    const uint16_t frame_header = 0xfd18;
    uint32_t addr;
    
    if (m == 4)
        addr = shoe.a[r] - size;
    else {
        call_ea_addr(M);
        addr = shoe.dat;
    }
    
    lset(addr, 2, frame_header);
    if (shoe.abort)
        return ;
    
    if (m == 4)
        shoe.a[r] = addr;
}

void dis_frestore(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 101 MMMMMM);
    ~decompose(op, 1111 001 101 mmmrrr);
    
    if (m == 3)
        sprintf(dis.str, "frestore (a%u)+", r);
    else
        sprintf(dis.str, "frestore %s", decode_ea_addr(M));
}

void inst_frestore(uint16_t op, uint16_t ext)
{
    verify_supervisor();
    
    ~decompose(op, 1111 001 101 MMMMMM);
    ~decompose(op, 1111 001 101 mmmrrr);
    
    uint32_t addr, size;
    
    if (m == 3)
        addr = shoe.a[r];
    else {
        call_ea_addr(M);
        addr = shoe.dat;
    }
    
    const uint16_t word = lget(addr, 2);
    if (shoe.abort) return ;
    
    // XXX: These frame sizes are different on 68881/68882/68040
    if ((word & 0xff00) == 0x0000)
        size = 4; // NULL state frame
    else if ((word & 0xff) == 0x0018)
        size = 0x1c; // IDLE state frame
    else if ((word & 0xff) == 0x00b4)
        size = 0xb8; // BUSY state frame
    else {
        slog("Frestore encountered an unknown state frame 0x%04x\n", word);
        assert("inst_frestore: bad state frame");
        return ;
    }
    
    if (m==3) {
        shoe.a[r] += size;
        slog("frestore: changing shoe.a[%u] += %u\n", r, size);
    }
}

typedef struct {
    uint8_t inexact;
    uint8_t dat[4][12];
} fmovecr_t;

fmovecr_t fmovecr_pi = {1, 0x40, 0x00, 0x00, 0x00, 0xc9, 0x0f, 0xda, 0xa2, 0x21, 0x68, 0xc2, 0x35, 0x40, 0x00, 0x00, 0x00, 0xc9, 0x0f, 0xda, 0xa2, 0x21, 0x68, 0xc2, 0x34, 0x40, 0x00, 0x00, 0x00, 0xc9, 0x0f, 0xda, 0xa2, 0x21, 0x68, 0xc2, 0x34, 0x40, 0x00, 0x00, 0x00, 0xc9, 0x0f, 0xda, 0xa2, 0x21, 0x68, 0xc2, 0x35, };
fmovecr_t fmovecr_log10_2 = {1, 0x3f, 0xfd, 0x00, 0x00, 0x9a, 0x20, 0x9a, 0x84, 0xfb, 0xcf, 0xf7, 0x98, 0x3f, 0xfd, 0x00, 0x00, 0x9a, 0x20, 0x9a, 0x84, 0xfb, 0xcf, 0xf7, 0x98, 0x3f, 0xfd, 0x00, 0x00, 0x9a, 0x20, 0x9a, 0x84, 0xfb, 0xcf, 0xf7, 0x98, 0x3f, 0xfd, 0x00, 0x00, 0x9a, 0x20, 0x9a, 0x84, 0xfb, 0xcf, 0xf7, 0x99, };
fmovecr_t fmovecr_e = {1, 0x40, 0x00, 0x00, 0x00, 0xad, 0xf8, 0x54, 0x58, 0xa2, 0xbb, 0x4a, 0x9a, 0x40, 0x00, 0x00, 0x00, 0xad, 0xf8, 0x54, 0x58, 0xa2, 0xbb, 0x4a, 0x9a, 0x40, 0x00, 0x00, 0x00, 0xad, 0xf8, 0x54, 0x58, 0xa2, 0xbb, 0x4a, 0x9a, 0x40, 0x00, 0x00, 0x00, 0xad, 0xf8, 0x54, 0x58, 0xa2, 0xbb, 0x4a, 0x9b, };
fmovecr_t fmovecr_log2_e = {1, 0x3f, 0xff, 0x00, 0x00, 0xb8, 0xaa, 0x3b, 0x29, 0x5c, 0x17, 0xf0, 0xbc, 0x3f, 0xff, 0x00, 0x00, 0xb8, 0xaa, 0x3b, 0x29, 0x5c, 0x17, 0xf0, 0xbb, 0x3f, 0xff, 0x00, 0x00, 0xb8, 0xaa, 0x3b, 0x29, 0x5c, 0x17, 0xf0, 0xbb, 0x3f, 0xff, 0x00, 0x00, 0xb8, 0xaa, 0x3b, 0x29, 0x5c, 0x17, 0xf0, 0xbc, };
fmovecr_t fmovecr_log10_e = {0, 0x3f, 0xfd, 0x00, 0x00, 0xde, 0x5b, 0xd8, 0xa9, 0x37, 0x28, 0x71, 0x95, 0x3f, 0xfd, 0x00, 0x00, 0xde, 0x5b, 0xd8, 0xa9, 0x37, 0x28, 0x71, 0x95, 0x3f, 0xfd, 0x00, 0x00, 0xde, 0x5b, 0xd8, 0xa9, 0x37, 0x28, 0x71, 0x95, 0x3f, 0xfd, 0x00, 0x00, 0xde, 0x5b, 0xd8, 0xa9, 0x37, 0x28, 0x71, 0x95, };
fmovecr_t fmovecr_zero = {0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_ln_2 = {1, 0x3f, 0xfe, 0x00, 0x00, 0xb1, 0x72, 0x17, 0xf7, 0xd1, 0xcf, 0x79, 0xac, 0x3f, 0xfe, 0x00, 0x00, 0xb1, 0x72, 0x17, 0xf7, 0xd1, 0xcf, 0x79, 0xab, 0x3f, 0xfe, 0x00, 0x00, 0xb1, 0x72, 0x17, 0xf7, 0xd1, 0xcf, 0x79, 0xab, 0x3f, 0xfe, 0x00, 0x00, 0xb1, 0x72, 0x17, 0xf7, 0xd1, 0xcf, 0x79, 0xac, };
fmovecr_t fmovecr_ln_10 = {1, 0x40, 0x00, 0x00, 0x00, 0x93, 0x5d, 0x8d, 0xdd, 0xaa, 0xa8, 0xac, 0x17, 0x40, 0x00, 0x00, 0x00, 0x93, 0x5d, 0x8d, 0xdd, 0xaa, 0xa8, 0xac, 0x16, 0x40, 0x00, 0x00, 0x00, 0x93, 0x5d, 0x8d, 0xdd, 0xaa, 0xa8, 0xac, 0x16, 0x40, 0x00, 0x00, 0x00, 0x93, 0x5d, 0x8d, 0xdd, 0xaa, 0xa8, 0xac, 0x17, };
fmovecr_t fmovecr_10_0 = {0, 0x3f, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_10_1 = {0, 0x40, 0x02, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_10_2 = {0, 0x40, 0x05, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_10_4 = {0, 0x40, 0x0c, 0x00, 0x00, 0x9c, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0c, 0x00, 0x00, 0x9c, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0c, 0x00, 0x00, 0x9c, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0c, 0x00, 0x00, 0x9c, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_10_8 = {0, 0x40, 0x19, 0x00, 0x00, 0xbe, 0xbc, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x19, 0x00, 0x00, 0xbe, 0xbc, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x19, 0x00, 0x00, 0xbe, 0xbc, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x19, 0x00, 0x00, 0xbe, 0xbc, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_10_16 = {0, 0x40, 0x34, 0x00, 0x00, 0x8e, 0x1b, 0xc9, 0xbf, 0x04, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00, 0x8e, 0x1b, 0xc9, 0xbf, 0x04, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00, 0x8e, 0x1b, 0xc9, 0xbf, 0x04, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00, 0x8e, 0x1b, 0xc9, 0xbf, 0x04, 0x00, 0x00, 0x00, };
fmovecr_t fmovecr_10_32 = {1, 0x40, 0x69, 0x00, 0x00, 0x9d, 0xc5, 0xad, 0xa8, 0x2b, 0x70, 0xb5, 0x9e, 0x40, 0x69, 0x00, 0x00, 0x9d, 0xc5, 0xad, 0xa8, 0x2b, 0x70, 0xb5, 0x9d, 0x40, 0x69, 0x00, 0x00, 0x9d, 0xc5, 0xad, 0xa8, 0x2b, 0x70, 0xb5, 0x9d, 0x40, 0x69, 0x00, 0x00, 0x9d, 0xc5, 0xad, 0xa8, 0x2b, 0x70, 0xb5, 0x9e, };
fmovecr_t fmovecr_10_64 = {1, 0x40, 0xd3, 0x00, 0x00, 0xc2, 0x78, 0x1f, 0x49, 0xff, 0xcf, 0xa6, 0xd5, 0x40, 0xd3, 0x00, 0x00, 0xc2, 0x78, 0x1f, 0x49, 0xff, 0xcf, 0xa6, 0xd5, 0x40, 0xd3, 0x00, 0x00, 0xc2, 0x78, 0x1f, 0x49, 0xff, 0xcf, 0xa6, 0xd5, 0x40, 0xd3, 0x00, 0x00, 0xc2, 0x78, 0x1f, 0x49, 0xff, 0xcf, 0xa6, 0xd6, };
fmovecr_t fmovecr_10_128 = {1, 0x41, 0xa8, 0x00, 0x00, 0x93, 0xba, 0x47, 0xc9, 0x80, 0xe9, 0x8c, 0xe0, 0x41, 0xa8, 0x00, 0x00, 0x93, 0xba, 0x47, 0xc9, 0x80, 0xe9, 0x8c, 0xdf, 0x41, 0xa8, 0x00, 0x00, 0x93, 0xba, 0x47, 0xc9, 0x80, 0xe9, 0x8c, 0xdf, 0x41, 0xa8, 0x00, 0x00, 0x93, 0xba, 0x47, 0xc9, 0x80, 0xe9, 0x8c, 0xe0, };
fmovecr_t fmovecr_10_256 = {1, 0x43, 0x51, 0x00, 0x00, 0xaa, 0x7e, 0xeb, 0xfb, 0x9d, 0xf9, 0xde, 0x8e, 0x43, 0x51, 0x00, 0x00, 0xaa, 0x7e, 0xeb, 0xfb, 0x9d, 0xf9, 0xde, 0x8d, 0x43, 0x51, 0x00, 0x00, 0xaa, 0x7e, 0xeb, 0xfb, 0x9d, 0xf9, 0xde, 0x8d, 0x43, 0x51, 0x00, 0x00, 0xaa, 0x7e, 0xeb, 0xfb, 0x9d, 0xf9, 0xde, 0x8e, };
fmovecr_t fmovecr_10_512 = {1, 0x46, 0xa3, 0x00, 0x00, 0xe3, 0x19, 0xa0, 0xae, 0xa6, 0x0e, 0x91, 0xc7, 0x46, 0xa3, 0x00, 0x00, 0xe3, 0x19, 0xa0, 0xae, 0xa6, 0x0e, 0x91, 0xc6, 0x46, 0xa3, 0x00, 0x00, 0xe3, 0x19, 0xa0, 0xae, 0xa6, 0x0e, 0x91, 0xc6, 0x46, 0xa3, 0x00, 0x00, 0xe3, 0x19, 0xa0, 0xae, 0xa6, 0x0e, 0x91, 0xc7, };
fmovecr_t fmovecr_10_1024 = {1, 0x4d, 0x48, 0x00, 0x00, 0xc9, 0x76, 0x75, 0x86, 0x81, 0x75, 0x0c, 0x17, 0x4d, 0x48, 0x00, 0x00, 0xc9, 0x76, 0x75, 0x86, 0x81, 0x75, 0x0c, 0x17, 0x4d, 0x48, 0x00, 0x00, 0xc9, 0x76, 0x75, 0x86, 0x81, 0x75, 0x0c, 0x17, 0x4d, 0x48, 0x00, 0x00, 0xc9, 0x76, 0x75, 0x86, 0x81, 0x75, 0x0c, 0x18, };
fmovecr_t fmovecr_10_2048 = {1, 0x5a, 0x92, 0x00, 0x00, 0x9e, 0x8b, 0x3b, 0x5d, 0xc5, 0x3d, 0x5d, 0xe5, 0x5a, 0x92, 0x00, 0x00, 0x9e, 0x8b, 0x3b, 0x5d, 0xc5, 0x3d, 0x5d, 0xe5, 0x5a, 0x92, 0x00, 0x00, 0x9e, 0x8b, 0x3b, 0x5d, 0xc5, 0x3d, 0x5d, 0xe5, 0x5a, 0x92, 0x00, 0x00, 0x9e, 0x8b, 0x3b, 0x5d, 0xc5, 0x3d, 0x5d, 0xe6, };
fmovecr_t fmovecr_10_4096 = {1, 0x75, 0x25, 0x00, 0x00, 0xc4, 0x60, 0x52, 0x02, 0x8a, 0x20, 0x97, 0x9b, 0x75, 0x25, 0x00, 0x00, 0xc4, 0x60, 0x52, 0x02, 0x8a, 0x20, 0x97, 0x9a, 0x75, 0x25, 0x00, 0x00, 0xc4, 0x60, 0x52, 0x02, 0x8a, 0x20, 0x97, 0x9a, 0x75, 0x25, 0x00, 0x00, 0xc4, 0x60, 0x52, 0x02, 0x8a, 0x20, 0x97, 0x9b, };

const int _fpu_round_map[4] = {FE_TONEAREST, FE_TOWARDZERO, FE_DOWNWARD, FE_UPWARD};
#define fpu_set_round() assert(0 == fesetround(_fpu_round_map[shoe.fpcr.b.mc_rnd]))
#define fpu_reset_round() assert(0 == fesetround(FE_TONEAREST))

static void fpu_set_cc(long double f)
{
    // Set condition codes
    shoe.fpsr.raw &= 0x00ffffff;
    shoe.fpsr.b.cc_nan = (0 != isnan(f));
    if (!shoe.fpsr.b.cc_nan) {
        shoe.fpsr.b.cc_n = (0 != signbit(f));
        if (isinf(f))
            shoe.fpsr.b.cc_i = 1;
        else
            shoe.fpsr.b.cc_z = (f == 0.0);
    }
}

static long double fpu_set_reg(long double f, uint8_t r)
{
    // Round the number according to the mode control byte
    {
        fpu_set_round();
        
        if (shoe.fpcr.b.mc_prec == 1) {
            const float tmp = (float)f;
            f = tmp;
        } else if (shoe.fpcr.b.mc_prec == 2) {
            const double tmp = (double)f;
            f = tmp;
        }
        
        fpu_reset_round();
    }
    
    // Store it
    shoe.fp[r] = f;
    return f;
}

// fpu_set_reg_cc() and fpu_set_ea() set the condition codes. (what else should they set?)
static void fpu_set_reg_cc(long double f, uint8_t r)
{
    fpu_set_cc(fpu_set_reg(f, r));
}

static void x87_to_motorola(long double x87, uint8_t motorola[12])
{
    uint8_t *x87_ptr = (uint8_t*)&x87;
    motorola[0] = x87_ptr[9];
    motorola[1] = x87_ptr[8];
    motorola[2] = 0;
    motorola[3] = 0;
    motorola[4] = x87_ptr[7];
    motorola[5] = x87_ptr[6];
    motorola[6] = x87_ptr[5];
    motorola[7] = x87_ptr[4];
    motorola[8] = x87_ptr[3];
    motorola[9] = x87_ptr[2];
    motorola[10] = x87_ptr[1];
    motorola[11] = x87_ptr[0];
}

static long double motorola_to_x87(const uint8_t motorola[12])
{
    uint8_t x87[12];
    
    x87[11] = 0;
    x87[10] = 0;
    x87[9] = motorola[0];
    x87[8] = motorola[1];
    
    x87[7] = motorola[4];
    x87[6] = motorola[5];
    x87[5] = motorola[6];
    x87[4] = motorola[7];
    x87[3] = motorola[8];
    x87[2] = motorola[9];
    x87[1] = motorola[10];
    x87[0] = motorola[11];
    return *(long double*)&x87[0];
}

void inst_fmovecr(uint16_t op, uint16_t ext)
{
    ~decompose(ext, 010111 rrr xxxxxxx);
    
    fmovecr_t *c = &fmovecr_zero;
    
    switch (x) {
        case 0x00: c = &fmovecr_pi; break;
        case 0x0b: c = &fmovecr_log10_2; break;
        case 0x0c: c = &fmovecr_e; break;
        case 0x0d: c = &fmovecr_log2_e; break;
        case 0x0e: c = &fmovecr_log10_e; break;
        case 0x0f: c = &fmovecr_zero; break;
        case 0x30: c = &fmovecr_ln_2; break;
        case 0x31: c = &fmovecr_ln_10; break;
        case 0x32: c = &fmovecr_10_0; break;
        case 0x33: c = &fmovecr_10_1; break;
        case 0x34: c = &fmovecr_10_2; break;
        case 0x35: c = &fmovecr_10_4; break;
        case 0x36: c = &fmovecr_10_8; break;
        case 0x37: c = &fmovecr_10_16; break;
        case 0x38: c = &fmovecr_10_32; break;
        case 0x39: c = &fmovecr_10_64; break;
        case 0x3a: c = &fmovecr_10_128; break;
        case 0x3b: c = &fmovecr_10_256; break;
        case 0x3c: c = &fmovecr_10_512; break;
        case 0x3d: c = &fmovecr_10_1024; break;
        case 0x3e: c = &fmovecr_10_2048; break;
        case 0x3f: c = &fmovecr_10_4096; break;
    }
    
    // The constants in the 68881's ROM must be in the "intermediate" format, because they're rounded differently based on fpcr.rnd
    const long double f = motorola_to_x87(c->dat[shoe.fpcr.b.mc_rnd]);
    
    fpu_set_reg_cc(f, r);
    
    slog("inst_fmovecr: set fp%u=%.30Lg\n", r, shoe.fp[r]);
    
    // fpu_finalize_exceptions();
}

void dis_fmovecr(uint16_t op, uint16_t ext)
{
    ~decompose(ext, 010111 rrr xxxxxxx);
    
    sprintf(dis.str, "fmovecr.x 0x%02x,fp%u", x, r);
}

void inst_fmovem_control(uint16_t op, uint16_t ext)
{
    ~decompose(op,  1111 001 000 mmmrrr);
    ~decompose(op,  1111 001 000 MMMMMM);
    ~decompose(ext, 10d CSI 0000000000);
    
    const uint32_t count = C + S + I;
    const uint32_t size = count * 4;
    
    if (count == 0) // I don't know if this is even a valid instruction
        return ;
    
    if ((m == 0 || m == 1) && (count > 1)) { // data and addr reg modes are valid, but only if count==1
        throw_illegal_instruction();
        return ;
    }
    
    uint32_t addr, buf[3];
    uint32_t i;
    
    if (d) { // reg to memory
        i=0;
        if (C) buf[i++] = shoe.fpcr.raw;
        if (S) buf[i++] = shoe.fpsr.raw;
        if (I) buf[i++] = shoe.fpiar;

        if (m == 0) {
            shoe.d[r] = buf[0];
            return ;
        }
        else if (m == 1) {
            shoe.a[r] = buf[0];
            return ;
        }
        else if (m == 3)
            addr = shoe.a[r];
        else if (m == 4)
            addr = shoe.a[r] - size;
        else {
            call_ea_addr(M);
            addr = shoe.dat;
        }
        
        for (i=0; i<count; i++) {
            lset(addr + (i*4), 4, buf[i]);
            if (shoe.abort)
                return ;
        }
    }
    else { // mem to reg
        if (m == 0) // data reg
            buf[0] = shoe.d[r];
        else if (m == 1) // addr reg
            buf[0] = shoe.a[r];
        else {
            if (m == 3) // post-increment
                addr = shoe.a[r];
            else if (m == 4) // pre-decrement
                addr = shoe.a[r] - size;
            else if (M == 0x3c) // immediate
                addr = shoe.pc;
            else {
                call_ea_addr(M); // call_ea_addr() should work for all other modes
                addr = shoe.dat;
            }
            
            for (i=0; i<count; i++) {
                buf[i] = lget(addr + (i*4), 4);
                if (shoe.abort)
                    return ;
            }
        }
        
        i = 0;
        
        if (C) {
            uint8_t round = shoe.fpcr.b.mc_rnd;
            shoe.fpcr.raw = buf[i++];
            uint8_t newround = shoe.fpcr.b.mc_rnd;
            
            if (round != newround) {
                slog("inst_fmovem_control: HEY: round %u -> %u\n", round, newround);
            }
        }
        if (S) shoe.fpsr.raw = buf[i++];
        if (I) shoe.fpiar = buf[i++];
        
        // Commit immediate-EA-mode PC change
        if (M == 0x3c)
            shoe.pc += size;
    }
        
    // Commit pre/post-inc/decrement
    
    if (m == 3)
        shoe.a[r] += size;
    if (m == 4)
        shoe.a[r] -= size;
    
    
    
    slog("inst_fmove_control: notice: (EA = %u/%u %08x CSI = %u%u%u)\n", m, r, (uint32_t)shoe.dat, C, S, I);
}

void dis_fmovem_control(uint16_t op, uint16_t ext)
{
    ~decompose(op,  1111 001 000 mmmrrr);
    ~decompose(op,  1111 001 000 MMMMMM);
    ~decompose(ext, 10d CSI 0000000000);
    
    if (d)
        sprintf(dis.str, "fmovem.l [%u%u%u],%s\n", C, S, I, decode_ea_addr(M)); // <- XXX: decode_ea_addr() is the wrong function to use
    else
        sprintf(dis.str, "fmovem.l %s,[%u%u%u]\n", decode_ea_addr(M), C, S, I); // <- XXX: decode_ea_addr() is the wrong function to use
}


static uint8_t fpu_test_cc(uint8_t cc)
{
    const uint8_t z = shoe.fpsr.b.cc_z;
    const uint8_t n = shoe.fpsr.b.cc_n;
    const uint8_t nan = shoe.fpsr.b.cc_nan;
    
    switch (cc & 0x0f) {
        case 0: // false
            return 0;
        case 1: // equal
            return z;
        case 2: // greater than
            return !(nan || z || n);
        case 3: // greater than or equal
            return z || !(nan || n);
        case 4: // less than
            return n && !(nan || z);
        case 5: // less than or equal
            return z || (n && !nan);
        case 6: // greater or less than
            return !(nan || z);
        case 7: // ordered
            return !nan;
        case 8: // unordered
            return nan;
        case 9: // not (greater or less than)
            return nan || z;
        case 10: // not (less than or equal)
            return nan || !(n || z);
        case 11: // not (less than)
            return nan || (z || !n);
        case 12: // not (greater than or equal)
            return nan || (n && !z);
        case 13: // not (greater than)
            return nan || z || n;
        case 14: // not equal
            return !z;
        case 15: // true
            return 1;
    }
    
    assert(0);
    return 0;
}

void inst_fbcc(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 01 s 0bcccc); // b => raise BSUN if NaN
    
    uint32_t displacement;
    if (s) {
        const uint16_t ext2 = nextword();
        displacement = (ext << 16) | ext2;
    }
    else {
        const int16_t tmp = ext;
        const int32_t tmp2 = tmp;
        displacement = tmp2;
    }
    
    if (b) {
        slog("inst_fbcc: fixme: Got a CC that wants to set BSUN, not implemented\n");
        //assert(0); // FIXME: implement BSUN, or uncomment this
    }
    
    if (fpu_test_cc(c)) {
        const uint32_t addr = shoe.orig_pc + 2 + displacement;
        shoe.pc = addr;
    }
}

const char *fpu_cc_names[32] = {
    "f", "eq", "ogt", "oge", "olt", "ole", "ogl", "or",
    "un", "ueq", "ugt", "uge", "ult", "ule", "ne", "t",
    "sf", "seq", "gt", "ge", "lt", "le", "gl", "gle",
    "ngle", "ngl", "nle", "nlt", "nge", "ngt", "sne", "st"
};

void dis_fbcc(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 01 s 0ccccc); // only the low 5 bits of cc are significant
    
    uint32_t displacement;
    if (s) {
        const uint16_t ext2 = dis_next_word();
        displacement = (ext << 16) | ext2;
    }
    else {
        const int16_t tmp = ext;
        const int32_t tmp2 = tmp;
        displacement = tmp2;
    }
    
    const uint32_t addr = dis.orig_pc + 2 + displacement;
    
    sprintf(dis.str, "fb%s.%c *0x%08x", fpu_cc_names[c], "wl"[s], addr);
}

static void reverse_order(uint8_t *buf, const uint32_t size)
{
    uint32_t i;
    for (i=0; i < (size/2); i++) {
        const uint8_t tmp = buf[i];
        buf[i] = buf[size-(1+i)];
        buf[size-(1+i)] = tmp;
    }
}

void inst_fmovem(uint16_t op, uint16_t ext)
{
    ~decompose(op,  1111 001 000 mmmrrr);
    ~decompose(op,  1111 001 000 MMMMMM);
    ~decompose(ext, 11 d ps 000 LLLLLLLL); // Static register mask
    ~decompose(ext, 11 0 00 000 0yyy0000); // Register for dynamic mode
    
    const uint8_t pre_mask = s ? shoe.d[y] : L; // pre-adjusted mask
    
    // Count the number of bits in the mask
    uint32_t count, maskcpy = pre_mask;
    for (count=0; maskcpy; maskcpy >>= 1)
        count += (maskcpy & 1);
    
    const uint32_t size = count * 12;
    
    // for predecrement mode, the mask is reversed
    uint8_t mask = 0;
    if (m == 4) {
        uint32_t i;
        for (i=0; i < 8; i++) {
            const uint8_t bit = (pre_mask << i) & 0x80;
            mask = (mask >> 1) | bit;
        }
    }
    else
        mask = pre_mask;
    
    uint32_t i, addr;
    
    // Find the EA
    if (m == 3) {
        addr = shoe.a[r];
        assert(p); // assert post-increment mask
    }
    else if (m == 4) {
        addr = shoe.a[r] - size;
        assert(!p); // assert pre-decrement mask
    }
    else {
        call_ea_addr(M);
        addr = shoe.dat;
        assert(p); // assert post-increment mask
    }
    
    slog("inst_fmovem: pre=%08x mask=%08x EA=%u/%u addr=0x%08x size=%u %s\n", pre_mask, mask, m, r, addr, size, d?"to mem":"from mem");
    
    if (d) {
        // Write those registers
        for (i=0; i<8; i++) {
            if (!(mask & (0x80 >> i)))
                continue;
            
            uint8_t buf[12];
            x87_to_motorola(shoe.fp[i], buf);
            
            slog("inst_fmovem: writing %Lf from fp%u", shoe.fp[i], i);
            uint32_t j;
            for (j=0; j<12; j++) {
                slog(" %02x", buf[j]);
                lset(addr, 1, buf[j]);
                addr++;
                if (shoe.abort)
                    return ;
            }
            slog("\n");
        }
    }
    else {
        // Read those registers
        for (i=0; i<8; i++) {
            if (!(mask & (0x80 >> i)))
                continue;
            
            uint8_t buf[12];
            uint32_t j;
            for (j=0; j<12; j++) {
                buf[j] = lget(addr, 1);
                addr++;
                if (shoe.abort)
                    return ;
            }
            shoe.fp[i] = motorola_to_x87(buf);
            
            slog("inst_fmovem: read %Lf to fp%u\n", shoe.fp[i], i);
        }
    }
    
    // Commit the write for pre/post-inc/decrement
    if (m == 3)
        shoe.a[r] += size;
    else if (m == 4)
        shoe.a[r] -= size;
    
    //slog("inst_fmovem: notice: not implemented (EA = %u/%u, mask=0x%02x)\n", m, r, mask);
    
}

void dis_fmovem(uint16_t op, uint16_t ext)
{
    ~decompose(op,  1111 001 000 mmmrrr);
    ~decompose(op,  1111 001 000 MMMMMM);
    ~decompose(ext, 11 d ps 000 LLLLLLLL); // Static register mask
    ~decompose(ext, 11 0 00 000 0yyy0000); // Register for dynamic mode
    
    sprintf(dis.str, "fmovem ???");
}

enum {
    format_L = 0,
    format_S = 1,
    format_X = 2,
    format_Ps = 3,
    format_W = 4,
    format_D = 5,
    format_B = 6,
    format_Pd = 7
} fpu_formats;
/*
 * 0 L     long word integer
 * 1 S     single precision real
 * 2 X     extended precision real
 * 3 P{#k} packed decimal real with static k factor
 * 4 W     word integer
 * 5 D     double precision real
 * 6 B     byte integer
 * 7 P{Dn} packed decimal real with dynamic k factor
 */

static void fpu_read_ea_commit(uint8_t mr, uint8_t format)
{
    const uint8_t m = mr >> 3;
    const uint8_t r = mr & 7;
    const uint8_t sizes[8] = {4, 4, 12, 12, 2, 8, 1, 12};
    
    if (m == 3)
        shoe.a[r] += sizes[format];
    else if (m == 4)
        shoe.a[r] -= sizes[format];
}

// Note: fpu_read_ea modifies shoe.pc, fpu_read_ea_commit modies shoe.a[r] for pre/post-inc/decrement
static long double fpu_read_ea(uint8_t mr, uint8_t format)
{
    const uint8_t m = mr >> 3;
    const uint8_t r = mr & 7;
    const uint8_t sizes[8] = {4, 4, 12, 12, 2, 8, 1, 12};
    
    long double data, result;
    uint32_t addr;
    
    // If mode==a-reg, or mode==data reg and the size is > 4 bytes, no dice
    if ((m == 1) ||
        ((m == 0) && (sizes[format] > 4))) {
        throw_illegal_instruction();
        return 0.0;
    }
    
    switch (m) {
        case 0: {
            if (format == format_S) {
                float tmp = shoe.d[r];
                data = tmp;
            }
            else if (format == format_B) {
                int8_t tmp = shoe.d[r];
                data = tmp;
            }
            else if (format == format_W) {
                int16_t tmp = shoe.d[r];
                data = tmp;
            }
            else if (format == format_L) {
                int32_t tmp = shoe.d[r];
                data = tmp;
            }
            
            goto got_data;
        }
            
        case 3:
            addr = shoe.a[r];
            assert(!( r==7 && sizes[format]==1));
            goto got_address;
            
        case 4:
            addr = shoe.a[r] - sizes[format];
            assert(!( r==7 && sizes[format]==1));
            goto got_address;
            
        case 7:
            if (r == 4) {
                addr = shoe.pc;
                shoe.pc += sizes[format];
                goto got_address;
            }
            
            // fall through to default:
            
        default: {
            
            shoe.mr=mr;
            ea_addr();
            if (shoe.abort)
                return 0.0;
            
            addr = (uint32_t)shoe.dat;
            goto got_address;
        }
    }
    
got_address:
    
    {
        uint8_t buf[12];
        uint8_t *ptr = &buf[sizes[format]];
        uint32_t i;
        
        slog("inst_f fpu_read_ea: format=%u, data =", format);
        for (i=0; i<sizes[format]; i++) {
            ptr--;
            *ptr = lget(addr+i, 1);
            slog(" %02x", *ptr);
            if (shoe.abort)
                return 0.0;
        }
        
        switch (format) {
            case format_B: {
                int8_t tmp = ptr[0];
                data = tmp;
                break;
            }
            case format_W: {
                int16_t tmp = *(int16_t*)ptr;
                data = tmp;
                break;
            }
            case format_L: {
                int32_t tmp = *(int32_t*)ptr;
                data = tmp;
                break;
            }
            case format_S: {
                float tmp = *(float*)ptr;
                data = tmp;
                break;
            }
            case format_D: {
                double tmp = *(double*)ptr;
                data = tmp;
                break;
            }
            case format_X: {
                reverse_order(ptr, 12);
                data = motorola_to_x87(ptr);
                break;
            }
            default: {
                assert(!"unsupported format (packed something)");
            }
        }
    }
    
got_data:
    
    fpu_set_round();
    result = data;
    fpu_reset_round();
    slog(" data=%Lf result=%Lf\n", data, result);
    return result;
}


static void fpu_write_ea(uint8_t mr, uint8_t format, long double orig_data)
{
    fpu_set_round();
    const long double data = orig_data;
    fpu_reset_round();
    
    const uint8_t m = mr >> 3;
    const uint8_t r = mr & 7;
    const uint8_t sizes[8] = {4, 4, 12, 12, 2, 8, 1, 12};
    uint8_t buf[12], *ptr = &buf[0];
    uint32_t addr, i;
    
    // If mode==a-reg, or mode==data reg and the size is > 4 bytes, no dice
    if ((m == 1) ||
        ((m == 0) && (sizes[format] > 4))) {
        throw_illegal_instruction();
        return ;
    }
    
    slog("inst_f fpu_write_ea EA=%u/%u data=%Lf format=%u\n", m, r, data, format);
    
    // Convert to the appropriate format
    
    switch (format) {
        case format_B: {
            int8_t tmp = data;
            *((int8_t*)ptr) = tmp;
            goto write_to_mem;
        }
        case format_W: {
            int16_t tmp = data;
            *((int16_t*)ptr) = tmp;
            slog("inst_f fpu_write_ea formatted=%u (0x%04x)\n", *((int16_t*)ptr), *((uint16_t*)ptr));
            break;
        }
        case format_L: {
            int32_t tmp = data;
            *((int32_t*)ptr) = tmp;
            break;
        }
        case format_S: {
            float tmp = data;
            *((float*)ptr) = tmp;
            break;
        }
        case format_D: {
            double tmp = data;
            *((double*)ptr) = tmp;
            break;
        }
        case format_X: {
            x87_to_motorola(data, ptr);
            goto write_to_mem; // ptr is already big endian
        }
        default: {
            assert(!"unsupported format (packed something)");
        }
    }

swap_order:
    reverse_order(buf, sizes[format]);
    
    
write_to_mem:
    // Lookup the EA

    switch (m) {
        case 0: {
            if (format == format_B) {
                int8_t tmp = data;
                set_d(r, tmp, 1);
            }
            else if (format == format_W) {
                int16_t tmp = data;
                set_d(r, tmp, 2);
            }
            else if (format == format_L) {
                int32_t tmp = data;
                shoe.d[r] = tmp;
            }
            else if (format == format_S) {
                float tmp = data;
                *((float*)&shoe.d[r]) = tmp;
            }
            
            goto done;
        }
        case 3: // post-increment
            addr = shoe.a[r];
            assert(!( r==7 && sizes[format]==1));
            break;
        case 4: // pre-decrement
            addr = shoe.a[r] - sizes[format];
            assert(!( r==7 && sizes[format]==1));
            break;
        default:
            call_ea_addr(mr);
            addr = (uint32_t)shoe.dat;
            break;
    }
    
    // Copy the formatted data into the EA
    slog("inst_f fpu_write_ea: addr=0x%08x\n", addr);
    for (i=0; i < sizes[format]; i++) {
        lset(addr + i, 1, buf[i]);
        if (shoe.abort)
            return ;
    }

done: // set condition codes and update pre/post-inc/decrement registers
    
    // Set condition codes
    shoe.fpsr.raw &= 0x00ffffff;
    shoe.fpsr.b.cc_nan = (0 != isnan(data));
    if (!shoe.fpsr.b.cc_nan) {
        shoe.fpsr.b.cc_n = (0 != signbit(data));
        if (isinf(data))
            shoe.fpsr.b.cc_i = 1;
        else
            shoe.fpsr.b.cc_z = (data == 0.0);
    }
    
    if (m == 3)
        shoe.a[r] += sizes[format];
    else if (m == 4)
        shoe.a[r] -= sizes[format];
}

void inst_fmove(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 000 MMMMMM);
    ~decompose(op, 1111 001 000 mmmrrr);
    ~decompose(ext, 0 E V aaa zzz KKKKKKK);
    
    const uint8_t format = a;
    
    if (K == ~b(1000100) || K == ~b(1000000)) {
        assert(!"inst_fmove: This is either a K-value, or somebody called fmove and specified the secret precision bits");
    }
    
    // E==0 => Don't use EA (reg->reg)
    // E==1 => Use EA
    // V==0 => reg->reg or mem->reg
    // V==1 => reg->mem
    
    // Load the source value into 'data'
    
    long double data;
    
    if (E && !V) { // mem -> reg
        data = fpu_read_ea(M, format);
        if (shoe.abort)
            return ;
    }
    else if (!E) { // reg -> mem
        data = shoe.fp[a];
    }
    else { // reg -> reg
        data = shoe.fp[z];
    }
    
    
    // XXX: Check for exceptions?
    
    // Write the result
    
    if (E && V) { // reg -> mem
        fpu_write_ea(M, format, data);
        if (shoe.abort)
            return ;
    }
    else if (!V) { // mem -> reg
        fpu_set_reg_cc(data, z);
        fpu_read_ea_commit(M, format);
    }
    else { // reg -> reg
        fpu_set_reg_cc(data, z);
    }
    
    const uint8_t sizes[8] = {4, 4, 12, 12, 2, 8, 1, 12};
    slog("inst_fmove src=%Lf size=%u a=%u z=%u to-mem=%u useEA=%u EA = %u/%u\n", data, sizes[format], a, z, V, E, m, r);
}

void dis_fnop(uint16_t op, uint16_t ext)
{
    sprintf(dis.str, "fnop");
}

void inst_fnop(uint16_t op, uint16_t ext)
{
}

void dis_fmove(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 000 MMMMMM);
    ~decompose(op, 1111 001 000 mmmrrr);
    ~decompose(ext, 0 E V aaa bbb KKKKKKK);
    
    // E==0 => reg to reg
    // E==1 => mem to reg / reg to mem
    // V==0 => reg->reg or mem->reg
    // V==1 => reg->mem
    
    
    sprintf(dis.str, "fmove ???");
    
}

void dis_fmath(uint16_t op, uint16_t ext)
{
    sprintf(dis.str, "fmath ??");
}

static void fpu_set_fpsr_quotient(long double a, long double b, long double result)
{
    // Thanks for being super vague on the meaning of this register, 68881 documentation
    
    const long double quo = truncl((a - result) / b);
    const uint8_t sign = signbit(quo);
    const uint64_t quo_int = fabsl(quo);
    
    shoe.fpsr.b.qu_quotient = quo_int & 0x7f;
    shoe.fpsr.b.qu_s = sign;
}

void inst_fmath(uint16_t op, uint16_t ext)
{
    ~decompose(op, 1111 001 000 MMMMMM);
    ~decompose(ext, 0 a 0 sss ddd eeeeeee);
    
    const uint8_t src_in_ea = a;
    const uint8_t source_specifier = s;
    const uint8_t dest_register = d;
    const uint8_t extension = e;
    
    uint8_t do_write_back_result = 1;
    
    long double source, dest, result;
    
    if (src_in_ea) {
        source = fpu_read_ea(M, source_specifier);
        slog("inst_fmath: source = %u/%u = %Lf", M>>3, M&7, source);
        if ((M>>3) == 3)
            slog(" a[%u]=0x%08x", M&7, shoe.a[M&7]);
        
        if (shoe.abort)
            return ;
    }
    else {
        source = shoe.fp[source_specifier];
        slog("inst_fmath: source = fp%u = %Lf", source_specifier, source);
    }
    
    dest = shoe.fp[dest_register];
    slog("  dest = fp%u = %Lf\n", dest_register, dest);
    
    switch (e) {
        case ~b(0000001): {// fpu_inst_fint
            const uint8_t dir = shoe.fpcr.b.mc_rnd;
            
            // {FE_TONEAREST, FE_TOWARDZERO, FE_DOWNWARD, FE_UPWARD};
            
            if (dir == 0)
                result = roundl(source);
            else if (dir == 1)
                result = truncl(source);
            else if (dir == 2)
                result = floorl(source);
            else
                result = ceill(source);
            
            slog("inst_fint: source = %Lf result = %Lf round=%u\n", source, result, dir);
            
            break;
        }
        case ~b(0000010): assert(!"fpu_inst_fsinh;");
        case ~b(0000011): // fpu_inst_fintrz
            slog("inst_fintrz dest = %Lf source = %Lf\n", dest, source);
            result = truncl(source);
            break;
            
        case ~b(0000110): // flognp1
            slog("inst_flognp1 dest = %Lf source = %Lf\n", dest, source);
            assert(source > -1.0);
            result = log1pl(source);
            break;
        case ~b(0001000): assert(!"fpu_inst_fetoxm1;");
        case ~b(0001001): assert(!"fpu_inst_ftanh;");
        case ~b(0001010): // fatan
            slog("inst_fatan dest = %Lf source = %Lf\n", dest, source);
            result = atanl(source);
            break;
            
        case ~b(0001100): assert(!"fpu_inst_fasin;");
        case ~b(0001101): assert(!"fpu_inst_fatanh;");
        case ~b(0001110): // fsin
            slog("inst_fsin dest = %Lf source = %Lf\n", dest, source);
            result = sinl(source);
            break;
        case ~b(0001111): assert(!"fpu_inst_ftan;");
        case ~b(0010000): // fetox
            slog("inst_fetox dest = %Lf source = %Lf\n", dest, source);
            result = expl(source);
            break;
        case ~b(0010001): assert(!"fpu_inst_ftwotox;");
        case ~b(0010010): assert(!"fpu_inst_ftentox;");
        case ~b(0010100): assert(!"fpu_inst_flogn;");
        case ~b(0010101): assert(!"fpu_inst_flog10;");
        case ~b(0010110): assert(!"fpu_inst_flog2;");
        case ~b(0011001): assert(!"fpu_inst_fcosh;");
        case ~b(0011100): assert(!"fpu_inst_facos;");
        case ~b(0011101): // fcos
            slog("fpu_inst_fcos dest = %Lf source = %Lf\n", dest, source);
            result = cosl(source);
            break;
            
        case ~b(0011110): {// fpu_inst_fgetexp
            if (!((source > 0) || (source < 0)))
                result = source; // positive or negative zero
            else if (!isfinite(source)) {
                assert(!"fgetexp: isinfl(source)");
                // returns NAN and an exception bit - not implemented for the moment
            }
            else {
                // Extract the debiased exponent from the 80-bit source
                uint8_t motorola[12];
                x87_to_motorola(source, motorola);
                int32_t exp = (motorola[0] & 0x7f) << 8;
                exp |= motorola[1];
                exp -= 16383; // debias
                result = exp;
            }
            break;
        }
        case ~b(0011111): assert(!"fpu_inst_fgetman;");
        case ~b(0100001):
            // don't forget to set fpu_set_fpsr_quotient();
            assert(!"fpu_inst_fmod;");
        
        case ~b(0100100): assert(!"fpu_inst_fsgldiv");
            
        case ~b(0100101): { // fpu_inst_frem
            assert(source != 0.0);
            result = remainderl(dest, source);
            fpu_set_fpsr_quotient(dest, source, result);
            slog("inst_frem: dest = %Lf source = %Lf quot = %u result = %Lf\n", dest, source, shoe.fpsr.b.qu_quotient, result);
            break;
        }
        case ~b(0100110): assert(!"fpu_inst_fscale;");
            
        case ~b(0111000): { // fpu_inst_fcmp
            const long double diff = dest - source;
            slog("inst_fcmp: dest = %Lf source = %Lf\n", dest, source);
            fpu_set_cc(diff);
            do_write_back_result = 0; // don't write result back to register
            break;
        }
        case ~b(0111010): { // fpu_inst_ftst
            slog("fpu_inst_ftst: dest = %Lf\n");
            fpu_set_cc(source);
            do_write_back_result = 0; // don't write result back to register
            break;
        }
            
        case ~b(1011100):
        case ~b(1011000):
            assert(!"inst_fabs: can't handle");
        case ~b(0011000):// fpu_inst_fabs
            result = fabsl(source);
            slog("inst_fabs: source=%Lf result=%Lf\n", source, result);
            break;
            
        case ~b(1100010):
        case ~b(1100110):
            assert(!"can't handle");
        case ~b(0100010): { // fpu_inst_fadd
            slog("inst_fadd dest = %Lf source = %Lf\n", dest, source);
            result = dest + source;
            break;
        }
            
        case ~b(1100000):
        case ~b(1100100):
            assert(!"can't handle");
        case ~b(0100000): { // fpu_inst_fdiv
            assert(source != 0.0);
            slog("inst_fdiv dest = %Lf source = %Lf\n", dest, source);
            
            result = dest / source;
            break;
        }
            
            
        case ~b(1100011):
        case ~b(1100111):
            assert(!"can't handle");
        case ~b(0100011): { // fpu_inst_fmul
            slog("inst_fmul dest = %Lf source = %Lf\n", dest, source);
            result = source * dest;
            break;
        }
            
        case ~b(1011010):
        case ~b(1011110):
            assert(!"fneg: can't handle");
        case ~b(0011010): // fneg
            slog("inst_fneg dest = %Lf source = %Lf\n", dest, source);
            result = -source;
            break;
            
        case ~b(1000001):
        case ~b(1000101):
            assert(!"can't handle");
        case ~b(0000100): { // fpu_inst_fsqrt
            slog("inst_fsqrt dest = %Lf source = %Lf\n", dest, source);
            result = sqrtl(source);
            break;
        }
            
        case ~b(1101000):
        case ~b(1101100):
            assert(!"can't handle");
        case ~b(0101000): { // fpu_inst_fsub
            slog("inst_fsub dest = %Lf source = %Lf\n", dest, source);
            result = dest - source;
            break;
        }
            
        case ~b(0110000) ... ~b(0110111):
            assert(!"fpu_inst_fsincos;");
        
        default:
            assert(!"inst_fmath: unknown instruction");
    }
    
    // Finalize the read, if source was in memory
    if (src_in_ea) {
        fpu_read_ea_commit(M, source_specifier);
    }
    
    // Only write back the result if necessary (fcmp doesn't do this)
    if (do_write_back_result) {
        slog("inst_fmath: result = %Lf\n", result);
        fpu_set_reg_cc(result, dest_register);
    }
}




// Setup the jump table for fpu instructions
// XXX: come up with a better, unified system for decoding instructions
void fpu_setup_jump_table()
{
    uint32_t i;
    
    
    fpu_inst_table[fpu_inst_fnop].emu = inst_fnop;
    fpu_inst_table[fpu_inst_fnop].dis = dis_fnop;
    
    fpu_inst_table[fpu_inst_fbcc].emu = inst_fbcc;
    fpu_inst_table[fpu_inst_fbcc].dis = dis_fbcc;
    
    fpu_inst_table[fpu_inst_fmovecr].emu = inst_fmovecr;
    fpu_inst_table[fpu_inst_fmovecr].dis = dis_fmovecr;
    
    fpu_inst_table[fpu_inst_fmove].emu = inst_fmove;
    fpu_inst_table[fpu_inst_fmove].dis = dis_fmove;
    
    fpu_inst_table[fpu_inst_fmovem].emu = inst_fmovem;
    fpu_inst_table[fpu_inst_fmovem].dis = dis_fmovem;
    
    fpu_inst_table[fpu_inst_fmovem_control].emu = inst_fmovem_control;
    fpu_inst_table[fpu_inst_fmovem_control].dis = dis_fmovem_control;
    
    fpu_inst_table[fpu_inst_frestore].emu = inst_frestore;
    fpu_inst_table[fpu_inst_frestore].dis = dis_frestore;

    fpu_inst_table[fpu_inst_fsave].emu = inst_fsave;
    fpu_inst_table[fpu_inst_fsave].dis = dis_fsave;
    
    const fpu_inst_name_t _fmath[] = {
        fpu_inst_fsincos,
        fpu_inst_fint,
        fpu_inst_fsinh,
        fpu_inst_fintrz,
        fpu_inst_flognp1,
        fpu_inst_fetoxm1,
        fpu_inst_ftanh,
        fpu_inst_fatan,
        fpu_inst_fatanh,
        fpu_inst_fsin,
        fpu_inst_ftan,
        fpu_inst_fetox,
        fpu_inst_ftwotox,
        fpu_inst_ftentox,
        fpu_inst_flogn,
        fpu_inst_flog10,
        fpu_inst_flog2,
        fpu_inst_fcosh,
        fpu_inst_facos,
        fpu_inst_fcos,
        fpu_inst_fgetexp,
        fpu_inst_fgetman,
        fpu_inst_fmod,
        fpu_inst_fsgldiv,
        fpu_inst_fsglmul,
        fpu_inst_frem,
        fpu_inst_fscale,
        fpu_inst_fcmp,
        fpu_inst_ftst,
        fpu_inst_fabs,
        fpu_inst_fadd,
        fpu_inst_fdiv,
        fpu_inst_fmul,
        fpu_inst_fneg,
        fpu_inst_fsqrt,
        fpu_inst_fsub
    };
    
    for (i=0; i < sizeof(_fmath) / sizeof(fpu_inst_name_t); i++) {
        fpu_inst_table[_fmath[i]].emu = inst_fmath;
        fpu_inst_table[_fmath[i]].dis = dis_fmath;
    }
}




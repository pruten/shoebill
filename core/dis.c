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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "../core/shoebill.h"
#include "../core/mc68851.h"

struct dis_t dis;
uint16_t dis_op;

//
// Helper routines
//

uint16_t dis_next_word (void)
{
    uint16_t next = dis.binary[dis.pos];
    next <<= 8;
    next |= dis.binary[dis.pos+1];
    dis.pos+=2;
    return next;
}

uint32_t dis_next_long (void)
{
    uint32_t next = dis_next_word();
    return (next << 16) | dis_next_word();
}

//
// EA decoder routines
//

void disass_ea_extended (char *buf, uint8_t mr)
{
    const uint32_t start_pc = dis.orig_pc + dis.pos;
    const uint32_t ext_a = dis_next_word();
    ~decompose(ext_a, d rrr w ss F b i zz 0 III);
    
    // d == index register type
    // r == index register 
    // w == word/long-word index size
    // s == scale factor
    // F == extension word format (0==brief, 1==full)
    // b == base register suppress
    // i == index suppress
    // z == base displacement size
    // I == index/indirect selection
    
    // slog("index_reg_type=%u, index_reg=%u, index_sz=%u, scale=%u, brief/ful=%u, supress_base=%u, suppress_index=%u, base_disp_sz=%u, I=%x\n",
        // d, r, w, s, F, b, i, z, I);
    
    if (F == 0) { // If this is the brief extension word
        char base_disp[32] = "0x0";
        char base_addr[32] = "0x0";
        char index[32] = "";
        
        // use the sign extended least significant byte in the extension word
        int8_t base_disp_addr = (int8_t)(ext_a & 0xff);
        sprintf(base_disp, "%s0x%x", (base_disp_addr<0)?"-":"", abs(base_disp_addr));
        
        // find the base address
        if (~bmatch(mr, 00xx1xxx))  // consult the MR, use the PC?
            sprintf(base_addr, "pc");
        else // otherwise, it's aX
            sprintf(base_addr, "a%u", mr&7);
        
        // find the index value
        sprintf(index, "%c%u.%c", "da"[d], r, "wl"[w]);
        if (s > 0)  // if there's an actual scale value
            sprintf(index+strlen(index), "*%u", 1<<s);
        
        // create the disassembled EA string
        // (note: I'm curious to see whether this is at all accurate. Motorola's extra-vague
        //        documentation doesn't help.)
        sprintf(buf, "%s,%s,%s", base_disp, base_addr, index);
        return ;
    }
    else { // if this is a full extension word
        char base_disp[32] = "0x0";
        char base_addr[32] = "0x0";
        char index[32] = "";
        char outer_disp[32] = "";
        
        // find the base address (a? or pc)
        if (b == 0) { // only if it isn't suppressed
            if (~bmatch(mr, 00xx1xxx)) { // consult the MR,
                sprintf(base_addr, "pc");
            } else {
                sprintf(base_addr, "a%u", mr&7);
            }
        }
        
        // Find the index
        if (i == 0) { // only if it isn't suppressed
            sprintf(index, "%c%u.%c", "da"[d], r, "wl"[w]);
            if (s > 0)  // if there's an actual scale value
                sprintf(index+strlen(index), "*%u", 1<<s);
        }
        
        // find the base displacement
        if (z > 1) { // but only if the size is > null
            uint32_t base_disp_addr;
            if (z == 2) // the base displacement is a sign-extended word
                base_disp_addr = (int16_t)dis_next_word();
            else { // else, it's a long word
                base_disp_addr = dis_next_word();
                base_disp_addr = (base_disp_addr<<16) | dis_next_word();
            }
            sprintf(base_disp, "0x%x", base_disp_addr);
        }
        
        // find the outer displacement
        switch ((i<<3)|I) { // based on I/IS
            case ~b(0010): case ~b(0110): case ~b(1010): {
                // sign-extended word-length outer displacemnt
                uint32_t addr = (int16_t)(dis_next_word());
                sprintf(outer_disp, "0x%x", addr);
                break;
            }
            case ~b(0011): case ~b(0111): case ~b(1011): {
                // long word outer displacement
                uint32_t addr = dis_next_word();
                addr = (addr << 16) | dis_next_word();
                sprintf(outer_disp, "0x%x", addr);
                break;
            }
        }
        
        // now generate a disassembled EA string
        
        // switch on the index/indirect selection
        switch ((i<<3)|I) {
            case ~b(1100) ... ~b(1111): case ~b(0100): { // invalid
                sprintf(buf, "???");
                return ;
            }
            case ~b(0001): case ~b(0010): case ~b(0011): {
                // indirect preindexed with * outer displacement
                // ([bd,An,Xn.SIZE*SCALE],od)
                if (strlen(outer_disp)) 
                    sprintf(buf, "[%s,%s,%s],%s", base_disp, base_addr, index, outer_disp);
                else
                    sprintf(buf, "[%s,%s,%s]", base_disp, base_addr, index);
                return ;
            }
            case ~b(0101): case ~b(0110): case ~b(0111): {
                // indirect postindexed with * outer displacement
                // ([bd,An],Xn.SIZE*SCALE,od)
                if (strlen(outer_disp))
                    sprintf(buf, "[%s,%s],%s,%s", base_disp, base_addr, index, outer_disp);
                else
                    sprintf(buf, "[%s,%s],%s", base_disp, base_addr, index);
                return ;
            }
            case ~b(1001): case ~b(1010): case ~b(1011): {
                // Memory indirect with * outer displacement
                // ([bd,An],od)
                if (strlen(outer_disp)) 
                    sprintf(buf, "[%s,%s],%s", base_disp, base_addr, outer_disp);
                else
                    sprintf(buf, "[%s,%s]", base_disp, base_addr);
                return ;
            }
            case ~b(0000): {
                // No memory indirect action (with index)
                sprintf(buf, "%s,%s,%s", base_disp, base_addr, index);
                return; 
            }
            case ~b(1000): {
                // No memory indirect action (without index)
                sprintf(buf, "%s,%s", base_disp, base_addr);
                return ;
            }
        }
    }
    // never get here
    sprintf(buf, "NEVER GET HERE!");
    return ;
}
            
            

char* decode_ea_rw (uint8_t mr, uint8_t sz)
{
    const uint8_t mode = (mr >> 3) & 7, reg = (mr & 7);
    char *str = dis.ea_str_internal + dis.ea_last_pos_internal;
    dis.ea_last_pos_internal = (dis.ea_last_pos_internal+256) % 1024;
    switch (mode) {
        case 0: { // Data register direct mode
            sprintf(str, "d%u", reg);
            return str;
        }
        case 1: { // address register direct mode
            sprintf(str, "a%u", reg);
            return str;
        }
        case 2: { // address register indirect mode
            sprintf(str, "(a%u)", reg);
            return str;
        }
        case 3: { // address register indirect with postincrement mode
            sprintf(str, "(a%u)+", reg);
            return str;
        }
        case 4: { // address register indirect with predecrement mode
            sprintf(str, "-(a%u)", reg);
            return str;
        }
        case 5: { // address register indirect with displacement mode
            int16_t displacement =  ((int16_t)dis_next_word());
            sprintf(str, "%s0x%x(a%u)", (displacement>=0)?"":"-", abs(displacement), reg);
            return str;
        }
        case 6: { 
            str[0] = '(';
            disass_ea_extended(str+1, mr);
            sprintf(str + strlen(str), ")");
            return str;
        }
        case 7: {
            switch (reg) {
                case 0: { // absolute short addressing mode
                    const int32_t addr = (int16_t)(dis_next_word());
                    sprintf(str, "(0x%08x)", addr);
                    return str;
                }
                case 1: { // absolute long addressing mode
                    uint32_t addr = dis_next_word();
                    addr = (addr<<16) | dis_next_word();
                    sprintf(str, "(0x%08x)", addr);
                    return str;
                }
                case 2: { // program counter indirect with displacement mode
                    const uint16_t ext = dis_next_word();
                    sprintf(str, "(0x%08x)", dis.orig_pc + 2 + ((int16_t)ext));
                    return str;
                }
                case 3: { // (program counter - evil 68020 addr modes)
                    str[0] = '(';
                    disass_ea_extended(str+1, mr);
                    sprintf(str + strlen(str), ")");
                    return str;
                }
                case 4: { // immediate data
                    const uint16_t ext = dis_next_word();
                    if (sz == 1) {
                        sprintf(str, "0x%02x", ext&0xff);
                    } else if (sz == 2) {
                        sprintf(str, "0x%04x", ext);
                    } else {
                        uint32_t i;
                        assert((sz & 1) == 0);
                        sprintf(str, "0x%04x", ext);
                        for (i=2; i<sz; i+=2) {
                            const uint16_t ext2 = dis_next_word();
                            sprintf(str + strlen(str), "%04x", ext2);
                        }
                    }
                    return str;
                }
                case 5 ... 7: {
                    sprintf(str, "???");
                    return str;
                }
            }
        }
    }
    return "error: dis: Never get here!";
}

char* decode_ea_addr (uint8_t mr)
{
    const uint8_t mode = (mr >> 3) & 7, reg = (mr & 7);
    char *str = dis.ea_str_internal + dis.ea_last_pos_internal;
    dis.ea_last_pos_internal = (dis.ea_last_pos_internal+256) % 1024;
    switch (mode) {
        case 0:
        case 1: { // Data/address register direct mode
            sprintf(str, "???");
            return str;
        }
        case 2: { // address register indirect mode
            sprintf(str, "(a%u)", reg);
            return str;
        }
        case 3: { // address register indirect with postincrement mode
            sprintf(str, "(a%u)+", reg);
            return str;
        }
        case 4: { // address register indirect with predecrement mode
            sprintf(str, "-(a%u)", reg);
            return str;
        }
        case 5: { // address register indirect with displacement mode
            int16_t displacement =  ((int16_t)dis_next_word());
            sprintf(str, "%s0x%x(a%u)", (displacement>=0)?"":"-", abs(displacement), reg);
            return str;
        }
        case 6: { 
            str[0] = '(';
            disass_ea_extended(str+1, mr);
            sprintf(str + strlen(str), ")");
            return str;
        }
        case 7: {
            switch (reg) {
                case 0: { // absolute short addressing mode
                    const int32_t addr = (int16_t)(dis_next_word());
                    sprintf(str, "(0x%08x)", addr);
                    return str;
                }
                case 1: { // absolute long addressing mode
                    uint32_t addr = dis_next_word() << 16;
                    addr |= dis_next_word();
                    sprintf(str, "(0x%08x)", addr);
                    return str;
                }
                case 2: { // program counter indirect with displacement mode
                    const uint16_t ext = dis_next_word();
                    sprintf(str, "(0x%08x)", dis.orig_pc + 2 + ((int16_t)ext));
                    return str;
                }
                case 3: { // (program counter - evil 68020 addr modes)
                    str[0] = '(';
                    disass_ea_extended(str+1, mr);
                    sprintf(str + strlen(str), ")");
                    return str;
                }
                case 4: { // immediate data
                    sprintf(str, "???");
                    return str;
                }
                case 5 ... 7: {
                    sprintf(str, "???");
                    return str;
                }
            }
        }
    }
    return "Error: decode_ea_addr: never get here!";
}

//
// Disassembler instruction implementations
//

void dis_reset() {
    sprintf(dis.str, "reset");
}

void dis_asx_reg () {
    ~decompose(dis_op, 1110 ccc d ss i 00 rrr);
    if (i) {
        sprintf(dis.str, "as%c.%c d%u,d%u", "rl"[d], "bwl"[s], c, r);
    } else {
        const uint8_t count = (c==0)?8:c;
        sprintf(dis.str, "as%c.%c %u,d%u", "rl"[d], "bwl"[s], count, r);
    }
}

void dis_asx_mem () {
    sprintf(dis.str, "asx_mem?");
}

void dis_lsx_reg () {
    ~decompose(dis_op, 1110 ccc d ss i 01 rrr);
    if (i) {
        sprintf(dis.str, "ls%c.%c d%u,d%u", "rl"[d], "bwl"[s], c, r);
    } else {
        const uint8_t count = (c==0)?8:c;
        sprintf(dis.str, "ls%c.%c %u,d%u", "rl"[d], "bwl"[s], count, r);
    }
}

void dis_lsx_mem () {
    ~decompose(dis_op, 1110 001d 11 MMMMMM);
    sprintf(dis.str, "ls%c %s", "rl"[d], decode_ea_rw(M, 2));
}

void dis_roxx_reg () {
    ~decompose(dis_op, 1110 ccc d ss i 10 rrr);
    
    if (i)
        sprintf(dis.str, "rox%c.%c d%u,d%u", "rl"[d], "bwl"[s], c, r);
    else
        sprintf(dis.str, "rox%c.%c %u,d%u", "rl"[d], "bwl"[s], c?c:8, r);
}

void dis_roxx_mem () {
    sprintf(dis.str, "roxx_mem?");
}

void dis_rox_reg () {
    ~decompose(dis_op, 1110 ccc d ss i 11 rrr);
    
    if (i)
        sprintf(dis.str, "ro%c.%c d%u,d%u", "rl"[d], "bwl"[s], c, r);
    else
        sprintf(dis.str, "ro%c.%c %u,d%u", "rl"[d], "bwl"[s], c?c:8, r);
}

void dis_rox_mem () {
    sprintf(dis.str, "rox_mem?");
}

void dis_moveq () {
    ~decompose(dis_op, 0111 rrr 0 dddddddd);
    const int32_t dat = ((int8_t)d);
    sprintf(dis.str, "moveq.l 0x%x,d%u", dat, r);
}

void dis_add () {
    ~decompose(dis_op, 1101 rrr d ss MMMMMM);
    if (d) {
        sprintf(dis.str, "add.%c d%u,%s", "bwl"[s], r, decode_ea_rw(M, 1<<s));
    } else {
        sprintf(dis.str, "add.%c %s,d%u", "bwl"[s], decode_ea_rw(M, 1<<s), r);
    }
}

void dis_adda () {
    ~decompose(dis_op, 1101 rrr s 11 MMMMMM);
    sprintf(dis.str, "adda.%c %s,a%u", "wl"[s], decode_ea_rw(M, 2+2*s), r);
}

void dis_addx () {
    ~decompose(dis_op, 1101 xxx 1 ss 00 m yyy);
    
    if (!m) {
        sprintf(dis.str, "addx.%c d%u,d%u", "bwl"[s], y, x);
    } else {
        sprintf(dis.str, "addx.%c -(a%u),-(a%u)", "bwl"[s], y, x);
    }
}

void dis_cmp () {
    ~decompose(dis_op, 1011 rrr ooo MMMMMM);
    sprintf(dis.str, "cmp.%c %s,d%u", "bwl"[o], decode_ea_rw(M, 1<<o), r);
}

void dis_cmpi () {
    ~decompose(dis_op, 0000 1100 ss MMMMMM);
    uint8_t sz = 1<<s;
    uint32_t immed;
    if (s < 2) {
        immed = chop(dis_next_word(), sz);
    } else {
        immed = dis_next_word();
        immed = (immed << 16) | dis_next_word();
    }
    sprintf(dis.str, "cmpi.%c 0x%x,%s", "bwl"[s], immed, decode_ea_rw(M, sz));
}

void dis_ori () {
    ~decompose(dis_op, 0000 0000 ss MMMMMM);
    const uint8_t sz = 1<<s;
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(dis_next_word() & 0xff);
    } else if (s==1) {
        immed = (int16_t)dis_next_word();
    } else {
        immed = dis_next_word();
        immed = (immed << 16) | dis_next_word();
    }
    sprintf(dis.str, "ori.%c 0x%x,%s", "bwl"[s], immed, decode_ea_rw(M, sz));
}

void dis_andi () {
    ~decompose(dis_op, 0000 0010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(dis_next_word() & 0xff);
    } else if (s==1) {
        immed = (int16_t)dis_next_word();
    } else {
        immed = dis_next_word();
        immed = (immed << 16) | dis_next_word();
    }
    sprintf(dis.str, "andi.%c 0x%x,%s", "bwl"[s], immed, decode_ea_rw(M, sz));
}

void dis_addi () {
    ~decompose(dis_op, 0000 0110 ss MMMMMM);
    const uint8_t sz = 1<<s;
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(dis_next_word() & 0xff);
    } else if (s==1) {
        immed = (int16_t)dis_next_word();
    } else {
        immed = dis_next_word();
        immed = (immed << 16) | dis_next_word();
    }
    sprintf(dis.str, "addi.%c 0x%x,%s", "bwl"[s], immed, decode_ea_rw(M, sz));
}

void dis_eori () {
    ~decompose(dis_op, 0000 1010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(dis_next_word() & 0xff);
    } else if (s==1) {
        immed = (int16_t)dis_next_word();
    } else {
        immed = dis_next_word();
        immed = (immed << 16) | dis_next_word();
    }
    sprintf(dis.str, "eori.%c 0x%x,%s", "bwl"[s], immed, decode_ea_rw(M, sz));
}
    
void dis_eori_to_ccr() {
    const uint16_t val = 0xff & dis_next_word();
    sprintf(dis.str, "eori.b 0x%02x,ccr", val);
}

void dis_eori_to_sr() {
    const uint16_t val = dis_next_word();
    sprintf(dis.str, "eori.w 0x%04x,sr", val);
}

void dis_movep() {
    const int16_t disp = dis_next_word();
    ~decompose(dis_op, 0000 ddd 1 s r 001 aaa);
    
    if (r) { // reg -> mem
        sprintf(dis.str, "movep.%c d%u,%s0x%x(a%u)", "wl"[s], d,
                (disp >= 0) ? "" : "-", abs(disp), a);
    }
    else { // mem -> reg
        sprintf(dis.str, "movep.%c %s0x%x(a%u),d%u", "wl"[s],
                (disp >= 0) ? "" : "-", abs(disp), a, d);
    }
    
}

void dis_bfextu() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfextu %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u", (w==0)?32:w);
    
    sprintf(dis.str + strlen(dis.str), "},d%u", r);
}

void dis_bfchg() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfchg %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u}", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u}", (w==0)?32:w);
}

void dis_bfexts() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfexts %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u", (w==0)?32:w);
    
    sprintf(dis.str + strlen(dis.str), "},d%u", r);
}

void dis_bfclr() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfclr %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u}", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u}", (w==0)?32:w);
}

void dis_bfset() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfset %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u}", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u}", (w==0)?32:w);
}

void dis_bfffo() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);

    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfffo %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u", (w==0)?32:w);
    
    sprintf(dis.str + strlen(dis.str), "},d%u", r);
}

void dis_bftst() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bftst %s{", ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u}", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u}", (w==0)?32:w);
}

void dis_bfins() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 1110 1111 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    char *ea = decode_ea_addr(M);
    if ((M>>3)==0)  // addr modes or data reg
        sprintf(ea, "d%u", M&7);
    sprintf(dis.str, "bfins d%u,%s{", r, ea);
    
    if (F) sprintf(dis.str + strlen(dis.str), "d%u:", f&7);
    else sprintf(dis.str + strlen(dis.str), "%u:", f);
    
    if (W) sprintf(dis.str + strlen(dis.str), "d%u}", w&7);
    else sprintf(dis.str + strlen(dis.str), "%u}", (w==0)?32:w);
}

void dis_btst_reg() {
    ~decompose(dis_op, 0000 rrr 100 MMMMMM);
    // sz==4 if M==000xxx
    if ((M >> 3) == 0)
        sprintf(dis.str, "btst.l d%u,%s", r, decode_ea_rw(M, 4));
    else
        sprintf(dis.str, "btst.b d%u,%s", r, decode_ea_rw(M, 1));
}

void dis_bchg_reg() {
    ~decompose(dis_op, 0000 rrr 101 MMMMMM);
    const uint32_t sz = (M >> 3) ? 4 : 1;
    sprintf(dis.str, "bchg %du,%s", r%32, decode_ea_rw(M, sz));
}

void dis_bclr_reg() {
     ~decompose(dis_op, 0000 rrr 110 MMMMMM);
    const uint32_t sz = (M >> 3) ? 4 : 1;
    sprintf(dis.str, "bclr d%u,%s", r%32, decode_ea_rw(M, sz));
}

void dis_bset_reg() {
     ~decompose(dis_op, 0000 rrr 111 MMMMMM);
    const uint32_t sz = (M >> 3) ? 4 : 1;
    sprintf(dis.str, "bset %u,%s", r%32, decode_ea_rw(M, sz));
}

void dis_subi () {
    ~decompose(dis_op, 0000 0100 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(dis_next_word() & 0xff);
    } else if (s==1) {
        immed = (int16_t)dis_next_word();
    } else {
        immed = dis_next_word();
        immed = (immed << 16) | dis_next_word();
    }
    uint8_t neg=0;
    if (((int32_t)immed) < 0) {
        immed = 0-immed;
        neg=1;
    }
    
    sprintf(dis.str, "subi.%c %s0x%x,%s", "bwl"[s], neg?"-":"", immed, decode_ea_rw(M, sz));
}

void dis_long_mul () {
    ~decompose(dis_op, 0100 1100 00 MMMMMM);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 0 LLL u s 0000000 HHH);
    // L low longword register, H high word register
    // u (signed/unsigned?) s (32 or 64 bit?)
    if (s) 
        sprintf(dis.str, "mul%c.l %s,d%u:d%u", "us"[u], decode_ea_rw(M, 4), H, L);
    else
        sprintf(dis.str, "mul%c.l %s,d%u", "us"[u], decode_ea_rw(M, 4), L);
}

void dis_long_div () {
    ~decompose(dis_op, 0100 1100 01 MMMMMM);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 0 QQQ u s 0000000 RRR);
    // Q quotient R remainder
    // u (signed/unsigned?) s (do store quotient?)
    char dest[6];
    
    sprintf(dest, "d%u", R);
    if (Q != R) 
        sprintf(dest+2, ":d%u", Q);
    sprintf(dis.str, "div%c%s.l %s,%s", "us"[u], s?"l":"", decode_ea_rw(M, 4), dest);
}

void dis_cmpm () {
    ~decompose(dis_op, 1011 xxx 1 ss 001 yyy);
    
    sprintf(dis.str, "cmpm.%c (a%u)+,(a%u)+", "bwl"[s], y, x);
}

void dis_cmpa () {
    ~decompose(dis_op, 1011 rrr o11 MMMMMM);
    sprintf(dis.str, "cmpa.%c %s,a%u", "wl"[o], decode_ea_rw(M, 2<<o), r);
}

void dis_eor () {
    ~decompose(dis_op, 1011 rrr 1ss MMMMMM);
    sprintf(dis.str, "eor.%c d%u,%s", "bwl"[s], r, decode_ea_rw(M, 1<<s));
}

void dis_addq () {
    ~decompose(dis_op, 0101 ddd 0 ss MMMMMM);
    const uint8_t dat = ((d==0)?8:d);
    sprintf(dis.str, "addq.%c %u,%s", "bwl"[s], dat, decode_ea_rw(M, 1<<s));
}

void dis_subq () {
    ~decompose(dis_op, 0101 ddd 1 ss MMMMMM);
    const uint8_t dat = ((d==0)?8:d);
    sprintf(dis.str, "subq.%c %u,%s", "bwl"[s], dat, decode_ea_rw(M, 1<<s));
}

void dis_movea () {
    ~decompose(dis_op, 00 ss rrr 001 MMMMMM);
    if (!(s >> 1))
        sprintf(dis.str, "movea.b ???");
    else 
        sprintf(dis.str, "movea.%c %s,a%u", "lw"[s&1], decode_ea_rw(M, 1<<s), r);
}

void dis_move () {
    ~decompose(dis_op, 00 ss RRR MMM oooooo);
    // (oooooo = source EA), (MMMRRR = dest EA)
    const uint8_t sizes[4] = {0, 1, 4, 2};
    const char *sourceStr = decode_ea_rw(o, sizes[s]);
    const char *destStr = decode_ea_rw((M<<3)|R, sizes[s]);
    sprintf(dis.str, "move.%c %s,%s", "?blw"[s], sourceStr, destStr);
}

void dis_move_d_to_d () {
    ~decompose(dis_op, 00 ss DDD 000 000 SSS);
    sprintf(dis.str, "move.%c d%u,d%u", "?blw"[s], S, D);
}

void dis_move_to_d () {
    ~decompose(dis_op, 00 ss rrr 000 MMMMMM);
    const uint8_t sizes[4] = {0, 1, 4, 2};
    
    sprintf(dis.str,
            "move.%c %s,d%u",
            "?blw"[s],
            decode_ea_rw(M, sizes[s]),
            r);
}

void dis_move_from_d () {
    ~decompose(dis_op, 00 ss RRR MMM 000 rrr);
    const uint8_t sizes[4] = {0, 1, 4, 2};
    
    sprintf(dis.str,
            "move.%c d%u,%s",
            "?blw"[s],
            r,
            decode_ea_rw((M<<3) | R, sizes[s]));
            
}

void dis_scc () {
    const char *condition_names[16] = {
        "t", "ra", "hi", "ls", "cc", "cs", "ne", "eq", 
        "vc", "vs", "pl", "mi", "ge", "lt", "gt", "le"
    };
    ~decompose(dis_op, 0101 cccc 11 MMMMMM);
    sprintf(dis.str, "s%s %s", condition_names[c], decode_ea_rw(M, 1));
}

void dis_dbcc () {
    // ra => relative address (?)
    const char *condition_names[16] = {
        "t", "ra", "hi", "ls", "cc", "cs", "ne", "eq", 
        "vc", "vs", "pl", "mi", "ge", "lt", "gt", "le"
    };
    ~decompose(dis_op, 0101 cccc 11001 rrr);
    uint32_t new_pc = dis.orig_pc+2;
    new_pc += (int16_t)dis_next_word();
    sprintf(dis.str, "db%s d%u,*0x%08x", condition_names[c], r, new_pc);
}

void dis_bcc () {
    uint32_t new_pc = dis.orig_pc+2;
    char size = 'b';
    ~decompose(dis_op, 0110 cccc dddddddd);
    
    // ra => relative address (?)
    // sr => subroutine (?)
    const char *condition_names[16] = {
        "ra", "sr", "hi", "ls", "cc", "cs", "ne", "eq", 
        "vc", "vs", "pl", "mi", "ge", "lt", "gt", "le"
    };
    
    // figure out the new PC
    if ((d==0) || (d==0xff)) {
        const uint16_t ext = dis_next_word();
        if (d==0xff) {
            size = 'l';
            uint32_t mylong = ((uint32_t)ext)<<16;
            mylong |= dis_next_word();
            new_pc += mylong;
        }
        else {
            size = 'w';
            new_pc += ((int16_t)ext);
        }
    }
    else {
        uint8_t tmp = d;
        new_pc += ((int8_t)d);
    }
    
    sprintf(dis.str, "b%s.%c *0x%08x", condition_names[c], size, new_pc);
}

void dis_bsr () {
    dis_bcc(); // bsr is bcc where cc==F
}

void dis_subx() {
    ~decompose(dis_op, 1001 yyy 1 ss 00 m xxx);
    if (m) 
        sprintf(dis.str, "subx.%c -(a%u),-(a%u)", "bwl"[s], x, y);
    else
        sprintf(dis.str, "subx.%c d%u,d%u", "bwl"[s], x, y);
}

void dis_btst_immediate() {
    ~decompose(dis_op, 0000 1000 11 MMMMMM);
    ~decompose(dis_op, 0000 1000 11 mmmrrr);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    if ((M >> 3) == 0)
        sprintf(dis.str, "btst.l %u,%s", b % 32, decode_ea_rw(M, 4));
    else
        sprintf(dis.str, "btst.b %u,%s", b % 8, decode_ea_rw(M, 1));
}

void dis_move_from_ccr() {
    ~decompose(dis_op, 0100 0010 11 MMMMMM);
    sprintf(dis.str, "move.w ccr,%s", decode_ea_rw(M, 2));
}

void dis_move_to_ccr() {
    ~decompose(dis_op, 0100 0100 11 MMMMMM);
    sprintf(dis.str, "move.w %s,ccr", decode_ea_rw(M, 2));
}

void dis_neg() {
    ~decompose(dis_op, 0100 0100 ss MMMMMM);
    sprintf(dis.str, "neg.%c %s", "bwl"[s], decode_ea_rw(M, 1<<s));
}

void dis_tst() {
    ~decompose(dis_op, 0100 1010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    sprintf(dis.str, "tst.%c %s", "bwl"[s], decode_ea_rw(M,sz));
}

void dis_clr() {
    ~decompose(dis_op, 0100 0010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    sprintf(dis.str, "clr.%c %s", "bwl"[s], decode_ea_rw(M,sz));
}

void dis_bclr_immediate() {
    ~decompose(dis_op, 0000 1000 10 MMMMMM);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = b % (is_data_reg ? 32 : 8);
    
    sprintf(dis.str, "bclr.%c %u,%s", (sz==1)?'b':'l', shift, decode_ea_rw(M, sz));
}

void dis_bchg_immediate() {
    ~decompose(dis_op, 0000 1000 10 MMMMMM);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = b % (is_data_reg ? 32 : 8);
    
    sprintf(dis.str, "bchg.%c %u,%s", (sz==1)?'b':'l', shift, decode_ea_rw(M, sz));
}

void dis_bset_immediate() {
    ~decompose(dis_op, 0000 1000 11 MMMMMM);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = b % (is_data_reg ? 32 : 8);
    
    sprintf(dis.str, "bset.%c %u,%s", (sz==1)?'b':'l', shift, decode_ea_rw(M, sz));
}

void dis_sub() {
    ~decompose(dis_op, 1001 rrr dss MMMMMM);
    const uint8_t sz = 1<<s;
    
    if (d) { // <ea> - Dn -> <ea>
        sprintf(dis.str, "sub.%c d%u,%s", "bwl"[s], r, decode_ea_rw(M, sz));
    } else {
        sprintf(dis.str, "sub.%c %s,d%u", "bwl"[s], decode_ea_rw(M, sz), r);
    }
}

void dis_suba () {
    ~decompose(dis_op, 1001 rrr s 11 MMMMMM);
    sprintf(dis.str, "suba.%c %s,a%u", "wl"[s], decode_ea_rw(M, 2+2*s), r);
}

void dis_move_to_sr() {
    ~decompose(dis_op, 0100 0110 11 MMMMMM);
    sprintf(dis.str, "move.w %s,sr", decode_ea_rw(M, 2));
}

void dis_move_from_sr() {
    ~decompose(dis_op, 0100 0000 11 MMMMMM);
    sprintf(dis.str, "move.w sr,%s", decode_ea_rw(M, 2));
}

void dis_negx() {
    ~decompose(dis_op, 0100 0000 ss MMMMMM);
    sprintf(dis.str, "negx.%c %s", "bwl"[s], decode_ea_rw(M, 1<<s));
}

void dis_not() {
    ~decompose(dis_op, 0100 0110 ss MMMMMM);
    sprintf(dis.str, "not.%c %s", "bwl"[s], decode_ea_rw(M, 1<<s));
}

void dis_lea () {
    ~decompose(dis_op, 0100 rrr 111 MMMMMM);
    sprintf(dis.str, "lea %s,a%u", decode_ea_addr(M), r);
}

void dis_nop () {
    sprintf(dis.str, "nop");
}

void dis_jsr () {
    ~decompose(dis_op, 0100 1110 11 MMMMMM);
    sprintf(dis.str, "jsr %s", decode_ea_addr(M));
}

void dis_chk () {
    ~decompose(dis_op, 0100 rrr 1s 0 MMMMMM);
    const uint8_t sz = s ? 2 : 4; // 3->word, 2->long word
    sprintf(dis.str, "chk.%c %s,d%u", "lw"[s], decode_ea_rw(M, sz), r);
}
void dis_jmp () {
    ~decompose(dis_op, 0100 1110 11 MMMMMM);
    sprintf(dis.str, "jmp %s", decode_ea_addr(M));
}

void dis_unknown () {
    sprintf(dis.str, "???");
}

void dis_link_word () {
    ~decompose(dis_op, 0100 1110 0101 0 rrr);
    const int16_t ext = dis_next_word();
    
    sprintf(dis.str, "link.w a%u,%d", r, ext);
}

void dis_unlk () {
    ~decompose(dis_op, 0100 1110 0101 1 rrr);
    sprintf(dis.str, "unlk a%u", r);
}

void dis_rts () {
    sprintf(dis.str, "rts");
}

void dis_cinv() {
    ~decompose(dis_op, 1111 0100 cc 0 ss rrr);
    switch (s) {
        case 1:
            sprintf(dis.str, "cinvl %cc,(a%u)", "ndib"[c], r);
            break ;
        case 2:
            sprintf(dis.str, "cinvp %cc,(a%u)", "ndib"[c], r);
            break ;
        case 3:
            sprintf(dis.str, "cinva %cc", "ndib"[c]);
    }
}

void dis_cpush() {
    ~decompose(dis_op, 1111 0100 cc 1 ss rrr);
    switch (s) {
        case 1:
            sprintf(dis.str, "cpushl %cc,(a%u)", "ndib"[c], r);
            break ;
        case 2:
            sprintf(dis.str, "cpushp %cc,(a%u)", "ndib"[c], r);
            break ;
        case 3:
            sprintf(dis.str, "cpusha %cc", "ndib"[c]);
    }
}

void dis_link_long() {
    ~decompose(dis_op, 0100 1000 0000 1 rrr); 
    uint32_t disp = dis_next_word() << 16;
    disp |= dis_next_word();
    sprintf(dis.str, "link.l a%u,%d", r, (int32_t)disp);
}

void dis_pea() {
    ~decompose(dis_op, 0100 1000 01 MMMMMM);
    sprintf(dis.str, "pea %s", decode_ea_addr(M));
}

void dis_nbcd() {
    ~decompose(dis_op, 0100 1000 00 MMMMMM);
    sprintf(dis.str, "nbcd %s", decode_ea_rw(M, 1));
}

void dis_sbcd() {
    ~decompose(dis_op, 1000 yyy 10000 r xxx);
    if (r)
        sprintf(dis.str, "sbcd d%u,d%u", x, y);
    else
        sprintf(dis.str, "sbcd -(a%u),-(a%u)", x, y);
}

void dis_pack() {
    sprintf(dis.str, "pack???");
}

void dis_unpk() {
    sprintf(dis.str, "unpk???");
}

void dis_divu() {
    ~decompose(dis_op, 1000 rrr 011 MMMMMM);
    sprintf(dis.str, "divu.w %s,d%u", decode_ea_rw(M, 2), r);
}

void dis_divs() {
    ~decompose(dis_op, 1000 rrr 111 MMMMMM);
    sprintf(dis.str, "divs.w %s,d%u", decode_ea_rw(M, 2), r);
}

void dis_bkpt() {
    ~decompose(dis_op, 0100 1000 0100 1 vvv);
    sprintf(dis.str, "bkpt %u", v);
}

void dis_swap() {
    ~decompose(dis_op, 0100 1000 0100 0 rrr);
    sprintf(dis.str, "swap d%u", r);
}

void dis_abcd() {
    ~decompose(dis_op, 1100 xxx 10000 m yyy);
    
    if (m)
        sprintf(dis.str, "abcd.b -(a%u),-(a%u)", y, x);
    else
        sprintf(dis.str, "abcd.b d%u,d%u", y, x);
}

void dis_muls() {
    ~decompose(dis_op, 1100 rrr 011 MMMMMM);
    sprintf(dis.str, "muls.w %s,d%u", decode_ea_rw(M, 2), r);
}

void dis_mulu() {
    ~decompose(dis_op, 1100 rrr 011 MMMMMM);
    sprintf(dis.str, "mulu.w %s,d%u", decode_ea_rw(M, 2), r);
}

void dis_exg() {
    ~decompose(dis_op, 1100 xxx 1 ppppp yyy);
    
    if (p == ~b(01000))  // data reg mode
        sprintf(dis.str, "exg d%u,d%u", x, y);
    else if (p == ~b(01001))  // address reg mode
        sprintf(dis.str, "exg a%u,a%u", x, y);
    else if (p == ~b(10001))  // data/addr reg mode
        sprintf(dis.str, "exg d%u,a%u", x, y);
    else
        sprintf(dis.str, "exg ???\n");
}

void dis_stop() {
    uint16_t ext = dis_next_word();
    sprintf(dis.str, "stop 0x%04x", ext);
}

void dis_rte() {
    sprintf(dis.str, "rte");
}

void dis_rtr() {
    sprintf(dis.str, "rtr");
}

void dis_rtd() {
    const int16_t ext = dis_next_word();
    sprintf(dis.str, "rtd %d", ext);
}

void dis_move_usp() {
    ~decompose(dis_op, 0100 1110 0110 d rrr);
    if (d)
        sprintf(dis.str, "move.l usp,a%u", r);
    else
        sprintf(dis.str, "move.l a%u,usp", r);
}

void dis_and() {
    ~decompose(dis_op, 1100 rrr dss MMMMMM);
    const uint8_t sz = 1<<s;
    if (d) 
        sprintf(dis.str, "and.%c d%u,%s", "bwl"[s], r, decode_ea_rw(M, sz));
    else
        sprintf(dis.str, "and.%c %s,d%u", "bwl"[s], decode_ea_rw(M, sz), r);
}

void dis_or() {
    ~decompose(dis_op, 1000 rrr dss MMMMMM);
    const uint8_t sz = 1<<s;
    if (d) {
        sprintf(dis.str, "or.%c d%u,%s", "bwl"[s], r, decode_ea_rw(M,sz));
    }
    else {
        sprintf(dis.str, "or.%c %s,d%u", "bwl"[s], decode_ea_rw(M,sz), r);
    }
}

void dis_ext() {
    ~decompose(dis_op, 0100100 ooo 000 rrr);
    switch (o) {
        case ~b(010): // sign-extend LiB to word
            sprintf(dis.str, "ext.w d%u", r);
            return ;
        case ~b(011): // ... to long
            sprintf(dis.str, "ext.l d%u", r);
            return ;
        case ~b(111): // byte to long
            sprintf(dis.str, "extb.l d%u", r);
            return ;
    }
    sprintf(dis.str, "ext???");
}

void dis_andi_to_sr() {
    const uint16_t ext = dis_next_word();
    sprintf(dis.str, "andi.w 0x%x,sr", ext);
}

void dis_andi_to_ccr() {
    const uint16_t ext = dis_next_word();
    sprintf(dis.str, "andi.b 0x%x,ccr", ext & 0xff);
}

void dis_ori_to_sr() {
    const uint16_t ext = dis_next_word();
    sprintf(dis.str, "ori.w 0x%x,sr", ext);
}

void dis_ori_to_ccr() {
    const uint16_t ext = dis_next_word();
    sprintf(dis.str, "ori.b 0x%x,sr", ext & 0xff);
}


void dis_movem() {
    ~decompose(dis_op, 0100 1D00 1s MMMMMM);
    const uint16_t mask = dis_next_word();
    
    // movem supports "control alterable" addressing modes AND predecrement mode,
    // so disassemble that predecrement edge case here:
    char tmp[8], *ea_str = &tmp[0];
    if ((M>>3) == ~b(100))
        sprintf(ea_str, "-(a%u)", M&7);
    else if ((M>>3) == ~b(011))
        sprintf(ea_str, "(a%u)+", M&7);
    else 
        ea_str = decode_ea_addr(M);
        
    // For predecrement-mode, the mask is backwards, for nobody's convenience
    const uint8_t is_mask_backwards = (((M>>3)==4) && (D==0));
    
    
    char regstr[50] = "";
    int8_t a[8], d[8], ai=0, di=0;
    uint32_t i, j;
    
    // movem traditionally has a format like "movem (a7),d2-d5/a1"
    // Also, this implementation is very, very ugly, and I am very embarrassed about it.
    
    uint8_t newmask[16];
    for (i=0; i<16; i++) 
        newmask[i] = is_mask_backwards ? (1&(mask>>(15-i))) : ((mask>>i)&1);
        
    if (mask) {
        for (i=0; i<16; i++) {
            if ((i/8) && newmask[i])  // addr reg
                sprintf(regstr + strlen(regstr), "a%u/", i%8);
            else if (newmask[i]) // data reg
                sprintf(regstr + strlen(regstr), "d%u/", i%8);
        }
        regstr[strlen(regstr)-1] = 0;
    }
    else {
        sprintf(regstr, "0");
    }
    
    if (D) { // memory - to - register
        sprintf(dis.str, "movem.%c %s,%s", "wl"[s], ea_str, regstr);
    } 
    else { // register - to - memory
        sprintf(dis.str, "movem.%c %s,%s", "wl"[s], regstr, ea_str);
    }
}

void dis_movec() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 0100 1110 0111 101x);
    ~decompose(ext, t rrr cccccccccccc);
    char *c_reg;
    switch (c) {
        case 0x002:  // CACR
            c_reg = "cacr";
            break;
        case 0x000: // SFC
            c_reg = "sfc";
            break;
        case 0x001: // DFC
            c_reg = "dfc";
            break;
        case 0x800: // USP
            c_reg = "usp";
            break;
        case 0x801: // VBR
            c_reg = "vbr";
            break;
        case 0x802: // CAAR (not supported on 68040)
            c_reg = "caar";
            break;
        case 0x803: // MSP
            c_reg = "msp";
            break;
        case 0x804: // ISP
            c_reg = "isp";
            break;
        case 0x003: // TC
            c_reg = "tc";
            break;
        case 0x004: // ITT0
            c_reg = "itt0";
            break;
        case 0x005: // ITT1
            c_reg = "itt1";
            break;
        case 0x006: // DTT0
            c_reg = "dtt0";
            break;
        case 0x007: // DTT1
            c_reg = "dtt1";
            break;
        case 0x805: // MMUSR
            c_reg = "mmusr";
            break;
        case 0x806: // URP
            c_reg = "urp";
            break;
        case 0x807: // SRP
            c_reg = "srp";
            break;
        default:
            c_reg = "???";
            break;
    }
    if (x) sprintf(dis.str, "movec.l %s%u,%s", t?"a":"d", r, c_reg);
    else sprintf(dis.str, "movec.l %s,%s%u", c_reg, t?"a":"d", r);
}

void dis_moves() {
    const uint16_t ext = dis_next_word();
    ~decompose(dis_op, 0000 1110 ss MMMMMM);
    ~decompose(ext, a rrr d 00000000000);
    const uint8_t sz = 1<<s;
    
    if (d)
        sprintf(dis.str, "moves.%c %c%u,%s", "bwl"[s], "da"[a], r, decode_ea_rw(M, sz));
    else
        sprintf(dis.str, "moves.%c %s,%c%u", "bwl"[s], decode_ea_rw(M, sz), "da"[a], r);
}

void dis_cas() {
    sprintf(dis.str, "cas ???");
}

void dis_cas2() {
    sprintf(dis.str, "cas2 ???");
}

void dis_trap() {
    ~decompose(dis_op, 0100 1110 0100 vvvv);
    sprintf(dis.str, "trap %u", v);
}

void dis_a_line() {
    sprintf(dis.str, "%s", atrap_names[dis_op&0xfff]);
}

void dis_callm () {
    sprintf(dis.str, "callm???");
}

void dis_chk2_cmp2 () {
    sprintf(dis.str, "chk2_cmp2???");
}

void dis_illegal () {
    sprintf(dis.str, "illegal");
}

void dis_move16 () {
    sprintf(dis.str, "move16???");
}

void dis_rtm () {
    ~decompose(dis_op, 0000 0110 1100 d rrr);
    sprintf(dis.str, "rtm %c%u", "da"[d], r);
}

void dis_tas () {
    ~decompose(dis_op, 1000 rrr 011 MMMMMM);
    sprintf(dis.str, "tas.b %s", decode_ea_rw(M, 1));
}

void dis_trapcc() {
    ~decompose(dis_op, 0101 cccc 11111 ooo);
    
    const char *condition_names[16] = {
        "t", "ra", "hi", "ls", "cc", "cs", "ne", "eq",
        "vc", "vs", "pl", "mi", "ge", "lt", "gt", "le"
    };
    
    uint32_t data;
    switch (o) {
        case 2:
            data = dis_next_word();
            sprintf(dis.str, "trap%s.w 0x%04x", condition_names[c], data);
            break;
        case 3:
            data = dis_next_long();
            sprintf(dis.str, "trap%s.l 0x%08x", condition_names[c], data);
            break;
        case 4:
            sprintf(dis.str, "trap%s", condition_names[c]);
            break;
    }
}

void dis_trapv() {
    sprintf(dis.str, "trapv");
}

void dis_mc68851_decode() {
    ~decompose(dis_op, 1111 000 a b c MMMMMM);
    
    // prestore or psave
    if (a) {
        if (~bmatch(dis_op, 1111 000 101 xxxxxx))
            dis_mc68851_prestore();
        else if (~bmatch(dis_op, 1111 000 100 xxxxxx))
            dis_mc68851_psave();
        else
            sprintf(dis.str, "p_unknown");
        return ;
    }
    
    // pbcc
    if (~bmatch(dis_op, 1111 000 01x 00xxxx)) {
        dis_mc68851_pbcc();
        return ;
    }
    
    const uint16_t ext = dis_next_word();
    
    // pdbcc, ptrapcc, pscc
    if (~bmatch(dis_op, 1111 000 001 xxxxxx)) {
        ~decompose(dis_op, 1111 000 001 mmm rrr);
        // These all just store a condition code in the extension word
        ~decompose(ext, 0000 0000 00 cccccc);
        
        if (m == 1)
            dis_mc68851_pdbcc(c);
        else if ((m == ~b(111)) && (r > 2))
            dis_mc68851_ptrapcc(c);
        else
            dis_mc68851_pscc(c);
        return ;
    }
    
    // dis_op must have the form (1111 000 000 xxxxxx) now
    ~decompose(ext, XXX YYY 00 0000 0000);
    switch (X) {
        case 1: // pflush, pload, pvalid
            if (Y == ~b(000))
                dis_mc68851_pload(ext);
            else if ((Y == ~b(010)) || (Y == ~b(011)))
                dis_mc68851_pvalid(ext);
            else
                dis_mc68851_pflush(ext);
            return ;
        case 2: // pmove format 1
            dis_mc68851_pmove(ext);
            return ;
        case 3: // pmove formats 2 and 3,
            dis_mc68851_pmove(ext);
            return ;
        case 4: // ptest
            dis_mc68851_ptest(ext);
            return ;
        case 5: // pflushr
            dis_mc68851_pflushr(ext);
            return ;
        default:
            sprintf(dis.str, "p_unknown");
            return ;
    }
    assert(!"never get here");
}

const char *_fcc_names[32] = {
    "f", "eq", "ogt", "oge", "olt", "ole", "ogl", "or",
    "un", "ueq", "ugt", "uge", "ult", "ule", "ne", "t",
    "sf", "seq", "gt", "ge", "lt", "le", "gl", "gle",
    "ngle", "ngl", "nle", "nlt", "nge", "ngt", "sne", "st"
};

static void dis_fmove_to_mem(uint16_t ext)
{
    const uint8_t _format_sizes[8] = {4, 4, 12, 12, 2, 8, 1, 12};
    ~decompose(dis_op, 1111 001 000 MMMMMM);
    ~decompose(ext, 011 fff sss kkkkkkk);
    
    sprintf(dis.str, "fmove.%c", "lsxpwdbp"[f]);
    if (f == 3)
        sprintf(dis.str + strlen(dis.str), "{#%u}", k);
    else
        sprintf(dis.str + strlen(dis.str), "{d%u}", k >> 4);
    
    sprintf(dis.str + strlen(dis.str), " fp%u,%s", s,
            decode_ea_rw(M, _format_sizes[f]));
}

static void dis_fmovem_control(uint16_t ext)
{
    ~decompose(dis_op, 1111 001 000 MMMMMM);
    ~decompose(ext, 10 d CSI 0000 000000);
    
    sprintf(dis.str, "fmovem.l ");
    const uint16_t count = C + S + I;
    if (count == 0)
        sprintf(dis.str + strlen(dis.str), "0,");
    
    if (C)
        sprintf(dis.str + strlen(dis.str), "fpcr%s", (count > 1)?"&":",");
    
    if (S)
        sprintf(dis.str + strlen(dis.str), "fpsr%s", ((S+I) > 1)?"&":",");
    
    if (I)
        sprintf(dis.str + strlen(dis.str), "fpiar,");
    
    sprintf(dis.str + strlen(dis.str), "%s", decode_ea_rw(M, count * 4));
}

static void dis_fmovem(uint16_t ext)
{
    ~decompose(dis_op, 1111 001 000 mmmrrr);
    ~decompose(dis_op, 1111 001 000 MMMMMM);
    ~decompose(ext, 11 d ps 000 LLLLLLLL); // Static register mask
    ~decompose(ext, 11 0 00 000 0yyy0000); // Register for dynamic mode
    
    if (s) { // if dynamic mode
        if (d) // mem -> reg
            sprintf(dis.str, "fmovem.x %s,a%u", decode_ea_rw(M, 4), y);
        else
            sprintf(dis.str, "fmovem.x a%u,%s", y, decode_ea_rw(M, 4));
        return;
    }
    
    uint32_t i, count=0;
    char list[64] = "";
    uint8_t oldmask = L, mask = L;
    
    // for predecrement mode, the mask is reversed
    if (m == 4) {
        for (i=0; i<8; i++) {
            mask <<= 1;
            mask |= (oldmask & 1);
            oldmask >>= 1;
            count++;
        }
    }
    
    for (i=0; i<8; i++) {
        if (mask & 0x80)
            sprintf(list + strlen(list), "fp%u.", i);
        mask <<= 1;
    }
    
    if (d) // mem -> reg
        sprintf(dis.str, "fmovem.x %s,%s", decode_ea_rw(M, 4), list);
    else // reg -> mem
        sprintf(dis.str, "fmovem.x %s,%s", list, decode_ea_rw(M, 4));
}

void dis_fscc () {
    ~decompose(dis_op, 1111 001 001 MMMMMM);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 0000 0000 00 0ccccc);
    
    sprintf(dis.str, "fs%s.b %s", _fcc_names[c], decode_ea_rw(M, 1));
}

void dis_fbcc () {
    ~decompose(dis_op, 1111 001 01s 0ccccc);
    
    uint32_t new_pc = dis.orig_pc + 2;
    if (s == 0) {
        const int16_t tmp = dis_next_word();
        new_pc += tmp;
    }
    else
        new_pc += dis_next_long();
    
    sprintf(dis.str, "fb%s.%c *0x%08x", _fcc_names[c], "wl"[s], new_pc);
}

void dis_fsave () {
    ~decompose(dis_op, 1111 001 100 MMMMMM);
    sprintf(dis.str, "fsave %s", decode_ea_addr(M));
}

void dis_frestore () {
    ~decompose(dis_op, 1111 001 101 MMMMMM);
    sprintf(dis.str, "frestore %s", decode_ea_addr(M));
}

void dis_fpu_other () {
    ~decompose(dis_op, 1111 001 000 MMMMMM);
    
    const uint16_t ext = dis_next_word();
    ~decompose(ext, ccc xxx yyy eeeeeee);
    
    switch (c) {
        case 0: // Reg to reg
            dis_fmath(dis_op, ext, dis.str);
            return;
            
        case 1: // unused
            sprintf(dis.str, "f???");
            return;
            
        case 2: // Memory->reg & movec
            dis_fmath(dis_op, ext, dis.str);
            return;
            
        case 3: // reg->mem
            dis_fmove_to_mem(ext);
            return;
            
        case 4: // mem -> sys ctl registers
        case 5: // sys ctl registers -> mem
            dis_fmovem_control(ext);
            return;
            
        case 6: // movem to fp registers
        case 7: // movem to memory
            dis_fmovem(ext);
            return;
    }
}

void dis_fdbcc () {
    ~decompose(dis_op, 1111 001 001 001 rrr);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 0000 0000 00 0ccccc);
    const int16_t disp = dis_next_word();
    
    // FIXME: 68kprm is helpfully unclear about which address
    //        to add the displacement. Based on cpDBcc, dbcc, bcc, and fbcc,
    //        I'm guessing it starts at the address *of* the displacement
    const uint32_t newpc = dis.orig_pc + 4 + disp;
    
    sprintf(dis.str, "fdb%s d%u,0x%08x", _fcc_names[c], r, newpc);
}

void dis_ftrapcc () {
    ~decompose(dis_op, 1111 001 001 111 ooo);
    const uint16_t ext = dis_next_word();
    ~decompose(ext, 0000 0000 00 0ccccc);
    uint32_t data;
    
    switch (o) {
        case 2:
            data = dis_next_word();
            sprintf(dis.str, "ftrap%s.w 0x%04x", _fcc_names[c], data);
            break;
        case 3:
            data = dis_next_long();
            sprintf(dis.str, "ftrap%s.l 0x%08x", _fcc_names[c], data);
            break;
        case 4:
            sprintf(dis.str, "ftrap%s", _fcc_names[c]);
            break;
        default:
            sprintf(dis.str, "ftrap????");
            break;
    }
}


void dis_fnop () {
    const uint16_t ext = dis_next_word();
    sprintf(dis.str, "fnop");
}

#include "dis_decoder_guts.c"

/*
 * Disassembler entry point
 */
void disassemble_inst(uint8_t binary[32], uint32_t orig_pc, char *str, uint32_t *instlen)
{
    dis.pos = 0;
    memcpy(dis.binary, binary, 32);
    dis.ea_last_pos_internal = 0; // start the ea decoder's ring buffer at 0
    dis.orig_pc = orig_pc;
    dis.str = str;
    
    dis_op = dis_next_word(); // dis_decode() can only see dis_op
    
    dis_instruction_to_pointer[dis_opcode_map[dis_op]]();
    
    if (instlen) *instlen = dis.pos;
}


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
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "../core/shoebill.h"
#include "../core/mc68851.h"

#define verify_supervisor() {if (!sr_s()) {throw_privilege_violation(); return;}}

extern struct dis_t dis;
extern uint16_t dis_op;

void inst_mc68851_prestore() {
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}

void inst_mc68851_psave(){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}

void inst_mc68851_pbcc(){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}


void inst_mc68851_pdbcc(uint16_t cond){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}

void inst_mc68851_ptrapcc(uint16_t cond){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}

void inst_mc68851_pscc(uint16_t cond){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}


void inst_mc68851_pload(uint16_t ext){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}

void inst_mc68851_pvalid(uint16_t ext){
    slog("%s: Error, not implemented!\n", __func__);
    assert(!"blowup");
}

void inst_mc68851_pflushr(uint16_t ext){
    verify_supervisor();
    slog("pflushr!");
    // Just nuke the entire cache
    memset(shoe.pmmu_cache[0].valid_map, 0, PMMU_CACHE_SIZE/8);
    memset(shoe.pmmu_cache[1].valid_map, 0, PMMU_CACHE_SIZE/8);
    
    /* Invalidate the pc cache */
    invalidate_pccache();
}

void inst_mc68851_pflush(uint16_t ext){
    verify_supervisor();
    slog("pflush!");
    memset(shoe.pmmu_cache[0].valid_map, 0, PMMU_CACHE_SIZE/8);
    memset(shoe.pmmu_cache[1].valid_map, 0, PMMU_CACHE_SIZE/8);
    // slog("%s: Error, not implemented!\n", __func__);
    
    /* Invalidate the pc cache */
    invalidate_pccache();
}

void inst_mc68851_pmove(uint16_t ext){

    verify_supervisor();

    ~decompose(shoe.op, 1111 000 000 MMMMMM);
    ~decompose(ext, fff ppp w 0000 nnn 00);
    
    /* 
     * For simplicity, just blow away pccache whenever
     * the PMMU state changes at all
     */
    if (!w)
        invalidate_pccache();
    
    
    // instruction format #1
    if (f == ~b(010)) {
        const uint8_t sizes[8] = {4, 8, 8, 8, 1, 1, 1, 2};
        
        // if accessing d or a reg, and sz==8 bytes, that's bogus
        if (((M>>3) < 2) && (sizes[p]==8)) {
            throw_illegal_instruction();
            return ;
        }
        
        if (!w) { // if we're reading from EA
            call_ea_read(M, sizes[p]);
            call_ea_read_commit(M, sizes[p]);
        }
        
        switch (p) {
            case 0: // tc
                if (!w) {
                    shoe.tc = shoe.dat & 0x83FFFFFF;
                    shoe.tc_is = (shoe.tc >> 16) & 0xf;
                    shoe.tc_ps = (shoe.tc >> 20) & 0xf;
                    shoe.tc_pagesize = 1 << shoe.tc_ps;
                    shoe.tc_pagemask = shoe.tc_pagesize - 1;
                    shoe.tc_is_plus_ps = shoe.tc_is + shoe.tc_ps;
                    shoe.tc_enable = (shoe.tc >> 31) & 1;
                    shoe.tc_sre = (shoe.tc >> 25) & 1;
                }
                else {
                    shoe.dat = shoe.tc;
                    //if (!tc_fcl()) assert(!"pmove->tc: function codes not supported\n");
                    
                }
                break;
            case 1: // drp
                if (!w) shoe.drp = shoe.dat & 0xffff0203ffffffff;
                else shoe.dat = shoe.drp;
                break;
            case 2: // srp
                if (!w) shoe.srp = shoe.dat & 0xffff0203ffffffff;
                else shoe.dat = shoe.srp;
                break;
            case 3: // crp
                if (!w) shoe.crp = shoe.dat & 0xffff0203ffffffff;
                else shoe.dat = shoe.crp;
                break;
            case 4: // cal
                if (!w) shoe.cal = shoe.dat & ~b(11100000);
                else shoe.dat = shoe.cal;
                break;
            case 5: // val
                if (!w) shoe.val = shoe.dat & ~b(11100000);
                else shoe.dat = shoe.val;
                break;
            case 6: // scc
                if (!w) shoe.scc = shoe.dat;
                else shoe.dat = shoe.scc;
                break;
            case 7: // ac
                if (!w) shoe.ac = shoe.dat & ~b(0000000010110011);
                else shoe.dat = shoe.ac;
                break;
        }
        
        if (w)
            call_ea_write(M, sizes[p]);
        return ;
    }
    else if (f == ~b(011)) {
        if (p == 0) { // psr
            if (!w) shoe.psr.word = shoe.dat & ~b(11111111 10000111);
            else shoe.dat = shoe.psr.word;
        }
        else if (p==1) { // pcsr
            if (!w) shoe.pcsr = shoe.dat;
            else shoe.dat = shoe.pcsr;
        }
        else {
            assert(!"Unknown register number for pmove format 3!\n");
            throw_illegal_instruction();
        }
        
        if (w)
            call_ea_write(M, 2);
        return ;
    }
    
    // TODO: implement instruction formats 2 and 3 (for BAD/BAC, etc.)
    assert(!"Unsupported pmove instruction format");
    throw_illegal_instruction();
}

static int64_t ptest_search(const uint32_t _logical_addr, const uint64_t rootp)
{
    // const uint64_t rootp = (tc_sre() && sr_s()) ? shoe.srp : shoe.crp;
    uint8_t desc_level = 0;
    int64_t desc_addr = -1; // address of the descriptor (-1 -> register)
    uint8_t wp = 0; // Whether any descriptor in the search has wp (write protected) set
    uint8_t i;
    uint64_t desc = rootp; // Initial descriptor is the root pointer descriptor
    uint8_t desc_size = 1; // And the root pointer descriptor is always 8 bytes (1==8 bytes, 0==4 bytes)
    uint8_t used_bits = shoe.tc_is; // Keep track of how many bits will be the effective "page size"
    // (If the table search terminates early (before used_bits == ts_ps()),
    //  then this will be the effective page size. That is, the number of bits
    //  we or into the physical addr from the virtual addr)
    
    shoe.psr.word = 0;
    
    desc_addr = -1; // address of the descriptor (-1 -> register)
    
    /* We'll keep shifting logical_addr such that its most significant bits are the next ti-index */
    uint32_t logical_addr = _logical_addr << used_bits;
    
    // TODO: Check limit here
    
    // If root descriptor is invalid, throw a bus error
    if (rp_dt(rootp) == 0) {
        shoe.psr.bits.b = 1; // bus error
        shoe.psr.bits.n = 0;
        return desc_addr;
    }
    
    // desc is a page descriptor, skip right ahead to search_done:
    if (rp_dt(rootp) == 1)
        goto search_done;
    
    for (i=0; i < 4; i++) { // (the condition is unnecessary - just leaving it in for clarity)
        // desc must be a table descriptor here
        
        const uint8_t ti = tc_ti(i);
        used_bits += ti;
        
        // TODO: do the limit check here
        
        // Find the index into our current i-level table
        const uint32_t index = logical_addr >> (32-ti);
        logical_addr <<= ti;
        
        // load the child descriptor
        if (_logical_addr == 0)
            slog("Loading descriptor s=%llu from addr=0x%08x\n", desc_dt(desc, desc_size) & 1, (uint32_t)desc_table_addr(desc));
        const uint32_t table_base_addr = desc_table_addr(desc);
        const uint8_t s = desc_dt(desc, desc_size) & 1;
        
        // desc = pget(table_base_addr + (4 << s)*index, (4 << s));
        get_desc(table_base_addr + (4 << s)*index, (4 << s));
        desc_size = s;
        
        // Desc may be a table descriptor, page descriptor, or indirect descriptor
        
        const uint8_t dt = desc_dt(desc, desc_size);
        if (_logical_addr == 0)
            slog("i=%u desc = 0x%llx dt=%u\n", i, desc, dt);
        
        // If this descriptor is invalid, throw a bus error
        if (dt == 0) {
            shoe.psr.bits.i = 1; // invalid
            assert(desc_level <= 7);
            shoe.psr.bits.n = desc_level;
            return desc_addr;
        }
        
        if (dt == 1) {
            // TODO: do a limit check here
            goto search_done; // it's a page descriptor
        }
        else if ((i==3) || (tc_ti(i+1)==0)) {
            desc_size = desc_dt(desc, s) & 1;
            
            /* Otherwise, if this is a leaf in the tree, it's an indirect descriptor.
             The size of the page descriptor is indicated by DT in the indirect descriptor. (or is it???) */
            //desc = pget(desc & 0xfffffff0, (4 << desc_size));
            get_desc(desc & 0xfffffff0, (4 << desc_size));
            
            // I think it's possible for an indirect descriptor to point to an invalid descriptor...
            if (desc_dt(desc, desc_size) == 0) {
                shoe.psr.bits.i = 1; // invalid
                assert(desc_level <= 7);
                shoe.psr.bits.n = desc_level;
                return desc_addr;
            }
            
            goto search_done;
        }
        
        // Now it must be a table descriptor
        
        // TODO: set the U (used) bit in this table descriptor
        
        wp |= desc_wp(desc, desc_size); // or in the wp flag for this table descriptor
        
    }
    
    // never get here
    assert(!"translate_logical_addr: never get here");
    
    
search_done:
    // Desc must be a page descriptor
    
    // NOTE: The limit checks have been done already
    
    // TODO: check U (used) bit
    
    wp |= desc_wp(desc, desc_size); // or in the wp flag for this page descriptor
    
    shoe.psr.bits.w = wp;
    
    if (desc & (desc_size ? desc_m_long : desc_m_short))
        shoe.psr.bits.m = 1;
    
    assert(desc_level <= 7);
    shoe.psr.bits.n = desc_level;
    
    return desc_addr;
}

void inst_mc68851_ptest(uint16_t ext){
    verify_supervisor();

    ~decompose(shoe.op, 1111 0000 00 MMMMMM);
    ~decompose(ext, 100 LLL R AAAA FFFFF); // Erata in 68kPRM - F is 6 bits, and A is 3

    assert(shoe.tc_enable); // XXX: Throws some exception if tc_enable isn't set
    assert(tc_fcl() == 0); // XXX: I can't handle function code lookups, and I don't want to
    assert(L == 7); // XXX: Not currently handling searching to a particular level

    // Find the function code
    uint8_t fc;
    if (F>>4) // Function code is the low 4 bits of F
        fc = F & 0xf;
    else if ((F >> 3) == 1) // Function code is in shoe.d[F & 7]
        fc = shoe.d[F & 7] & 0xf;
    else if (F == 1) // Function code is SFC
        fc = shoe.sfc;
    else if (F == 0) // Function code is DFC
        fc = shoe.dfc;
    else
        assert(!"ptest: unknown FC bits in instruction");
    
    // Pick a root pointer based on the function code
    uint64_t rootp;
    if (fc == 1)
        rootp = shoe.crp;
    else if (fc == 5)
        rootp = shoe.srp;
    else {
        slog("ptest: I can't handle this FC: %u pc=0x%08x\n", fc, shoe.orig_pc);
        assert(!"ptest: I can't handle this FC");
    }
    
    call_ea_addr(M);
    
    const int64_t desc_addr = ptest_search(shoe.dat, rootp);
    
    if ((desc_addr >= 0) && (A >> 3)) {
        shoe.a[A & 7] = (uint32_t)desc_addr;
    }

    

    // slog("%s: Error, not implemented!\n", __func__);
}

void dis_mc68851_prestore() {
    sprintf(dis.str, "prestore");
}

void dis_mc68851_psave() {
    sprintf(dis.str, "psave");
}

void dis_mc68851_pbcc() {
    sprintf(dis.str, "pbcc");
}


void dis_mc68851_pdbcc(uint16_t cond) {
    sprintf(dis.str, "pdbcc");
}

void dis_mc68851_ptrapcc(uint16_t cond) {
    sprintf(dis.str, "ptrapcc");
}

void dis_mc68851_pscc(uint16_t cond) {
    sprintf(dis.str, "pscc");
}


void dis_mc68851_pload(uint16_t ext) {
    sprintf(dis.str, "pload");
}

void dis_mc68851_pvalid(uint16_t ext) {
    sprintf(dis.str, "pvalid");
}

void dis_mc68851_pflush(uint16_t ext) {
    sprintf(dis.str, "pflush");
}

void dis_mc68851_pmove(uint16_t ext) {
    ~decompose(dis_op, 1111 000 000 MMMMMM);
    ~decompose(ext, fff ppp w 0000 nnn 00);
    
    // instruction format #1
    if (f == ~b(010)) {
        const uint8_t sizes[8] = {4, 8, 8, 8, 1, 1, 1, 2};
        char *names[8] = {
            "tc", "drp", "srp", "crp", "cal", "val", "scc", "ac"
        };
        
        if (w)
            sprintf(dis.str, "pmove %s,%s", names[p], decode_ea_rw(M, sizes[p]));
        else
            sprintf(dis.str, "pmove %s,%s", decode_ea_rw(M, sizes[p]), names[p]);
        
        return ;
    }
    else if (f == ~b(011)) {
        char *names[8] = {"psr" , "pcsr", "?", "?", "?", "?", "?", "?"};
        if (w)
            sprintf(dis.str, "pmove %s,%s", names[p], decode_ea_rw(M, 2));
        else
            sprintf(dis.str, "pmove %s,%s", decode_ea_rw(M, 2), names[p]);
        
        return ;
    }
    
    // TODO: implement instruction formats 2 and 3 (for BAD/BAC, etc.)
    sprintf(dis.str, "pmove ???");
    
}

void dis_mc68851_ptest(uint16_t ext) {
    ~decompose(dis_op, 1111 0000 00 MMMMMM);
    ~decompose(ext, 100 LLL R AAAA FFFFF); // Erata in 68kPRM - F is 6 bits, and A is 3

    if (A >> 3) 
        sprintf(dis.str, "ptest%c 0x%x,%s,%u,a%u", "wr"[R], F, decode_ea_addr(M), L, A & 7);
    else
        sprintf(dis.str, "ptest%c 0x%x,%s,%u", "wr"[R], F, decode_ea_addr(M), L);
}

void dis_mc68851_pflushr(uint16_t ext) {
    sprintf(dis.str, "pflushr");
}





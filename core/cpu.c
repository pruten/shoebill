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
#include <assert.h>
#include "../core/shoebill.h"
#include "../core/mc68851.h"

global_shoebill_context_t shoe;

static _Bool _cc_t() {return 1;}
static _Bool _cc_f() {return 0;}
static _Bool _cc_hi() {return !sr_c() && !sr_z();}
static _Bool _cc_ls() {return sr_c() || sr_z();}
static _Bool _cc_cc() {return !sr_c();}
static _Bool _cc_cs() {return sr_c();}
static _Bool _cc_ne() {return !sr_z();}
static _Bool _cc_eq() {return sr_z();}
static _Bool _cc_vc() {return !sr_v();}
static _Bool _cc_vs() {return sr_v();}
static _Bool _cc_pl() {return !sr_n();}
static _Bool _cc_mi() {return sr_n();}
static _Bool _cc_ge() {return (sr_n() && sr_v()) || (!sr_n() && !sr_v());}
static _Bool _cc_lt() {return (sr_n() && !sr_v()) || (!sr_n() && sr_v());}
static _Bool _cc_gt() {return (sr_n() && sr_v() && !sr_z()) || (!sr_n() && !sr_v() && !sr_z());}
static _Bool _cc_le() {return sr_z() || (sr_n() && !sr_v()) || (!sr_n() && sr_v());}
typedef _Bool (*_cc_func)();
static const _cc_func evaluate_cc[16] = {
    _cc_t, _cc_f, _cc_hi, _cc_ls, _cc_cc, _cc_cs, _cc_ne, _cc_eq,
    _cc_vc, _cc_vs, _cc_pl, _cc_mi, _cc_ge, _cc_lt, _cc_gt, _cc_le
};


#define nextword() ({const uint16_t w=lget(shoe.pc,2); if (shoe.abort) {return;}; shoe.pc+=2; w;})
#define nextlong() ({const uint32_t L=lget(shoe.pc,4); if (shoe.abort) {return;}; shoe.pc+=4; L;})
#define verify_supervisor() {if (!sr_s()) {throw_privilege_violation(); return;}}

~newmacro(inst, 2, {
    # This macro encapsulates an instruction implementation
    my ($name, $code) = @$args;
    
    my $out = "void inst_$name () {\n" . $code . "}";
    return $out;
})

~inst(callm, {
    assert(!"callm: error, not implemented\n");
})

~inst(chk2_cmp2, {
    assert(!"chk2_cmp2: error: not implemented\n");
})

~inst(illegal, {
    assert(!"illegal: error: not implemented\n");
})

~inst(move16, {
    assert(!"move16: SHOULD NOT BE CALLED\n"); // shouldn't be called by 68020 decoder
})

~inst(rtm, {
    assert(!"rtm: error: not implemented\n");
})

~inst(tas, {
    assert(!"tas: error: not implemented\n");
})

~inst(trapcc, {
    ~decompose(shoe.op, 0101 cccc 11111 xyz);
    
    // (xyz) == (100) -> sz=0
    // (xyz) == (010) -> sz=2
    // (xyz) == (011) -> sz=4
    const uint32_t sz = y << (z+1); // too clever
    const uint32_t next_pc = shoe.pc + sz;
    
    if (evaluate_cc[c]())
        throw_frame_two(shoe.sr, next_pc, 7, shoe.orig_pc);
    else
        shoe.pc = next_pc;
})

~inst(trapv, {
    if (sr_v())
        throw_frame_two(shoe.sr, shoe.pc, 7, shoe.orig_pc);
})

~inst(asx_reg, {
    ~decompose(shoe.op, 1110 ccc d ss i 00 rrr);
    const uint8_t sz = 1<<s;
    
    uint8_t count, lastb;
    if (i) 
        count = shoe.d[c] % 64;
    else 
        count = (c==0)?8:c;
        
    if (d) { // left shift
        uint32_t dat = get_d(r, sz);
        const uint8_t origmib = mib(dat, sz);
        uint8_t i=0, deltamib=0, lastb=0;
        for (; i<count; i++) {
            lastb = mib(dat, sz);
            dat <<= 1;
            if ( mib(dat, sz) != origmib )
                deltamib = 1;
        }
        set_d(r, dat, sz);
        if (count) {
            set_sr_x(lastb);
        }
        set_sr_n(mib(dat, sz));
        set_sr_z(get_d(r, sz) == 0);
        set_sr_v(deltamib);
        set_sr_c(lastb);
        return ;
    }
    else { // right shift
        uint32_t dat = get_d(r, sz);
        const uint32_t tmask = (mib(dat,sz)) << (sz*8-1); // 0 if mib==0, 0x80, 0x8000, or 0x80000000 if mib==1
        
        //printf("asr: dat=%u sz=%u count=%u", dat, sz, count);
        
        uint8_t i=0, lastb=0;
        for (; i < count; i++) {
            lastb = dat & 1;
            dat >>= 1;
            dat |= tmask;
        }
        set_d(r, dat, sz);
        if (count) {
            set_sr_x(lastb);
        }
        set_sr_n(mib(dat, sz));
        set_sr_z(get_d(r, sz) == 0);
        set_sr_v(0);
        set_sr_c(lastb);
        
        //printf(" result=%u\n", get_d(r, sz));
    }
})

~inst(asx_mem, {
    ~decompose(shoe.op, 1110 000d 11 MMMMMM);
    
    call_ea_read(M, 2);
    const uint16_t dat = shoe.dat;
    if (d) { // left shift <ea> 1 bit
        
        const uint16_t R = dat << 1;
        const uint16_t lost = (dat >> 15);
        
        shoe.dat = R;
        call_ea_write(M, 2);
        
        set_sr_x(lost);
        set_sr_c(lost);
        set_sr_v((R >> 15) != (dat >> 15)); // set if the MSB changes (at anytime during the shift operation)
        set_sr_n(R>>15);
        set_sr_z(R==0);
    }
    else { // right shift <ea> 1 bit
        const uint16_t R = (uint16_t)(((int16_t)dat) >> 1);
        const uint16_t lost = dat & 1;
        
        printf("asr_mem: shifted 0x%04x to 0x%04x lost=%u\n", dat, R, lost);
        
        shoe.dat = R;
        call_ea_write(M, 2);
        
        set_sr_x(lost);
        set_sr_c(lost);
        set_sr_v(0);
        set_sr_n(R>>15);
        set_sr_z(R==0);
    }
})

// GCC returns garbage when you shift a value by >= its size: ((uint32_t)123)<<32) == garbage
// So we'll use macros for risky shifts
#define shiftl(_v, _b) ({const uint32_t v=(_v), b=(_b); const uint32_t r=(b>=32)?0:(v<<b); r;})
#define shiftr(_v, _b) ({const uint32_t v=(_v), b=(_b); const uint32_t r=(b>=32)?0:(v>>b); r;})
~inst(lsx_reg, {
    ~decompose(shoe.op, 1110 ccc d ss i 01 rrr);
    const uint8_t sz = 1<<s;
    
    uint8_t count, lastb;
    if (i) 
        count = shoe.d[c] % 64;
    else 
        count = (c==0)?8:c;
    
    const uint32_t dat = get_d(r, sz);
    
    if (d) { // left shift
        const uint32_t shifted = shiftl(dat, count);
        lastb = mib(shiftl(dat, count-1), sz);
        set_d(r, shifted, sz);
    } else { // right shift
        const uint32_t shifted = shiftr(dat, count);
        lastb = shiftr(dat, count-1) & 1;
        set_d(r, shifted, sz);
    }
    
    // set control codes
    if (count) {
        set_sr_x(lastb);
    }
    set_sr_n(mib(shoe.d[r], sz));
    set_sr_z(chop(shoe.d[r], sz) == 0);
    set_sr_v(0);
    set_sr_c(lastb && (count != 0));
})

~inst(lsx_mem, {
    ~decompose(shoe.op, 1110 001d 11 MMMMMM);
    
    call_ea_read(M, 2);
    if (d) { // left shift <ea> 1 bit
        const uint16_t R = shoe.dat << 1;
        const uint8_t lost = (shoe.dat >> 15);
        
        shoe.dat = R;
        call_ea_write(M, 2);
        
        set_sr_x(lost);
        set_sr_c(lost);
        set_sr_v(0);
        set_sr_n(R>>15);
        set_sr_z(R==0);
    } 
    else { // right shift <ea> 1 bit
        const uint16_t R = shoe.dat>>1;
        const uint8_t lost = shoe.dat & 1;
        
        shoe.dat = R;
        call_ea_write(M, 2);
        
        set_sr_x(lost);
        set_sr_c(lost);
        set_sr_v(0);
        set_sr_n(R>>15);
        set_sr_z(R==0);
    }
})

// FIXME: rox_reg and roxx_reg can be made O(1)

~inst(roxx_reg, {
    ~decompose(shoe.op, 1110 ccc d ss i 10 rrr);
    
    const uint8_t sz = 1<<s;
    uint32_t data = get_d(r, sz);
    const uint32_t hi_bit = (sz * 8) - 1;
    const uint32_t hi_mask = 1 << hi_bit;
    
    uint32_t carry = 0;
    uint32_t extend = sr_x();
    uint8_t count = i ? (shoe.d[c] & 63) : ( c ? c : 8) ;
    uint32_t j;
    
    
    
    if (!d) { // right
        for (j=0; j<count; j++) {
            carry = data & 1;
            data >>= 1;
            data |= (extend << hi_bit);
            extend = carry;
        }
    }
    else { // left
        for (j=0; j<count; j++) {
            carry = (data & hi_mask) != 0;
            data <<= 1;
            data |= extend;
            extend = carry;
        }
    }
    
    set_d(r, data, sz);
    
    set_sr_x(extend);
    set_sr_v(0);
    set_sr_c(carry);
    set_sr_n(mib(data, sz));
    set_sr_z(data == 0);
})

~inst(roxx_mem, {
    assert(!"roxx_mem: unimplemented\n");
})

~inst(rox_reg, {
    ~decompose(shoe.op, 1110 ccc d ss i 11 rrr);
    
    const uint8_t sz = 1<<s;
    uint32_t data = get_d(r, sz);
    const uint32_t hi_bit = (sz * 8) - 1;
    const uint32_t hi_mask = 1 << hi_bit;
    
    uint32_t carry = 0;
    uint8_t count = i ? (shoe.d[c] & 63) : ( c ? c : 8) ;
    uint32_t j;
    
    // printf("rox_reg: count = %u\n", count);
    
    if (!d) { // right
        for (j=0; j<count; j++) {
            carry = data & 1;
            data >>= 1;
            data |= (carry << hi_bit);
        }
    }
    else { // left
        for (j=0; j<count; j++) {
            carry = (data & hi_mask) != 0;
            data <<= 1;
            data |= carry;
        }
    }
    
    set_d(r, data, sz);
    
    set_sr_v(0);
    set_sr_c(carry);
    set_sr_n(mib(data, sz));
    set_sr_z(data == 0);
})

~inst(rox_mem, {
    assert(!"rox_mem: unimplemented\n");
})

~inst(sbcd, {
    assert(!"Hey! inst_sbcd isn't implemented!\n");
})

~inst(pack, {
    assert(!"Hey! inst_pack isn't implemented!\n");
})

~inst(unpk, {
    assert(!"Hey! inst_unpk isn't implemented!\n");
})

~inst(divu, {
    ~decompose(shoe.op, 1000 rrr 011 MMMMMM);
    
    call_ea_read(M, 2);
    
    const uint32_t dividend = shoe.d[r];
    const uint16_t divisor = (uint16_t)shoe.dat;
    
    if (divisor == 0) {
        throw_frame_two(shoe.orig_sr, shoe.uncommitted_ea_read_pc, 5, shoe.orig_pc);
        return ;
    }
    
    call_ea_read_commit(M, 2);
    
    const uint32_t quotient_32 = dividend / divisor;
    const uint32_t remainder_32 = dividend % divisor;
  
    shoe.d[r] = (remainder_32 << 16) | (quotient_32 & 0xffff);
    
    set_sr_c(0);
    set_sr_n((quotient_32>>15)&1);
    set_sr_z((quotient_32 & 0xffff) == 0);
    set_sr_v(quotient_32 >> 16);
})

~inst(divs, {
    ~decompose(shoe.op, 1000 rrr 111 MMMMMM);
    
    call_ea_read(M, 2);
    
    const int32_t dividend = (int32_t)shoe.d[r];
    const uint16_t u_divisor = (uint16_t)shoe.dat;
    const int16_t s_divisor = (int16_t)shoe.dat;
    
    if (s_divisor == 0) {
        throw_frame_two(shoe.orig_sr, shoe.uncommitted_ea_read_pc, 5, shoe.orig_pc);
        return ;
    }
    
    call_ea_read_commit(M, 2);
    
    const int32_t s_quotient_32 = dividend / s_divisor;
    const int32_t s_remainder_32 = dividend % s_divisor;
    
    const uint32_t u_quotient_32 = (uint32_t)s_quotient_32;
    const uint32_t u_remainder_32 = (uint32_t)s_remainder_32;
    
    shoe.d[r] = (u_remainder_32 << 16) | (u_quotient_32 & 0xffff);
    
    set_sr_c(0);
    set_sr_n((u_quotient_32>>15)&1);
    set_sr_z((u_quotient_32 & 0xffff) == 0);
    set_sr_v(u_quotient_32 >> 16);
})

~inst(bkpt, {
    assert(!"Hey! inst_bkpt isn't implemented!\n");
})

~inst(swap, {
    ~decompose(shoe.op, 0100 1000 0100 0 rrr);
    shoe.d[r] = (shoe.d[r]>>16) | (shoe.d[r]<<16);
    set_sr_c(0);
    set_sr_v(0);
    set_sr_z(shoe.d[r]==0);
    set_sr_n(mib(shoe.d[r], 4));
})

~inst(abcd, {
    ~decompose(shoe.op, 1100 xxx 10000 m yyy);
    uint8_t packed_x, packed_y;
    const uint8_t extend = sr_x() ? 1 : 0;
    
    // printf("abcd: pc=0x%08x extend=%u m=%u x=%u y=%u\n", shoe.orig_pc, extend, m, x, y);
    
    if (m) {
        // predecrement mem to predecrement mem
        // FIXME: these addresses aren't predecremented (check whether a7 is incremented 2 bytes)
        assert(!"acbd is broken");
        packed_x = lget(shoe.a[x], 1);
        if (shoe.abort) return ;
        packed_y = lget(shoe.a[y], 1);
        if (shoe.abort) return ;
    }
    else {
        packed_x = shoe.d[x] & 0xff;
        packed_y = shoe.d[y] & 0xff;
    }
    
    if (((packed_x & 0xF) > 9) || (((packed_x>>4) & 0xF) > 9) || ((packed_y & 0xF) > 9) || (((packed_y>>4) & 0xF) > 9))
        assert(!"abcd: badly packed byte");
    
    const uint8_t unpacked_x = (packed_x & 0xf) + ((packed_x >> 4) * 10);
    const uint8_t unpacked_y = (packed_y & 0xf) + ((packed_y >> 4) * 10);
    const uint8_t sum = unpacked_x + unpacked_y + extend;
    const uint8_t unpacked_sum = sum % 100;
    const uint8_t carry = (sum >= 100);
    const uint8_t packed_sum = ((unpacked_sum / 10) << 4) | (unpacked_sum % 10);
    
    // printf("abcd: packed_x = 0x%02x(%u) packed_y = 0x%02x(%u) sum=0x%02x(%u) carry=%u\n", packed_x, unpacked_x, packed_y, unpacked_y, packed_sum, unpacked_sum, carry);
    
    if (unpacked_sum)
        set_sr_z(0);
    set_sr_c(carry);
    set_sr_x(carry);
    
    if (m) {
        lset(shoe.a[x]-1, 1, packed_sum);
        if (shoe.abort) return ;
        
        shoe.a[x]--;
        if (x != y)
            shoe.a[y]--;
    }
    else {
        set_d(x, packed_sum, 1);
    }

})

~inst(muls, {
    ~decompose(shoe.op, 1100 rrr 011 MMMMMM);
    
    call_ea_read(M, 2);
    call_ea_read_commit(M, 2);
    
    const uint16_t u_a = get_d(r, 2);
    const uint16_t u_b = shoe.dat;
    const int16_t s_a = (int16_t)u_a;
    const int16_t s_b = (int16_t)u_b;
    
    const int32_t s_result = ((int32_t)s_a) * ((int32_t)s_b);
    const uint32_t result = (uint32_t)s_result;
    
    //printf("muls: %d * %d = %d\n", s_a, s_b, s_result);
    
    shoe.d[r] = result;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_z(result == 0);
    set_sr_n(mib(result, 4));
})

/* short form unsigned multiply */
~inst(mulu, {
    ~decompose(shoe.op, 1100 rrr 011 MMMMMM);
    
    call_ea_read(M, 2);
    call_ea_read_commit(M, 2);
    
    const uint16_t a = (uint16_t) get_d(r, 2);
    const uint16_t b = (uint16_t) shoe.dat;
    const uint32_t result = ((uint32_t)a) * ((uint32_t)b);
    
    shoe.d[r] = result;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_z(result == 0);
    set_sr_n(mib(result, 4));
})

~inst(exg, {
    ~decompose(shoe.op, 1100 xxx 1 ppppp yyy);
    
    if (p == ~b(01000)) { // data reg mode
        const uint32_t tmp = shoe.d[x];
        shoe.d[x] = shoe.d[y];
        shoe.d[y] = tmp;
    }
    else if (p == ~b(01001)) { // address reg mode
        const uint32_t tmp = shoe.a[x];
        shoe.a[x] = shoe.a[y];
        shoe.a[y] = tmp;
    }
    else if (p == ~b(10001)) { // addr/reg mode
        const uint32_t tmp = shoe.d[x];
        shoe.d[x] = shoe.a[y];
        shoe.a[y] = tmp;
    }
    else {
        // I'm not sure whether decode.c will map opcodes with other modes here
        assert(!"inst_exg: bad op mode");
    }
})

~inst(stop, {
    verify_supervisor();
    
    const uint16_t ext = nextword();
    set_sr(ext);
    
    shoe.cpu_thread_notifications |= SHOEBILL_STATE_STOPPED;
})

~inst(rtr, {
    const uint16_t ccr = lget(shoe.a[7], 2);
    if (shoe.abort) return ;
    
    const uint32_t pc = lget(shoe.a[7]+2, 4);
    if (shoe.abort) return ;
    
    shoe.a[7] += 6;
    
    // We don't need to use sr_set() here because only changing the condition codes
    shoe.sr = (shoe.sr & 0xff00) | (ccr & 0x00ff);
    shoe.pc = pc;
})

~inst(rtd, {
    const int16_t disp = nextword();
    const uint32_t new_pc = lget(shoe.a[7], 4);
    if (shoe.abort) return ;
    
    shoe.pc = new_pc;
    shoe.a[7] += 4;
    shoe.a[7] += disp;
})

~inst(rte, {
    verify_supervisor();
    
    const uint16_t sr = lget(shoe.a[7], 2);
    if (shoe.abort) return ;
    
    const uint32_t pc = lget(shoe.a[7]+2, 4);
    if (shoe.abort) return ;
    
    const uint16_t format_word = lget(shoe.a[7]+6, 2);
    if (shoe.abort) return ;
    
    // printf("rte: sr=0x%04x pc=0x%08x format=0x%04x, post-pop a7=0x%08x\n", sr, pc, format_word, shoe.a[7]+8);
    
    switch (format_word >> 12) {
        case 0:
        case 1: {
            shoe.a[7] += 8;
            shoe.pc = pc;
            set_sr(sr);
            return ;
        }
        case 0xa: {
            // FIXME: I should probably check the special status word here
            shoe.a[7] += 16 * 2;
            shoe.pc = pc;
            set_sr(sr);
            return ;
        }
        case 0xb: {
            //FIXME: Check the special status word + version
            shoe.a[7] += 46 * 2;
            shoe.pc = pc;
            set_sr(sr);
            return ;
        }
        default:
            printf("rte?? I don't recognize this exception format! 0x%04x\n", format_word);
            assert(!"rte?? Don't recognize this exception format!\n");
            break;
    }
    
})

~inst(move_usp, {
    verify_supervisor();
    ~decompose(shoe.op, 0100 1110 0110 d rrr);
    
    if (d) {
        make_stack_pointers_valid();
        shoe.a[r] = shoe.usp;
    }
    else {
        make_stack_pointers_valid();
        shoe.usp = shoe.a[r];
        load_stack_pointer();
    }
})

~inst(and, {
    ~decompose(shoe.op, 1100 rrr dss MMMMMM);
    const uint8_t sz = 1<<s;
    
    if (d) { 
        // Dn ^ <ea> -> <ea>
        //  S     D      R
        call_ea_read(M, sz);
        const uint32_t R = chop(shoe.dat & shoe.d[r], sz);
        shoe.dat = R;
        call_ea_write(M, sz);
        
        set_sr_c(0);
        set_sr_v(0);
        set_sr_z(R==0);
        set_sr_n(mib(R, sz));
        return ;
    }
    else { 
        // <ea> ^ Dn -> Dn
        //   S     D    R
        call_ea_read(M, sz);
        call_ea_read_commit(M, sz);
        
        const uint32_t R = chop(shoe.dat & shoe.d[r], sz);
        set_d(r, R, sz);
        
        set_sr_c(0);
        set_sr_v(0);
        set_sr_z(R==0);
        set_sr_n(mib(R, sz));
        return ;
    }
})

~inst(or, {
    ~decompose(shoe.op, 1000 rrr dss MMMMMM);
    const uint8_t sz = 1<<s;
    
    if (d) { 
        // Dn V <ea> -> <ea>
        //  S     D      R
        call_ea_read(M, sz);
        const uint32_t R = chop(shoe.dat | shoe.d[r], sz);
        shoe.dat = R;
        call_ea_write(M, sz);
        
        set_sr_c(0);
        set_sr_v(0);
        set_sr_z(R==0);
        set_sr_n(mib(R, sz));
        return ;
    }
    else { 
        // <ea> V Dn -> Dn
        //   S     D    R
        call_ea_read(M, sz);
        call_ea_read_commit(M, sz);
        
        const uint32_t R = chop(shoe.dat | shoe.d[r], sz);
        set_d(r, R, sz);
        
        set_sr_c(0);
        set_sr_v(0);
        set_sr_z(R==0);
        set_sr_n(mib(R, sz));
        return ;
    }
})


~inst(moveq, {
    ~decompose(shoe.op, 0111 rrr 0 dddddddd);
    const int32_t dat = ((int8_t)d);
    shoe.d[r] = dat;
    set_sr_c(0);
    set_sr_v(0);
    set_sr_z(d==0);
    set_sr_n(d>>7);
    // printf("dat = %x, shoe.d[%u] = %x\n", dat, r, shoe.d[r]);
    // printf("I'm called, right?\n");
})

~inst(add, {
    ~decompose(shoe.op, 1101 rrr d ss MMMMMM);
    const uint8_t sz = 1<<s;
    uint8_t Sm, Dm, Rm;
    
    call_ea_read(M, sz);
    if (d) { // store the result in EA
        // source is Dn, dest is <ea>, result is <ea>
        Sm = mib(shoe.d[r], sz);
        Dm = mib(shoe.dat, sz);
        shoe.dat += get_d(r, sz);
        set_sr_z(chop(shoe.dat, sz) == 0);
        Rm = mib(shoe.dat, sz);
        call_ea_write(M, sz);
    }
    else { // store the result in d[r]
        // source is <ea>, dest in Dn, result is Dn
        Sm = mib(shoe.dat, sz);
        Dm = mib(shoe.d[r], sz);
        const uint32_t R = shoe.dat + get_d(r, sz);
        set_sr_z(chop(R, sz) == 0);
        Rm = mib(R, sz);
        set_d(r, R, sz);
        call_ea_read_commit(M, sz);
    }
    set_sr_v((Sm && Dm && !Rm) || (!Sm && !Dm && Rm));
    set_sr_c((Sm && Dm) || (!Rm && Dm) || (Sm && !Rm));
    set_sr_x(sr_c());
    set_sr_n(Rm);
})

~inst(adda, {
    ~decompose(shoe.op, 1101 rrr s11 MMMMMM)
    const uint8_t sz = 2 + 2*s;
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    if (s) { // long
        shoe.a[r] += shoe.dat;
    } else { // word
        const int16_t ea = shoe.dat;
        shoe.a[r] += ea;
    }
})

~inst(addx, {
    ~decompose(shoe.op, 1101 xxx 1 ss 00 r yyy);
    const uint8_t sz = 1<<s;
    
    const uint8_t extend_bit = (sr_x() != 0);
    
    uint8_t Sm, Dm, Rm;
    
    if (!r) { // x and y are data registers
        // Sm = mib(shoe.d[y] + extend_bit, sz); // FIXME: I am almost certain this is wrong
        Sm = mib(shoe.d[y], sz);
        Dm = mib(shoe.d[x], sz);
        
        const uint32_t R = shoe.d[y] + extend_bit + shoe.d[x];
        
        // printf("addx: S d[%u] = 0x%x, D d[%u] = 0x%x, R = 0x%x\n", crop(shoe.d[y], sz), crop(shoe.d[x], sz), crop(R, sz));
        
        set_d(x, R, sz);
        Rm = mib(R, sz);
        if (chop(R, sz)) // Z is cleared only if result is non-zero
            set_sr_z(0);
        
    }
    else { // memory to memory
        // FIXME: Definitely broken!
        // The subx version thinks that (sz==1) means predecrement 2 bytes, but I have no idea where it got that logic from.
        assert(y!=7 && x!=7);
        const uint32_t predec_y = shoe.a[y] - sz;
        const uint32_t predec_x = shoe.a[x] - sz;
        
        const uint32_t S = lget(predec_y, 4);
        if (shoe.abort) return ;
        const uint32_t D = lget(predec_x, 4);
        if (shoe.abort) return ;
        
        // Sm = mib(S + extend_bit, sz); // FIXME: sure you want to do this?
        Sm = mib(S, sz);
        Dm = mib(D, sz);
        
        const uint32_t R = S + extend_bit + D;
        Rm = mib(R, sz);
        
        const uint32_t chopped_R = chop(R, sz);
        lset(chopped_R, sz, shoe.dat);
        if (shoe.abort) return ;
        
        shoe.a[y] = predec_y;
        shoe.a[x] = predec_x;
        
        if (chopped_R) // Z is cleared only if result is non-zero
            set_sr_z(0);
        
    }
    
    set_sr_v((Sm && Dm && !Rm) || (!Sm && !Dm && Rm));
    set_sr_c((Sm && Dm) || (!Rm && Dm) || (Sm && !Rm));
    set_sr_x(sr_c());
    set_sr_n(Rm);
})

~inst(cmp, {
    // cmp <ea>, Dn
    // <ea> -> source
    // Dn -> dest
    // (Dn-<ea>) -> result
    ~decompose(shoe.op, 1011 rrr ooo MMMMMM);
    const uint8_t sz = 1<<o;
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    const uint8_t Sm = ea_n(sz);
    const uint8_t Dm = mib(get_d(r,sz),sz);
    const uint32_t R = shoe.d[r]-shoe.dat;
    const uint8_t Rm = mib(R, sz);
    
    set_sr_z(chop(R,sz)==0);
    set_sr_n(Rm);
    set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
    set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
})

~inst(cmpi, {
    ~decompose(shoe.op, 0000 1100 ss MMMMMM);
    const uint8_t sz = 1<<s;
    uint32_t immed;
    
    if (s < 2) {
        immed = chop(nextword(), sz);
    } else {
        immed = nextlong();
    }
    
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    
    // subtract (EA-immediate)
    const uint8_t Sm = mib(immed, sz);
    const uint8_t Dm = ea_n(sz);
    const uint32_t R = shoe.dat - immed;
    const uint8_t Rm = mib(R, sz);
    set_sr_z(chop(R,sz)==0);
    set_sr_n(Rm);
    set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
    set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
})

~inst(cmpa, {
    // D - S -> cc
    // Dn, ea
    ~decompose(shoe.op, 1011 rrr o11 MMMMMM);
    const uint8_t sz = 2<<o;
    
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    
    if (!o) { // shoe.dat needs to be sign extended
        const int16_t a = shoe.dat;
        const int32_t b = (int32_t)a;
        shoe.dat = b;
    }
    
    const uint32_t R = shoe.a[r] - shoe.dat;
    const uint8_t Rm = mib(R, 4);
    const uint8_t Sm = mib(shoe.dat, 4);
    const uint8_t Dm = mib(shoe.a[r], 4);
    set_sr_z(R == 0); /* <- This is where A/UX 3.0.0 started working */
    set_sr_n(Rm);
    set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
    set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
})

~inst(cmpm, {
    ~decompose(shoe.op, 1011 xxx 1 ss 001 yyy);
    
    const uint8_t sz = 1 << s;
    const uint32_t ax = shoe.a[x];
    const uint32_t ay = shoe.a[y] + ((x==y) ? sz : 0); // if x==y, then we pop the stack twice
    
    const uint32_t dst = lget(ax, sz);
    if (shoe.abort) return ;
    
    const uint32_t src = lget(ay, sz);
    if (shoe.abort) return ;
    
    // *I believe* that if x==y, then that register will only be incremented *once*
    // WRONG!
    
    if (sz == 1)
        assert(x!=7 && y!=7);
    
    if (x == y) {
        shoe.a[x] += sz + sz;
    }
    else {
        shoe.a[x] += sz;
        shoe.a[y] += sz;
    }
    
    const uint8_t Sm = ea_n(sz);
    const uint8_t Dm = mib(dst,sz);
    const uint32_t R = dst - src;
    const uint8_t Rm = mib(R, sz);
    
    set_sr_z(chop(R, sz) == 0);
    set_sr_n(Rm);
    set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
    set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
})

~inst(eor, {
    ~decompose(shoe.op, 1011 rrr 1ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    call_ea_read(M, sz);
    shoe.dat ^= shoe.d[r];
    const uint32_t R = shoe.dat;
    call_ea_write(M, sz);
    
    set_sr_v(0);
    set_sr_c(0);
    set_sr_z(ea_z(sz));
    set_sr_n(ea_n(sz));
})

~inst(long_mul, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 0100 1100 00 MMMMMM);
    ~decompose(ext, 0 LLL u s 0000000 HHH);
    
    call_ea_read(M, 4);
    call_ea_read_commit(M, 4);
    
    uint64_t R;
    
    if (!u) { // if unsigned...
        const uint32_t S = shoe.d[L];
        const uint32_t D = shoe.dat;
        R = ((uint64_t)S) * ((uint64_t)D);
    } else { // if signed...
        const int32_t S = shoe.d[L];
        const int32_t D = shoe.dat;
        const int64_t R_signed = ((int64_t)S) * ((int64_t)D);
        R = R_signed;
        
        //printf("long_muls: %d * %d = (int64_t)%lld (int32_t)%d\n", S, D, R, (uint32_t)(R&0xffffffff));
    }
    
    shoe.d[L] = (uint32_t)R;
    if (s) { // 64-bit result
        shoe.d[H] = (uint32_t)(R>>32);
        set_sr_n((uint8_t)(R>>63));
        set_sr_z(R==0);
        set_sr_v(0);
        set_sr_c(0);
    } else { // 32-bit result
        // FIXME: I'm not positive this behavior is right:
        // Is sr_n set if the quad-word is negative? or if the long-word is?
        // I'm presuming long-word, but I didn't actually check.
        // Also check whether sr_z is set if R==0
        // (The documentation is vague)
        //
        // Update: I checked, and this seems to be right
        set_sr_v((R>>32)!=0);
        set_sr_n((R>>31)&1);
        set_sr_z((R&0xffffffff)==0);
        set_sr_c(0);
    }
})

~inst(long_div, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 0100 1100 01 MMMMMM);
    ~decompose(ext, 0 qqq u s 0000000 rrr);
    call_ea_read(M, 4);
    
    const uint32_t divisor = shoe.dat;
    if (divisor == 0) {
        throw_frame_two(shoe.orig_sr, shoe.uncommitted_ea_read_pc, 5, shoe.orig_pc);
        return ;
    }
    
    call_ea_read_commit(M, 4);
    
    uint64_t dividend;
    if (s)
        dividend = (((uint64_t)shoe.d[r])<<32) | shoe.d[q];
    else {
        if (u) // if signed-mode, we need to sign extend 32-bit dividends (This caused a quickdraw bug that took forever to debug!)
            dividend = ((int32_t)shoe.d[q]);
        else
            dividend = shoe.d[q];
    }
    
    uint64_t Q;
    uint32_t R;
    if (u) { // if signed
        Q = (uint64_t)((int64_t)(((int64_t)dividend) / ((int32_t)divisor)));
        R = (uint32_t)((int32_t)(((int64_t)dividend) % ((int32_t)divisor)));
    } else { // if unsigned
        Q = ((uint64_t)dividend) / ((uint32_t)divisor);
        R = ((uint64_t)dividend) % ((uint32_t)divisor);
    }
    
    if (s) { // if 64 bit dividend,
        const uint8_t overflow = ((Q>>32)!=0);
        set_sr_z((Q & 0xffffffff) == 0);
        set_sr_n((Q>>31)&1);
        set_sr_v(overflow);
        set_sr_c(0);
        if (!overflow) { // only modify the registers if !overflow
            shoe.d[r] = R;
            shoe.d[q] = (uint32_t)Q;
        }
    } else { // if 32 bit dividend
        set_sr_z((Q & 0xffffffff) == 0);
        set_sr_n((Q>>31)&1);
        set_sr_v(0); // can't overflow with 32 bit dividend
        set_sr_c(0);
        shoe.d[r] = R;
        shoe.d[q] = (uint32_t)Q; // if r==q, then only set r[q]=Q
    }
})

~inst(addq, {
    ~decompose(shoe.op, 0101 ddd 0 ss MMMMMM);
    const uint8_t dat = d + ((!d)<<3);
    if ((s==0) && ((M>>3) == 1)) {
        // There's a very subtle distinciton in the M68000 Family Programmer's Reference Manual,
        // "Only word and long operations can be used with address registers" with subq, but
        // "Word and long operations are also allowed on the address registers" with addq.
        // Hmm...
        // What it actually means is "If size==byte, illegal instruction. If size==word -> pretend size is long."
        throw_illegal_instruction();
        return ;
    }
    else if ((M>>3) == 1) { // size is always long if using addr register, CCodes aren't set.
        shoe.a[M&7] += dat;
        return ;
    }
    
    call_ea_read(M, 1<<s);
    const uint8_t Sm = 0; // Sm should be always be 0 because source is always 1-8 (I think)
    const uint8_t Dm = ea_n(1<<s);
    shoe.dat += dat;
    const uint8_t Rm = ea_n(1<<s);
    
    set_sr_v((Sm && Dm && !Rm) || (!Sm && !Dm && Rm));
    set_sr_c((Sm && Dm) || (!Rm && Dm) || (Sm && !Rm));
    set_sr_x(sr_c());
    set_sr_n(Rm);
    set_sr_z(ea_z(1<<s));
    call_ea_write(M, 1<<s);
})

~inst(subq, {
    ~decompose(shoe.op, 0101 ddd 1 ss MMMMMM);
    const uint8_t dat = d + ((!d)<<3);
    
    if ((s==0) && ((M>>3) == 1)) { // Reject byte-size for addr registers
        throw_illegal_instruction();
        return ;
    }
    else if ((M>>3) == 1) { // Use long-size for addr registers
        shoe.a[M&7] -= dat;
        return ;
    }
    
    call_ea_read(M, 1<<s);
    const uint8_t Sm = 0; // Sm should be always be 0 because source is always 1-8 (I think)
    const uint8_t Dm = ea_n(1<<s);
    shoe.dat -= dat;
    const uint8_t Rm = ea_n(1<<s);
    
    set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
    set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
    set_sr_x(sr_c());
    set_sr_n(Rm);
    set_sr_z(ea_z(1<<s));
    call_ea_write(M, 1<<s);
})

~inst(movea, {
    ~decompose(shoe.op, 00 ab rrr 001 MMMMMM);
    
    const uint8_t sz = b ? 2 : 4;
    
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    if (b) { // word-size, sign extend shoe.dat
        const int16_t dat = shoe.dat;
        shoe.a[r] = (int32_t)dat;
    } else {
        shoe.a[r] = shoe.dat;
    }
})

~inst(move_d_to_d, {
    ~decompose(shoe.op, 00 ab RRR 000 000 rrr); // r=source, R=dest
    
    const uint8_t sz = 1<<(a+(!b)); // (1=byte, 3=word, 2=long)
    const uint32_t val = chop(shoe.d[r], sz);
    
    set_d(R, val, sz);
    
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(mib(val, sz));
    set_sr_z(val == 0);
})

~inst(move_from_d, {
    ~decompose(shoe.op, 00 ab RRR MMM 000 rrr); // r=source, MR=dest
    
    const uint8_t sz = 1<<(a+(!b)); // (1=byte, 3=word, 2=long)
    const uint32_t val = chop(shoe.d[r], sz);
    
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(mib(val, sz));
    set_sr_z(val == 0);
    
    shoe.dat = val;
    call_ea_write((M << 3) | R, sz);
})

~inst(move_to_d, {
    ~decompose(shoe.op, 00 ab rrr 000 mmmmmm); // m=source, r=dest
    const uint8_t sz = 1<<(a+(!b)); // (1=byte, 3=word, 2=long)
    
    call_ea_read(m, sz);
    call_ea_read_commit(m, sz);
    
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(ea_n(sz));
    set_sr_z(ea_z(sz));
    set_d(r, shoe.dat, sz);
})

~inst(move, {
    ~decompose(shoe.op, 00 ab RRR MMM mmm rrr); // mr=source, MR=dest
    const uint8_t sz = 1<<(a+(!b)); // (1=byte, 3=word, 2=long)
    
    call_ea_read((m<<3) | r, sz);
    call_ea_read_commit((m<<3) | r, sz); // We aren't writing-back, we're reading from one EA and writing to another
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(ea_n(sz));
    set_sr_z(ea_z(sz));
    
    shoe.mr = (M << 3) | R;
    shoe.sz = sz;
    ea_write();
    
    // There's a problem here
    // If the source EA is -(a7) or (a7)+, and ea_write fails while running in user-mode
    // Then it'll switch to supervisor mode while throwing the exception, and swap out USP for
    // MSP. Then modifying a[7] here will corrupt MSP.
    // So in this case, we need to switch back to the original a7 (set_sr(shoe.orig_sr)) before modifying it
    // (This is hacky)
    
    if (shoe.abort) {
        if (m == 4 || m == 3) {
            const uint16_t new_sr = shoe.sr;
            const uint8_t delta = ((r==7) && (sz==1)) ? 2 : sz;
            
            set_sr(shoe.orig_sr); // See hack comment above
            
            // if read was a post-increment, pre-decrement, then we need to rollback that change
            if (m == 3)  // postincrement
                shoe.a[r] -= delta;
            else // predecrement
                shoe.a[r] += delta;
            
            set_sr(new_sr);
        }
    }
})

~inst(not, {
    ~decompose(shoe.op, 0100 0110 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    call_ea_read(M, sz);
    shoe.dat = ~~shoe.dat;
    set_sr_z(ea_z(sz));
    set_sr_n(ea_n(sz));
    set_sr_v(0);
    set_sr_c(0);
    call_ea_write(M, sz);
})


~inst(reset, {
    verify_supervisor();
    
    // Reset does a number of things (so... look them up)
    // 1: reset the "enabled" bit of the TC register
    
    printf("Reset! (not implemented)\n");
    assert(!"reset called");
    //dbg_state.running = 0;
})

~inst(movec, {
    verify_supervisor();
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 0100 1110 0111 101x);
    ~decompose(ext, t rrr cccccccccccc);
    uint32_t *reg = t?(shoe.a):(shoe.d);
    switch (c) {
        case 0x002: { // CACR
            // FIXME: I think more bits are usable in CACR
            if (x) shoe.cacr = reg[r] & 0x80008080;
            else reg[r] = shoe.cacr;
            return ;
        }
        /* // These are >'020 registers
         case 0x003: { // TC
            // TC is a 16 bit register, but movec is always a 32bit
            if (x) shoe.tc = reg[r] & ~b(1100 0000 0000 0000);
            else reg[r] = shoe.tc & ~b(1100 0000 0000 0000);
            return ;
        }
        case 0x006: // DTT0
            if (x) shoe.dtt0 = reg[r] & ~b(11111111 11111111 11100011 01100100);
            else reg[r] = shoe.dtt0 & ~b(11111111 11111111 11100011 01100100);
            return ;
        case 0x007: // DTT1
            if (x) shoe.dtt1 = reg[r] & ~b(11111111 11111111 11100011 01100100);
            else reg[r] = shoe.dtt1 & ~b(11111111 11111111 11100011 01100100);
            return ; */
        case 0x801: // VBR
            if (x) shoe.vbr = reg[r];
            else reg[r] = shoe.vbr;
            return ;
        case 0x800: // USP
            make_stack_pointers_valid();
            if (x) shoe.usp = reg[r];
            else reg[r] = shoe.usp;
            load_stack_pointer();
            return ;
        // (NO!) case 0x802: // CAAR (not supported on 68040)
        case 0x000: // SFC
            if (x) shoe.sfc = reg[r] & 7;
            else reg[r] = shoe.sfc & 7;
            return ;
        case 0x001: // DFC
            if (x) shoe.dfc = reg[r] & 7;
            else reg[r] = shoe.dfc & 7;
            return ;
        case 0x803: // MSP
        case 0x804: // ISP
        case 0x004: // ITT0
        case 0x005: // ITT1
        case 0x805: // MMUSR
        case 0x806: // URP
        case 0x807: // SRP
            printf("inst_movec: error! I don't support this condition code yet! (0x%03x)\n", c);
            assert(!"inst_movec: error: unknown condition\n");
            return ;
            
        default:
            return throw_illegal_instruction();
    }
})

~inst(moves, {
    verify_supervisor();
    const uint16_t ext = nextword();
    
    ~decompose(shoe.op, 0000 1110 ss MMMMMM);
    ~decompose(ext, a rrr d 00000000000);
    
    if ((M>>3) < 2) {
        // Dn and An addressing modes not supported
        throw_illegal_instruction();
        return ;
    }
    
    const uint8_t fc = d ? shoe.dfc : shoe.sfc;
    const uint8_t sz = 1<<s;
    
    /* From 68851 documentation
     * 0: undefined
     * 1: User data space
     * 2: User code space
     * 3: undefined
     * 4: undefined
     * 5: Supervisor data space
     * 6: Supervisor code space
     * 7: CPU space
     */
    
    // For now, only supporting fc 1 (user data space)
    if (fc != 1) {
        printf("inst_moves: error: hit fc=%u\n", fc);
        assert(!"inst_moves: error, hit weird function code");
        return ;
    }
    
    uint32_t addr;
    if ((M>>3) == 4) {
        assert((M&7) != 7);
        // Predecrement
        addr = shoe.a[M & 7] - sz;
    }
    else if ((M>>3) < 4) {
        if ((M>>3) == 3) assert((M&7) != 7);
        // Post increment or addr indirect
        addr = shoe.a[M & 7];
    }
    else {
        // all other EA modes
        call_ea_addr(M);
        addr = (uint32_t)shoe.dat;
    }
    
    if (d) {
        const uint32_t data = chop(a ? shoe.a[r] : shoe.d[r], sz);
        
        // Motorola's 68k documentation has a note that for moves An,(An)+ or -(An),
        // the value of An written is incremented or decremented
        // XXX: implement this
        assert(! ((a && (M&7)==r) && (((M>>3) == 3) || ((M>>3) == 4))) );
        
        lset_fc(addr, sz, data, fc);
        if (shoe.abort)
            return ;
    }
    else {
        uint32_t data = lget_fc(addr, sz, fc);
        if (shoe.abort)
            return ;
        
        // if destination is address register, data is sign extended to 32 bits
        if (a && (sz == 1)) {
            int8_t d8 = data;
            int32_t d32 = d8;
            data = d32;
        }
        else if (a && (sz == 2)) {
            int16_t d16 = data;
            int32_t d32 = d16;
            data = d32;
        }
        
        if (a)
            shoe.a[r] = data;
        else
            set_d(r, data, sz);
        
    }
    
    // commit predecrement/postincrement addr reg changes
    if ((M>>3) == 4)
        shoe.a[M & 7] -= sz;
    else if ((M>>3) == 3)
        shoe.a[M & 7] += sz;
    
})

~inst(cas, {
    assert(!"inst_cas: error: not implemented!");
})

~inst(cas2, {
    assert(!"inst_cas2: error: not implemented!");
})

~inst(move_to_sr, {
    verify_supervisor();
    ~decompose(shoe.op, 0100 0110 11 MMMMMM);
    call_ea_read(M, 2); // sr is 2 bytes
    call_ea_read_commit(M, 2);
    
    set_sr(shoe.dat);
})

~inst(move_from_sr, {
    verify_supervisor();
    ~decompose(shoe.op, 0100 0000 11 MMMMMM);
    shoe.dat = shoe.sr;
    call_ea_write(M, 2);
})

~inst(neg, {
    ~decompose(shoe.op, 0100 0100 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    call_ea_read(M, sz);
    const uint32_t D = shoe.dat;
    const uint32_t R = 0 - D;
    const uint8_t Dm = mib(D, sz);
    const uint8_t Rm = mib(R, sz);
    shoe.dat = R;
    call_ea_write(M, sz);
    
    const uint32_t result = chop(R, sz);
    
    set_sr_z(result == 0);
    set_sr_c(result != 0);
    set_sr_x(result != 0);
    set_sr_v(Dm && Rm);
    set_sr_n(Rm);
})

~inst(negx, {
    ~decompose(shoe.op, 0100 0000 ss MMMMMM);
    
    const uint8_t x = (sr_x() != 0);
    
    if (s == 3) { // is it possible for the decoder to send s==3 here?
        throw_illegal_instruction();
        return ;
    }
    
    const uint8_t sz = 1<<s;
    call_ea_read(M, sz);
    
    const uint32_t D = shoe.dat;
    const uint32_t R = 0 - D - x;
    const uint8_t Dm = mib(D, sz);
    const uint8_t Rm = mib(R, sz);
    shoe.dat = R;
    call_ea_write(M, sz);
    
    const uint32_t c = chop(R, sz);
    if (c != 0)
        set_sr_z(0);
    set_sr_n(Rm);
    set_sr_v(Dm && Rm);
    set_sr_c(Dm || Rm);
    set_sr_x(Dm || Rm);
})

~inst(move_from_ccr, {
    ~decompose(shoe.op, 0100 0010 11 MMMMMM);
    shoe.dat = shoe.sr & 0xff;
    call_ea_write(M, 2);
})

~inst(move_to_ccr, {
    ~decompose(shoe.op, 0100 0100 11 MMMMMM);
    call_ea_read(M, 2); // sr is 2 bytes
    call_ea_read_commit(M, 2);
    
    const uint16_t new_sr = (shoe.sr & 0xff00) | (shoe.dat & 0xff);
    
    set_sr(new_sr);
})

~inst(tst, {
    ~decompose(shoe.op, 0100 1010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    set_sr_n(ea_n(sz));
    set_sr_z(ea_z(sz));
    set_sr_v(0);
    set_sr_c(0);
})

~inst(clr, {
    ~decompose(shoe.op, 0100 0010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    shoe.dat = 0;
    call_ea_write(M, sz);
    set_sr_n(0);
    set_sr_z(1);
    set_sr_v(0);
    set_sr_c(0);
})

~inst(lea, {
    ~decompose(shoe.op, 0100 rrr 111 MMMMMM);
    call_ea_addr(M);
    shoe.a[r] = shoe.dat;
})

~inst(sub, {
    ~decompose(shoe.op, 1001 rrr dss MMMMMM);
    const uint8_t sz = 1<<s;
    
    // make sure the high order bytes of shoe.dat are cleared 
    // (I don't think this should be necessary, ea_*() should guarantee the highorder bytes are cleared)
    shoe.dat = 0; 
    call_ea_read(M, sz);
    if (d) { // <ea> - Dn -> <ea>
        const uint32_t result = shoe.dat - get_d(r, sz);
        {
            const uint8_t Sm = mib(shoe.d[r], sz);
            const uint8_t Dm = mib(shoe.dat, sz);
            const uint8_t Rm = mib(result, sz);
            set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
            set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
            set_sr_x(sr_c());
            set_sr_n(Rm);
            set_sr_z(chop(result,sz)==0);
        }
        shoe.dat = result;
        call_ea_write(M, sz);
    }
    else { // Dn - <ea> -> Dn
        const uint32_t result = get_d(r, sz) - shoe.dat;
        {
            const uint8_t Sm = mib(shoe.dat, sz);
            const uint8_t Dm = mib(shoe.d[r], sz);
            const uint8_t Rm = mib(result, sz);
            set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
            set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
            set_sr_x(sr_c());
            set_sr_n(Rm);
            set_sr_z(chop(result,sz)==0);
        }
        set_d(r, result, sz);
        call_ea_read_commit(M, sz);
    }
})

~inst(subi, {
    ~decompose(shoe.op, 0000 0100 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    // ea = ea - immed
    // R    D    S
    
    // fetch immediate word
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(nextword() & 0xff);
    } else if (s==1) {
        immed = (int16_t)nextword();
    } else {
        immed = nextlong();
    }
    
    // fetch the destination operand
    call_ea_read(M, sz);
    
    // do the subtraction
    const uint32_t result = shoe.dat - immed;
    // find the MIBs for source, dest, and result
    const uint8_t Sm = mib(immed, sz);
    const uint8_t Dm = mib(shoe.dat, sz);
    const uint8_t Rm = mib(result, sz);
    // set sr flags
    set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
    set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
    set_sr_x(sr_c());
    set_sr_n(Rm);
    set_sr_z(chop(result,sz)==0);
    
    // writeback
    shoe.dat = chop(result, sz);
    call_ea_write(M, sz);
})
    
~inst(addi, {
    ~decompose(shoe.op, 0000 0110 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    // fetch immediate word
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(nextword() & 0xff);
    } else if (s==1) {
        immed = (int16_t)nextword();
    } else {
        immed = nextlong();
    }
    
    call_ea_read(M, sz);
    const uint32_t Sm = mib(immed, sz);
    const uint32_t Dm = mib(shoe.dat, sz);
    const uint32_t R = shoe.dat + immed;
    const uint32_t Rm = mib(R, sz);
    shoe.dat = R;
    call_ea_write(M, sz);
    
    set_sr_z(chop(R, sz)==0);
    set_sr_n(mib(R, sz));
    set_sr_v((Sm && Dm && !Rm) || (!Sm && !Dm && Rm));
    const uint8_t carry = ((Sm && Dm) || (!Rm && Dm) || (Sm && !Rm));
    set_sr_c(carry);
    set_sr_x(carry);
})

~inst(eori, {
    ~decompose(shoe.op, 0000 1010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    // fetch immediate word
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(nextword() & 0xff);
    } else if (s==1) {
        immed = (int16_t)nextword();
    } else {
        immed = nextlong();
    }
    
    call_ea_read(M, sz);
    const uint32_t R = shoe.dat ^ immed;
    shoe.dat = R;
    call_ea_write(M, sz);
    
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(mib(R, sz));
    set_sr_z(chop(R, sz)==0);
})
    

~inst(andi, {
    ~decompose(shoe.op, 0000 0010 ss MMMMMM);
    const uint8_t sz = 1<<s;
    
    // fetch immediate word
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(nextword() & 0xff);
    } else if (s==1) {
        immed = (int16_t)nextword();
    } else {
        immed = nextlong();
    }
    
    // fetch the destination operand
    call_ea_read(M, sz);
    // do the AND
    const uint32_t result = shoe.dat & immed;
    // writeback
    shoe.dat = result;
    call_ea_write(M, sz);
    
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(mib(result, sz));
    set_sr_z(chop(result,sz)==0);
})

~inst(ori, {
    ~decompose(shoe.op, 0000 0000 ss MMMMMM)
    const uint8_t sz = 1<<s;
    
    // fetch immediate word
    uint32_t immed;
    if (s==0) {
        immed = (int8_t)(nextword() & 0xff);
    } else if (s==1) {
        immed = (int16_t)nextword();
    } else {
        immed = nextlong();
    }
    
    // fetch the destination operand
    call_ea_read(M, sz);
    // do the AND
    const uint32_t result = shoe.dat | immed;
    set_sr_v(0);
    set_sr_c(0);
    set_sr_n(mib(result, sz));
    set_sr_z(chop(result,sz)==0);
    
    // writeback
    shoe.dat = chop(result, sz);
    call_ea_write(M, sz);
})

~inst(btst_immediate, {
    ~decompose(shoe.op, 0000 1000 00 MMMMMM);
    const uint16_t ext = nextword();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    /* 
     * "When a data register is the destination, any of the 32 bits can be
     * specified by a modulo 32-bit number." Otherwise, it's modulo 8.
     */
    if ((M >> 3) == 0) {
        set_sr_z(! ((shoe.d[M&7] >> (ext % 32)) & 1) );
        return ;
    }
    
    const uint8_t n = b % 8;
    call_ea_read(M, 1); 
    call_ea_read_commit(M, 1);
    
    set_sr_z(!((shoe.dat >> n)&1));
})

~inst(pea, {
    ~decompose(shoe.op, 0100 1000 01 MMMMMM);
    // fetch the EA
    call_ea_addr(M);
    // push it onto the stack
    lset(shoe.a[7]-4, 4, shoe.dat);
    if (shoe.abort) return ;
    // decrement the stack pointer if lset didn't abort
    shoe.a[7] -= 4;
})
    
~inst(subx, {
    ~decompose(shoe.op, 1001 yyy 1 ss 00 m xxx);
    const uint8_t sz = 1<<s;
    
    if (m) {
        assert(x!=7 && y!=7);
        const uint8_t predecrement_sz = s?(1<<s):2; // predecrement-mode for size==byte decrements by 2 bytes
        const uint32_t predecrement_x = shoe.a[x]-predecrement_sz;
        const uint32_t predecrement_y = shoe.a[y]-predecrement_sz;

        const uint32_t src = lget(predecrement_x, sz);
        if (shoe.abort) return ;
        
        const uint32_t dst = lget(predecrement_y, sz);
        if (shoe.abort) return ;
        
        const uint32_t result = dst - src - (sr_x()?1:0);
        
        lset(predecrement_y, sz, result);
        
        shoe.a[x] = predecrement_x;
        shoe.a[y] = predecrement_y;
        
        const uint8_t Sm = mib(src, sz);
        const uint8_t Dm = mib(dst, sz);
        const uint8_t Rm = mib(result, sz);
        
        set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
        set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
        set_sr_x(sr_c());
        set_sr_n(Rm);
        if (chop(result, sz)!=0) 
            set_sr_z(0);
    }
    else {
        const uint32_t src = shoe.d[x];
        const uint32_t dst = shoe.d[y];
        const uint32_t result = dst - src - (sr_x()?1:0);
        
        set_d(y, result, sz);
        
        const uint8_t Sm = mib(src, sz);
        const uint8_t Dm = mib(dst, sz);
        const uint8_t Rm = mib(result, sz);
        
        set_sr_v((!Sm && Dm && !Rm) || (Sm && !Dm && Rm));
        set_sr_c((Sm && !Dm) || (Rm && !Dm) || (Sm && Rm));
        set_sr_x(sr_c());
        set_sr_n(Rm);
        if (chop(result, sz)!=0) 
            set_sr_z(0);
    }
})

~inst(suba, {
    ~decompose(shoe.op, 1001 rrr s11 MMMMMM);
    const uint8_t sz = 2<<s;
    
    // suba <ea>, An
    // An - <ea> -> An
    // D      S      R
    
    call_ea_read(M, sz);
    call_ea_read_commit(M, sz);
    // if sz==word, sign extend source to long
    const uint32_t S = s ? shoe.dat : ((uint32_t) ((int16_t)shoe.dat)); 
    const uint32_t R = shoe.a[r] - S;
    shoe.a[r] = R;
    
    // don't set any condition codes!
})

~inst(dbcc, {
    ~decompose(shoe.op, 0101 cccc 11001 rrr);
    if (evaluate_cc[c]()) {
        shoe.pc += 2;
    }
    else {
        const int16_t disp = nextword();
        const uint16_t newd = get_d(r, 2) - 1;
        set_d(r, newd, 2);
        if (newd != 0xffff)
            shoe.pc = shoe.pc + disp - 2;
    }
})

~inst(bsr, {
    ~decompose(shoe.op, 0110 0001 dddddddd);
    uint32_t new_pc = shoe.pc;
    
    // find the new PC
    if ((d==0) || (d==0xff)) {
        if (d==0xff) {
            new_pc += nextlong();
        }
        else
            new_pc += ((int16_t)nextword());
    }
    else {
        uint8_t tmp = d;
        new_pc += ((int8_t)d);
    }
    
    lset(shoe.a[7]-4, 4, shoe.pc);
    if (shoe.abort) return ;
    shoe.a[7] -= 4;
    shoe.pc = new_pc;
})

~inst(bcc, {
    const uint32_t orig_pc = shoe.pc;
    ~decompose(shoe.op, 0110 cccc dddddddd);
    
    if (evaluate_cc[c]()) {
        if (d == 0) {
            const int16_t ext = (int16_t)nextword();
            shoe.pc = orig_pc + ext;
        }
        else if (d == 0xff) {
            shoe.pc = orig_pc + nextlong();
        }
        else {
            const int8_t tmp = (int8_t)d;
            shoe.pc = orig_pc + tmp;
        }
    }
    else {
        if (d == 0) shoe.pc += 2;
        else if (d == 0xff) shoe.pc += 4;
    }
})

~inst(scc, {
    ~decompose(shoe.op, 0101 cccc 11 MMMMMM);
    
    shoe.dat = evaluate_cc[c]() ? 0xff : 0;
    call_ea_write(M, 1);
})

~inst(nop, {})

~inst(chk, {
    ~decompose(shoe.op, 0100 rrr 1s 0 MMMMMM);
    
    const uint8_t sz = s ? 2 : 4;
    
    call_ea_read(M, sz);
    
    int32_t reg, ea;
    
    if (s) { // word
        int16_t tmp = shoe.d[r] & 0xffff;
        reg = tmp;
        tmp = shoe.dat & 0xffff;
        ea = tmp;
    }
    else { // long word
        reg = shoe.d[r];
        ea = shoe.dat & 0xffffffff;
    }
    
    if ((reg < 0) || (reg > ea)) {
        set_sr_n((reg < 0));
        throw_frame_two(shoe.sr, shoe.pc, 6, shoe.orig_pc);
    }
    
    call_ea_read_commit(M, sz);

    return ;
})

~inst(jsr, {
    ~decompose(shoe.op, 0100 1110 10 MMMMMM);
    call_ea_addr(M);
    
    // my quadra 800 doesn't object when a7 is odd... 
    // so, no extra error checking needed
    lset(shoe.a[7]-4, 4, shoe.pc);
    if (shoe.abort) return ;
    
    // printf("jsr: writing pc (0x%08x) to *0x%08x (phys=0x%08x)\n", shoe.pc, shoe.a[7]-4, shoe.physical_addr);
    
    shoe.a[7] -= 4;
    shoe.pc = shoe.dat;
    /*
    if (sr_s()) {
        //return ;
        coff_symbol *symb = coff_find_func(shoe.coff, shoe.pc);
        
        if (symb) {
            sprintf(ring_tmp, "CALL (s) %s+%u 0x%08x\n", symb->name, shoe.pc-symb->value, shoe.pc);
            ring_print(ring_tmp);
        }
        else {
            sprintf(ring_tmp, "CALL (s) unknown+ 0x%08x\n", shoe.pc);
            ring_print(ring_tmp);
        }
        
    }
    else {
        char *name = (char*)"unknown";
        uint32_t value = 0;
        if ((shoe.pc >= 0x40000000) && (shoe.pc < 0x50000000)) {
            uint32_t i, addr = shoe.pc % (shoe.physical_rom_size);
            for (i=0; macii_rom_symbols[i].name; i++) {
                if (macii_rom_symbols[i].addr > addr) {
                    break;
                }
                name = (char*)macii_rom_symbols[i].name;
                value = macii_rom_symbols[i].addr;
            }
        }
        
        
        sprintf(ring_tmp, "CALL (u) %s+%u 0x%08x\n", name, shoe.pc-value, shoe.pc);
        ring_print(ring_tmp);
    }
    */
    
})

~inst(jmp, {
    ~decompose(shoe.op, 0100 1110 11 MMMMMM);
    call_ea_addr(M);
    shoe.pc = shoe.dat;
})

~inst(link_word, {
    ~decompose(shoe.op, 0100 1110 0101 0 rrr);
    const int16_t ext = nextword();
    
    // push the contents of the address register onto the stack
    lset(shoe.a[7]-4, 4, shoe.a[r]);
    if (shoe.abort) return ;
    shoe.a[7] -= 4;
    
    // load the updated stack pointer into the address register
    shoe.a[r] = shoe.a[7];
    
    // add the (sign-extended) displacement value to the stack pointer
    shoe.a[7] += ext;
})

~inst(unlk, {
    ~decompose(shoe.op, 0100 1110 0101 1 rrr);
    
    const uint32_t pop = lget(shoe.a[r], 4);
    if (shoe.abort) return ;
    
    // loads the stack pointer from the address register
    shoe.a[7] = shoe.a[r]+4;
    
    // loads the address register with the long word popped from the stack
    shoe.a[r] = pop;
})

~inst(rts, {
    const uint32_t pop = lget(shoe.a[7], 4);
    if (shoe.abort) return ;
    
    shoe.a[7] += 4;
    shoe.pc = pop;
    
    /*
    if (sr_s() && 0) {
        // return ;
        coff_symbol *symb = coff_find_func(shoe.coff, shoe.pc);
        if (symb)
            printf("RETURN TO %s+%u 0x%08x\n", symb->name, shoe.pc-symb->value, shoe.pc);
    }
    else if (0){
        char *name = (char*)"unknown";
        uint32_t value = 0;
        if ((shoe.pc >= 0x40000000) && (shoe.pc < 0x50000000)) {
            uint32_t i, addr = shoe.pc % (shoe.physical_rom_size);
            for (i=0; macii_rom_symbols[i].name; i++) {
                if (macii_rom_symbols[i].addr > addr) {
                    break;
                }
                name = (char*)macii_rom_symbols[i].name;
                value = macii_rom_symbols[i].addr;
            }
        }
        
        printf("RETURN TO %s+%u 0x%08x\n", name, shoe.pc-value, shoe.pc);
    }*/
})


~inst(link_long, {
    ~decompose(shoe.op, 0100 1000 0000 1 rrr);
    const uint32_t disp = nextlong();
    
    // push the contents of the address register onto the stack
    lset(shoe.a[7]-4, 4, shoe.a[r]);
    if (shoe.abort) return ;
    shoe.a[7] -= 4;
    
    // load the updated stack pointer into the address register
    shoe.a[r] = shoe.a[7];
    
    // add the displacement value to the stack pointer
    shoe.a[7] += disp;
})

~inst(movem, {
    const uint16_t mask = nextword();
    ~decompose(shoe.op, 0100 1d00 1s MMMMMM);
    const uint8_t sz = 2<<s; // s==0 => short, s==1 => long
    uint32_t i;
    
    if (d) { // memory->register
        if (~bmatch(M, xx100xxx)) { // predecrement isn't allowed for mem->reg
            throw_illegal_instruction();
            return ;
        }
        if (~bmatch(M, xx011xxx)) // if postincrement,
            shoe.dat = shoe.a[M&7]; // hand-parse this address mode
        else
            call_ea_addr(M); // otherwise, ea_addr() can handle it
        
        // extract the bitfields for a and d registers,
        const uint8_t dfield = mask&0xff;
        const uint8_t afield = (mask>>8)&0xff;
        
        // read in the data registers
        for (i=0; i<8; i++) {
            if ((dfield >> i) & 1) {
                uint32_t tmp;
                if (s) 
                    tmp = lget(shoe.dat, 4);
                else 
                    tmp = (uint32_t)((int32_t)((int16_t)lget(shoe.dat, 2))); // sign-extend if short-mode
                if (shoe.abort) goto abort;
                shoe.d[i] = tmp;
                shoe.dat += sz;
            }
        }
        
        // read in the address registers
        for (i=0; i<8; i++) {
            if ((afield >> i) & 1) {
                uint32_t tmp;
                if (s) 
                    tmp = lget(shoe.dat, 4);
                else 
                    tmp = (uint32_t)((int32_t)((int16_t)lget(shoe.dat, 2))); // sign-extend if short-mode
                if (shoe.abort) goto abort;
                shoe.a[i] = tmp;
                shoe.dat += sz;
            }
        }
        
        // update register if postincrement
        if (~bmatch(M, xx011xxx))
            shoe.a[M&7] = shoe.dat;
    }
    else { // register->memory
        if (~bmatch(M, xx011xxx)) { // postincrement isn't allowed for reg->mem
            throw_illegal_instruction();
            return ;
        }
        uint32_t addr;
        uint16_t newmask = mask; // if predecrement-mode, bit-reversed mask. Regular mask otherwise.
        uint16_t numbits = 0; // the number of set bits in mask
        
        if (~bmatch(M, xx100xxx)) { // if predecrement,
            uint16_t maskcopy;
            
            // printf("*0x%08x movem %02x\n", shoe.orig_pc, shoe.op);
            
            for (maskcopy=mask, i=0; i < 16; i++) {
                newmask = (newmask<<1) | (maskcopy & 1); // build a flipped version of mask
                numbits += (maskcopy & 1); // count the bits in mask, to pre-predecrement the addr
                maskcopy >>= 1;
            }
            addr = shoe.a[M&7] - numbits * sz; // hand-parse and pre-predecrement the addr
            
            // XXX: This is broken - according to the documentation, 68020 will write (a7-sz)
            // if it's written during pre-decrement mode.
            // shoe.a[M&7] -= sz; // "pre-decrement" the address register itself (see abort:)
        }
        else {
            call_ea_addr(M); // otherwise, ea_addr() can handle it
            addr = shoe.dat;
        }
        
        const uint32_t addr_copy = addr;
        const uint8_t dfield = newmask&0xff;
        const uint8_t afield = (newmask>>8)&0xff;
        
        // write the data registers
        for (i=0; i<8; i++) {
            // FIXME: determine what happens when the predecrementing address register is written
            if ((dfield >> i) & 1) {
                lset(addr, sz, shoe.d[i]);
                if (shoe.abort) 
                    goto abort; // FIXME: figure out how to abort cleanly
                addr += sz;
            }
        }
        
        // write the address registers
        for (i=0; i<8; i++) {
            // FIXME: determine what happens when the predecrementing address register is written
            if ((afield >> i) & 1) {
                
                // For the MC68020-40, for predecrement mode, if the address register
                // is written to memory, it is "predecremented" by the size of the operation.
                // (Literally the "size" of the operation, 2 or 4 bytes)
                
                uint32_t data = shoe.a[i];
                // if this is pre-dec mode, and we're pushing the address reg specified in the EA
                if ( (M>>3) == 4 && (M&7)==i ) {
                    data -= sz;
                    printf("movem: For movem %u/%u, deciding to write 0x%08x for a%u\n", M>>3, M&7, data, M&7);
                }
                
                lset(addr, sz, data);
                if (shoe.abort) 
                    goto abort; // FIXME: figure out how to abort cleanly
                addr += sz;
            }
        }
        
        
        // update register if predecrement
        if (~bmatch(M, xx100xxx))
            shoe.a[M&7] = addr_copy;
    }
    
    return ;
    
abort:
    // For the MC68020-40, for predecrement mode, if the address register is written to memory, it is "predecremented" by the size of the operation. (Literally, 2 or 4 bytes.)
    // So undo that here:
    
    // XXX: This is broken. We should only undo the predecrement if we DID predecrement
    
    //if (~bmatch(M, xx100xxx))
        //shoe.a[M&7] += sz;
    
    return ;
})
    
~inst(nbcd, {
    assert(!"inst_nbcd: fixme: I'm not implemented!\n");
})

~inst(eori_to_ccr, {
    const uint16_t val = 0xff & nextword();
    const uint16_t new_sr = shoe.sr ^ val;
    
    set_sr(new_sr);
})

~inst(eori_to_sr, {
    verify_supervisor();
    
    const uint16_t new_sr = shoe.sr ^ nextword();
    
    set_sr(new_sr);
})

~inst(movep, {
    assert(!"inst_movep: fixme: I'm not implemented!\n");
})


void write_bitfield(const uint32_t width, const uint32_t offset, const uint32_t M, const uint32_t ea, const uint32_t raw_field)
{
    const int32_t soffset = offset;
    const uint32_t field = bitchop(raw_field, width); // make sure that field is all 0's beyond (width)
    
    // special case for EA-mode == data register
    if ((M>>3) == 0) { 
        const uint32_t reg = shoe.d[M&7]; // the EA reg
        const uint32_t ones = 0xffffffff >> (32-width); // unrotated mask
        if (soffset >= 0) {
            const uint32_t mask =         ~~(rrot32(ones  << (32-width), soffset&31));
            const uint32_t rotatedfield =    rrot32(field << (32-width), soffset&31); 
            shoe.d[M&7] = (shoe.d[M&7] & mask) | rotatedfield;
        }
        else {
            const uint32_t mask =         ~~(lrot32(ones  << (32-width), (-soffset)&31));
            const uint32_t rotatedfield =    lrot32(field  << (32-width), (-soffset)&31);
            shoe.d[M&7] = (shoe.d[M&7] & mask) | rotatedfield;
        }
    }
    else {
        const uint32_t bit_offset = offset & 7;
        // byte_offset = offset (arithmetic-right-shift) 3
        const uint32_t byte_offset = ((offset|(offset>>1)|(offset>>2)) & 0xe0000000) | (offset>>3);
        
        // call_ea_addr(M);
        
        const uint32_t first_byte_addr = ea + byte_offset;
        //printf("debug: extract_bitfield: addr = 0x%08x, first_byte_addr = 0x%08x\n", ea, first_byte_addr);
        
        
        // if the field is contained entirely within the first byte
        if (width <= (8-bit_offset)) {
            // yes this is convoluted, it simply creates a chopped/rotated mask that matches the bits in the first byte
            const uint8_t byte_mask = bitchop(0xff, width) << (8-(bit_offset+width));
            const uint8_t field_mask = field << (8-(bit_offset+width));
            const uint8_t old_byte = lget(first_byte_addr, 1);
            const uint8_t new_byte = (old_byte & (~~byte_mask)) | field_mask;
            
            lset(first_byte_addr, 1, new_byte);
            //printf("write_bitfield: byte_mask = 0x%02x field_mask = 0x%02x bit_offset=%u byte_offset=%u\n", byte_mask, field_mask, bit_offset, byte_offset);
            //printf("write_bitfield: changing byte at 0x%08x from 0x%02x to 0x%02x\n",
                   //first_byte_addr, old_byte, new_byte);
            if (shoe.abort) return ;
        }
        else {
            uint32_t boff = bit_offset;
            uint32_t curwidth = 8-bit_offset;
            uint32_t remaining_width = width;
            uint32_t addr = first_byte_addr;
            uint32_t remaining_field = field<<(32-width); // left-aligned
            while (remaining_width > 0) {
                const uint8_t byte = lget(addr, 1);
                if (shoe.abort) return ;
                
                const uint8_t mask = ~~(((uint8_t)((0xff) << (8-curwidth))) >> boff);
                const uint8_t field_chunk = remaining_field >> (32-curwidth);
                const uint8_t rotated_chunk = (field_chunk << (8-curwidth)) >> boff;
                
                // printf("mask  = 0x%02x\nfield = 0x%02x\n", mask, rotated_chunk);
                
                lset(addr, 1, (byte & mask) | rotated_chunk);
                //printf("write_bitfield: changing byte at 0x%08x from 0x%02x to 0x%02x\n",
                       //addr, byte, (byte & mask) | rotated_chunk);
                if (shoe.abort) return ;
                
                addr++;
                remaining_field <<= curwidth;
                remaining_width -= curwidth;
                curwidth = (remaining_width>8)? 8 : remaining_width;
                boff = 0;
            }
        }
    }
}
    
uint32_t extract_bitfield(const uint32_t width, const uint32_t offset, const uint32_t M, const uint32_t ea)
{
    const int32_t soffset = offset;
    uint32_t field;
    
    // special case for EA-mode == data register
    if ((M>>3) == 0) { 
        const uint32_t reg = shoe.d[M&7]; // the EA reg
        
        // OKAY: I think I have this figured out. It was laborious to discover, so don't forget it.
        // When mode=datareg, the register is ROTATED by the offset, not shifted. 
        // This has nonintuitive consequences.
        // {Offset=-27, length=32, data=1} will say a bit is set at the (-27+5)th position
        
        if (soffset >= 0) 
            field = lrot32(reg, soffset&31) >> (32-width);
        else 
            field = rrot32(reg, (-soffset)&31) >> (32-width);
    }
    else {
        const uint32_t bit_offset = offset & 7;
        // byte_offset = offset (arithmetic-right-shift) 3
        const uint32_t byte_offset = ((offset|(offset>>1)|(offset>>2)) & 0xe0000000) | (offset>>3);
        
        // this is call_ea_addr(M);
        // shoe.mr=M;
        // ea_addr();
        // if (shoe.abort) return 0;
        
        //printf("debug: extract_bitfield: offset = 0x%08x, byte_offset = %d, bit_offset = %d\n", offset, byte_offset, bit_offset);
        
        const uint32_t first_byte_addr = ea + byte_offset;
        //printf("debug: extract_bitfield: addr = 0x%08x, first_byte_addr = 0x%08x\n", ea, first_byte_addr);
        
        field = bitchop(lget(first_byte_addr, 1), 8-bit_offset);
        //printf("debug: extract_bitfield: first byte field (low %u bits): 0x%02x\n", 8-bit_offset, field);
        if (shoe.abort) return 0;
        if (width > (8-bit_offset)) { // if the data isn't entirely contained in the first byte
            uint32_t last_long = lget(first_byte_addr+1, 4);
            if (shoe.abort) return 0;
            field = (field<<(width - (8-bit_offset))) | // first_byte, left shifted
            (last_long >> (32 - (width - (8-bit_offset)))); // last_long, right shifted
        }
        else // otherwise, right-align the first byte
            field >>= (8-(bit_offset+width));
    }
    
    return field;
}
    
~inst(bfextu, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1001 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    
    shoe.d[r] = field;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1));
    set_sr_z(field == 0);
})
    
~inst(bfchg, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1011 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    write_bitfield(width, offset, M, ea, ~~field);
    if (shoe.abort) return;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1));
    set_sr_z(field == 0);
})
    
~inst(bfexts, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1011 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    const uint32_t mib = (field >> (width-1))&1;
    
    const uint32_t maskA = (((uint32_t)0) - mib) << 1;
    const uint32_t maskB = (maskA << (width-1));
    const uint32_t result = maskB | field;
    shoe.d[r] = result;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(mib);
    set_sr_z(field == 0);
})
    
~inst(bfclr, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1011 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    write_bitfield(width, offset, M, ea, 0);
    if (shoe.abort) return;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1));
    set_sr_z(field == 0);
})
    
~inst(bfset, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1011 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    write_bitfield(width, offset, M, ea, 0xffffffff);
    if (shoe.abort) return;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1));
    set_sr_z(field == 0);
})
    
~inst(bftst, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1000 11 MMMMMM);
    ~decompose(ext, 0 000 Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1));
    set_sr_z(field == 0);
})
    
~inst(bfffo, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1110 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    const uint32_t field = extract_bitfield(width, offset, M, ea);
    if (shoe.abort) return;
    
    uint32_t i;
    for (i=1; (i<=width) && ((field>>(width-i))&1)==0; i++) ;
    shoe.d[r] = offset+i-1;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1));
    set_sr_z(field == 0);
})
    
~inst(bfins, {
    const uint16_t ext = nextword();
    ~decompose(shoe.op, 1110 1011 11 MMMMMM);
    ~decompose(ext, 0 rrr Ffffff Wwwwww);
    
    const uint32_t width_ua = W ? (shoe.d[w]%32) : w; // unadjusted, [0, 31]
    const uint32_t width = (width_ua==0)?32:width_ua; // width_ua==0 => width==32 [1, 32]
    const uint32_t offset = F ? (shoe.d[f]) : f; // [0, 31]
    
    const uint32_t field = bitchop(shoe.d[r], width);
    // printf("write_bitfield: writing %u at offset=%u width=%u\n", field, offset, width);
    
    uint32_t ea = 0;
    if (M >> 3) { // If ea isn't data reg mode (handled separately in *_bitfield())
        call_ea_addr(M);
        ea = shoe.dat;
    }
    
    write_bitfield(width, offset, M, ea, field);
    if (shoe.abort) return;
    
    set_sr_c(0);
    set_sr_v(0);
    set_sr_n(field >> (width-1)); // set according to the inserted value
    set_sr_z(field == 0);
})

~inst(btst_reg, {
    ~decompose(shoe.op, 0000 rrr 100 MMMMMM);
    
    if ((M>>3)==0) { // if the dest is a data reg, handle specially
        set_sr_z(! ((shoe.d[M&7]>>(shoe.d[r]%32)) & 1) );
        return ;
    }
    
    call_ea_read(M, 1); 
    call_ea_read_commit(M, 1);
    set_sr_z(! ((shoe.dat >> (shoe.d[r] % 8)) & 1));
})

~inst(bchg_reg, {
    ~decompose(shoe.op, 0000 rrr 101 MMMMMM);
    
    // The LSB bit is idx:0, MSB is idx:7 or :31
    
    if ((M>>3)==0) { // if the dest is a data reg, handle specially
        set_sr_z((shoe.d[M&7] & (1 << (shoe.d[r]%32))) == 0);
        
        shoe.d[M&7] ^= (1 << (shoe.d[r]%32));
        
        return ;
    }
    
    call_ea_read(M, 1);
    
    const uint32_t z = ((shoe.d[r] & (1 << (shoe.d[r] % 8))) == 0);
    
    // shoe.dat ^= (0x80 >> (shoe.d[r] % 8)); // NO NO! BAD
    shoe.dat ^= (1 << (shoe.d[r] % 8));
    
    call_ea_write(M, 1);
    
    set_sr_z(z);
})

~inst(bclr_reg, {
    ~decompose(shoe.op, 0000 rrr 111 MMMMMM);
    
    const uint8_t is_data_reg = (M>>3) == 0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = shoe.d[r] % (is_data_reg ? 32 : 8);
    
    call_ea_read(M, sz);
    
    set_sr_z(((shoe.dat >> shift) & 1) == 0);
    
    shoe.dat &= ~~(1 << shift);
    call_ea_write(M, sz);
})
    
~inst(bclr_immediate, {
    ~decompose(shoe.op, 0000 1000 11 MMMMMM);
    const uint16_t ext = nextword();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = b % (is_data_reg ? 32 : 8);
    
    call_ea_read(M, sz);
    
    set_sr_z(((shoe.dat >> shift) & 1) == 0);
    
    shoe.dat &= ~~(1 << shift);
    call_ea_write(M, sz);
})

~inst(bset_reg, {
    ~decompose(shoe.op, 0000 rrr 111 MMMMMM);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = shoe.d[r] % (is_data_reg ? 32 : 8);
    
    call_ea_read(M, sz);
    
    set_sr_z(((shoe.dat >> shift) & 1) == 0);
    
    shoe.dat |= (1 << shift);
    call_ea_write(M, sz);
})
    
~inst(bset_immediate, {
    ~decompose(shoe.op, 0000 1000 11 MMMMMM);
    const uint16_t ext = nextword();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = b % (is_data_reg ? 32 : 8);
    
    call_ea_read(M, sz);
    
    set_sr_z(((shoe.dat >> shift) & 1) == 0);
    
    shoe.dat |= (1 << shift);
    call_ea_write(M, sz);
})

~inst(bchg_immediate, {
    ~decompose(shoe.op, 0000 1000 01 MMMMMM);
    const uint16_t ext = nextword();
    ~decompose(ext, 00000000 bbbbbbbb);
    
    const uint8_t is_data_reg = (M>>3)==0;
    const uint8_t sz = is_data_reg ? 4 : 1;
    const uint8_t shift = b % (is_data_reg ? 32 : 8);
    
    call_ea_read(M, sz);
    
    set_sr_z(((shoe.dat >> shift) & 1) == 0);
    
    shoe.dat ^= (1 << shift);
    call_ea_write(M, sz);
})
    
~inst(ext, {
    ~decompose(shoe.op, 0100 100 ooo 000 rrr);
    switch (o) {
        case ~b(010): { // byte -> word
            uint16_t val = (int8_t)get_d(r, 1);
            set_d(r, val, 2);
            break;
        } case ~b(011): { // word -> long
            uint32_t val = (int16_t)get_d(r, 2);
            set_d(r, val, 4);
            break;
        } case ~b(111): { // byte -> long
            uint32_t val = (int8_t)get_d(r, 1);
            set_d(r, val, 4);
            break;
        } default:
            throw_illegal_instruction();
            return ;
    }
    set_sr_v(0);
    set_sr_c(0);
    // if the LSb of o is 1, then result size == long
    set_sr_z(get_d(r, 2+2*(o&1))==0);
    set_sr_n(mib(shoe.d[r], 2+2*(o&1))); 
})
    
~inst(andi_to_sr, {
    verify_supervisor();
    const uint16_t ext = nextword();
    
    set_sr(shoe.sr & ext);
})

~inst(andi_to_ccr, {
    const uint16_t ext = nextword();
    
    set_sr(shoe.sr & (ext & 0xff));
})

~inst(ori_to_sr, {
    verify_supervisor();
    const uint16_t ext = nextword();
    
    set_sr(shoe.sr | ext);
})

~inst(ori_to_ccr, {
    const uint16_t ext = nextword();
    
    set_sr(shoe.sr | (ext & 0xff));
})
    
~inst(mc68851_decode, {
    ~decompose(shoe.op, 1111 000 a b c MMMMMM);
    
    // prestore or psave
    if (a) {
        if (~bmatch(shoe.op, 1111 000 101 xxxxxx))
            inst_mc68851_prestore();
        else if (~bmatch(shoe.op, 1111 000 100 xxxxxx))
            inst_mc68851_psave();
        else
            throw_illegal_instruction();
        return ;
    }
    
    // pbcc
    if (~bmatch(shoe.op, 1111 000 01x 00xxxx)) {
        inst_mc68851_pbcc();
        return ;
    }
    
    const uint16_t ext = nextword();
    
    // pdbcc, ptrapcc, pscc
    if (~bmatch(shoe.op, 1111 000 001 xxxxxx)) {
        ~decompose(shoe.op, 1111 000 001 mmm rrr);
        // These all just store a condition code in the extension word
        ~decompose(ext, 0000 0000 00 cccccc);
        
        if (m == 1) 
            inst_mc68851_pdbcc(c);
        else if ((m == ~b(111)) && (r > 2))
            inst_mc68851_ptrapcc(c);
        else 
            inst_mc68851_pscc(c);
        return ;
    }
    
    // shoe.op must have the form (1111 000 000 xxxxxx) now
    ~decompose(ext, XXX YYY 00 0000 0000);
    switch (X) {
        case 1: // pflush, pload, pvalid
            if (Y == ~b(000))
                inst_mc68851_pload(ext);
            else if ((Y == ~b(010)) || (Y == ~b(011)))
                inst_mc68851_pvalid(ext);
            else
                inst_mc68851_pflush(ext);
            return ;
        case 2: // pmove format 1
            inst_mc68851_pmove(ext);
            return ;
        case 3: // pmove formats 2 and 3,
            inst_mc68851_pmove(ext);
            return ;
        case 4: // ptest
            inst_mc68851_ptest(ext);
            return ;
        case 5: // pflushr
            inst_mc68851_pflushr(ext);
            return ;
        default:
            throw_illegal_instruction();
            return ;
    }
    assert(!"never get here");
})

void dump_ring();
~inst(unknown, {
    printf("Unknown instruction (0x%04x)!\n", shoe.op);
    /*if (shoe.op == 0x33fe) {
        dump_ring();
        assert(!"dumped");
    }*/
    throw_illegal_instruction();
})
    
~inst(a_line, {
    
    /*if (shoe.op == 0xA9EB || shoe.op == 0xA9EC) {
        shoe.suppress_exceptions = 1;
        
        uint32_t fp_op = lget(shoe.a[7]+0, 2);
        uint32_t fp_operand_addr = lget(shoe.a[7]+2, 4);
        
        printf("%s : op 0x%04x (", (shoe.op == 0xA9EB) ? "FP68K" : "Elems68K", fp_op);
        
        uint8_t buf[10];
        uint32_t i;
        for (i=0; i<10; i++) {
            buf[i] = lget(fp_operand_addr+i, 1);
            printf("%02x ", buf[i]);
        }
        
        {
            uint8_t castable[12] = {
                buf[9], buf[8], buf[7], buf[6], buf[5],
                buf[4], buf[3], buf[2], buf[1], buf[0],
                0, 0
            };
            printf("%Lf)\n", *(long double*)&castable[0]);
        }
        
        
        
        shoe.abort = 0;
        shoe.suppress_exceptions = 0;
    }*/
    
    throw_illegal_instruction();
})
    
void trap_debug()
{
    shoe.suppress_exceptions = 1;
    
    const uint32_t syscall = lget(shoe.a[7]+4, 4);
    printf("syscall = %u\n", shoe.d[0]);
    

    switch (shoe.d[0]) {
        case 4: {
            uint32_t fd = lget(shoe.a[7]+4, 4);
            uint32_t buf = lget(shoe.a[7]+8, 4);
            uint32_t len = lget(shoe.a[7]+12, 4);
            printf("write(%u, 0x%08x, %u) \"", fd, buf, len);
            
            uint32_t i;
            for (i=0; i<len; i++) {
                printf("%c", (uint32_t)lget(buf+i, 1));
            }
            printf("\"\n");
            
            break;
        }
        /*case 5: {
            uint32_t path_p = lget(shoe.a[7]+4, 4);
            uint32_t oflag = lget(shoe.a[7]+8, 4);
            
            printf("shoe.orig_pc = 0x%08x\n", shoe.orig_pc);
            printf("open(0x%08x, %u) \"", path_p, oflag);
            uint32_t i;
            for (i=0; 1; i++) {
                uint8_t c = lget(path_p+i, 1);
                if (c == 0) break;
                printf("%c", c);
            }
            printf("\"\n");
            break;
        }*/
        case 59: // exece
        case 11: { // exec
            char name[64];
            uint32_t path_p = lget(shoe.a[7]+4, 4);
            printf("exec(0x%08x, ...) \"", path_p);
            uint32_t i;
            for (i=0; i < 64; i++) {
                name[i] = lget(path_p+i, 1);
                if (name[i] == 0) break;
            }
            name[63] = 0;
            printf("%s\"\n", name);
            
            break;
        }
    }

    shoe.abort = 0;
    shoe.suppress_exceptions = 0;
}

~inst(trap, {
    ~decompose(shoe.op, 0100 1110 0100 vvvv);
    
    const uint32_t vector_num = 32 + v;
    const uint32_t vector_offset = vector_num * 4;
    
    // trap_debug();
    
    set_sr_s(1);
    
    push_a7(vector_offset, 2);
        if (shoe.abort) goto fail;
    push_a7(shoe.pc, 4);
        if (shoe.abort) goto fail;
    push_a7(shoe.orig_sr, 2);
        if (shoe.abort) goto fail;
    
    const uint32_t newpc = lget(shoe.vbr + vector_offset, 4);
        if (shoe.abort) goto fail;
    
    shoe.pc = newpc;
    return ;
    
fail:
    assert(!"trap - push_a7 raised shoe.abort\n"); // I can't handle this yet
})


#include "inst_decoder_guts.c"


void cpu_step()
{
    // remember the PC and SR (so we can throw exceptions later)
    shoe.orig_pc = shoe.pc;
    shoe.orig_sr = shoe.sr;
    
    // Is this an odd address? Throw an address exception!
    if (shoe.pc & 1) {
        // throw_address_error(shoe.pc, 0);
        // I'm leaving this assert in here for now because it almost always indicates a bug in the emulator when it fires
        assert(!"What do I do here?");
        return ;
    }
    
    // Fetch the next instruction word
    shoe.op = lget(shoe.pc, 2);
    
    // If there was an exception, then the pc changed. Restart execution from the beginning.
    if (shoe.abort) {
        shoe.abort = 0;
        return ;
    }
    shoe.pc+=2;
    
    inst_instruction_to_pointer[inst_opcode_map[shoe.op]]();
    
    /* The abort flag indicates that a routine should stop trying to execute the
     instruction and return immediately to cpu_step(), usually to begin
     exception processing */
    
    shoe.abort = 0; // clear the abort flag
    return ;
}


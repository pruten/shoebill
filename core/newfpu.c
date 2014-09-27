/*
 * Copyright (c) 2013-2014, Peter Rutenbar <pruten@gmail.com>
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
#include <string.h>
#include <assert.h>
#include "../core/shoebill.h"
#include "../core/SoftFloat/softfloat.h"

#pragma mark Structures and macros

// Mode control byte
#define mc_rnd  (fpu->fpcr.b._mc_rnd)
#define mc_prec (fpu->fpcr.b._mc_prec)

// Exception enable byte
#define ee_inex1 (fpu->fpcr.b._ee_inex1)
#define ee_inex2 (fpu->fpcr.b._ee_inex2)
#define ee_dz    (fpu->fpcr.b._ee_dz)
#define ee_unfl  (fpu->fpcr.b._ee_unfl)
#define ee_ovfl  (fpu->fpcr.b._ee_ovfl)
#define ee_operr (fpu->fpcr.b._ee_operr)
#define ee_snan  (fpu->fpcr.b._ee_snan)
#define ee_bsun  (fpu->fpcr.b._ee_bsun)

// Accrued exception byte
#define ae_inex (fpu->fpsr.b._ae_inex)
#define ae_dz   (fpu->fpsr.b._ae_dz)
#define ae_unfl (fpu->fpsr.b._ae_unfl)
#define ae_ovfl (fpu->fpsr.b._ae_ovfl)
#define ae_iop  (fpu->fpsr.b._ae_iop)

// Exception status byte
#define es_inex1 (fpu->fpsr.b._es_inex1)
#define es_inex2 (fpu->fpsr.b._es_inex2)
#define es_dz    (fpu->fpsr.b._es_dz)
#define es_unfl  (fpu->fpsr.b._es_unfl)
#define es_ovfl  (fpu->fpsr.b._es_ovfl)
#define es_operr (fpu->fpsr.b._es_operr)
#define es_snan  (fpu->fpsr.b._es_snan)
#define es_bsun  (fpu->fpsr.b._es_bsun)

// Quotient byte
#define qu_quotient (fpu->fpsr.b._qu_quotient)
#define qu_s        (fpu->fpsr.b._qu_s) /* quotient sign */

// Condition codes
#define cc_nan (fpu->fpsr.b._cc_nan)
#define cc_i (fpu->fpsr.b._cc_i)
#define cc_z (fpu->fpsr.b._cc_z)
#define cc_n (fpu->fpsr.b._cc_n)


typedef struct {
    uint32_t fpiar; // FPU iaddr
    
    union { // fpcr, fpu control register
        struct {
            // Mode control byte
            uint16_t _mc_zero : 4; // zero/dummy
            uint16_t _mc_rnd  : 2; // rounding mode
            uint16_t _mc_prec : 2; // rounding precision
            // Exception enable byte
            uint16_t _ee_inex1 : 1; // inexact decimal input
            uint16_t _ee_inex2 : 1; // inxact operation
            uint16_t _ee_dz    : 1; // divide by zero
            uint16_t _ee_unfl  : 1; // underflow
            uint16_t _ee_ovfl  : 1; // overflow
            uint16_t _ee_operr : 1; // operand error
            uint16_t _ee_snan  : 1; // signalling not a number
            uint16_t _ee_bsun  : 1; // branch/set on unordered
        } b;
        
        uint16_t raw;
    } fpcr;
    
    union { // fpsr, fpu status register
        struct {
            // Accrued exception byte
            uint32_t _dummy1  : 3; // dummy/zero
            uint32_t _ae_inex : 1; // inexact
            uint32_t _ae_dz   : 1; // divide by zero
            uint32_t _ae_unfl : 1; // underflow
            uint32_t _ae_ovfl : 1; // overflow
            uint32_t _ae_iop  : 1; // invalid operation
            // Exception status byte
            uint32_t _es_inex1 : 1; // inexact decimal input
            uint32_t _es_inex2 : 1; // inxact operation
            uint32_t _es_dz    : 1; // divide by zero
            uint32_t _es_unfl  : 1; // underflow
            uint32_t _es_ovfl  : 1; // overflow
            uint32_t _es_operr : 1; // operand error
            uint32_t _es_snan  : 1; // signalling not a number
            uint32_t _es_bsun  : 1; // branch/set on unordered
            // Quotient byte
            uint32_t _qu_quotient : 7;
            uint32_t _qu_s        : 1;
            // Condition code byte
            uint32_t _cc_nan  : 1; // not a number
            uint32_t _cc_i    : 1; // infinity
            uint32_t _cc_z    : 1; // zero
            uint32_t _cc_n    : 1; // negative
            uint32_t _dummy2  : 4; // dummy/zero
        } b;
        uint32_t raw;
    } fpsr;
    
    floatx80 fp[8]; // 80 bit floating point general registers
    
} fpu_state_t;


#define fpu_get_state_ptr() fpu_state_t *fpu = (fpu_state_t*)shoe.fpu_state
#define nextword() ({const uint16_t w=lget(shoe.pc,2); if (shoe.abort) {return;}; shoe.pc+=2; w;})
#define nextlong() ({const uint32_t L=lget(shoe.pc,4); if (shoe.abort) {return;}; shoe.pc+=4; L;})
#define verify_supervisor() {if (!sr_s()) {throw_privilege_violation(); return;}}

#pragma mark FPU exception throwers
enum fpu_vector_t {
    fpu_vector_ftrapcc = 7,
    fpu_vector_fline = 11,
    fpu_vector_coprocessor_protocol_violation = 13, // won't be using this one
    fpu_vector_bsun = 48,
    fpu_vector_inexact = 49,
    fpu_vector_divide_by_zero = 50,
    fpu_vector_underflow = 51,
    fpu_vector_operr = 52,
    fpu_vector_overflow = 53,
    fpu_vector_snan = 54
};

#define expush(_dat, _sz) {\
    const uint32_t sz = (_sz); \
    lset(shoe.a[7] - sz, sz, (_dat)); \
    if (shoe.abort) assert(!"fpu: expush: double fault during lset!"); \
    shoe.a[7] -= sz; \
}

static void throw_fpu_pre_instruction_exception(enum fpu_vector_t vector)
{
    throw_frame_zero(shoe.orig_sr, shoe.orig_pc, vector);
}
// Note: I may be able to get away without implementing the
//       mid-instruction exception.

/*
 * _bsun_test() is called by every inst_f*cc instruction
 * to test whether the bsun exception is enabled, throw an
 * exception if so, and otherwise just set the appropriate
 * bit in fpsr, and update the accrued exception byte.
 */
static _Bool _bsun_test()
{
    fpu_get_state_ptr();
    
    // BSUN counts against the IOP accrued exception bit
    ae_iop = 1;
    
    // Set the BSUN exception status bit
    es_bsun = 1;
    
    // If the BSUN exception isn't enabled, then we can just return
    if (!ee_bsun)
        return 0; // 0 -> elected not to throw an exception
    
    throw_fpu_pre_instruction_exception(fpu_vector_bsun);
    return 1;
}

#pragma mark Second-hop instructions

static void inst_fmath (const uint16_t ext)
{
    ~decompose(shoe.op, 1111 001 000 MMMMMM);
    ~decompose(ext, 0 a 0 sss ddd eeeeeee);
    
    const uint8_t src_in_ea = a;
    const uint8_t source_specifier = s;
    const uint8_t dest_register = d;
    const uint8_t extension = e;
    
    _Bool do_write_back_result = 1;
    
    float128 source, result;
    
    if (src_in_ea) {
        source = _fpu_read_ea(M, source_specifier);
        if (shoe.abort)
            return ;
    }
    else
        source = floatx80_to_float128(fpu->fp[source_specifier]);
    
    float128 dest = floatx80_to_float128(fpu->fp[dest_register]);
    
    
    assert(!"fmath");
}

static void inst_fmove (const uint16_t ext)
{
    assert(!"fmove");
}

static void inst_fmovem_control (const uint16_t ext)
{
    assert(!"fmovem_control");
}

static void inst_fmovem (const uint16_t ext)
{
    assert(!"fmovem");
}


#pragma mark First-hop decoder table inst implementations
/* 
 * The table generated by decoder_gen.c will refer directly
 * to these instructions. inst_fpu_other() will handle all
 * other FPU instructions.
 */

static _Bool fpu_test_cc(uint8_t cc)
{
    fpu_get_state_ptr();
    const _Bool z = cc_z;
    const _Bool n = cc_n;
    const _Bool nan = cc_nan;
    
    switch (cc & 0x0f) {
        case 0: // false
            return 0;
        case 1: // equal
            return z;
        case 2: // greater than
            return !(nan | z | n);
        case 3: // greater than or equal
            return z | !(nan | n);
        case 4: // less than
            return n & !(nan | z);
        case 5: // less than or equal
            return z | (n & !nan);
        case 6: // greater or less than
            return !(nan | z);
        case 7: // ordered
            return !nan;
        case 8: // unordered
            return nan;
        case 9: // not (greater or less than)
            return nan | z;
        case 10: // not (less than or equal)
            return nan | !(n | z);
        case 11: // not (less than)
            return nan | (z | !n);
        case 12: // not (greater than or equal)
            return nan | (n & !z);
        case 13: // not (greater than)
            return nan | z | n;
        case 14: // not equal
            return !z;
        case 15: // true
            return 1;
    }
    
    assert(0);
    return 0;
}

void inst_fscc () {
    fpu_get_state_ptr();
    
    // fscc can throw an exception
    fpu->fpiar = shoe.orig_pc;
    
    const uint16_t ext = nextword();
    
    ~decompose(shoe.op, 1111 001 001 MMMMMM);
    ~decompose(ext, 0000 0000 000 b cccc);
    
    /*
     * inst_f*cc instructions throw a pre-instruction exception
     * if b && cc_nan
     */
    if (b && _bsun_test())
        return ;
    
    shoe.dat = fpu_test_cc(c) ? 0xff : 0;
    
    call_ea_write(M, 1);
}

void inst_fbcc () {
    fpu_get_state_ptr();
    
    // fbcc can throw an exception
    fpu->fpiar = shoe.orig_pc;
    
    ~decompose(shoe.op, 1111 001 01 s 0bcccc); // b => raise BSUN if NaN
    const uint8_t sz = 2 << s;
    
    /*
     * inst_f*cc instructions throw a pre-instruction exception 
     * if b && cc_nan
     */
    if (b && _bsun_test())
        return ;
    
    if (fpu_test_cc(c)) {
        const uint16_t ext = nextword();
        uint32_t displacement;
    
        if (s) {
            const uint16_t ext2 = nextword();
            displacement = (ext << 16) | ext2;
        }
        else
            displacement = (int16_t)ext;
        
        shoe.pc = shoe.orig_pc + 2 + displacement;
    }
    else
        shoe.pc += sz;
}

void inst_fsave () {
    fpu_get_state_ptr();
    verify_supervisor();
    
    // Don't modify fpiar for fsave
    
    ~decompose(shoe.op, 1111 001 100 MMMMMM);
    ~decompose(shoe.op, 1111 001 100 mmmrrr);
    
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

void inst_frestore () {
    fpu_get_state_ptr();
    verify_supervisor();
    
    // Don't modify fpiar for frestore
    
    ~decompose(shoe.op, 1111 001 101 MMMMMM);
    ~decompose(shoe.op, 1111 001 101 mmmrrr);
    
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

void inst_fdbcc () {
    fpu_get_state_ptr();
    ~decompose(shoe.op, 1111 001 001 001 rrr);
    
    // fdbcc can throw an exception
    fpu->fpiar = shoe.orig_pc;
    
    const uint16_t ext = nextword();
    ~decompose(ext, 0000 0000 000 b cccc);
    
    /*
     * inst_f*cc instructions throw a pre-instruction exception
     * if b && cc_nan
     */
    if (b && _bsun_test())
        return ;
    
    if (fpu_test_cc(c)) {
        shoe.pc += 2;
    }
    else {
        const int16_t disp = nextword();
        const uint16_t newd = get_d(r, 2) - 1;
        set_d(r, newd, 2);
        if (newd != 0xffff)
            shoe.pc = shoe.orig_pc + 2 + disp;
    }
}

void inst_ftrapcc () {
    fpu_get_state_ptr();
    ~decompose(shoe.op, 1111 001 001 111 xyz);
    
    // ftrapcc can throw an exception
    fpu->fpiar = shoe.orig_pc;
    
    // (xyz) == (100) -> sz=0
    // (xyz) == (010) -> sz=2
    // (xyz) == (011) -> sz=4
    const uint32_t sz = y << (z+1);
    const uint32_t next_pc = shoe.orig_pc + 2 + sz;
    
    const uint16_t ext = nextword();
    ~decompose(ext, 0000 0000 000 b cccc);
    
    /*
     * inst_f*cc instructions throw a pre-instruction exception
     * if b && cc_nan
     */
    if (b && _bsun_test())
        return ;
    
    if (fpu_test_cc(c))
        throw_frame_two(shoe.sr, next_pc, 7, shoe.orig_pc);
    else
        shoe.pc = next_pc;
}

void inst_fnop() {
    // This is technically fbcc, so we should set fpiar too
    fpu->fpiar = shoe.orig_pc;
}

void inst_fpu_other () {
    fpu_get_state_ptr();
    ~decompose(shoe.op, 1111 001 000 MMMMMM);
    
    const uint16_t ext = nextword();
    ~decompose(ext, ccc xxx yyy eeeeeee);
    
    switch (c) {
        case 0: // Reg to reg
            fpu->fpiar = shoe.orig_pc; // fmath() can throw an exception
            inst_fmath(ext);
            return;
            
        case 1: // unused
            throw_illegal_instruction();
            return;
            
        case 2: // Memory->reg & movec
            fpu->fpiar = shoe.orig_pc; // fmath() can throw an exception
            inst_fmath(ext);
            return;
            
        case 3: // reg->mem
            fpu->fpiar = shoe.orig_pc; // fmove() can throw an exception
            inst_fmove(ext);
            return;
            
        case 4: // mem -> sys ctl registers
        case 5: // sys ctl registers -> mem
            // fmovem_control() cannot throw an FPU exception (don't modify fpiar)
            inst_fmovem_control(ext);
            return;
            
        case 6: // movem to fp registers
        case 7: // movem to memory
            // fmovem() cannot throw an FPU exception (don't modify fpiar)
            inst_fmovem(ext);
            return;
    }
    
    assert(0); // never get here
    return;
}

#pragma mark FPU-state initialization and reset

void fpu_initialize()
{
    fpu_state_t *fpu = (fpu_state_t*)p_alloc(shoe.pool, sizeof(fpu_state_t));
    memset(fpu, sizeof(fpu_state_t), 0);
    shoe.fpu_state = fpu;
}

void fpu_reset()
{
    p_free(shoe.fpu_state);
    fpu_initialize();
}

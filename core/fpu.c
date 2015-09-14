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
    
    // State for the static fmath instruction implementations
    float128 source, dest, result;
    _Bool write_back;
    uint16_t extension_word; // only set in inst_fmath(), only used by fsincos
    uint8_t fmath_op;
} fpu_state_t;

enum rounding_precision_t {
    prec_extended = 0,
    prec_single = 1,
    prec_double = 2,
};

enum rounding_mode_t {
    mode_nearest = 0,
    mode_zero = 1,
    mode_neg = 2,
    mode_pos = 3
};

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
static const uint8_t _format_sizes[8] = {4, 4, 12, 12, 2, 8, 1, 12};
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

#define fpu_get_state_ptr() fpu_state_t *fpu = (fpu_state_t*)shoe.fpu_state
#define nextword() ({const uint16_t w = pccache_nextword(shoe.pc); if sunlikely(shoe.abort) return; shoe.pc += 2; w;})
#define nextlong() ({const uint32_t L = pccache_nextlong(shoe.pc); if sunlikely(shoe.abort) return; shoe.pc += 4; L;})
#define verify_supervisor() {if (!sr_s()) {throw_privilege_violation(); return;}}

#pragma mark FPU exception stuff
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

/*
 * Map the exception bit positions (in fpsr and fpcr)
 * to their corresponding exception vector numbers.
 */
const uint8_t _exception_bit_to_vector[8] = {
    48, // bsun
    54, // snan
    52, // operr
    53, // ovfl
    51, // unfl
    50, // dz
    49, // inex2
    49, // inex1
};

static void throw_fpu_pre_instruction_exception(enum fpu_vector_t vector)
{
    throw_frame_zero(shoe.orig_sr, shoe.orig_pc, vector);
}
/*
 * Note: I may be able to get away without implementing the
 *       mid-instruction exception.
 */

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

static void _throw_illegal_instruction()
{
    // assert(!"throw_illegal_instruction!");
    throw_illegal_instruction();
}

#pragma mark Float format translators (to/from big-endian motorola format)

static void _floatx80_to_int8(floatx80 *f, uint8_t *ptr)
{
    uint32_t tmp = floatx80_to_int32(*f);
    ptr[0] = tmp & 0xff;
}

static void _floatx80_to_int16(floatx80 *f, uint8_t *ptr)
{
    uint32_t tmp = floatx80_to_int32(*f);
    ptr[0] = (tmp >> 8) & 0xff;
    ptr[1] = (tmp >> 0) & 0xff;
}

static void _floatx80_to_int32(floatx80 *f, uint8_t *ptr)
{
    uint32_t tmp = floatx80_to_int32(*f);
    ptr[0] = (tmp >> 24) & 0xff;
    ptr[1] = (tmp >> 16) & 0xff;
    ptr[2] = (tmp >> 8) & 0xff;
    ptr[3] = (tmp >> 0) & 0xff;
}

static void _floatx80_to_single(floatx80 *f, uint8_t *ptr)
{
    const float32 tmp = floatx80_to_float32(*f);
    ptr[0] = (tmp >> 24) & 0xff;
    ptr[1] = (tmp >> 16) & 0xff;
    ptr[2] = (tmp >> 8) & 0xff;
    ptr[3] = (tmp >> 0) & 0xff;
}

static void _floatx80_to_double(floatx80 *f, uint8_t *ptr)
{
    const float64 tmp = floatx80_to_float64(*f);
    ptr[0] = (tmp >> 56) & 0xff;
    ptr[1] = (tmp >> 48) & 0xff;
    ptr[2] = (tmp >> 40) & 0xff;
    ptr[3] = (tmp >> 32) & 0xff;
    ptr[4] = (tmp >> 24) & 0xff;
    ptr[5] = (tmp >> 16) & 0xff;
    ptr[6] = (tmp >> 8) & 0xff;
    ptr[7] = (tmp >> 0) & 0xff;
}

static void _floatx80_to_extended(floatx80 *f, uint8_t *ptr)
{
    ptr[0] = (f->high >> 8) & 0xff;
    ptr[1] = (f->high >> 0) & 0xff;
    ptr[2] = 0;
    ptr[3] = 0;
    ptr[4] = (f->low >> 56) & 0xff;
    ptr[5] = (f->low >> 48) & 0xff;
    ptr[6] = (f->low >> 40) & 0xff;
    ptr[7] = (f->low >> 32) & 0xff;
    ptr[8] = (f->low >> 24) & 0xff;
    ptr[9] = (f->low >> 16) & 0xff;
    ptr[10] = (f->low >> 8) & 0xff;
    ptr[11] = (f->low >> 0) & 0xff;
}

static float128 _int8_to_intermediate(int8_t byte)
{
    return int32_to_float128((int32_t)byte);
}

static float128 _int16_to_intermediate(int16_t sh)
{
    return int32_to_float128((int32_t)sh);
}

static float128 _int32_to_intermediate(int32_t in)
{
    return int32_to_float128(in);
}

static float128 _single_to_intermediate(uint32_t f)
{
    assert(sizeof(uint32_t) == sizeof(float32));
    return float32_to_float128((float32)f);
}

/*
 * _double_to_intermediate(d): d needs to be 68k-native order (8 bytes)
 */
static float128 _double_to_intermediate(uint8_t *d)
{
    assert(sizeof(uint64_t) == sizeof(float64));
    
    return float64_to_float128((float64) ntohll(*(uint64_t*)d));
}

/*
 * _extended_to_intermediate(e): e needs to be 68k-native order (12 bytes)
 */
static float128 _extended_to_intermediate(uint8_t *e)
{
    /*
     * softfloat floatx80 format:
     * uint64_t low; // the low part of the extended float (significand, low exponent bits)
     * uint16_t high; // the high part, sign, high exponent bits
     */
    floatx80 x80 = {
        .high = (e[0] << 8) | e[1],
        .low = ntohll(*(uint64_t*)&e[4])
    };
    return floatx80_to_float128(x80);
}

static void _extended_to_floatx80(uint8_t *bytes, floatx80 *f)
{
    f->high = (bytes[0] << 8) | bytes[1];
    f->low = ntohll(*(uint64_t*)&bytes[4]);
}

/*
 * Set softfloat's rounding mode
 * (fpcr.mc_rnd and softfloat use different values for these modes)
 */
static void _set_rounding_mode(enum rounding_mode_t mode)
{
    const int8_t rounding_map[4] = {
        float_round_nearest_even, float_round_to_zero,
        float_round_up, float_round_down
    };
    
    float_rounding_mode = rounding_map[mode];
}

#pragma mark EA routines

/*
 * Read-commit merely updates the address register
 * for pre/post-inc/decrement
 */
static void _fpu_read_ea_commit(const uint8_t format)
{
    ~decompose(shoe.op, 0000 0000 00 mmmrrr);
    
    if (m == 3) // post-increment
        shoe.a[r] += _format_sizes[format];
    else if (m == 4) // pre-decrement
        shoe.a[r] -= _format_sizes[format];
    
    /* 
     * Note: still unsure about what happens when
     *       mode=pre/postincdecrement, size==1, and register==a7
     *       (is the change +-2 bytes? or 1?)
     */
    if (((m == 3) || (m == 4)) && (_format_sizes[format] == 1) && (r == 7))
        assert(!"size==1, reg==a7");
}

static long double _floatx80_to_long_double(floatx80 f80);

static void _fpu_write_ea(uint8_t mr, uint8_t format, floatx80 *f, uint8_t K)
{
    fpu_get_state_ptr();
    
    const uint8_t m = mr >> 3;
    const uint8_t r = mr & 7;
    const uint8_t size = _format_sizes[format];
    uint8_t buf[12], *ptr = &buf[0];
    uint32_t addr, i;
    
    if ((m == 1) ||
        ((m == 0) && (size > 4))) {
        /* If mode==a-reg, or mode==data reg and the size is > 4 bytes, no dice */
        _throw_illegal_instruction();
        return ;
    }
    else if ((m == 7) && (r > 1)) {
        /* If this is otherwise an illegal addr mode... */
        _throw_illegal_instruction();
        return ;
    }
    
    const _Bool is_nan = ((f->high << 1) == 0xfffe) && f->low;
    
    {
        const long double tmp = _floatx80_to_long_double(*f);
        slog("FPU: fpu_write_ea EA=%u/%u data=%Lf format=%u\n", m, r, tmp, format);
    }
    
    /* Initialize softfloat's exceptions bits/rounding mode */
    
    float_exception_flags = 0;
    _set_rounding_mode(mc_rnd);
    
    /* Convert to the appropriate format */
    
    switch (format) {
        case format_B: {
            _floatx80_to_int8(f, ptr);
            break;
        }
        case format_W: {
            _floatx80_to_int16(f, ptr);
            break;
        }
        case format_L: {
            _floatx80_to_int32(f, ptr);
            break;
        }
        case format_S: {
            _floatx80_to_single(f, ptr);
            break;
        }
        case format_D: {
            _floatx80_to_double(f, ptr);
            break;
        }
        case format_X: {
            _floatx80_to_extended(f, ptr);
            break;
        }
        default: {
            assert(!"unsupported format (packed something!)");
        }
    }
    
    /* Write to memory */
    
    switch (m) {
        case 0: {
            if (format == format_B)
                set_d(r, ptr[0], 1);
            else if (format == format_W)
                set_d(r, ntohs(*(uint16_t*)ptr), 2);
            else if ((format == format_L) || (format == format_S))
                set_d(r, ntohl(*(uint32_t*)ptr), 4);
            else
                assert(!"how did I get here?");
            goto done;
        }
        case 1:
            assert(!"how did I get here again!");
            
        case 2:
            addr = shoe.a[r];
            break;
        case 3:
            addr = shoe.a[r];
            assert(!( r==7 && size==1));
            break;
        case 4: // pre-decrement
            addr = shoe.a[r] - size;
            assert(!( r==7 && size==1));
            break;
        default:
            call_ea_addr(mr);
            addr = (uint32_t)shoe.dat;
            break;
    }
    
    /* Copy the formatted data into *addr */
    
    /*{
        slog("FPU:  fpu_write_ea: addr=0x%08x data=", addr);
        for (i=0; i<size; i++)
            printf("%02x", ptr[i]);
        printf("\n");
    }*/
    
    for (i=0; i<size; i++) {
        lset(addr + i, 1, buf[i]);
        if (shoe.abort) return ;
    }
    
    
done:
    /* 
     * Set exception bits and update pre/post/blah registers.
     * note: condition codes are not modified
     */
    
    es_bsun = 0;
    es_snan = 0;
    es_operr = 0;
    es_ovfl = 0;
    es_unfl = 0;
    es_dz = 0;
    es_inex2 = 0;
    es_inex1 = 0;
    
    switch (format) {
        format_B:
        format_W:
        format_L:
            /* Set snan, operr, and/or inex2 */
            es_snan = is_nan;
            es_operr = ((float_exception_flags & float_flag_invalid) != 0);
            es_inex2 = ((float_exception_flags & float_flag_inexact) != 0);
            break;
        
        format_S:
        format_D:
        format_X:
            /* Set snan, ovfl, unfl, and/or inex2 */
            es_snan = is_nan;
            es_ovfl = ((float_exception_flags & float_flag_overflow) != 0);
            es_unfl = ((float_exception_flags & float_flag_underflow) != 0);
            es_inex2 = ((float_exception_flags & float_flag_inexact) != 0);
            break;
            
        format_Pd:
        format_Ps:
            /* Set snan, operr, and/or inex2 */
            assert(!"you better implement packed formats");
            break;
    }
    
    /* Update the accrued exception bits */
    ae_iop |= es_bsun | es_snan | es_operr;
    ae_ovfl |= es_ovfl;
    ae_unfl |= (es_unfl & es_inex2); // yes, &
    ae_dz |= es_dz;
    ae_inex |= es_inex1 | es_inex2 | es_ovfl;
    
    /* Are any exceptions both set and enabled? */
    if (fpu->fpsr.raw & fpu->fpcr.raw & 0x0000ff00) {
        /*
         * Then we need to throw an exception.
         * The exception is sent to the vector for
         * the highest priority exception, and the priority
         * order is (high->low) bsan, snan, operr, ovfl, unfl, dz, inex2/1
         * (which is the order of the bits in fpsr/fpcr).
         * Iterate over the bits in order, and throw the
         * exception to whichever bit is set first.
         */
        uint8_t i, throwable = (fpu->fpsr.raw & fpu->fpcr.raw) >> 8;
        
        assert(throwable);
        for (i=0; 1; i++) {
            if (throwable & 0x80)
                break;
            throwable <<= 1;
        }
        
        /*
         * Convert the exception bit position
         * to the correct vector number, and throw
         * a (pre-instruction) exception.
         */
        throw_fpu_pre_instruction_exception(_exception_bit_to_vector[i]);
        
        return ;
    }
    
    /* Finalize registers, and we're done */
    
    if (m == 3)
        shoe.a[r] += size;
    else if (m == 4)
        shoe.a[r] -= size;
}

/*
 * Note: fpu_read_ea modifies shoe.pc, and fpu_read_ea_commit
 *        modifies shoe.a[x] for pre/post-inc/decrement
 * Returns false if we're aborting
 */
static _Bool _fpu_read_ea(const uint8_t format, float128 *result)
{
    fpu_get_state_ptr();
    
    ~decompose(shoe.op, 0000 0000 00 mmmrrr);
    
    const uint8_t size = _format_sizes[format];
    uint32_t addr = 0;
    
    /*
     * Step 1: find the effective address, store it in addr
     *         (or the actual data, if unavailable)
     */
    
    slog("FPU: read_ea: mr=%u%u f=%c ", m, r, "lsxpwdb?"[format]);
    
    switch (m) {
        case 0:
            if (format == format_S)
                *result = _single_to_intermediate(shoe.d[r]);
            else if (format == format_B)
                *result = _int8_to_intermediate(shoe.d[r] & 0xff);
            else if (format == format_W)
                *result = _int16_to_intermediate(shoe.d[r] & 0xffff);
            else if (format == format_L)
                *result = int32_to_float128(shoe.d[r]);
            else {
                /*
                 * No other format can be used with a data register
                 * (because they require >4 bytes)
                 */
                _throw_illegal_instruction();
                return 0;
            }
            slog("raw=0x%x", chop(shoe.d[r], size));
            goto got_data;
            
        case 1:
            /* Address regisers can't be used */
            _throw_illegal_instruction();
            return 0;
        
        case 3:
            addr = shoe.a[r];
            assert(!( r==7 && size==1));
            goto got_address;
            
        case 4:
            addr = shoe.a[r] - size;
            assert(!( r==7 && size==1));
            goto got_address;
            
        case 7:
            if (r == 4) {
                addr = shoe.pc;
                shoe.pc += size;
                goto got_address;
            }
            
            // fall through to default:
            
        default: {
            ~decompose(shoe.op, 0000 0000 00 MMMMMM);
            shoe.mr = M;
            ea_addr();
            if (shoe.abort)
                return 0;
            
            addr = (uint32_t)shoe.dat;
            goto got_address;
        }

    }
    
got_address:
    
    /*
     * Step 2: Load the data from the effective address
     */
    
    //slog("raw=0x");
    if (size <= 4) {
        const uint32_t raw = lget(addr, size);
        if (shoe.abort)
            return 0;
        //printf("%x ", raw);
        switch (format) {
            case format_B:
                *result = _int8_to_intermediate(raw & 0xff);
                break;
            case format_W:
                *result = _int16_to_intermediate(raw & 0xffff);
                break;
            case format_L:
                *result = _int32_to_intermediate(raw);
                break;
            case format_S:
                *result = _single_to_intermediate(raw);
                break;
            default:
                assert(0); /* never get here */
        }
    }
    else { // if (size > 4) -> if format is double, extended, or packed
        uint8_t buf[12];
        uint32_t i;
        
        for (i = 0; i < size; i++) {
            buf[i] = lget(addr + i, 1);
            slog("%02x", buf[i]);
            if (shoe.abort)
                return 0;
        }
        
        switch (format) {
            case format_D:
                *result = _double_to_intermediate(buf);
                break;
            case format_X:
                *result = _extended_to_intermediate(buf);
                break;
            case format_Ps:
            // case format_Pd: // not possible as a src specifier
                // FIXME: implement packed formats
                assert(!"Somebody tried to use a packed format!\n");
                // _throw_illegal_instruction();
                // return 0;
            default:
                assert(0); // never get here
        }
    }
    
got_data:
    //printf("\n");
    return 1;
}

#pragma mark Hacky low-precision transcendental implementations
/*
 * s -> sign, e -> biased exponent
 * ma -> 48 high bits of the mantissa
 * mb -> 64 low bits of the mantissa
 */
#define _assemble_float128(s, e, ma, mb) ({ \
    const uint64_t _ma = (ma), _mb = (mb); \
    const uint64_t _e = (e), _s = (s); \
    float128 f = { \
        .high = ((_s != 0) << 16) | (_e & 0x7fff), \
        .low = _mb \
    }; \
    f.high = ((f.high) << 48) | _ma; \
    f; \
})

#define HACKY_MATH_X86
#ifdef HACKY_MATH_X86
#define NATIVE double

double _to_native(float128 f128)
{
    float64 f64 = float128_to_float64(f128);
    double result;
    uint8_t *ptr = (uint8_t*)&result;
    ptr[7] = (f64 >> 56) & 0xff;
    ptr[6] = (f64 >> 48) & 0xff;
    ptr[5] = (f64 >> 40) & 0xff;
    ptr[4] = (f64 >> 32) & 0xff;
    ptr[3] = (f64 >> 24) & 0xff;
    ptr[2] = (f64 >> 16) & 0xff;
    ptr[1] = (f64 >> 8) & 0xff;
    ptr[0] = (f64 >> 0) & 0xff;
    return result;
}

float128 _from_native(double n)
{
    float64 f64 = 0;
    uint8_t *ptr = (uint8_t*)&n;
    f64 = (f64 << 8) | ptr[7];
    f64 = (f64 << 8) | ptr[6];
    f64 = (f64 << 8) | ptr[5];
    f64 = (f64 << 8) | ptr[4];
    f64 = (f64 << 8) | ptr[3];
    f64 = (f64 << 8) | ptr[2];
    f64 = (f64 << 8) | ptr[1];
    f64 = (f64 << 8) | ptr[0];
    return float64_to_float128(f64);
}

#include <math.h>
#define _native_cos(a) cos(a)
#define _native_acos(a) acos(a)
#define _native_cosh(a) cosh(a)
#define _native_sin(a) sin(a)
#define _native_asin(a) asin(a)
#define _native_sinh(a) sinh(a)
#define _native_tan(a) tan(a)
#define _native_atan(a) atan(a)
#define _native_tanh(a) tanh(a)
#define _native_atanh(a) atanh(a)
#define _native_pow(a, b) pow((a), (b))
#define _native_exp(a) exp(a)
#define _native_expm1(a) (exp(a) - 1.0) /* or expm1() */
#define _native_log10(a) log10(a)
#define _native_log2(a) (log(a) / log(2.0)) /* or log2() */
#define _native_log(a) log(a)
#define _native_log1p(a) log((a) + 1.0) /* or log1p() */
double  _native_tentox(a) {
    /*
     * This is a dumb workaround for a clang bug on OS X 10.10
     * Clang wants to optimize pow(10.0, a) to __exp(a), but
     * __exp doesn't exist in the 10.8 SDK. So I'm using powl()
     * instead, which doesn't have an equivalent optimized function.
     */
    return (double)powl(10.0, (long double)a);
}

const double _native_e = 2.71828182845904509;
const double _native_10 = 10.0;
const double _native_2 = 2.0;
const double _native_1 = 1.0;

#elif (defined(HACKY_MATH_PPC))
#error "PowerPC hacky math isn't implemented yet"
#else
#error "You need to define HACKY_MATH_X86, or implement one for your arch"
#endif

static float128 _hack_cos (float128 x) {
    return _from_native(_native_cos(_to_native(x)));
}

static float128 _hack_acos (float128 x) {
    return _from_native(_native_acos(_to_native(x)));
}

static float128 _hack_cosh (float128 x) {
    return _from_native(_native_cosh(_to_native(x)));
}

static float128 _hack_sin (float128 x) {
    return _from_native(_native_sin(_to_native(x)));
}

static float128 _hack_asin (float128 x) {
    return _from_native(_native_asin(_to_native(x)));
}

static float128 _hack_sinh (float128 x) {
    return _from_native(_native_sinh(_to_native(x)));
}

static float128 _hack_tan (float128 x) {
    return _from_native(_native_tan(_to_native(x)));
}

static float128 _hack_atan (float128 x) {
    return _from_native(_native_atan(_to_native(x)));
}

static float128 _hack_tanh (float128 x) {
    return _from_native(_native_tanh(_to_native(x)));
}

static float128 _hack_atanh (float128 x) {
    return _from_native(_native_atanh(_to_native(x)));
}

static float128 _hack_etox (float128 x) {
    return _from_native(_native_exp(_to_native(x)));
}

static float128 _hack_etoxm1 (float128 x) {
    return _from_native(_native_expm1(_to_native(x)));
}

static float128 _hack_log10 (float128 x) {
    return _from_native(_native_log10(_to_native(x)));
}

static float128 _hack_log2 (float128 x) {
    return _from_native(_native_log2(_to_native(x)));
}

static float128 _hack_logn (float128 x) {
    return _from_native(_native_log(_to_native(x)));
}

static float128 _hack_lognp1 (float128 x) {
    return _from_native(_native_log1p(_to_native(x)));
}

static float128 _hack_tentox (float128 x) {
    return _from_native(_native_tentox(_to_native(x)));
}

static float128 _hack_twotox (float128 x) {
    return _from_native(_native_pow(_native_2, _to_native(x)));
}

#pragma mark FMATH! and all its helpers

/* Function prototypes from SoftFloat/softfloat-specialize.h */
char float128_is_nan(float128 a);
char float128_is_signaling_nan (float128 a);


static void inst_fmath_fmovecr (void)
{
    fpu_get_state_ptr();

    /*
     * FYI: these constants are stored in the "intermediate" 85-bit
     *      format in the 6888x rom. This has the side effect that
     *      they are rounded according to fpcr.mc_rnd.
     *      We emulate the intermediate 85-bit format with float128.
     */
    
    switch (fpu->fmath_op) {
        case 0x00: // pi
            fpu->result = _assemble_float128(0, 0x4000, 0x921fb54442d1, 0x8469898cc51701b8);
            break;
        case 0x0b: // log_10(2)
            fpu->result = _assemble_float128(0, 0x3ffd, 0x34413509f79f, 0xef311f12b35816f9);
            break;
        case 0x0c: // e
            fpu->result = _assemble_float128(0, 0x4000, 0x5bf0a8b14576, 0x95355fb8ac404e7a);
            break;
        case 0x0d: // log_2(e)
            fpu->result = _assemble_float128(0, 0x3fff, 0x71547652b82f, 0xe1777d0ffda0d23a);
            break;
        case 0x0e: // log_10(e)
            // NOTE: 68881 doesn't set inex2 for this one
            // Also note: that's bogus. 68881 uses 3 trailing mantissa bits to do rounding,
            // and those bits are non-zero for this number, so it must actually be stored
            // incorrectly in the ROM.
            // I'll emulate this by truncating the float128 mantissa.
            
            // fpu->result = _assemble_float128(0, 0x3ffd, 0xbcb7b1526e50, 0xe32a6ab7555f5a67);
            fpu->result = _assemble_float128(0, 0x3ffd, 0xbcb7b1526e50, 0xe32a000000000000);
            break;
        case 0x0f: // 0.0
            fpu->result = _assemble_float128(0, 0, 0, 0);
            break;
        case 0x30: // ln(2)
            fpu->result = _assemble_float128(0, 0x3ffe, 0x62e42fefa39e, 0xf35793c7673007e5);
            break;
        case 0x31: // ln(10)
            fpu->result = _assemble_float128(0, 0x4000, 0x26bb1bbb5551, 0x582dd4adac5705a6);
            break;
        case 0x32: // 1 (68kprm has typesetting issues everywhere. This one says 100, but means 10^0.)
            fpu->result = _assemble_float128(0, 0x3fff, 0x0, 0x0);
            break;
        case 0x33: // 10
            fpu->result = _assemble_float128(0, 0x4002, 0x400000000000, 0x0);
            break;
        case 0x34: // 10^2
            fpu->result = _assemble_float128(0, 0x4005, 0x900000000000, 0x0);
            break;
        case 0x35: // 10^4
            fpu->result = _assemble_float128(0, 0x400c, 0x388000000000, 0x0);
            break;
        case 0x36: // 10^8
            fpu->result = _assemble_float128(0, 0x4019, 0x7d7840000000, 0x0);
            break;
        case 0x37: // 10^16
            fpu->result = _assemble_float128(0, 0x4034, 0x1c37937e0800, 0x0);
            break;
        case 0x38: // 10^32
            fpu->result = _assemble_float128(0, 0x4069, 0x3b8b5b5056e1, 0x6b3be04000000000);
            break;
        case 0x39: // 10^64
            fpu->result = _assemble_float128(0, 0x40d3, 0x84f03e93ff9f, 0x4daa797ed6e38ed6);
            break;
        case 0x3a: // 10^128
            fpu->result = _assemble_float128(0, 0x41a8, 0x27748f9301d3, 0x19bf8cde66d86d62);
            break;
        case 0x3b: // 10^256
            fpu->result = _assemble_float128(0, 0x4351, 0x54fdd7f73bf3, 0xbd1bbb77203731fd);
            break;
        case 0x3c: // 10^512
            fpu->result = _assemble_float128(0, 0x46a3, 0xc633415d4c1d, 0x238d98cab8a978a0);
            break;
        case 0x3d: // 10^1024
            fpu->result = _assemble_float128(0, 0x4d48, 0x92eceb0d02ea, 0x182eca1a7a51e316);
            break;
        case 0x3e: // 10^2048
            fpu->result = _assemble_float128(0, 0x5a92, 0x3d1676bb8a7a, 0xbbc94e9a519c6535);
            break;
        case 0x3f: // 10^4096
            fpu->result = _assemble_float128(0, 0x7525, 0x88c0a4051441, 0x2f3592982a7f0094);
            break;
        default:
            /*
             * I wanted to include the actual values for the other ROM offsets,
             * but they might be proprietary. Most of them are 0 anyways, and some
             * cause FPU exceptions, even with all exceptions disabled... (?)
             * 68040 FPSP just returns 0, so we'll do that too.
             */
            fpu->result = _assemble_float128(0, 0, 0, 0);
            return ;
    }
}

/*
 * This is quick macro.pl macro to build a jump table
 * for fmath instructions. It is probably slightly slower 
 * to use a jump table rather than a big switch statement,
 * but I think it looks cleaner.
 */
~newmacro(create_fmath_jump_table, 0, {
    my $name_map = {};
    my $op_map = {};
    my $add = sub {
        my $op = shift;
        my $name = lc(shift);
        my $mode = 'foo';
        my $arch = 68881;
        
        foreach my $arg (@_) {
            if (($arg eq 'monadic') or ($arg eq 'dyadic')) {
                $mode = $arg;
            }
            elsif ($arg == 68040) {
                $arch = $arg;
            }
            else {
                croak("bad arg $arg");
            }
        }
        
        croak("didn't specify mode") if ($mode eq "foo");
        croak("dup $op $name") if exists $op_map->{$op};
        croak("bogus") if ($op > 127);
        
        $op_map->{$op} = {op => $op, name => $name, mode => $mode, arch => $arch};
        $name_map->{$name} = $op_map->{$op};
    };
    
    $add->(~b(1000000), 'fmove',    'monadic', 68040);
    $add->(~b(1000100), 'fmove',    'monadic', 68040);
    $add->(~b(0000000), 'fmove',    'monadic');
    
    $add->(~b(0000001), 'fint',     'monadic');
    $add->(~b(0000010), 'fsinh',    'monadic');
    $add->(~b(0000011), 'fintrz',   'monadic');
    
    $add->(~b(1000001), 'fsqrt',    'monadic', 68040);
    $add->(~b(1000101), 'fsqrt',    'monadic', 68040);
    $add->(~b(0000100), 'fsqrt',    'monadic');
    
    $add->(~b(0000110), 'flognp1',  'monadic');
    $add->(~b(0001000), 'fetoxm1',  'monadic');
    $add->(~b(0001001), 'ftanh',    'monadic');
    $add->(~b(0001010), 'fatan',    'monadic');
    $add->(~b(0001100), 'fasin',    'monadic');
    $add->(~b(0001101), 'fatanh',   'monadic');
    $add->(~b(0001110), 'fsin',     'monadic');
    $add->(~b(0001111), 'ftan',     'monadic');
    $add->(~b(0010000), 'fetox',    'monadic');
    $add->(~b(0010001), 'ftwotox',  'monadic');
    $add->(~b(0010010), 'ftentox',  'monadic');
    $add->(~b(0010100), 'flogn',    'monadic');
    $add->(~b(0010101), 'flog10',   'monadic');
    $add->(~b(0010110), 'flog2',    'monadic');
    
    $add->(~b(1011000), 'fabs',     'monadic', 68040);
    $add->(~b(1011100), 'fabs',     'monadic', 68040);
    $add->(~b(0011000), 'fabs',     'monadic');
    
    $add->(~b(0011001), 'fcosh',    'monadic');
    
    $add->(~b(1011010), 'fneg',     'monadic', 68040);
    $add->(~b(1011110), 'fneg',     'monadic', 68040);
    $add->(~b(0011010), 'fneg',     'monadic');
    
    $add->(~b(0011100), 'facos',    'monadic');
    $add->(~b(0011101), 'fcos',     'monadic');
    $add->(~b(0011110), 'fgetexp',  'monadic');
    $add->(~b(0011111), 'fgetman',  'monadic');
    
    $add->(~b(1100000), 'fdiv',     'dyadic', 68040);
    $add->(~b(1100100), 'fdiv',     'dyadic', 68040);
    $add->(~b(0100000), 'fdiv',     'dyadic');
    
    $add->(~b(0100001), 'fmod',     'dyadic');
    
    $add->(~b(1100010), 'fadd',     'dyadic', 68040);
    $add->(~b(1100110), 'fadd',     'dyadic', 68040);
    $add->(~b(0100010), 'fadd',     'dyadic');
    
    $add->(~b(1100011), 'fmul',     'dyadic', 68040);
    $add->(~b(1100111), 'fmul',     'dyadic', 68040);
    $add->(~b(0100011), 'fmul',     'dyadic');
    
    $add->(~b(0100100), 'fsgldiv',  'dyadic');
    $add->(~b(0100101), 'frem',     'dyadic');
    $add->(~b(0100110), 'fscale',   'dyadic');
    $add->(~b(0100111), 'fsglmul',  'dyadic');
    
    $add->(~b(1101000), 'fsub',     'dyadic', 68040);
    $add->(~b(1101100), 'fsub',     'dyadic', 68040);
    $add->(~b(0101000), 'fsub',     'dyadic');
    
    $add->(~b(0110000), 'fsincos',  'monadic');
    $add->(~b(0110001), 'fsincos',  'monadic');
    $add->(~b(0110010), 'fsincos',  'monadic');
    $add->(~b(0110011), 'fsincos',  'monadic');
    $add->(~b(0110100), 'fsincos',  'monadic');
    $add->(~b(0110101), 'fsincos',  'monadic');
    $add->(~b(0110110), 'fsincos',  'monadic');
    $add->(~b(0110111), 'fsincos',  'monadic');
    
    
    $add->(~b(0111000), 'fcmp',     'dyadic');
    $add->(~b(0111010), 'ftst',     'monadic');
    
    my $map_str = "fmath_impl_t *_fmath_map[128] = {\n";
    my @inst_flags = (0) x 128;
    
    for (my $i=0; $i < 128; $i++) {
        my $func_ptr = "NULL";
        if (exists $op_map->{$i}) {
            $func_ptr = 'inst_fmath_' . $op_map->{$i}->{name};
            if ($op_map->{$i}->{mode} eq 'dyadic') {
                $inst_flags[$i] |= 1;
            }
            if ($op_map->{$i}->{arch} == 68040) {
                $inst_flags[$i] |= 2;
            }
        }

        $map_str .= "\t" . $func_ptr . ",\n";
    }
    $map_str .= "};\n\nuint8_t _fmath_flags[128] = {\n";
    
    for (my $i=0; $i < 128; $i++) {
        $map_str .= "\t" . sprintf('0x%02x', $inst_flags[$i]) . ",\n";
    }
    $map_str .= "};\n";
    
    $map_str .= "const char *_fmath_names[128] = {\n";
    for (my $i=0; $i < 128; $i++) {
        my $name = "f???";
        if (exists $op_map->{$i}) {
            $name = $op_map->{$i}->{name};
        }
        $map_str .= "\t\"" . $name . "\",\n";
    }
    $map_str .= "};\n";
    
    return $map_str;
})

static _Bool _float128_is_zero (float128 f)
{
    return ((f.high << 1) == 0) && (f.low == 0);
}

static _Bool _float128_is_neg (float128 f)
{
    return f.high >> 63;
}

static _Bool _float128_is_infinity (float128 f)
{
    const uint64_t frac_a = f.high & 0x0000ffffffffffff;
    const uint64_t frac_b = f.low;
    const uint16_t exp = (f.high >> 48) & 0x7fff;
    
    return (exp == 0x7fff) && ((frac_a | frac_b) == 0);
}

static _Bool _float128_is_nan (float128 f)
{
    const uint64_t frac_a = f.high & 0x0000ffffffffffff;
    const uint64_t frac_b = f.low;
    const uint16_t exp = (f.high >> 48) & 0x7fff;
    
    return (exp == 0x7fff) && ((frac_a | frac_b) != 0);
}

const float128 _nan128 = {
    .high = 0xFFFF800000000000ULL,
    .low = 0
};

const float128 _one128 = {
    .high = 0x3fff000000000000ULL,
    .low = 0
};

const float128 _zero128 = {
    .high = 0,
    .low = 0
};

static void inst_fmath_fabs ()
{
    fpu_get_state_ptr();
    
    /* Clear the sign bit */
    fpu->result = fpu->source;
    fpu->result.high <<= 1;
    fpu->result.high >>= 1;
}

static void inst_fmath_facos ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* Find the absolute value of source */
    float128 tmp = fpu->source;
    tmp.high <<= 1;
    tmp.high >>= 1;
    
    /* If source is zero, result is +pi/2 */
    if (source_zero) {
        fpu->result = _assemble_float128(0, 0x3fff, 0x921fb54442d1, 0x8469898cc51701b8);
        return;
    }
    /* If source isn't in range [-1, 1], return nan, set operr */
    else if (!float128_le(tmp, _one128)) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    
    fpu->result = _hack_acos(fpu->source);
    /* Set inex2?? */
}

static void inst_fmath_fadd ()
{
    fpu_get_state_ptr();
    
    fpu->result = float128_add(fpu->dest, fpu->source);
    
    /* 
     * Throw operr (and return NaN) if operands are infinities
     * with opposite signs. (I *think* softfloat is doing this
     * corectly - the code's hard to read.)
     */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
    /* Throw ovfl if the op overflowed */
    if (float_exception_flags & float_flag_overflow)
        es_ovfl = 1;
    
    /* Throw unfl if the op overflowed */
    if (float_exception_flags & float_flag_underflow)
        es_unfl = 1;
}

static void inst_fmath_fasin ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* Find the absolute value of source */
    float128 tmp = fpu->source;
    tmp.high <<= 1;
    tmp.high >>= 1;
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If source isn't in range [-1, 1], return nan, set operr */
    else if (!float128_le(tmp, _one128)) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    
    fpu->result = _hack_asin(fpu->source);
    /* Set inex2?? */
    /* Set unfl?? */
}

static void inst_fmath_fatan ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If source is inf, result is +-pi/2 */
    else if (source_inf) {
        fpu->result = _assemble_float128(source_sign, 0x3fff, 0x921fb54442d1, 0x8469898cc51701b8);
        return ;
    }
    
    fpu->result = _hack_atan(fpu->source);
    /* Set inex2?? */
    /* Set unfl?? */
}

static void inst_fmath_fatanh ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* Take the absolute value of source */
    float128 tmp = fpu->source;
    tmp.high <<= 1;
    tmp.high >>= 1;
    
    /* If source is 0, return source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If |source| == 1.0, set dz, return +-inf */
    else if (float128_eq(tmp, _one128)) {
        es_dz = 1;
        fpu->result = _assemble_float128(source_sign, 0x7fff, 0, 0);
        return;
    }
    /* If |source| > 1.0, set operr, return nan */
    else if (!float128_le(tmp, _one128)) {
        es_operr = 1;
        fpu->result = _nan128;
        return ;
    }
    
    fpu->result = _hack_atanh(fpu->source);
}

static void inst_fmath_fcmp ()
{
    fpu_get_state_ptr();
    const uint8_t source_zero = _float128_is_zero(fpu->source);
    const uint8_t source_inf = _float128_is_infinity(fpu->source);
    const uint8_t source_sign = _float128_is_neg(fpu->source);
    const uint8_t dest_zero = _float128_is_zero(fpu->dest);
    const uint8_t dest_inf = _float128_is_infinity(fpu->dest);
    const uint8_t dest_sign = _float128_is_neg(fpu->dest);
    
    /* If the operands are "in-range", then we actually need to compare them */
    if (!(source_zero | source_inf | dest_zero | dest_inf)) {
        // dest - source.
        // dest == zource -> Z
        // dest < source -> N
        if (float128_eq(fpu->dest, fpu->source))
            cc_z = 1;
        else if (float128_le(fpu->dest, fpu->source))
            cc_n = 1;
    }
    else {
        const uint8_t offset = ((source_zero << 5) | (source_inf << 4) | (source_sign << 3) |
                                (dest_zero << 2) | (dest_inf << 1) | (dest_sign << 0));
        /* Precomputed answers for all the possible combinations */
        cc_z = (0x0000303008040201ULL >> offset) & 1;
        cc_n = (0x00002a2a083b0a3bULL >> offset) & 1;
        
        slog("FPU: fcmp: hit uncommon case: offset=0x%x z=%u n=%u\n", offset, cc_z, cc_n);
    }
    
    /* Don't write the result back to the register */
    fpu->write_back = 0;
}

static void inst_fmath_fcos ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* If source is zero, result is +1.0 */
    if (source_zero) {
        fpu->result = _one128;
        return;
    }
    /* If source is inf, result is nan, and set operr */
    else if (source_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    
    fpu->result = _hack_cos(fpu->source);
    /* Set inex2?? */
}

static void inst_fmath_fcosh ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* If source is zero, result is +1.0 */
    if (source_zero) {
        fpu->result = _one128;
        return;
    }
    /* If source is +/- inf, result is +inf */
    else if (source_inf) {
        fpu->result = _assemble_float128(0, 0x7fff, 0, 0);
        return;
    }
    
    fpu->result = _hack_cosh(fpu->source);
}

static void inst_fmath_fdiv ()
{
    fpu_get_state_ptr();
    
    fpu->result = float128_div(fpu->dest, fpu->source);
    
    /* Throw operr (and return NaN) if both operands are zero */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw divide-by-zero if dividend is zero */
    if (float_exception_flags & float_flag_divbyzero)
        es_dz = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
    /* Throw ovfl if the op overflowed */
    if (float_exception_flags & float_flag_overflow)
        es_ovfl = 1;
    
    /* Throw unfl if the op overflowed */
    if (float_exception_flags & float_flag_underflow)
        es_unfl = 1;
}

static void inst_fmath_fetox ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, result is +1.0 */
    if (source_zero) {
        fpu->result = _one128;
        return ;
    }
    /* if source is -inf, result is +0.0 */
    else if (source_inf && source_sign) {
        fpu->result = _zero128;
        return ;
    }
    /* if source is +inf, result is +inf */
    else if (source_inf) {
        fpu->result = fpu->source;
        return ;
    }
    
    fpu->result = _hack_etox(fpu->source);
}

static void inst_fmath_fetoxm1 ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    const float128 negone = _assemble_float128(1, 0x3fff, 0, 0);
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return ;
    }
    /* if source is -inf, result is +0.0 */
    else if (source_inf && source_sign) {
        fpu->result = negone;
        return ;
    }
    /* if source is +inf, result is +inf */
    else if (source_inf) {
        fpu->result = fpu->source;
        return ;
    }
    
    fpu->result = _hack_etoxm1(fpu->source);
}

static void inst_fmath_fgetexp ()
{
    fpu_get_state_ptr();
    
    /* If source is INF, set operr and return NaN */
    if (((fpu->source.high << 1) == 0xfffe000000000000ULL) && (fpu->source.low == 0)) {
        es_operr = 1;
        fpu->result.high = 0xffff000000000000ULL;
        fpu->result.low = 0xc000000000000000ULL;
        return ;
    }
    
    /*
     * If source is 0, return source.
     * According to 68881 docs, the result needs to have 
     * the same sign as the source (why?)
     */
    if (((fpu->source.high << 1) == 0) && (fpu->source.low == 0)) {
        fpu->result = fpu->source;
        return ;
    }
    
    /*
     * Otherwise, extract the biased exponent, convert it
     * to a two's complement integer, and store that value
     * as a float.
     */
    const uint32_t biased = (fpu->source.high << 1) >> 49;
    fpu->result = int32_to_float128(((int32_t)biased) - 16383);
}

static void inst_fmath_fgetman ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* If source is inf, set operr, result is nan */
    if (source_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return;
    }
    /* If source is zero, result is source */
    else if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    
    /*
     * fgetman "returns" the value of the source's mantissa,
     * which I think just means it resets the exponent to 0x3fff (biased 0)
     * FIXME: test this
     */
    fpu->result = fpu->source;
    fpu->result.high &= 0x8000ffffffffffffULL;
    fpu->result.high |= 0x3fff000000000000ULL;
}

static void inst_fmath_fint ()
{
    fpu_get_state_ptr();
    
    fpu->result = float128_round_to_int(fpu->source);
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
}

static void inst_fmath_fintrz ()
{
    fpu_get_state_ptr();
    
    /* Same as fint, but force the round-to-zero mode */
    
    const signed char old_round_mode = float_rounding_mode;
    float_rounding_mode = float_round_to_zero;
    fpu->result = float128_round_to_int(fpu->source);
    float_rounding_mode = old_round_mode;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
}

static void inst_fmath_flog10 ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, set dz, result is -inf */
    if (source_zero) {
        fpu->result = _assemble_float128(1, 0x7fff, 0, 0);
        es_dz = 1;
        return;
    }
    /* If source is negative, set operr, result is nan */
    else if (source_sign) {
        fpu->result = _nan128;
        es_operr = 1;
        return;
    }
    /* If source is +inf, result is +inf. */
    else if (source_inf) {
        fpu->result = fpu->source;
        return;
    }
    
    fpu->result = _hack_log10(fpu->source);
}

static void inst_fmath_flog2 ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, set dz, result is -inf */
    if (source_zero) {
        fpu->result = _assemble_float128(1, 0x7fff, 0, 0);
        es_dz = 1;
        return;
    }
    /* If source is negative, set operr, result is nan */
    else if (source_sign) {
        fpu->result = _nan128;
        es_operr = 1;
        return;
    }
    /* If source is +inf, result is +inf. */
    else if (source_inf) {
        fpu->result = fpu->source;
        return;
    }
    
    fpu->result = _hack_log2(fpu->source);
}

static void inst_fmath_flognp1 ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    const float128 negone = _assemble_float128(1, 0x3fff, 0, 0);
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If source is -1.0, set dz, result is -inf */
    else if (float128_eq(negone, fpu->source)) {
        es_dz = 1;
        fpu->result = _assemble_float128(1, 0x7fff, 0, 0);
        return;
    }
    /* If source < -1.0, set operr, result is nan */
    else if (float128_lt(fpu->source, negone)) {
        es_operr = 1;
        fpu->result = _nan128;
        return;
    }
    /* If source is +inf, result is +inf. */
    else if (source_inf) {
        fpu->result = fpu->source;
        return;
    }
    
    fpu->result = _hack_lognp1(fpu->source);
}

static void inst_fmath_flogn ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, set dz, result is -inf */
    if (source_zero) {
        fpu->result = _assemble_float128(1, 0x7fff, 0, 0);
        es_dz = 1;
        return;
    }
    /* If source is negative, set operr, result is nan */
    else if (source_sign) {
        fpu->result = _nan128;
        es_operr = 1;
        return;
    }
    /* If source is +inf, result is +inf. */
    else if (source_inf) {
        fpu->result = fpu->source;
        return;
    }
    
    fpu->result = _hack_logn(fpu->source);
}

static void inst_fmath_fmove ()
{
    fpu_get_state_ptr();
    
    fpu->result = fpu->source;
}

static void inst_fmath_fmul ()
{
    fpu_get_state_ptr();
    
    fpu->result = float128_mul(fpu->dest, fpu->source);
    
    /*
     * Throw operr (and return NaN) if one operand is infinity
     * and the other is zero.
     */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
    /* Throw ovfl if the op overflowed */
    if (float_exception_flags & float_flag_overflow)
        es_ovfl = 1;
    
    /* Throw unfl if the op overflowed */
    if (float_exception_flags & float_flag_underflow)
        es_unfl = 1;
}

static void inst_fmath_fneg ()
{
    fpu_get_state_ptr();
    
    /* Flip the sign bit */
    fpu->result = fpu->source;
    fpu->result.high ^= (1ULL << 63);
    
    /* 
     * FIXME: you're supposed to throw UNFL if this is a
     *        denormalized number, I think.
     */
}

static void inst_fmath_frem ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool dest_zero = _float128_is_zero(fpu->dest);
    const _Bool dest_inf = _float128_is_infinity(fpu->dest);
    
    /* I just assume the quotient/sign are 0 for the following cases */
    qu_quotient = 0;
    qu_s = 0;
    
    /* If source is zero, result is nan */
    if (source_zero) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    /* If dest (but not source) is zero, result is that zero */
    else if (dest_zero) {
        fpu->result = fpu->dest;
        return ;
    }
    /* If dest is infinity, result is nan */
    else if (dest_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    /* If source, but not dest, is infinity, result is dest */
    else if (source_inf) {
        fpu->result = fpu->dest;
        return ;
    }
    
    /* -- We're past the edge cases, do the actual op -- */
    
    const signed char old_round_mode = float_rounding_mode;
    
    /* frem uses round-to-nearest */
    float_rounding_mode = float_round_nearest_even;
    
    float128 N = float128_div(fpu->dest, fpu->source);
    N = float128_round_to_int(N);
    
    float_rounding_mode = old_round_mode;
    
    fpu->result = float128_sub(fpu->dest, float128_mul(fpu->source, N));
    
    /* FIXME: not sure how to set unfl reliably */
    
    _Bool sign = N.high >> 63; /* Remember the sign */
    N.high <<= 1; /* Clear the sign */
    N.high >>= 1;
    uint32_t final = float128_to_int32(N); /* Get the integer of the quotient */
    qu_quotient = final & 0x7f;
    qu_s = sign;
}

static void inst_fmath_fmod ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool dest_zero = _float128_is_zero(fpu->dest);
    const _Bool dest_inf = _float128_is_infinity(fpu->dest);
    
    /* I just assume the quotient/sign are 0 for the following cases */
    qu_quotient = 0;
    qu_s = 0;
    
    /* If source is zero, result is nan */
    if (source_zero) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    /* If dest (but not source) is zero, result is that zero */
    else if (dest_zero) {
        fpu->result = fpu->dest;
        return ;
    }
    /* If dest is infinity, result is nan */
    else if (dest_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    /* If source, but not dest, is infinity, result is dest */
    else if (source_inf) {
        fpu->result = fpu->dest;
        return ;
    }
    
    /* -- We're past the edge cases, do the actual op -- */
    
    const signed char old_round_mode = float_rounding_mode;
    
    /* fmod uses round-to-zero */
    float_rounding_mode = float_round_to_zero;
    
    float128 N = float128_div(fpu->dest, fpu->source);
    N = float128_round_to_int(N);
    
    float_rounding_mode = old_round_mode;
    
    fpu->result = float128_sub(fpu->dest, float128_mul(fpu->source, N));
    
    /* FIXME: not sure how to set unfl reliably */
    
    _Bool sign = N.high >> 63; /* Remember the sign */
    N.high <<= 1; /* Clear the sign */
    N.high >>= 1;
    uint32_t final = float128_to_int32(N); /* Get the integer of the quotient */
    qu_quotient = final & 0x7f;
    qu_s = sign;
}

static void inst_fmath_fscale ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    const _Bool dest_zero = _float128_is_zero(fpu->dest);
    const _Bool dest_inf = _float128_is_infinity(fpu->dest);
    
    int32_t factor, orig_exponent, exponent;
    
    /* If the source is inf, the result is nan and set operr */
    if (source_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    /* Else, if dest is inf or zero, the result is dest */
    else if (dest_inf || dest_zero) {
        fpu->result = fpu->dest;
        return ;
    }
    
    /* 
     * If the source has a huge magnitude, it's definitely
     * going to over/underflow
     */
    if (float128_le(int32_to_float128(65536), fpu->source))
        goto overflow;
    else if (float128_le(fpu->source, int32_to_float128(-65536)))
        goto underflow;

    /* Find the scaling factor. This shoudn't raise any exceptions */
    factor = float128_to_int32_round_to_zero(fpu->source);
    
    assert(float_exception_flags == 0);
    
    orig_exponent = (int32_t)((fpu->dest.high >> 48) & 0x7fff);
    exponent = factor + orig_exponent;
    
    if (exponent < 0)
        goto underflow;
    else if (exponent >= 0x7fff)
        goto overflow;
    else if ((exponent == 0) && (orig_exponent > 0)) {
        uint64_t m_high;
        /* 
         * Edge case: if the 80-bit input was {exp=1, mantissa=1},
         * the 128-bit version will be {exp=1, mantissa=0}.
         * If we're subtracting 1, the result will be {exp=0, mantissa=0}
         * which is not correct. We need to account for the implicit bit.
         */
        fpu->result = fpu->dest;
        fpu->result.low >>= 1;
        fpu->result.low |= (fpu->result.high << 63);
        m_high = (fpu->result.high & 0x0000ffffffffffffULL) >> 1;
        fpu->result.high &= 0x8000000000000000ULL;
        fpu->result.high |= m_high;
    }
    else if ((orig_exponent == 0) && (exponent > 0)) {
        uint32_t i;
        /*
         * Edge case 2: if the original value was subnormal, and we're
         * adding to the exponent, then the result may normal or subnormal,
         * and we need to adjust the implicit bit accordingly.
         */
        
        fpu->result = fpu->dest;
        float128 _two128 = int32_to_float128(2);
        for (i=0; i < exponent; i++)
            fpu->result = float128_mul(fpu->result, _two128);
        /* 
         * FIXME: this is a really bad implementation, but I doubt anyone
         * will ever hit this condition. It's also probably not exactly
         * correct. The motorola docs say that 68881 will never return an
         * unnormal number as the result of an operation, but if you add
         * an arbitrary value to the exponent of a subnormal number, you
         * can get an unnormal number. Does the 68881 specially handle this?
         */
    }
    else {
        /* Common case */
        fpu->result = fpu->dest;
        fpu->result.high &= 0x8000ffffffffffffULL;
        fpu->result.high |= (((uint64_t)exponent) << 48);
    }
    return ;
    
overflow:
    /* result is +-inf, set overflow */
    fpu->result = _assemble_float128(source_sign, 0x7fff, 0, 0);
    es_ovfl = 1;
    return ;
underflow:
    /* result is +-zero, set underflow */
    fpu->result = _assemble_float128(source_sign, 0, 0, 0);
    es_unfl = 1;
    return ;
}

static void inst_fmath_fsgldiv ()
{
    fpu_get_state_ptr();
    
    float128 source = fpu->source;
    float128 dest = fpu->dest;
    
    /* Dump the low 88 bits of the source/dest mantissas */
    source.low = 0;
    source.high &= 0xffffffffff000000;
    dest.low = 0;
    dest.high &= 0xffffffffff000000;
    
    fpu->result = float128_div(dest, source);
    
    /* Throw operr (and return NaN) if both operands are zero */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw divide-by-zero if dividend is zero */
    if (float_exception_flags & float_flag_divbyzero)
        es_dz = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
    /* Throw ovfl if the op overflowed */
    if (float_exception_flags & float_flag_overflow)
        es_ovfl = 1;
    
    /* Throw unfl if the op overflowed */
    if (float_exception_flags & float_flag_underflow)
        es_unfl = 1;
}

static void inst_fmath_fsglmul ()
{
    fpu_get_state_ptr();
    
    /*
     * As far as I can tell, fsglmul/fsgldiv use an ALU
     * for the mantissa that is only 24-bits wide. Everything
     * else is done with regular internal precision.
     */
    
    float128 source = fpu->source;
    float128 dest = fpu->dest;
    
    /* Dump the low 88 bits of the source/dest mantissas */
    source.low = 0;
    source.high &= 0xffffffffff000000;
    dest.low = 0;
    dest.high &= 0xffffffffff000000;
    
    fpu->result = float128_mul(dest, source);
    
    /*
     * Throw operr (and return NaN) if one operand is infinity
     * and the other is zero.
     */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
    /* Throw ovfl if the op overflowed */
    if (float_exception_flags & float_flag_overflow)
        es_ovfl = 1;
    
    /* Throw unfl if the op overflowed */
    if (float_exception_flags & float_flag_underflow)
        es_unfl = 1;
}

static void inst_fmath_fsin ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If source is inf, result is nan, and set operr */
    else if (source_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    
    fpu->result = _hack_sin(fpu->source);
    /* Set inex2?? */
}

static void inst_fmath_fsincos ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const uint16_t cos_reg = fpu->extension_word & 0x7;
    float128 cos_result;
    
    /* 
     * This is incredibly inefficient, but it's fine for now
     * floatx80 -> float128 -> float64 -> native -> sin -> float128 -> floatx80
     * floatx80 -> float128 -> float64 -> native -> cos -> float128 -> floatx80
     */
    
    /* If source is inf, result is nan, and set operr */
    if (source_inf) {
        fpu->result = _nan128;
        cos_result = _nan128;
        es_operr = 1;
        goto write_cos;
    }
    /* If the source is zero, cos is +1.0, sin is +-0.0 */
    else if (source_zero) {
        fpu->result = fpu->source;
        cos_result = _one128;
        goto write_cos;
    }
    
    fpu->result = _hack_sin(fpu->source);
    cos_result = _hack_cos(fpu->source);
    
write_cos:
    
    /* FIXME: 68kprm doesn't clarify how the cosine result is rounded */
    fpu->fp[cos_reg] = float128_to_floatx80(cos_result);
}

static void inst_fmath_fsinh ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* If source is zero or inf, return source */
    if (source_zero || source_inf) {
        fpu->result = fpu->source;
        return;
    }
    
    fpu->result = _hack_sinh(fpu->source);
}

static void inst_fmath_fsqrt ()
{
    fpu_get_state_ptr();
    
    fpu->result = float128_sqrt(fpu->source);
    
    /* Throw operr (and return NaN) if the operand is < 0 */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
}

static void inst_fmath_fsub ()
{
    fpu_get_state_ptr();
    
    fpu->result = float128_sub(fpu->dest, fpu->source);
    
    /*
     * Throw operr (and return NaN) if operands are infinities
     * with equal signs. (I *think* softfloat is doing this
     * corectly - the code's hard to read.)
     *
     * Both 68kprm and 68881 docs say that (+inf) - (-inf) = (-inf)
     * but I presume that's a typo, and it's supposed to be (+inf)
     */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    /* Throw inex2 if the result is inexact */
    if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
    
    /* Throw ovfl if the op overflowed */
    if (float_exception_flags & float_flag_overflow)
        es_ovfl = 1;
    
    /* Throw unfl if the op overflowed */
    if (float_exception_flags & float_flag_underflow)
        es_unfl = 1;
}

static void inst_fmath_ftan ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If source is inf, result is nan, and set operr */
    else if (source_inf) {
        fpu->result = _nan128;
        es_operr = 1;
        return ;
    }
    
    fpu->result = _hack_tan(fpu->source);
    /* Set inex2?? */
}

static void inst_fmath_ftanh ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, result is source */
    if (source_zero) {
        fpu->result = fpu->source;
        return;
    }
    /* If source is +/- inf, result is +/- 1.0 */
    else if (source_inf) {
        fpu->result = _assemble_float128(source_sign, 0x3fff, 0, 0);
        return;
    }
    
    fpu->result = _hack_tanh(fpu->source);
}

static void inst_fmath_ftentox ()
{
    fpu_get_state_ptr();
    
    // _hack_ftentox() is broken on clang 3.5 on osx 10.10
    // (tries to optimize pow(10.0, x) to __exp10(x), and __exp10
    //  isn't implemented in the 10.8 SDK)
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, result is +1.0 */
    if (source_zero) {
        fpu->result = _one128;
        return ;
    }
    /* if source is -inf, result is +0.0 */
    else if (source_inf && source_sign) {
        fpu->result = _zero128;
        return ;
    }
    /* if source is +inf, result is +inf */
    else if (source_inf) {
        fpu->result = fpu->source;
        return ;
    }
    
    fpu->result = _hack_tentox(fpu->source);
}

static void inst_fmath_ftst ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    cc_z = source_zero;
    cc_i = source_inf;
    cc_n = source_sign;
    
    /* Don't write the result back to the register */
    fpu->write_back = 0;
}

static void inst_fmath_ftwotox ()
{
    fpu_get_state_ptr();
    
    const _Bool source_zero = _float128_is_zero(fpu->source);
    const _Bool source_inf = _float128_is_infinity(fpu->source);
    const _Bool source_sign = _float128_is_neg(fpu->source);
    
    /* If source is zero, result is +1.0 */
    if (source_zero) {
        fpu->result = _one128;
        return ;
    }
    /* if source is -inf, result is +0.0 */
    else if (source_inf && source_sign) {
        fpu->result = _zero128;
        return ;
    }
    /* if source is +inf, result is +inf */
    else if (source_inf) {
        fpu->result = fpu->source;
        return ;
    }
    
    fpu->result = _hack_twotox(fpu->source);
}

typedef void (fmath_impl_t)(void);
#define FMATH_TYPE_DYADIC 1
#define FMATH_TYPE_68040 2
~create_fmath_jump_table()


/*
 * Take fpu->result, and round and crop it to the
 * preferred precision, then return the result as
 * a floatx80. (Set all the appropriate exception bits
 * too)
 *
 * ALSO!! This checks and sets underflow/overflow
 */
static floatx80 _fmath_round_intermediate_result ()
{
    fpu_get_state_ptr();
    floatx80 final;
    
    float_exception_flags = 0; // (so we can know if the result is inexact)
    _set_rounding_mode(mc_rnd); // Set the preferred rounding mode
    
    if (mc_prec == prec_extended) { // extended precision
        final = float128_to_floatx80(fpu->result);
        es_inex2 |= ((float_exception_flags & float_flag_inexact) != 0);
        es_unfl |= ((float_exception_flags & float_flag_underflow) != 0);
        es_ovfl |= ((float_exception_flags & float_flag_overflow) != 0);
    }
    else if (mc_prec == prec_double) { // double precision
        float64 tmp = float128_to_float64(fpu->result);
        es_inex2 |= ((float_exception_flags & float_flag_inexact) != 0);
        es_unfl |= ((float_exception_flags & float_flag_underflow) != 0);
        es_ovfl |= ((float_exception_flags & float_flag_overflow) != 0);
        final = float64_to_floatx80(tmp);
    }
    else if (mc_prec == prec_single) { // single precision
        float32 tmp = float128_to_float32(fpu->result);
        es_inex2 |= ((float_exception_flags & float_flag_inexact) != 0);
        es_unfl |= ((float_exception_flags & float_flag_underflow) != 0);
        es_ovfl |= ((float_exception_flags & float_flag_overflow) != 0);
        final = float32_to_floatx80(tmp);
    }
    else
        assert(!"bogus precision mode???");
    
    return final;
}

static void _fmath_set_condition_codes (floatx80 val)
{
    fpu_get_state_ptr();
    const uint64_t frac = val.low;
    const uint32_t exp = val.high & 0x7fff;
    const _Bool sign = val.high >> 15;
    
    /* Check for zero */
    cc_z = ((exp == 0) && (frac == 0));
    
    /* Check for negative */
    cc_n = sign;
    
    /* Check for NaN */
    cc_nan = ((exp == 0x7fff) && ((frac << 1) != 0));
    
    /* Check for infinity */
    cc_i = ((exp == 0x7fff) && ((frac << 1) == 0));
}

static void _fmath_handle_nans ()
{
    fpu_get_state_ptr();
    
    const _Bool is_dyadic = _fmath_flags[fpu->fmath_op] & FMATH_TYPE_DYADIC;
    const _Bool is_signaling = float128_is_signaling_nan(fpu->source) ||
                                (is_dyadic && float128_is_signaling_nan(fpu->dest));
    const _Bool is_source_nan = float128_is_nan(fpu->source);
    const _Bool is_dest_nan = is_dyadic && float128_is_nan(fpu->dest);
    
    /*
     * If the dest is NaN, or both are NaN, let the result be set to dest.
     * (with signaling disabled)
     */
    if (is_dest_nan)
        fpu->result = fpu->dest;
    else {
        assert(is_source_nan);
        fpu->result = fpu->source;
    }
    
    /* Set the snan exception status bit */
    es_snan = is_signaling;
    
    /* Silence the result */
    // Signaling -> 0
    // Non-signaling -> 1
    fpu->result.high |= 0x800000000000;
}

void dis_fmath (uint16_t op, uint16_t ext, char *output)
{
    ~decompose(op, 1111 001 000 MMMMMM);
    ~decompose(ext, 0 a 0 sss ddd eeeeeee);
    
    const uint8_t src_in_ea = a;
    const uint8_t source_specifier = s;
    const uint8_t dest_register = d;
    const uint8_t extension = e;
    
    /* If this is fmovecr */
    if (src_in_ea && (source_specifier == 7)) {
        const char *name = NULL;
        switch (extension) {
            case 0x00: name = "pi"; break;
            case 0x0b: name = "log_10(2)"; break;
            case 0x0c: name = "c"; break;
            case 0x0d: name = "log_2(e)"; break;
            case 0x0e: name = "log_10(e)"; break;
            case 0x0f: name = "0.0"; break;
            case 0x30: name = "ln(2)"; break;
            case 0x31: name = "ln(10)"; break;
            case 0x32: name = "1.0"; break;
            case 0x33: name = "10.0"; break;
            case 0x34: name = "10.0^2"; break;
            case 0x35: name = "10.0^4"; break;
            case 0x36: name = "10.0^8"; break;
            case 0x37: name = "10.0^16"; break;
            case 0x38: name = "10.0^32"; break;
            case 0x39: name = "10.0^64"; break;
            case 0x3a: name = "10.0^128"; break;
            case 0x3b: name = "10.0^256"; break;
            case 0x3c: name = "10.0^512"; break;
            case 0x3d: name = "10.0^1024"; break;
            case 0x3e: name = "10.0^2048"; break;
            case 0x3f: name = "10.0^4096"; break;
        }
        if (name == NULL)
            sprintf(output, "fmovecr.x #%u,fp%u", extension, dest_register);
        else
            sprintf(output, "fmovecr.x %s,fp%u", name, dest_register);
        return;
    }
    
    if (_fmath_map[e] == NULL) {
            /* This instruction isn't defined */
            sprintf(output, "fmath???");
    }
    else if (_fmath_map[e] == inst_fmath_fsincos) {
        /* fsincos.<fmt> <ea>,FPc:FPs */
        if (src_in_ea)
            sprintf(output, "fsincos.%c %s,fp%u:fp%u", "lsxpwdb?"[source_specifier],
                    decode_ea_rw(M, _format_sizes[source_specifier]), e & 7, dest_register);
        else
            sprintf(output, "fsincos.x fp%u,fp%u:fp%u", source_specifier, e & 7, dest_register);
    }
    else if (_fmath_map[e] == inst_fmath_ftst) {
        /* ftst.<fmt> <source> */
        if (src_in_ea)
            sprintf(output, "ftst.%c %s", "lsxpwdb?"[source_specifier],
                    decode_ea_rw(M, _format_sizes[source_specifier]));
        else
            sprintf(output, "ftst.x fp%u", dest_register);
    }
    else {
        /* f<inst>.<fmt> <source>,<dest> */
        if (src_in_ea)
            sprintf(output, "%s.%c %s,fp%u", _fmath_names[e], "lsxpwdb?"[source_specifier],
                    decode_ea_rw(M, _format_sizes[source_specifier]), dest_register);
        else
            sprintf(output, "%s.x fp%u,fp%u", _fmath_names[e],
                    source_specifier, dest_register);
    }
}

static long double _float128_to_long_double(float128 f128)
{
    long double result;
    uint8_t *ptr = (uint8_t*)&result;
    
    int8_t old = float_exception_flags;
    floatx80 f80 = float128_to_floatx80(f128);
    float_exception_flags = old;
    
    ptr[9] = (f80.high >> 8) & 0xff;
    ptr[8] = (f80.high >> 0) & 0xff;
    ptr[7] = (f80.low >> 56) & 0xff;
    ptr[6] = (f80.low >> 48) & 0xff;
    ptr[5] = (f80.low >> 40) & 0xff;
    ptr[4] = (f80.low >> 32) & 0xff;
    ptr[3] = (f80.low >> 24) & 0xff;
    ptr[2] = (f80.low >> 16) & 0xff;
    ptr[1] = (f80.low >> 8) & 0xff;
    ptr[0] = (f80.low >> 0) & 0xff;
    
    return result;
}

static long double _floatx80_to_long_double(floatx80 f80)
{
    long double result;
    uint8_t *ptr = (uint8_t*)&result;
    
    ptr[9] = (f80.high >> 8) & 0xff;
    ptr[8] = (f80.high >> 0) & 0xff;
    ptr[7] = (f80.low >> 56) & 0xff;
    ptr[6] = (f80.low >> 48) & 0xff;
    ptr[5] = (f80.low >> 40) & 0xff;
    ptr[4] = (f80.low >> 32) & 0xff;
    ptr[3] = (f80.low >> 24) & 0xff;
    ptr[2] = (f80.low >> 16) & 0xff;
    ptr[1] = (f80.low >> 8) & 0xff;
    ptr[0] = (f80.low >> 0) & 0xff;
    
    return result;
}

static void inst_fmath (const uint16_t ext)
{
    fpu_get_state_ptr();
    
    floatx80 rounded_result;
    
    ~decompose(shoe.op, 1111 001 000 MMMMMM);
    ~decompose(ext, 0 a 0 sss ddd eeeeeee);
    
    const uint8_t src_in_ea = a;
    const uint8_t source_specifier = s;
    const uint8_t dest_register = d;
    const uint8_t extension = e;
    
    slog("FPU:---\n");
    
    /* Throw illegal instruction for 040-only ops */
    if (_fmath_flags[e] & FMATH_TYPE_68040) {
        _throw_illegal_instruction();
        return;
    }
    
    /*
     * All the documented fmath ops have an implementation in
     * _fmath_map[]. If it's NULL, it's not documented; throw
     * an exception.
     * This probably matches what the 68040's behavior (I haven't
     * checked), but the 68881 doesn't do this.
     * 68881 throws an illegal instruction exception for all
     * opcodes where the high (6th) bit of e is set.
     * All other instructions seem to short circuit to the 
     * nearest documented instruction.
     * FIXME: consider implementing this behavior.
     */
    if (_fmath_map[e] == NULL) {
        /* Unless this is fmovecr, where the extension doesn't matter */
        if (!(src_in_ea && (source_specifier == 7))) {
            _throw_illegal_instruction();
            return ;
        }
    }
    
    /* We only need to load the dest reg for dyadic ops */
    if (_fmath_flags[e] & FMATH_TYPE_DYADIC)
        fpu->dest = floatx80_to_float128(fpu->fp[dest_register]);
    
    /*
     * We'll shrink the precision and perform rounding
     * just prior to writing back the result.
     * Certain instructions override the precision
     * in fpcr, so keep track of the prefered prec here.
     */
    enum rounding_precision_t rounding_prec = mc_prec;
    
    /*
     * For all the intermediate calculations, we
     * probably want to use nearest-rounding mode.
     */
    _set_rounding_mode(mode_nearest);
    
    /* Reset fpsr's exception flags */
    es_inex1 = 0; // this is only set for imprecisely-rounded packed inputs (not implemented)
    es_inex2 = 0; // set if we ever lose precision (during the op or during rounding)
    es_dz = 0;    // set if we divided by zero
    es_unfl = 0;  // set if we underflowed (inex2 should be set too, I think)
    es_ovfl = 0;  // set if we overflowed (inex2 should be set too, I think)
    es_operr = 0; // set if there was an instruction specific operand error
    es_snan = 0;  // Set if one of the inputs was a signaling NaN
    es_bsun = 0;  // never set here
    
    /* Clear the whole CC register byte */
    fpu->fpsr.raw &= 0x00ffffff;
    
    fpu->write_back = 1; // let "do-write-back" be the default behavior
    fpu->fmath_op = e;
    
    /* Handle fmovecr */
    if (src_in_ea && (source_specifier == 7)) { // fmovecr
        /* 
         * 68kprm says M should be ~b(000000), but apparently
         * any value will work for fmovecr
         */
        slog("FPU: fmovecr %u,fp%u\n", e, dest_register);
        inst_fmath_fmovecr();
        goto computation_done;
    }
    
    /*
     * Read in the source from the EA or from a register.
     * In either case, convert the value to a float128,
     * (that's our version of the 85-bit "intermediate" format)
     */
    if (src_in_ea) {
        if (!_fpu_read_ea(source_specifier, &fpu->source))
            return ;
        slog("FPU: %s.%c ", _fmath_names[e], "lsxpwdb?"[source_specifier]);
    }
    else {
        fpu->source = floatx80_to_float128(fpu->fp[source_specifier]);
        slog("FPU: %s.x ", _fmath_names[e], "lsxpwdb?"[source_specifier]);
    }

    
    /*{
        long double tmp = _float128_to_long_double(fpu->source);
        printf("%Lf,fp%u\n", tmp, dest_register);
    }*/
    
    /* fsincos needs this to know which register to write cos */
    fpu->extension_word = ext;
    
    /*
     * If the source is NaN, or this is a dyadic (two-operand)
     * instruction, and the second operand (fpu->dest) is NaN,
     * then the result is predetermined: NaN
     */
    if (float128_is_nan(fpu->source) ||
             (((_fmath_flags[e] & FMATH_TYPE_DYADIC) &&
               float128_is_nan(fpu->dest)))) {
        _fmath_handle_nans();
        /*
         * The result is guaranteed to be NaN, so we need
         * to set cc_nan for ftst and fcmp, who bypass
         * _fmath_set_condition_codes()
         */
        cc_nan = 1;
        goto computation_done;
    }
    
    /* Reset softfloat's exception flags */
    float_exception_flags = 0;
    
    /* 
     * Otherwise, call the extension-specific helper function.
     * Guarantees: Neither source nor dest are NaN
     *             SoftFloat's exception flags have been cleared
     */
    _fmath_map[e]();
    
    /* 
     * At this point, the "computation"-phase (I forget what the correct
     * 6888x term is) is over. Now we check exception bits, throw exceptions,
     * compute condition codes, and round and store the result.
     */
computation_done:
    
    /* 
     * Convert the 128-bit result to the specified precision.
     * FIXME: do we need to do rounding for ftst/fcmp?
     */
    if (fpu->write_back)
        rounded_result = _fmath_round_intermediate_result();
    
    
    /* Update the accrued exception bits */
    
    assert(!es_bsun); // no fmath op can throw es_bsun
    
    ae_iop |= es_bsun | es_snan | es_operr;
    ae_ovfl |= es_ovfl;
    ae_unfl |= (es_unfl & es_inex2); // yes, &
    ae_dz |= es_dz;
    ae_inex |= es_inex1 | es_inex2 | es_ovfl;
    
    slog("FPU: bsun=%u snan=%u operr=%u ovfl=%u unfl=%u dz=%u inex1=%u inex2=%u\n",
           es_bsun, es_snan, es_operr, es_ovfl, es_unfl, es_dz, es_inex1, es_inex2);
    
    /* Are any exceptions both set and enabled? */
    if (fpu->fpsr.raw & fpu->fpcr.raw & 0x0000ff00) {
        /* 
         * Then we need to throw an exception.
         * The exception is sent to the vector for
         * the highest priority exception, and the priority
         * order is (high->low) bsan, snan, operr, ovfl, unfl, dz, inex2/1
         * (which is the order of the bits in fpsr/fpcr).
         * Iterate over the bits in order, and throw the
         * exception to whichever bit is set first.
         */
        uint8_t i, throwable = (fpu->fpsr.raw & fpu->fpcr.raw) >> 8;
        
        slog("FPU: throw exception! 0x%08x\n", throwable);
        
        assert(throwable);
        for (i=0; 1; i++) {
            if (throwable & 0x80)
                break;
            throwable <<= 1;
        }
        
        /*
         * FIXME: are condition codes ever set if an exception is thrown?
         * I think not, so clear the whole CC register byte
         */
        fpu->fpsr.raw &= 0x00ffffff;
        
        /*
         * Convert the exception bit position
         * to the correct vector number, and throw
         * a (pre-instruction) exception.
         */
        throw_fpu_pre_instruction_exception(_exception_bit_to_vector[i]);
        
        return ;
    }
    
    /*
     * Otherwise, no exceptions to throw!
     * We're definitely running to completion now,
     * so commit ea-read changes
     */
    _fpu_read_ea_commit(source_specifier);
    
    if (fpu->write_back) {
        /* Calculate the condition codes from the result. */
        _fmath_set_condition_codes(rounded_result);
        
        /* Write back the result, and we're done! */
        if (fpu->write_back) {
            fpu->fp[dest_register] = rounded_result;
            
            long double tmp = _floatx80_to_long_double(rounded_result);
            slog("FPU: result = %Lf\n", tmp);
        }
    }
}

#pragma mark Second-hop non-fmath instructions

/*
 * reg->mem fmove (fmath handles all other fmoves)
 */
static void inst_fmove (const uint16_t ext)
{
    fpu_get_state_ptr();
    
    ~decompose(shoe.op, 1111 001 000 MMMMMM);
    ~decompose(shoe.op, 1111 001 000 mmmrrr);
    ~decompose(ext, 011 fff sss KKKKKKK);
    
    _fpu_write_ea(M, f, &fpu->fp[s], K);
}

static void inst_fmovem_control (const uint16_t ext)
{
    fpu_get_state_ptr();
    
    ~decompose(shoe.op,  1111 001 000 mmmrrr);
    ~decompose(shoe.op,  1111 001 000 MMMMMM);
    ~decompose(ext, 10d CSI 0000000000);
    
    const uint32_t count = C + S + I;
    const uint32_t size = count * 4;
    uint32_t addr, buf[3];
    uint32_t i;
    
    /* I don't know if this is even a valid instruction */
    if (count == 0)
        return ;
    
    /* data and addr reg modes are valid, but only if count==1 */
    if ((m == 0 || m == 1) && (count > 1)) {
        _throw_illegal_instruction();
        return ;
    }
    
    if (d) { // reg to memory
        i=0;
        if (C) buf[i++] = fpu->fpcr.raw;
        if (S) buf[i++] = fpu->fpsr.raw;
        if (I) buf[i++] = fpu->fpiar;
        
        if (m == 0) {
            if (count == 1)
                shoe.d[r] = buf[0];
            else
                _throw_illegal_instruction();
            return ;
        }
        else if (m == 1) {
            if ((count == 1) && I)
                shoe.a[r] = buf[0];
            else
                _throw_illegal_instruction();
            return ;
        }
        else if (m == 3)
            addr = shoe.a[r];
        else if (m == 4)
            addr = shoe.a[r] - size;
        else {
            if ((m==7) && (r!=0 || r!=1)) {
                /* Not allowed for reg->mem */
                _throw_illegal_instruction();
                return;
            }
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
        if (m == 0) {// data reg
            if (count == 1)
                buf[0] = shoe.d[r];
            else
                _throw_illegal_instruction();
            return;
        }
        else if (m == 1) {// addr reg
            if ((count == 1) && I)
                buf[0] = shoe.a[r];
            else
                _throw_illegal_instruction();
            return;
        }
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
            uint8_t round = fpu->fpcr.b._mc_rnd;
            fpu->fpcr.raw = buf[i++];
            uint8_t newround = fpu->fpcr.b._mc_rnd;
            
            if (round != newround) {
                slog("FPU: inst_fmovem_control: HEY: round %u -> %u\n", round, newround);
            }
        }
        if (S) fpu->fpsr.raw = buf[i++];
        if (I) fpu->fpiar = buf[i++];
        
        // Commit immediate-EA-mode PC change
        if (M == 0x3c)
            shoe.pc += size;
    }
    
    // Commit pre/post-inc/decrement
    
    if (m == 3)
        shoe.a[r] += size;
    if (m == 4)
        shoe.a[r] -= size;
    
    
    
    slog("FPU: inst_fmove_control: notice: (EA = %u/%u %08x CSI = %u%u%u)\n", m, r, (uint32_t)shoe.dat, C, S, I);
    
    
}

static void inst_fmovem (const uint16_t ext)
{
    fpu_get_state_ptr();
    
    ~decompose(shoe.op,  1111 001 000 mmmrrr);
    ~decompose(shoe.op,  1111 001 000 MMMMMM);
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
    
    slog("FPU: inst_fmovem: pre=%08x mask=%08x EA=%u/%u addr=0x%08x size=%u %s\n", pre_mask, mask, m, r, addr, size, d?"to mem":"from mem");
    
    if (d) {
        // Write those registers
        for (i=0; i<8; i++) {
            if (!(mask & (0x80 >> i)))
                continue;
            
            uint8_t buf[12];
            _floatx80_to_extended(&fpu->fp[i], buf);
            
            // slog("inst_fmovem: writing %Lf from fp%u", fpu->fp[i], i);
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
            _extended_to_floatx80(buf, &fpu->fp[i]);
            
            // slog("inst_fmovem: read %Lf to fp%u\n", shoe.fp[i], i);
        }
    }
    
    // Commit the write for pre/post-inc/decrement
    if (m == 3)
        shoe.a[r] += size;
    else if (m == 4)
        shoe.a[r] -= size;
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
    if (b && cc_nan && _bsun_test())
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
    if (b && cc_nan && _bsun_test())
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
        assert(!"inst_frestore: bad state frame");
        return ;
    }
    
    if (m==3) {
        shoe.a[r] += size;
        slog("FPU: frestore: changing shoe.a[%u] += %u\n", r, size);
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
    if (b && cc_nan && _bsun_test())
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
    if (b && cc_nan && _bsun_test())
        return ;
    
    if (fpu_test_cc(c))
        throw_frame_two(shoe.sr, next_pc, 7, shoe.orig_pc);
    else
        shoe.pc = next_pc;
}

void inst_fnop() {
    // This is technically fbcc
    inst_fbcc();
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
            _throw_illegal_instruction();
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

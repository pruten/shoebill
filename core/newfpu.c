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
#define nextword() ({const uint16_t w=lget(shoe.pc,2); if (shoe.abort) {return;}; shoe.pc+=2; w;})
#define nextlong() ({const uint32_t L=lget(shoe.pc,4); if (shoe.abort) {return;}; shoe.pc+=4; L;})
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

#pragma mark Float format translators

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

static float128 _float_to_intermediate(uint32_t f)
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
    
    switch (m) {
        case 0:
            if (format == format_S)
                *result = _float_to_intermediate(shoe.d[r]);
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
                throw_illegal_instruction();
                return 0;
            }
            goto got_data;
            
        case 1:
            /* Address regisers can't be used */
            throw_illegal_instruction();
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
    
    if (size <= 4) {
        const uint32_t raw = lget(addr, size);
        if (shoe.abort)
            return 0;
        
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
                *result = _int32_to_intermediate(raw);
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
                // throw_illegal_instruction();
                // return 0;
            default:
                assert(0); // never get here
        }
    }
    
got_data:
    
    return 1;
}

#pragma mark FMATH! and all its helpers

/* Function prototypes from SoftFloat/softfloat-specialize.h */
char float128_is_nan(float128 a);
char float128_is_signaling_nan (float128 a);

/*
 * s -> sign, e -> biased exponent
 * ma -> 48 high bits of the mantissa
 * mb -> 64 low bits of the mantissa
 */
#define _assemble_float128(s, e, ma, mb) ({ \
    const uint64_t _ma = (ma), _mb = (mb); \
    float128 f = { \
        .high = (((s) != 0) << 16) | ((e) & 0x7fff), \
        .low = _mb \
    }; \
    f.high = ((f.high) << 48) | _ma; \
    f; \
})


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
            // FIXME: 68881 doesn't set inex2 for this one
            // (are the three intermediate bits zeros? I didn't check)
            fpu->result = _assemble_float128(0, 0x3fff, 0x71547652b82f, 0xe1777d0ffda0d23a);
            break;
        case 0x0e: // log_10(e)
            fpu->result = _assemble_float128(0, 0x3ffd, 0xbcb7b1526e50, 0xe32a6ab7555f5a67);
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
    
    return $map_str;
})

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
    
    assert(!"fmath: facos not implemented");
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
}

static void inst_fmath_fasin ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fasin not implemented");
}

static void inst_fmath_fatan ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fatan not implemented");
}

static void inst_fmath_fatanh ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fatanh not implemented");
}

static void inst_fmath_fcmp ()
{
    fpu_get_state_ptr();
    
    /* Don't write the result back to the register */
    fpu->write_back = 0;
    
    fpu->result = float128_sub(fpu->dest, fpu->source);
    
    /*
     * The 68881 docs say fcmp doesn't throw any exceptions
     * based on the result, but I'm not sure I believe it.
     
     if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
     
     if (float_exception_flags & float_flag_inexact)
        es_inex2 = 1;
     */
}

static void inst_fmath_fcos ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fcos not implemented");
}

static void inst_fmath_fcosh ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fcosh not implemented");
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
}

static void inst_fmath_fetox ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fetox not implemented");
}

static void inst_fmath_fetoxm1 ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fetoxm1 not implemented");
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
    
    assert(!"fmath: fgetman not implemented");
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
    
    assert(!"fmath: flog10 not implemented");
}

static void inst_fmath_flog2 ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: flog2 not implemented");
}

static void inst_fmath_flogn ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: flogn not implemented");
}

static void inst_fmath_flognp1 ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: flognp1 not implemented");
}

static void inst_fmath_fmod ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fmod not implemented");
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
}

static void inst_fmath_fneg ()
{
    fpu_get_state_ptr();
    
    /* Flip the sign bit */
    fpu->result = fpu->dest;
    fpu->result.high ^= (1ULL << 63);
}

static void inst_fmath_frem ()
{
    fpu_get_state_ptr();
    float128 tmp;
    
    fpu->result = float128_rem(fpu->dest, fpu->source);
    
    /*
     * Throw operr (and return NaN) if source is zero,
     * or if dest is infinity.
     */
    if (float_exception_flags & float_flag_invalid)
        es_operr = 1;
    
    // FIXME: !! set quotient byte!
    // you may be able to use the local "q" bits64 in
    // float128_rem()
    
    
    /*
     * errata: 68kprm has typesetting issues (or typos?)
     *         if source==inf, don't set operr!
     */
    
}

static void inst_fmath_fscale ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fscale not implemented");
}

static void inst_fmath_fsgldiv ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fsgldiv not implemented");
}

static void inst_fmath_fsglmul ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fsglmul not implemented");
}

static void inst_fmath_fsin ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fsin not implemented");
}

static void inst_fmath_fsincos ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fsincos not implemented");
}

static void inst_fmath_fsinh ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: fsinh not implemented");
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
}

static void inst_fmath_ftan ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: ftan not implemented");
}

static void inst_fmath_ftanh ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: ftanh not implemented");
}

static void inst_fmath_ftentox ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: ftentox not implemented");
}

static void inst_fmath_ftst ()
{
    fpu_get_state_ptr();
    
    /* Don't write the result back to the register */
    fpu->write_back = 0;
    
    /* ftst just sets the cond codes according to the source */
    fpu->result = fpu->source;
}

static void inst_fmath_ftwotox ()
{
    fpu_get_state_ptr();
    
    assert(!"fmath: ftwotox not implemented");
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
    
    /* Clear the whole CC register byte */
    fpu->fpsr.raw &= 0x00ffffff;
    
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
    
    /* Throw illegal instruction for 040-only ops */
    if (_fmath_flags[e] & FMATH_TYPE_68040) {
        throw_illegal_instruction();
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
        throw_illegal_instruction();
        return ;
    }

    /* 
     * Read in the source from the EA or from a register.
     * In either case, convert the value to a float128,
     * (that's our version of the 85-bit "intermediate" format)
     */
    if (src_in_ea) {
        if (!_fpu_read_ea(source_specifier, &fpu->source))
            return ;
    }
    else
        fpu->source = floatx80_to_float128(fpu->fp[source_specifier]);
    
    
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
    
    /* Reset softfloat's exception flags */
    float_exception_flags = 0;
    
    /* Reset fpsr's exception flags */
    es_inex1 = 0; // this is only set for imprecisely-rounded packed inputs (not implemented)
    es_inex2 = 0; // set if we ever lose precision (during the op or during rounding)
    es_dz = 0;    // set if we divided by zero
    es_unfl = 0;  // set if we underflowed (inex2 should be set too, I think)
    es_ovfl = 0;  // set if we overflowed (inex2 should be set too, I think)
    es_operr = 0; // set if there was an instruction specific operand error
    es_snan = 0;  // Set if one of the inputs was a signaling NaN
    es_bsun = 0;  // never set here
    
    fpu->write_back = 1; // let "do-write-back" be the default behavior
    fpu->fmath_op = e;
    
    /* Handle fmovecr */
    if (src_in_ea && (source_specifier == 7)) { // fmovecr
        /* 
         * 68kprm says M should be ~b(000000), but apparently
         * any value will work for fmovecr
         */
        inst_fmath_fmovecr();
        goto computation_done;
    }
    
    /*
     * If the source is NaN, or this is a dyadic (two-operand)
     * instruction, and the second operand (fpu->dest) is NaN,
     * then the result is predetermined: NaN
     */
    if (float128_is_nan(fpu->source) ||
             (((_fmath_flags[e] & FMATH_TYPE_DYADIC) &&
               float128_is_nan(fpu->dest)))) {
        _fmath_handle_nans();
        goto computation_done;
    }
    
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
    
    /* Convert the 128-bit result to the specified precision */
    /*
     * FIXME: If fpu->write_back==0, should we still go through rounding?
     *        The condition codes will still need to be set. Should they
     *        be set based on the intermediate result or rounded result?
     */
    rounded_result = _fmath_round_intermediate_result();
    
    
    /* Update the accrued exception bits */
    
    assert(!es_bsun); // no fmath op can throw es_bsun
    
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
    
    /*
     * Otherwise, no exceptions to throw!
     * Calculate the condition codes from the result.
     */
    _fmath_set_condition_codes(rounded_result);
    
    /*
     * We're definitely running to completion now,
     * so commit ea-read changes
     */
    _fpu_read_ea_commit(source_specifier);
    
    /* Write back the result, and we're done! */
    if (fpu->write_back)
        fpu->fp[dest_register] = rounded_result;
}

#pragma mark Second-hop non-fmath instructions

static void inst_fmove (const uint16_t ext)
{
    fpu_get_state_ptr();
    
    assert(!"fmove");
}

static void inst_fmovem_control (const uint16_t ext)
{
    fpu_get_state_ptr();
    
    assert(!"fmovem_control");
}

static void inst_fmovem (const uint16_t ext)
{
    fpu_get_state_ptr();
    
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

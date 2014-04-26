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

#ifndef _SHOEBILL_H
#define _SHOEBILL_H

#include <stdio.h>
#include <time.h>
#include <stdint.h> 
#include <sys/time.h>
#include <pthread.h>
//#include <histedit.h>

// void ring_print(const char *str);
// extern char *ring_tmp;

#include "coff.h"

// -- Global constants --

    // bit manipulation
        #define bitchop(v, s) ({const uint32_t _v = (v), _s = 32 - (s); (_v << _s) >> _s;})
        // #define bitchop(v, s) ((v) & (0xffffffff>>(32-(s))))
        #define bitchop_64(v, s) ({const uint64_t _v = (v); const uint32_t _s = 64 - (s); (_v << _s) >> _s;})
        // #define bitchop_64(v, s) ((v) & (0xffffffffffffffff>>(64-(s))))
        #define chop(v, s) bitchop_64((v), (s)*8)
        #define mib(v, s) (((v)>>((s)*8-1))&1)

        #define lrot32(v, r) (uint32_t)(((((uint64_t)(v))<<(r)) | (((uint64_t)(v))>>(32-(r)))))
        #define rrot32(v, r) lrot32((v), 32-(r))


    // set register
        #define set_reg__(reg, val, s)  do { \
            const uint32_t mask_v=(0xffffffff)>>(8*(4-(s))), mask_r=~mask_v; \
            (reg) = ((reg)&mask_r) | ((val)&mask_v);\
        } while (0)
        #define get_reg__(reg, s) chop(reg, s)
        
        #define get_d(n,s) get_reg__(shoe.d[n], s)
        #define get_a(n,s) get_reg__(shoe.a[n], s)
        #define set_d(n,val,s) set_reg__(shoe.d[n], val, s)

		
	// sr masks
		#define sr_c() (shoe.sr&1)
		#define sr_v() ((shoe.sr>>1)&1)
		#define sr_z() ((shoe.sr>>2)&1)
		#define sr_n() ((shoe.sr>>3)&1)
		#define sr_x() ((shoe.sr>>4)&1)
        #define sr_mask() ((shoe.sr>>8)&7)
		#define sr_m() ((shoe.sr>>12)&1)
		#define sr_s() ((shoe.sr>>13)&1)
		#define sr_t0() ((shoe.sr>>14)&1)
		#define sr_t1() ((shoe.sr>>15)&1)

        // a7 can actually point to isp, msp, or usp, depending on bits in the status register.
        // So every time we modify those bits, save a7 to the correct internal register.
        #define make_stack_pointers_valid() { \
            if (sr_s() && sr_m()) \
                shoe.msp = shoe.a[7]; \
            else if (sr_s()) \
                shoe.isp = shoe.a[7]; \
            else \
                shoe.usp = shoe.a[7]; \
        }
        
        // Load the correct stack pointer into a7 based on bits in sr
        #define load_stack_pointer() { \
            if (sr_s() && sr_m()) \
                shoe.a[7] = shoe.msp; \
            else if (sr_s()) \
                shoe.a[7] = shoe.isp; \
            else \
                shoe.a[7] = shoe.usp; \
        }
        
        // set the status register, swapping a7 if necessary
        #define set_sr(newsr) { \
            make_stack_pointers_valid(); \
            shoe.sr = newsr & 0xf71f; \
            load_stack_pointer(); \
        }

		#define set_sr_c(b) {shoe.sr &= (~(1<<0)); shoe.sr |= (((b)!=0)<<0);}
		#define set_sr_v(b) {shoe.sr &= (~(1<<1)); shoe.sr |= (((b)!=0)<<1);}
		#define set_sr_z(b) {shoe.sr &= (~(1<<2)); shoe.sr |= (((b)!=0)<<2);}
		#define set_sr_n(b) {shoe.sr &= (~(1<<3)); shoe.sr |= (((b)!=0)<<3);}
		#define set_sr_x(b) {shoe.sr &= (~(1<<4)); shoe.sr |= (((b)!=0)<<4);}
        #define set_sr_mask(m) {shoe.sr &= (~(7<<8)); shoe.sr |= ((((uint16_t)(m))&7) << 8);}
		// Be careful when setting these bits
        #define set_sr_m(b) {make_stack_pointers_valid(); shoe.sr &= (~(1<<12)); shoe.sr |= (((b)!=0)<<12); load_stack_pointer();}
		#define set_sr_s(b) {make_stack_pointers_valid(); shoe.sr &= (~(1<<13)); shoe.sr |= (((b)!=0)<<13); load_stack_pointer();}
		#define set_sr_t0(b) {shoe.sr &= (~(1<<14)); shoe.sr |= (((b)!=0)<<14);}
		#define set_sr_t1(b) {shoe.sr &= (~(1<<15)); shoe.sr |= (((b)!=0)<<15);}

    // MMU
        #define tc_enable() (shoe.tc >> 31)
        #define tc_sre() ((shoe.tc >> 25) & 1)
        #define tc_fcl() ((shoe.tc >> 24) & 1)
        #define tc_ps() ((shoe.tc >> 20) & 0xf)
        #define tc_is() ((shoe.tc >> 16) & 0xf)
        #define tc_tia() ((shoe.tc >> 12) & 0xf)
        #define tc_tib() ((shoe.tc >> 8) & 0xf)
        #define tc_tic() ((shoe.tc >> 4) & 0xf)
        #define tc_tid() (shoe.tc & 0xf)
        #define tc_ti(n) ((shoe.tc >> ((3-(n))*4)) & 0xf)

        #define rp_lu(x) ((uint32_t)(((x) >> 63) & 1))
        #define rp_limit(x) ((uint32_t)(((x) >> 48) & 0x7fff))
        #define rp_sg(x) ((uint32_t)(((x) >> 41) & 1))
        #define rp_dt(x) ((uint32_t)(((x) >> 32) & 3))
        #define rp_addr(x) ((uint32_t)(((x) >> 0) & 0xfffffff0))

    // misc
        #define ea_n(s) ((shoe.dat>>((s)*8-1))&1)
        #define ea_z(s) (chop(shoe.dat, (s))==0)

// alloc_pool.c
typedef struct _alloc_pool_t {
    struct _alloc_pool_t *prev, *next;
    uint32_t size, magic;
} alloc_pool_t;

void* p_alloc(alloc_pool_t *pool, uint64_t size);
void* p_realloc(void *ptr, uint64_t size);
void p_free(void *ptr);
void p_free_pool(alloc_pool_t *pool);
alloc_pool_t* p_new_pool(void);



typedef struct dbg_breakpoint_t {
    struct dbg_breakpoint_t *next;
    uint32_t addr;
    uint64_t num;
} dbg_breakpoint_t;

typedef struct {
    // EditLine *el;
    uint8_t mode;
    uint8_t ignore_interrupts;
    uint8_t connected;
    uint64_t breakpoint_counter;
    dbg_breakpoint_t *breakpoints;
} debugger_state_t;

typedef enum {
    adb_talk,
    adb_listen,
    adb_reset,
    adb_flush
} adb_command_type_t;

typedef struct {
    adb_command_type_t command_type;
    
    uint8_t command_byte; // The command byte passed in during state 0
    uint8_t command_device_id; // the device id in the command byte
    uint8_t command_reg; // the register in the command byte
    
    uint8_t service_request, timeout;
    uint16_t pending_service_requests;
    uint8_t pending_poll;
    uint8_t poll; // a poll is in progress
    
    
    uint8_t state; // State machine state (0 => host send command, 1 => even byte, 2 => odd byte, 3 => idle
    uint8_t data_i, data_len;
    uint8_t data[8];
    
    pthread_mutex_t lock;
    
} adb_state_t;

typedef struct {
    uint8_t ifr, ier, rega, regb, ddrb, ddra, sr;
} via_state_t;

typedef struct {
    uint8_t scsi_id;
    uint32_t num_blocks, block_size;
    FILE *f;
    const char *image_path;
} scsi_device_t;

#define KEYBOARD_STATE_MAX_KEYS 128
typedef struct {
    struct {
        uint8_t code_a, code_b;
    } keys[KEYBOARD_STATE_MAX_KEYS];
    
    int down_modifiers; // Modifiers that we've already told the OS are down (shift, ctrl,
    uint32_t key_i;
    uint8_t last_modifier_mask;
} keyboard_state_t;

typedef struct {
    int32_t old_x, old_y;
    int32_t delta_x, delta_y;
    uint8_t button_down;
    uint8_t changed;
} mouse_state_t;

typedef struct {
    uint8_t *buf_base;
    uint32_t buf_size;
    
    uint32_t h_offset; // offset in bytes for each horizontal line
    
    uint8_t vsync;
    
    // unit8_t via_interrupt_flag;
    
    uint8_t depth;
    uint8_t clut[256 * 3];
    uint32_t clut_idx;
    
} video_state_t;

typedef struct {
    // lsb==phase0, msb==L7
    uint8_t latch;
    
    // Registers
    uint8_t data, status, mode, handshake;
} iwm_state_t;


typedef struct {
    uint32_t (*read_func)(uint32_t, uint32_t, uint8_t);
    void (*write_func)(uint32_t, uint32_t, uint32_t, uint8_t);
    void *ctx;
    uint8_t slotnum, connected, interrupts_enabled;
    int32_t glut_window_id;
    long double interrupt_rate, last_fired;
} nubus_card_t;

typedef struct {
    uint32_t logical_value : 24; // At most the high 24 bits of the logical address
    uint32_t used_bits : 5;
    uint32_t wp : 1; // whether the page is write protected
    uint32_t modified : 1; // whether the page has been modified
    uint32_t unused1 : 1;
    uint32_t unused2 : 8;
    uint32_t physical_addr : 24;
} pmmu_cache_entry_t;

typedef struct {
    uint64_t emu_start_time;
    struct timeval last_60hz_tick; // for via1 ca1
    
} via_clock_t;

typedef struct {
    
#define SHOEBILL_STATE_STOPPED (1<<9)
    
    // bits 0-6 are CPU interrupt priorities
    // bit 8 indicates that STOP was called
    volatile uint32_t cpu_thread_notifications;
    
    pthread_mutex_t cpu_thread_lock;
    pthread_mutex_t via_clock_thread_lock;
    pthread_mutex_t cpu_freeze_lock;
    
    // -- PMMU caching structures ---
#define PMMU_CACHE_KEY_BITS 10
#define PMMU_CACHE_SIZE (1<<PMMU_CACHE_KEY_BITS)
    struct {
        pmmu_cache_entry_t entry[PMMU_CACHE_SIZE];
        uint8_t valid_map[PMMU_CACHE_SIZE / 8];
    } pmmu_cache[2];
    
    // -- Assorted CPU state variables --
    uint16_t op; // the first word of the instruction we're currently running
    uint16_t orig_sr; // the sr before we began executing the instruction
    uint32_t orig_pc; // the address of the instruction we're currently running
    uint16_t exception;
    _Bool abort;
    _Bool suppress_exceptions;
    
    // -- Physical memory --
    uint8_t *physical_mem_base;
    uint32_t physical_mem_size;
    uint8_t *physical_rom_base;
    uint32_t physical_rom_size;
    
    uint32_t physical_size; // <- Size of transfer
    uint32_t logical_size; // <- Size of transfer
    uint32_t physical_addr; // <- Address for physical reads/writes
    uint32_t logical_addr; // <- Address for logical reads/writes
    uint64_t physical_dat; // <- Data for physical fetches is put/stored here
    uint64_t logical_dat; // <- Data for logical fetches is put/stored here
	
    _Bool logical_is_write; // <- boolean: true iff the operation is logical_set()
    uint8_t logical_fc; // logical function code
    
    // -- Interrupts/VIA chips --
    
    // uint8_t stopped; // whether STOP was called
    // uint8_t pending_interrupt; // (0 -> no pending interrupt, >0 -> interrupt priority (autovector))
    via_state_t via[2];
    adb_state_t adb;
    keyboard_state_t key;
    mouse_state_t mouse;
    iwm_state_t iwm;
    
    nubus_card_t slots[16];
    // video_state_t video;
    
    // -- Registers --
    uint32_t d[8];
    uint32_t a[8];
    uint32_t pc;
    
    uint32_t vbr; // vector base register
    uint32_t sfc; // source function code
    uint32_t dfc; // destination function code
    uint32_t cacr; // cache control register
    	
    uint32_t usp; // user stack pointer
    uint32_t isp; // interrupt stack pointer
    uint32_t msp; // master stack pointer
    
    uint16_t sr; // status register (use a macro to modify sr!)
    
    // 68851 registers
    uint64_t crp, srp, drp; // user/supervisor/DMA root pointers
    uint32_t tc; // translation control
    uint16_t pcsr; // PMMU cache status
    uint16_t ac; // access control
    uint16_t bad[8]; // breakpoint acknowledge data registers
    uint16_t bac[8]; // breakpoint acknowledge control registers
    uint8_t cal; // current access level
    uint8_t val; // validate access level
    uint8_t scc; // stack change control
    union {
        uint16_t word;
        struct {
            uint16_t n : 3; // number-of-levels
            uint16_t unused : 4; // (zeroed out)
            uint16_t c : 1; // globally shared
            uint16_t g : 1; // gate
            uint16_t m : 1; // modified
            uint16_t i : 1; // invalid
            uint16_t w : 1; // write-protected
            uint16_t a : 1; // access level violation
            uint16_t s : 1; // supervisor-only
            uint16_t l : 1; // limit violation
            uint16_t b : 1; // bus error
        } bits;
    } psr;
    
    // FPU registers
    uint32_t fpiar; // FPU iaddr
    
    union { // fpcr, fpu control register
        struct {
            // Mode control byte
            uint16_t mc_zero : 4; // zero/dummy
            uint16_t mc_rnd  : 2; // rounding mode
            uint16_t mc_prec : 2; // rounding precision
            // Exception enable byte
            uint16_t ee_inex1 : 1; // inexact decimal input
            uint16_t ee_inex2 : 1; // inxact operation
            uint16_t ee_dz    : 1; // divide by zero
            uint16_t ee_unfl  : 1; // underflow
            uint16_t ee_ovfl  : 1; // overflow
            uint16_t ee_operr : 1; // operand error
            uint16_t ee_snan  : 1; // signalling not a number
            uint16_t ee_bsun  : 1; // branch/set on unordered
        } b;
        
        uint16_t raw;
    } fpcr;
    
    union { // fpsr, fpu status register
        struct {
            // Accrued exception byte
            uint32_t dummy1  : 3; // dummy/zero
            uint32_t ae_inex : 1; // inexact
            uint32_t ae_dz   : 1; // divide by zero
            uint32_t ae_unfl : 1; // underflow
            uint32_t ae_ovfl : 1; // overflow
            uint32_t ae_iop  : 1; // invalid operation
            // Exception status byte
            uint32_t es_inex1 : 1; // inexact decimal input
            uint32_t es_inex2 : 1; // inxact operation
            uint32_t es_dz    : 1; // divide by zero
            uint32_t es_unfl  : 1; // underflow
            uint32_t es_ovfl  : 1; // overflow
            uint32_t es_operr : 1; // operand error
            uint32_t es_snan  : 1; // signalling not a number
            uint32_t es_bsun  : 1; // branch/set on unordered
            // Quotient byte
            uint32_t qu_quotient : 7;
            uint32_t qu_s        : 1;
            // Condition code byte
            uint32_t cc_nan  : 1; // not a number
            uint32_t cc_i    : 1; // infinity
            uint32_t cc_z    : 1; // zero
            uint32_t cc_n    : 1; // negative
            uint32_t dummy2  : 4; // dummy/zero
        } b;
        uint32_t raw;
    } fpsr;
    
    long double fp[8]; // 80 bit floating point general registers
    
    
    // -- EA state --
    uint32_t uncommitted_ea_read_pc; // set by ea_read(). It's the PC that ea_read_commit will set.
    uint64_t dat; // the raw input/output for the transaction
    uint32_t extended_addr; // EA returned by ea_decode_extended()
    uint32_t extended_len; // number of instruction bytes used by ea_decode_extended()
    uint8_t sz; // the size of the EA transaction
    uint8_t mr; // a 6-bit mode/reg pair
    
    via_clock_t via_clocks;
    
    
    struct timeval start_time; // when the emulator started (for computing timer interrupts)
    uint64_t total_ticks; // how many 60hz ticks have been generated
    
    coff_file *coff; // Data/symbols from the unix kernel
    
    coff_file *launch; // FIXME: delete me: coff symbols from aux 1.1.1 launch binary
    
    scsi_device_t scsi_devices[8]; // SCSI devices
    
    debugger_state_t dbg;
    alloc_pool_t *pool;
} global_shoebill_context_t;

extern global_shoebill_context_t shoe; // declared in cpu.c

// fpu.c functions
void inst_fpu_decode(void);
void dis_fpu_decode(void);
void fpu_setup_jump_table();

// cpu.c fuctions
void cpu_step (void);
void inst_decode (void);

// exception.c functions

void throw_bus_error(uint32_t addr, uint8_t is_write);
void throw_long_bus_error(uint32_t addr, uint8_t is_write);
void throw_address_error();
void throw_illegal_instruction();
void throw_privilege_violation();
void throw_divide_by_zero();
void throw_frame_two (uint16_t sr, uint32_t next_pc, uint32_t vector_num, uint32_t orig_pc);


// mem.c functions

//void physical_get (void);
typedef void (*physical_get_ptr) (void);
typedef void (*physical_set_ptr) (void);
extern const physical_get_ptr physical_get_jump_table[16];
extern const physical_set_ptr physical_set_jump_table[16];

#define physical_set() physical_set_jump_table[shoe.physical_addr >> 28]()
#define pset(addr, s, val) {shoe.physical_addr=(addr); shoe.physical_size=(s); shoe.physical_dat=(val); physical_set();}

#define physical_get() physical_get_jump_table[shoe.physical_addr >> 28]()
#define pget(addr, s) ({shoe.physical_addr=(addr); shoe.physical_size=(s); physical_get(); shoe.physical_dat;})

void logical_get (void);
#define lget_fc(addr, s, fc) ({ \
    shoe.logical_addr=(addr); \
    shoe.logical_size=(s); \
    shoe.logical_fc = (fc); \
    logical_get(); \
    shoe.logical_dat; \
})

#define lget(addr, s) ({ \
    shoe.logical_addr=(addr); \
    shoe.logical_size=(s); \
    shoe.logical_fc = (sr_s() ? 5 : 1); \
    logical_get(); \
    shoe.logical_dat; \
})

void logical_set (void);
#define lset_fc(addr, s, val, fc) {\
    shoe.logical_addr=(addr); \
    shoe.logical_size=(s); \
    shoe.logical_dat=(val); \
    shoe.logical_fc = (fc); \
    logical_set();\
}
#define lset(addr, s, val) { \
    lset_fc((addr), (s), (val), sr_s() ? 5 : 1) \
}


void ea_read();
void ea_read_commit();
void ea_write();
void ea_addr();

#define call_ea_read(M, s) {shoe.mr=(M);shoe.sz=(s);ea_read();if (shoe.abort) return;}
#define call_ea_write(M, s) {shoe.mr=(M);shoe.sz=(s);ea_write();if (shoe.abort) return;}
#define call_ea_read_commit(M, s) {shoe.mr=(M);shoe.sz=(s);ea_read_commit();if (shoe.abort) return;}
#define call_ea_addr(M) {shoe.mr=(M);ea_addr();if (shoe.abort) return;}

#define push_a7(_dat, _sz) {shoe.a[7]-=(_sz);lset(shoe.a[7], (_sz), (_dat));}

// 68851 MMU stuff

#define desc_dt(d,s) (((d) >> (32 * (s))) & 3)
#define desc_table_addr(d) ((uint32_t)((d) & 0xFFFFFFF0))
#define desc_page_addr(d) ((uint32_t)((d) & 0xffffff00))
#define desc_wp(d,s) ((((d) >> (32 * (s))) >> 2) & 1)
#define desc_m(d,s)  ((((d) >> (32 * (s))) >> 4) & 1)

#define desc_m_long (((uint64_t)1) << 36)
#define desc_m_short (((uint64_t)1) << 4)

#define get_desc(_addr, _size) { \
    desc = pget((_addr), (_size)); \
    desc_addr = (_addr); \
    desc_level++; \
}
//    if (shoe.dbg) \
//        printf("desc_addr *0x%08x = 0x%llx\n", (uint32_t)(_addr), desc); \
// }


// dis.c functions
void disassemble_inst(uint8_t binary[24], uint32_t orig_pc, char *str, uint32_t *instlen);
char* decode_ea_rw (uint8_t mr, uint8_t sz);
char* decode_ea_addr (uint8_t mr);
void dis_decode(void);
uint16_t dis_next_word (void);
char* decode_ea_addr (uint8_t mr);
char* decode_ea_rw (uint8_t mr, uint8_t sz);
struct dis_t {
    // static
    uint8_t binary[24]; // raw instruction (up to 24 bytes)
    uint32_t orig_pc; // the PC when disassemble_inst() was called
    
    // volatile
    char ea_str_internal[1024]; // data for storing decoded ea strings (ring buffer)
    uint32_t ea_last_pos_internal;
    uint32_t pos; // the current computed length of the instruction
    
    // return
    char *str; // the final returned string
};

// IWM / floppy
uint8_t iwm_dma_read();
void iwm_dma_write();

// ncr5380 (scsi)
void scsi_reg_read();
void scsi_reg_write();
uint8_t scsi_dma_read();
uint32_t scsi_dma_read_long();
void scsi_dma_write(uint8_t byte);
void scsi_dma_write_long(uint32_t dat);

// via1 & via2 (+ CPU interrupts)
void check_time();
void via_raise_interrupt(uint8_t vianum, uint8_t ifr_bit);
void process_pending_interrupt();
void via_reg_read();
void via_reg_write();
void *via_clock_thread(void *arg);

// VIA registers
#define VIA_ORB 0
#define VIA_ORA 1
#define VIA_DDRB 2
#define VIA_DDRA 3
#define VIA_T1C_LO 4
#define VIA_T1C_HI 5
#define VIA_T1L_LO 6
#define VIA_T1L_HI 7
#define VIA_T2C_LO 8
#define VIA_T2C_HI 9
#define VIA_SR 10
#define VIA_ACR 11
#define VIA_PCR 12
#define VIA_IFR 13
#define VIA_IER 14
#define VIA_ORA_AUX 15

// IFR interrupt bits
#define IFR_CA2 0
#define IFR_CA1 1
#define IFR_SHIFT_REG 2
#define IFR_CB2 3
#define IFR_CB1 4
#define IFR_TIMER1 5
#define IFR_TIMER2 6
#define IFR_IRQ 7

// adb / keyboard / mouse stuff

void adb_handle_state_change(uint8_t old_state, uint8_t new_state);
void adb_request_service_request(uint8_t id);

extern const char *atrap_names[4096];

struct macii_rom_symbols_t {
    uint32_t addr;
    const char *name;
};
extern const struct macii_rom_symbols_t macii_rom_symbols[];

// Emulated Toby Frame Buffer nubus card
void nubus_tfb_init(uint8_t slotnum);
uint32_t nubus_tfb_read_func(uint32_t, uint32_t, uint8_t);
void nubus_tfb_write_func(uint32_t, uint32_t, uint32_t, uint8_t);

// Shoebill Virtual Video Card
void nubus_video_init(void *_ctx, uint8_t slotnum,
                      uint16_t width, uint16_t height, uint16_t scanline_width,
                      double refresh_rate);

uint32_t nubus_video_read_func(const uint32_t rawaddr, const uint32_t size,
                               const uint8_t slotnum);
void nubus_video_write_func(const uint32_t rawaddr, const uint32_t size,
                            const uint32_t data, const uint8_t slotnum);

// debug_server.c
void *debug_cpu_thread (void *arg);



#endif // _SHOEBILL_H

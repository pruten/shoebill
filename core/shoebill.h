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


#if (defined WIN32) || (defined _WIN64)

#ifndef ntohl
/* Assumes that all windows platforms are little endian */
#define ntohs(_v) ({const uint16_t v = (_v); (v>>8) | (v<<8);})
#define ntohl(_v) ({const uint32_t v = (_v); (v>>24) | (((v>>16)&0xff)<<8) | (((v>>8)&0xff)<<16) | ((v&0xff)<<24);})
#define ntohll(_x) ({uint64_t x = (_x); (((uint64_t)ntohl((uint32_t)x))<<32) | ntohl(x>>32);})
#define htons(_v) ntohs(_v)
#define htonl(_v) ntohl(_v)
#endif

#else /* #if (defined WIN32) || (defined _WIN64) */
#include <arpa/inet.h>

#if (defined __APPLE__)

#include <machine/endian.h>
#include <libkern/OSByteOrder.h>
#ifndef ntohll
#define ntohll(x) OSSwapBigToHostInt64(x)
#endif
#else /* #if (defined __APPLE__) */

#if __BYTE_ORDER  == __LITTLE_ENDIAN
#define ntohll(_x) ({uint64_t x = (_x); (((uint64_t)ntohl((uint32_t)x))<<32) | ntohl(x>>32);})
#else
#define ntohll(_x) (_x)
#endif

#endif /* #if (defined __APPLE__) */
#endif /* #if (defined WIN32) || (defined _WIN64) */

#if __BYTE_ORDER  == __LITTLE_ENDIAN
#define fix_endian(x) do { \
    switch (sizeof(x)) { \
        case 1: break; \
        case 2: (x) = ntohs(x); break; \
        case 4: (x) = ntohl(x); break; \
        case 8: (x) = ntohll(x); break; \
        default: assert(!"bogus size"); \
}} while (0)
#else
#define fix_endian(x)
#endif


#define slikely(e) (__builtin_expect(!!(e), 1))
#define sunlikely(e) (__builtin_expect(!!(e), 0))

/*
 * core_api.c stuff
 */


typedef void (*shoebill_pram_callback_t) (void *param, const uint8_t addr, const uint8_t byte);

typedef struct {
    uint32_t ram_size;
    const char *rom_path;
    const char *aux_kernel_path; // almost always "/unix"
    
    _Bool aux_verbose : 1; // Whether to boot A/UX in verbose mode
    _Bool aux_autoconfig : 1; // Whether to run A/UX autoconfig
    _Bool debug_mode : 1; // Whether to enable hacks that debugger depends on
    
    uint16_t root_ctrl, swap_ctrl;
    uint8_t root_drive, swap_drive;
    uint8_t root_partition, swap_partition;
    uint8_t root_cluster;
    
    /* Devices at the 7 possible target SCSI ids */
    struct {
        const char *path;
    } scsi_devices[7]; // scsi id #7 is the initiator (can't be a target)
    
    /* Initialize pram[] with initial PRAM data */
    uint8_t pram[256];
    
    /*
     * This callback is called whenever a PRAM byte is changed.
     * It blocks the CPU, so try to return immediately.
     */
    shoebill_pram_callback_t pram_callback;
    void *pram_callback_param;
    
    char error_msg[8192];
} shoebill_config_t;

typedef struct {
    const uint8_t *buf;
    uint16_t width, height, scan_width, depth;
} shoebill_video_frame_info_t;

/* Take a shoebill_config_t structure and configure the global emulator context */
uint32_t shoebill_initialize(shoebill_config_t *params);

void shoebill_restart (void);

/* Call this after shoebill_initialize() to configure a video card */
uint32_t shoebill_install_video_card(shoebill_config_t *config, uint8_t slotnum,
                                     uint16_t width, uint16_t height);

uint32_t shoebill_install_tfb_card(shoebill_config_t *config, uint8_t slotnum);

/* Call this after shoebill_initialize() to add an ethernet card */
uint32_t shoebill_install_ethernet_card(shoebill_config_t *config, uint8_t slotnum, uint8_t ethernet_addr[6]);

/* Get a video frame from a particular video card */
shoebill_video_frame_info_t shoebill_get_video_frame(uint8_t slotnum, _Bool just_params);

/* Call this after rendering a video frame to send a VBL interrupt */
void shoebill_send_vbl_interrupt(uint8_t slotnum);

/* Call to validate input pram and zap if invalid */
void shoebill_validate_or_zap_pram(uint8_t *pram, _Bool forcezap);

/*
 * These keyboard modifier constants match the ones used
 * in NSEvent shifted right by 16 bits.
 */
enum {
    modCapsLock = 1 << 0,
    modShift    = 1 << 1,
    modControl  = 1 << 2,
    modOption   = 1 << 3,
    modCommand  = 1 << 4
};

void shoebill_key(uint8_t down, uint8_t key);
void shoebill_key_modifier(uint8_t modifier_mask);
void shoebill_mouse_move(int32_t x, int32_t y);
void shoebill_mouse_move_delta (int32_t x, int32_t y);
void shoebill_mouse_click(uint8_t down);

void shoebill_start();
void shoebill_stop();

void slog(const char *fmt, ...);

uint8_t* shoebill_extract_kernel(const char *disk_path, const char *kernel_path, char *error_str, uint32_t *len);



/*
 * Internal shoebill stuff
 */



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
            shoe.sr = (newsr) & 0xf71f; \
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
        #define _tc_enable() (shoe.tc >> 31) // _tc_enable,sre,ps,is are all extracted in shoe.tc_*
        #define _tc_sre() ((shoe.tc >> 25) & 1)
        #define tc_fcl() ((shoe.tc >> 24) & 1)
        #define _tc_ps() ((shoe.tc >> 20) & 0xf)
        #define _tc_is() ((shoe.tc >> 16) & 0xf)
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

/*
 * alloc_pool.c
 */
#define POOL_START_MAGIC 0x231eb4af
#define POOL_END_MAGIC 0xb09f39f1
#define POOL_ALLOC_TYPE 0
#define POOL_CHILD_LINK 1
#define POOL_HEAD 2
typedef struct _alloc_pool_t {
    uint32_t start_magic;
    struct _alloc_pool_t *prev, *next;

    union {
        struct {
            uint64_t size;
        } alloc;
        struct {
            struct _alloc_pool_t *child; // pointer to the child's HEAD
        } child_link;
        struct {
            struct _alloc_pool_t *parent_link; // pointer to the parent's CHILD_LINK
        } head;
    } t;
    
    uint32_t type;
    uint32_t end_magic;
} alloc_pool_t;

void* p_alloc(alloc_pool_t *pool, uint64_t size);
void* p_realloc(void *ptr, uint64_t size);
void p_free(void *ptr);
void p_free_pool(alloc_pool_t *pool);
alloc_pool_t* p_new_pool(alloc_pool_t *parent_pool);

/*
 * redblack.c
 */

typedef uint32_t rb_key_t;

typedef struct _rb_node {
    struct _rb_node *left, *right, *parent;
    rb_key_t key;
    uint8_t is_red : 1;
} rb_node;

typedef struct {
    rb_node *root;
    alloc_pool_t *pool;
    uint32_t sz;
} rb_tree;


rb_tree* rb_new(alloc_pool_t *pool, uint32_t sz);
void rb_free (rb_tree *tree);

uint8_t rb_insert (rb_tree *root, rb_key_t key, void *value, void *old_value);
uint8_t rb_find (rb_tree *tree, rb_key_t key, void *value);
uint8_t rb_index (rb_tree *tree, uint32_t index, rb_key_t *key, void *value);
uint32_t rb_count (rb_tree *tree);


/*
 * coff.c
 */

typedef struct {
    char *name;
    uint32_t value;
    uint16_t scnum, type;
    uint8_t sclass, numaux;
} coff_symbol;

// informed by http://www.delorie.com/djgpp/doc/coff/scnhdr.html
typedef struct {
    char name[8];
    uint32_t p_addr;
    uint32_t v_addr;
    uint32_t sz;
    uint32_t data_ptr;
    uint32_t reloc_ptr;
    uint32_t line_ptr;
    uint16_t num_relocs;
    uint16_t num_lines;
    uint32_t flags;
    
    uint8_t *data;
} coff_section;

// data for this segment appears in the file, but shouldn't be copied into memory
#define coff_copy 0x0010
#define coff_text 0x0020
#define coff_data 0x0040
#define coff_bss  0x0080

typedef struct {
    uint16_t magic;
    uint16_t num_sections;
    uint32_t timestamp;
    uint32_t symtab_offset;
    uint32_t num_symbols;
    uint16_t opt_header_len;
    uint16_t flags;
    uint8_t *opt_header;
    coff_section *sections;
    rb_tree *func_tree;
    coff_symbol *symbols;
    alloc_pool_t *pool;
} coff_file;

coff_symbol* coff_find_func(coff_file *coff, uint32_t addr);
coff_symbol* coff_find_symbol(coff_file *coff, const char *name);

coff_file* coff_parse(uint8_t *buf, uint32_t buflen, alloc_pool_t *parent_pool);
coff_file* coff_parse_from_path(const char *path, alloc_pool_t *parent_pool);
void coff_free(coff_file *coff);
uint32_t be2native (uint8_t **dat, uint32_t bytes);
void print_coff_info(coff_file *coff);


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
    uint8_t ifr, ier, ddrb, ddra, sr, acr, pcr;
    
    uint8_t rega_input, regb_input;
    uint8_t rega_output, regb_output;
    
    uint16_t t1c, t2c, t1l;
    long double t1_last_set, t2_last_set;
    _Bool /*t1_interrupt_enabled,*/ t2_interrupt_enabled; // whether the "one-shot" interrupt can fire
} via_state_t;

#define PRAM_READ 1
#define PRAM_WRITE 2
typedef struct {
    uint8_t data[256];
    uint8_t last_bits;
    
    // FSM
    uint8_t command[8];
    uint8_t byte, mode, command_i, bit_i;
    
    shoebill_pram_callback_t callback;
    void *callback_param;
} pram_state_t;

void init_via_state (uint8_t pram_data[256], shoebill_pram_callback_t callback, void *callback_param);
void init_adb_state();
void init_scsi_bus_state();
void init_iwm_state();

void reset_via_state();
void reset_adb_state();
void reset_scsi_bus_state();
void reset_iwm_state();

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
    // lsb==phase0, msb==L7
    uint8_t latch;
    
    // Registers
    uint8_t data, status, mode, handshake;
} iwm_state_t;

enum scsi_bus_phase {
    BUS_FREE = 0,
    ARBITRATION,
    SELECTION,
    RESELECTION,
    COMMAND,
    DATA_OUT,
    DATA_IN,
    STATUS,
    MESSAGE_IN,
    MESSAGE_OUT
};

typedef struct {
    // Phase
    enum scsi_bus_phase phase;
    
    // Scsi bus signals
    
    uint8_t init_bsy:1; // BSY, driven by initiator
    uint8_t target_bsy:1; // BSY, driven by target
    
    uint8_t sel:1; // SEL, driven by both target and initiator
    
    uint8_t rst:1; // RST, driven by both target and initiator
    
    uint8_t cd:1;  // C/D (control or data), driven by target
    uint8_t io:1;  // I/O, driven by target
    uint8_t ack:1; // ACK, driven by initiator
    uint8_t msg:1; // MSG, driven by target
    uint8_t atn:1; // ATN, driven by initiator
    uint8_t req:1; // REQ, driven by target
    
    uint8_t data; // DB0-7, data lines, driven by both target and initiator
    
    // NCR 5380 registers
    uint8_t initiator_command;
    uint8_t mode;
    uint8_t target_command;
    uint8_t select_enable; // probably not implementing this...
    
    // Arbitration state
    uint8_t init_id; // initiator ID (as a bit mask) (usually 0x80)
    
    // Selection state
    uint8_t target_id; // target ID (as an int [0, 7])
    
    // transfer buffers
    uint8_t buf[512 * 256];
    uint32_t bufi;
    uint32_t in_len, in_i;
    uint32_t out_len, out_i;
    uint32_t write_offset;
    uint8_t status_byte;
    uint8_t message_byte; // only one-byte messages supported for now
    
    // hack
    uint8_t dma_send_written; // Gets set whenever register 5 (start_dma_send) is written to, and cleared randomly.
    // This is because aux 1.1.1 sends an extra byte after sending the write command, and that's not
    // part of the write data. start_dma_send will be written when the data is actually starting.
    uint8_t sent_status_byte_via_reg0; // Gets set when the status byte is red via register 0.
    // This lets us know it's safe to switch to the MESSAGE_IN phase
    
} scsi_bus_state_t;

typedef struct {
    uint8_t r, g, b, a;
} video_ctx_color_t;

typedef struct {
    video_ctx_color_t *temp_buf, *clut;
    uint8_t *rom, *direct_buf;
    
    uint32_t pixels;
    
    uint16_t width, height, scanline_width, line_offset;
    
    uint16_t depth, clut_idx;
} shoebill_card_video_t;

typedef struct {
    uint8_t *direct_buf, *temp_buf, *clut, *rom;
    uint16_t depth, clut_idx, line_offset;
    uint8_t vsync;
} shoebill_card_tfb_t;

typedef struct {
    // Card ROM (4kb)
    uint8_t rom[0x1000];
    
    // Card RAM (16kb buffer, apparently)
    uint8_t ram[0x4000];
    
    // Card MAC address
    uint8_t ethernet_addr[6];
    
    // Card slot number
    uint8_t slotnum;
    
    // -- thread state --
    uint8_t recv_buf[4096], send_buf[4096];
    uint16_t recv_len, send_len;
    _Bool teardown, send_ready;
    
    pthread_t sender_pid, receiver_pid;
    pthread_mutex_t lock, sender_cond_mutex;
    pthread_cond_t sender_cond;
    
    // -- registers --
    
    uint8_t cr; // command register, all pages, read/write
    
    // Page 0 registers
    uint8_t isr; // interrupt status register, read/write
    uint8_t imr; // interrupt mask register, write
    
    uint8_t dcr; // data configuration register (write)
    uint8_t tcr; // transmit configuration register (write)
    uint8_t rcr; // receive configuration register (write)
    
    uint8_t pstart; // receive buffer start pointer (write)
    uint8_t pstop; // receive buffer boundary (write)
    uint8_t bnry; // a different kind of receive buffer boundary (read/write)
    
    uint8_t tpsr; // transmit page start pointer (write)
    uint16_t tbcr; // transmit buffer count register (write)
    
    uint8_t rsr; // receive status register (read)
    
    
    // Page 1 registers (read/write)
    uint8_t mar[8]; // multicast address
    uint8_t par[6]; // physical address
    uint8_t curr; // current page
    
    
    int tap_fd;
} shoebill_card_ethernet_t;

typedef enum {
    card_none = 0, // Empty slot
    card_toby_frame_buffer, // Original Macintosh II video card
    card_shoebill_video, // Fancy 21st-century Shoebill video card
    card_shoebill_ethernet // "Register-compatible" Apple EtherTalk card
} card_names_t;

typedef struct {
    uint32_t (*read_func)(uint32_t, uint32_t, uint8_t);
    void (*write_func)(uint32_t, uint32_t, uint32_t, uint8_t);
    void (*destroy_func)(uint8_t);

    uint8_t slotnum;
    _Bool connected;
    _Bool interrupts_enabled;
    
    void *ctx;
    card_names_t card_type;
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

#define unstop_cpu_thread() do {\
    assert(pthread_mutex_lock(&shoe.cpu_stop_mutex) == 0); \
    assert(pthread_cond_signal(&shoe.cpu_stop_cond) == 0); \
    assert(pthread_mutex_unlock(&shoe.cpu_stop_mutex) == 0); \
} while (0)

typedef struct {
    
    _Bool running;
    
#define SHOEBILL_STATE_STOPPED (1 << 8)
#define SHOEBILL_STATE_RETURN (1 << 9)
    
    // bits 0-6 are CPU interrupt priorities
    // bit 8 indicates that STOP was called
    volatile uint32_t cpu_thread_notifications;
    volatile uint32_t via_thread_notifications;
    
    pthread_mutex_t cpu_thread_lock;
    pthread_mutex_t via_clock_thread_lock; // synchronizes shoebill_start() and the starting of via_clock_thread()
    pthread_mutex_t via_cpu_lock; // synchronizes reads/writes of VIA registers and via_clock_thread()
    
    // The pthread condition/mutex pair for yielding CPU on STOP, and waking up upon receiving an interrupt
    pthread_mutex_t cpu_stop_mutex;
    pthread_cond_t cpu_stop_cond;
    
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
    
#define invalidate_pccache() do {shoe.pccache_use_srp = 2;} while (0)
    uint32_t pccache_use_srp; // 1 -> use srp, 0 -> use crp, other -> pccache is invalid
    uint32_t pccache_logical_page;
    uint8_t *pccache_ptr;
    
    // -- PMMU caching structures ---
#define PMMU_CACHE_KEY_BITS 10
#define PMMU_CACHE_SIZE (1<<PMMU_CACHE_KEY_BITS)
    struct {
        pmmu_cache_entry_t entry[PMMU_CACHE_SIZE];
        uint8_t valid_map[PMMU_CACHE_SIZE / 8];
    } pmmu_cache[2];
    
    // -- EA state --
    uint32_t uncommitted_ea_read_pc; // set by ea_read(). It's the PC that ea_read_commit will set.
    uint64_t dat; // the raw input/output for the transaction
    uint32_t extended_addr; // EA returned by ea_decode_extended()
    uint32_t extended_len; // number of instruction bytes used by ea_decode_extended()
    uint8_t sz; // the size of the EA transaction
    uint8_t mr; // a 6-bit mode/reg pair
    
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
    
    uint32_t tc_pagesize, tc_pagemask; // page size and page mask
    uint8_t tc_ps, tc_is, tc_is_plus_ps, tc_enable, tc_sre; // commonly read bits in shoe.tc
    
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
    
    // fpu_state_t pointer
    // (declared here as a void*, to prevent other files
    //  from needing to include SoftFloat/softfloat.h)
    void *fpu_state;
    
    
    // -- Interrupts/VIA chips --
    
    via_state_t via[2];
    via_clock_t via_clocks;
    adb_state_t adb;
    pram_state_t pram;
    keyboard_state_t key;
    mouse_state_t mouse;
    
    iwm_state_t iwm;
    
    scsi_bus_state_t scsi;
    scsi_device_t scsi_devices[8]; // SCSI devices
    
    nubus_card_t slots[16];
    
    coff_file *coff; // Data/symbols from the unix kernel
    
    pthread_t cpu_thread_pid, via_thread_pid;
    
    debugger_state_t dbg;
    alloc_pool_t *pool;
    
    shoebill_config_t config_copy; // copy of the config structure passed to shoebill_initialize()
} global_shoebill_context_t;

extern global_shoebill_context_t shoe; // declared in cpu.c

// fpu.c functions
void inst_fscc();
void inst_fbcc();
void inst_fsave();
void inst_frestore();
void inst_ftrapcc();
void inst_fdbcc();
void inst_fnop();
void inst_fpu_other();

void dis_fscc();
void dis_fbcc();
void dis_fsave();
void dis_frestore();
void dis_ftrapcc();
void dis_fdbcc();
void dis_fnop();
void dis_fpu_other();
void dis_fmath (uint16_t op, uint16_t ext, char *output);

void fpu_initialize();
void fpu_reset();

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
void throw_frame_zero(uint16_t sr, uint32_t pc, uint16_t vector_num);


// mem.c functions

uint16_t pccache_nextword(uint32_t pc);
uint32_t pccache_nextlong(uint32_t pc);

//void physical_get (void);
typedef void (*physical_get_ptr) (void);
typedef void (*physical_set_ptr) (void);
extern const physical_get_ptr physical_get_jump_table[16];
extern const physical_set_ptr physical_set_jump_table[16];

#define physical_set() physical_set_jump_table[shoe.physical_addr >> 28]()
#define pset(addr, s, val) do { \
    shoe.physical_addr=(addr); \
    shoe.physical_size=(s); \
    shoe.physical_dat=(val); \
    physical_set(); \
} while (0)

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
#define lget(addr, s) lget_fc((addr), (s), (sr_s() ? 5 : 1))

void logical_set (void);
#define lset_fc(addr, s, val, fc) do { \
    shoe.logical_addr=(addr); \
    shoe.logical_size=(s); \
    shoe.logical_dat=(val); \
    shoe.logical_fc = (fc); \
    logical_set();\
} while (0)
#define lset(addr, s, val) lset_fc((addr), (s), (val), sr_s() ? 5 : 1)

typedef void (*_ea_func) (void);
extern const _ea_func ea_read_jump_table[64];
extern const _ea_func ea_read_commit_jump_table[64];
extern const _ea_func ea_write_jump_table[64];
extern const _ea_func ea_addr_jump_table[64];
    
#define ea_read() ea_read_jump_table[shoe.mr]()
#define ea_read_commit() ea_read_commit_jump_table[shoe.mr]()
#define ea_write() ea_write_jump_table[shoe.mr]()
#define ea_addr() ea_addr_jump_table[shoe.mr]()


#define call_ea_read(M, s) {shoe.mr=(M);shoe.sz=(s);ea_read();if sunlikely(shoe.abort) return;}
#define call_ea_write(M, s) {shoe.mr=(M);shoe.sz=(s);ea_write();if sunlikely(shoe.abort) return;}
#define call_ea_read_commit(M, s) {shoe.mr=(M);shoe.sz=(s);ea_read_commit();if sunlikely(shoe.abort) return;}
#define call_ea_addr(M) {shoe.mr=(M);ea_addr();if sunlikely(shoe.abort) return;}

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
void via_raise_interrupt(uint8_t vianum, uint8_t ifr_bit);
void process_pending_interrupt();
void via_read_raw();
void via_write_raw();
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
#define IFR_TIMER2 5
#define IFR_TIMER1 6
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
void nubus_tfb_init(void *_ctx, uint8_t slotnum);
uint32_t nubus_tfb_read_func(uint32_t, uint32_t, uint8_t);
void nubus_tfb_write_func(uint32_t, uint32_t, uint32_t, uint8_t);
shoebill_video_frame_info_t nubus_tfb_get_frame(shoebill_card_tfb_t *ctx,
                                                _Bool just_params);

// Shoebill Virtual Video Card
void nubus_video_init(void *_ctx, uint8_t slotnum,
                      uint16_t width, uint16_t height, uint16_t scanline_width);

uint32_t nubus_video_read_func(const uint32_t rawaddr, const uint32_t size,
                               const uint8_t slotnum);
void nubus_video_write_func(const uint32_t rawaddr, const uint32_t size,
                            const uint32_t data, const uint8_t slotnum);
shoebill_video_frame_info_t nubus_video_get_frame(shoebill_card_video_t *ctx,
                                                  _Bool just_params);

// Apple EtherTalk
void nubus_ethernet_init(void *_ctx, uint8_t slotnum, uint8_t ethernet_addr[6]);
uint32_t nubus_ethernet_read_func(uint32_t, uint32_t, uint8_t);
void nubus_ethernet_write_func(uint32_t, uint32_t, uint32_t, uint8_t);
void nubus_ethernet_destroy_func(uint8_t);

// Sound (Apple Sound Chip)
void sound_dma_write_raw(uint16_t addr, uint8_t sz, uint32_t data);
uint32_t sound_dma_read_raw(uint16_t addr, uint8_t sz);

#endif // _SHOEBILL_H

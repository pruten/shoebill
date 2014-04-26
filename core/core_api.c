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
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include "shoebill.h"
#include "coff.h"
#include "core_api.h"

/*
char *ring, *ring_tmp;
const uint32_t ring_len = 64 * 1024 * 1024;
uint32_t ring_i = 0;

void print_mmu_rp(uint64_t rp)
{
    printf("lu=%u limit=0x%x sg=%u dt=%u addr=0x%08x\n", rp_lu(rp), rp_limit(rp), rp_sg(rp), rp_dt(rp), rp_addr(rp));
}

void printregs()
{
	sprintf(ring_tmp+strlen(ring_tmp), "[d0]%08x  [d1]%08x  [d2]%08x  [d3]%08x\n", shoe.d[0], shoe.d[1], shoe.d[2], shoe.d[3]);
	sprintf(ring_tmp+strlen(ring_tmp), "[d4]%08x  [d5]%08x  [d6]%08x  [d7]%08x\n", shoe.d[4], shoe.d[5], shoe.d[6], shoe.d[7]);
	sprintf(ring_tmp+strlen(ring_tmp), "[a0]%08x  [a1]%08x  [a2]%08x  [a3]%08x\n", shoe.a[0], shoe.a[1], shoe.a[2], shoe.a[3]);
	sprintf(ring_tmp+strlen(ring_tmp), "[a4]%08x  [a5]%08x  [a6]%08x  [a7]%08x\n", shoe.a[4], shoe.a[5], shoe.a[6], shoe.a[7]);
	sprintf(ring_tmp+strlen(ring_tmp), "[pc]%08x  [sr]%c%c%c%c%c%c%c  [tc]%08x\n", shoe.pc,
           sr_s()?'S':'s',
           sr_m()?'M':'m',
           sr_x()?'X':'x',
           sr_n()?'N':'n',
           sr_z()?'Z':'z',
           sr_v()?'V':'v',
           sr_c()?'C':'c',
           shoe.tc
           );
    
    sprintf(ring_tmp+strlen(ring_tmp), "[vbr]%08x\n", shoe.vbr);
    
    //printf("srp: ");
    //print_mmu_rp(shoe.srp);
    
    //printf("crp: ");
    //print_mmu_rp(shoe.crp);
 
    sprintf(ring_tmp+strlen(ring_tmp), "tc: e=%u sre=%u fcl=%u ps=%u is=%u (tia=%u tib=%u tic=%u tid=%u)\n\n",
           tc_enable(), tc_sre(), tc_fcl(), tc_ps(), tc_is(), tc_tia(), tc_tib(), tc_tic(), tc_tid());
    
}

void dump_ring()
{
    uint32_t i = ring_i+1;
    
    while (i != ring_i) {
        fwrite(&ring[i], 1, 1, stdout);
        i = (i+1) % ring_len;
    }
}

void ring_print(const char *str)
{
    uint32_t i;
    for (i=0; str[i]; i++) {
        ring[ring_i] = str[i];
        ring_i = (ring_i+1) % ring_len;
    }
}

void print_pc()
{
    char str[1024];
    uint8_t binary[64];
    uint32_t i;
    uint32_t len;
    const char *name = NULL;
    
    if ((shoe.pc >= 0x40000000) && (shoe.pc < 0x50000000)) {
        uint32_t i, addr = shoe.pc % (shoe.physical_rom_size);
        for (i=0; macii_rom_symbols[i].name; i++) {
            if (macii_rom_symbols[i].addr > addr) {
                break;
            }
            name = macii_rom_symbols[i].name;
        }
    }
    else if (sr_s()) { // these symbols are only meaningful in supervisor mode
        coff_symbol *symb = coff_find_func(shoe.coff, shoe.pc);
        if (symb && strlen(symb->name))
            name = symb->name;
    }
    else
        name = "";
    
    if ((name == NULL) || (name[0] == 0))
        return;
    if (strncmp("scsi", name, 4) != 0)
        return ;
    
    const uint16_t old_abort = shoe.abort;
    shoe.suppress_exceptions = 1;
    
    for (i=0; i<32; i++) {
        binary[i] = (uint8_t) lget(shoe.pc+i, 1);
    }
    
    disassemble_inst(binary, shoe.pc, str, &len);
    
    sprintf(ring_tmp, "*0x%08x %s [ ", shoe.pc, name ? name : "");
    for (i=0; i<len; i+=2) {
        sprintf(ring_tmp+strlen(ring_tmp), "%02x%02x ", binary[i], binary[i+1]);
    }
    sprintf(ring_tmp+strlen(ring_tmp), "]  %s\n", str);
    
    printregs();
    
    for (i=0; ring_tmp[i]; i++) {
        ring[ring_i] = ring_tmp[i];
        ring_i = (ring_i+1) % ring_len;
    }
    
    shoe.abort = old_abort;
    shoe.suppress_exceptions = 0;
    
}
*/

void shoebill_start()
{
    pthread_mutex_unlock(&shoe.cpu_thread_lock);
    pthread_mutex_unlock(&shoe.via_clock_thread_lock);
}

void *_cpu_thread (void *arg) {
     
    pthread_mutex_lock(&shoe.cpu_thread_lock);
    
    // ring = calloc(ring_len, 1);
    // ring_tmp = malloc(128 * 1024);
    
    while (1) {
        if (shoe.cpu_thread_notifications) {
            
            // If there's an interrupt pending
            if (shoe.cpu_thread_notifications & 0xff) {
                // process_pending_interrupt() may clear SHOEBILL_STATE_STOPPED
                process_pending_interrupt();
            }
            
            if (shoe.cpu_thread_notifications & SHOEBILL_STATE_STOPPED) {
                continue; // FIXME: yield or block on a condition variable here
            }
        }
        // print_pc();
        cpu_step();
    }
}

/*static void _cpu_loop_debug()
{
    while (1) {
        if (shoe.cpu_thread_notifications) {
            // I think we can safely ignore "stop" instructions for A/UX in debug mode
            shoe.cpu_thread_notifications &= ~SHOEBILL_STATE_STOPPED;
            
            if (shoe.cpu_thread_notifications & 0xff) {
                process_pending_interrupt();
            }
            
        }
    }
}*/
 
/*void shoebill_cpu_stepi (void)
{
    if (shoe.cpu_mode != CPU_MODE_FREEZE)
        return ;
    
    shoe.cpu_mode = CPU_MODE_STEPI;
    pthread_mutex_unlock(&shoe.cpu_freeze_lock);
    
    // Just spin until the instruction completes - it should be quick
    while (shoe.cpu_mode != CPU_MODE_STEPI_COMPLETE)
        pthread_yield_np();
    
    pthread_mutex_lock(&shoe.cpu_freeze_lock);
    shoe.cpu_mode = CPU_MODE_FREEZE;
}

void shoebill_cpu_freeze (void)
{
    pthread_mutex_lock(&shoe.cpu_freeze_lock);
    shoe.cpu_mode = CPU_MODE_FREEZE;
    shoe.cpu_thread_notifications &= SHOEBILL_STATE_SWITCH_MODE;
    
    while (shoe.cpu_thread_notifications & SHOEBILL_STATE_SWITCH_MODE);
        pthread_yield_np();
}*/

/*
 * The A/UX bootloader blasts this structure into memory
 * somewhere before jumping into the kernel to commmunicate
 * stuff like: which scsi device/partition is root?
 */
struct __attribute__ ((__packed__)) kernel_info {
    // Auto data
    uint32_t auto_magic;
    uint32_t auto_id[16];
    uint32_t auto_version[16];
    uint32_t auto_command;
    
    uint16_t root_ctrl;
    uint8_t root_drive;
    uint8_t root_cluster;
    
    struct __attribute__ ((__packed__)) sect_info {
        uint32_t vstart;
        uint32_t pstart;
        uint32_t size;
    } si[3];
    
    uint16_t machine_type; // Not the same as gestalt IDs!
    uint32_t drive_queue_offset;
    uint16_t ki_flags;
    uint8_t ki_version; // always 1
    uint8_t root_partition;
    uint16_t swap_ctrl;
    uint8_t swap_drive;
    uint8_t swap_partition;
    
    // A series of DrvQEl (drive queue elements) follow this structure
};

/* Inside Macintosh: Files 2-85 throughtfully provides this information
 * on the secret internal flags:
 * 
 * The File Manager also maintains four flag bytes preceding each drive queue element.
 * These bytes contain the following information:
 * Byte (from low address to high address)
 * 0    Bit 7=1 if the volume on the drive is locked
 * 1    0 if no disk in drive; 1 or 2 if disk in drive; 8 if nonejectable disk in drive; $FC–$FF if
 *      disk was ejected within last 1.5 seconds; $48 if disk in drive is nonejectable but driver
 *      wants a call
 * 2    Used internally during system startup
 * 3    Bit 7=0 if disk is single-sided
 *      You can read these flags by subtracting 4 bytes from the beginning of a drive queue element,
 *      as illustrated in Listing 2-11.
 */

struct __attribute__ ((__packed__)) drive_queue_element {
    /* Secret internal flags */
    uint8_t unknown_1 : 7;
    uint8_t is_locked : 1; // 1 if locked, 0 -> unlocked
    uint8_t is_in_drive;
    uint8_t unknown_2;
    uint8_t unknown_3 : 7;
    uint8_t double_sided : 1; // 0 if single-sided, presumably 1 if HDD or double-sided
    /* --- */
    
    uint32_t next_offset;
    int16_t qType; // 1 -> both dQDrvSz and dQDrvSz2 are used
    int16_t dQDrive; // drive number (scsi target ID)
    int16_t dQRefNum; // Driver reference number (I'm thinking -33 is the right value here)
    int16_t dQFSID; // Filesystem identifier (0 is best, I think)
    uint16_t dQDrvSz; // low 16 bits of numblocks
    uint16_t dQDrvSz2; // high 16 bits of numblocks
};

/*
 * Initialize the Macintosh lomem variables, or some of them anyways.
 * Debugging weird errors revealed that A/UX actually cares about
 * some of them.
 */
static void _init_macintosh_lomem_globals (const uint32_t offset)
{
    /*
     * FIXME: explain what I'm doing here
     */
    
    uint32_t i;
    
    // Fill the entire lomem space with 0xBB to make debugging easier
    // if somebody tries to read an uninitialized lomem value
//    for (i=0; i<0x4000; i++)
//        pset(offset + i, 1, 0xBB);
    
    // There are the lomem globals that matter, apparently
    
    pset(offset+0x12f, 1, 0x02);       // CPUFlag = 0x02 (MC68020)
    pset(offset+0x31a, 4, 0x00ffffff); // Lo3Bytes (always 0x00ffffff)
    pset(offset+0x28e, 2, 0x3fff);     // ROM85 (always 0x3fff, I think?)
    // universal info ptr. is allowed to be null on Mac II, (I THINK)
    pset(offset+0xdd8, 4, 0);
    pset(offset+0x1d4, 4, 0x50000000); // VIA (via1 ptr)
    pset(offset+0x1d8, 4, 0x50004000); // SCC
    
    #define hwCbSCSI (1<<15)
    #define hwCbClock (1<<14)
    #define hwCbExPRAM (1<<13)
    #define hwCbFPU (1<<12)
    #define hwCbMMU (1<<11)
    #define hwCbADB (1<<10)
    #define hwCbAUX (1<<9) /* not sure if I need to set this one */
    const uint16_t HWCfgFlags = hwCbSCSI | hwCbClock | hwCbFPU | hwCbMMU | hwCbADB;
    pset(offset+0xb22, 2, HWCfgFlags); // HWCfgFlags
}


/*
 * Setup and blast the kernel_info structure into memory
 * before booting A/UX.
 * Side-effects: sets CPU registers d0 and a0
 */
static void _init_kernel_info(shoebill_control_t *control, scsi_device_t *disks, uint32_t offset)
{
    struct kernel_info ki, *p;
    uint32_t i, p_addr;
    
    p_addr = offset + 0x3c00;
    p = (struct kernel_info*) (uint64_t)p_addr;
    
    /*
     * On boot, the kernel looks for this magic constant in d0 to see whether
     * the bootloader setup a kernel_info structure.
     */
    shoe.d[0] = 0x536d7201; // "Smr\1"? Something version 1?
    shoe.a[0] = (uint32_t)p; // Address of the kernel_info structure
    
    /* ----- Setup kernel info structure ------ */
    
    ki.auto_magic = 0x50696773; // 'Pigs' (Pigs in space?)
    
    for (i=0; i<16; i++) {
        ki.auto_id[i] = 0x0000ffff;
        ki.auto_version[i] = 0;
    }
    
    // FIXME: I need to stick the auto_id for each nubus card in here
    // ki.auto_id[0xb] = 0x5; // Macintosh II video card has an auto_id of 5 (I guess?)
    
    ki.auto_command = control->aux_autoconfig; // AUTO_NONE/AUTO_CONFIG
    
    /*
     * Note: ctrl -> SCSI controller chip
     *       drive -> SCSI Target ID
     *       partition -> partition
     *       cluster -> A set of partitions on a given drive
     *                  (Used by escher/eschatology somehow)
     */
    
    ki.root_ctrl = control->root_ctrl;
    ki.swap_ctrl = control->swap_ctrl;
    
    ki.root_drive = control->root_drive;
    ki.swap_drive = control->swap_drive;
    
    ki.root_partition = control->root_partition;
    ki.swap_partition = control->swap_partition;
    
    ki.root_cluster = control->root_cluster;
    
    // Find the text, data, and bss segments in the kernel
    for (i = 0; i < shoe.coff->num_sections; i++) {
        coff_section *s = &shoe.coff->sections[i];
        uint8_t sect;
        
        if (strcmp(s->name, ".text") == 0)
            sect = 0;
        else if (strcmp(s->name, ".data") == 0)
            sect = 1;
        else if (strcmp(s->name, ".bss") == 0)
            sect = 2;
        else
            continue;
        
        ki.si[sect].vstart = s->v_addr;
        ki.si[sect].pstart = s->p_addr;
        ki.si[sect].size = s->sz;
    }
    
    ki.machine_type = 4; // Macintosh II?
    
    // +4 because the DrvQEl structure has a hidden "flags" field 4 bytes below the pointer
    ki.drive_queue_offset = sizeof(struct kernel_info) + 4;
    
    ki.ki_flags = control->aux_verbose;
    ki.ki_version = 1;
    
    /* ----- Copy ki into memory ----- */
#define ki_pset(_f) pset((uint32_t)&p->_f, sizeof(p->_f), ki._f)
    ki_pset(auto_magic);
    for (i=0; i<16; i++) {
        ki_pset(auto_id[i]);
        ki_pset(auto_version[i]);
    }
    ki_pset(auto_command);
    
    ki_pset(root_ctrl);
    ki_pset(root_drive);
    ki_pset(root_cluster);
    
    for (i=0; i<3; i++) {
        ki_pset(si[i].vstart);
        ki_pset(si[i].pstart);
        ki_pset(si[i].size);
    }
    
    ki_pset(machine_type);
    ki_pset(drive_queue_offset);
    ki_pset(ki_flags);
    ki_pset(ki_version);
    ki_pset(root_partition);
    ki_pset(swap_ctrl);
    ki_pset(swap_drive);
    ki_pset(swap_partition);
    
    /* ---- Copy DrvQEl elements into memory ---- */
    // FIXME: btw, this is probably wrong. DrvQEl elements are supposed to be partitions, I think
    
    uint32_t addr = p_addr + sizeof(struct kernel_info);
    uint32_t num_disks = 0;
    for (i=0; i<7; i++)
        if (disks[i].f) num_disks++;
        
    for (i=0; i<7; i++) {
        if (disks[i].f == NULL)
            continue;
        
        num_disks--;
        
        pset(addr, 4, 0x00080000); addr += 4; // magic internal flags (non-ejectable)
        pset(addr, 4, num_disks ? 0x14 : 0); addr += 4; // offset to next drvqel
        pset(addr, 2, 1); addr += 2; // Use both dQDrvSzs
        pset(addr, 2, i | 8); addr += 2; // SCSI ID (with bit 3 always set?)
        // pset(addr, 2, 0xffdf); addr += 2; // dQRefNum: not sure if this is right
        pset(addr, 2, (i | 0x20) ^ 0xffff); addr += 2; // dQRefNum: not sure if this is right
        pset(addr, 2, 0); addr += 2; // FSID
        pset(addr, 2, disks[i].num_blocks & 0xffff); addr += 2; // low 16 bits of num_blocks
        pset(addr, 2, disks[i].num_blocks >> 16); addr += 2; // high bits
    }
}

/*
 * Compute the checksum for a Macintosh ROM file
 */
static uint32_t _compute_rom_checksum (const uint8_t *rom, const uint32_t len)
{
    uint32_t i, checksum;
    for (i=4, checksum=0; i < len; i+=2) {
        const uint16_t word = (rom[i]<<8) + rom[i+1];
        checksum += word;
    }
    return checksum;
}

static uint32_t _load_rom (shoebill_control_t *control, uint8_t **_rom_data, uint32_t *_rom_size)
{
    uint32_t i, rom_size;
    uint8_t *rom_data = (uint8_t*)malloc(64 * 1024);
    FILE *f = fopen(control->rom_path, "r");
    
    if (f == NULL) {
        sprintf(control->error_msg, "Couldn't open rom path [%s]\n", control->rom_path);
        goto fail;
    }
    
    for (rom_size = 0; rom_size < (2*1024*1024); rom_size += (64*1024)) {
        rom_data = (uint8_t*)realloc(rom_data, rom_size + (64*1024));
        if (fread(rom_data + rom_size, 64*1024, 1, f) != 1)
            break;
    }
    
    // Rom_size had better be a power of two
    if ((rom_size & (rom_size - 1)) != 0) {
        sprintf(control->error_msg,
                "Rom is probably corrupt (size not a power of two %u)\n",
                rom_size);
        goto fail;
    }
    
    // Check the checksum
    const uint32_t computed_checksum = _compute_rom_checksum(rom_data, rom_size);
    const uint32_t purported_checksum = ntohl(*(uint32_t*)rom_data);
    if (computed_checksum != purported_checksum) {
        sprintf(control->error_msg,
                "Rom checksum doesn't match (computed=0x%08x, expected=0x%08x)\n",
                computed_checksum, purported_checksum);
        goto fail;
    }
    
    rom_data = realloc(rom_data, rom_size);
    assert(rom_data);
    
    *_rom_size = rom_size;
    *_rom_data = rom_data;
    
    fclose(f);
    return 1;
    
fail:
    if (rom_data) free(rom_data);
    if (f) fclose(f);
    return 0;
}

static uint32_t _open_disk_images (shoebill_control_t *control, scsi_device_t *disks)
{
    uint32_t i;
    
    for (i=0; i<8; i++) {
        shoe.scsi_devices[i].scsi_id = i;
        shoe.scsi_devices[i].block_size = 0;
        shoe.scsi_devices[i].num_blocks = 0;
        shoe.scsi_devices[i].image_path = "dummy";
        shoe.scsi_devices[i].f = NULL;
    }
    
    for (i=0; i<7; i++) {
        struct stat stat_buf;
        const char *path = control->scsi_devices[i].path;
        
        if (!path) continue;
        
        FILE *f = fopen(path, "r+");
        
        if (f == NULL) {
            sprintf(control->error_msg, "Couldn't open scsi id #%u disk [%s]\n", i, path);
            goto fail;
        }
        
        disks[i].scsi_id = i;
        disks[i].f = f;
        disks[i].image_path = path;
        
        if (fstat(fileno(f), &stat_buf)) {
            sprintf(control->error_msg, "Couldn't fstat() scsi id #%u disk [%s]\n", i, path);
            goto fail;
        }
        else if (stat_buf.st_size % 512) {
            sprintf(control->error_msg, "Not aligned to 512 byte blocks: [%s]\n", path);
            goto fail;
        }
        
        disks[i].block_size = 512;
        disks[i].num_blocks = stat_buf.st_size / 512;
    }
    
    return 1;
    
fail:
    for (i=0; i<7; i++)
        if (disks[i].f) fclose(disks[i].f);
    memset(disks, 0, 7 * sizeof(scsi_device_t));
    return 0;
}

static uint32_t _load_aux_kernel(shoebill_control_t *control, coff_file *coff, uint32_t *_pc)
{
    uint32_t j, i, pc = 0xffffffff;
    for (i = 0; i < coff->num_sections; i++) {
        coff_section *s = &coff->sections[i];
        
        // Don't load a "copy" segment
        if (s->flags & coff_copy)
            continue;
        
        if ((s->flags & coff_text) || (s->flags & coff_data)) {
            /* copy text or data section */
            
            for (j = 0; j < s->sz; j++)
                pset(s->p_addr+j, 1, s->data[j]);
            
            if (strcmp(s->name, "pstart") == 0)
                pc = s->p_addr;
        }
        else if (s->flags & coff_bss) {
            /* Create an empty .bss segment */
            
            for (j = 0; j < s->sz; j++)
                pset(s->p_addr+j, 1, 0);
        }
    }
    
    if (pc == 0xffffffff) {
        sprintf(control->error_msg, "This unix kernel doesn't contain a pstart segment\n");
        return 0;
    }

    *_pc = pc; // Entry point to the kernel
    
    return 1;
    
fail:
    return 0;
}

uint32_t shoebill_install_video_card(shoebill_control_t *control, uint8_t slotnum,
                                     uint16_t width, uint16_t height,
                                     double refresh_rate)
{
    shoebill_card_video_t *ctx = &control->slots[slotnum].card.video;
    
    if (control->slots[slotnum].card_type != card_none) {
        sprintf(control->error_msg, "This slot (%u) already has a card\n", slotnum);
        return 0;
    }
    
    // Make sure the scanline width is a multiple of 32 pixels, and is at least 32 pixels
    // beyond the end of the display. If scanline_width==width, A/UX 2.0 will wrap the mouse around
    // the edge of the screen.
    uint32_t scanline_width = width + (32 - (width % 32)) + 32;
    
    scanline_width = width; // FIXME: undo this
    
    shoe.slots[slotnum].connected = 1;
    shoe.slots[slotnum].read_func = nubus_video_read_func;
    shoe.slots[slotnum].write_func = nubus_video_write_func;
    shoe.slots[slotnum].interrupt_rate = refresh_rate;
    shoe.slots[slotnum].last_fired = 0;
    shoe.slots[slotnum].interrupts_enabled = 1;
    nubus_video_init(ctx, slotnum, width, height, scanline_width, refresh_rate);
    return 1;
}

/*
 * Given a config_control_t structure, configure and initialize
 * the emulator.
 * This is the first function you should call if you're writing an
 * interface to Shoebill.
 */
uint32_t shoebill_initialize(shoebill_control_t *control)
{
    uint32_t i, j, pc = 0xffffffff;
    coff_file *coff = NULL;
    scsi_device_t disks[8];
    uint8_t *rom_data = NULL, *kernel_data = NULL;
    uint32_t rom_size = 0, kernel_size;
    
    
    memset(&disks[0], 0, 8 * sizeof(scsi_device_t));
    memset(&shoe, 0, sizeof(global_shoebill_context_t));
    
    shoe.pool = p_new_pool();
    
    fpu_setup_jump_table();
    
    // Try to load the ROM
    if (control->rom_path == NULL) {
        sprintf(control->error_msg, "No rom file specified\n");
        goto fail;
    }
    else if (!_load_rom(control, &rom_data, &rom_size))
            goto fail;
    
    // Try to load the A/UX kernel
    if (control->aux_kernel_path == NULL) {
        sprintf(control->error_msg, "No A/UX kernel specified\n");
        goto fail;
    }
    else if (!control->scsi_devices[0].path || strlen((char*)control->scsi_devices[0].path)==0) {
        sprintf(control->error_msg, "The root A/UX disk needs to be at scsi ID 0\n");
        goto fail;
    }
    
    // Load the kernel from the disk at scsi id #0
    kernel_data = shoebill_extract_kernel((char*)control->scsi_devices[0].path,
                                 control->aux_kernel_path,
                                 control->error_msg,
                                 &kernel_size);
    if (!kernel_data)
        goto fail;
    
    coff = coff_parse(kernel_data, kernel_size);
    free(kernel_data);
    
    if (coff == NULL) {
        sprintf(control->error_msg, "Can't open that A/UX kernel [%s]\n",
                control->aux_kernel_path);
        goto fail;
    }
    shoe.coff = coff;
    
    // Try to open the disk images
    if (!_open_disk_images(control, disks))
        goto fail;
    
    // Allocate and configure the rom and memory space
    
    if (control->ram_size < (1024*1024)) {
        sprintf(control->error_msg, "%u bytes is too little ram\n", control->ram_size);
        goto fail;
    }
    
    shoe.physical_rom_size = rom_size;
    shoe.physical_rom_base = valloc(rom_size+8); // +8 because of physical_get hack
    memcpy(shoe.physical_rom_base, rom_data, rom_size);
    free(rom_data);
    rom_data = NULL;
    
    shoe.physical_mem_size = control->ram_size;
    shoe.physical_mem_base = valloc(control->ram_size+8); // +8 because of physical_get hack
    memset(shoe.physical_mem_base, 0, shoe.physical_mem_size);
    
    // Initialize Macintosh lomem variables that A/UX actually cares about
    
    #define AUX_LOMEM_OFFSET 0x50000
    _init_macintosh_lomem_globals(AUX_LOMEM_OFFSET);
    
    // Initialize A/UX's kernel_info structure
    
    _init_kernel_info(control, disks, AUX_LOMEM_OFFSET);
    
    // Load A/UX kernel COFF segments into memory (returns PC, the entry point into the kernel)
    
    if (!_load_aux_kernel(control, coff, &pc))
        goto fail;
    
    /*
     * Load it all into the internal global shoebill state
     * (Can't fail after this point)
     */
    
    // FIXME: Don't do this! Rewrite the via timers!
    gettimeofday(&shoe.start_time, NULL);
    shoe.total_ticks = 0;
    
    // Put the adb chip in state 3 (idle)
    shoe.adb.state = 3;
    pthread_mutex_init(&shoe.adb.lock, NULL);
    
    set_sr(0x2000);
    shoe.pc = pc;
    memcpy(shoe.scsi_devices, disks, 8 * sizeof(scsi_device_t));
    
    pthread_mutex_init(&shoe.cpu_thread_lock, NULL);
    pthread_mutex_init(&shoe.via_clock_thread_lock, NULL);
    
    pthread_mutex_lock(&shoe.cpu_thread_lock);
    pthread_mutex_lock(&shoe.via_clock_thread_lock);
    
    pthread_create(&control->cpu_thread_pid, NULL, _cpu_thread, NULL);
    // pthread_create(&control->cpu_thread_pid, NULL, debug_cpu_thread, NULL);
    pthread_create(&control->via_thread_pid, NULL, via_clock_thread, NULL);
    
    return 1;
    
fail:
    if (rom_data) free(rom_data);
    
    for (i=0; i<7; i++)
        if (disks[i].f) fclose(disks[i].f);
    
    if (shoe.physical_rom_base) free(shoe.physical_rom_base);
    if (shoe.physical_mem_base) free(shoe.physical_mem_base);
    
    // No way to free *coff yet
    return 0;
}

static void _send_key(uint8_t code)
{
    if ((shoe.key.key_i+1) < KEYBOARD_STATE_MAX_KEYS) {
        shoe.key.keys[shoe.key.key_i].code_a = code;
        shoe.key.keys[shoe.key.key_i].code_b = 0xff;
        shoe.key.key_i++;
    }
}

void shoebill_key(uint8_t down, uint8_t key)
{
    const uint8_t down_mask = down ? 0 : 0x80;
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    _send_key(key | down_mask);
    
    adb_request_service_request(3);
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_key_modifier(uint8_t modifier_mask)
{
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    const uint8_t changed_mask = shoe.key.last_modifier_mask ^ modifier_mask;
    shoe.key.last_modifier_mask = modifier_mask;
    
    if (changed_mask & modShift) {
        _send_key(((modifier_mask & modShift) ? 0 : 0x80) | 0x38);
    }
    if (changed_mask & modControl) {
        _send_key(((modifier_mask & modControl) ? 0 : 0x80) | 0x36);
    }
    if (changed_mask & modOption) {
        _send_key(((modifier_mask & modOption) ? 0 : 0x80) | 0x3a);
    }
    if (changed_mask & modCommand) {
        _send_key(((modifier_mask & modCommand) ? 0 : 0x80) | 0x37);
    }
    
    
    adb_request_service_request(3);
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_mouse_move(int32_t x, int32_t y)
{
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    int32_t delta_x = x - shoe.mouse.old_x;
    int32_t delta_y = y - shoe.mouse.old_y;
    
    shoe.mouse.old_x = x;
    shoe.mouse.old_y = y;
    
    shoe.mouse.delta_x += delta_x;
    shoe.mouse.delta_y += delta_y;
    
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_mouse_move_delta (int32_t x, int32_t y)
{
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    shoe.mouse.delta_x += x;
    shoe.mouse.delta_y += y;
    
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_mouse_click(uint8_t down)
{
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    shoe.mouse.button_down = (down != 0);
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);
    
    pthread_mutex_unlock(&shoe.adb.lock);
}


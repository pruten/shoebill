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
#include <signal.h>
#include <stdarg.h>
#include <time.h>
#include "../core/shoebill.h"


void shoebill_start()
{
    shoe.running = 1;
    pthread_mutex_unlock(&shoe.via_clock_thread_lock);
    pthread_mutex_unlock(&shoe.cpu_thread_lock);
}

void shoebill_stop()
{
    uint32_t i;
    
    // Tear down the CPU / timer threads
    shoe.cpu_thread_notifications |= SHOEBILL_STATE_RETURN;
    shoe.via_thread_notifications = SHOEBILL_STATE_RETURN;
    
    pthread_mutex_lock(&shoe.via_clock_thread_lock);
    pthread_mutex_unlock(&shoe.via_clock_thread_lock);
    pthread_join(shoe.via_thread_pid, NULL);
    pthread_mutex_destroy(&shoe.via_clock_thread_lock);
    
    // wake up the CPU thread if it was STOPPED
    unstop_cpu_thread();
    
    pthread_join(shoe.cpu_thread_pid, NULL);
    pthread_mutex_destroy(&shoe.cpu_thread_lock);
    
    pthread_mutex_destroy(&shoe.via_cpu_lock);
    
    pthread_mutex_destroy(&shoe.cpu_stop_mutex);
    pthread_cond_destroy(&shoe.cpu_stop_cond);
    
    shoe.running = 0;
    
    // Destroy all the nubus cards
    for (i=0; i<15; i++)
        if (shoe.slots[i].destroy_func)
            shoe.slots[i].destroy_func(i);
    
    // Close all the SCSI disk images
    for (i=0; i<8; i++) {
        if (shoe.scsi_devices[i].f)
            fclose(shoe.scsi_devices[i].f);
        shoe.scsi_devices[i].f = NULL;
    }
    
    // Free the alloc pool
    p_free_pool(shoe.pool);
    
    // Zero the global context
    memset(&shoe, 0, sizeof(shoe));
}

static void _await_interrupt (void)
{
    struct timeval now;
    struct timespec later;
    
    assert(pthread_mutex_lock(&shoe.cpu_stop_mutex) == 0);
    
    gettimeofday(&now, NULL);
    later.tv_sec = now.tv_sec;
    later.tv_nsec = (now.tv_usec * 1000) + (1000000000 / 60);
    if (later.tv_nsec >= 1000000000) {
        later.tv_nsec -= 1000000000;
        later.tv_sec++;
    }
    
    /* Only wait for (1/60) seconds - an interrupt should have fired by then */
    pthread_cond_timedwait(&shoe.cpu_stop_cond,
                           &shoe.cpu_stop_mutex,
                           &later);
    assert(pthread_mutex_unlock(&shoe.cpu_stop_mutex) == 0);
}

void *_cpu_thread (void *arg)
{
    pthread_mutex_lock(&shoe.cpu_thread_lock);
    
    while (1) {
        if sunlikely(shoe.cpu_thread_notifications) {
            
            // If there's an interrupt pending
            if slikely(shoe.cpu_thread_notifications & 0xff) {
                // process_pending_interrupt() may clear SHOEBILL_STATE_STOPPED
                process_pending_interrupt();
            }
            
            if sunlikely(shoe.cpu_thread_notifications & SHOEBILL_STATE_RETURN) {
                pthread_mutex_unlock(&shoe.cpu_thread_lock);
                return NULL;
            }
            
            if (shoe.cpu_thread_notifications & SHOEBILL_STATE_STOPPED) {
                _await_interrupt();
                continue;
            }
        }
        cpu_step();
    }
}

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

/* Inside Macintosh: Files 2-85 thoughtfully provides this information
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
    pset(offset+0xb22, 2, 0xfc00); // HWCfgFlags
}


/*
 * Setup and blast the kernel_info structure into memory
 * before booting A/UX.
 * Side-effects: sets CPU registers d0 and a0
 */
static void _init_kernel_info(shoebill_config_t *config, scsi_device_t *disks, uint32_t offset)
{
    struct kernel_info ki;
    const uint32_t ki_addr = offset + 0x3c00;
    uint32_t i;
    
    /*
     * On boot, the kernel looks for this magic constant in d0 to see whether
     * the bootloader setup a kernel_info structure.
     */
    shoe.d[0] = 0x536d7201; // "Smr\1"? Something version 1?
    shoe.a[0] = (uint32_t)ki_addr; // Address of the kernel_info structure
    
    /* ----- Setup kernel info structure ------ */
    
    ki.auto_magic = 0x50696773; // 'Pigs' (Pigs in space?)
    
    for (i=0; i<16; i++) {
        ki.auto_id[i] = 0x0000ffff;
        ki.auto_version[i] = 0;
    }
    
    // FIXME: I need to stick the auto_id for each nubus card in here
    ki.auto_id[9] = 0x0050; // Macintosh II video card has an auto_id of 5 (I guess?)
    
    ki.auto_command = config->aux_autoconfig; // AUTO_NONE/AUTO_CONFIG
    
    /*
     * Note: ctrl -> SCSI controller chip
     *       drive -> SCSI Target ID
     *       partition -> partition
     *       cluster -> A set of partitions on a given drive
     *                  (Used by escher/eschatology somehow)
     */
    
    ki.root_ctrl = config->root_ctrl;
    ki.swap_ctrl = config->swap_ctrl;
    
    ki.root_drive = config->root_drive;
    ki.swap_drive = config->swap_drive;
    
    ki.root_partition = config->root_partition;
    ki.swap_partition = config->swap_partition;
    
    ki.root_cluster = config->root_cluster;
    
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
    
    ki.ki_flags = config->aux_verbose;
    ki.ki_version = 1;
    
    /* ----- Copy ki into memory ----- */
    fix_endian(ki.auto_magic);
    for (i=0; i<16; i++) {
        fix_endian(ki.auto_id[i]);
        fix_endian(ki.auto_version[i]);
    }
    fix_endian(ki.auto_command);
    
    fix_endian(ki.root_ctrl);
    fix_endian(ki.root_drive);
    fix_endian(ki.root_cluster);
    
    for (i=0; i<3; i++) {
        fix_endian(ki.si[i].vstart);
        fix_endian(ki.si[i].pstart);
        fix_endian(ki.si[i].size);
    }
    
    fix_endian(ki.machine_type);
    fix_endian(ki.drive_queue_offset);
    fix_endian(ki.ki_flags);
    fix_endian(ki.ki_version);
    fix_endian(ki.root_partition);
    fix_endian(ki.swap_ctrl);
    fix_endian(ki.swap_drive);
    fix_endian(ki.swap_partition);
    
    for (i=0; i<sizeof(struct kernel_info); i++)
        pset(ki_addr+i, 1, ((uint8_t*)&ki)[i]);
    
    /* ---- Copy DrvQEl elements into memory ---- */
    // FIXME: btw, this is probably wrong. DrvQEl elements are supposed to be partitions, I think
    
    uint32_t addr = ki_addr + sizeof(struct kernel_info);
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

static uint32_t _load_rom (shoebill_config_t *config, uint8_t **_rom_data, uint32_t *_rom_size)
{
    uint32_t i, rom_size;
    uint8_t *rom_data = (uint8_t*)p_alloc(shoe.pool, 64 * 1024);
    FILE *f = fopen(config->rom_path, "rb");
    
    if (f == NULL) {
        sprintf(config->error_msg, "Couldn't open rom path [%s]\n", config->rom_path);
        goto fail;
    }
    
    for (rom_size = 0; rom_size < (2*1024*1024); rom_size += (64*1024)) {
        rom_data = (uint8_t*)p_realloc(rom_data, rom_size + (64*1024));
        if (fread(rom_data + rom_size, 64*1024, 1, f) != 1)
            break;
    }
    
    // Rom_size had better be a power of two
    if ((rom_size & (rom_size - 1)) != 0) {
        sprintf(config->error_msg,
                "Rom is probably corrupt (size not a power of two %u)\n",
                rom_size);
        goto fail;
    }
    
    // Check the checksum
    const uint32_t computed_checksum = _compute_rom_checksum(rom_data, rom_size);
    const uint32_t purported_checksum = ntohl(*(uint32_t*)rom_data);
    if (computed_checksum != purported_checksum) {
        sprintf(config->error_msg,
                "Rom checksum doesn't match (computed=0x%08x, expected=0x%08x)\n",
                computed_checksum, purported_checksum);
        goto fail;
    }
    
    // rom_data = p_realloc(rom_data, rom_size);
    assert(rom_data);
    
    *_rom_size = rom_size;
    *_rom_data = rom_data;
    
    fclose(f);
    return 1;
    
fail:
    if (rom_data) p_free(rom_data);
    if (f) fclose(f);
    return 0;
}

static uint32_t _open_disk_images (shoebill_config_t *config, scsi_device_t *disks)
{
    uint32_t i;
    
    for (i=0; i<8; i++) {
        disks[i].scsi_id = i;
        disks[i].block_size = 0;
        disks[i].num_blocks = 0;
        disks[i].image_path = "dummy";
        disks[i].f = NULL;
    }
    
    for (i=0; i<7; i++) {
        struct stat stat_buf;
        const char *path = config->scsi_devices[i].path;
        char *tmp;
        
        if (!path) continue;
        
        FILE *f = fopen(path, "r+b");
        
        if (f == NULL) {
            sprintf(config->error_msg, "Couldn't open scsi id #%u disk [%s]\n", i, path);
            goto fail;
        }
        
        disks[i].scsi_id = i;
        disks[i].f = f;
        tmp = p_alloc(shoe.pool, strlen(path) + 1);
        strcpy(tmp, path);
        disks[i].image_path = tmp;
                            
        
        if (fstat(fileno(f), &stat_buf)) {
            sprintf(config->error_msg, "Couldn't fstat() scsi id #%u disk [%s]\n", i, path);
            goto fail;
        }
        else if (stat_buf.st_size % 512) {
            sprintf(config->error_msg, "Not aligned to 512 byte blocks: [%s]\n", path);
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

static uint32_t _load_aux_kernel(shoebill_config_t *config, coff_file *coff, uint32_t *_pc)
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
        sprintf(config->error_msg, "This unix kernel doesn't contain a pstart segment\n");
        return 0;
    }

    *_pc = pc; // Entry point to the kernel
    
    return 1;
    
fail:
    return 0;
}

uint32_t shoebill_install_video_card(shoebill_config_t *config, uint8_t slotnum,
                                     uint16_t width, uint16_t height)
{
    shoebill_card_video_t *ctx;
    
    if (shoe.slots[slotnum].card_type != card_none) {
        sprintf(config->error_msg, "This slot (%u) already has a card\n", slotnum);
        return 0;
    }
    
    ctx = p_alloc(shoe.pool, sizeof(shoebill_card_video_t));
    shoe.slots[slotnum].ctx = ctx;
    
    shoe.slots[slotnum].card_type = card_shoebill_video;
    
    // Make sure the scanline width is a multiple of 32 pixels, and is at least 32 pixels
    // beyond the end of the display. If scanline_width==width, A/UX 2.0 will wrap the mouse around
    // the edge of the screen.
    uint32_t scanline_width = width + (32 - (width % 32)) + 32;
    
    scanline_width = width; // FIXME: undo this
    
    shoe.slots[slotnum].connected = 1;
    shoe.slots[slotnum].read_func = nubus_video_read_func;
    shoe.slots[slotnum].write_func = nubus_video_write_func;
    shoe.slots[slotnum].destroy_func = NULL;
    shoe.slots[slotnum].interrupts_enabled = 1;
    nubus_video_init(ctx, slotnum, width, height, scanline_width);
    return 1;
}

uint32_t shoebill_install_tfb_card(shoebill_config_t *config, uint8_t slotnum)
{
    shoebill_card_tfb_t *ctx;
    
    if (shoe.slots[slotnum].card_type != card_none) {
        sprintf(config->error_msg, "This slot (%u) already has a card\n", slotnum);
        return 0;
    }
    
    ctx = p_alloc(shoe.pool, sizeof(shoebill_card_tfb_t));
    shoe.slots[slotnum].ctx = ctx;
    
    shoe.slots[slotnum].card_type = card_toby_frame_buffer;
    
    shoe.slots[slotnum].connected = 1;
    shoe.slots[slotnum].read_func = nubus_tfb_read_func;
    shoe.slots[slotnum].write_func = nubus_tfb_write_func;
    shoe.slots[slotnum].destroy_func = NULL;
    shoe.slots[slotnum].interrupts_enabled = 1;
    nubus_tfb_init(ctx, slotnum);
    return 1;
}

uint32_t shoebill_install_ethernet_card(shoebill_config_t *config, uint8_t slotnum, uint8_t ethernet_addr[6], int tap_fd)
{
    shoebill_card_ethernet_t *ctx;
    
    if (shoe.slots[slotnum].card_type != card_none) {
        sprintf(config->error_msg, "This slot (%u) already has a card\n", slotnum);
        return 0;
    }
    
    ctx = p_alloc(shoe.pool, sizeof(shoebill_card_ethernet_t));
    shoe.slots[slotnum].ctx = ctx;
    
    shoe.slots[slotnum].card_type = card_shoebill_ethernet;
    shoe.slots[slotnum].connected = 1;
    shoe.slots[slotnum].read_func = nubus_ethernet_read_func;
    shoe.slots[slotnum].write_func = nubus_ethernet_write_func;
    shoe.slots[slotnum].destroy_func = nubus_ethernet_destroy_func;
    shoe.slots[slotnum].interrupts_enabled = 1;
    nubus_ethernet_init(ctx, slotnum, ethernet_addr, tap_fd);
    return 1;
}

shoebill_video_frame_info_t shoebill_get_video_frame(uint8_t slotnum,
                                                     _Bool just_params)
{
    shoebill_video_frame_info_t result;
    
    if (!shoe.running) {
        memset(&result, 0, sizeof(result));
        return result;
    }
    
    void *ctx = shoe.slots[slotnum].ctx;
    
    if (shoe.slots[slotnum].card_type == card_toby_frame_buffer)
        return nubus_tfb_get_frame(ctx, just_params);
    else if (shoe.slots[slotnum].card_type == card_shoebill_video)
        return nubus_video_get_frame(ctx, just_params);
    
    assert(!"Unknown card type");
    
    memset(&result, 0, sizeof(result));
    return result;
}

/*
 * Given a shoebill_config_t structure, configure and initialize
 * the emulator.
 * This is the first function you should call if you're writing an
 * interface to Shoebill.
 */
uint32_t shoebill_initialize(shoebill_config_t *config)
{
    uint32_t i, j, pc = 0xffffffff;
    coff_file *coff = NULL;
    scsi_device_t disks[8];
    uint8_t *rom_data = NULL, *kernel_data = NULL;
    uint32_t rom_size = 0, kernel_size;
    
    
    memset(&disks[0], 0, 8 * sizeof(scsi_device_t));
    memset(&shoe, 0, sizeof(global_shoebill_context_t));
    
    // Keep a copy of *config in shoe.config_copy, so shoebill_reset() can refer to it
    memcpy(&shoe.config_copy, config, sizeof(shoebill_config_t));
    
    shoe.pool = p_new_pool(NULL);
    
    fpu_initialize();
    
    // Try to load the ROM
    if (config->rom_path == NULL) {
        sprintf(config->error_msg, "No rom file specified\n");
        goto fail;
    }
    else if (!_load_rom(config, &rom_data, &rom_size))
            goto fail;
    
    // Try to load the A/UX kernel
    if (config->aux_kernel_path == NULL) {
        sprintf(config->error_msg, "No A/UX kernel specified\n");
        goto fail;
    }
    else if (!config->scsi_devices[0].path || strlen((char*)config->scsi_devices[0].path)==0) {
        sprintf(config->error_msg, "The root A/UX disk needs to be at scsi ID 0\n");
        goto fail;
    }
    
    // Load the kernel from the disk at scsi id #0
    kernel_data = shoebill_extract_kernel((char*)config->scsi_devices[0].path,
                                 config->aux_kernel_path,
                                 config->error_msg,
                                 &kernel_size);
    if (!kernel_data)
        goto fail;
    
    coff = coff_parse(kernel_data, kernel_size, shoe.pool);
    free(kernel_data); // kernel_data was allocated with malloc()
    
    if (coff == NULL) {
        sprintf(config->error_msg, "Can't open that A/UX kernel [%s]\n",
                config->aux_kernel_path);
        goto fail;
    }
    shoe.coff = coff;
    
    // Try to open the disk images
    if (!_open_disk_images(config, disks))
        goto fail;
    
    // Allocate and configure the rom and memory space
    
    if (config->ram_size < (1024*1024)) {
        sprintf(config->error_msg, "%u bytes is too little ram\n", config->ram_size);
        goto fail;
    }
    
    shoe.physical_rom_size = rom_size;
    shoe.physical_rom_base = p_alloc(shoe.pool, rom_size+8); // +8 because of physical_get hack
    memcpy(shoe.physical_rom_base, rom_data, rom_size);
    p_free(rom_data);
    rom_data = NULL;
    
    shoe.physical_mem_size = config->ram_size;
    shoe.physical_mem_base = p_alloc(shoe.pool, config->ram_size+8); // +8 because of physical_get hack
    memset(shoe.physical_mem_base, 0, shoe.physical_mem_size);
    
    // Initialize Macintosh lomem variables that A/UX actually cares about
    
    #define AUX_LOMEM_OFFSET 0x50000
    _init_macintosh_lomem_globals(AUX_LOMEM_OFFSET);
    
    // Initialize A/UX's kernel_info structure
    
    _init_kernel_info(config, disks, AUX_LOMEM_OFFSET);
    
    // Load A/UX kernel COFF segments into memory (returns PC, the entry point into the kernel)
    
    if (!_load_aux_kernel(config, coff, &pc))
        goto fail;
    
    /*
     * Load it all into the internal global shoebill state
     * (Can't fail after this point)
     */
    
    /*
     * FIXME: to implement clean resetting, everything with a global structure needs
     *        an initialization function. Starting here with via/pram...
     */
    init_via_state(config->pram, config->pram_callback, config->pram_callback_param);
    init_adb_state();
    init_scsi_bus_state();
    init_iwm_state();
    init_asc_state();
    
    /* Invalidate the pc cache */
    invalidate_pccache();
    
    set_sr(0x2000);
    shoe.pc = pc;
    memcpy(shoe.scsi_devices, disks, 8 * sizeof(scsi_device_t));
    
    pthread_mutex_init(&shoe.via_cpu_lock, NULL);
    pthread_mutex_init(&shoe.via_clock_thread_lock, NULL);
    
    pthread_mutex_lock(&shoe.via_clock_thread_lock);
    pthread_create(&shoe.via_thread_pid, NULL, via_clock_thread, NULL);
    
    /*
     * config->debug_mode is a hack - the debugger implements its own CPU thread
     */
    
    pthread_cond_init(&shoe.cpu_stop_cond, NULL);
    pthread_mutex_init(&shoe.cpu_stop_mutex, NULL);
    pthread_mutex_init(&shoe.cpu_thread_lock, NULL);
    
    pthread_mutex_lock(&shoe.cpu_thread_lock);
    if (!config->debug_mode)
        pthread_create(&shoe.cpu_thread_pid, NULL, _cpu_thread, NULL);
    
    return 1;
    
fail:
    if (rom_data) p_free(rom_data);
    
    for (i=0; i<7; i++)
        if (disks[i].f) fclose(disks[i].f);
    
    if (shoe.physical_rom_base) p_free(shoe.physical_rom_base);
    if (shoe.physical_mem_base) p_free(shoe.physical_mem_base);
    
    p_free_pool(shoe.pool);
    memset(&shoe, 0, sizeof(global_shoebill_context_t));
    
    return 0;
}

/* 
 * This should only be called from the context of cpu_thread
 * Specifically, inst_reset()
 */
void shoebill_restart (void)
{
    coff_file *coff;
    uint32_t pc, kernel_size;
    uint8_t *kernel_data;
    
    // block other threads from twiddling shoe.adb
    pthread_mutex_lock(&shoe.adb.lock);
    
    // FIXME: block via thread from firing timers
    
    // zero memory
    memset(shoe.physical_mem_base, 0, shoe.physical_mem_size);
    
    // clear the pmmu cache
    memset(shoe.pmmu_cache, 0, sizeof(shoe.pmmu_cache));
    
    // Invalidate the pc cache
    invalidate_pccache();
    
    // Reset all CPU registers
    memset(shoe.d, 0, sizeof(shoe.d));
    memset(shoe.a, 0, sizeof(shoe.a));
    shoe.vbr = 0;
    shoe.sfc = 0;
    shoe.dfc = 0;
    shoe.cacr = 0;
    shoe.usp = 0;
    shoe.isp = 0;
    shoe.msp = 0;
    
    // Reset all pmmu registers
    shoe.crp = shoe.srp = shoe.drp = 0;
    shoe.tc = 0;
    shoe.tc_pagesize = shoe.tc_pagemask = 0;
    shoe.tc_ps = shoe.tc_is = shoe.tc_is_plus_ps = shoe.tc_enable = shoe.tc_sre = 0;
    shoe.pcsr = 0;
    shoe.ac = 0;
    memset(shoe.bad, 0, sizeof(shoe.bad));
    memset(shoe.bac, 0, sizeof(shoe.bac));
    shoe.cal = 0;
    shoe.val = 0;
    shoe.scc = 0;
    shoe.psr.word = 0;
    
    // Reset all FPU registers
    fpu_reset();
    
    // Free the old unix coff_file,
    coff_free(shoe.coff);
    
    // Close the disk at scsi id #0
    fclose(shoe.scsi_devices[0].f);
    
    // Reload the kernel from that disk
    kernel_data = shoebill_extract_kernel((char*)shoe.scsi_devices[0].image_path,
                                          shoe.config_copy.aux_kernel_path,
                                          shoe.config_copy.error_msg,
                                          &kernel_size);
    
    // FIXME: handle this more gracefully
    assert(kernel_data && "can't reload the kernel from the root filesystem");
    
    // Re-parse the kernel binary
    coff = coff_parse(kernel_data, kernel_size, shoe.pool);
    free(kernel_data); // kernel_data was allocated with malloc()
    
    // FIXME: also handle this more gracefully
    assert(coff && "can't parse the kernel");
    
    // Re-open the root disk image
    shoe.scsi_devices[0].f = fopen(shoe.scsi_devices[0].image_path, "r+b");
    assert(shoe.scsi_devices[0].f && "couldn't reopen the disk image at scsi id #0"); // FIXME: and this
    
    shoe.coff = coff;
    
    // Initialize Macintosh lomem variables that A/UX actually cares about
    _init_macintosh_lomem_globals(AUX_LOMEM_OFFSET);
    
    // Initialize A/UX's kernel_info structure
    _init_kernel_info(&shoe.config_copy, shoe.scsi_devices, AUX_LOMEM_OFFSET);
    
    // Load A/UX kernel COFF segments into memory (returns PC, the entry point into the kernel)
    if (!_load_aux_kernel(&shoe.config_copy, shoe.coff, &pc))
        assert(!"_load_aux_kernel failed, and I can't handle that yet...");
    
    // reset all devices
    reset_adb_state();
    reset_scsi_bus_state();
    reset_iwm_state();
    reset_via_state();
    
    set_sr(0x2000);
    shoe.pc = pc;
    shoe.cpu_thread_notifications = 0;
    
    pthread_mutex_unlock(&shoe.adb.lock);
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
    if (!shoe.running)
        return ;
    
    const uint8_t down_mask = down ? 0 : 0x80;
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    _send_key(key | down_mask);
    
    adb_request_service_request(3);
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_key_modifier(uint8_t modifier_mask)
{
    if (!shoe.running)
        return ;
    
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
    if (!shoe.running)
        return ;
    
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
    if (!shoe.running)
        return ;
    
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    shoe.mouse.delta_x += x;
    shoe.mouse.delta_y += y;
    
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_mouse_click(uint8_t down)
{
    if (!shoe.running)
        return ;
    
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    shoe.mouse.button_down = (down != 0);
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);
    
    pthread_mutex_unlock(&shoe.adb.lock);
}

void shoebill_send_vbl_interrupt(uint8_t slotnum)
{
    if (!shoe.running)
        return ;
    
    assert((slotnum >= 9) && (slotnum <= 14) && shoe.slots[slotnum].connected);
    
    if (shoe.slots[slotnum].interrupts_enabled) {
        shoe.via[1].rega_input &= ~b(00111111) & ~~(1 << (slotnum - 9));
        via_raise_interrupt(2, IFR_CA1);
    }
}

void shoebill_validate_or_zap_pram(uint8_t *pram, _Bool forcezap)
{
    if (!forcezap) {
        if (memcmp(pram + 0xc, "NuMc", 4) == 0)
            return ;
    }
    
    memset(pram, 0, 256);
    memcpy(pram + 0xc, "NuMc", 4); // Mark PRAM as "valid"
    
    /*
     * Set text box I-beam blink speed and mouse acceleration to something reasonable
     */
    pram[9] = 0x88;
}

void slog(const char *fmt, ...)
{
    va_list args;
 
    return ;
    
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    
    fflush(stdout);
}












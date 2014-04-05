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

#ifndef _CORE_API_H
#define _CORE_API_H

#include <stdint.h>

typedef enum {
    card_none = 0, // Empty slot
    card_toby_frame_buffer, // Original Macintosh II video card
    card_shoebill_video, // Fancy 21st-century Shoebill video card
    card_shoebill_ethernet // FIXME: doesn't exist yet
} card_names_t;

typedef struct {
    uint8_t *frame_buffer;
} shoebill_card_tfb_t;

typedef struct {
    uint8_t r, g, b, a;
} video_ctx_color_t;

typedef struct {
    video_ctx_color_t *direct_buf, *clut;
    uint8_t *indexed_buf, *rom;
    
    uint32_t pixels;
    
    uint16_t width, height, scanline_width;
    
    uint16_t depth, clut_idx;
    
    double refresh_rate;
} shoebill_card_video_t;

typedef struct {
    // Doesn't exist yet
} shoebill_card_ethernet_t;

typedef struct {
/*
 * Fill in this part of the struct 
 * before you call shoebill_initialize()
 */
    uint32_t ram_size;
    const char *rom_path;
    const char *aux_kernel_path;
    
    uint8_t aux_verbose : 1; // Whether to boot A/UX in verbose mode
    uint8_t aux_autoconfig : 1; // Whether to run A/UX autoconfig
    
    uint16_t root_ctrl, swap_ctrl;
    uint8_t root_drive, swap_drive;
    uint8_t root_partition, swap_partition;
    uint8_t root_cluster;
    
    /* Devices at the 7 possible target SCSI ids */
    struct {
        const char *path;
        uint64_t size; // will be filled in later
    } scsi_devices[7]; // scsi device #7 is the initiator (can't be a target)
    
/* ^^^ You can stop now  ^^^ */
    
    /* Cards installed in the 16 (really, just 0x9 to 0xe) nubus slots */
    struct {
        card_names_t card_type;
        union {
            shoebill_card_tfb_t tfb;
            shoebill_card_video_t video;
            shoebill_card_ethernet_t ethernet;
        } card;
    } slots[16];
    
    pthread_t cpu_thread_pid, via_thread_pid;
    
    char error_msg[8192];
} shoebill_control_t;

uint32_t shoebill_initialize(shoebill_control_t *params);

uint32_t shoebill_install_video_card(shoebill_control_t *control, uint8_t slotnum,
                                     uint16_t width, uint16_t height,
                                     double refresh_rate);

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

uint8_t* shoebill_extract_kernel(const char *disk_path, const char *kernel_path, char *error_str, uint32_t *len);

#endif

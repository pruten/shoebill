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
#include <assert.h>
#include <stdlib.h>
#include "../core/shoebill.h"

/* --- Physical_get jump table --- */
#pragma mark Physical_get jump table

void _physical_get_ram (void)
{
    uint64_t *addr;
    if slikely(shoe.physical_addr < shoe.physical_mem_size)
        addr = (uint64_t*)&shoe.physical_mem_base[shoe.physical_addr];
    else
        addr = (uint64_t*)&shoe.physical_mem_base[shoe.physical_addr % shoe.physical_mem_size];
    
    const uint8_t bits = (8 - shoe.physical_size) * 8;
    shoe.physical_dat = ntohll(*addr) >> bits;
}

void _physical_get_rom (void)
{
    uint64_t *addr = (uint64_t*)&shoe.physical_rom_base[shoe.physical_addr & (shoe.physical_rom_size-1)];
    
    const uint8_t bits = (8 - shoe.physical_size) * 8;
    shoe.physical_dat = ntohll(*addr) >> bits;
}

void _physical_get_io (void)
{
    switch (shoe.physical_addr & 0x5003ffff) {
        case 0x50000000 ... 0x50003fff: // VIA1 + VIA2
            via_read_raw();
            return ;
        case 0x50004000 ... 0x50005fff: {// SCC
            //slog("physical_get: got read to SCC\n");
            const uint32_t a = shoe.physical_addr & 0x1fff;
            if (a == 2 && shoe.physical_size==1)
                shoe.physical_dat = 0x4; // R0TXRDY
            return ;
        }
        case 0x50006000 ... 0x50007fff: // SCSI (pseudo-DMA with DRQ?)
            assert(shoe.logical_size == 4);
            shoe.physical_dat = scsi_dma_read_long();
            return ;
        case 0x50010000 ... 0x50011fff: // SCSI (normal mode?)
            scsi_reg_read();
            return ;
        case 0x50012000 ... 0x50013fff: // SCSI (pseudo-DMA with no DRQ?)
            assert(shoe.logical_size == 1);
            shoe.physical_dat = scsi_dma_read();
            return ;
        case 0x50014000 ... 0x50015fff: // Sound
            // slog("physical_get: got read to sound\n");
            shoe.physical_dat = sound_dma_read_raw(shoe.physical_addr & 0x1fff, shoe.physical_size);
            // slog("soundsound read : register 0x%04x sz=%u\n",
                   //shoe.physical_addr - 0x50014000, shoe.physical_size);
            // shoe.physical_dat = 0;
            return ;
        case 0x50016000 ... 0x50017fff: // SWIM (IWM?)
            // slog("physical_get: got read to IWM\n");
            shoe.physical_dat = iwm_dma_read();
            return ;
        default:
            //slog("physical_get: got read to UNKNOWN IO ADDR %x\n", shoe.physical_addr);
            return ;
    }
}

void _physical_get_super_slot (void)
{
    const uint32_t slot = shoe.physical_addr >> 28;
    if slikely(shoe.slots[slot].connected)
        shoe.physical_dat = shoe.slots[slot].read_func(shoe.physical_addr,
                                                       shoe.physical_size,
                                                       slot);
    else
        shoe.abort = 1; // throw a bus error for reads to disconnected slots
        // XXX: Do super slot accesses raise bus errors?
}

void _physical_get_standard_slot (void)
{
    const uint32_t slot = (shoe.physical_addr >> 24) & 0xf;
    if slikely(shoe.slots[slot].connected)
        shoe.physical_dat = shoe.slots[slot].read_func(shoe.physical_addr,
                                                       shoe.physical_size,
                                                       slot);
    else
        shoe.abort = 1; // throw a bus error for reads to disconnected slots
}

const physical_get_ptr physical_get_jump_table[16] = {
    _physical_get_ram, // 0x0
    _physical_get_ram, // 0x1
    _physical_get_ram, // 0x2
    _physical_get_ram, // 0x3
    _physical_get_rom, // 0x4
    _physical_get_io,  // 0x5
    _physical_get_super_slot, // 0x6
    _physical_get_super_slot, // 0x7
    _physical_get_super_slot, // 0x8
    _physical_get_super_slot, // 0x9
    _physical_get_super_slot, // 0xa
    _physical_get_super_slot, // 0xb
    _physical_get_super_slot, // 0xc
    _physical_get_super_slot, // 0xd
    _physical_get_super_slot, // 0xe
    _physical_get_standard_slot // 0xf
};

/* --- Physical_set jump table --- */
#pragma mark Physical_set jump table

void _physical_set_ram (void)
{
    uint8_t *addr;
    if (shoe.physical_addr >= shoe.physical_mem_size)
        addr = &shoe.physical_mem_base[shoe.physical_addr % shoe.physical_mem_size];
    else
        addr = &shoe.physical_mem_base[shoe.physical_addr];
    
    if ((shoe.physical_addr >= 0x100) && (shoe.physical_addr < (0x8000))) {
        slog("LOMEM set: *0x%08x = 0x%x\n", shoe.physical_addr, (uint32_t)chop(shoe.physical_dat, shoe.physical_size));
    }
    
    const uint32_t sz = shoe.physical_size;
    switch (sz) {
        case 1:
            *addr = (uint8_t)shoe.physical_dat;
            return ;
            
        case 2:
            *((uint16_t*)addr) = htons((uint16_t)shoe.physical_dat);
            return ;
            
        case 4:
            *((uint32_t*)addr) = htonl((uint32_t)shoe.physical_dat);
            return ;
            
        case 8:
            *(uint64_t*)addr = ntohll(shoe.physical_dat);
            return ;
            
        default: {
            uint64_t q = shoe.physical_dat;
            uint8_t i;
            for (i=1; i<=sz; i++) {
                addr[sz-i] = (uint8_t)q;
                q >>= 8;
            }
            return ;
        }
    }
}

void _physical_set_rom (void)
{
    // nothing happens when you try to modify ROM
}

void _physical_set_io (void)
{
    switch (shoe.physical_addr & 0x5003ffff) {
        case 0x50000000 ... 0x50003fff: // VIA1 + VIA2
            via_write_raw();
            return ;
        case 0x50004000 ... 0x50005fff: // SCC
            //slog("physical_set: got write to SCC\n");
            return ;
        case 0x50006000 ... 0x50007fff: // SCSI (pseudo-DMA with DRQ?)
            assert(shoe.physical_size == 4);
            scsi_dma_write_long(shoe.physical_dat);
            return ;
        case 0x50010000 ... 0x50011fff: // SCSI (normal mode?)
            scsi_reg_write();
            return ;
        case 0x50012000 ... 0x50013fff: // SCSI (pseudo-DMA with no DRQ?)
            assert(shoe.physical_size == 1);
            scsi_dma_write(shoe.physical_dat);
            return ;
        case 0x50014000 ... 0x50015fff: // Sound
            sound_dma_write_raw(shoe.physical_addr & 0x1fff, shoe.physical_size, shoe.physical_dat);
            // slog("soundsound write: register 0x%04x sz=%u dat=0x%x\n",
                   // shoe.physical_addr - 0x50014000, shoe.physical_size, (uint32_t)shoe.physical_dat);
            // slog("physical_set: got write to sound\n");
            return ;
        case 0x50016000 ... 0x50017fff: // SWIM (IWM?)
            //slog("physical_set: got write to IWM\n");
            iwm_dma_write();
            return ;
        default:
            //slog("physical_set: got write to UNKNOWN IO ADDR %x\n", shoe.physical_addr);
            return ;
    }
}

void _physical_set_super_slot (void)
{
    const uint32_t slot = shoe.physical_addr >> 28;
    if (shoe.slots[slot].connected)
        shoe.slots[slot].write_func(shoe.physical_addr,
                                    shoe.physical_size,
                                    shoe.physical_dat,
                                    slot);
}

void _physical_set_standard_slot (void)
{
    const uint32_t slot = (shoe.physical_addr >> 24) & 0xf;
    if (shoe.slots[slot].connected)
        shoe.slots[slot].write_func(shoe.physical_addr,
                                    shoe.physical_size,
                                    shoe.physical_dat,
                                    slot);
}

const physical_set_ptr physical_set_jump_table[16] = {
    _physical_set_ram, // 0x0
    _physical_set_ram, // 0x1
    _physical_set_ram, // 0x2
    _physical_set_ram, // 0x3
    _physical_set_rom, // 0x4
    _physical_set_io,  // 0x5
    _physical_set_super_slot, // 0x6
    _physical_set_super_slot, // 0x7
    _physical_set_super_slot, // 0x8
    _physical_set_super_slot, // 0x9
    _physical_set_super_slot, // 0xa
    _physical_set_super_slot, // 0xb
    _physical_set_super_slot, // 0xc
    _physical_set_super_slot, // 0xd
    _physical_set_super_slot, // 0xe
    _physical_set_standard_slot // 0xf
};

/* --- PMMU logical address translation --- */
#pragma mark PMMU logical address translation

#define PMMU_CACHE_CRP 0
#define PMMU_CACHE_SRP 1

/*typedef struct {
    uint32_t logical_value : 24; // At most the high 24 bits of the logical address
    uint32_t used_bits : 5;
    uint32_t wp : 1; // whether the page is write protected
    uint32_t modified : 1; // whether the page has been modified
    uint32_t unused1 : 1;
    uint32_t unused2 : 8;
    uint32_t physical_addr : 24;
 } pmmu_cache_entry;
 
 struct {
    pmmu_cache_entry entry[512];
    uint8_t valid_map[512 / 8];
 } pmmu_cache[2];*/

#define write_back_desc() { \
    if (desc_addr < 0) { \
        *rootp_ptr = desc; \
    } else { \
        pset((uint32_t)desc_addr, 4<<desc_size, desc); \
    } \
}


static _Bool check_pmmu_cache_write(void)
{
    const _Bool use_srp = (shoe.tc_sre && (shoe.logical_fc >= 5));
    
    // logical addr [is]xxxxxxxxxxxx[ps] -> value xxxxxxxxxxxx
    const uint32_t value = (shoe.logical_addr << shoe.tc_is) >> shoe.tc_is_plus_ps;
    // value xxx[xxxxxxxxx] -> key xxxxxxxxx
    const uint32_t key = value & (PMMU_CACHE_SIZE - 1); // low PMMU_CACHE_KEY_BITS bits
    
    const pmmu_cache_entry_t entry = shoe.pmmu_cache[use_srp].entry[key];
    
    const _Bool is_set = (shoe.pmmu_cache[use_srp].valid_map[key >> 3] >> (key & 7)) & 1;
    
    const uint32_t ps_mask = 0xffffffff >> entry.used_bits;
    const uint32_t v_mask = ~~ps_mask;
    
    shoe.physical_addr = ((entry.physical_addr<<8) & v_mask) | (shoe.logical_addr & ps_mask);
    return is_set && (entry.logical_value == value) && entry.modified && !entry.wp;
}

static _Bool check_pmmu_cache_read(void)
{
    const _Bool use_srp = (shoe.tc_sre && (shoe.logical_fc >= 5));
    
    // logical addr [is]xxxxxxxxxxxx[ps] -> value xxxxxxxxxxxx
    const uint32_t value = (shoe.logical_addr << shoe.tc_is) >> shoe.tc_is_plus_ps;
    // value xxx[xxxxxxxxx] -> key xxxxxxxxx
    const uint32_t key = value & (PMMU_CACHE_SIZE - 1); // low PMMU_CACHE_KEY_BITS bits
    
    const pmmu_cache_entry_t entry = shoe.pmmu_cache[use_srp].entry[key];
    
    const _Bool is_set = (shoe.pmmu_cache[use_srp].valid_map[key >> 3] >> (key & 7)) & 1;
    
    const uint32_t ps_mask = 0xffffffff >> entry.used_bits;
    const uint32_t v_mask = ~~ps_mask;
    
    shoe.physical_addr = ((entry.physical_addr<<8) & v_mask) | (shoe.logical_addr & ps_mask);
    return is_set && (entry.logical_value == value);
}


static void translate_logical_addr()
{
    const uint8_t use_srp = (shoe.tc_sre && (shoe.logical_fc >= 5));
    assert((0x66 >> shoe.logical_fc) & 1); // we only support these FCs for now
    
    uint64_t *rootp_ptr = (use_srp ? (&shoe.srp) : (&shoe.crp));
    const uint64_t rootp = *rootp_ptr;
    uint8_t desc_did_change = 0;
    uint8_t desc_level = 0;
    int64_t desc_addr = -1; // address of the descriptor (-1 -> register)
    uint8_t wp = 0; // Whether any descriptor in the search has wp (write protected) set
    uint8_t i;
    uint64_t desc = rootp; // Initial descriptor is the root pointer descriptor
    uint8_t desc_size = 1; // And the root pointer descriptor is always 8 bytes (1==8 bytes, 0==4 bytes)
    uint8_t used_bits = shoe.tc_is; // Keep track of how many bits will be the effective "page size"
    // (If the table search terminates early (before used_bits == ts_ps()),
    //  then (32 - used_bits) will be the effective page size. That is, the number of bits
    //  we or into the physical addr from the virtual addr)
    
    desc_addr = -1; // address of the descriptor (-1 -> register)
    
    /* We'll keep shifting logical_addr such that its most significant bits are the next ti-index */
    uint32_t logical_addr = shoe.logical_addr << used_bits;
    
    // TODO: Check limit here
    
    // If root descriptor is invalid, throw a bus error
    if sunlikely(rp_dt(rootp) == 0) {
        throw_bus_error(shoe.logical_addr, shoe.logical_is_write);
        return ;
    }
    
    // desc is a page descriptor, skip right ahead to search_done:
    if (rp_dt(rootp) == 1)
        goto search_done;
    
    // for (i=0; i < 4; i++) { // (the condition is unnecessary - just leaving it in for clarity)
    for (i=0; 1; i++) {
        // desc must be a table descriptor here
        
        const uint8_t ti = tc_ti(i);
        used_bits += ti;
        
        // TODO: do the limit check here
        
        // Find the index into our current i-level table
        const uint32_t index = logical_addr >> (32-ti);
        logical_addr <<= ti;
        
        // load the child descriptor
        const uint32_t table_base_addr = desc_table_addr(desc);
        const uint8_t s = desc_dt(desc, desc_size) & 1;
        
        // desc = pget(table_base_addr + (4 << s)*index, (4 << s));
        get_desc(table_base_addr + (4 << s)*index, (4 << s));
        desc_size = s;
        
        // Desc may be a table descriptor, page descriptor, indirect descriptor, or invalid descriptor
        
        const uint8_t dt = desc_dt(desc, desc_size);
        
        // If this descriptor is invalid, throw a bus error
        if sunlikely(dt == 0) {
            throw_bus_error(shoe.logical_addr, shoe.logical_is_write);
            return ;
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
            if sunlikely(desc_dt(desc, desc_size) == 0) {
                throw_bus_error(shoe.logical_addr, shoe.logical_is_write);
                return ;
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
    //       (or would be, if they were implemented yet)
    
    // TODO: update U (used) bit
    
    wp |= desc_wp(desc, desc_size); // or in the wp flag for this page descriptor
    
    // And finally throw a bus error
    if sunlikely(wp && shoe.logical_is_write) {
        throw_bus_error(shoe.logical_addr, shoe.logical_is_write);
        return ;
    }
    
    // If we're writing, set the modified flag
    if (shoe.logical_is_write && !(desc & ( desc_size ? desc_m_long : desc_m_short ))) {
        desc |= ( desc_size ? desc_m_long : desc_m_short );
        write_back_desc();
    }
    
    const uint32_t ps_mask = 0xffffffff >> used_bits;
    const uint32_t v_mask = ~~ps_mask;
    
    const uint32_t paddr = (desc_page_addr(desc) & v_mask) | (shoe.logical_addr & ps_mask);
    shoe.physical_addr = paddr;
    
    
    /* --- insert this translation into pmmu_cache --- */
    
    // logical addr [is]xxxxxxxxxxxx[ps] -> value xxxxxxxxxxxx
    const uint32_t value = (shoe.logical_addr << shoe.tc_is) >> shoe.tc_is_plus_ps;
    // value xxx[xxxxxxxxx] -> key xxxxxxxxx
    const uint32_t key = value & (PMMU_CACHE_SIZE-1); // low PMMU_CACHE_KEY_BITS bits
    
    pmmu_cache_entry_t entry;
    shoe.pmmu_cache[use_srp].valid_map[key/8] |= (1 << (key & 7));
    entry.logical_value = value;
    entry.physical_addr = (desc_page_addr(desc)) >> 8;
    entry.wp = wp;
    entry.modified = desc_m(desc, desc_size);
    entry.used_bits = used_bits;
    shoe.pmmu_cache[use_srp].entry[key] = entry;
    
}


void logical_get (void)
{
    
    // If address translation isn't enabled, this is a physical address
    if sunlikely(!shoe.tc_enable) {
        shoe.physical_addr = shoe.logical_addr;
        shoe.physical_size = shoe.logical_size;
        physical_get();
        if sunlikely(shoe.abort) {
            shoe.abort = 0;
            throw_long_bus_error(shoe.logical_addr, 0);
            return ;
        }
        shoe.logical_dat = shoe.physical_dat;
        return ;
    }
    
    const uint32_t logical_size = shoe.logical_size;
    const uint32_t logical_addr = shoe.logical_addr;
    
    const uint32_t pagemask = shoe.tc_pagemask;
    const uint32_t pageoffset = logical_addr & pagemask;
    
    // Common case: the read is contained entirely within a page
    if slikely(!((pageoffset + logical_size - 1) >> shoe.tc_ps)) {
        if sunlikely(!check_pmmu_cache_read()) {
            shoe.logical_is_write = 0;
            translate_logical_addr();
            if sunlikely(shoe.abort)
                return ;
        }
        
        if slikely(shoe.physical_addr < shoe.physical_mem_size) {
            // Fast path
            shoe.logical_dat = ntohll(*(uint64_t*)&shoe.physical_mem_base[shoe.physical_addr]) >> ((8-logical_size)*8);
        }
        else {
            shoe.physical_size = logical_size;
            physical_get();
            if sunlikely(shoe.abort) {
                shoe.abort = 0;
                throw_long_bus_error(logical_addr, 0);
                return ;
            }
            shoe.logical_dat = shoe.physical_dat;
        }
    }
    else { // This read crosses a page boundary
        const uint32_t addr_a = logical_addr;
        const uint32_t size_b = (pageoffset + logical_size) & pagemask;
        const uint32_t size_a = logical_size - size_b;
        const uint32_t addr_b = addr_a + size_a;
        
        shoe.logical_is_write = 0;
        
        shoe.logical_addr = addr_a;
        shoe.logical_size = size_a;
        if sunlikely(!check_pmmu_cache_read()) {
            translate_logical_addr();
            if sunlikely(shoe.abort)
                return ;
        }
        
        const uint32_t p_addr_a = shoe.physical_addr;
        
        shoe.logical_addr = addr_b;
        shoe.logical_size = size_b;
        if sunlikely(!check_pmmu_cache_read()) {
            translate_logical_addr();
            if sunlikely(shoe.abort)
                return ;
        }
        
        const uint32_t p_addr_b = shoe.physical_addr;
        shoe.physical_size = size_b;
        physical_get();
        if sunlikely(shoe.abort) {
            shoe.abort = 0;
            throw_long_bus_error(shoe.logical_addr, 0);
            return ;
        }
        const uint64_t fetch_b = shoe.physical_dat;
        
        shoe.physical_addr = p_addr_a;
        shoe.physical_size = size_a;
        physical_get();
        if sunlikely(shoe.abort) {
            shoe.abort = 0;
            throw_long_bus_error(shoe.logical_addr, 0);
            return ;
        }
        
        shoe.logical_dat = (shoe.physical_dat << (size_b*8)) | fetch_b;
    }
}

void logical_set (void)
{
    // If address translation isn't enabled, this is a physical address
    if sunlikely(!shoe.tc_enable) {
        shoe.physical_addr = shoe.logical_addr;
        shoe.physical_size = shoe.logical_size;
        shoe.physical_dat = shoe.logical_dat;
        physical_set();
        return ;
    }
    
    const uint32_t logical_size = shoe.logical_size;
    const uint32_t logical_addr = shoe.logical_addr;
    
    const uint32_t pagemask = shoe.tc_pagemask;
    const uint32_t pageoffset = logical_addr & pagemask;
    
    // Make the translate function fail if the page is write-protected
    shoe.logical_is_write = 1;
    
    // Common case: this write is contained entirely in one page
    if slikely(!((pageoffset + logical_size - 1) >> shoe.tc_ps)) {
        // Common case: the write is contained entirely within a page
        if sunlikely(!check_pmmu_cache_write()) {
            translate_logical_addr();
            if sunlikely(shoe.abort)
                return ;
        }
        
        shoe.physical_size = shoe.logical_size;
        shoe.physical_dat = shoe.logical_dat;
        physical_set();
    }
    else { // This write crosses a page boundary
        const uint32_t addr_a = shoe.logical_addr;
        const uint32_t size_b = (pageoffset + logical_size) & pagemask;
        const uint32_t size_a = shoe.logical_size - size_b;
        const uint32_t addr_b = addr_a + size_a;
        const uint64_t data_a = shoe.logical_dat >> (size_b*8);
        const uint64_t data_b = bitchop_64(shoe.logical_dat, size_b*8);
        
        shoe.logical_addr = addr_a;
        shoe.logical_size = size_a;
        if sunlikely(!check_pmmu_cache_write()) {
            translate_logical_addr();
            if sunlikely(shoe.abort)
                return ;
        }
        const uint32_t p_addr_a = shoe.physical_addr;
        
        shoe.logical_addr = addr_b;
        shoe.logical_size = size_b;
        if sunlikely(!check_pmmu_cache_write()) {
            translate_logical_addr();
            if sunlikely(shoe.abort)
                return ;
        }
        const uint32_t p_addr_b = shoe.physical_addr;
        
        shoe.physical_addr = p_addr_a;
        shoe.physical_size = size_a;
        shoe.physical_dat = data_a;
        physical_set();
        
        shoe.physical_addr = p_addr_b;
        shoe.physical_size = size_b;
        shoe.physical_dat = data_b;
        physical_set();
        
        return ;
    }
}

/* --- PC cache routines --- */
#pragma mark PC cache routines

static uint16_t pccache_miss(const uint32_t pc)
{
    const uint32_t pagemask = shoe.tc_pagemask;
    const uint32_t pageoffset = pc & pagemask;
    uint32_t paddr;
    
    /*
     * I think the instruction decoder uses these
     * these function codes:
     * 6 -> supervisor program space,
     * 2 -> user program space
     */
    shoe.logical_fc = sr_s() ? 6 : 2;
    shoe.logical_addr = pc;
    if sunlikely(!check_pmmu_cache_read()) {
        shoe.logical_is_write = 0;
        translate_logical_addr();
        if sunlikely(shoe.abort)
            goto fail;
    }
    
    paddr = shoe.physical_addr ^ pageoffset;
    
    shoe.pccache_use_srp = shoe.tc_sre && sr_s();
    shoe.pccache_logical_page = pc ^ pageoffset;
    
    if (paddr < 0x40000000) {
        /* Address in RAM */
        
        if sunlikely(paddr >= shoe.physical_mem_size)
            paddr %= shoe.physical_mem_size;
        
        shoe.pccache_ptr = &shoe.physical_mem_base[paddr];
        return ntohs(*(uint16_t*)(shoe.pccache_ptr + pageoffset));
    }
    else if (paddr < 0x50000000) {
        /* Address in ROM */
        shoe.pccache_ptr = &shoe.physical_rom_base[paddr & (shoe.physical_rom_size - 1)];
        return ntohs(*(uint16_t*)(shoe.pccache_ptr + pageoffset));
    }
    
    /*
     * For now, only supporting reads from RAM and ROM.
     * This could easily be supported by just calling
     * physical_get() and leaving the cache invalid,
     * but I don't think A/UX ever tries to execute outside
     * RAM/ROM.
     */
    assert(!"pccache_miss: neither RAM nor ROM!\n");
    
fail:
    invalidate_pccache();
    return 0;
}

uint16_t pccache_nextword(const uint32_t pc)
{
    if (sunlikely(pc & 1))
        goto odd_addr;
    
    if slikely(shoe.tc_enable) {
        const uint32_t pc_offset = pc & shoe.tc_pagemask;
        const uint32_t pc_page = pc ^ pc_offset;
        const uint32_t use_srp = shoe.tc_sre && sr_s();
        
        /* If the cache exists and is valid */
        if slikely((shoe.pccache_use_srp == use_srp) && (shoe.pccache_logical_page == pc_page)) {
            // printf("pccache_nextword: hit: pc=%x\n", pc);
            return ntohs(*(uint16_t*)(shoe.pccache_ptr + pc_offset));
        }
        // printf("pccache_nextword: miss: pc=%x\n", pc);
        
        return pccache_miss(pc);
    }
    else {
        uint32_t paddr = pc;
        
        if (paddr < 0x40000000) {
            /* Address in RAM */
            
            if sunlikely(paddr >= shoe.physical_mem_size)
                paddr %= shoe.physical_mem_size;
    
            return ntohs(*(uint16_t*)(&shoe.physical_mem_base[paddr]));
        }
        else if (paddr < 0x50000000) {
            /* Address in ROM */
            
            return ntohs(*(uint16_t*)&shoe.physical_rom_base[paddr & (shoe.physical_rom_size - 1)]);
        }
        assert(!"!tc_enable: neither RAM nor RAM\n");
    }
    
odd_addr:
    assert(!"odd pc address!\n");
    return 0;
}

uint32_t pccache_nextlong(const uint32_t pc)
{
    if slikely(shoe.tc_enable) {
        const uint32_t lastpage = shoe.pccache_logical_page;
        const uint32_t pc_offset = pc & shoe.tc_pagemask;
        const uint32_t pc_page = pc ^ pc_offset;
        const uint32_t use_srp = shoe.tc_sre && sr_s();
        
        
        /* If the cache exists, is valid, and the read is contained entirely within 1 page */
        if slikely((shoe.pccache_use_srp == use_srp) && (lastpage == pc_page) && !((pc_offset + 3) >> shoe.tc_ps)) {
            const uint32_t result = ntohl(*(uint32_t*)(shoe.pccache_ptr + pc_offset));
            if (sunlikely(pc_offset & 1))
                goto odd_addr;
            return result;
        }
        
        const uint32_t result_high = pccache_nextword(pc) << 16;
        if sunlikely(shoe.abort)
            return 0;
        
        return result_high | pccache_nextword(pc + 2);
    }
    else {
        uint32_t paddr = pc;
        
        if sunlikely(paddr & 1)
            goto odd_addr;
        
        if (paddr < 0x40000000) {
            /* Address in RAM */
            
            if sunlikely(paddr >= shoe.physical_mem_size)
                paddr %= shoe.physical_mem_size;
            
            return ntohl(*(uint32_t*)(&shoe.physical_mem_base[paddr]));
        }
        else if (paddr < 0x50000000) {
            /* Address in ROM */
            
            return ntohl(*(uint32_t*)&shoe.physical_rom_base[paddr & (shoe.physical_rom_size - 1)]);
        }
        assert(!"!tc_enable: neither RAM nor RAM\n");
    }

odd_addr:
    assert(!"odd pc address!\n");
    return 0;
}


/* --- EA routines --- */
#pragma mark EA routines

#define nextword(pc) ({const uint16_t w = pccache_nextword(pc); if sunlikely(shoe.abort) return; (pc) += 2; w;})
#define nextlong(pc) ({const uint32_t L = pccache_nextlong(pc); if sunlikely(shoe.abort) return; (pc) += 4; L;})

// ea_decode_extended() - find the EA for those hiddeous 68020 addr modes
static void ea_decode_extended()
{
    const uint32_t start_pc = shoe.pc; // the original PC
    uint32_t mypc = start_pc; // our local PC (don't modify up the real PC)
    const uint32_t ext_a = nextword(mypc); // the extension word
    ~decompose(ext_a, d rrr w ss F b i zz 0 III)
    
    // d == index register type
    // r == index register
    // w == word/long-word index size
    // s == scale factor
    // F == extension word format (0==brief, 1==full)
    // b == base register suppress
    // i == index suppress
    // z == base displacement size
    // I == index/indirect selection
    
    
    if (F == 0) { // If this is the brief extension word
        // use the sign-extended least significant byte in the extension word
        uint32_t base_disp = (int8_t)(ext_a & 0xff);
        
        // load the base_address
        uint32_t base_addr;
        if (~bmatch(shoe.mr, 00xx1xxx)) { // consult the MR, use the PC?
            base_addr = start_pc; // start at the beginning of the extension word
        }
        else { // otherwise, it's shoe.a[shoe.mr&7]
            base_addr = shoe.a[shoe.mr&7];
        }
        
        // load the index value
        uint32_t index_val;
        if (w==0) { // use signed-extended lower word of the register
            if (d==0) // shoe.d[]
                index_val = (int16_t)(get_d(r, 2));
            else // shoe.a[]
                index_val = (int16_t)(get_a(r, 2));
        } else { // use entire register
            if (d==0) index_val = shoe.d[r];
            else index_val = shoe.a[r];
        }
        // Scale the index value
        index_val <<= s;
        
        // the brief extension word is implicitly preindexed
        shoe.extended_addr = base_addr + base_disp + index_val;
        shoe.extended_len = mypc - start_pc;
        //slog("I found address 0x%x\n", shoe.extended_addr);
        return ;
    }
    else { // If this is a full extension word,
        
        // first find the base address, which may be shoe.a[?] or shoe.pc
        uint32_t base_addr = 0;
        if (b == 0) { // only if it isn't suppressed
            if (~bmatch(shoe.mr, 00xx1xxx)) { // consult the MR,
                base_addr = start_pc; // start at the beginning of the extension word
            }
            else { // otherwise, it's shoe.a[shoe.mr&7]
                base_addr = shoe.a[shoe.mr&7];
            }
        }
        
        // Find the index value
        uint32_t index_val = 0;
        if (i == 0) { // only if it isn't suppressed
            if (w==0) { // use signed-extended lower word of the register
                if (d==0) // shoe.d[]
                    index_val = (int16_t)(get_d(r, 2));
                else // shoe.a[]
                    index_val = (int16_t)(get_a(r, 2));
            } else { // use entire register
                if (d==0) index_val = shoe.d[r];
                else index_val = shoe.a[r];
            }
            // Scale the index value
            index_val <<= s;
        }
        
        // Find the base displacement
        uint32_t base_disp = 0;
        // ... but only if the size is > null
        if (z > 1) {
            if (z == 2) { // if word-length, fetch nextword() and sign-extend
                base_disp = (int16_t)(nextword(mypc));
            } else { // otherwise, it's a longword
                base_disp = nextlong(mypc);
            }
        }
        
        // Find the outer displacement
        uint32_t outer_disp = 0;
        // based on the I/IS behavior
        switch ((i<<3)|I) {
            case ~b(0010): case ~b(0110): case ~b(1010):
                // sign-extended word-length outer displacement
                outer_disp = (int16_t)nextword(mypc);
                break;
            case ~b(0011): case ~b(0111): case ~b(1011): {
                // long word outer displacement
                outer_disp = nextlong(mypc);
                break ;
            }
        }
        
        //slog("D/A=%u, reg=%u, W/L=%u, Scale=%u, F=%u, BS=%u, IS=%u, BDSize=%u, I/IS=%u\n",
        //d, r, w, s, F, b, i, z, I);
        //slog("base_addr=%x, index_val=%x, base_disp=%x, outer_disp=%x\n",
        //base_addr, index_val, base_disp, outer_disp);
        
        // Now mash all these numbers together to get an EA
        switch ((i<<3)|I) {
            case ~b(0001): case ~b(0010): case ~b(0011):
            case ~b(1001): case ~b(1010): case ~b(1011): {
                // Indirect preindexed
                const uint32_t intermediate = lget(base_addr + base_disp + index_val, 4);
                if sunlikely(shoe.abort) return ;
                shoe.extended_addr = intermediate + outer_disp;
                shoe.extended_len = mypc - start_pc;
                // slog("addr=0x%x len=%u\n", shoe.extended_addr, shoe.extended_len);
                return ;
            }
                
            case ~b(0101): case ~b(0110): case ~b(0111): {
                // Indirect postindexed
                const uint32_t intermediate = lget(base_addr + base_disp, 4);
                if sunlikely(shoe.abort) return ;
                shoe.extended_addr = intermediate + index_val + outer_disp;
                shoe.extended_len = mypc - start_pc;
                return ;
            }
                
            case ~b(1000): case ~b(0000): {
                // No memory indirect action
                // EA = base_addr + base_disp + index
                shoe.extended_addr = base_addr + base_disp + index_val;
                shoe.extended_len = mypc - start_pc;
                return ;
            }
            default:
                assert(!"ea_decode_extended: oh noes! invalid I/IS!\n");
                // I think that 68040 *doesn't* throw an exception here...
                // FIXME: figure out what actually happens here
                break;
        }
    }
}


// Data register direct mode
void _ea_000_read (void)
{
    shoe.dat = get_d(shoe.mr & 7, shoe.sz);
}
void _ea_000_write (void)
{
    set_d(shoe.mr & 7, shoe.dat, shoe.sz);
}


// address register direct mode
void _ea_001_read (void)
{
    shoe.dat = get_a(shoe.mr & 7, shoe.sz);
}
void _ea_001_write (void)
{
    assert(shoe.sz==4);
    shoe.a[shoe.mr & 7] = shoe.dat;
}


// address register indirect mode
void _ea_010_read (void)
{
    shoe.dat = lget(shoe.a[shoe.mr & 7], shoe.sz);
}
void _ea_010_write (void)
{
    lset(shoe.a[shoe.mr & 7], shoe.sz, shoe.dat);
}
void _ea_010_addr (void)
{
    shoe.dat = shoe.a[shoe.mr & 7];
}


// address register indirect with postincrement mode
void _ea_011_read (void)
{
    shoe.dat = lget(shoe.a[shoe.mr & 7], shoe.sz);
}
void _ea_011_read_commit (void)
{
    const uint8_t reg = shoe.mr & 7;
    shoe.a[reg] += (((reg==7) && (shoe.sz==1)) ? 2 : shoe.sz);
}
void _ea_011_write (void)
{
    const uint8_t reg = shoe.mr & 7;
    const uint8_t delta = ((reg==7) && (shoe.sz==1)) ? 2 : shoe.sz;
    
    lset(shoe.a[reg], shoe.sz, shoe.dat);
    if slikely(!shoe.abort)
        shoe.a[reg] += delta;
}


// address register indirect with predecrement mode
void _ea_100_read (void)
{
    const uint8_t reg = shoe.mr & 7;
    const uint8_t delta = ((reg==7) && (shoe.sz==1))?2:shoe.sz;
    shoe.dat = lget(shoe.a[reg]-delta, shoe.sz);
}
void _ea_100_read_commit (void)
{
    const uint8_t reg = shoe.mr & 7;
    shoe.a[reg] -= (((reg==7) && (shoe.sz==1)) ? 2 : shoe.sz);
}
void _ea_100_write (void)
{
    const uint8_t reg = shoe.mr & 7;
    const uint8_t delta = ((reg==7) && (shoe.sz==1)) ? 2 : shoe.sz;
    
    lset(shoe.a[reg] - delta, shoe.sz, shoe.dat);
    if slikely(!shoe.abort)
        shoe.a[reg] -= delta;
}


// address register indirect with displacement mode
void _ea_101_read (void)
{
    shoe.uncommitted_ea_read_pc = shoe.pc;
    const int16_t disp = nextword(shoe.uncommitted_ea_read_pc);
    shoe.dat = lget(shoe.a[shoe.mr & 7] + disp, shoe.sz);
}
void _ea_101_read_commit (void)
{
    shoe.pc += 2;
}
void _ea_101_write (void)
{
    const int16_t disp = nextword(shoe.pc);
    lset(shoe.a[shoe.mr & 7] + disp, shoe.sz, shoe.dat);
}
void _ea_101_addr (void)
{
    const int16_t disp = nextword(shoe.pc);
    shoe.dat = shoe.a[shoe.mr & 7] + disp;
}


// memory/address register indirect with index
void _ea_110_read (void)
{
    ea_decode_extended();
    if slikely(!shoe.abort)
        shoe.dat = lget(shoe.extended_addr, shoe.sz);
    shoe.uncommitted_ea_read_pc = shoe.pc + shoe.extended_len;
}
void _ea_110_read_commit (void)
{
    shoe.pc = shoe.uncommitted_ea_read_pc;
}
void _ea_110_write (void)
{
    ea_decode_extended();
    if slikely(!shoe.abort) {
        lset(shoe.extended_addr, shoe.sz, shoe.dat);
        if slikely(!shoe.abort)
            shoe.pc += shoe.extended_len;
    }
}
void _ea_110_addr (void)
{
    ea_decode_extended();
    if slikely(!shoe.abort) {
        shoe.dat = shoe.extended_addr;
        shoe.pc += shoe.extended_len;
    }
}


// absolute short addressing mode
void _ea_111_000_read (void)
{
    shoe.uncommitted_ea_read_pc = shoe.pc;
    const int32_t addr = (int16_t)nextword(shoe.uncommitted_ea_read_pc);
    shoe.dat = lget((uint32_t)addr, shoe.sz);
}
void _ea_111_000_read_commit (void)
{
    shoe.pc += 2;
}
void _ea_111_000_write (void)
{
    const int32_t addr = (int16_t)nextword(shoe.pc);
    lset((uint32_t)addr, shoe.sz, shoe.dat);
}
void _ea_111_000_addr (void)
{
    const int32_t addr = (int16_t)nextword(shoe.pc);
    shoe.dat = (uint32_t)addr;
}


// absolute long addressing mode
void _ea_111_001_read (void)
{
    shoe.uncommitted_ea_read_pc = shoe.pc;
    const uint32_t addr = nextlong(shoe.uncommitted_ea_read_pc);
    shoe.dat = lget(addr, shoe.sz);
}
void _ea_111_001_read_commit (void)
{
    shoe.pc += 4;
}
void _ea_111_001_write (void)
{
    const uint32_t addr = nextlong(shoe.pc);
    lset(addr, shoe.sz, shoe.dat);
}
void _ea_111_001_addr (void)
{
    const uint32_t addr = nextlong(shoe.pc);
    shoe.dat = addr;
}

// program counter indirect with displacement mode
void _ea_111_010_read (void)
{
    const uint32_t base_pc = shoe.pc;
    shoe.uncommitted_ea_read_pc = base_pc;
    const int16_t disp = nextword(shoe.uncommitted_ea_read_pc);
    shoe.dat = lget(base_pc + disp, shoe.sz);
}
void _ea_111_010_read_commit (void)
{
    shoe.pc += 2;
}
void _ea_111_010_addr (void)
{
    const uint32_t oldpc = shoe.pc;
    const int16_t displacement = nextword(shoe.pc);
    shoe.dat = oldpc + displacement;
}


// (program counter ...)
void _ea_111_011_read (void)
{
    ea_decode_extended();
    if slikely(!shoe.abort)
        shoe.dat = lget(shoe.extended_addr, shoe.sz);
    shoe.uncommitted_ea_read_pc = shoe.pc + shoe.extended_len;
}
void _ea_111_011_read_commit (void)
{
    shoe.pc = shoe.uncommitted_ea_read_pc;
}
void _ea_111_011_addr (void)
{
    ea_decode_extended();
    if slikely(!shoe.abort) {
        shoe.dat = shoe.extended_addr;
        shoe.pc += shoe.extended_len;
    }
}


// immediate data
void _ea_111_100_read (void)
{
    if (shoe.sz == 1) {
        shoe.uncommitted_ea_read_pc = shoe.pc + 2;
        shoe.dat = lget(shoe.pc, 2) & 0xff;
    }
    else {
        shoe.uncommitted_ea_read_pc = shoe.pc + shoe.sz;
        shoe.dat = lget(shoe.pc, shoe.sz);
    }
}
void _ea_111_100_read_commit (void)
{
    shoe.pc = shoe.uncommitted_ea_read_pc;
}


// illegal EA mode
void _ea_illegal (void)
{
    throw_illegal_instruction();
}

// nothing to do
void _ea_nop (void)
{
}


const _ea_func ea_read_jump_table[64] = {
    // Data register direct mode
    _ea_000_read,
    _ea_000_read,
    _ea_000_read,
    _ea_000_read,
    _ea_000_read,
    _ea_000_read,
    _ea_000_read,
    _ea_000_read,
    // address register direct mode
    _ea_001_read,
    _ea_001_read,
    _ea_001_read,
    _ea_001_read,
    _ea_001_read,
    _ea_001_read,
    _ea_001_read,
    _ea_001_read,
    // address register indirect mode
    _ea_010_read,
    _ea_010_read,
    _ea_010_read,
    _ea_010_read,
    _ea_010_read,
    _ea_010_read,
    _ea_010_read,
    _ea_010_read,
    // address register indirect with postincrement mode
    _ea_011_read,
    _ea_011_read,
    _ea_011_read,
    _ea_011_read,
    _ea_011_read,
    _ea_011_read,
    _ea_011_read,
    _ea_011_read,
    // address register indirect with predecrement mode
    _ea_100_read,
    _ea_100_read,
    _ea_100_read,
    _ea_100_read,
    _ea_100_read,
    _ea_100_read,
    _ea_100_read,
    _ea_100_read,
    // address register indirect with displacement mode
    _ea_101_read,
    _ea_101_read,
    _ea_101_read,
    _ea_101_read,
    _ea_101_read,
    _ea_101_read,
    _ea_101_read,
    _ea_101_read,
    // memory/address register indirect with index
    _ea_110_read,
    _ea_110_read,
    _ea_110_read,
    _ea_110_read,
    _ea_110_read,
    _ea_110_read,
    _ea_110_read,
    _ea_110_read,
    // absolute short addressing mode
    _ea_111_000_read,
    // absolute long addressing mode
    _ea_111_001_read,
    // program counter indirect with displacement mode
    _ea_111_010_read,
    // program counter indirect with index
    _ea_111_011_read,
    // immediate
    _ea_111_100_read,
    // The rest are illegal EA modes
    _ea_illegal,
    _ea_illegal,
    _ea_illegal
};

const _ea_func ea_read_commit_jump_table[64] = {
    // Data register direct mode
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    // address register direct mode
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    // address register indirect mode
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    _ea_nop,
    // address register indirect with postincrement mode
    _ea_011_read_commit,
    _ea_011_read_commit,
    _ea_011_read_commit,
    _ea_011_read_commit,
    _ea_011_read_commit,
    _ea_011_read_commit,
    _ea_011_read_commit,
    _ea_011_read_commit,
    // address register indirect with predecrement mode
    _ea_100_read_commit,
    _ea_100_read_commit,
    _ea_100_read_commit,
    _ea_100_read_commit,
    _ea_100_read_commit,
    _ea_100_read_commit,
    _ea_100_read_commit,
    _ea_100_read_commit,
    // address register indirect with displacement mode
    _ea_101_read_commit,
    _ea_101_read_commit,
    _ea_101_read_commit,
    _ea_101_read_commit,
    _ea_101_read_commit,
    _ea_101_read_commit,
    _ea_101_read_commit,
    _ea_101_read_commit,
    // memory/address register indirect with index
    _ea_110_read_commit,
    _ea_110_read_commit,
    _ea_110_read_commit,
    _ea_110_read_commit,
    _ea_110_read_commit,
    _ea_110_read_commit,
    _ea_110_read_commit,
    _ea_110_read_commit,
    // absolute short addressing mode
    _ea_111_000_read_commit,
    // absolute long addressing mode
    _ea_111_001_read_commit,
    // program counter indirect with displacement mode
    _ea_111_010_read_commit,
    // program counter indirect with index
    _ea_111_011_read_commit,
    // immediate
    _ea_111_100_read_commit,
    // The rest are illegal EA modes
    NULL,
    NULL,
    NULL
};

const _ea_func ea_write_jump_table[64] = {
    // Data register direct mode
    _ea_000_write,
    _ea_000_write,
    _ea_000_write,
    _ea_000_write,
    _ea_000_write,
    _ea_000_write,
    _ea_000_write,
    _ea_000_write,
    // address register direct mode
    _ea_001_write,
    _ea_001_write,
    _ea_001_write,
    _ea_001_write,
    _ea_001_write,
    _ea_001_write,
    _ea_001_write,
    _ea_001_write,
    // address register indirect mode
    _ea_010_write,
    _ea_010_write,
    _ea_010_write,
    _ea_010_write,
    _ea_010_write,
    _ea_010_write,
    _ea_010_write,
    _ea_010_write,
    // address register indirect with postincrement mode
    _ea_011_write,
    _ea_011_write,
    _ea_011_write,
    _ea_011_write,
    _ea_011_write,
    _ea_011_write,
    _ea_011_write,
    _ea_011_write,
    // address register indirect with predecrement mode
    _ea_100_write,
    _ea_100_write,
    _ea_100_write,
    _ea_100_write,
    _ea_100_write,
    _ea_100_write,
    _ea_100_write,
    _ea_100_write,
    // address register indirect with displacement mode
    _ea_101_write,
    _ea_101_write,
    _ea_101_write,
    _ea_101_write,
    _ea_101_write,
    _ea_101_write,
    _ea_101_write,
    _ea_101_write,
    // memory/address register indirect with index
    _ea_110_write,
    _ea_110_write,
    _ea_110_write,
    _ea_110_write,
    _ea_110_write,
    _ea_110_write,
    _ea_110_write,
    _ea_110_write,
    // absolute short addressing mode
    _ea_111_000_write,
    // absolute long addressing mode
    _ea_111_001_write,
    // program counter indirect with displacement mode
    _ea_illegal,
    // program counter indirect with index
    _ea_illegal,
    // immediate
    _ea_illegal,
    // The rest are illegal EA modes
    _ea_illegal,
    _ea_illegal,
    _ea_illegal
};


const _ea_func ea_addr_jump_table[64] = {
    // Data register direct mode
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    // address register direct mode
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    // address register indirect mode
    _ea_010_addr,
    _ea_010_addr,
    _ea_010_addr,
    _ea_010_addr,
    _ea_010_addr,
    _ea_010_addr,
    _ea_010_addr,
    _ea_010_addr,
    // address register indirect with postincrement mode
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    // address register indirect with predecrement mode
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    _ea_illegal,
    // address register indirect with displacement mode
    _ea_101_addr,
    _ea_101_addr,
    _ea_101_addr,
    _ea_101_addr,
    _ea_101_addr,
    _ea_101_addr,
    _ea_101_addr,
    _ea_101_addr,
    // memory/address register indirect with index
    _ea_110_addr,
    _ea_110_addr,
    _ea_110_addr,
    _ea_110_addr,
    _ea_110_addr,
    _ea_110_addr,
    _ea_110_addr,
    _ea_110_addr,
    // absolute short addressing mode
    _ea_111_000_addr,
    // absolute long addressing mode
    _ea_111_001_addr,
    // program counter indirect with displacement mode
    _ea_111_010_addr,
    // program counter indirect with index
    _ea_111_011_addr,
    // immediate
    _ea_illegal,
    // The rest are illegal EA modes
    _ea_illegal,
    _ea_illegal,
    _ea_illegal
};












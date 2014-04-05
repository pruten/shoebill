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
#include <machine/endian.h>
#include <arpa/inet.h>
#include <assert.h>
#include <stdlib.h>
#include "../core/shoebill.h"

static void translate_logical_addr();

void _logical_get (void);
void logical_get (void) {
    const uint32_t addr = shoe.logical_addr;
    // uint8_t super = sr_s();
    
    _logical_get();
    
//    if ((addr >= 0x100) && (addr < 0x2000)) {
//        printf("LOMEM READ: 0x%08x = 0x%llx\n", addr, shoe.logical_dat);
//    }
}

void physical_get (void) {
    
    switch (shoe.physical_addr) {
        case 0 ... 0x3FFFFFFF: {
            void *addr = &shoe.physical_mem_base[shoe.physical_addr & (shoe.physical_mem_size-1)];
            switch (shoe.physical_size) {
                case 4:
                    shoe.physical_dat = ntohl(*(uint32_t*)addr);
                    return ;
                case 2:
                    shoe.physical_dat = ntohs(*(uint16_t*)addr);
                    return ;
                case 1:
                    shoe.physical_dat = *(uint8_t*)addr;
                    return ;
                default: {
                    const uint32_t s = shoe.physical_size;
                    uint64_t q = 0;
                    uint32_t i;
                    for (i=0; i<s; i++)
                        q = (q << 8) | ((uint8_t*)addr)[i];
                    shoe.physical_dat = q;
                    //printf("Reading quad from *0x%08x = 0x%llx\n", shoe.physical_addr, q);
                    return ;
                }
            }
            assert(!"never get here");
        }
        case 0x40000000 ... 0x4FFFFFFF: { // ROM
            void *addr = &shoe.physical_rom_base[shoe.physical_addr & (shoe.physical_rom_size-1)];
            switch (shoe.physical_size) {
                case 4:
                    shoe.physical_dat = ntohl(*(uint32_t*)addr);
                    return ;
                case 2:
                    shoe.physical_dat = ntohs(*(uint16_t*)addr);
                    return ;
                case 1:
                    shoe.physical_dat = *(uint8_t*)addr;
                    return ;
                default: {
                    const uint32_t s = shoe.physical_size;
                    uint64_t q = 0;
                    uint32_t i;
                    for (i=0; i<s; i++)
                        q = (q << 8) | ((uint8_t*)addr)[i];
                    shoe.physical_dat = q;
                    return ;
                }
            }
            assert(!"never get here");
        }
        case 0x50000000 ... 0x50ffffff: { // IO
            switch (shoe.physical_addr & 0x5003ffff) {
                case 0x50000000 ... 0x50001fff: // VIA1
                    via_reg_read();
                    // printf("physical_get: got read to VIA1 (%x & 0x5003ffff = %x)\n", shoe.physical_addr, shoe.physical_addr & 0x5003ffff);
                    return ;
                case 0x50002000 ... 0x50003fff: // VIA2
                    via_reg_read();
                    // printf("physical_get: got read to VIA2\n");
                    return ;
                case 0x50004000 ... 0x50005fff: {// SCC
                    //printf("physical_get: got read to SCC\n");
                    const uint32_t a = shoe.physical_addr & 0x1fff;
                    if (a == 2 && shoe.physical_size==1)
                        shoe.physical_dat = 0x4; // R0TXRDY
                    return ;
                }
                case 0x50006000 ... 0x50007fff: // SCSI (pseudo-DMA with DRQ?)
                    //printf("physical_get: got read to SCSI low\n");
                    //assert(!"physical_get: got read to SCSI low");
                    assert(shoe.logical_size == 4);
                    shoe.physical_dat = scsi_dma_read_long();
                    return ;
                case 0x50010000 ... 0x50011fff: // SCSI (normal mode?)
                    scsi_reg_read();
                    return ;
                case 0x50012000 ... 0x50013fff: // SCSI (pseudo-DMA with no DRQ?)
                    assert(shoe.logical_size == 1);
                    shoe.physical_dat = scsi_dma_read();
                    // printf("physical_get: got read to SCSI hi\n");
                    // assert(!"physical_get: got read to SCSI hi\n");
                    return ;
                case 0x50014000 ... 0x50015fff: // Sound
                    printf("physical_get: got read to sound\n");
                    return ;
                case 0x50016000 ... 0x50017fff: // SWIM (IWM?)
                    // printf("physical_get: got read to IWM\n");
                    shoe.physical_dat = iwm_dma_read();
                    return ;
                default:
                    printf("physical_get: got read to UNKNOWN IO ADDR %x\n", shoe.physical_addr);
                    return ;
            }
        }
        case 0x60000000 ... 0xefffffff: { // Nubus super slot space
            const uint32_t slot = shoe.physical_addr >> 28;
            if (shoe.slots[slot].connected)
                shoe.physical_dat = shoe.slots[slot].read_func(shoe.physical_addr,
                                                               shoe.physical_size,
                                                               slot);
            else
                shoe.abort = 1; // throw a bus error for reads to disconnected slots
                                // XXX: Do super slot accesses raise bus errors?
            return ;
        }
        case 0xf0000000 ... 0xffffffff: {
            const uint32_t slot = (shoe.physical_addr >> 24) & 0xf;
            if (shoe.slots[slot].connected)
                shoe.physical_dat = shoe.slots[slot].read_func(shoe.physical_addr,
                                                               shoe.physical_size,
                                                               slot);
            else
                shoe.abort = 1; // throw a bus error for reads to disconnected slots
            return ;
        }
            
        default: {
            printf("physical_get: I got an access to 0x%x (sz=%u), but I don't know how to handle it...\n",
                   shoe.physical_addr, shoe.physical_size);
            shoe.physical_dat = 0;
            return ;
        }
    }
}

void physical_set (void)
{
    switch (shoe.physical_addr) {
        case 0 ... 0x3FFFFFFF: { // The first RAM bank (64MB)
            const uint32_t dat = htonl(shoe.physical_dat) >> ((4-shoe.physical_size)*8);
            void *addr = &shoe.physical_mem_base[shoe.physical_addr & (shoe.physical_mem_size-1)];
            switch (shoe.physical_size) {
                case 4:
                    *((uint32_t*)addr) = dat;
                    return ;
                case 2:
                    *((uint16_t*)addr) = (uint16_t)dat;
                    return ;
                case 1:
                    *((uint8_t*)addr) = (uint8_t)dat;
                    return ;
                    // case 8: { } // I should do something fast for case 8 here
                default: {
                    const uint32_t s = shoe.physical_size;
                    uint64_t q = shoe.physical_dat;
                    uint32_t i;
                    for (i=0; i<s; i++) {
                        ((uint8_t*)addr)[s-(i+1)] = q & 0xff;
                        q >>= 8;
                    }
                    return ;
                }
            }
            assert(!"never get here");
        }
        case 0x40000000 ... 0x4FFFFFFF: { // ROM
            // Nothing happens when you try to modify ROM
            return ;
        }
        case 0x50000000 ... 0x50ffffff: { // IO
            switch (shoe.physical_addr & 0x5003ffff) {
                case 0x50000000 ... 0x50001fff: // VIA1
                    via_reg_write();
                    // printf("physical_set: got write to VIA1\n");
                    return ;
                case 0x50002000 ... 0x50003fff: // VIA2
                    via_reg_write();
                    // printf("physical_set: got write to VIA2\n");
                    return ;
                case 0x50004000 ... 0x50005fff: // SCC
                    //printf("physical_set: got write to SCC\n");
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
                    //printf("physical_set: got write to SCSI hi\n");
                    //assert(!"physical_set: got write to SCSI hi\n");
                    return ;
                case 0x50014000 ... 0x50015fff: // Sound
                    printf("physical_set: got write to sound\n");
                    return ;
                case 0x50016000 ... 0x50017fff: // SWIM (IWM?)
                    //printf("physical_set: got write to IWM\n");
                    iwm_dma_write();
                    return ;
                default:
                    printf("physical_set: got write to UNKNOWN IO ADDR %x\n", shoe.physical_addr);
                    return ;
            }
        }
        case 0x60000000 ... 0xefffffff: { // Nubus super slot space
            const uint32_t slot = shoe.physical_addr >> 28;
            if (shoe.slots[slot].connected)
                shoe.slots[slot].write_func(shoe.physical_addr,
                                            shoe.physical_size,
                                            shoe.physical_dat,
                                            slot);
            return ;
        }
        case 0xf0000000 ... 0xffffffff: { // Nubus standard slot space
            const uint32_t slot = (shoe.physical_addr >> 24) & 0xf;
            if (shoe.slots[slot].connected)
                shoe.slots[slot].write_func(shoe.physical_addr,
                                            shoe.physical_size,
                                            shoe.physical_dat,
                                            slot);
            return ;
            // Writing to a disconnected slot won't cause a bus error on macii
        }

        default: {
            printf("physical_set: I got a write (addr=0x%x) (val=0x%llx) (sz=%u), but I don't know how to handle it...\n",
                   shoe.physical_addr, shoe.physical_dat, shoe.physical_size);
            return ;
        }
    }
}

void _logical_get (void)
{
    
    // If address translation isn't enabled, this is a physical address
    if (!tc_enable()) {
        shoe.physical_addr = shoe.logical_addr;
        shoe.physical_size = shoe.logical_size;
        physical_get();
        if (shoe.abort) {
            shoe.abort = 0;
            throw_long_bus_error(shoe.logical_addr, 0);
            return ;
        }
        shoe.logical_dat = shoe.physical_dat;
        return ;
    }
    
    const uint32_t logical_size = shoe.logical_size;
    const uint32_t logical_addr = shoe.logical_addr;
    
    const uint16_t ps = tc_ps(); // log2 of the page size
    const uint32_t pagesize = 1 << ps; // the page size
    const uint32_t pagemask = pagesize-1; // a mask of the page bits
    const uint32_t pageoffset = logical_addr & pagemask;
    
    shoe.logical_is_write = 0;
    
    // If the the data access will wrap across multiple pages, then fuck
    if ((pageoffset + logical_size - 1) >> ps) {
        const uint32_t addr_a = shoe.logical_addr;
        const uint32_t size_b = (pageoffset + logical_size) & pagemask;
        const uint32_t size_a = shoe.logical_size - size_b;
        const uint32_t addr_b = addr_a + size_a;
        
        printf("ps = %u, pagesize=%u, pagemask=%x, pageoffset=%x\n", ps, pagesize, pagemask, pageoffset);
        printf("multiple page get: addr_a=0x%08x size_a=%u  addr_b=0x%08x size_b=%u\n", addr_a, size_a, addr_b, size_b);
        shoe.logical_addr = addr_a;
        shoe.logical_size = size_a;
        translate_logical_addr();
        if (shoe.abort)
            return ;
        
        const uint32_t p_addr_a = shoe.physical_addr;
        shoe.physical_size = size_a;
        physical_get();
        if (shoe.abort) {
            shoe.abort = 0;
            throw_long_bus_error(shoe.logical_addr, 0);
            return ;
        }
        const uint64_t fetch_a = shoe.physical_dat;
        
        shoe.logical_addr = addr_b;
        shoe.logical_size = size_b;
        translate_logical_addr();
        if (shoe.abort)
            return ;
        
        const uint32_t p_addr_b = shoe.physical_addr;
        shoe.physical_size = size_b;
        physical_get();
        if (shoe.abort) {
            shoe.abort = 0;
            throw_long_bus_error(shoe.logical_addr, 0);
            return ;
        }
        
        printf("multiple_page_get: p_addr_a = 0x%08x, p_addr_b = 0x%08x\n", p_addr_a, p_addr_b);
        
        shoe.logical_dat = (fetch_a << (size_b*8)) | shoe.physical_dat;
        
        printf("multiple_page_get: logical_dat = (%llx | %llx) = %llx\n", fetch_a, shoe.physical_dat, shoe.logical_dat);
        
        return ;
    }
    
    // Common case: the read is contained entirely within a page
    translate_logical_addr();
    if (shoe.abort)
        return ;
    
    shoe.physical_size = shoe.logical_size;
    physical_get();
    if (shoe.abort) {
        shoe.abort = 0;
        throw_long_bus_error(shoe.logical_addr, 0);
        return ;
    }
    shoe.logical_dat = shoe.physical_dat;
    
}

void logical_set (void)
{
    if ((shoe.logical_addr >= 0xaf2) && (shoe.logical_addr < 0xaf6) && (shoe.logical_size == 1) && ((shoe.logical_dat&0xff) == 0xff)) {
        shoe.logical_dat = 0;
    }
//    if ((shoe.logical_addr >= 0x12fffff6) && (shoe.logical_addr <= 0x12ffffff))
//        printf("aux3: setting 0x%08x = 0x%x\n", shoe.logical_addr, (uint32_t)shoe.logical_dat);
//    
//    if ((shoe.logical_addr >= 0x100) && (shoe.logical_addr < 0x2000)) {
//        printf("LOMEM WRITE: 0x%08x = 0x%x\n", shoe.logical_addr, (uint32_t) shoe.logical_dat);
//    }
    
    // If address translation isn't enabled, this is a physical address
    if (!tc_enable()) {
        shoe.physical_addr = shoe.logical_addr;
        shoe.physical_size = shoe.logical_size;
        shoe.physical_dat = shoe.logical_dat;
        physical_set();
        return ;
    }
    
    const uint32_t logical_size = shoe.logical_size;
    const uint32_t logical_addr = shoe.logical_addr;
    
    const uint16_t ps = tc_ps(); // log2 of the page size
    const uint32_t pagesize = 1 << ps; // the page size
    const uint32_t pagemask = pagesize-1; // a mask of the page bits
    const uint32_t pageoffset = logical_addr & pagemask;
    
    // Make the translate function fail if the page is write-protected
    shoe.logical_is_write = 1;
    
    // If the the data access will wrap across multiple pages, then fuck
    if ((pageoffset + logical_size - 1) >> ps) {
        const uint32_t addr_a = shoe.logical_addr;
        const uint32_t size_b = (pageoffset + logical_size) & pagemask;
        const uint32_t size_a = shoe.logical_size - size_b;
        const uint32_t addr_b = addr_a + size_a;
        const uint64_t data_a = shoe.logical_dat >> (size_b*8);
        const uint64_t data_b = bitchop_64(shoe.logical_dat, size_b*8);
        
        printf("ps = %u, pagesize=%u, pagemask=%x, pageoffset=%x\n", ps, pagesize, pagemask, pageoffset);
        printf("multiple page set: addr_a=0x%08x size_a=%u  addr_b=0x%08x size_b=%u\n", addr_a, size_a, addr_b, size_b);
        
        shoe.logical_addr = addr_a;
        shoe.logical_size = size_a;
        translate_logical_addr();
        if (shoe.abort)
            return ;
        const uint32_t p_addr_a = shoe.physical_addr;
        
        shoe.logical_addr = addr_b;
        shoe.logical_size = size_b;
        translate_logical_addr();
        if (shoe.abort)
            return ;
        const uint32_t p_addr_b = shoe.physical_addr;
        
        printf("multiple page set: paddr_a=0x%08x (data_a=0x%llx) paddr_b=0x%08x (data_b=0x%llx) (orig_dat=0x%llx)\n", p_addr_a, data_a, p_addr_b, data_b, shoe.logical_dat);
        
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
    
    // Common case: the write is contained entirely within a page
    translate_logical_addr();
    if (shoe.abort)
        return ;
    
    // printf("L(0x%08x) = P(0x%08x)\n", shoe.logical_addr, shoe.physical_addr);
    
    shoe.physical_size = shoe.logical_size;
    shoe.physical_dat = shoe.logical_dat;
    physical_set();
    /*if (!((tc_sre() && sr_s())))
     printf("set *0x%08x (phys 0x%08x) = %llx\n", shoe.logical_addr, shoe.physical_addr, shoe.physical_dat);
     
     if ((shoe.logical_addr >> 24) == 0x3f)
     printf("set *0x%08x (phys 0x%08x) = %llx\n", shoe.logical_addr, shoe.physical_addr, shoe.physical_dat);
     */
}

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

static void translate_logical_addr()
{
    const uint8_t use_srp = (tc_sre() && (shoe.logical_fc >= 4));
    
    uint32_t debug_predicted = 0;
    // First check for an entry in pmmu_cache
    
    // logical addr [is]xxxxxxxxxxxx[ps] -> value xxxxxxxxxxxx
    const uint32_t value = (shoe.logical_addr << tc_is()) >> (tc_is() + tc_ps());
    // value xxx[xxxxxxxxx] -> key xxxxxxxxx
    const uint32_t key = value & 511; // low 9 bits
    
    // Has this cache entry been set yet?
    if ((shoe.pmmu_cache[use_srp].valid_map[key/8] >> (key & 7)) & 1) {
        const pmmu_cache_entry_t entry = shoe.pmmu_cache[use_srp].entry[key];
        
        // Do the values match?
        if (entry.logical_value == value) {
            
            // Is this a write, but the page isn't marked "modified"?
            if (!(shoe.logical_is_write && !entry.modified)) {
                
                // Is this a write, and the page is write-protected?
                if (!(shoe.logical_is_write && entry.wp)) {
                    const uint32_t ps_mask = 0xffffffff >> entry.used_bits;
                    const uint32_t v_mask = ~~ps_mask;
                    shoe.physical_addr = ((entry.physical_addr<<8) & v_mask) + (shoe.logical_addr & ps_mask);
                    // debug_predicted = ((entry.physical_addr<<8) & v_mask) + (shoe.logical_addr & ps_mask);
                    return ;
                }
                // This should be rare, just let the lookup handle it
            }
            // The page descriptor needs to be marked "modified"
        }
        // Values don't match, this is a different address
    }

 
    uint64_t *rootp_ptr = (use_srp ? (&shoe.srp) : (&shoe.crp));
    const uint64_t rootp = *rootp_ptr;
    uint8_t desc_did_change = 0;
    uint8_t desc_level = 0;
    int64_t desc_addr = -1; // address of the descriptor (-1 -> register)
    uint8_t wp = 0; // Whether any descriptor in the search has wp (write protected) set
    uint8_t i;
    uint64_t desc = rootp; // Initial descriptor is the root pointer descriptor
    uint8_t desc_size = 1; // And the root pointer descriptor is always 8 bytes (1==8 bytes, 0==4 bytes)
    uint8_t used_bits = tc_is(); // Keep track of how many bits will be the effective "page size"
                                 // (If the table search terminates early (before used_bits == ts_ps()),
                                 //  then this will be the effective page size. That is, the number of bits
                                 //  we or into the physical addr from the virtual addr)
    
    desc_addr = -1; // address of the descriptor (-1 -> register)
    
    /* We'll keep shifting logical_addr such that its most significant bits are the next ti-index */
    uint32_t logical_addr = shoe.logical_addr << used_bits;
    
    // TODO: Check limit here
    
    // If root descriptor is invalid, throw a bus error
    if (rp_dt(rootp) == 0) {
        throw_bus_error(shoe.logical_addr, shoe.logical_is_write);
        return ;
    }
    
    // desc is a page descriptor, skip right ahead to search_done:
    if (rp_dt(rootp) == 1)
        goto search_done;
    
    for (i=0; i < 4; i++) { // (the condition is unnecessary - just leaving it in for clarity)
        // desc must be a table descriptor here
        
        const uint8_t ti = tc_ti(i);
        used_bits += ti;
        
        // TODO: do the limit check here
        
        // Find the index into our current i-level table
        const uint32_t index = logical_addr >> (32-ti);
        logical_addr <<= ti;
        
        // load the child descriptor
        if (shoe.logical_addr == 0)
            printf("Loading descriptor s=%llu from addr=0x%08x\n", desc_dt(desc, desc_size) & 1, (uint32_t)desc_table_addr(desc));
        const uint32_t table_base_addr = desc_table_addr(desc);
        const uint8_t s = desc_dt(desc, desc_size) & 1;
        
        // desc = pget(table_base_addr + (4 << s)*index, (4 << s));
        get_desc(table_base_addr + (4 << s)*index, (4 << s));
        desc_size = s;
        
        // Desc may be a table descriptor, page descriptor, or indirect descriptor
        
        const uint8_t dt = desc_dt(desc, desc_size);
        if (shoe.logical_addr == 0)
            printf("i=%u desc = 0x%llx dt=%u\n", i, desc, dt);
        
        // If this descriptor is invalid, throw a bus error
        if (dt == 0) {
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
            if (desc_dt(desc, desc_size) == 0) {
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
    
    // TODO: update U (used) bit
    
    wp |= desc_wp(desc, desc_size); // or in the wp flag for this page descriptor
    
    // And finally throw a bus error
    if (wp && shoe.logical_is_write) {
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
    
    const uint32_t paddr = (desc_page_addr(desc) & v_mask) + (shoe.logical_addr & ps_mask);
    shoe.physical_addr = paddr;
    
    if (shoe.logical_addr == 0) {
        printf("Success! desc = 0x%llx sz=%u bytes\n", desc, 4<<desc_size);
        printf("OLD: desc_page_addr() = 0x%08x, physical addr = 0x%08x\n", desc_page_addr(desc), (desc_page_addr(desc) & v_mask) + (shoe.logical_addr & ps_mask));
        printf("CUR: desc_page_addr() = 0x%08x, physical addr = 0x%08x\n", desc_page_addr(desc), shoe.physical_addr);
    }
    
    // Insert this translation into the cache
    
    pmmu_cache_entry_t entry;
    shoe.pmmu_cache[use_srp].valid_map[key/8] |= (1 << (key & 7));
    entry.logical_value = value;
    entry.physical_addr = (desc_page_addr(desc)) >> 8;
    entry.wp = wp;
    entry.modified = desc_m(desc, desc_size);
    entry.used_bits = used_bits;
    shoe.pmmu_cache[use_srp].entry[key] = entry;
    
    // if (debug_predicted)
    //    assert(debug_predicted == shoe.physical_addr);
}



//
// EA routines begin here:
//

#define nextword(pc) ({const uint16_t w=lget((pc),2);if (shoe.abort){return;}(pc)+=2; w;})

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
        //printf("I found address 0x%x\n", shoe.extended_addr);
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
                base_disp = nextword(mypc);
                base_disp = (base_disp<<16) | nextword(mypc);
            }
        }
        
        // Find the outer displacement
        uint32_t outer_disp = 0;
        // based on the I/IS behavior
        switch ((i<<3)|I) {
            case 0b0010: case 0b0110: case 0b1010:
                // sign-extended word-length outer displacement
                outer_disp = (int16_t)nextword(mypc);
                break;
            case 0b0011: case 0b0111: case 0b1011: {
                // long word outer displacement
                outer_disp = nextword(mypc);
                outer_disp = (outer_disp<<16) | nextword(mypc);
                break ;
            }
        }
        
        //printf("D/A=%u, reg=%u, W/L=%u, Scale=%u, F=%u, BS=%u, IS=%u, BDSize=%u, I/IS=%u\n",
        //d, r, w, s, F, b, i, z, I);
        //printf("base_addr=%x, index_val=%x, base_disp=%x, outer_disp=%x\n",
        //base_addr, index_val, base_disp, outer_disp);
        
        // Now mash all these numbers together to get an EA
        switch ((i<<3)|I) {
            case 0b0001: case 0b0010: case 0b0011:
            case 0b1001: case 0b1010: case 0b1011: {
                // Indirect preindexed
                const uint32_t intermediate = lget(base_addr + base_disp + index_val, 4);
                if (shoe.abort) return ;
                shoe.extended_addr = intermediate + outer_disp;
                shoe.extended_len = mypc - start_pc;
                // printf("addr=0x%x len=%u\n", shoe.extended_addr, shoe.extended_len);
                return ;
            }
                
            case 0b0101: case 0b0110: case 0b0111: {
                // Indirect postindexed
                const uint32_t intermediate = lget(base_addr + base_disp, 4);
                if (shoe.abort) return ;
                shoe.extended_addr = intermediate + index_val + outer_disp;
                shoe.extended_len = mypc - start_pc;
                return ;
            }
                
            case 0b1000: case 0b0000: {
                // No memory indirect action
                // EA = base_addr + base_disp + index
                shoe.extended_addr = base_addr + base_disp + index_val;
                shoe.extended_len = mypc - start_pc;
                return ;
            }
            default:
                printf("ea_decode_extended: oh noes! invalid I/IS!\n");
                // I think that 68040 *doesn't* throw an exception here...
                // FIXME: figure out what actually happens here
                break;
        }
    }
}


void ea_read()
{
    const uint8_t mode = (shoe.mr>>3)&7, reg = (shoe.mr&7);
    shoe.uncommitted_ea_read_pc = shoe.pc;
    switch (mode) {
        case 0: { // Data register direct mode
            shoe.dat = get_d(reg, shoe.sz);
            return ;
        }
        case 1: { // address register direct mode
            shoe.dat = get_a(reg, shoe.sz);
            return ;
        }
        case 2: { // address register indirect mode
            shoe.dat = lget(shoe.a[reg], shoe.sz);
            return ;
        }
        case 3: { // address register indirect with postincrement mode
            shoe.dat = lget(shoe.a[reg], shoe.sz);
            // printf("ea_read(): %u %u not implemented\n", mode, reg);
            return ;
        }
        case 4: { // address register indirect with predecrement mode
            const uint8_t delta = ((reg==7) && (shoe.sz==1))?2:shoe.sz;
            shoe.dat = lget(shoe.a[reg]-delta, shoe.sz);
            return ;
        }
        case 5: { // address register indirect with displacement mode
            const uint16_t u_disp = nextword(shoe.uncommitted_ea_read_pc);
            const int16_t disp = (int16_t)u_disp; // sign-extend word to long
            shoe.dat = lget(shoe.a[reg]+disp, shoe.sz);
            return ;
        }
        case 6: {
            // printf("ea_read(): %u %u not implemented\n", mode, reg);
            ea_decode_extended();
            if (shoe.abort) return ;
            shoe.dat = lget(shoe.extended_addr, shoe.sz);
            //eaprintf("read(0x%x) = 0x%x\n", shoe.extended_addr, shoe.dat);
            return ;
        }
        case 7: {
            switch (reg) {
                case 0: { // absolute short addressing mode
                    const int32_t addr = (int16_t)nextword(shoe.uncommitted_ea_read_pc);
                    shoe.dat = lget((uint32_t)addr, shoe.sz);
                    return ;
                }
                case 1: { // absolute long addressing mode
                    //printf("ea_read(): %u %u not implemented\n", mode, reg);
                    uint32_t addr = nextword(shoe.uncommitted_ea_read_pc);
                    addr = (addr << 16) | nextword(shoe.uncommitted_ea_read_pc);
                    // printf("addr = 0x%x\n", addr);
                    shoe.dat = lget(addr, shoe.sz);
                    // printf("ea_write(): %u %u not implemented\n", mode, reg);
                    return ;
                }
                case 2: { // program counter indirect with displacement mode
                    const uint32_t base_pc = shoe.uncommitted_ea_read_pc;
                    const uint16_t u_disp = nextword(shoe.uncommitted_ea_read_pc);
                    const int16_t disp = (int16_t)u_disp;
                    
                    shoe.dat = lget(base_pc + disp, shoe.sz);
                    
                    return ;
                }
                case 3: { // (program counter ...)
                    ea_decode_extended();
                    if (shoe.abort) return ;
                    shoe.dat = lget(shoe.extended_addr, shoe.sz);
                    return ;
                }
                case 4: { // immediate data
                    const uint16_t ext = nextword(shoe.uncommitted_ea_read_pc);
                    if (shoe.sz==1) {
                        shoe.dat = ext & 0xff;
                    } else if (shoe.sz == 2) {
                        shoe.dat = ext;
                    } else if (shoe.sz == 4) {
                        const uint16_t ext2 = nextword(shoe.uncommitted_ea_read_pc);
                        shoe.dat = ((uint32_t)ext)<<16;
                        shoe.dat |= ext2;
                    } else if (shoe.sz == 8) {
                        uint64_t q;
                        q = (ext << 16) | nextword(shoe.uncommitted_ea_read_pc);
                        q = (ext << 16) | nextword(shoe.uncommitted_ea_read_pc);
                        q = (ext << 16) | nextword(shoe.uncommitted_ea_read_pc);
                        shoe.dat = q;
                    }
                    return ;
                }
                case 5 ... 7: {
                    throw_illegal_instruction();
                    return ;
                }
            }
        }
    }
}

void ea_read_commit()
{
    const uint8_t mode = (shoe.mr>>3)&7, reg = (shoe.mr&7);
    switch (mode) {
        case 0: { // Data register direct mode
            return ; // nothing to do
        }
        case 1: { // address register direct mode
            return ; // nothing to do
        }
        case 2: { // address register indirect mode
            return ; // nothing to do
        }
        case 3: { // address register indirect with postincrement mode
            const uint8_t delta = ((reg==7) && (shoe.sz==1))?2:shoe.sz;
            shoe.a[reg] += delta;
            //printf("ea_read_commit(): %u %u not implemented\n", mode, reg);
            return ;
        }
        case 4: { // address register indirect with predecrement mode
            const uint8_t delta = ((reg==7) && (shoe.sz==1))?2:shoe.sz;
            shoe.a[reg] -= delta;
            return ;
        }
        case 5: { // address register indirect with displacement mode
            shoe.pc += 2; // this mode fetches one word
            return ;
        }
        case 6: {
            //printf("ea_read_commit(): %u %u not implemented\n", mode, reg);
            ea_decode_extended();
            if (shoe.abort) return ;
            shoe.pc += shoe.extended_len;
            return ;
        }
        case 7: {
            switch (reg) {
                case 0: { // absolute short addressing mode
                    shoe.pc+=2;
                    return ;
                }
                case 1: { // absolute long addressing mode
                    shoe.pc+=4;
                    return ;
                }
                case 2: { // program counter indirect with displacement mode
                    shoe.pc+=2;
                    return ;
                }
                case 3: { // (program counter ...)
                    ea_decode_extended();
                    if (shoe.abort) return ;
                    shoe.pc += shoe.extended_len;
                    return ;
                }
                case 4: { // immediate data
                    if (shoe.sz==1) shoe.pc += 2;
                    else shoe.pc += shoe.sz;
                    return ;
                }
                case 5 ... 7: {
                    throw_illegal_instruction();
                    return ;
                }
            }
        }
    }
}

void ea_write()
{
    const uint8_t mode = (shoe.mr>>3)&7, reg = (shoe.mr&7);
    switch (mode) {
        case 0: { // Data register direct mode
            set_d(reg, shoe.dat, shoe.sz);
            return ;
        }
        case 1: { // address register direct mode
            assert(shoe.sz==4);
            //set_a(reg, shoe.dat, shoe.sz);
            shoe.a[reg] = shoe.dat;
            return ;
        }
        case 2: { // address register indirect mode
            lset(shoe.a[reg], shoe.sz, shoe.dat);
            return ;
        }
        case 3: { // address register indirect with postincrement mode
            const uint8_t delta = ((reg==7) && (shoe.sz==1))?2:shoe.sz;
            
            lset(shoe.a[reg], shoe.sz, shoe.dat);
            if (!shoe.abort)
                shoe.a[reg] += delta;
            return ;
        }
        case 4: { // address register indirect with predecrement mode
            const uint8_t delta = ((reg==7) && (shoe.sz==1))?2:shoe.sz;
            lset(shoe.a[reg]-delta, shoe.sz, shoe.dat);
            if (!shoe.abort)
                shoe.a[reg] -= delta;
            return ;
        }
        case 5: { // address register indirect with displacement mode
            const uint16_t u_disp = nextword(shoe.pc);
            const int16_t disp = (int16_t)u_disp; // sign-extend word to long
            lset(shoe.a[reg]+disp, shoe.sz, shoe.dat);
            return ;
        }
        case 6: {
            //printf("ea_write(): %u %u not implemented\n", mode, reg);
            ea_decode_extended();
            if (shoe.abort) return ;
            lset(shoe.extended_addr, shoe.sz, shoe.dat);
            if (shoe.abort) return ;
            shoe.pc += shoe.extended_len;
            //printf("write(0x%x) = 0x%x\n", shoe.extended_addr, shoe.dat);
            return ;
        }
        case 7: {
            switch (reg) {
                case 0: { // absolute short addressing mode
                    const int32_t addr = (int16_t)nextword(shoe.pc);
                    lset((uint32_t)addr, shoe.sz, shoe.dat);
                    return ;
                }
                case 1: { // absolute long addressing mode
                    uint32_t addr = nextword(shoe.pc);
                    addr = (addr << 16) | nextword(shoe.pc);
                    lset(addr, shoe.sz, shoe.dat);
                    // printf("ea_write(): %u %u not implemented\n", mode, reg);
                    return ;
                }
                case 2: { // program counter indirect with displacement mode
                    printf("ea_write(): %u %u not implemented\n", mode, reg);
                    throw_illegal_instruction();
                    return ;
                }
                case 3: { // (program counter ...)
                    printf("ea_write(): %u %u not implemented\n", mode, reg);
                    throw_illegal_instruction();
                    return ;
                }
                case 4: { // immediate data
                    printf("ea_write(): %u %u not implemented\n", mode, reg);
                    throw_illegal_instruction();
                    return ;
                }
                case 5 ... 7: {
                    throw_illegal_instruction();
                    return ;
                }
            }
        }
    }
}

void ea_addr()
{
    const uint8_t mode = (shoe.mr>>3)&7, reg = (shoe.mr&7);
    switch (mode) {
        case 0: { // Data register direct mode
            // this must be invalid.  Figure out which exception this throws.
            printf("ea_addr(): %u %u not implemented, pc = 0x%08x\n", mode, reg, shoe.orig_pc);
            return ;
        }
        case 1: { // address register direct mode
            printf("ea_addr(): %u %u not implemented\n", mode, reg);
            // this must be invalid.  Figure out which exception this throws.
            return ;
        }
        case 2: { // address register indirect mode
            shoe.dat = shoe.a[reg];
            return ;
        }
        case 3: { // address register indirect with postincrement mode
            printf("ea_addr(): %u %u not implemented\n", mode, reg);
            return ;
        }
        case 4: { // address register indirect with predecrement mode
            printf("ea_addr(): %u %u not implemented\n", mode, reg);
            return ;
        }
        case 5: { // address register indirect with displacement mode
            int16_t disp = nextword(shoe.pc);
            shoe.dat = shoe.a[reg] + disp;
            return ;
        }
        case 6: {
            // printf("ea_addr(): %u %u not implemented\n", mode, reg);
            ea_decode_extended();
            if (shoe.abort) return ;
            shoe.dat = shoe.extended_addr;
            shoe.pc += shoe.extended_len;
            return ;
        }
        case 7: {
            switch (reg) {
                case 0: { // absolute short addressing mode
                    int32_t addr = (int16_t)nextword(shoe.pc);
                    shoe.dat = (uint32_t)addr;
                    return ;
                }
                case 1: { // absolute long addressing mode
                    uint32_t addr = (nextword(shoe.pc)) << 16;
                    addr |= (nextword(shoe.pc));
                    shoe.dat = addr;
                    return ;
                }
                case 2: { // program counter indirect with displacement mode
                    const uint32_t oldpc = shoe.pc;
                    const uint16_t displacement = nextword(shoe.pc);
                    shoe.dat = oldpc + (int16_t)displacement;
                    return ;
                }
                case 3: { // (program counter ...)
                    ea_decode_extended();
                    if (shoe.abort) return ;
                    shoe.dat = shoe.extended_addr;
                    shoe.pc += shoe.extended_len;
                    return ;
                }
                case 4: { // immediate data
                    printf("ea_addr(): %u %u not implemented, pc = 0x%08x\n", mode, reg, shoe.orig_pc);
                    return ;
                }
                case 5 ... 7: {
                    throw_illegal_instruction();
                    return ;
                }
            }
        }
    }
}
















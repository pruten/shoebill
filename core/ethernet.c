/*
 * Copyright (c) 2014, Peter Rutenbar <pruten@gmail.com>
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

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "shoebill.h"

#include "ethernet_rom/rom.c"

static uint32_t compute_nubus_crc(uint8_t *rom, uint32_t len)
{
    uint32_t i, sum = 0;
    
    for (i=0; i<len; i++) {
        uint8_t byte = rom[i];
        
        if (i==(len-9) || i==(len-10) || i==(len-11) || i==(len-12))
            byte = 0;
        
        sum = (sum << 1) + (sum >> 31) + byte;
    }
    
    rom[len-9] = sum & 0xff;
    rom[len-10] = (sum >> 8) & 0xff;
    rom[len-11] = (sum >> 16) & 0xff;
    rom[len-12] = (sum >> 24) & 0xff;
    
    return sum;
}

#define ETHPAGE() (ctx->cr >> 6)
const char *eth_r0_reg_names[16] = {
    "cr", "clda0", "clda1", "bnry",
    "tsr", "ncr", "fifo", "isr",
    "crda0", "crda1", "reserved1", "reserved2",
    "rsr", "cntr0", "cntr1", "cntr2"
};
const char *eth_1_reg_names[16] = {
    "cr", "par0", "par1", "par2",
    "par3", "par4", "par5", "curr",
    "mar0", "mar1", "mar2", "mar3",
    "mar4", "mar5", "mar6", "mar7"
};
const char *eth_w0_reg_names[16] = {
    "cr", "pstart", "pstop", "bnry",
    "tpsr", "tbcr0", "tbcr1", "isr",
    "rsar0", "rsar1", "rbcr0", "rbcr1",
    "rcr", "tcr", "dcr", "imr"
};

// command register bit masks
enum ether_cr_masks {
    cr_stp = 1<<0, // stop
    cr_sta = 1<<1, // start
    cr_txp = 1<<2, // transmit packet
    cr_rd0 = 1<<3, // remote dma command (0)
    cr_rd1 = 1<<4, // remote dma command (1)
    cr_rd2 = 1<<5, // remote dma command (2)
    cr_ps0 = 1<<6, // page select (0)
    cr_ps1 = 1<<7, // page select (1)
};

// interrupt service register bit masks
enum ether_isr_masks {
    isr_prx = 1<<0, // packet received
    isr_ptx = 1<<1, // packet transmitted
    isr_rxe = 1<<2, // receive error
    isr_txe = 1<<3, // transmit error
    isr_ovw = 1<<4, // overwrite warning
    isr_cnt = 1<<5, // counter overflow
    isr_rdc = 1<<6, // remote dma complete
    isr_rst = 1<<7, // reset status (not actually an interrupt)
};

// interrupt mask register bit masks
enum ether_imr_masks {
    imr_pxre = 1<<0, // packet received interrupt enable
    imr_ptxe = 1<<1, // packet transmitted interrupt enable
    imr_rxee = 1<<2, // receive error interrupt enable
    imr_txee = 1<<3, // transmit error interrupt enable
    imr_ovwe = 1<<4, // overwrite warning interrupt enable
    imr_cnte = 1<<5, // counter overflow interrupt enable
    imr_rdce = 1<<6, // dma complete
};

// receive configuration register bit masks
enum ether_rcr_masks {
    rcr_sep = 1<<0, // save error packets
    rcr_ar = 1<<1, // accept runt packets
    rcr_ab = 1<<2, // accept broadcast
    rcr_am = 1<<3, // accept multicast
    rcr_pro = 1<<4, // promiscuous physical
    rcr_mon = 1<<5, // monitor mode
};

// transmit configuration register bit masks
enum ether_tcr_masks {
    tcr_crc = 1<<0, // inhibit crc
    tcr_lb0 = 1<<1, // encoded loopback control (0)
    tcr_lb1 = 1<<2, // encoded loopback control (1)
    tcr_atd = 1<<3, // auto transmit disable
    tcr_ofst = 1<<4, // collision offset enable
};

// data configuration register bit masks
enum ether_dcr_masks {
    dcr_wts = 1<<0, // word transfer select
    dcr_bos = 1<<1, // byte order select
    dcr_las = 1<<2, // long address select
    dcr_ls = 1<<3, // loopback select
    dcr_arm = 1<<4, // auto-initialize remote
    dcr_ft0 = 1<<5, // fifo threshhold select (0)
    dcr_ft1 = 1<<6, // fifo threshhold select (1)
};

void nubus_ethernet_init(void *_ctx, uint8_t slotnum, uint8_t ethernet_addr[6])
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)_ctx;
    
    memset(ctx, 0, sizeof(shoebill_card_ethernet_t));
    memcpy(ctx->rom, _ethernet_rom, 4096);
    
    memcpy(ctx->ethernet_addr, ethernet_addr, 6);
    memcpy(ctx->rom, ethernet_addr, 6);
    ctx->rom[6] = 0x00;
    ctx->rom[7] = 0x00;
    
    /*
     * The first 8 bytes contain the MAC address
     * and aren't part of the CRC
     */
    compute_nubus_crc(&ctx->rom[8], 4096 - 8);
    
    ctx->cr |= cr_stp; // "STP powers up high"
    ctx->isr |= isr_rst; // I presume ISR's RST powers up high too
}

uint32_t nubus_ethernet_read_func(const uint32_t rawaddr,
                                  const uint32_t size,
                                  const uint8_t slotnum)
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)shoe.slots[slotnum].ctx;
    uint32_t result = 0;
    
    switch ((rawaddr >> 16) & 0xf) {
        case 0xd: { // ram
            const uint16_t addr = rawaddr & 0xfff;
            uint8_t *ram = ctx->ram;
            
            if (size == 1)
                result = ram[addr];
            else if (size == 2) {
                result = ram[addr] << 8;
                result |= ram[(addr+1) & 0xfff];
            }
            else
                assert(!"read: bogus size");
            
            slog("ethernet: reading from ram addr 0x%x sz=%u ", addr, size);
            
            goto done;
        }
        case 0xe: { // registers
            // For some reason, the register address bits are all inverted
            const uint8_t reg = 15 ^ ((rawaddr >> 2) & 15);
            assert(size == 1);
            
            {
                const char *name = "???";
                if (ETHPAGE() == 0) name = eth_r0_reg_names[reg];
                else if (ETHPAGE() == 1) name = eth_1_reg_names[reg];
                slog("ethernet: reading from register %u (%s) (raw=0x%x) pc=0x%x ", reg, name, rawaddr, shoe.pc);
            }
            
            if (reg == 0) { // command register (exists in all pages)
                result = ctx->cr;
                goto done;
            } else if (ETHPAGE() == 0) { // page 0
                switch (reg) {
                    default:
                        assert(!"never get here");
                        goto done;
                    case 1: // clda0 (current local dma address 0)
                        goto done;
                    
                    case 2: // clda1 (current local dma address 1)
                        goto done;
                    
                    case 3: // bnry (boundary pointer)
                        result = ctx->bnry;
                        goto done;
                    
                    case 4: // tsr (transmit status)
                        goto done;
                        
                    case 5: // ncr (number of collisions)
                        goto done;
                        
                    case 6: // fifo
                        goto done;
                        
                    case 7: // isr (interrupt status register)
                        result = ctx->isr;
                        goto done;
                        
                    case 8: // crda0 (current remote DMA address 0)
                        goto done;
                        
                    case 9: // crda1 (current remote DMA address 1)
                        goto done;
                        
                    case 10: // reserved 1
                        assert("read to reserved 1");
                        goto done;
                        
                    case 11: // reserved 2
                        assert(!"read to reserved 2");
                        goto done;
                    
                    case 12: // rsr (receive status register)
                        goto done;
                    
                    case 13: // cntr0 (tally counter 0 (frame alignment errors))
                        goto done;
                    
                    case 14: // cntr1 (tally counter 1 (crc errors))
                        goto done;
                    
                    case 15: // cntr2 (tally counter 2 (missed packet errors))
                        goto done;
                }
            } else if (ETHPAGE() == 1) { // page 1
                switch (reg) {
                    default:
                        assert(!"never get here");
                        goto done;
                    case 1: // par (physical address)
                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                        result = ctx->par[reg - 1];
                        goto done;
                    case 7: // curr (current page register)
                        result = ctx->curr;
                        goto done;
                    case 8: // mar (multicast address)
                    case 9:
                    case 10:
                    case 11:
                    case 12:
                    case 13:
                    case 14:
                    case 15:
                        result = ctx->mar[reg - 8];
                        goto done;
                }
            } else
                assert(!"read: Somebody accessed page 2 or 3!");
            
            assert(!"never get here");
            goto done;
        }
        case 0xf: { // rom
            // Byte lanes = 0101 (respond to shorts)
            // respond to (addr & 3 == 0) and (addr & 3 == 2)
            // xxxx00 xxxx10
            if ((rawaddr & 1) == 0)
                result = ctx->rom[(rawaddr >> 1) % 4096];
            
            slog("ethernet: reading from rom addr=%x ", rawaddr);
            
            goto done;
        }
        default: // Not sure what happens when you access a different addr
            assert(!"read: unknown ethernet register");
    }
    
done:

    slog("result = 0x%x\n", result);
    // slog("ethernet: reading 0x%x sz=%u from addr 0x%x\n", result, size, rawaddr);
    
    return result;
}


void nubus_ethernet_write_func(const uint32_t rawaddr,
                               const uint32_t size,
                               const uint32_t data,
                               const uint8_t slotnum)
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)shoe.slots[slotnum].ctx;
    uint32_t i;
    // slog("ethernet: writing 0x%x sz=%u to addr 0x%x\n", data, size, rawaddr);
    
    switch ((rawaddr >> 16) & 0xf) {
        case 0xd: { // ram
            const uint16_t addr = rawaddr & 0xfff;
            uint8_t *ram = ctx->ram;
            

            if (size == 1)
                ram[addr] = data;
            else if (size == 2) {
                ram[addr] = data >> 8;
                ram[(addr+1) & 0xfff] = data & 0xff;
            }
            else
                assert(!"write: bogus size");
            
            slog("ethernet: writing 0x%x sz=%u to ram addr 0x%x\n", data, size, addr);
            
            goto done;
        }
        case 0xe: { // registers
            // For some reason, the register address bits are all inverted
            const uint8_t reg = 15 ^ ((rawaddr >> 2) & 15);
            assert(size == 1);
        
            if (reg == 0) { // command register (exists in all pages)
                ctx->cr = data;
                goto done;
            } else if (ETHPAGE() == 0) { // page 0
                
                {
                    const char *name = "???";
                    if (ETHPAGE() == 0) name = eth_w0_reg_names[reg];
                    else if (ETHPAGE() == 1) name = eth_1_reg_names[reg];
                    slog("ethernet: writing 0x%02x to register %u (%s) (rawaddr=0x%x) pc=0x%x\n", data, reg, name, rawaddr, shoe.pc);
                }
                
                switch (reg) {
                    default:
                        assert(!"never get here");
                        goto done;
                    
                    case 1: // pstart (page start)
                        ctx->pstart = data;
                        goto done;
                    
                    case 2: // pstop (page stop)
                        ctx->pstop = data;
                        goto done;
                    
                    case 3: // bnry (boundary pointer)
                        ctx->bnry = data;
                        goto done;
                    
                    case 4: // tpsr (transmit page start address)
                        ctx->tpsr = data;
                        goto done;
                    
                    case 5: // tbcr0 (transmit byte count 0)
                        ctx->tbcr = (ctx->tbcr & 0xff00) | data;
                        goto done;
                    
                    case 6: // tbcr1 (transmit byte count 1)
                        ctx->tbcr = (ctx->tbcr & 0x00ff) | (data<<8);
                        goto done;
                    
                    case 7: // isr (interrupt status)
                        ctx->isr = data;
                        goto done;
                    
                    case 8: // rsar0 (remote start address 0)
                        goto done;
                    
                    case 9: // rsar1 (remote start address 1)
                        goto done;
                    
                    case 10: // rbcr0 (remote byte count 0)
                        goto done;
                    
                    case 11: // rbcr1 (remote byte count 1)
                        goto done;
                    
                    case 12: // rcr (receive configuration)
                        ctx->rcr = data;
                        goto done;
                    
                    case 13: // tcr (transmit configuration)
                        ctx->tcr = data;
                        goto done;
                    
                    case 14: // dcr (data configuration)
                        ctx->dcr = data;
                        goto done;
                    
                    case 15: // imr (interrupt mask)
                        goto done;
                }
            } else if (ETHPAGE() == 1) { // page 1
                switch (reg) {
                    default:
                        assert(!"never get here");
                        goto done;
                    case 1: // par (physical address)
                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                        ctx->par[reg - 1] = data;
                        goto done;
                    case 7: // curr (current page register)
                        ctx->curr = data;
                        goto done;
                    case 8: // mar (multicast address)
                    case 9:
                    case 10:
                    case 11:
                    case 12:
                    case 13:
                    case 14:
                    case 15:
                        ctx->mar[reg - 8] = data;
                        goto done;
                }
            } else
                assert(!"write: Somebody accessed page 2 or 3!");
            
            assert(!"never get here");
            goto done;
        }
        default:
            assert(!"write: unknown ethernet register");
    }

done:
    
    return;
}







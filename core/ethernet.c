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

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h>


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

// receive status register
enum ether_rsr_masks {
    rsr_prx = 1<<0, // packet received intact
    rsr_crc = 1<<1, // crc error
    rsr_fae = 1<<2, // frame alignment error
    rsr_fo = 1<<3, // fifo overrun
    rsr_mpa = 1<<4, // missed packet
    rsr_phy = 1<<5, // physical/multicast address (0->phys, 1->multi)
    rsr_dis = 1<<6, // received disabled
    rsr_dfr = 1<<7, // deferring
};

static void _nubus_interrupt(uint8_t slotnum)
{
    shoe.via[1].rega_input &= 0x3f & ~(1 << (slotnum - 9));
    via_raise_interrupt(2, IFR_CA1);
}

static void _clear_nubus_interrupt(uint8_t slotnum)
{
    shoe.via[1].rega_input |= (1 << (slotnum - 9));
}

/*
 * How many recv buffers (256-byte buffers) does the
 * given number of bytes require?
 */
#define eth_recv_required_bufs(a) ({ \
    const uint32_t sz = (a); \
    (sz >> 8) + ((sz & 0xff) != 0); \
})

/*
 * The number of 256-byte buffers available for writing
 * in the receive buffer (between ctx->curr and ctx->bnry)
 */
#define eth_recv_free_bufs() ({ \
    const uint8_t boundary = (ctx->bnry >= ctx->pstop) ? ctx->pstart : ctx->bnry; \
    const uint8_t curr = (ctx->curr >= ctx->pstop) ? ctx->pstart : ctx->curr; \
    const uint8_t total_bufs = ctx->pstop - ctx->pstart; \
    uint8_t f; \
    if (curr == boundary) \
        f = 0; /* This shouldn't happen */ \
    else if (curr > boundary) \
        f = (ctx->pstop - curr) + (boundary- ctx->pstart) - 1; \
    else \
        f = boundary - curr - 1; \
    f; \
})


void *_ethernet_receiver_thread(void *arg)
{
    const uint8_t multicast_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)arg;
    uint8_t *buf = malloc(4096);
    assert(buf);
    
    // While nubus_ethernet_destroy() hasn't been called
    while (!ctx->teardown) {
        struct timeval tv;
        fd_set fdset;
        int ret;
        uint32_t i;
        
        FD_ZERO(&fdset);
        FD_SET(ctx->tap_fd, &fdset);
        
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        
        ret = select(ctx->tap_fd + 1, &fdset, NULL, NULL, &tv);
        assert(ret != -1);
        
        if (FD_ISSET(ctx->tap_fd, &fdset)) {
            FD_CLR(ctx->tap_fd, &fdset);
            
            /*
             * Read in the next packet, leaving space for the 4 byte
             * header
             */
            int actual_packet_length = read(ctx->tap_fd, buf + 4, 4092);
            
            slog("ethernet: received packet bnry=%x curr=%x pstart=%x pstop=%x cr=%x ret=%d frame=0x%02x%02x\n",
                 ctx->bnry, ctx->curr, ctx->pstart, ctx->pstop, ctx->cr, actual_packet_length,
                 buf[0x10], buf[0x11]);
            
            /*
             * If it's a bogus packet length, reject it
             * (what's the actual minimum allowable packet length?)
             */
            if (actual_packet_length <= 12) {
                slog("ethernet: too small len\n");
                continue;
            }
            
            /* I'm sure A/UX can't handle > 2kb packets */
            if (actual_packet_length > 2048) {
                slog("ethernet: too high len\n");
                continue;
            }
            
            /* If it's neither multicast nor addressed to us, reject it */
            if ((memcmp(buf + 4, ctx->ethernet_addr, 6) != 0) &&
                (memcmp(buf + 4, multicast_addr, 6) != 0)) {
                slog("ethernet: bad address\n");
                continue;
            }
            
            /* A/UX seems to expect a minimum packet length (60 bytes??) */
            if (actual_packet_length < 60)
                actual_packet_length = 60;
            
            /* The number of bytes to write + the 4 byte header */
            const uint32_t received_bytes = actual_packet_length + 4;
            
            pthread_mutex_lock(&ctx->lock);
            
            /*
             * If the card isn't initialized yet, just drop the packet
             */
            if (ctx->cr & cr_stp) {
                slog("ethernet: uninit\n");
                pthread_mutex_unlock(&ctx->lock);
                continue;
            }
            
            /*
             * If the receive-register state is bogus, just drop the
             * packet
             */
            if ((ctx->pstop <= ctx->pstart) ||
                (ctx->curr < ctx->pstart) ||
                (ctx->bnry < ctx->pstart) ||
                (ctx->pstop > 0x40) ||
                (ctx->pstart == 0)) {
                // This shouldn't happen if the card is initialized
                assert(!"ethernet: receive register state is bogus");
                pthread_mutex_unlock(&ctx->lock);
                continue;
            }
            
            slog("ethernet: success, req=%u free=%u\n", eth_recv_required_bufs(received_bytes), eth_recv_free_bufs());
            
            /*
             * If there isn't enough buffer space to store the packet,
             * block until ctx->bnry is modified.
             */
            const uint8_t required_bufs = eth_recv_required_bufs(received_bytes);
            while (eth_recv_free_bufs() < required_bufs) {
                pthread_mutex_unlock(&ctx->lock);
                
                if (ctx->teardown)
                    goto bail;
                
                printf("ethernet: sleeping\n");
                usleep(50); // FIXME: use a cond variable here
                pthread_mutex_lock(&ctx->lock);
            }
            
            /* Roll around ctx->curr if necessary */
            if (ctx->curr >= ctx->pstop)
                ctx->curr = ctx->pstart;
            
            const uint8_t orig_curr = ctx->curr;
            
            /* Copy the packet to card RAM */
            for (i = 0; i < required_bufs; i++) {
                assert(ctx->curr != ctx->bnry); // this can't happen if we did our math right earlier
                
                uint8_t *ptr = &ctx->ram[ctx->curr * 256];
                memcpy(ptr, &buf[i * 256], 256);
                
                ctx->curr++;
                if (ctx->curr >= ctx->pstop)
                    ctx->curr = ctx->pstart;
            }
            assert(ctx->curr != ctx->bnry); // this can't happen if we did our math right earlier
            
            /* The packet was received intact */
            ctx->rsr = rsr_prx;
            
            /* Fill in the 4 byte packet header */
            ctx->ram[orig_curr * 256 + 0] = ctx->rsr;
            ctx->ram[orig_curr * 256 + 1] = ctx->curr;
            ctx->ram[orig_curr * 256 + 2] = received_bytes & 0xff; // low byte
            ctx->ram[orig_curr * 256 + 3] = (received_bytes >> 8) & 0xff; // high byte
            
            /* If the prx interrupt is enabled, interrupt */
            if (ctx->imr & imr_pxre) {
                ctx->isr |= isr_prx;
                _nubus_interrupt(ctx->slotnum);
            }
            
            slog("ethernet: received packet (len=%d)\n", ret);
            
            pthread_mutex_unlock(&ctx->lock);
        }
    }
   
bail:
    
    free(buf);
    
    return NULL;
}

/*
_Bool sent_arp_response = 0;

const uint8_t arp_response[60] = {
    0x66, 0x66, 0x66, 0x66, 0x66, 0x66, // router's MAC address
    0x22, 0x33, 0x55, 0x77, 0xbb, 0xdd, // card MAC address
    0x08, 0x06, // ARP frame
    0x00, 0x01, // Ethernet
    0x08, 0x00, // IP
    0x06, // MAC size
    0x04, // IP size
    0x00, 0x02, // reply
    0x66, 0x66, 0x66, 0x66, 0x66, 0x66, // router's MAC address
    192, 168, 2, 1, // router IP address
    0x22, 0x33, 0x55, 0x77, 0xbb, 0xdd, // card MAC address
    192, 168, 2, 100, // card IP address
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 // padding
};

void _test_write_packet(shoebill_card_ethernet_t *ctx)
{
    slog("ethernet: writing packet to curr=0x%02x\n", ctx->curr);
    
    uint8_t *ptr = &ctx->ram[ctx->curr * 256];
    
    // The packet was received intact
    ctx->rsr = rsr_prx;
    
    // The next packet address is the next 256 byte chunk
    ctx->curr += 1;
    
    ptr[0] = ctx->rsr;
    ptr[1] = ctx->curr; // next packet ptr (8 bit)
    ptr[2] = 60; // low byte of the packet size
    ptr[3] = 0; // high byte of the packet size
    memcpy(ptr + 4, arp_response, 60); // the packet
    
    if (ctx->imr & imr_pxre)
        ctx->isr |= isr_prx;
    
    _nubus_interrupt(ctx->slotnum);
}
*/

void *_ethernet_sender_thread(void *arg)
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)arg;
    
    slog("ethernet: ethernet_sender_thread starts...\n");
    
    // While nubus_ethernet_destroy() hasn't been called
    while (!ctx->teardown) {
        struct timeval now;
        struct timespec later;
        int ret;
        
        // Wait on the condition variable, with a timeout of 100ms
        // slog("ethernet: locking cond mutex...\n");
        pthread_mutex_lock(&ctx->sender_cond_mutex);
        // slog("ethernet: locked cond mutex...\n");
        gettimeofday(&now, NULL);
        later.tv_sec = now.tv_sec;
        later.tv_nsec = (now.tv_usec * 1000) + (1000000000 / 10);
        if (later.tv_nsec >= 1000000000) {
            later.tv_nsec -= 1000000000;
            later.tv_sec++;
        }
        
        // slog("ethernet: waiting on cond...\n");
        pthread_cond_timedwait(&ctx->sender_cond,
                               &ctx->sender_cond_mutex,
                               &later);
        assert(pthread_mutex_unlock(&ctx->sender_cond_mutex) == 0);
        
        // Only proceed if there's a packet ready to send
        if (!ctx->send_ready)
            continue;
        
        slog("ethernet: sender thread wakes up...\n");
        
        ctx->send_ready = 0;
        
        // --- Send the packet here ---
        assert(ctx->tbcr <= 2048); // sanity check the packet len
        assert(ctx->tbcr >= 42);
        
        ret = write(ctx->tap_fd, ctx->ram, ctx->tbcr);
        if (ret != ctx->tbcr) {
            slog("ethernet: write() returned %d, not %d errno=%d\n", ret, ctx->tbcr, errno);
        }
        
        // Lock the ethernet context (we're going to manipulate the ethernet registers)
        pthread_mutex_lock(&ctx->lock);
        
        // indicate that the packet has been sent
        ctx->cr &= ~cr_txp; // clear the command register txp bit
        ctx->isr |= isr_ptx; // interrupt status: packet transmitted with no errors
        
        // the "packet transmitted" interrupt really should be enabled
        if (ctx->imr & imr_ptxe) {
            _nubus_interrupt(ctx->slotnum);
            slog("ethernet: sender: sending interrupt to slot %u\n", ctx->slotnum);
        }
        
        assert(pthread_mutex_unlock(&ctx->lock) == 0);
    }
    
    return NULL;
}

void nubus_ethernet_init(void *_ctx, uint8_t slotnum, uint8_t ethernet_addr[6], int tap_fd)
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)_ctx;
    memset(ctx, 0, sizeof(shoebill_card_ethernet_t));
    memcpy(ctx->rom, _ethernet_rom, 4096);
    
    memcpy(ctx->ethernet_addr, ethernet_addr, 6);
    memcpy(ctx->rom, ethernet_addr, 6);
    ctx->rom[6] = 0x00;
    ctx->rom[7] = 0x00;
    
    ctx->slotnum = slotnum; // so the threads know which slot this is
    
    pthread_mutex_init(&ctx->lock, NULL);
    pthread_cond_init(&ctx->sender_cond, NULL);
    pthread_mutex_init(&ctx->sender_cond_mutex, NULL);
    
    pthread_create(&ctx->sender_pid, NULL, _ethernet_sender_thread, ctx);
    pthread_create(&ctx->receiver_pid, NULL, _ethernet_receiver_thread, ctx);
    
    /*
     * The first 8 bytes contain the MAC address
     * and aren't part of the CRC
     */
    compute_nubus_crc(&ctx->rom[8], 4096 - 8);
    
    ctx->cr |= cr_stp; // "STP powers up high"
    ctx->isr |= isr_rst; // I presume ISR's RST powers up high too
    
    /* Platform-specific tap code */
    ctx->tap_fd = tap_fd;
}

void nubus_ethernet_destroy_func(uint8_t slotnum)
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)shoe.slots[slotnum].ctx;
    
    ctx->teardown = 1;
    pthread_join(ctx->sender_pid, NULL);
    pthread_join(ctx->receiver_pid, NULL);
    
    pthread_mutex_destroy(&ctx->lock);
    pthread_mutex_destroy(&ctx->sender_cond_mutex);
    pthread_cond_destroy(&ctx->sender_cond);
}

uint32_t nubus_ethernet_read_func(const uint32_t rawaddr,
                                  const uint32_t size,
                                  const uint8_t slotnum)
{
    shoebill_card_ethernet_t *ctx = (shoebill_card_ethernet_t*)shoe.slots[slotnum].ctx;
    uint32_t result = 0;
    
    pthread_mutex_lock(&ctx->lock);
    
    switch ((rawaddr >> 16) & 0xf) {
        case 0xd: { // ram
            const uint16_t addr = rawaddr & 0x3fff;
            uint8_t *ram = ctx->ram;
            
            if (size == 1)
                result = ram[addr];
            else if (size == 2) {
                result = ram[addr] << 8;
                result |= ram[(addr+1) & 0x3fff];
            }
            else
                assert(!"read: bogus size");
            
            // slog("ethernet: reading from ram addr 0x%x sz=%u ", addr, size);
            
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
                        
                        // test test test
                        // if we're reading isr_ptx for the first time,
                        // send a test packet (but never again)
                        /*if ((result & isr_ptx) && (!sent_arp_response)) {
                            sent_arp_response = 1;
                            _test_write_packet(ctx);
                        }*/
                        
                        goto done;
                        
                    case 8: // crda0 (current remote DMA address 0)
                        goto done;
                        
                    case 9: // crda1 (current remote DMA address 1)
                        goto done;
                        
                    case 10: // reserved 1
                        assert(!"read to reserved 1");
                        goto done;
                        
                    case 11: // reserved 2
                        assert(!"read to reserved 2");
                        goto done;
                    
                    case 12: // rsr (receive status register)
                        result = ctx->rsr;
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

    pthread_mutex_unlock(&ctx->lock);
    
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
    
    pthread_mutex_lock(&ctx->lock);
    
    switch ((rawaddr >> 16) & 0xf) {
        case 0xd: { // ram
            const uint16_t addr = rawaddr & 0x3fff;
            uint8_t *ram = ctx->ram;
            
            if (size == 1)
                ram[addr] = data;
            else if (size == 2) {
                ram[addr] = data >> 8;
                ram[(addr+1) & 0x3fff] = data & 0xff;
            }
            else
                assert(!"write: bogus size");
            
            // slog("ethernet: writing 0x%x sz=%u to ram addr 0x%x\n", data, size, addr);
            
            goto done;
        }
        case 0xe: { // registers
            // For some reason, the register address bits are all inverted
            const uint8_t reg = 15 ^ ((rawaddr >> 2) & 15);
            assert(size == 1);
        
            {
                const char *name = "???";
                if (ETHPAGE() == 0) name = eth_w0_reg_names[reg];
                else if (ETHPAGE() == 1) name = eth_1_reg_names[reg];
                slog("ethernet: writing 0x%02x to register %u (%s) (rawaddr=0x%x) pc=0x%x\n", data, reg, name, rawaddr, shoe.pc);
            }
            
            if (reg == 0) { // command register (exists in all pages)
                
                // If we're setting TXP, wake up the sender thread
                if (((ctx->cr & cr_txp) == 0) &&
                    ((data & cr_txp) != 0)) {
                    ctx->send_ready = 1;
                    assert(pthread_mutex_lock(&ctx->sender_cond_mutex) == 0);
                    assert(pthread_cond_signal(&ctx->sender_cond) == 0);
                    assert(pthread_mutex_unlock(&ctx->sender_cond_mutex) == 0);
                }
                
                // if we're setting STA, clear isr_rst
                if (data & cr_sta)
                    ctx->isr &= ~isr_rst;
                
                // FIXME: if we're setting STP, then we probably need to set isr_rst
                
                ctx->cr = data;
                goto done;
            } else if (ETHPAGE() == 0) { // page 0
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
                    
                    case 7: { // isr (interrupt status)
                        // writing 1's clears the bits in the ISR
                        uint8_t mask = data & 0x7f; // but not the RST bit
                        ctx->isr &= ~mask;
                        
                        /*
                         * If there are packets yet to be processed,
                         * then continue to assert the isr_prx bit
                         */
                        uint8_t inc_boundary = ctx->bnry + 1;
                        if (inc_boundary >= ctx->pstop)
                            inc_boundary = ctx->pstart;
                        if (ctx->curr != inc_boundary)
                            ctx->isr |= isr_prx;
                            
                        /*
                         * If prx and ptx are no longer asserted,
                         * then we may clear the nubus interrupt.
                         */
                        if (((ctx->isr & (isr_prx | isr_ptx)) == 0) &&
                            ((ctx->cr & cr_stp) == 0))
                            _clear_nubus_interrupt(slotnum);
                        
                        goto done;
                    }
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
                        ctx->imr = data & 0x7f;
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
    
    pthread_mutex_unlock(&ctx->lock);
    return;
}







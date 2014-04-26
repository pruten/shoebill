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
#include <sys/time.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include "../core/shoebill.h"

char *via_reg_str[16] = {
    "orb",
    "ora",
    "ddrb",
    "ddra",
    "t1c-l",
    "t1c-h",
    "t1l-l",
    "t1l-h",
    "t2c-l",
    "t2c-h",
    "sr",
    "acr",
    "pcr",
    "ifr",
    "ier",
    "ora15"
};

#define set_pending_interrupt(pri) ({ \
    shoe.cpu_thread_notifications |= (1<<(pri)); \
})

// Have a VIA chip raise an interrupt
void via_raise_interrupt(uint8_t vianum, uint8_t ifr_bit)
{
    assert((vianum == 1) || (vianum == 2));
    
    via_state_t *via = &shoe.via[vianum - 1];
    // Always set the bit in ifr (I think)
    via->ifr |= (1 << ifr_bit);
    
    // Only if the bit is enabled in IER do we raise a cpu interrupt
    if (via->ier & (1 << ifr_bit)) 
        set_pending_interrupt(vianum);
    else
        printf("didn't set pending interrupt\n");
}


void process_pending_interrupt ()
{
    // FIXME: address errors on lget() here aren't handled
    
    uint32_t i;
    const uint8_t pending_interrupt = shoe.cpu_thread_notifications & 0xff;
    uint8_t priority;
    
    // Find the highest-priority pending interrupt, if any
    
    if (pending_interrupt == 0)
        return ;
    else if (pending_interrupt & (1<<7))
        priority = 7;
    else {
        for (priority=6; priority > 0; priority--) {
            if (pending_interrupt & (1<<priority))
                break;
        }
        
        // Ignore interrupt levels less than sr_mask (pri 7 is NMI)
        if (priority <= sr_mask())
            return ;
    }
    
    // If the CPU was stopped, unstop it
    shoe.cpu_thread_notifications &= ~~SHOEBILL_STATE_STOPPED;
    
    const uint16_t vector_offset = (priority + 24) * 4;
    
    printf("Interrupt pri %u! mask=%u vector_offset=0x%08x\n", priority, sr_mask(), vector_offset);
    
    // Save the old SR, and switch to supervisor mode
    const uint16_t old_sr = shoe.sr;
    set_sr_s(1);
    
    // Write a "format 0" exception frame to ISP or MSP
    push_a7(0x0000 | vector_offset, 2);
    printf("interrupt: pushed format 0x%04x to 0x%08x\n", 0x0000 | vector_offset, shoe.a[7]);
        assert(!shoe.abort);
    
    push_a7(shoe.pc, 4);
    printf("interrupt: pushed pc 0x%08x to 0x%08x\n", shoe.pc, shoe.a[7]);
        assert(!shoe.abort);
    
    push_a7(old_sr, 2);
    printf("interrupt: pushed sr 0x%04x to 0x%08x\n", old_sr, shoe.a[7]);
        assert(!shoe.abort);
    
    if (sr_m()) {
        // clear sr_m, and write a format 1 exception to the ISP
        const uint16_t old_sr2 = shoe.sr;
        
        set_sr_m(0);
        
        push_a7(0x1000 | vector_offset, 2);
            assert(!shoe.abort);
        
        push_a7(shoe.pc, 4);
            assert(!shoe.abort);
        
        push_a7(old_sr2, 2);
            assert(!shoe.abort);
    }
    
    /*
     * "When processing an interrupt exception, the MC68020/EC020 first makes an internal copy of the SR,
     * sets the privilege level to supervisor, suppresses tracing, and sets the processor interrupt mask 
     * level to the level of the interrupt being serviced."
     */
    set_sr_mask(priority);
    set_sr_t0(0);
    set_sr_t1(0);
    
    // Fetch the autovector handler address
    const uint32_t newpc = lget(shoe.vbr + vector_offset, 4);
    printf("autovector handler = *0x%08x = 0x%08x\n", shoe.vbr + vector_offset, newpc);
    assert(!shoe.abort);
    
    shoe.pc = newpc;
    
    // Clear this pending interrupt bit
    shoe.cpu_thread_notifications &= ~~(1 << priority);
}

/*
 Reset: 
 Host sends command, switch to state 0
    Device sends byte 1
 Host switches to state 2
    Device sends byte 2
 Host switches to state 3
 
 Talk:
 Host sends command, switch to state 0
    Device sends byte 0 (even)
 Hosts switches to state 1 (even = "just got even byte")
    Device sends byte 1 (odd)
 Host switches to state 2 (odd = "just got odd byte")
    Device sends byte 2 (even)
 
 
 */



// via1 ORB bits abcd efgh
// cd -> adb FSM state
// e -> adb timeout occurred / service request (?)
// f/g/h nvram stuff

#define VIA_REGB_DONE 8

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

uint16_t counter;

void via_reg_read ()
{
    const uint8_t vianum = (shoe.physical_addr >= 0x50002000) ? 2 : 1;
    const uint8_t reg = (shoe.physical_addr >> 9) & 15;
    via_state_t *via = &shoe.via[vianum - 1];
    
    printf("via_reg_read: reading from via%u reg %s (%u)\n", vianum, via_reg_str[reg], reg);
    
    switch (reg) {
        case VIA_IER:
            // According to the eratta, bit 7 is always set during a read
            shoe.physical_dat = via->ier | 0x80;
            break ;
        case VIA_IFR: {
            // Figure out whether any enabled interrupts are set, and set IRQ accordingly
            const uint8_t irq = (via->ifr & via->ier & 0x7f) ? 0x80 : 0x0;
            shoe.physical_dat = (via->ifr & 0x7f) | irq;
            break ;
        }
            
        case VIA_SR:
            shoe.physical_dat = via->sr;
            break;
            
        case VIA_ORB:
            shoe.physical_dat = via->regb;
            break;
            
        case VIA_ORA_AUX:
        case VIA_ORA:
            //if ((vianum==2) && !(via->ifr & (1<<IFR_CA1)))
                //via->rega = 0x3f;
            shoe.physical_dat = via->rega;
            break;
            
        case VIA_DDRB:
            shoe.physical_dat = via->ddrb;
            break;
            
        case VIA_DDRA:
            shoe.physical_dat = via->ddra;
            break;
            
        case VIA_T2C_LO:
            // XXX: A/UX 3.0.1 tries to precisely time a huge dbra loop
            //      using via timer2. It won't accept any result shorter than
            //      0x492, and hypothetically, this emulator could execute the
            //      loop faster than that (although not currently). So this is
            //      a super dumb hack that always returns a delta-t of 0x492
            //      between sequential reads from t2c.
            //      (oh, also, a/ux 3.0.1 cleverly reads from both t2c_lo and _hi
            //      simultaneously by doing a word-size read at VIA+0x11ff)
            counter -= 0x492;
            shoe.physical_dat = 0xffff & ((counter >> 8) | (counter << 8));
            break;
            
        default:
            printf("via_reg_read: (unhandled!)\n");
            break;
    }
}

void via_reg_write()
{
    const uint8_t vianum = (shoe.physical_addr >= 0x50002000) ? 2 : 1;
    const uint8_t reg = (shoe.physical_addr >> 9) & 15;
    const uint8_t data = (uint8_t)shoe.physical_dat;
    via_state_t *via = &shoe.via[vianum - 1];
    
    printf("via_reg_write: writing 0x%02x to via%u reg %s (%u)\n", (uint8_t)shoe.physical_dat, vianum, via_reg_str[reg], reg);
    
    switch (reg) {
        case VIA_IER: {
            const uint8_t bits = data & 0x7f;
            if (data >> 7) // if we're setting these bits
                via->ier |= bits;
            else // else, unsetting them
                via->ier &= ~~bits;
            
            // Raise a cpu-interrupt if any via interrupts are newly enabled
            if (via->ier & via->ifr & 0x7f)
                set_pending_interrupt(vianum);
            
            break ;
        }
        case VIA_IFR:
            // clear the specified bits
            via->ifr &= ~~data;

            break ;
        
        case VIA_SR:
            via->sr = data;
            break;
            
        case VIA_ORB: {
            via->regb = data;
            
            if (vianum == 1) {
                const uint8_t adb_state = (data >> 4) & 3;
                if (shoe.adb.state != adb_state) {
                    const uint8_t old_state = shoe.adb.state;
                    shoe.adb.state = adb_state;
                    
                    adb_handle_state_change(old_state, adb_state);
                }
            }
            
            break;
        }
            
        case VIA_ORA_AUX:
        case VIA_ORA:
            via->rega = data;
            break;
            
        case VIA_DDRB:
            via->ddrb = data;
            break;
            
        case VIA_DDRA:
            via->ddra = data;
            break;
        
        default:
            printf("via_reg_read: (unhandled!)\n");
            break;
    }
}

// FIXME: check_time() is bad and needs rewritten
void check_time()
{
    struct timeval now, delta_tv;
    const uint32_t hz = 10;
    
    // return ;
    
    gettimeofday(&now, NULL);
    
    delta_tv.tv_sec = now.tv_sec - shoe.start_time.tv_sec;
    if (now.tv_usec < shoe.start_time.tv_usec) {
        delta_tv.tv_sec--;
        delta_tv.tv_usec = (now.tv_usec + 1000000) - shoe.start_time.tv_usec;
    }
    else
        delta_tv.tv_usec = now.tv_usec - shoe.start_time.tv_usec;
    
    uint64_t delta = delta_tv.tv_sec * 1000;
    delta += delta_tv.tv_usec / 1000;
    
    uint64_t ticks = delta / hz;
    if (ticks <= shoe.total_ticks)
        return ;
    
    shoe.total_ticks = ticks;
    //printf("ticks = %llu\n", ticks);
    
    via_raise_interrupt(1, IFR_CA1);
    //
    shoe.via[1].rega = 0b00111101;
    via_raise_interrupt(2, IFR_CA1);
}

static long double _now (void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    long double secs = tv.tv_sec;
    long double usecs = tv.tv_usec;
    
    return secs + (usecs / 1000000.0);
}

#define fire(s) ({assert((s) >= 0); if (earliest_next_timer > (s)) earliest_next_timer = (s);})
void *via_clock_thread(void *arg)
{
    pthread_mutex_lock(&shoe.via_clock_thread_lock);
    
    const long double start_time = _now();
    uint64_t ca1_ticks = 0, ca2_ticks = 0;
    uint32_t i;
    
    while (1) {
        const long double now = _now();
        
        long double earliest_next_timer = 1.0;
        
        // Check whether the 60.15hz timer should fire (via1 CA1)
        const uint64_t expected_ca1_ticks = ((now - start_time) * 60.15L);
        if (expected_ca1_ticks > ca1_ticks) {
            // Figure out when the timer should fire next
            const long double next_firing = (1.0L/60.15L) - fmodl(now - start_time, 1.0L/60.15L);
            fire(next_firing);
            
            ca1_ticks = expected_ca1_ticks;
            
            // Raise VIA1 CA1
            via_raise_interrupt(1, IFR_CA1);
        }
        
        // Check whether the 1hz timer should fire (via1 CA2)
        const uint64_t expected_ca2_ticks = now - start_time;
        if (expected_ca2_ticks > ca2_ticks) {
            const long double next_firing = 1.0L - fmodl(now - start_time, 1.0L);
            fire(next_firing);
            
            ca2_ticks = expected_ca2_ticks;
            
            via_raise_interrupt(1, IFR_CA2);
            
            /*via_raise_interrupt(1, IFR_TIMER1);
            via_raise_interrupt(1, IFR_TIMER2);
            via_raise_interrupt(2, IFR_TIMER1);
            via_raise_interrupt(2, IFR_TIMER2);*/
        }
        
        // Check if any nubus cards have interrupt timers
        shoe.via[1].rega = 0b00111111;
        for (i=9; i<15; i++) {
            if (!shoe.slots[i].connected)
                continue;
            
            if (now >= (shoe.slots[i].last_fired + (1.0L/shoe.slots[i].interrupt_rate))) {
                shoe.slots[i].last_fired = now;
                
                fire(1.0L/shoe.slots[i].interrupt_rate);
                
                if (shoe.slots[i].interrupts_enabled) {
                    // shoe.via[1].rega = 0b00111111 & ~~(1<<(i-9));
                    shoe.via[1].rega &= 0b00111111 & ~~(1<<(i-9));
                    via_raise_interrupt(2, IFR_CA1);
                    printf("Fired nubus interrupt %u\n", i);
                }
            }
        }
        
        
        
        
        usleep((useconds_t)(earliest_next_timer * 1000000.0L));
    }
}



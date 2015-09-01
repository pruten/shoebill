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
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include "../core/shoebill.h"

char *via_reg_str[16] = {
    "regb",
    "rega",
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
    
    // if the CPU was stopped, wake it up
    if (shoe.cpu_thread_notifications & SHOEBILL_STATE_STOPPED) {
        unstop_cpu_thread();
    }
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
    
    shoe.cpu_thread_notifications &= ~~SHOEBILL_STATE_STOPPED;
    
    const uint16_t vector_offset = (priority + 24) * 4;
    
    slog("Interrupt pri %u! mask=%u vector_offset=0x%08x\n", priority, sr_mask(), vector_offset);
    
    // Save the old SR, and switch to supervisor mode
    const uint16_t old_sr = shoe.sr;
    set_sr_s(1);
    
    // Write a "format 0" exception frame to ISP or MSP
    push_a7(0x0000 | vector_offset, 2);
    slog("interrupt: pushed format 0x%04x to 0x%08x\n", 0x0000 | vector_offset, shoe.a[7]);
        assert(!shoe.abort);
    
    push_a7(shoe.pc, 4);
    slog("interrupt: pushed pc 0x%08x to 0x%08x\n", shoe.pc, shoe.a[7]);
        assert(!shoe.abort);
    
    push_a7(old_sr, 2);
    slog("interrupt: pushed sr 0x%04x to 0x%08x\n", old_sr, shoe.a[7]);
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
    slog("autovector handler = *0x%08x = 0x%08x\n", shoe.vbr + vector_offset, newpc);
    assert(!shoe.abort);
    
    shoe.pc = newpc;
    
    // Clear this pending interrupt bit
    shoe.cpu_thread_notifications &= ~~(1 << priority);
}


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

// Interrupt flag register bits
#define VIA_IFR_CA2 (1<<0)
#define VIA_IFR_CA1 (1<<1)
#define VIA_IFR_SHIFT_REG (1<<2)
#define VIA_IFR_CB2 (1<<3)
#define VIA_IFR_CB1 (1<<4)
#define VIA_IFR_T2 (1<<5)
#define VIA_IFR_T1 (1<<6)
#define VIA_IFR_IRQ (1<<7)

static long double _now (void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    const long double secs = tv.tv_sec;
    const long double usecs = tv.tv_usec;
    const long double result = secs + (usecs / 1000000.0);
    return result;
}

static void handle_pram_write_byte (void)
{
    pram_state_t *pram = &shoe.pram;
    
    slog("PRAMPRAM: wrote_byte 0x%02x\n", pram->byte);
    
    pram->mode = PRAM_READ;
    pram->byte = 0;
}

static void handle_pram_read_byte (void)
{
    pram_state_t *pram = &shoe.pram;
    
    assert(pram->command_i < 8);
    pram->command[pram->command_i++] = pram->byte;
    
    slog("PRAMPRAM: read_byte: 0x%02x\n", pram->byte);
    
    // If this is a pram-read/write...
    if ((pram->command[0] & 0x78) == 0x38) {
        const _Bool isget = pram->command[0] >> 7;
        const uint8_t addr = (pram->command[0] << 5) | ((pram->command[1] >> 2) & 0x1f);
        
        if ((pram->command_i == 3) && !isget) { // complete set command
            pram->mode = PRAM_READ; // stay in read-mode
            pram->data[addr] = pram->command[2];
            
            if (pram->callback)
                pram->callback(pram->callback_param, addr, pram->command[2]);
            
            slog("PRAMPRAM: setting pram addr 0x%02x = 0x%02x\n", addr, pram->command[2]);
            
            pram->byte = 0;
            pram->command_i = 0;
            
            return ;
        }
        else if ((pram->command_i == 2) && isget) { // complete get command
            pram->mode = PRAM_WRITE; // switch to write-mode
            pram->byte = pram->data[addr];
            pram->command_i = 0;
            
            slog("PRAMPRAM: fetching pram addr 0x%02x (= 0x%02x)\n", addr, pram->byte);
            
            return ;
        }
        else { // incomplete command, keep reading
            assert(pram->command_i < 4);
            pram->mode = PRAM_READ; // keep reading
            
            return ;
        }
    }
    
    // if this is clock-read/write
    else if (~bmatch(pram->command[0], x 00x xx 01)) {
        const _Bool isget = pram->command[0] >> 7;
        const uint8_t addr = (pram->command[0] >> 2) & 3;
        const _Bool mysterybit = (pram->command[0] >> 4) & 1; // FIXME: What does this do?
        
        if ((pram->command_i == 2) && !isget) { // complete set command
            pram->mode = PRAM_READ; // stay in read-mode
            
            slog("PRAMPRAM: setting time byte %u to 0x%02x (mysterybit=%u)\n", addr, pram->command[1], mysterybit);
            
            pram->byte = 0;
            pram->command_i = 0;
            return ;
        }
        else if ((pram->command_i == 1) && isget) { // complete get command
            const uint32_t now = time(NULL) + 0x7c25b080;
            //uint32_t now = 0xafd56d80; // Tue, 24 Jun 1997 12:26:40 GMT
            const uint8_t now_byte = now >> (8*addr);
            
            pram->mode = PRAM_WRITE;
            pram->byte = now_byte;
            pram->command_i = 0;
            
            slog("PRAMPRAM: fetching time byte %u of 0x%08x (mysterybit=%u)\n", addr, now, mysterybit);
            return ;
        }
        else { // incomplete command, keep reading
            assert(pram->command_i < 3);
            pram->mode = PRAM_READ;
            
            return ;
        }
    }
    
    // This is mystery command # 2
    else if (pram->command[0] == 0x35) {
        // Arrives in pairs of two bytes
        if (pram->command_i == 2) {
            slog("PRAMPRAM: mystery command 2 0x%02x 0x%02x (?))\n", pram->command[0], pram->command[1]);
            
            pram->mode = PRAM_READ;
            pram->command_i = 0;
            return ;
        }
        else { // keep reading
            assert(pram->command_i < 3);
            pram->mode = PRAM_READ;
            
            return ;
        }
    
    }
    
    
    slog("PRAMPRAM: don't understand this command\n");
    pram->command_i = 0;
    pram->mode = PRAM_READ;
}

static void handle_pram_state_change (void)
{
    pram_state_t *pram = &shoe.pram;
    
    // If rtcClock or rtcEnable changed, then the state machine needs updating
    if (pram->last_bits == (shoe.via[0].regb_output & shoe.via[0].ddrb & 6))
        return ;
    
    slog("PRAMPRAM: pram->last_bits = %u, (shoe.via[0].regb & 6) = %u\n", pram->last_bits, (shoe.via[0].regb_output & shoe.via[0].ddrb & 6));
    
    // it doesn't matter what the last rtcData value was
    const _Bool last_rtcClock = (pram->last_bits >> 1) & 1;
    const _Bool last_rtcEnable = (pram->last_bits >> 2) & 1;
    
    const _Bool rtcData = shoe.via[0].regb_output & 1;
    const _Bool rtcClock = (shoe.via[0].regb_output >> 1) & 1;
    const _Bool rtcEnable = (shoe.via[0].regb_output >> 2) & 1;
    
    slog("PRAMPRAM: bits changed %u%ux -> %u%u%u\n", last_rtcEnable, last_rtcClock, rtcEnable, rtcClock, rtcData);
    
    if (rtcEnable) {
        // rtcEnable==true => the RTC chip is enabled and we are talking to it
        // Not sure what happens when you toggle data/clock bits while rtcEnable is asserted...
        if (last_rtcEnable)
            slog("PRAMPRAM: toggled bits while rtcEnable was asserted!\n");
        goto done;
    }
    
    if (!rtcEnable && last_rtcEnable) {
        // if rtcEnable went from hi to low, then reset all the state stuff
        pram->mode = PRAM_READ;
        pram->command_i = 0; // the current command byte we're working on
        pram->bit_i = 0; // the current bit num we're reading/writing
        pram->byte = 0; // the current byte we're reading/writing
        memset(pram->command, 0, 8);
        goto done;
    }
    
    
    switch (pram->mode) {
        case PRAM_READ: {
            // if rtcClock goes from low to hi, then rtcData represents a new bit
            if (rtcClock && !last_rtcClock) {
                pram->byte <<= 1;
                pram->byte |= rtcData;
                pram->bit_i++;
            }
            
            if ((shoe.via[0].ddrb & 1) == 0) {
                // This is input-mode -- should be output-mode
                slog("PRAMPRAM: BOGUS MODE ddrb&1 == 0\n");
            }
            
            
            if (pram->bit_i >= 8) {
                pram->bit_i = 0;
                handle_pram_read_byte();
            }
            goto done;
        }
            
        case PRAM_WRITE: {
            // if rtcClock goes from hi to low, load in the new rtcData bit
            if (!rtcClock && last_rtcClock) {
                const uint8_t newData = (pram->byte >> (7 - pram->bit_i)) & 1;
                shoe.via[0].regb_input &= 0xfe;
                shoe.via[0].regb_input |= newData;
            }
            
            // if B goes from low to hi, skip to the next bit
            if (rtcClock && !last_rtcClock)
                pram->bit_i++;
            
            assert((shoe.via[0].ddrb & 1) == 0);
            
            if (pram->bit_i >= 8) {
                pram->bit_i = 0;
                handle_pram_write_byte();
            }
            goto done;
        }
            
        default:
            assert(!"can't get here");
    }
    
done:
    
    // Remember the last state of the bits
    pram->last_bits = (shoe.via[0].regb_output & shoe.via[0].ddrb & 6);
}

void reset_via_state (void)
{
    uint8_t pram_data[256];
    shoebill_pram_callback_t callback = shoe.pram.callback;
    void *callback_param = shoe.pram.callback_param;
    
    memcpy(pram_data, shoe.pram.data, 256);
    
    init_via_state(pram_data, callback, callback_param);
}

void init_via_state (uint8_t pram_data[256], shoebill_pram_callback_t callback, void *callback_param)
{
    /* -- Zero everything -- */
    
    memset(&shoe.pram, 0, sizeof(pram_state_t));
    memset(&shoe.via, 0, 2 * sizeof(via_state_t));
    
    // Jeez, keep this straight!
    // DDR 0 -> input (from pins to OS)
    //     1 -> output (from OS to pins)
    
    /* -- Initialize VIA1 -- */
    
    /* VIA 1 reg A
     * Bit 7 - input  - vSCCWrReq
     * Bit 6 - input  - CPU.ID1
     * Bit 5 - output - vHeadSel
     * Bit 4 - output - vOverlay
     * Bit 3 - output - vSync
     * Bit 2-0 unused
    */
    shoe.via[0].ddra = ~b(00111000);
    
    /* VIA 1 reg B 
     * Bit 7 - output - vSndEnb
     * Bit 6 - unused
     * Bit 5 - output - vFDesk2
     * Bit 4 - output - vFDesk1
     * Bit 3 - input  - vFDBInt
     * Bit 2 - output - rTCEnb
     * Bit 1 - output - rtcClk
     * Bit 0 - in/out - rtcData (initialize to output)
     */
    shoe.via[0].ddrb = ~b(10110111); // A/UX apparently neglects to initialize ddra/b
    
    /* -- Initialize VIA2 -- */
    
    /* VIA 2 reg A
     * Bit 7 - unused
     * Bit 6 - unused
     * Bit 5 - Interrupt for slot 15
     * ...
     * Bit 0 - Interrupt for slot 9
     */
    shoe.via[1].ddra = 0x00; // via2/rega consists of input pins for nubus interrupts
    shoe.via[1].rega_input = ~b(00111111); // no nubus interrupts currently asserted
    
    /* VIA 2 reg B
     * Bit 7 - output - v2VBL
     * Bit 6 - input  - v2SNDEXT
     * Bit 5 - input  - v2TM0A (nubus transfer what??)
     * Bit 4 - input  - v2TM1A
     * Bit 3 - output - AMU/PMMU control
     * Bit 2 - output - v2PowerOff (but leave this in input mode)
     * Bit 1 - output - v2BusLk
     * Bit 0 - output - v2cdis
     */
    shoe.via[1].ddrb = 0x00;
    shoe.via[1].ddrb = ~b(10001011);
    // FIXME: apparently via2/regb bit 7 is tied to VIA1, and driven by timer T1, to
    //        generate 60.15hz (really 60.0hz) interrupts on VIA1
    //        emulate this more accurately!
    
    // The power unit is wired to bit 2, waiting for it to be set 0.
    // I guess we're supposed to leave it as an input, because the shutdown
    // routine first sets the bit to 0, *then* switches the direction to output.
    // FIXME: verify that this is correct
    
    /* -- Initialize PRAM -- */
    
    pram_state_t *pram = &shoe.pram;
    pram->mode = PRAM_READ;
    
    memcpy(pram->data, pram_data, 256);
    pram->callback = callback;
    pram->callback_param = callback_param;
    
    /* -- Init clock stuff -- */
    const long double now = _now();
    shoe.via[0].t1_last_set = now;
    shoe.via[0].t2_last_set = now;
    shoe.via[1].t1_last_set = now;
    shoe.via[1].t2_last_set = now;
}

#define E_CLOCK 783360
#define V2POWEROFF_MASK 0x04

#define _via_get_delta_counter(last_set) ({ \
    const long double delta_t = now - (last_set); \
    const long double delta_ticks = fmodl((delta_t * (long double)E_CLOCK), 0x80000000); \
    /* The VIA timers decrement by 2 for every E_CLOCK tick */ \
    const uint32_t delta_counter = ((uint32_t)delta_ticks) << 1; \
    delta_counter; \
})

// from the pins' perspective
#define VIA_REGA_PINS(n) ((shoe.via[(n)-1].rega_output & shoe.via[(n)-1].ddra) | \
                          (shoe.via[(n)-1].rega_input & (~~shoe.via[(n)-1].ddra)))

#define VIA_REGB_PINS(n) ((shoe.via[(n)-1].regb_output & shoe.via[(n)-1].ddrb) | \
                          (shoe.via[(n)-1].regb_input & (~~shoe.via[(n)-1].ddrb)))

static void _via_poweroff(void)
{
    slog("Poweroff!\n");
    // exit(0);
}

static uint8_t via_read_reg(const uint8_t vianum, const uint8_t reg, const long double now)
{
    via_state_t *via = &shoe.via[vianum - 1];
    
    slog("via_reg_read: reading from via%u reg %s (%u) (shoe.pc = 0x%08x)\n", vianum, via_reg_str[reg], reg, shoe.pc);
    
    switch (reg) {
        case VIA_ACR:
            return via->acr;
            
        case VIA_PCR:
            return via->pcr;
            
        case VIA_IER:
            // According to the eratta, bit 7 is always set during a read
            return via->ier | 0x80;
            
        case VIA_IFR: {
            // Figure out whether any enabled interrupts are set, and set IRQ accordingly
            const uint8_t irq = (via->ifr & via->ier & 0x7f) ? 0x80 : 0x0;
            return (via->ifr & 0x7f) | irq;
        }
            
        case VIA_SR:
            return via->sr;

        case VIA_ORB: {
            /*
             * FIXME: this is not exactly correct.
             *        if input latching is enabled, then the value of input pins
             *        is held in escrow until a CB1 transition occurs. I'm not doing that.
             */
            slog("via_reg_read: FYI: regb_output=0x%02x regb_input=0x%02x ddrb=0x%02x combined=0x%02x\n",
                   shoe.via[vianum-1].regb_output, shoe.via[vianum-1].regb_input, via->ddrb, VIA_REGB_PINS(vianum));
            return VIA_REGB_PINS(vianum);
        }
            
        case VIA_ORA_AUX:
        case VIA_ORA: {
            /*
             * FIXME: This is not exactly correct either, and it behaves differently from regb
             *        Reading regA never returns the contents of the output register directly,
             *        it returns the the value of the pins - unless input latching is enabled,
             *        then it holds the pin values in escrow until a CA1 transition occurs.
             *        I'm just returning the value of the "pins"
             */
            return VIA_REGA_PINS(vianum);
        }
            
        case VIA_DDRB:
            return via->ddrb;
            
        case VIA_DDRA:
            return via->ddra;

        case VIA_T2C_HI: {
            uint16_t counter = via->t2c - (uint16_t)_via_get_delta_counter(via->t2_last_set);
            
            /*
             * This is a hack to allow A/UX 3.x.x to boot on fast hosts.
             * The A/UX 3 kernel calls a function, SetUpTimeK, during boot
             * to run a giant drba-to-self loop and time it via T2C.
             * If the loop completes too quickly, (quicker than 0x492 E_CLOCK
             * ticks on 3.0.0), it rejects it and tries again.
             * 
             * SetUpTimeK reads T2C twice and compares them to determine the
             * time elapsed. I don't want to count on getting the order correct, 
             * however. Therefore, we will fake it out by randomly adding 0x500
             * to the the value of the clock whenever it's read by SetUpTimeK.
             * By the Monte Carlo method, we'll eventually get a case where
             * two sequential reads differ by at least 0x500.
             *
             * FIXME: optimize this better, stop using coff_find_func()
             */
            if (sr_s()) {
                coff_symbol *symb = coff_find_func(shoe.coff, shoe.pc);
                if (symb && strcmp(symb->name, "SetUpTimeK") == 0) {
                    if (random() & 1)
                        counter += 0x500;
                }
            }
            
            return counter >> 8;
        }
        case VIA_T2C_LO: {
            const uint16_t counter = via->t2c - (uint16_t)_via_get_delta_counter(via->t2_last_set);
            via->ifr &= ~~VIA_IFR_T2; // Read from T2C_LOW clears TIMER 2 interrupt
            return (uint8_t)counter;
        }
            
        case VIA_T1C_LO:
            via->ifr &= ~~VIA_IFR_T1; // Read from T1C_LOW clears TIMER 1 interrupt
            return 0; // FIXME
            
        case VIA_T1C_HI:
            return 0; // FIXME
            
        case VIA_T1L_LO:
            return 0; // FIXME
            
        case VIA_T1L_HI:
            return 0; // FIXME
    }
    assert(!"never get here");
}

static void via_write_reg(const uint8_t vianum, const uint8_t reg, const uint8_t data, const long double now)
{
    via_state_t *via = &shoe.via[vianum - 1];
    
    slog("via_reg_write: writing 0x%02x to via%u reg %s (%u) (pc=0x%08x)\n", data, vianum, via_reg_str[reg], reg, shoe.pc);
    
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
            
            via->regb_output = data;
            
            if (vianum == 1) {
                const uint8_t adb_state = (data >> 4) & 3; // just assume that the corresponding ddrb bits are marked "output"
                if (shoe.adb.state != adb_state) {
                    const uint8_t old_state = shoe.adb.state;
                    shoe.adb.state = adb_state;
                    
                    adb_handle_state_change(old_state, adb_state);
                }
                
                handle_pram_state_change();
            }
            
            if ((vianum == 2) &&
                (via->ddrb & V2POWEROFF_MASK) &&
                !(via->regb_output & V2POWEROFF_MASK))
                _via_poweroff();
            
            break;
        }
            
        case VIA_ORA_AUX:
        case VIA_ORA: {
            
            via->rega_output = data;
            
            break;
        }
            
        case VIA_DDRB: {
            via->ddrb = data;
            if ((vianum == 2) &&
                (via->ddrb & V2POWEROFF_MASK) &&
                !(via->regb_output & V2POWEROFF_MASK))
                _via_poweroff();
            break;
        }
            
        case VIA_DDRA:
            via->ddra = data;
            break;
        
        case VIA_ACR:
            via->acr = data;
            break;
            
        case VIA_PCR:
            via->pcr = data;
            break;
            
        case VIA_T2C_LO:
            break;
            
        case VIA_T2C_HI:
            via->ifr &= ~~VIA_IFR_T2; // Write to T2C_HI clears TIMER 2 interrupt
            via->t2_last_set = now;
            via->t2_interrupt_enabled = 1;
            break;
            
        case VIA_T1C_LO:
            break;
            
        case VIA_T1C_HI:
            via->ifr &= ~~VIA_IFR_T1; // Write to T1C_HI clears TIMER 1 interrupt
            break;
            
        case VIA_T1L_LO:
            break;
            
        case VIA_T1L_HI:
            break;
    }
}

void via_write_raw (void)
{
    const uint8_t vianum = ((shoe.physical_addr >> 13) & 1) + 1;
    const uint8_t reg = (shoe.physical_addr >> 9) & 15;
    
    pthread_mutex_lock(&shoe.via_cpu_lock);
    
    if (shoe.physical_size == 1) {
        const long double now = ((reg >= VIA_T1C_LO) && (reg <= VIA_T2C_HI)) ? _now() : 0.0;
        // Common case: writing to only one register
        
        via_write_reg(vianum, reg, (uint8_t)shoe.physical_dat, now);
    }
    else if ((shoe.physical_size == 2) && ((shoe.physical_addr & 0x1ff) == 0x1ff)) {
        const long double now = ((reg >= VIA_T1C_LO) && ((reg+1) <= VIA_T2C_HI)) ? _now() : 0.0;
        // Uncommon case: writing to two registers simultaneously
        
        slog("via_write_raw: writing to two registers simultaneously %u and %u (0x%x)\n", reg, reg+1 , (uint32_t)shoe.physical_dat);
        
        assert(reg != 15); // If A/UX is trying to write to two VIA chips simultanously, that's not cool
        
        via_write_reg(vianum, reg, (uint8_t)(shoe.physical_dat >> 8), now);
        via_write_reg(vianum, reg+1, (uint8_t)shoe.physical_dat, now);
    }
    else
        assert(!"Writing multiple bytes to the same VIA register!");

    pthread_mutex_unlock(&shoe.via_cpu_lock);
}

void via_read_raw (void)
{
    const uint8_t vianum = ((shoe.physical_addr >> 13) & 1) + 1;
    const uint8_t reg = (shoe.physical_addr >> 9) & 15;
    
    pthread_mutex_lock(&shoe.via_cpu_lock);
    
    if (shoe.physical_size == 1) {
        const long double now = ((reg >= VIA_T1C_LO) && (reg <= VIA_T2C_HI)) ? _now() : 0.0;
        
        // Common case: reading only one register
        shoe.physical_dat = via_read_reg(vianum, reg, now);
    }
    else if ((shoe.physical_size == 2) && ((shoe.physical_addr & 0x1ff) == 0x1ff)) {
        const long double now = ((reg >= VIA_T1C_LO) && ((reg+1) <= VIA_T2C_HI)) ? _now() : 0.0;
        
        // Uncommon case: reading from two registers simultaneously
        
        slog("via_read_raw: reading from two registers simultaneously %u and %u\n", reg, reg+1);
        
        assert(reg != 15); // If A/UX is trying to read from two VIA chips simultaneously, that's not cool
        
        uint16_t result = via_read_reg(vianum, reg, now);
        result = (result << 8) | via_read_reg(vianum, reg+1, now);
        shoe.physical_dat = result;
        
    }
    else
        assert(!"Reading multiple bytes from the same VIA register!");
    
    pthread_mutex_unlock(&shoe.via_cpu_lock);
}

#define fire(s) ({assert((s) >= 0); if (earliest_next_timer > (s)) earliest_next_timer = (s);})
void *via_clock_thread(void *arg)
{
    pthread_mutex_lock(&shoe.via_clock_thread_lock);
    // const long double multiplier = 1.0 / 60.0;
    const long double multiplier = 1.0;
    const long double start_time = multiplier * _now();
    uint64_t ca1_ticks = 0, ca2_ticks = 0;
    uint32_t i;
    
    while (1) {
        pthread_mutex_lock(&shoe.via_cpu_lock);
        
        const long double now = multiplier * _now();
        
        long double earliest_next_timer = 1.0;
        const uint32_t via1_t2_delta = _via_get_delta_counter(shoe.via[0].t2_last_set);
        
        
        /*
         * Check whether the 60hz timer should fire (via1 CA1)
         *
         * Note! Inside Macintosh claims this should be 60.15hz,
         * but every version of A/UX configures the timer to be
         * exactly 60.0hz
         */
        const uint64_t expected_ca1_ticks = ((now - start_time) * 60.0L);
        if (expected_ca1_ticks > ca1_ticks) {
            // Figure out when the timer should fire next
            const long double next_firing = (1.0L/60.0L) - fmodl(now - start_time, 1.0L/60.0L);
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
        
        // I'm only checking VIA1 T2, since the time manager only seems to use/care about that timer
        if (shoe.via[0].t2_interrupt_enabled) {
            if (via1_t2_delta >= shoe.via[0].t2c) {
                shoe.via[0].t2_interrupt_enabled = 0;
                via_raise_interrupt(1, IFR_TIMER2);
            }
            else {
                fire((long double)(shoe.via[0].t2c - via1_t2_delta) / (E_CLOCK / 2.0));
            }
        }
                                                              
        pthread_mutex_unlock(&shoe.via_cpu_lock);
        
        usleep((useconds_t)(earliest_next_timer * 1000000.0L));
        
        if (shoe.via_thread_notifications & SHOEBILL_STATE_RETURN) {
            pthread_mutex_unlock(&shoe.via_clock_thread_lock);
            return NULL;
        }
    }
}



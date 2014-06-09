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
#include <pthread.h>
#include "../core/shoebill.h"
#include <assert.h>

// via1 ORB bits abcd efgh
// cd -> adb FSM state
// e -> adb timeout occurred / service request (?)
// f/g/h nvram stuff

#define VIA_REGB_ADB_STATUS 8

/*
 Reset:
 
 -> OS writes reset command to sr, sets state 0
 <- chip sets ADB status line, raises interrupt
 -> OS dummy-reads sr, sets state 2
 <- chip sets ADB status line, raises interrupt
 -> OS dummy-reads sr, sets state 3
 
 
 Talk:
 -> OS writes talk command to sr, sets state 0
 <- chip responds with an interrupt. Status=0 -> poll(unsolicited?), status=1 -> response?
 -> OS sets state=1
 <- chip responds with an interrupt. Status=0 -> timeout, status=1 -> not-timedout?
 -> OS reads first byte from sr, sets state=2
 <- chip responds with interrupt. Status=0 -> service request, service=1 no-service-reques
 -> OS reads second byte from sr, sets state=3
 
 
 Listen:
 -> OS writes listen command to sr, sets state 0
 <- chip 
 ???
 
 Keyboard:
 -> OS sends EXISTS (TALK) for id 2, reg 3
 <- (if no keyboard: timeout. If keyboard: respond with some data)
 
 -> OS sends FLUSH
 <- Keyboard responds
 
 -> OS sends LISTEN for id 2, reg 3 (data: 0x2201 - sets the service request enable bit)
 <- Keyboard respnods
 
 -> OS sends TALK for id 2 reg 2
 <- Keyboard responds with some keyboard setup data
 
 -> OS sends TALK for id 2 reg 0
 <- if there's a key, keyboard returns key data. Otherwise, response times out.
 
*/

void reset_adb_state()
{
    pthread_mutex_t lock = shoe.adb.lock;
    
    memset(&shoe.adb, 0, sizeof(adb_state_t));
    memset(&shoe.key, 0, sizeof(keyboard_state_t));
    memset(&shoe.mouse, 0, sizeof(mouse_state_t));
    
    // Put the adb chip in state 3 (idle)
    shoe.adb.state = 3;
    
    shoe.adb.lock = lock;
}

void init_adb_state()
{
    memset(&shoe.adb, 0, sizeof(adb_state_t));
    memset(&shoe.key, 0, sizeof(keyboard_state_t));
    memset(&shoe.mouse, 0, sizeof(mouse_state_t));
    
    // Put the adb chip in state 3 (idle)
    shoe.adb.state = 3;
    
    pthread_mutex_init(&shoe.adb.lock, NULL);
}

void adb_start_service_request()
{
    //slog("adb_start_service_request: pending_requests = 0x%02x\n", shoe.adb.pending_service_requests);
    if (shoe.adb.pending_service_requests) {
        shoe.adb.service_request = 1;
        
        shoe.adb.poll = shoe.adb.pending_poll;
        shoe.adb.pending_poll = 0;
        
        via_raise_interrupt(1, IFR_SHIFT_REG);
    }
}

void adb_request_service_request(uint8_t id)
{
    shoe.adb.pending_service_requests |= (1 << id);
    shoe.adb.pending_poll = 1;
    
    if (shoe.adb.state == 3) {
        adb_start_service_request();
    }
}

static void keyboard_talk(uint8_t reg)
{
    shoe.adb.timeout = 0;
    
    switch (reg) {
        case 0:
            if (shoe.key.key_i > 0) {
                shoe.adb.data[0] = shoe.key.keys[0].code_b;
                shoe.adb.data[1] = shoe.key.keys[0].code_a;
                shoe.adb.data_len = 2;
                shoe.key.key_i--;
                memmove(&shoe.key.keys[0], &shoe.key.keys[1], shoe.key.key_i * sizeof(shoe.key.keys[0]));
            }
            else
                shoe.adb.timeout = 1;

            break ;
            
        case 2:
            // All the modifier keys are up
            shoe.adb.data[0] = ~b(01111111);
            shoe.adb.data[1] = ~b(11100111);
            shoe.adb.data_len = 2;
            break ;
            
        case 1:
            shoe.adb.timeout = 1;
            break ;
            
        case 3:
            shoe.adb.data[0] = 0x02; // device address == 2 -> keyboard
            shoe.adb.data[1] = 0x02; // device handler ID == 0x03 -> Apple Extended Keyboard
            shoe.adb.data_len = 2;
            break ;
    }
    slog("keyboard_talk: reg=%u timeout=%u data=0x%02x%02x datalen=%u\n", reg, shoe.adb.timeout, shoe.adb.data[0], shoe.adb.data[1], shoe.adb.data_len);
}

static void mouse_talk(uint8_t reg)
{
    shoe.adb.timeout = 0;
    
    slog("mouse_talk: reg=%u\n", reg);
    switch (reg) {
            
        case 0:
            if (shoe.mouse.changed) {
                const int32_t hi_delta_limit = 32;
                const int32_t low_delta_limit = -32;
                
                int32_t x = shoe.mouse.delta_x;
                int32_t y = shoe.mouse.delta_y;
                
                //slog("mouse_talk: x=%d, y=%d button=%u\n", shoe.mouse.delta_x, shoe.mouse.delta_y, shoe.mouse.button_down);
                
                
                if (x > hi_delta_limit) x = hi_delta_limit;
                if (x < low_delta_limit) x = low_delta_limit;
                if (y > hi_delta_limit) y = hi_delta_limit;
                if (y < low_delta_limit) y = low_delta_limit;
                
                shoe.adb.data[1] = x & 0x7f;
                shoe.adb.data[0] = y & 0x7f;
                if (!shoe.mouse.button_down) {
                    //shoe.adb.data[1] |= 0x80;
                    shoe.adb.data[0] |= 0x80;
                }
                // slog("mouse_talk: ")
                
                
                shoe.adb.data_len = 2;
                
                shoe.mouse.delta_x = 0;
                shoe.mouse.delta_y = 0;
                
                shoe.mouse.changed = 0;
            }
            else
                shoe.adb.timeout = 1;

            return ;
            
        case 1:
            assert(!"Can't handle reg 1");
            
        case 2:
            assert(!"Can't handle reg 2");
            
        case 3:
            shoe.adb.data[0] = 3; // device address: 3
            shoe.adb.data[1] = 1; // handler ID: 1
            shoe.adb.data_len = 2;
            return ;
            
    }
}

static void adb_handle_state_zero(uint8_t command_byte, uint8_t is_poll) // "Command" state
{
    via_state_t *via = &shoe.via[0];
    const uint8_t id = command_byte >> 4; // the target device ID
    const uint8_t reg = command_byte & 3;
    
    // Figure out the command type (reset/flush/talk/listen)
    
    if ((command_byte & 0xf) == 0) // reset
        shoe.adb.command_type = adb_reset;
    else if ((command_byte & 0xf) == 1) // flush
        shoe.adb.command_type = adb_flush;
    else if (~bmatch(command_byte, xxxx 11 xx)) // talk
        shoe.adb.command_type = adb_talk;
    else if (~bmatch(command_byte, xxxx 10 xx)) // listen
        shoe.adb.command_type = adb_listen;
    else
        assert(!"What is this adb state-0 command? xxxx 01xx");
    
    slog("adb_handle_state_zero: command_byte=0x%02x, id=%u, reg=%u\n", command_byte, id, reg);
    
    shoe.adb.command_device_id = id;
    shoe.adb.command_reg = reg;
    
    // Flush/reset/listen and talk-with-timeout need data_i initialized to 0
    shoe.adb.data_i = 0;
    shoe.adb.data_len = 0;
    
    // If talk, go ask they keyboard/mouse if they have anything to say
    if (shoe.adb.command_type == adb_talk) {
        
        if (id == 2) {
            keyboard_talk(reg);
        }
        else if (id == 3) {
            mouse_talk(reg);
        }
        else { // timeout
            shoe.adb.timeout = 1;
        }
        
        // If there was a service request pending for this device, it is now handled.
        shoe.adb.pending_service_requests &= ~~(1 << id);
    }
    
    shoe.adb.poll = 0;
    
    via->regb_input |= VIA_REGB_ADB_STATUS;
    via_raise_interrupt(1, IFR_SHIFT_REG);
}

static void adb_handle_state_one (void) // "Even" state
{
    via_state_t *via = &shoe.via[0];
    
    slog("adb_handle_state_one: ");
    if (shoe.adb.poll) {
        // Upon receiving a service request, the adb controller sends a TALK/reg=0 to the last accessed device
        adb_handle_state_zero((shoe.adb.command_device_id << 4) | 0x0c, 1);
    }
    
    switch (shoe.adb.command_type) {
        case adb_flush:
        case adb_reset:
            assert(!"adb_handle_state_one: unexpected command type");
            break;
            
        case adb_talk:
            slog("adb_talk: ");
            if (shoe.adb.timeout) {
                shoe.adb.timeout = 0;
                via->regb_input &= ~~VIA_REGB_ADB_STATUS; // adb_status_line cleared == timeout
                via_raise_interrupt(1, IFR_SHIFT_REG);
                slog("timeout\n");
                return ;
            }
            
            if (shoe.adb.data_i < shoe.adb.data_len)
                via->sr = shoe.adb.data[shoe.adb.data_i++];
            else
                via->sr = 0;
            
            slog("set sr = 0x%02x\n", via->sr);
            
            break;
        
        case adb_listen:
            slog("adb_listen: ");
            if (shoe.adb.timeout) {
                shoe.adb.timeout = 0;
                via->regb_input &= ~~VIA_REGB_ADB_STATUS; // adb_status_line cleared == timeout
                via_raise_interrupt(1, IFR_SHIFT_REG);
                slog("timeout\n");
                return ;
            }
            
            if (shoe.adb.data_i < 8)
                shoe.adb.data[shoe.adb.data_i++] = via->sr;
            else
                assert(!"OS made us listen to > 8 bytes");
            
            slog("loaded sr = 0x%02x\n", via->sr);
            
            break;
    }
    
    via->regb_input |= VIA_REGB_ADB_STATUS; // adb_status_line set == didn't-timeout
    via_raise_interrupt(1, IFR_SHIFT_REG);
}

static void adb_handle_state_two (void) // "Odd" state
{
    via_state_t *via = &shoe.via[0];
    
    slog("adb_handle_state_two: ");
    
    // If this transaction was part of a service request, clear the service_request flag now
    if (shoe.adb.service_request) {
        shoe.adb.service_request = 0;
        via->regb_input &= ~~VIA_REGB_ADB_STATUS; // adb_status_line cleared == service request
        slog("(service request) ");
    }
    else
        via->regb_input |= VIA_REGB_ADB_STATUS; // adb_status_line set == no-service request
    
    switch (shoe.adb.command_type) {
        case adb_flush:
        case adb_reset:
            slog("adb_flush/reset\n");
            break;
            
        case adb_talk:
            slog("adb_talk: ");
            if (shoe.adb.data_i < shoe.adb.data_len)
                via->sr = shoe.adb.data[shoe.adb.data_i++];
            else
                via->sr = 0;
            slog("set sr = 0x%02x\n", via->sr);
            break;
            
        case adb_listen:
            slog("adb_listen: ");
            if (shoe.adb.data_i < 8)
                shoe.adb.data[shoe.adb.data_i++] = via->sr;
            else
                assert(!"OS made us listen to > 8 bytes");
            slog("read sr = 0x%02x\n", via->sr);
            break;
    }
    
    via_raise_interrupt(1, IFR_SHIFT_REG);
}

static void adb_handle_state_three (void) // "idle" state
{
    slog("adb_handle_state_three: completed for id %u\n", shoe.adb.command_device_id);
    
    switch (shoe.adb.command_type) {
        case adb_reset:
        case adb_flush:
        case adb_talk:
            break;
            
        case adb_listen:
            slog("adb_handle_state_three: listen completed for id %u, reg %u, data_len = %u {%02x %02x}\n",
                   shoe.adb.command_device_id, shoe.adb.command_reg, shoe.adb.data_i, shoe.adb.data[0], shoe.adb.data[1]);
            break;
    }
    
    adb_start_service_request();
}

void adb_handle_state_change(uint8_t old_state, uint8_t new_state)
{
    via_state_t *via = &shoe.via[0];
    
    slog("%s: lock\n", __func__); fflush(stdout);
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    shoe.adb.state = new_state;
    
    switch (new_state) {
        case 0: 
            shoe.adb.command_byte = via->sr;
            adb_handle_state_zero(shoe.adb.command_byte, 0);
            break ;
        
        case 1:
            adb_handle_state_one();
            break ;
            
        case 2:
            adb_handle_state_two();
            break ;
        
        case 3:
            adb_handle_state_three();
            break ;
    }
    
    slog("%s: unlock\n", __func__); fflush(stdout);
    pthread_mutex_unlock(&shoe.adb.lock);
}


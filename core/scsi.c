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
#include <assert.h>
#include <ctype.h>
#include "shoebill.h"

// Target command register bits
#define TARGET_COMM_LAST_BYTE_SENT (1<<7)
#define TARGET_COMM_ASSERT_REQ (1<<3)
#define TARGET_COMM_ASSERT_MSG (1<<2)
#define TARGET_COMM_ASSERT_CD (1<<1)
#define TARGET_COMM_ASSERT_IO (1<<0)

// Initiator command register bits
#define INIT_COMM_ASSERT_RST (1<<7)
#define INIT_COMM_TEST_MODE (1<<6) // write-only
#define INIT_COMM_ARBITRATION_IN_PROGRESS (1<<6) // read-only
#define INIT_COMM_LOST_ARBITRATION (1<<5) // read-only
#define INIT_COMM_ASSERT_ACK (1<<4)
#define INIT_COMM_ASSERT_BSY (1<<3)
#define INIT_COMM_ASSERT_SEL (1<<2)
#define INIT_COMM_ASSERT_ATN (1<<1)
#define INIT_COMM_ASSERT_DATA_BUS (1<<0)

// Current scsi control register bits
#define CURR_SCSI_CONTROL_RST (1<<7)
#define CURR_SCSI_CONTROL_BSY (1<<6)
#define CURR_SCSI_CONTROL_REQ (1<<5)
#define CURR_SCSI_CONTROL_MSG (1<<4)
#define CURR_SCSI_CONTROL_CD (1<<3)
#define CURR_SCSI_CONTROL_IO (1<<2)
#define CURR_SCSI_CONTROL_SEL (1<<1)
#define CURR_SCSI_CONTROL_PARITY (1<<0)

// Bus and status register bits
#define BUS_STATUS_ACK (1<<0)
#define BUS_STATUS_ATN (1<<1)
#define BUS_STATUS_BUSY_ERROR (1<<2)
#define BUS_STATUS_PHASE_MATCH (1<<3)
#define BUS_STATUS_INTERRUPT_REQUEST_ACTIVE (1<<4)
#define BUS_STATUS_PARITY_ERROR (1<<5)
#define BUS_STATUS_DMA_REQUEST (1<<6)
#define BUS_STATUS_END_OF_DMA (1<<7)

// Mode register bits
#define MODE_BLOCK_MODE_DMA (1<<7)
#define MODE_TARGET_MODE (1<<6)
#define MODE_ENABLE_PARITY_CHECKING (1<<5)
#define MODE_ENABLE_PARITY_INTERRUPT (1<<4)
#define MODE_ENABLE_EOP_INTERRUPT (1<<3)
#define MODE_MONITOR_BUSY (1<<2)
#define MODE_DMA_MODE (1<<1)
#define MODE_ARBITRATE (1<<0)

// Invalid scsi device ID
#define INVALID_ID 8

const char *scsi_read_reg_str[8] = {
    "current_scsi_data_bus",
    "initiator_command",
    "mode",
    "target_command",
    "current_scsi_control",
    "bus_and_status",
    "input_data",
    "reset_interrupt"
};

const char *scsi_write_reg_str[8] = {
    "output_data",
    "initiator_command",
    "mode",
    "target_command",
    "id_select",
    "start_dma_send",
    "start_dma_target_receive",
    "start_dma_initiator_receive"
};

enum scsi_bus_phase {
    BUS_FREE = 0,
    ARBITRATION,
    SELECTION,
    RESELECTION,
    COMMAND,
    DATA_OUT,
    DATA_IN,
    STATUS,
    MESSAGE_IN,
    MESSAGE_OUT
};

typedef struct {
    // Phase
    enum scsi_bus_phase phase;
    
    // Scsi bus signals
    
    uint8_t init_bsy:1; // BSY, driven by initiator
    uint8_t target_bsy:1; // BSY, driven by target
    
    uint8_t sel:1; // SEL, driven by both target and initiator
    
    uint8_t rst:1; // RST, driven by both target and initiator
    
    uint8_t cd:1;  // C/D (control or data), driven by target
    uint8_t io:1;  // I/O, driven by target
    uint8_t ack:1; // ACK, driven by initiator
    uint8_t msg:1; // MSG, driven by target
    uint8_t atn:1; // ATN, driven by initiator
    uint8_t req:1; // REQ, driven by target
    
    uint8_t data; // DB0-7, data lines, driven by both target and initiator
    
    // NCR 5380 registers
    uint8_t initiator_command;
    uint8_t mode;
    uint8_t target_command;
    uint8_t select_enable; // probably not implementing this...
    
    // Arbitration state
    uint8_t init_id; // initiator ID (as a bit mask) (usually 0x80)
    
    // Selection state
    uint8_t target_id; // target ID (as an int [0, 7])
    
    // transfer buffers
    uint8_t buf[512 * 256];
    uint32_t bufi;
    uint32_t in_len, in_i;
    uint32_t out_len, out_i;
    uint32_t write_offset;
    uint8_t status_byte;
    uint8_t message_byte; // only one-byte messages supported for now
    
    // hack
    uint8_t dma_send_written; // Gets set whenever register 5 (start_dma_send) is written to, and cleared randomly.
                              // This is because aux 1.1.1 sends an extra byte after sending the write command, and that's not
                              // part of the write data. start_dma_send will be written when the data is actually starting.
    uint8_t sent_status_byte_via_reg0; // Gets set when the status byte is red via register 0.
        // This lets us know it's safe to switch to the MESSAGE_IN phase
    
} scsi_bus_state_t;

scsi_bus_state_t scsi;

static void switch_status_phase (uint8_t status_byte)
{
    
    printf("scsi_reg_something: switching to STATUS phase\n");
    
    scsi.phase = STATUS;
    scsi.status_byte = status_byte;
    scsi.msg = 0;
    scsi.cd = 1;
    scsi.io = 1;
    
    scsi.bufi = 0;
    
    // Phase mismatch (I think)
    via_raise_interrupt(2, 0);
}

static void switch_command_phase (void)
{
    printf("scsi_reg_something: switching to COMMAND phase\n");
    scsi.phase = COMMAND;
    
    scsi.msg = 0;
    scsi.cd = 1;
    scsi.io = 0;
    
    scsi.bufi = 0;
    
    // Phase mismatch, probably
    via_raise_interrupt(2, 0);
}

static void switch_message_in_phase (uint8_t message_byte)
{
    printf("scsi_reg_something: switching to MESSAGE_IN phase\n");
    
    scsi.phase = MESSAGE_IN;
    scsi.msg = 1;
    scsi.cd = 1;
    scsi.io = 1;
    
    scsi.message_byte = message_byte; // only one-byte messages supported for now
    
    // Phase mismatch, probably
    via_raise_interrupt(2, 0);
}

static void switch_bus_free_phase (void)
{
    printf("scsi_reg_something: switching to BUS_FREE phase\n");
    
    scsi.phase = BUS_FREE;
    
    scsi.msg = 0;
    scsi.cd = 0;
    scsi.io = 0;
    
    scsi.target_bsy = 0;
    scsi.req = 0;
    
    scsi.bufi = 0;
    // Phase mismatch not possible here.
}

static void switch_data_in_phase (void)
{
    printf("scsi_reg_something: switching to DATA_IN phase\n");
    
    scsi.phase = DATA_IN;
    
    scsi.msg = 0;
    scsi.cd = 0;
    scsi.io = 1;
    
    // Phase mismatch, probably
    via_raise_interrupt(2, 0);
}

static void switch_data_out_phase (void)
{
    printf("scsi_reg_something: switching to DATA_OUT phase\n");
    
    scsi.phase = DATA_OUT;

    scsi.msg = 0;
    scsi.cd = 0;
    scsi.io = 0;

    via_raise_interrupt(2, 0);
}

struct inquiry_response_t {
    uint8_t periph_device_type:5;
    uint8_t periph_qualifier:3;
    
    uint8_t device_type_modifier:7;
    uint8_t rmb:1;
    
    uint8_t ansi_vers:3;
    uint8_t ecma_vers:3;
    uint8_t iso_vers:2;
    
    uint8_t resp_format:4;
    uint8_t unused_1:2;
    uint8_t trmiop:1;
    uint8_t anec:1;
    
    uint8_t additional_length;
    
    uint8_t unused_2;
    uint8_t unused_3;
    
    uint8_t sftre:1;
    uint8_t cmdque:1;
    uint8_t unused_4:1;
    uint8_t linked:1;
    uint8_t sync:1;
    uint8_t wbus16:1;
    uint8_t wbus32:1;
    uint8_t reladr:1;
    
    uint8_t vendor_id[8];
    uint8_t product_id[16];
    uint8_t product_rev[4];
};

static void scsi_handle_inquiry_command(const uint8_t alloc_len)
{
    struct inquiry_response_t resp;
    
    assert(alloc_len >= 6);
    
    memset(&resp, 0, sizeof(struct inquiry_response_t));
    
    resp.periph_qualifier = 0; // I'm connected to the specified LUN (sure, whatever)
    resp.periph_device_type = 0; // 0 -> direct access device (hard disk)
    
    resp.rmb = 0; // not removable
    resp.device_type_modifier = 0; // Vendor-specific
    
    resp.ansi_vers = 1; // complies with ANSI X3.131-1986 (SCSI-1)
    resp.ecma_vers = 1; // whatever
    resp.iso_vers = 1; // whatever
    
    resp.anec = 0; // only applies to "processor" devices
    resp.trmiop = 0; // we don't support TERMINATE I/O PROCESS, whatever that is
    resp.resp_format = 0; // SCSI-1 INQUIRY response format
    
    resp.additional_length = 0; // no additional parameters
    
    resp.reladr = 0; // don't support relative addressing
    resp.wbus16 = 0; // these must be SCSI-2 things. (I'm looking at the ANSI SCSI-2 docs)
    resp.wbus32 = 0;
    resp.sync = 0; // don't support synchronous transfer
    resp.cmdque = 0; // don't support tagged command queuing
    resp.linked = 0; // don't support linked commands
    resp.sftre = 0; // don't support soft reset
    
    memcpy(resp.vendor_id, "Shoebill", 8);
    strcpy((char*)resp.product_id, "Phony SCSI disk");
    memcpy(resp.product_rev, "fded", 4);
    
    // XXX: added this because A/UX 3.0.1 requsts 6 bytes of the the inquiry response (sometimes?) I think it's polling for all attached scsi devices.
    // Fixme: figure out how to respond "not attached"
    if (alloc_len > sizeof(resp))
        scsi.in_len = sizeof(resp);
    else
        scsi.in_len = alloc_len;
    memcpy(scsi.buf, &resp, scsi.in_len);
    scsi.in_i = 0;
    
    switch_data_in_phase();
}

static void scsi_buf_set (uint8_t byte)
{
    assert(scsi.bufi <= sizeof(scsi.buf));
    scsi.buf[scsi.bufi++] = byte;
    
    if (scsi.phase == COMMAND) {
        const uint32_t cmd_len = (scsi.buf[0] >= 0x20) ? 10 : 6; // 10 or 6 byte command?
        
        assert(scsi.target_id < 8);
        scsi_device_t *dev = &shoe.scsi_devices[scsi.target_id];
        
        // If we need more data for this command, keep driving REQ
        if (scsi.bufi < cmd_len) {
            // scsi.req = 1;
            // FIXME: keep driving DMA_REQUEST too
            return ;
        }
        
        switch (scsi.buf[0]) {
            case 0: // test unit ready (6)
                printf("scsi_buf_set: responding to test-unit-ready\n");
                switch_status_phase(0); // switch to the status phase, with a status byte of 0
                break;
                
            case 0x15: // mode select (6)
                printf("scsi_buf_set: responding to mode-select\n");
                switch_status_phase(0);
                break;
                
            case 0x25: // read capacity (10)
                printf("scsi_buf_set: responding to read-capacity\n");
                // bytes [0,3] -> BE number of blocks
                scsi.buf[0] = (dev->num_blocks >> 24) & 0xff;
                scsi.buf[1] = (dev->num_blocks >> 16) & 0xff;
                scsi.buf[2] = (dev->num_blocks >> 8) & 0xff;
                scsi.buf[3] = (dev->num_blocks) & 0xff;
                
                // bytes [4,7] -> BE block size (needs to be 512)
                
                scsi.buf[4] = (dev->block_size >> 24) & 0xff;
                scsi.buf[5] = (dev->block_size >> 16) & 0xff;
                scsi.buf[6] = (dev->block_size >> 8) & 0xff;
                scsi.buf[7] = (dev->block_size) & 0xff;
                
                scsi.in_i = 0;
                scsi.in_len = 8;
                
                switch_data_in_phase();
                break;
                
            case 0x12: { // inquiry command (6)
                printf("scsi_buf_set: responding to inquiry\n");
                const uint8_t alloc_len = scsi.buf[4];
                
                scsi_handle_inquiry_command(alloc_len);
                break;
            }
                
            case 0x8: { // read (6)
                const uint32_t offset =
                    (scsi.buf[1] << 16) |
                    (scsi.buf[2] << 8 ) |
                    (scsi.buf[3]);
                const uint8_t len = scsi.buf[4];
                
                assert(dev->f);
                
                printf("scsi_buf_set: Responding to read at off=%u len=%u\n", offset, len);
                
                //assert(len <= 64);
                
                assert(dev->num_blocks > offset);
                
                if (len == 0) {
                    switch_status_phase(0);
                    break;
                }
                
                assert(0 == fseeko(dev->f, 512 * offset, SEEK_SET));
                assert(fread(scsi.buf, len * 512, 1, dev->f) == 1);
                
                
                scsi.in_len = len * 512;
                scsi.in_i = 0;
                
                switch_data_in_phase();
                break;
            }
            
            case 0xa: { // write (6)
                const uint32_t offset =
                    (scsi.buf[1] << 16) |
                    (scsi.buf[2] << 8 ) |
                    (scsi.buf[3]);
                const uint8_t len = scsi.buf[4];
                
                printf("scsi_buf_set: Responding to write at off=%u len=%u\n", offset, len);
                
                //assert(len <= 64);
                
                scsi.write_offset = offset;
                scsi.out_len = len * 512;
                scsi.out_i = 0;
                
                scsi.dma_send_written = 0; // reset here. The real data will come in after start_dma_send is written to.
                switch_data_out_phase();
                break;
            }
                
            default:
                assert(!"unknown commmand!");
                break;
        }
        
        scsi.bufi = 0;
    }
}

void init_scsi_bus_state ()
{
    memset(&scsi, 0, sizeof(scsi_bus_state_t));
    
    scsi.phase = BUS_FREE;
}

void scsi_reg_read ()
{
    const uint32_t reg = ((shoe.physical_addr & 0xffff) >> 4) & 0xf;

    //printf("\nscsi_reg_read: reading from register %s(%u) ", scsi_read_reg_str[reg], reg);
    
    switch (reg) {
        case 0: // Current scsi data bus register
            if (scsi.phase == ARBITRATION)
                shoe.physical_dat = 0; // I don't know why A/UX expects 0 here. It should be the initiator's ID, I think
            else if (scsi.phase == MESSAGE_IN) {
                shoe.physical_dat = scsi.message_byte; // one-byte messages supported for now
            }
            else if (scsi.phase == STATUS) {
                shoe.physical_dat = scsi.status_byte;
                scsi.sent_status_byte_via_reg0 = 1;
            }
            else
                assert(!"scsi_reg_read: reading data reg (0) from unknown phase\n");
            
            break;
            
        case 1: // Initiator command register
            
            if (scsi.phase == ARBITRATION &&
                (scsi.initiator_command & INIT_COMM_ARBITRATION_IN_PROGRESS)) {
                
                shoe.physical_dat = scsi.initiator_command;
                
                // the INIT_COMM_ARBITRATION_IN_PROGRESS bit is transient. Just clear
                // it after the first access (it needs to go hi, then later low)
                scsi.initiator_command &= ~INIT_COMM_ARBITRATION_IN_PROGRESS;
                
            }
            else
                shoe.physical_dat = scsi.initiator_command;
            
            break;
            
        case 2: // Mode register
            shoe.physical_dat = scsi.mode;
            break;
            
        case 3: // Target command register
            shoe.physical_dat = scsi.target_command & 0xf; // only the low 4 bits are significant
            break;
            
        case 4: { // Current SCSI control register
            uint8_t tmp = 0;
            tmp |= (scsi.sel * CURR_SCSI_CONTROL_SEL);
            tmp |= (scsi.io  * CURR_SCSI_CONTROL_IO);
            tmp |= (scsi.cd  * CURR_SCSI_CONTROL_CD);
            tmp |= (scsi.msg * CURR_SCSI_CONTROL_MSG);
            tmp |= (scsi.req * CURR_SCSI_CONTROL_REQ);
            tmp |= ((scsi.target_bsy || scsi.init_bsy) ? CURR_SCSI_CONTROL_BSY : 0);
            tmp |= (scsi.rst * CURR_SCSI_CONTROL_RST);
            shoe.physical_dat = tmp;
            break;
        }
            
        case 5: { // Bus and status register
            uint8_t tmp = 0;
            
            // Compute phase match (IO, CD, MSG match the assertions in target_command register)
            uint8_t phase_tmp = 0;
            {
                phase_tmp = (phase_tmp << 1) | scsi.msg;
                phase_tmp = (phase_tmp << 1) | scsi.cd;
                phase_tmp = (phase_tmp << 1) | scsi.io;
                phase_tmp = (phase_tmp == (scsi.target_command & 7));
            }
            
            tmp |= (scsi.ack * BUS_STATUS_ACK);
            tmp |= (scsi.atn * BUS_STATUS_ATN);
            tmp |= (phase_tmp * BUS_STATUS_PHASE_MATCH);
            
            // let's just say BUS_ERROR is always false (fixme: wrong)
            // let's just say INTERRUPT_REQUEST_ACTIVE is always false (fixme: wrong)
            // let's just say PARITY_ERROR is always false 
            tmp |= BUS_STATUS_DMA_REQUEST; // let's just say DMA_REQUEST is always true (fixme: wrong)
            shoe.physical_dat = tmp;
            break;
        }
            
        case 6: // Input data register
            shoe.physical_dat = 0;
            break;
            
        case 7: // Reset error / Interrupt register
            shoe.physical_dat = 0;
            break;
    }
    
    //printf("(set to 0x%02x)\n\n", (uint32_t)shoe.physical_dat);
}

void scsi_reg_write ()
{
    const uint32_t reg = ((shoe.physical_addr & 0xffff) >> 4) & 0xf;
    const uint8_t dat = shoe.physical_dat & 0xff;
    
    switch (reg) {
        case 0: // Output data register
            scsi.data = dat;
            break;
            
        case 1: { // Initiator command register
            scsi.initiator_command = dat;
            
            scsi.ack = ((scsi.initiator_command & INIT_COMM_ASSERT_ACK) != 0);
            scsi.rst = ((scsi.initiator_command & INIT_COMM_ASSERT_RST) != 0);
            scsi.init_bsy = ((scsi.initiator_command & INIT_COMM_ASSERT_BSY) != 0);
            scsi.sel = ((scsi.initiator_command & INIT_COMM_ASSERT_SEL) != 0);
            scsi.atn = ((scsi.initiator_command & INIT_COMM_ASSERT_ATN) != 0);
 
/*
// --- Arbitration ---
            // Check whether to switch from ARBITRATION to SELECTION
            if (scsi.sel && scsi.phase == ARBITRATION) {
                // Asserting SEL in arbitration phase means we switch to selection phase :)
                scsi.phase = SELECTION;
                scsi.target_id = INVALID_ID; // invalid ID
                printf("scsi_reg_write: selection phase\n");
                break;
            }
*/
// --- Selection ---
            // If we're in SELECTION, receive the target_id from scsi.data
            if (scsi.sel && (scsi.initiator_command & INIT_COMM_ASSERT_DATA_BUS) &&
                ((scsi.phase == ARBITRATION) || (scsi.phase == BUS_FREE)))
            {
                uint8_t id;
                for (id=0; (id < 8) && !(scsi.data & (1 << id)); id++) ;
                assert(id != 8);
                scsi.target_id = id;
                printf("scsi_reg_write: selected target id %u\n", id);
                scsi.target_bsy = 1; // target asserts BSY to acknowledge being selected
                scsi.phase = SELECTION;
                break;
            }
            
            // SELECTION ends when SEL gets unset
            if (!scsi.sel && scsi.phase == SELECTION) {
                printf("scsi_reg_write: switch to COMMAND phase\n"); // what's next?
                
                scsi.req = 1; // target asserts REQ after initiator deasserts SEL
                
                // Switch to COMMAND phase
                scsi.cd = 1;
                scsi.io = 0;
                scsi.msg = 0;
                scsi.phase = COMMAND;
                break;
            }
            
// --- Information transfer ---
            // If initiator asserts ACK, then target needs to deassert REQ
            // (I think this only makes sense for non-arbitration/selection/busfree situations
            if ((scsi.phase != BUS_FREE) && (scsi.phase != ARBITRATION) && (scsi.phase != SELECTION)) {
                
                // If this is the message_in phase, use the unsetting-ACK portion of the REQ/ACK handshake
                // to go to BUS_FREE.
                // Don't bother asserting REQ here. Also, switch_bus_free_phase() will deassert target_BSY.
                if (scsi.phase == MESSAGE_IN && !scsi.ack && !scsi.req) {
                    switch_bus_free_phase();
                    break ;
                }
                // If the status byte was read through register 0, then we need to manually switch to
                // message_in phase when the initiator sets ACK
                // Do this when the OS deasserts ACK. We know that ACK was previously asserted if !REQ.
                // (This is kinda hacky - maybe I can detect if ACK is deasserted by looking at the
                // previous value of reg1)
                else if (scsi.phase == STATUS && !scsi.ack && !scsi.req && scsi.sent_status_byte_via_reg0) {
                    scsi.req = 1;
                    switch_message_in_phase(0);
                }
                else {
                    scsi.req = !scsi.ack;
                }
            }
            
            break;
        }
            
        case 2: { // Mode register
            scsi.mode = dat;
            
            if (scsi.mode & MODE_ARBITRATE) {
                printf("scsi_reg_write: arbitration phase\n");
                scsi.phase = ARBITRATION;
                scsi.initiator_command |= INIT_COMM_ARBITRATION_IN_PROGRESS;
            }
            else {
                
            }
            
            break;
        }
        case 3: // Target command register
            scsi.target_command = dat & 0xf; // only the bottom 4 bits are writable
            break;
            
        case 4: // ID select register
            scsi.select_enable = dat;
            break;
            
        case 5: // Start DMA send
            scsi.dma_send_written = 1;
            via_raise_interrupt(2, 0);
            break;
            
        case 6: // Start DMA target receive
            break;
            
        case 7: // Start DMA initiator receive
            via_raise_interrupt(2, 0);
            break;
    }
    
    printf("\nscsi_reg_write: writing to register %s(%u) (0x%x)\n\n", scsi_write_reg_str[reg], reg, dat);
}

void scsi_dma_write_long(const uint32_t dat)
{
    scsi_dma_write((dat >> 24) & 0xff);
    scsi_dma_write((dat >> 16) & 0xff);
    scsi_dma_write((dat >> 8 ) & 0xff);
    scsi_dma_write(dat & 0xff);
}

void scsi_dma_write (const uint8_t byte)
{
    if (scsi.phase == COMMAND) {
        printf("scsi_reg_dma_write: writing COMMAND byte 0x%02x\n", byte);
        scsi_buf_set(byte);
    }
    else if (scsi.phase == DATA_OUT && scsi.dma_send_written) {
        scsi.buf[scsi.out_i++] = byte;
        
        //printf("scsi_reg_dma_write: writing DATA_OUT byte 0x%02x (%c)\n", byte, isprint(byte)?byte:'.');
        
        if (scsi.out_i >= scsi.out_len) {
            assert(scsi.target_id < 8);
            scsi_device_t *dev = &shoe.scsi_devices[scsi.target_id];
            assert(dev->f);
            
            assert(0 == fseeko(dev->f, 512 * scsi.write_offset, SEEK_SET));
            assert(fwrite(scsi.buf, scsi.out_len, 1, dev->f) == 1);
            fflush(dev->f);
            
            scsi.out_i = 0;
            scsi.out_len = 0;
            switch_status_phase(0);
        }
    }
    else if (scsi.phase == DATA_OUT) {
        printf("scsi_reg_dma_write: writing DATA_OUT byte (without scsi.dma_send_written) 0x%02x\n", byte);
    }
    else {
        printf("scsi_reg_dma_write: writing 0x%02x in UNKNOWN PHASE!\n", byte);
    }
    
}

uint32_t scsi_dma_read_long()
{
    uint32_t i, result = 0;
    
    for (i=0; i<4; i++) {
        result = (result << 8) + scsi_dma_read();
    }
    
    return result;
}

uint8_t scsi_dma_read ()
{
    uint8_t result = 0;
    
    if (scsi.phase == STATUS) {
        // If in the STATUS phase, return the status byte and switch back to COMMAND phase
        result = scsi.status_byte;
        switch_message_in_phase(0); 
    }
    else if (scsi.phase == DATA_IN) {
        assert(scsi.in_len > 0);
        result = scsi.buf[scsi.in_i++];
        if (scsi.in_i >= scsi.in_len) {
            scsi.in_i = 0;
            scsi.in_len = 0;
            
            switch_status_phase(0);
        }
    }
    
    //printf("scsi_reg_dma_read: called, returning 0x%02x\n", (uint8_t)result);
    
    return result;
}

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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "../core/shoebill.h"
#include "../core/coff.h"

#define DEBUG_MODE_STOPPED 0
#define DEBUG_MODE_RUNNING 1
#define DEBUG_MODE_STEP 2

#define SHOEBILL_DEBUG_PORT 0xfded
static int _start_debug_server(void)
{
    struct sockaddr_in addr;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    
    memset(&addr, 0, sizeof(addr));
    addr.sin_len = sizeof(struct sockaddr_in);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SHOEBILL_DEBUG_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    
    if (sock == -1) {
        assert(!"can't socket");
    }
    else if (bind(sock, (struct sockaddr *)&addr, sizeof(struct sockaddr)) != 0) {
        assert(!"can't bind");
        // return -1;
    }
    else if (listen(sock, 1) != 0) {
        assert(!"can't listen");
    }
    
    return sock;
}

void *debug_server_thread (void *arg)
{
    struct sockaddr_in addr;
    socklen_t sin_size = sizeof(struct sockaddr_in);
    uint8_t *inbuf = calloc(0x10000, 1);
    uint8_t *outbuf = calloc(0x10000, 1);
    int sock = _start_debug_server();
    int clientfd = accept(sock, (struct sockaddr*)&addr, &sin_size);
    
    shoe.dbg.connected = 1;
    shoe.dbg.mode = DEBUG_MODE_RUNNING;
    
    
    return NULL;
}

void *debug_cpu_thread (void *arg)
{
    memset(&shoe.dbg, 0, sizeof(shoe.dbg));
    shoe.dbg.mode = DEBUG_MODE_STOPPED;
    
    pthread_t server_thread_pid;
    pthread_create(&server_thread_pid,
                   NULL,
                   debug_server_thread,
                   NULL);
    
    /*
     * The CPU only runs once the debugger is connected, and the
     * emulator has started
     */
    pthread_mutex_lock(&shoe.cpu_thread_lock);
    while (!shoe.dbg.connected)
        usleep(1000);
    
    while (1) {
        if (shoe.dbg.mode == DEBUG_MODE_RUNNING) {
            if (!shoe.dbg.ignore_interrupts &&
                (shoe.cpu_thread_notifications & 0xff)) {
                process_pending_interrupt();
            }
            
            if (shoe.cpu_thread_notifications & SHOEBILL_STATE_STOPPED) {
                continue;
            }
            
            cpu_step();
        }
        else if (shoe.dbg.mode == DEBUG_MODE_STOPPED)
            pthread_yield_np();
        else if (shoe.dbg.mode == DEBUG_MODE_STEP) {
            cpu_step();
            shoe.dbg.mode = DEBUG_MODE_STOPPED;
        }
    }
}




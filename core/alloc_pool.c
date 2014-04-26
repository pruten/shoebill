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

#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include "../core/shoebill.h"

/*typedef struct _alloc_pool_t {
    struct _alloc_pool_t *prev, *next;
    uint32_t size, magic;
} alloc_pool_t;*/

void* p_alloc(alloc_pool_t *pool, uint64_t size)
{
    alloc_pool_t *buf = calloc(sizeof(alloc_pool_t) + size, 1);
    buf->size = size;
    buf->magic = 'moof';
    
    buf->next = pool->next;
    buf->prev = pool;
    
    if (pool->next)
        pool->next->prev = buf;
    pool->next = buf;
    
    return &buf[1];
}

void* p_realloc(void *ptr, uint64_t size)
{
    alloc_pool_t *header = &((alloc_pool_t*)ptr)[-1];
    alloc_pool_t *new_header = realloc(header, size + sizeof(alloc_pool_t));
    
    if (new_header)
        return &new_header[1];
    
    return NULL;
}

void p_free(void *ptr)
{
    alloc_pool_t *header = &((alloc_pool_t*)ptr)[-1];
    assert(header->magic == 'moof');
    
    if (header->next)
        header->next->prev = header->prev;
    
    if (header->prev)
        header->prev->next = header->next;
    
    free(header);
}

void p_free_pool(alloc_pool_t *pool)
{
    while (pool->prev)
        pool = pool->prev;
    
    while (pool) {
        alloc_pool_t *cur = pool;
        pool = cur->next;
        assert(cur->magic == 'moof');
        free(cur);
    }
}

alloc_pool_t* p_new_pool(void)
{
    alloc_pool_t *pool = calloc(sizeof(alloc_pool_t), 1);
    pool->magic = 'moof';
    return pool;
}
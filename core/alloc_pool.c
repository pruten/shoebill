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
#include <string.h>
#include "../core/shoebill.h"



/*
#define POOL_ALLOC_TYPE 0
#define POOL_CHILD_LINK 1
#define POOL_HEAD 2
typedef struct _alloc_pool_t {
    struct _alloc_pool_t *prev, *next;
    uint8_t type;
    union {
        struct {
            uint32_t size;
        } alloc;
        struct {
            struct _alloc_pool_t *child; // pointer to the child's HEAD
        } child_link;
        struct {
            struct _alloc_pool_t *parent_link; // pointer to the parent's CHILD_LINK
        } head;
    } t;
    
    uint32_t magic;
} alloc_pool_t;
*/

static void _check_pool(alloc_pool_t *pool)
{
    
}

static alloc_pool_t* _ptr_to_header(void *ptr)
{
    alloc_pool_t *apt = (alloc_pool_t*)ptr;
    return &apt[-1];
}

void* p_alloc(alloc_pool_t *pool, uint64_t size)
{
    alloc_pool_t *buf = calloc(sizeof(alloc_pool_t) + size, 1);
    
    buf->type = POOL_ALLOC_TYPE;
    buf->t.alloc.size = size;
    
    buf->start_magic = POOL_START_MAGIC;
    buf->end_magic = POOL_END_MAGIC;
    
    buf->next = pool->next;
    buf->prev = pool;
    
    if (pool->next)
        pool->next->prev = buf;
    pool->next = buf;
    
    return &buf[1];
}

void* p_realloc(void *ptr, uint64_t size)
{
    alloc_pool_t *header = _ptr_to_header(ptr);

    assert(header->start_magic == POOL_START_MAGIC);
    assert(header->end_magic == POOL_END_MAGIC);
    assert(header->type == POOL_ALLOC_TYPE);
    
    alloc_pool_t *new_header = realloc(header, size + sizeof(alloc_pool_t));
    
    if (new_header) {
        new_header->t.alloc.size = size;
        
        if (new_header->next)
            new_header->next->prev = new_header;
        
        if (new_header->prev)
            new_header->prev->next = new_header;
    
        return &new_header[1];
    }
    
    return NULL;
}

/* 
 * Free *any* kind of alloc_pool_t header
 */
static void _p_free_any(alloc_pool_t *header)
{
    assert(header->start_magic == POOL_START_MAGIC);
    assert(header->end_magic == POOL_END_MAGIC);
    
    if (header->next)
        header->next->prev = header->prev;
    
    if (header->prev)
        header->prev->next = header->next;
    
    free(header);
}

/*
 * Free an alloc_pool allocation (but not HEAD or CHILD_LINK)
 */
void p_free(void *ptr)
{
    alloc_pool_t *header = _ptr_to_header(ptr);
    assert(header->type == POOL_ALLOC_TYPE);
    memset(ptr, 0xaa, header->t.alloc.size);
    _p_free_any(header);
}

void p_free_pool(alloc_pool_t *pool)
{
    while (pool->prev)
        pool = pool->prev;
    
    while (pool) {
        alloc_pool_t *cur = pool;
        pool = cur->next;
        assert(cur->start_magic == POOL_START_MAGIC);
        assert(cur->end_magic == POOL_END_MAGIC);
        
        switch (cur->type) {
            case POOL_ALLOC_TYPE:
                _p_free_any(cur);
                break;
            case POOL_CHILD_LINK: {
                // p_free_pool will free and unlink cur
                // (its parent's CHILD_LINK)
                p_free_pool(cur->t.child_link.child);
                break;
            }
            case POOL_HEAD: {
                if (cur->t.head.parent_link) {
                    assert(cur->t.head.parent_link->type == POOL_CHILD_LINK);
                    _p_free_any(cur->t.head.parent_link);
                }
                _p_free_any(cur);
                break;
            }
            default:
                assert(!"unknown POOL_ type");
        }
    }
}

alloc_pool_t* p_new_pool(alloc_pool_t *parent_pool)
{
    alloc_pool_t *pool = calloc(sizeof(alloc_pool_t), 1);
    
    pool->start_magic = POOL_START_MAGIC;
    pool->end_magic = POOL_END_MAGIC;
    pool->type = POOL_HEAD;
    
    if (parent_pool) {
        alloc_pool_t *link = _ptr_to_header(p_alloc(parent_pool, 0));
        link->type = POOL_CHILD_LINK;
        link->t.child_link.child = pool; // child_link.child points to the child's HEAD
        
        pool->t.head.parent_link = link; // head.parent_link points to the parent's CHILD_LINK
    }
    else
        pool->t.head.parent_link = NULL;
    
    return pool;
}
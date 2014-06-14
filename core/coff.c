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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <assert.h>
#include "shoebill.h"

void symb_inorder(rb_node *cur) {
    const coff_symbol *sym = *(coff_symbol**)&cur[1];
    if (!sym) 
        return ;
    symb_inorder(cur->left);
    slog("0x%x %s\n", cur->key, sym->name);
    symb_inorder(cur->right);
}

#define _coff_buf_seek(__len) ({ \
    uint32_t _result = 1, _len=(__len); \
    if (_len > buflen) _result = 0; \
    else bufptr = _len; \
    _result; \
})

#define _coff_buf_read(_dst, __len) ({\
    uint32_t _result = 1, _len=(__len); \
    if ((bufptr + _len) > buflen) \
        _result = 0; \
    else {\
        memcpy((_dst), buf+bufptr, _len); \
        bufptr += _len; \
    } \
    _result; \
})

void coff_free(coff_file *coff)
{
    p_free_pool(coff->pool);
}

// Given a path to a COFF binary, create a coff_file structure and return a pointer.
coff_file* coff_parse(uint8_t *buf, uint32_t buflen, alloc_pool_t *parent_pool)
{
    uint8_t rawhead[20], *ptr;
    uint32_t i;
    coff_file *cf = NULL;
    uint32_t bufptr = 0;
    alloc_pool_t *pool = p_new_pool(parent_pool);
    
    // Pull out 20 bytes (the file header)
    if (!_coff_buf_read(rawhead, 20)) {
        slog("coff_parse: error: this binary is missing its file header\n");
        goto fail;
    }
    
    // Allocate a coff_file and copy in the header
    cf = (coff_file*)p_alloc(pool, sizeof(coff_file));
    cf->pool = pool;
    ptr = rawhead;
    
    cf->magic = be2native(&ptr, 2);
    cf->num_sections = be2native(&ptr, 2);
    cf->timestamp = be2native(&ptr, 4);
    cf->symtab_offset = be2native(&ptr, 4);
    cf->num_symbols = be2native(&ptr, 4);
    cf->opt_header_len = be2native(&ptr, 2);
    cf->flags = be2native(&ptr, 2);
    
    // A little sanity checking...
    if (cf->magic != 0x150) {
        slog("coff_parse: I don't recognize this magic number: 0x%04x\n", cf->magic);
        goto fail;
    }
    else if (cf->num_sections != 3) {
        //slog("coff_parse: warning: there are %u sections in this file (not 3, like I expect)\n", cf->num_sections);
        // FIXME: investigate all the other possible section types
    }
    
    // pull out cf->opt_header bytes (a.out-format header, I guess?)
    if (cf->opt_header_len > 0) {
        uint8_t *opt = p_alloc(cf->pool, cf->opt_header_len);
        if (!_coff_buf_read(opt, cf->opt_header_len)) {
            slog("coff_parse: I ran out of data pulling the optional header (%u bytes)\n", cf->opt_header_len);
            p_free(opt);
            goto fail;
        }
        cf->opt_header = opt;
    }
    
    // start pulling out sections
    cf->sections = p_alloc(cf->pool, cf->num_sections * sizeof(coff_section));
    for (i=0; i<cf->num_sections; i++) {
        // read the header
        uint8_t rawsec[40];
        if (!_coff_buf_read(rawsec, 40)) {
            slog("coff_parse: I ran out of data pulling section #%u\n", i+1);
            goto fail;
        }
        // and copy it into cf->sections[i]
        memcpy(cf->sections[i].name, rawsec, 8);
        ptr = &rawsec[8];
        cf->sections[i].p_addr = be2native(&ptr, 4);
        cf->sections[i].v_addr = be2native(&ptr, 4);
        cf->sections[i].sz = be2native(&ptr, 4);
        cf->sections[i].data_ptr = be2native(&ptr, 4);
        cf->sections[i].reloc_ptr = be2native(&ptr, 4);
        cf->sections[i].line_ptr = be2native(&ptr, 4);
        cf->sections[i].num_relocs = be2native(&ptr, 2);
        cf->sections[i].num_lines = be2native(&ptr, 2);
        cf->sections[i].flags = be2native(&ptr, 4);
        
        // a little bit of sanity checking:
        if (cf->sections[i].v_addr != cf->sections[i].p_addr) {
            //slog("coff_parse: warning: section %u's virtual_addr and physical_addr don't match: p=%x v=%x\n",
                // i+1, cf->sections[i].p_addr, cf->sections[i].v_addr);
            // This is okay for the unix kernel
        }
    }
    
    // pull out the corresponding raw data for each section
    for (i=0; i<cf->num_sections; i++) {
        uint8_t *data;
        
        // don't bother if there's no data
        if (cf->sections[i].sz == 0) {
            continue ;
        }
        
        // don't bother if it's .bss
        if (memcmp(cf->sections[i].name, ".bss\0\0\0\0", 8)==0) {
            continue ;
        }
        
        // seek to the position in the binary that holds this section's raw data
        if (!_coff_buf_seek(cf->sections[i].data_ptr)) {
            slog("coff_parse: I couldn't seek to 0x%x in section %u\n", cf->sections[i].data_ptr, i+1);
            goto fail;
        }
        
        // load the data and attach it to the section struct
        data = p_alloc(cf->pool, cf->sections[i].sz); // FIXME: sz might not be a sane value
        if (!_coff_buf_read(data, cf->sections[i].sz)) {
            slog("coff_parse: I couldn't fread section %u (%s)'s data (%u bytes)\n", i+1, cf->sections[i].name, cf->sections[i].sz);
            p_free(data);
            goto fail;
        }
        cf->sections[i].data = data;
    }
    
    // pull out the symbol table
    
    if (cf->num_symbols == 0) // if num_symbols==0, symtab_offset may be bogus
        return cf; // just return
    
    cf->func_tree = rb_new(cf->pool, sizeof(coff_symbol*));
    //slog("func_tree = %llx, *func_tree = %llx\n", cf->func_tree, *cf->func_tree);
    cf->symbols = (coff_symbol*)p_alloc(cf->pool, sizeof(coff_symbol) *cf->num_symbols);
    
    // Seek to the symbol table
    if (!_coff_buf_seek(cf->symtab_offset)) {
        slog("coff_parse: I couldn't seek to symtab_offset, 0x%x\n", cf->symtab_offset);
        goto fail;
    }
    
    for (i=0; i < cf->num_symbols; i++) {
        uint8_t raw_symb[18];
        if (!_coff_buf_read(raw_symb, 18)) {
            slog("coff_parse: I ran out of data pulling symbol #%u\n", i+1);
            goto fail;
        }
        
        // load the name
        if (*((uint32_t*)raw_symb) == 0) {
            uint8_t tmp_name[256];
            uint32_t j, offset, idx = cf->symtab_offset + 18*cf->num_symbols;
            for (j=4, offset=0; j<8; j++) offset = (offset<<8) | raw_symb[j];
            idx += offset;
            
            // slog("Loading from external: base idx=0x%x, offset=%u, addr=0x%x\n", idx-offset, offset, idx);
            
            if (!_coff_buf_seek(idx)) {
                slog("coff_parse: I ran out of data pulling symbol %u's name (idx=0x%x)\n", i+1, idx);
                goto fail;
            }
            for (j=0; (_coff_buf_read(&tmp_name[j], 1)) && tmp_name[j]; j++) {
                if (j >= 255) {
                    // slog("coff_parse: this symbol's name is too long: %u\n", i+1);
                    goto fail;
                }
            }
            cf->symbols[i].name = p_alloc(cf->pool, j+1);
            memcpy(cf->symbols[i].name, tmp_name, j);
            cf->symbols[i].name[j] = 0;
            _coff_buf_seek(cf->symtab_offset + (i+1)*18);
        }
        else {
            uint8_t tmp_name[9];
            memcpy(tmp_name, raw_symb, 8);
            tmp_name[8] = 0;
            cf->symbols[i].name = strdup((char*)tmp_name);
        }
        
        ptr = &raw_symb[8];
        cf->symbols[i].value = be2native(&ptr, 4);
        cf->symbols[i].scnum = be2native(&ptr, 2);
        cf->symbols[i].type = be2native(&ptr, 2);
        cf->symbols[i].sclass = raw_symb[16];
        cf->symbols[i].numaux = raw_symb[17];
 
        // FIXME: I need to handle numaux > 0.
        
        //if (cf->symbols[i].numaux > 0) {
            slog("%s\n", cf->symbols[i].name);
            slog("value=0x%08x scnum=0x%04x type=0x%04x sclass=0x%02x numaux=%u\n\n",
                cf->symbols[i].value, cf->symbols[i].scnum, cf->symbols[i].type, cf->symbols[i].sclass, cf->symbols[i].numaux);
        //}
        
    
        if (cf->symbols[i].sclass == 2 || cf->symbols[i].sclass == 3) {
            void *ptr = &cf->symbols[i];
            rb_insert(cf->func_tree, cf->symbols[i].value, &ptr, NULL);
            //slog("%s addr=0x%x\n", cf->symbols[i].name, cf->symbols[i].value);
        }
        // slog("%u: %s (class=%u)\n", i+1, cf->symbols[i].name, cf->symbols[i].sclass);
        
    }
    
    // symb_inorder(*cf->func_tree);
    
    // and we're done
    return cf;
    
fail:
    if (cf) {
        if (cf->opt_header) {
            p_free(cf->opt_header);
        }
        if (cf->sections) {
            for (i=0; i<cf->num_sections; i++) {
                if (cf->sections[i].data) {
                    p_free(cf->sections[i].data);
                }
            }
            p_free(cf->sections);
        }
        p_free(cf);
    }
    return NULL;
}

coff_file* coff_parse_from_path(const char *path, alloc_pool_t *parent_pool)
{
    FILE *f = fopen(path, "rb");
    uint8_t *buf = malloc(1);
    uint32_t i=0, tmp;
    coff_file *coff;
    
    do {
        buf = realloc(buf, i + 128*1024);
        assert(buf);
        tmp = fread(buf+i, 1, 128*1024, f);
        i += tmp;
        
        // don't load more than 64mb - there are surely no 64MB A/UX kernels
    } while ((tmp > 0) && (i < 64*1024*1024));
    
    
    coff = coff_parse(buf, i, parent_pool);
    free(buf);
    return coff;
}

// dump some data about a coff_file structure
void print_coff_info(coff_file *coff)
{
    char timebuf[32];
    time_t timestamp = coff->timestamp;
    uint32_t i;
    
    slog("Magic = 0x%04x\n", coff->magic);
    slog("Linked on %s", ctime_r(&timestamp, timebuf));
    slog("Num sections = %u\n", coff->num_sections);
    
    slog("debug: num_symbols=%u, symtab_offset=0x%x\n", coff->num_symbols, coff->symtab_offset);
    
    for (i=0; i<coff->num_sections; i++) {
        coff_section *s = &coff->sections[i];
        char name[9];
        memcpy(name, s->name, 8);
        name[8] = 0;
        slog("Section #%u: %s\n", i+1, name);
        slog("\taddr=0x%08x, len=0x%x, (debug: paddr=0x%08x, flags=0x%08x)\n", s->v_addr, s->sz, s->p_addr, s->flags);
    }
}

coff_symbol* coff_find_symbol(coff_file *coff, const char *name)
{
    uint32_t i;
    
    for (i=0; i < coff->num_symbols; i++) {
        if (strcmp(coff->symbols[i].name, name) == 0)
            return &coff->symbols[i];
    }
    return NULL;
}

coff_symbol* coff_find_func(coff_file *coff, uint32_t addr)
{
    rb_node *cur;
    coff_symbol *last = NULL;
    
    // slog("coff->num_symbols = %u\n", coff->num_symbols);
    if (coff->num_symbols == 0) 
        return NULL;
    cur = coff->func_tree->root;
    
    while (cur) {
        // slog("... iterating\n");
        if (addr < cur->key) 
            cur = cur->left;
        else if (addr > cur->key) {
            last = *(coff_symbol**)&cur[1];
            cur = cur->right;
        }
        else
            return *(coff_symbol**)&cur[1];
    }
    
    return last;
}
      

// big endian -> native int
uint32_t be2native (uint8_t **dat, uint32_t bytes)
{
    uint32_t i, v = 0;
    for (i=0; i < bytes; i++)
        v = (v<<8) | (*dat)[i];
    (*dat) += bytes;
    return v;
}
    

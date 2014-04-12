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

#ifndef _COFF_H
#define _COFF_H

#include <stdint.h>
#include "redblack.h"

typedef struct {
    char *name;
    uint32_t value;
    uint16_t scnum, type;
    uint8_t sclass, numaux;
} coff_symbol;

// informed by http://www.delorie.com/djgpp/doc/coff/scnhdr.html
typedef struct {
    char name[8];
    uint32_t p_addr;
    uint32_t v_addr;
    uint32_t sz;
    uint32_t data_ptr;
    uint32_t reloc_ptr;
    uint32_t line_ptr;
    uint16_t num_relocs;
    uint16_t num_lines;
    uint32_t flags;
    
    uint8_t *data;
} coff_section;

// data for this segment appears in the file, but shouldn't be copied into memory
#define coff_copy 0x0010
#define coff_text 0x0020
#define coff_data 0x0040
#define coff_bss  0x0080

typedef struct {
    uint16_t magic;
    uint16_t num_sections;
    uint32_t timestamp;
    uint32_t symtab_offset;
    uint32_t num_symbols;
    uint16_t opt_header_len;
    uint16_t flags;
    uint8_t *opt_header;
    coff_section *sections;
    rb_tree *func_tree;
    coff_symbol *symbols;
    
} coff_file;

coff_symbol* coff_find_func(coff_file *coff, uint32_t addr);
coff_symbol* coff_find_symbol(coff_file *coff, const char *name);

coff_file* coff_parse(uint8_t *buf, uint32_t buflen);
coff_file* coff_parse_from_path(const char *path);
uint32_t be2native (uint8_t **dat, uint32_t bytes);
void print_coff_info(coff_file *coff);


#endif // _COFF_H

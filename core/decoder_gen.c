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
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

typedef struct {
    uint8_t add;
    const char *descriptor;
} range_t;

typedef struct {
    range_t *ranges;
    uint32_t numranges;
    uint8_t use_ea;
    uint8_t allowed_ea_modes[12];
} range_group_t;

typedef struct {
    range_group_t *groups;
    uint32_t numgroups, curgroup;
    const char *name;
    uint8_t supervisor_only;
    uint8_t supported_architectures[5]; // 680x0
} inst_t;

typedef struct {
    inst_t inst[256];
    uint32_t num_instructions;
    uint32_t curinst;
    
    uint8_t inst_map[0x10000];
} generator_ctx_t;

generator_ctx_t ctx;

/* --- generator API --- */

// All but addr reg allowed
const uint8_t _ea_data[12] =      {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
// All but registers allowed
const uint8_t _ea_memory[12] =    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
// No regs, pre/post-inc/dec, or immediate
const uint8_t _ea_control[12] =   {0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1};
// Erratum: 68kprm table 2-4 says that "program counter memory indirect"
//          is alterable, but that can't be right.
//          Also it says that (xxx).L/W aren't alterable, but it seems they are.
const uint8_t _ea_alterable[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0};

enum ea_definitions {
    EA_000 = 0,
    EA_001,
    EA_010,
    EA_011,
    EA_100,
    EA_101,
    EA_110,
    EA_111_000,
    EA_111_001,
    EA_111_100,
    EA_111_010,
    EA_111_011
};

// For each of the 12 EA modes, this is bitmap of the allowed lower 6 bits
const uint64_t ea_mode_masks[12] = {
    0x00000000000000ff, // 000
    0x000000000000ff00, // 001
    0x0000000000ff0000, // 010
    0x00000000ff000000, // 011
    0x000000ff00000000, // 100
    0x0000ff0000000000, // 101
    0x00ff000000000000, // 110
    0x0100000000000000, // 111_000
    0x0200000000000000, // 111_001
    0x1000000000000000, // 111_100
    0x0400000000000000, // 111_010
    0x0800000000000000, // 111_011
};

void ea_data(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    group->use_ea = 1;
    memcpy(group->allowed_ea_modes, _ea_data, 12);
}
void ea_memory(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    group->use_ea = 1;
    memcpy(group->allowed_ea_modes, _ea_memory, 12);
}
void ea_control(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    group->use_ea = 1;
    memcpy(group->allowed_ea_modes, _ea_control, 12);
}
void ea_alterable(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    group->use_ea = 1;
    memcpy(group->allowed_ea_modes, _ea_alterable, 12);
}
void ea_control_alterable(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    uint32_t i;
    group->use_ea = 1;
    for (i=0; i<12; i++)
        group->allowed_ea_modes[i] = _ea_alterable[i] && _ea_control[i];
}
void ea_memory_alterable(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    uint32_t i;
    group->use_ea = 1;
    for (i=0; i<12; i++)
        group->allowed_ea_modes[i] = _ea_alterable[i] && _ea_memory[i];
}
void ea_data_alterable(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    uint32_t i;
    group->use_ea = 1;
    for (i=0; i<12; i++)
        group->allowed_ea_modes[i] = _ea_alterable[i] && _ea_data[i];
}
void ea_all(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    uint32_t i;
    group->use_ea = 1;
    for (i=0; i<12; i++) group->allowed_ea_modes[i] = 1;
}

void no_ea(inst_t *inst) {
    range_group_t *group = &inst->groups[inst->curgroup];
    assert(group->use_ea == 0xff);
    group->use_ea = 0;
}

void ea_add_mode(inst_t *inst, enum ea_definitions mode)
{
    range_group_t *group = &inst->groups[inst->curgroup];
    group->use_ea = 1;
    assert(!group->allowed_ea_modes[mode]);
    group->allowed_ea_modes[mode] = 1;
}

void ea_sub_mode(inst_t *inst, enum ea_definitions mode)
{
    range_group_t *group = &inst->groups[inst->curgroup];
    group->use_ea = 1;
    assert(group->allowed_ea_modes[mode]);
    group->allowed_ea_modes[mode] = 0;
}

void add_range(inst_t *inst, const char *descriptor)
{
    range_group_t *group = &inst->groups[inst->curgroup];
    group->ranges = realloc(group->ranges, sizeof(range_t) * (group->numranges+1));
    range_t *range = &group->ranges[group->numranges];
    group->numranges++;
    
    range->add = 1;
    range->descriptor = descriptor;
}

void sub_range(inst_t *inst, const char *descriptor)
{
    range_group_t *group = &inst->groups[inst->curgroup];
    group->ranges = realloc(group->ranges, sizeof(range_t) * (group->numranges+1));
    range_t *range = &group->ranges[group->numranges];
    group->numranges++;
    
    range->add = 0;
    range->descriptor = descriptor;
}

void supervisor_only(inst_t *inst)
{
    assert(!inst->supervisor_only);
    inst->supervisor_only = 1;
}

void set_range_group(inst_t *inst, uint32_t curgroup)
{
    assert(curgroup < inst->numgroups);
    inst->curgroup = curgroup;
}

inst_t* new_inst (const char *name, const char *architectures, uint32_t numgroups)
{
    uint32_t i, curinst = ctx.num_instructions;
    
    ctx.num_instructions++;
    ctx.curinst = curinst;
    
    inst_t *inst = &ctx.inst[curinst];
    
    memset(inst, 0, sizeof(inst_t));
    
    inst->name = name;
    inst->supervisor_only = 0;
    
    char tmp[128];
    if ((sscanf(architectures, "%s", tmp) == 1) && (strcasecmp(tmp, "all") == 0)) {
        for (i=0; i<5; i++)
            inst->supported_architectures[i] = 1;
    }
    else {
        char c;
        for (i=0; (c=architectures[i]) != 0; i++) {
            if (isspace(c)) continue;
            if (isdigit(c)) {
                c -= '0';
                assert(c <= 4);
                assert(!inst->supported_architectures[c]);
                inst->supported_architectures[c] = 1;
                continue ;
            }
            printf("Extraneous characters in arch string for %s\n", name);
            assert(!"extraneous characters in architecture string");
        }
    }
    
    inst->numgroups = numgroups;
    inst->groups = (range_group_t*) calloc(numgroups, sizeof(range_group_t));
    inst->curgroup = 0;
    
    for (i=0; i<numgroups; i++) {
        inst->groups[i].ranges = malloc(sizeof(range_t));
        inst->groups[i].use_ea = 0xff; // invalid
    }
    
    return inst;
}

/* --- Generator guts --- */

typedef struct {
    uint16_t fixed;
    uint16_t mask;
} desc_t;

static desc_t process_descriptor(uint32_t inst_num, range_group_t *group, uint32_t range_num)
{
    uint16_t fixed=0, mask=0, EA=0, bits=0;
    uint32_t i;
    desc_t result;
    char c;
    
    range_t *range = &group->ranges[range_num];
    for (i=0; (c=range->descriptor[i]) != 0; i++) {
        if (isspace(c)) continue;
        
        fixed <<= 1;
        mask <<= 1;
        EA <<= 1;
        if (c == '1' || c == '0') {
            fixed |= (c - '0');
            mask |= 1;
        }
        else if (c == 'M')
            EA |= 1; // both x and M will be considered wild bits
                     // Later, we'll subtract the unused EA modes from the bitmask
        else if (c != 'x')
            goto bad;
        
        bits++;
    }
    if (bits != 16)
        goto bad;
    else if (group->use_ea && (EA != 0x3f))
        goto bad;
    else if (!group->use_ea && (EA != 0))
        goto bad;
    
    result.fixed = fixed;
    result.mask = mask;
    
    return result;
    
bad:
    printf("bad descriptor for instruction %s '%s'\n",
           ctx.inst[inst_num].name, range->descriptor);
    assert(!"bad desc");
}

// Build a map (uint64_t) of the allowable lower 6 bits
static uint64_t process_ea_modes(range_group_t *group)
{
    uint64_t result = 0;
    uint32_t i;
    
    for (i=0; i<12; i++)
        if (group->allowed_ea_modes[i])
            result |= ea_mode_masks[i];
    return result;
}

static void process_group(uint32_t inst_num, uint32_t group_num)
{
    inst_t *inst = &ctx.inst[inst_num];
    range_group_t *group = &inst->groups[group_num];
    uint8_t bitmap[8192];
    uint32_t i;
    
    memset(bitmap, 0, 8192);
    
    if (group->use_ea == 0xff) {
        printf("inst %s group %u needs EA specified\n", inst->name, group_num);
        assert(!"blowup");
    }
    
    // add or subtract all the descriptors into the bitmap
    for (i=0; i<group->numranges; i++) {
        desc_t desc = process_descriptor(inst_num, group, i);
        uint32_t j;
        if (group->ranges[i].add) {
            for (j=0; j < 0x10000; j++) {
                if ((j & desc.mask) == desc.fixed)
                    bitmap[j >> 3] |= (1 << (j & 7)); // add this opcode
            }
        }
        else {
            for (j=0; j < 0x10000; j++) {
                if ((j & desc.mask) == desc.fixed)
                    bitmap[j >> 3] &= ~(1 << (j & 7)); // subtract this opcode
            }
        }
    }
    
    // Now we need to subtract the impossible and invalid EA modes, if use_ea
    if (group->use_ea) {
        // Negate the allowable modes map (making a disallowed modes map)
        const uint64_t disallowed_modes = ~process_ea_modes(group);
        for (i=0; i<0x10000; i++) {
            if (disallowed_modes & (1ULL << (i & 0x3f)))
                bitmap[i >> 3] &= ~(1 << (i & 7));
        }
    }
    
    // Now feed bitmap[] into the global instruction map
    for (i=0; i<0x10000; i++) {
        if (!(bitmap[i >> 3] & (1 << (i & 7)))) continue;
        
        if (ctx.inst_map[i] != 0) {
            printf("Error! instructions %s(%u) and %s(%u) overlap at opcode 0x%04x\n",
                   ctx.inst[ctx.inst_map[i]].name, ctx.inst_map[i],
                   ctx.inst[inst_num].name, inst_num, i);
            assert(!"blowup");
        }
        
        ctx.inst_map[i] = inst_num;
    }
    
}

void digest_definitions(uint32_t arch)
{
    uint32_t i;
    for (i=0; i<ctx.num_instructions; i++) {
        inst_t *inst = &ctx.inst[i];
        uint32_t j;
        
        if (!inst->supported_architectures[arch])
            continue;
        
        for (j=0; j<inst->numgroups; j++)
            process_group(i, j);
    }
    
}

void write_decoder (const char *prefix, const char *path)
{
    uint32_t i;
    char *file_path = malloc(strlen(path) + strlen(prefix) + 32);
    sprintf(file_path, "%s/%s_decoder_guts.c", path, prefix);
    
    FILE *f = fopen(file_path, "w");
    
    if (f == NULL) {
        printf("write_decoder: can't open %s for writing\n", path);
        assert(!"blowup");
    }
    
    fprintf(f, "const uint32_t %s_num_instructions = %u;\n\n", prefix, ctx.num_instructions);
    
    
    /* --- write inst_num -> function_pointer table --- */
    
    fprintf(f, "typedef void (*%s_func_ptr)();\n", prefix);
    fprintf(f, "%s_func_ptr %s_instruction_to_pointer[%u] = {\n", prefix, prefix, ctx.num_instructions);
    for (i=0; i<ctx.num_instructions-1; i++) {
        fprintf(f, "\t%s_%s,\n", prefix, ctx.inst[i].name);
    }
    fprintf(f, "\t%s_%s\n};\n\n", prefix, ctx.inst[i].name);
    
    /* --- write opcode -> inst_num table --- */
    
    fprintf(f, "const uint8_t %s_opcode_map[0x10000] = {\n", prefix);
    for (i=0; i < 0xffff; i++) {
        if ((i % 8) == 0)
            fprintf(f, "\t");
        fprintf(f, "0x%02x, ", ctx.inst_map[i]);
        if (((i + 1) % 8) == 0)
            fprintf(f, "\n");
    }
    fprintf(f, "0x%02x\n};\n\n", ctx.inst_map[i]);

}


void init() {
    memset(ctx.inst_map, 0x00, 0x10000); // 0x00 -> unset
    
    ctx.num_instructions = 1;
    ctx.inst[0].name = "unknown";
    ctx.inst[0].numgroups = 0;
}

void begin_definitions();
int main (int argc, char **argv)
{
    if (argc != 3) {
        printf("arguments: ./decoder_gen inst|dis intermediates/");
        return 0;
    }
    
    init();
    begin_definitions();
    digest_definitions(2); // build for 68020. Dunno if I'll ever support other architectures
    write_decoder(argv[1], argv[2]);
    
    // printf("num_instructions = %u\n", ctx.num_instructions);
    
    return 0;
}

/* --- Big list of definitions --- */

void begin_definitions()
{
    
    /* --- Unprivileged integer instructions --- */
    
    { // abcd
        inst_t *inst = new_inst("abcd", "all", 1);
        add_range(inst, "1100 xxx 10000 x xxx");
        no_ea(inst);
    }
    
    { // add
        inst_t *inst = new_inst("add", "all", 3);
        { // to-register (EA mode == addr register)
            set_range_group(inst, 0);
            add_range(inst, "1101 xxx 001 MMMMMM");
            add_range(inst, "1101 xxx 010 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        { // to-register (all other EA modes)
            set_range_group(inst, 1);
            add_range(inst, "1101 xxx 0xx MMMMMM");
            sub_range(inst, "1101 xxx 011 MMMMMM");
            ea_all(inst);
            ea_sub_mode(inst, EA_001);
        }
        { // to-EA
            set_range_group(inst, 2);
            add_range(inst, "1101 xxx 1xx MMMMMM");
            sub_range(inst, "1101 xxx 111 MMMMMM");
            ea_memory_alterable(inst);
        }
    }

    { // adda
        inst_t *inst = new_inst("adda", "all", 1);
        add_range(inst, "1101 xxx x11 MMMMMM");
        ea_all(inst);
    }
    
    { // addi
        inst_t *inst = new_inst("addi", "all", 1);
        add_range(inst, "0000 0110 xx MMMMMM");
        sub_range(inst, "0000 0110 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // addq
        inst_t *inst = new_inst("addq", "all", 2);
        { // ea mode == addr reg
            set_range_group(inst, 0);
            add_range(inst, "0101 xxx 0 01 MMMMMM");
            add_range(inst, "0101 xxx 0 10 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        {
            set_range_group(inst, 1);
            add_range(inst, "0101 xxx 0 xx MMMMMM");
            sub_range(inst, "0101 xxx 0 11 MMMMMM");
            ea_alterable(inst);
            ea_sub_mode(inst, EA_001);
        }
    }
    
    { // addx
        inst_t *inst = new_inst("addx", "all", 1);
        add_range(inst, "1101 xxx 1 xx 00 x xxx");
        sub_range(inst, "1101 xxx 1 11 00 x xxx");
        no_ea(inst);
    }
    
    { // and
        inst_t *inst  = new_inst("and", "all", 2);
        { // to register
            set_range_group(inst, 0);
            add_range(inst, "1100 xxx 0xx MMMMMM");
            sub_range(inst, "1100 xxx 011 MMMMMM");
            ea_data(inst);
        }
        { // to memory
            set_range_group(inst, 1);
            add_range(inst, "1100 xxx 1xx MMMMMM");
            sub_range(inst, "1100 xxx 111 MMMMMM");
            ea_memory_alterable(inst);
        }
    }
    
    { // andi
        inst_t *inst = new_inst("andi", "all", 1);
        add_range(inst, "0000 0010 xx MMMMMM");
        sub_range(inst, "0000 0010 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // andi_to_ccr
        inst_t *inst = new_inst("andi_to_ccr", "all", 1);
        add_range(inst, "0000 0010 00111100");
        no_ea(inst);
    }
    
    { // asx_reg
        inst_t *inst = new_inst("asx_reg", "all", 1);
        add_range(inst, "1110 xxx x xx x 00 xxx");
        sub_range(inst, "1110 xxx x 11 x 00 xxx");
        no_ea(inst);
    }
    
    { // asx_mem
        inst_t *inst = new_inst("asx_mem", "all", 1);
        add_range(inst, "1110 000 x 11 MMMMMM");
        ea_memory_alterable(inst);
    }
    
    { // bcc
        inst_t *inst = new_inst("bcc", "all", 1);
        add_range(inst, "0110 xxxx xxxxxxxx");
        sub_range(inst, "0110 0001 xxxxxxxx"); // this is BSR
        no_ea(inst);
    }
    
    {
        inst_t *inst = new_inst("bchg_reg", "all", 1);
        add_range(inst, "0000 xxx 101 MMMMMM");
        ea_data_alterable(inst);
    }
    
    {
        inst_t *inst = new_inst("bchg_immediate", "all", 1);
        add_range(inst, "0000 1000 01 MMMMMM");
        ea_data_alterable(inst);
    }
    
    {
        inst_t *inst = new_inst("bclr_reg", "all", 1);
        add_range(inst, "0000 xxx 110 MMMMMM");
        ea_data_alterable(inst);
    }
    
    {
        inst_t *inst = new_inst("bclr_immediate", "all", 1);
        add_range(inst, "0000 1000 10 MMMMMM");
        ea_data_alterable(inst);
    }
    

    { // bfchg
        inst_t *inst = new_inst("bfchg", "234", 1);
        add_range(inst, "1110 1010 11 MMMMMM");
        ea_control_alterable(inst); // Yes, control_alterable
        ea_add_mode(inst, EA_000);
    }
    
    { // bfclr
        inst_t *inst = new_inst("bfclr", "234", 1);
        add_range(inst, "1110 1100 11 MMMMMM");
        ea_control_alterable(inst); // Yes, control_alterable
        ea_add_mode(inst, EA_000);
    }
    
    { // bfexts
        inst_t *inst = new_inst("bfexts", "234", 1);
        add_range(inst, "1110 1011 11 MMMMMM");
        ea_control(inst); // Yes, control (doesn't alter)
        ea_add_mode(inst, EA_000);
    }
    
    { // bfextu
        inst_t *inst = new_inst("bfextu", "234", 1);
        add_range(inst, "1110 1001 11 MMMMMM");
        ea_control(inst); // Yes, control
        ea_add_mode(inst, EA_000);
    }
    
    { // bfffo
        inst_t *inst = new_inst("bfffo", "234", 1);
        add_range(inst, "1110 1101 11 MMMMMM"); // eratum: incorrect in 68kprm
        ea_control(inst); // Yes, control
        ea_add_mode(inst, EA_000);
    }
    
    { // bfins
        inst_t *inst = new_inst("bfins", "234", 1);
        add_range(inst, "1110 1111 11 MMMMMM");
        ea_control_alterable(inst); // Yes, control_alterable
        ea_add_mode(inst, EA_000);
    }
    
    { // bfset
        inst_t *inst = new_inst("bfset", "234", 1);
        add_range(inst, "1110 1110 11 MMMMMM");
        ea_control_alterable(inst); // Yes, control_alterable
        ea_add_mode(inst, EA_000);
    }
    
    { // bftst
        inst_t *inst = new_inst("bftst", "234", 1);
        add_range(inst, "1110 1000 11 MMMMMM");
        ea_control(inst); // Yes, control
        ea_add_mode(inst, EA_000);
    }
    
    { // bkpt
        inst_t *inst = new_inst("bkpt", "1234", 1); // MC68EC000 is also supported
        add_range(inst, "0100 1000 0100 1 xxx");
        no_ea(inst);
    }
    
    // This is just bcc with cc == 0 == TRUE
    /*{ // bra
        inst_t *inst = new_inst("bra", "all", 1);
        add_range(inst, "0110 0000 xxxxxxxx");
        no_ea(inst);
    }*/
    
    {
        inst_t *inst = new_inst("bset_reg", "all", 1);
        add_range(inst, "0000 xxx 111 MMMMMM");
        ea_data_alterable(inst);
    }
    
    {
        inst_t *inst = new_inst("bset_immediate", "all", 1);
        add_range(inst, "0000 1000 11 MMMMMM");
        ea_data_alterable(inst);
    }

    { // bsr
        inst_t *inst = new_inst("bsr", "all", 1); // long-mode is 68020-040 only
        add_range(inst, "0110 0001 xxxxxxxx");
        no_ea(inst);
    }
    
    { // btst_reg
        inst_t *inst = new_inst("btst_reg", "all", 1);
        add_range(inst, "0000 xxx 100 MMMMMM");
        ea_data(inst);
    }
    
    { // btst_immediate
        inst_t *inst = new_inst("btst_immediate", "all", 1);
        add_range(inst, "0000 1000 00 MMMMMM");
        ea_data(inst);
    }
    
    { // callm
        inst_t *inst = new_inst("callm", "2", 1);
        add_range(inst, "0000 0110 11 MMMMMM");
        ea_control(inst);
    }
    
    { // cas
        inst_t *inst = new_inst("cas", "234", 1);
        add_range(inst, "0000 1xx 011 MMMMMM");
        sub_range(inst, "0000 100 011 MMMMMM");
        ea_memory_alterable(inst);
    }
    
    { // cas2
        inst_t *inst = new_inst("cas2", "234", 1);
        add_range(inst, "0000 1 10 011111100");
        add_range(inst, "0000 1 11 011111100");
        no_ea(inst);
    }
    
    { // chk
        inst_t *inst = new_inst("chk", "all", 1);
        add_range(inst, "0100 xxx 10 0 MMMMMM");
        add_range(inst, "0100 xxx 11 0 MMMMMM");
        ea_data(inst);
    }
    
    { // chk2+cmp2 (These instructions are distinguished by the extension word)
        inst_t *inst = new_inst("chk2_cmp2", "234", 1);
        add_range(inst, "0000 0xx0 11 MMMMMM");
        sub_range(inst, "0000 0110 11 MMMMMM");
        ea_control(inst);
    }
    
    { // clr
        inst_t *inst = new_inst("clr", "all", 1);
        add_range(inst, "0100 0010 xx MMMMMM");
        sub_range(inst, "0100 0010 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // cmp
        inst_t *inst = new_inst("cmp", "all", 2);
        { // address mode reg (byte-mode isn't supported for address registers)
            set_range_group(inst, 0);
            add_range(inst, "1011 xxx 001 MMMMMM");
            add_range(inst, "1011 xxx 010 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        { // other modes
            set_range_group(inst, 1);
            add_range(inst, "1011 xxx 0xx MMMMMM");
            sub_range(inst, "1011 xxx 011 MMMMMM");
            ea_all(inst);
            ea_sub_mode(inst, EA_001);
        }
    }
    
    { // cmpa
        inst_t *inst = new_inst("cmpa", "all", 1);
        add_range(inst, "1011 xxx x11 MMMMMM");
        ea_all(inst);
    }
    
    { // cmpi
        inst_t *inst = new_inst("cmpi", "all", 1);
        add_range(inst, "0000 1100 xx MMMMMM");
        sub_range(inst, "0000 1100 11 MMMMMM");
        ea_data(inst);
        /* 
         * Erratum: documentation says "data addressing modes",
         * but the table doesn't list immediate mode
         */
        ea_sub_mode(inst, EA_111_100);
    }
    
    { // cmpm
        inst_t *inst = new_inst("cmpm", "all", 1);
        add_range(inst, "1011 xxx 1 xx 001 xxx");
        sub_range(inst, "1011 xxx 1 11 001 xxx");
        no_ea(inst);
    }
    
    // cmp2 is handled by chk2_cmp2
    
    { // DBcc
        inst_t *inst = new_inst("dbcc", "all", 1);
        add_range(inst, "0101 xxxx 11001 xxx");
        no_ea(inst);
    }
    
    { // divs
        inst_t *inst = new_inst("divs", "all", 1);
        add_range(inst, "1000 xxx 111 MMMMMM");
        ea_data(inst); // Erratum: 68kprm says "data alterable", but means "data"
    }
    
    { // divu
        inst_t *inst = new_inst("divu", "all", 1);
        add_range(inst, "1000 xxx 011 MMMMMM");
        ea_data(inst); // 68kprm gets it right here
    }
    
    { // divl (both signed and unsigned)
        inst_t *inst = new_inst("long_div", "234", 1);
        add_range(inst, "0100 1100 01 MMMMMM");
        ea_data(inst); // Erratum: 68kprm means "data" here
    }
    
    { // eor
        inst_t *inst = new_inst("eor", "all", 1);
        add_range(inst, "1011 xxx 1xx MMMMMM");
        sub_range(inst, "1011 xxx 111 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // eori
        inst_t *inst = new_inst("eori", "all", 1);
        add_range(inst, "0000 1010 xx MMMMMM");
        sub_range(inst, "0000 1010 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // eori_to_ccr
        inst_t *inst = new_inst("eori_to_ccr", "all", 1);
        add_range(inst, "0000 1010 0011 1100");
        no_ea(inst);
    }
    
    { // exg
        inst_t *inst = new_inst("exg", "all", 1);
        add_range(inst, "1100 xxx 1 01000 xxx");
        add_range(inst, "1100 xxx 1 01001 xxx");
        add_range(inst, "1100 xxx 1 10001 xxx");
        no_ea(inst);
    }
    
    { // ext
        inst_t *inst = new_inst("ext", "all", 1);
        add_range(inst, "0100 100 010 000 xxx");
        add_range(inst, "0100 100 011 000 xxx");
        add_range(inst, "0100 100 111 000 xxx");
        no_ea(inst);
    }
    
    { // illegal
        inst_t *inst = new_inst("illegal", "all", 1);
        add_range(inst, "0100 1010 1111 1100");
        no_ea(inst);
    }
    
    { // jmp
        inst_t *inst = new_inst("jmp", "all", 1);
        add_range(inst, "0100 1110 11 MMMMMM");
        ea_control(inst);
    }
    
    { // jsr
        inst_t *inst = new_inst("jsr", "all", 1);
        add_range(inst, "0100 1110 10 MMMMMM");
        ea_control(inst);
    }
    
    { // lea
        inst_t *inst = new_inst("lea", "all", 1);
        add_range(inst, "0100 xxx 111 MMMMMM");
        ea_control(inst);
    }
    
    { // link_word
        inst_t *inst = new_inst("link_word", "all", 1);
        add_range(inst, "0100 1110 0101 0 xxx"); // word
        no_ea(inst);
    }
    
    { // link_long
        inst_t *inst = new_inst("link_long", "all", 1);
        add_range(inst, "0100 1000 0000 1 xxx"); // long
        no_ea(inst);
    }
    
    { // lsx_reg
        inst_t *inst = new_inst("lsx_reg", "all", 1);
        add_range(inst, "1110 xxx x xx x 01 xxx");
        sub_range(inst, "1110 xxx x 11 x 01 xxx");
        no_ea(inst);
    }
    
    { // lsx_mem
        inst_t *inst = new_inst("lsx_mem", "all", 1);
        add_range(inst, "1110 001 x 11 MMMMMM");
        ea_memory_alterable(inst);
    }

    
    { // move
        inst_t *inst = new_inst("move", "all", 1);
        
        // I'm manually specifying this entire thing, since the EA description is too complicated
        no_ea(inst);
        
        // Add in all modes, all sizes
        add_range(inst, "00 xx xxxxxx xxxxxx");
        
        // subtract the invalid destination modes (001, 111_010, 111_011, 111_100)
        sub_range(inst, "00 xx xxx 001 xxxxxx");
        sub_range(inst, "00 xx 010 111 xxxxxx");
        sub_range(inst, "00 xx 011 111 xxxxxx");
        sub_range(inst, "00 xx 100 111 xxxxxx");
        
        // subtract the illegal destination modes (111_101, 111_110, 111_111)
        sub_range(inst, "00 xx 101 111 xxxxxx");
        sub_range(inst, "00 xx 110 111 xxxxxx");
        sub_range(inst, "00 xx 111 111 xxxxxx");
    
        // subtract the illegal source modes
        sub_range(inst, "00 xx xxxxxx 111 101");
        sub_range(inst, "00 xx xxxxxx 111 110");
        sub_range(inst, "00 xx xxxxxx 111 111");
        
        // Subtract address reg source mode for size==byte
        sub_range(inst, "00 01 xxxxxx 001 xxx");
        
        // subtract illegal size (00)
        sub_range(inst, "00 00 xxxxxx xxxxxx");
        
        // subtract move_from_d
        sub_range(inst, "00 xx xxxxxx 000xxx");
        
        // subtract move_to_d
        sub_range(inst, "00 xx xxx000 xxxxxx");
    }
    
    { // move_d_to_d
        inst_t *inst = new_inst("move_d_to_d", "all", 1);
        no_ea(inst);
        
        add_range(inst, "00 xx xxx 000 000 xxx");
        sub_range(inst, "00 00 xxx 000 000 xxx");
    }
    
    { // move_to_d
        inst_t *inst = new_inst("move_to_d", "all", 2);
        { // EA mode == addr register (byte-size not allowed)
            set_range_group(inst, 0);
            add_range(inst, "00 11 xxx000 MMMMMM");
            add_range(inst, "00 10 xxx000 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        { // all other EA modes
            set_range_group(inst, 1);
            add_range(inst, "00 xx xxx000 MMMMMM");
            sub_range(inst, "00 00 xxx000 MMMMMM");
            ea_all(inst);
            ea_sub_mode(inst, EA_001);
            ea_sub_mode(inst, EA_000); // EA_000 is handled by move_d_to_d
        }
    }
    
    { // move_from_d
        inst_t *inst = new_inst("move_from_d", "all", 1);
        
        // I'm manually specifying this entire thing, since the EA description is too complicated
        no_ea(inst);
        
        // Add in all modes, all sizes
        add_range(inst, "00 xx xxxxxx 000xxx");
        
        // subtract the invalid destination modes (001, 111_010, 111_011, 111_100)
        sub_range(inst, "00 xx xxx 001 xxxxxx");
        sub_range(inst, "00 xx 010 111 xxxxxx");
        sub_range(inst, "00 xx 011 111 xxxxxx");
        sub_range(inst, "00 xx 100 111 xxxxxx");
        
        // subtract the illegal destination modes (111_101, 111_110, 111_111)
        sub_range(inst, "00 xx 101 111 xxxxxx");
        sub_range(inst, "00 xx 110 111 xxxxxx");
        sub_range(inst, "00 xx 111 111 xxxxxx");
        
        // subtract illegal size (00)
        sub_range(inst, "00 00 xxxxxx xxxxxx");
        
        // subtract move_d_to_d
        sub_range(inst, "00 xx xxx 000 000xxx");
    }
    
    { // movea
        inst_t *inst = new_inst("movea", "all", 1);
        add_range(inst, "00 10 xxx 001 MMMMMM");
        add_range(inst, "00 11 xxx 001 MMMMMM");
        ea_all(inst);
    }
    
    { // move_from_ccr
        inst_t *inst = new_inst("move_from_ccr", "1234", 1);
        add_range(inst, "0100 0010 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // move_to_ccr
        inst_t *inst = new_inst("move_to_ccr", "all", 1);
        add_range(inst, "0100 0100 11 MMMMMM");
        ea_data(inst);
    }
    
    { // move_from_sr
        inst_t *inst = new_inst("move_from_sr", "all", 1); // NOTE: this is supervisor-only for 68010-68040
        add_range(inst, "0100 0000 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // move16
        inst_t *inst = new_inst("move16", "4", 1);
        add_range(inst, "1111 0110 001 00 xxx"); // postincrement source and destination
        add_range(inst, "1111 0110 000 xx xxx"); // absolute long address source or destination
        no_ea(inst);
    }
    
    { // movem
        inst_t *inst = new_inst("movem", "all", 2);
        { // to-mem
            set_range_group(inst, 0);
            add_range(inst, "0100 1000 1x MMMMMM");
            ea_control_alterable(inst);
            ea_add_mode(inst, EA_100);
        }
        { // to-reg
            set_range_group(inst, 1);
            add_range(inst, "0100 1100 1x MMMMMM");
            ea_control(inst);
            ea_add_mode(inst, EA_011);
        }
    }
    
    { // movep
        inst_t *inst = new_inst("movep", "all", 1);
        add_range(inst, "0000 xxx 1xx 001 xxx");
        no_ea(inst);
    }
    
    { // moveq
        inst_t *inst = new_inst("moveq", "all", 1);
        add_range(inst, "0111 xxx 0 xxxxxxxx");
        no_ea(inst);
    }
    
    { // muls
        inst_t *inst = new_inst("muls", "all", 1);
        add_range(inst, "1100 xxx 111 MMMMMM");
        ea_data(inst); // Erratum: 68kprm says "data alterable", but means "data"
    }
   
    { // mulu
        inst_t *inst = new_inst("mulu", "all", 1);
        add_range(inst, "1100 xxx 011 MMMMMM");
        ea_data(inst);
    }
    
    { // mulsl/mulul
        inst_t *inst = new_inst("long_mul", "234", 1);
        add_range(inst, "0100 1100 00 MMMMMM"); // Erratum: muls_long says "data alterable"
        ea_data(inst);
    }
    
    { // nbcd
        inst_t *inst = new_inst("nbcd", "all", 1);
        add_range(inst, "0100 1000 00 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // neg
        inst_t *inst = new_inst("neg", "all", 1);
        add_range(inst, "0100 0100 xx MMMMMM");
        sub_range(inst, "0100 0100 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // negx
        inst_t *inst = new_inst("negx", "all", 1);
        add_range(inst, "0100 0000 xx MMMMMM");
        sub_range(inst, "0100 0000 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // nop
        inst_t *inst = new_inst("nop", "all", 1);
        add_range(inst, "0100 1110 0111 0001");
        no_ea(inst);
    }
    
    { // not
        inst_t *inst = new_inst("not", "all", 1);
        add_range(inst, "0100 0110 xx MMMMMM");
        sub_range(inst, "0100 0110 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // or
        inst_t *inst = new_inst("or", "all", 2);
        { // to register
            set_range_group(inst, 0);
            add_range(inst, "1000 xxx 0xx MMMMMM");
            sub_range(inst, "1000 xxx 011 MMMMMM");
            ea_data(inst);
        }
        { // to memory
            set_range_group(inst, 1);
            add_range(inst, "1000 xxx 1xx MMMMMM");
            sub_range(inst, "1000 xxx 111 MMMMMM");
            ea_memory_alterable(inst);
        }
    }
    
    { // ori
        inst_t *inst = new_inst("ori", "all", 1);
        add_range(inst, "0000 0000 xx MMMMMM");
        sub_range(inst, "0000 0000 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // ori_to_ccr
        inst_t *inst = new_inst("ori_to_ccr", "all", 1);
        add_range(inst, "0000 0000 00 111100");
        no_ea(inst);
    }
    
    { // pack
        inst_t *inst = new_inst("pack", "234", 1);
        add_range(inst, "1000 xxx 10100 x xxx");
        no_ea(inst);
    }
    
    { // pea
        inst_t *inst = new_inst("pea", "all", 1);
        add_range(inst, "0100 1000 01 MMMMMM");
        ea_control(inst);
    }
    
    /*{ // rox
        inst_t *inst = new_inst("rox", "all", 2);
        { // register rotate
            set_range_group(inst, 0);
            add_range(inst, "1110 xxx x xx x 11 xxx");
            sub_range(inst, "1110 xxx x 11 x 11 xxx");
            no_ea(inst);
        }
        { // memory rotate
            set_range_group(inst, 1);
            add_range(inst, "1110 011 x 11 MMMMMM");
            ea_memory_alterable(inst);
        }
    }*/
    
    { // rox_reg
        inst_t *inst = new_inst("rox_reg", "all", 1);
        add_range(inst, "1110 xxx x xx x 11 xxx");
        sub_range(inst, "1110 xxx x 11 x 11 xxx");
        no_ea(inst);
    }
    
    { // rox_mem
        inst_t *inst = new_inst("rox_mem", "all", 1);
        add_range(inst, "1110 011 x 11 MMMMMM");
        ea_memory_alterable(inst);
    }

    
    /*{ // roxx
        inst_t *inst = new_inst("roxx", "all", 2);
        { // register rotate
            set_range_group(inst, 0);
            add_range(inst, "1110 xxx x xx x 10 xxx");
            sub_range(inst, "1110 xxx x 11 x 10 xxx");
            no_ea(inst);
        }
        { // memory rotate
            set_range_group(inst, 1);
            add_range(inst, "1110 010 x 11 MMMMMM");
            ea_memory_alterable(inst);
        }
    }*/
    
    { // roxx_reg
        inst_t *inst = new_inst("roxx_reg", "all", 1);
        add_range(inst, "1110 xxx x xx x 10 xxx");
        sub_range(inst, "1110 xxx x 11 x 10 xxx");
        no_ea(inst);
    }
    
    { // roxx_mem
        inst_t *inst = new_inst("roxx_mem", "all", 1);
        add_range(inst, "1110 010 x 11 MMMMMM");
        ea_memory_alterable(inst);
    }

    
    { // rtd
        inst_t *inst = new_inst("rtd", "1234", 1);
        add_range(inst, "0100 1110 0111 0100");
        no_ea(inst);
    }
    
    { // rtm
        inst_t *inst = new_inst("rtm", "2", 1);
        add_range(inst, "0000 0110 1100 x xxx");
        no_ea(inst);
    }
    
    { // rtr
        inst_t *inst = new_inst("rtr", "all", 1);
        add_range(inst, "0100 1110 0111 0111");
        no_ea(inst);
    }
    
    { // rts
        inst_t *inst = new_inst("rts", "all", 1);
        add_range(inst, "0100 1110 0111 0101");
        no_ea(inst);
    }
    
    { // sbcd
        inst_t *inst = new_inst("sbcd", "all", 1);
        add_range(inst, "1000 xxx 10000 x xxx");
        no_ea(inst);
    }
    
    { // scc
        inst_t *inst = new_inst("scc", "all", 1);
        add_range(inst, "0101 xxxx 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // sub
        inst_t *inst = new_inst("sub", "all", 3);
        { // to-register (EA mode == addr register)
            set_range_group(inst, 0);
            add_range(inst, "1001 xxx 001 MMMMMM");
            add_range(inst, "1001 xxx 010 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        { // to-register (all other EA modes)
            set_range_group(inst, 1);
            add_range(inst, "1001 xxx 0xx MMMMMM");
            sub_range(inst, "1001 xxx 011 MMMMMM");
            ea_all(inst);
            ea_sub_mode(inst, EA_001);
        }
        { // to-EA
            set_range_group(inst, 2);
            add_range(inst, "1001 xxx 1xx MMMMMM");
            sub_range(inst, "1001 xxx 111 MMMMMM");
            ea_memory_alterable(inst);
        }
    }
    
    { // suba
        inst_t *inst = new_inst("suba", "all", 1);
        add_range(inst, "1001 xxx x11 MMMMMM");
        ea_all(inst);
    }
    
    { // subi
        inst_t *inst = new_inst("subi", "all", 1);
        add_range(inst, "0000 0100 xx MMMMMM");
        sub_range(inst, "0000 0100 11 MMMMMM");
        ea_data_alterable(inst);
    }

    { // subq
        inst_t *inst = new_inst("subq", "all", 2);
        { // ea mode == addr reg
            set_range_group(inst, 0);
            add_range(inst, "0101 xxx 1 01 MMMMMM");
            add_range(inst, "0101 xxx 1 10 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        {
            set_range_group(inst, 1);
            add_range(inst, "0101 xxx 1 xx MMMMMM");
            sub_range(inst, "0101 xxx 1 11 MMMMMM");
            ea_alterable(inst);
            ea_sub_mode(inst, EA_001);
        }
    }
    
    { // subx
        inst_t *inst = new_inst("subx", "all", 1);
        add_range(inst, "1001 xxx 1 xx 00 x xxx");
        sub_range(inst, "1001 xxx 1 11 00 x xxx");
        no_ea(inst);
    }

    { // swap
        inst_t *inst = new_inst("swap", "all", 1);
        add_range(inst, "0100 1000 0100 0 xxx");
        no_ea(inst);
    }
    
    { // tas
        inst_t *inst = new_inst("tas", "all", 1);
        add_range(inst, "0100 1010 11 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // trap
        inst_t *inst = new_inst("trap", "all", 1);
        add_range(inst, "0100 1110 0100 xxxx");
        no_ea(inst);
    }
    
    { // trapcc
        inst_t *inst = new_inst("trapcc", "234", 1);
        add_range(inst, "0101 xxxx 11111 010");
        add_range(inst, "0101 xxxx 11111 011");
        add_range(inst, "0101 xxxx 11111 100");
        no_ea(inst);
    }
    
    { // trapv
        inst_t *inst = new_inst("trapv", "all", 1);
        add_range(inst, "0100 1110 0111 0110");
        no_ea(inst);
    }
    
    { // tst
        inst_t *inst = new_inst("tst", "all", 2);
        { // addr mode
            set_range_group(inst, 0);
            add_range(inst, "0100 1010 01 MMMMMM");
            add_range(inst, "0100 1010 10 MMMMMM");
            ea_add_mode(inst, EA_001);
        }
        { // other modes
            set_range_group(inst, 1);
            add_range(inst, "0100 1010 xx MMMMMM");
            sub_range(inst, "0100 1010 11 MMMMMM");
            ea_all(inst);
            ea_sub_mode(inst, EA_001);
        }
    }
    
    { // unlk
        inst_t *inst = new_inst("unlk", "all", 1);
        add_range(inst, "0100 1110 0101 1 xxx");
        no_ea(inst);
    }
    
    { // unpk
        inst_t *inst = new_inst("unpk", "234", 1);
        add_range(inst, "1000 xxx 11000 x xxx");
        no_ea(inst);
    }
    
    // --- supervisor instructions ---
    
    { // andi_to_sr
        inst_t *inst = new_inst("andi_to_sr", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0000 0010 0111 1100");
        no_ea(inst);
    }
    
    { // eori_to_sr
        inst_t *inst = new_inst("eori_to_sr", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0000 1010 0111 1100");
        no_ea(inst);
    }
    
    { // move_to_sr
        inst_t *inst = new_inst("move_to_sr", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0100 0110 11 MMMMMM");
        ea_data(inst);
    }
    
    { // move_usp
        inst_t *inst = new_inst("move_usp", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0100 1110 0110 x xxx");
        no_ea(inst);
    }
    
    { // movec
        inst_t *inst = new_inst("movec", "1234", 1);
        supervisor_only(inst);
        add_range(inst, "0100 1110 0111 101x");
        no_ea(inst);
    }
    
    { // moves
        inst_t *inst = new_inst("moves", "1234", 1);
        supervisor_only(inst);
        add_range(inst, "0000 1110 xx MMMMMM");
        sub_range(inst, "0000 1110 11 MMMMMM");
        ea_memory_alterable(inst);
    }
    
    { // ori_to_sr
        inst_t *inst = new_inst("ori_to_sr", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0000 0000 0111 1100");
        no_ea(inst);
    }
    
    { // reset
        inst_t *inst = new_inst("reset", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0100 1110 0111 0000");
        no_ea(inst);
    }
    
    { // rte
        inst_t *inst = new_inst("rte", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0100 1110 0111 0011");
        no_ea(inst);
    }
    
    { // stop
        inst_t *inst = new_inst("stop", "all", 1);
        supervisor_only(inst);
        add_range(inst, "0100 1110 0111 0010");
        no_ea(inst);
    }
    
    /* --- F/A-lines --- */
    
    { // a-line
        inst_t *inst = new_inst("a_line", "all", 1);
        add_range(inst, "1010 xxxx xxxx xxxx");
        no_ea(inst);
    }
    
    /* --- MMU (68851) instructions --- */
    
    {
        inst_t *inst = new_inst("mc68851_decode", "2", 1);
        add_range(inst, "1111 000 xxx xxxxxx");
        no_ea(inst);
    }
    
    /*{ // All other 68851 instructions
        inst_t *inst = new_inst("mc68851_op", "2", 1);
        add_range(inst, "1111 000 000 MMMMMM");
        ea_all(inst);
    }
    
    { // PScc
        inst_t *inst = new_inst("mc68851_pscc", "2", 1);
        add_range(inst, "1111 000 001 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // PDBcc
        inst_t *inst = new_inst("mc68851_pdbcc", "2", 1);
        add_range(inst, "1111 000 001 001 xxx");
        no_ea(inst);
    }
    
    { // PTRAPcc
        inst_t *inst = new_inst("mc68851_ptrapcc", "2", 1);
        add_range(inst, "1111 000 001 111 xxx");
        no_ea(inst);
    }
    
    { // pbcc
        inst_t *inst = new_inst("mc68851_pbcc", "2", 1);
        add_range(inst, "1111 000 01x xxxxxx");
        no_ea(inst);
    }
    
    { // psave
        inst_t *inst = new_inst("mc68851_psave", "2", 1);
        add_range(inst, "1111 000 100 MMMMMM");
        // Errata: 68kprm says "control" but means "control alterable"
        ea_control_alterable(inst);
        ea_add_mode(inst, EA_100);
    }
    
    { // prestore
        inst_t *inst = new_inst("mc68851_prestore", "2", 1);
        add_range(inst, "1111 000 101 MMMMMM");
        ea_control(inst);
        ea_add_mode(inst, EA_011);
    }*/
    
    /* --- FPU (68881) instructions --- */
    
    { // all other fpu ops
        inst_t *inst = new_inst("fpu_other", "2", 1);
        add_range(inst, "1111 001 000 MMMMMM");
        ea_all(inst);
    }
    
    { // FScc
        inst_t *inst = new_inst("fscc", "2", 1);
        add_range(inst, "1111 001 001 MMMMMM");
        ea_data_alterable(inst);
    }
    
    { // FDBcc
        inst_t *inst = new_inst("fdbcc", "2", 1);
        add_range(inst, "1111 001 001 001xxx");
        no_ea(inst);
    }
    
    { // FTRAPcc
        inst_t *inst = new_inst("ftrapcc", "2", 1);
        add_range(inst, "1111 001 001 111 010");
        add_range(inst, "1111 001 001 111 011");
        add_range(inst, "1111 001 001 111 100");
        no_ea(inst);
    }
    
    { // FBcc
        inst_t *inst = new_inst("fbcc", "2", 1);
        add_range(inst, "1111 001 01x xxxxxx");
        sub_range(inst, "1111 001 010 000000"); // fnop
        no_ea(inst);
    }
    
    { // fnop
        inst_t *inst = new_inst("fnop", "2", 1);
        add_range(inst, "1111 001 010 000000");
        no_ea(inst);
    }
    
    { // fsave
        inst_t *inst = new_inst("fsave", "2", 1);
        add_range(inst, "1111 001 100 MMMMMM");
        ea_control_alterable(inst);
        ea_add_mode(inst, EA_100);
    }
    
    { // frestore
        inst_t *inst = new_inst("frestore", "2", 1);
        add_range(inst, "1111 001 101 MMMMMM");
        ea_control(inst);
        ea_add_mode(inst, EA_011);
    }
    
}
 
    
    
    
    
    
    
    
    

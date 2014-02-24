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
#include <stdlib.h>
#include <assert.h>
#include <GLUT/glut.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include "../shoebill.h"
#include "../coff.h"

struct dbg_state_t dbg_state;

void print_mmu_rp(uint64_t rp)
{
    printf("lu=%u limit=0x%x sg=%u dt=%u addr=0x%08x\n", rp_lu(rp), rp_limit(rp), rp_sg(rp), rp_dt(rp), rp_addr(rp));
}

void printregs()
{
	printf("[d0]%08x  [d1]%08x  [d2]%08x  [d3]%08x\n", shoe.d[0], shoe.d[1], shoe.d[2], shoe.d[3]);
	printf("[d4]%08x  [d5]%08x  [d6]%08x  [d7]%08x\n", shoe.d[4], shoe.d[5], shoe.d[6], shoe.d[7]);
	printf("[a0]%08x  [a1]%08x  [a2]%08x  [a3]%08x\n", shoe.a[0], shoe.a[1], shoe.a[2], shoe.a[3]);
	printf("[a4]%08x  [a5]%08x  [a6]%08x  [a7]%08x\n", shoe.a[4], shoe.a[5], shoe.a[6], shoe.a[7]);
	printf("[pc]%08x  [sr]%c%c%c%c%c%c%c  [tc]%08x\n", shoe.pc,
           sr_s()?'S':'s',
           sr_m()?'M':'m',
           sr_x()?'X':'x',
           sr_n()?'N':'n',
           sr_z()?'Z':'z',
           sr_v()?'V':'v',
           sr_c()?'C':'c',
           shoe.tc
           );
    
    printf("[vbr]%08x\n", shoe.vbr);
    
    printf("srp: ");
    print_mmu_rp(shoe.srp);
    
    printf("crp: ");
    print_mmu_rp(shoe.crp);
    
    printf("tc: e=%u sre=%u fcl=%u ps=%u is=%u (tia=%u tib=%u tic=%u tid=%u)\n",
           tc_enable(), tc_sre(), tc_fcl(), tc_ps(), tc_is(), tc_tia(), tc_tib(), tc_tic(), tc_tid());
    
    printf("\n");
}

void print_pc()
{
    char str[1024];
    uint8_t binary[32];
    uint32_t i;
    uint32_t len;
    const char *name = NULL;
    
    if ((shoe.pc >= 0x40000000) && (shoe.pc < 0x50000000)) {
        uint32_t i, addr = shoe.pc % (shoe.physical_rom_size);
        for (i=0; macii_rom_symbols[i].name; i++) {
            if (macii_rom_symbols[i].addr > addr) {
                break;
            }
            name = macii_rom_symbols[i].name;
        }
    }
    else if (sr_s()) { // these symbols are only meaningful in supervisor mode
        coff_symbol *symb = coff_find_func(shoe.coff, shoe.pc);
        if (symb && strlen(symb->name))
            name = symb->name;
    }
    else {
        if ((shoe.pc >= 0x10000000) && (shoe.pc < 0x20000000)) {
            uint32_t i, addr = shoe.pc % (shoe.physical_rom_size);
            for (i=0; macii_rom_symbols[i].name; i++) {
                if (macii_rom_symbols[i].addr > addr) {
                    break;
                }
                name = macii_rom_symbols[i].name;
            }
        }
        else {
            coff_symbol *symb = coff_find_func(shoe.launch, shoe.pc);
            if (symb)
                name = symb->name;
        }
    }
    
    const uint16_t old_abort = shoe.abort;
    shoe.suppress_exceptions = 1;
    
    for (i=0; i<32; i++) {
        binary[i] = (uint8_t) lget(shoe.pc+i, 1);
    }
    
    disassemble_inst(binary, shoe.pc, str, &len);
    
    printf("*0x%08x %s [ ", shoe.pc, name ? name : "");
    for (i=0; i<len; i+=2) {
        printf("%02x%02x ", binary[i], binary[i+1]);
    }
    printf("]  %s\n", str);
    
    shoe.abort = old_abort;
    shoe.suppress_exceptions = 0;
    
}

static void dump_proc(uint32_t procnum)
{
    uint32_t u_proc_p;
    uint16_t pid;
    uint8_t do_print = 0, cpuflag;
    
    // Only dump this process state if we're in user mode
    if (sr_s())
        return ;
    
    shoe.suppress_exceptions = 1;
    cpuflag = lget(0x0000012f, 1);
    set_sr_s(1); // set supervisor mode so we can access the proc structure
    
    u_proc_p = lget(0x1ff01000, 4);
    if (shoe.abort)
        goto done;
    
    pid = lget(u_proc_p + 0x26, 2);
    if (shoe.abort)
        goto done;
    
    do_print = 1;
    
done:

    set_sr_s(0);
    shoe.abort = 0;
    shoe.suppress_exceptions = 0;

    if (do_print) {
        printf("pid = %u, cpuflag=0x%02x\n", pid, cpuflag);
        // print_pc();
        // printregs();
    }

}


void stepper()
{
    dbg_breakpoint_t *cur;
    uint32_t i;
    uint32_t len;
    
    check_time(); // FIXME: move this to another thread
    
    if (shoe.pending_interrupt) {
        process_pending_interrupt();
        shoe.stopped = 0;
    }
    else if (shoe.stopped) {
        // usleep(1);
        return ;
    }
    cpu_step();

    for (cur = dbg_state.breakpoints; cur != NULL; cur = cur->next) {
        if (shoe.pc == cur->addr) {
            printf("Hit breakpoint %llu *0x%08x\n", cur->num, shoe.pc);
            dbg_state.running = 0;
            return ;
        }
    }
    
    if (shoe.dbg) {
        // printf("*0x1ff01074 = %llx\n", lget(0x1ff01074, 4));
        print_pc();
        printregs();
    }
    
//    if ((shoe.pc >= 0x40000000) && (shoe.pc < 0x50000000)) {
//        //print_pc();
//        //printregs();
//    }
//    else {
//        coff_symbol *symb = coff_find_func(shoe.coff, shoe.pc);
//        if ((symb && (strcmp(symb->name, "scsitask")==0)) ||
//            /*(symb && (strcmp(symb->name, "scsiget")==0)) ||
//            (symb && (strcmp(symb->name, "scsi_out")==0)) ||
//            (symb && (strcmp(symb->name, "scsireq")==0)) ||
//            (symb && (strcmp(symb->name, "scsi_vio")==0)) ||
//            (symb && (strcmp(symb->name, "binit")==0)) ||*/
//            (symb && (strcmp(symb->name, "realvtopte")==0)) ||
//            (symb && (strcmp(symb->name, "realvtop")==0)) ||
//            (symb && (strcmp(symb->name, "realsvtop")==0)) || /*
//            (symb && (strcmp(symb->name, "sdcmd")==0)) ||
//            (symb && (strcmp(symb->name, "sdread")==0)) ||
//            (symb && (strcmp(symb->name, "gdpartinit")==0)) ||
//            (symb && (strcmp(symb->name, "scsi_in")==0)) ||
//            (symb && (strcmp(symb->name, "gddriveinit")==0)) ||
//            (symb && (strcmp(symb->name, "vio_init")==0)) ||
//            (symb && (strcmp(symb->name, "get_psr")==0)) ||
//            (symb && (strcmp(symb->name, "scsisched")==0)) ||
//            (symb && (strcmp(symb->name, "choosetask")==0)) || */
//            (symb && (strcmp(symb->name, "scsiselect")==0))) {
//            print_pc();
//            printregs();
//        }
//        else if ((shoe.pc > (0x0014ea02-32)) && (shoe.pc < (0x0014ea02+32))) {
//            print_pc();
//            printregs();
//        }
//    }
}


char *cli_prompt_callback(EditLine *el)
{
	return "~ ";
}

// Hack to clear line after ^C. el_reset() screws up tty when called from the signal handler.
void ch_reset(EditLine *el, int mclear);

void signal_callback(int sig)
{
    EditLine *el = dbg_state.el;
    (void) signal(SIGINT, signal_callback);
    (void) signal(SIGWINCH, signal_callback);
    
    switch (sig) {
        case SIGWINCH:
            el_resize(el);
            break ;
        case SIGINT:
            if (dbg_state.running) {
                dbg_state.running = 0;
            }
            else {
                printf("\n");
                ch_reset(el, 0);
                el_set(el, EL_REFRESH);
            }
            break ;
    }
    
	return ;
}

void verb_backtrace_handler (const char *line)
{
    const uint32_t old_abort = shoe.abort;
    shoe.suppress_exceptions = 1;
    shoe.abort = 0;
    
    // link
    //   push a6 to a7
    //   set a6 = a7
    //   set a7 = a7 - (some stack space)
    
    // jsr
    //   push return pointer to a7
    
    // call
    //   set a7 = a7 - (some stack space)
    //   push arguments to a7
    //   push return pointer to a7
    //   (jump to function)
    //   push 
    
    // bt algorithm
    //   set a7 = a6
    //   pop a7 -> a6
    //   pop a7 -> return pointer
    
    
    uint32_t i, j, a7, a6 = shoe.a[6];
    coff_symbol *symb;
    
    if (sr_s()) {
        symb = coff_find_func(shoe.coff, shoe.pc);
        printf("%u:  *0x%08x  %s+%u\n", 0, shoe.pc, (symb && strlen(symb->name))?symb->name:"?", shoe.pc - symb->value);
    }
    else
        printf("%u:  *0x%08x\n", 0, shoe.pc);
    
    for (i=1; 1; i++) {
        a7 = a6;
        const uint32_t last_a6 = lget(a7, 4);
        const uint32_t last_pc = lget(a7+4, 4);
        
        if ((last_a6 - a6) <= 1000) {
            printf("    {");
            for (j = a6+8; j < last_a6; j+=4) {
                uint32_t data = lget(j, 4);
                printf("%x, ", data);
            }
            printf("}\n");
        }
        
        if (sr_s()) {
            symb = coff_find_func(shoe.coff, last_pc);
            printf("%u:  *0x%08x  %s+%u\n", i, last_pc, (symb && strlen(symb->name))?symb->name:"?", last_pc - symb->value);
        }
        else
            printf("%u:  *0x%08x\n", i, last_pc);
    
        if ((last_a6 - a6) > 1000) {
            break;
        }
        
        a6 = last_a6;
    }
    
    shoe.suppress_exceptions = 0;
    shoe.abort = old_abort;
}

void verb_break_handler (const char *line)
{
    errno = 0;
    const uint32_t addr = (uint32_t) strtoul(line, NULL, 0);
    
    if (errno) {
        printf("errno: %d\n", errno);
        return ;
    }
    
    dbg_breakpoint_t *brk = calloc(sizeof(dbg_breakpoint_t), 1);
    brk->next = NULL;
    brk->addr = addr;
    brk->num = dbg_state.breakpoint_counter++;
    
    dbg_breakpoint_t **cur = &dbg_state.breakpoints;
    while (*cur)
        cur = &(*cur)->next;
    *cur = brk;
    
    printf("Set breakpoint %llu = *0x%08x\n", brk->num, brk->addr);
}

void verb_delete_handler (const char *line)
{
    errno = 0;
    uint64_t num = strtoull(line, NULL, 0);
    
    if (errno) {
        printf("errno: %d\n", errno);
        return ;
    }
    
    dbg_breakpoint_t **cur = &dbg_state.breakpoints;
    while (*cur) {
        if ((*cur)->num == num) {
            dbg_breakpoint_t *victim = *cur;
            *cur = (*cur)->next;
            free(victim);
            return ;
        }
        cur = &(*cur)->next;
    }
    
    printf("No such breakpoint (#%llu)\n", num);
}

void verb_help_handler (const char *line)
{
    printf("Help help help\n");
}

void verb_quit_handler (const char *line)
{
    printf("Quitting\n");
    fflush(stdout);
    exit(0);
}

void verb_continue_handler (const char *line)
{
    dbg_state.running = 1;
    while (dbg_state.running) {
        stepper();
    }
    print_pc();
}

void verb_stepi_handler (const char *line)
{
    shoe.stopped = 0;
    dbg_state.running = 1;
    cpu_step();
    dbg_state.running = 0;
    print_pc();
}

void verb_registers_handler (const char *line)
{
    printregs();
}

void verb_trace_toggle_handler (const char *line)
{
    shoe.dbg = !shoe.dbg;
}

void verb_examine_handler (const char *line)
{
    uint32_t addr = (uint32_t)strtoul(line, NULL, 0);
    uint32_t old_suppress = shoe.suppress_exceptions;
    
    shoe.suppress_exceptions = 1;
    printf("(uint32_t)*0x%08x = 0x%08x\n", addr, (uint32_t)lget(addr, 4));
    shoe.suppress_exceptions = old_suppress;
    
}

void verb_lookup_handler (const char *line)
{
    char *sym_name = malloc(strlen(line)+1);
    
    sscanf(line, "%s", sym_name);
    coff_symbol *symb = coff_find_symbol(shoe.coff, sym_name);
    
    free(sym_name);
    
    if (symb == NULL) {
        printf("Couldn't find \"%s\"\n", sym_name);
        return ;
    }
    
    printf("%s = *0x%08x\n", symb->name, symb->value);
}

struct verb_handler_table_t {
    const char *name;
    void (*func)(const char *);
} verb_handler_table[] =
{
    {"quit", verb_quit_handler},
    {"help", verb_help_handler},
    {"registers", verb_registers_handler},
    {"continue", verb_continue_handler},
    {"stepi", verb_stepi_handler},
    {"backtrace", verb_backtrace_handler},
    {"bt", verb_backtrace_handler},
    {"break", verb_break_handler},
    {"delete", verb_delete_handler},
    {"lookup", verb_lookup_handler},
    {"trace", verb_trace_toggle_handler},
    {"x", verb_examine_handler}
};

void execute_verb (const char *line)
{
    char verb[128];
	uint32_t max_len=0, max_i=0;
	const char *remainder;
    uint32_t i, matches = 0, match_i;
    
    if (sscanf(line, "%127s", verb) != 1)
        return ;
	
	// Skip past the verb
	for (remainder = line; *remainder && !isspace(*remainder); remainder++) ;
	
	// Skip past the space between the verb and the arguments
	for (; *remainder && isspace(*remainder); remainder++) ;

    const uint32_t verb_len = strlen(verb);
    for (i=0; i < (sizeof(verb_handler_table) / sizeof(struct verb_handler_table_t)); i++) {
        const uint32_t i_len = strlen(verb_handler_table[i].name);
        
        // If it's a perfect match,
        if (strcasecmp(verb, verb_handler_table[i].name)==0) {
            verb_handler_table[i].func(remainder);
            return ;
        }
        
        // Otherwise, see if it's a partial match
        if ((i_len >= verb_len) && strncasecmp(verb, verb_handler_table[i].name, verb_len)==0) {
            matches++;
            match_i = i;
        }
    }

    // Only execute the verb if it's an unambiguous match (matches == 1)
    if (matches == 1) {
        verb_handler_table[match_i].func(remainder);
        return ;
    }
	
    printf("  %s?\n", verb);
}

void *ui_thread (void *arg)
{
    EditLine *el;
    History *hist;
	HistEvent histev;
	
	const char *buf;
	int num;
	
	hist = history_init();
	history(hist, &histev, H_SETSIZE, 10000); // Remember 10000 previous user inputs
    
	el = el_init("Shoebill", stdin, stdout, stderr);
	dbg_state.el = el;
    
	el_set(el, EL_SIGNAL, 0);
	el_set(el, EL_PROMPT, cli_prompt_callback);
	el_set(el, EL_EDITOR, "emacs");
	el_set(el, EL_HIST, history, hist);
	
	(void) signal(SIGINT, signal_callback);
    (void) signal(SIGWINCH, signal_callback);
    
	while ((buf = el_gets(el, &num)) != NULL) {
		if (strcmp(buf, "\n")!=0) {
			execute_verb(buf);
			history(hist, &histev, H_ENTER, buf);
		}
	}
	
	el_end(el);
	history_end(hist);
	return NULL;
}

uint8_t *loadrom (char *path, uint32_t *len)
{
    FILE *f = fopen(path, "r");
    uint8_t *buf = calloc(1024, 1024);
    uint32_t i, sz, expected, checksum;
    
    if (!f) {
        printf("Couldn't open %s\n", path);
        return NULL;
    }
    
    for (sz=0; sz < (1024*1024); sz += (64*1024)) {
        printf("sz = %u\n", sz);
        if (fread(buf + sz, 64*1024, 1, f) != 1) {
            break;
        }
    }
    printf("sz = %u (done)\n", sz);
    
    fclose(f);
    
    if (sz == 0) {
        printf("loadrom: empty rom file\n");
        free(buf);
        return NULL;
    }
    
    for (i=0, expected=0; i<4; i++)
        expected = (expected << 8) + buf[i];
    
    for (i=4, checksum=0; i < sz; i+=2) {
        uint16_t word = (buf[i]<<8) + buf[i+1];
        checksum += word;
    }
    
    if (checksum != expected) {
        printf("Bad checksum (computed %08x, expected %08x)\n", checksum, expected);
        free(buf);
        return NULL;
    }
    
    *len = sz;
    return buf;
}

struct __attribute__ ((__packed__)) kernel_info {
    // Auto data
    uint32_t auto_magic;
    uint32_t auto_id[16];
    uint32_t auto_version[16];
    uint32_t auto_command;
    
    uint16_t root_ctrl;
    uint8_t root_drive;
    uint8_t root_cluster;

    struct __attribute__ ((__packed__)) sect_info {
        uint32_t vstart;
        uint32_t pstart;
        uint32_t size;
    } si[3];
    
    uint16_t machine_type; // Gestalt, I think? The "machine_type" for a quadra 950 doesn't match its gestalt, though.
    uint32_t drive_queue_offset;
    uint16_t ki_flags;
    uint8_t ki_version; // always 1
    uint8_t root_partition;
    uint16_t swap_ctrl;
    uint8_t swap_drive;
    uint8_t swap_partition;
};

void init_kernel_info()
{
    struct kernel_info ki, *p;
    uint32_t i;
    
    p = (struct kernel_info*)0x00003c00;
    
    shoe.d[0] = 0x536d7201;
    shoe.a[0] = (uint32_t)p;
    
    /* ----- Setup kernel info structure ----- */
    
    ki.auto_magic = 0x50696773; // 'Pigs' (Pigs in space?)
    
    for (i=0; i<16; i++) {
        ki.auto_id[i] = 0x0000ffff;
        ki.auto_version[i] = 0;
    }
    
    ki.auto_id[0xa] = 0x50; // Macintosh II video card has an auto_id of 5 (I guess?)
    ki.auto_id[0xb] = 0x5; // Macintosh II video card has an auto_id of 5 (I guess?)
    
    ki.auto_command = 0; // AUTO_RUN
    
    ki.root_ctrl = 0;
    ki.root_drive = 0;
    ki.root_cluster = 0;
    
    for (i = 0; i < shoe.coff->num_sections; i++) {
        coff_section *s = &shoe.coff->sections[i];
        uint8_t sect;
        
        if (strcmp(s->name, ".text") == 0)
            sect = 0;
        else if (strcmp(s->name, ".data") == 0)
            sect = 1;
        else if (strcmp(s->name, ".bss") == 0)
            sect = 2;
        else
            continue;
        
        ki.si[sect].vstart = s->v_addr;
        ki.si[sect].pstart = s->p_addr;
        ki.si[sect].size = s->sz;
    }

    ki.machine_type = 4; // Macintosh II?
    
    // +4 because the DrvQEl structure has a hidden "flags" field 4 bytes below the pointer
    ki.drive_queue_offset = sizeof(struct kernel_info) + 4;
    
    ki.ki_flags = 1; // KI_VERBOSE
    ki.ki_version = 1;
    ki.root_partition = 0;
    ki.swap_ctrl = 0;
    ki.swap_drive = 0;
    ki.swap_partition = 1;
    
    /* ----- Copy ki into memory ----- */
#define ki_pset(_f, _s) {pset((uint32_t)&p->_f, _s, ki._f); printf("Setting 0x%08x to %x\n", (uint32_t)&p->_f, ki._f);}
    ki_pset(auto_magic, 4);
    for (i=0; i<16; i++) {
        ki_pset(auto_id[i], 4);
        ki_pset(auto_version[i], 4);
    }
    ki_pset(auto_command, 4);
    
    ki_pset(root_ctrl, 2);
    ki_pset(root_drive, 1);
    ki_pset(root_cluster, 1);
    
    for (i=0; i<3; i++) {
        ki_pset(si[i].vstart, 4);
        ki_pset(si[i].pstart, 4);
        ki_pset(si[i].size, 4);
    }
    
    ki_pset(machine_type, 2);
    ki_pset(drive_queue_offset, 4);
    ki_pset(ki_flags, 2);
    ki_pset(ki_version, 1);
    ki_pset(root_partition, 1);
    ki_pset(swap_ctrl, 2);
    ki_pset(swap_drive, 1);
    ki_pset(swap_partition, 1);
    
    // Fake drive queue
    /*uint8_t dummy[0x14*2] = {
        // 00 08 01 00   00 00 00 14   00 01   00 08   FF D9   00 00   CE AD 00 2C
        0x00, 0x08, 0x01, 0x00,  0x00, 0x00, 0x00, 0x14,  0x00, 0x01,  0x00, 0x08,  0xFF, 0xD9,  0x00, 0x00,  0x0f, 0x71, 0x00, 0x01,
        0x00, 0x08, 0x01, 0x00,  0x00, 0x00, 0x00, 0x00,  0x00, 0x01,  0x00, 0x09,  0xFF, 0xDF,  0x00, 0x00,  0x10, 0x00, 0x00, 0x00
    };*/
    
    uint8_t dummy[0x14*2] = {
        // 00 08 01 00   00 00 00 14   00 01   00 08   FF D9   00 00   CE AD 00 2C
        0x00, 0x08, 0x00, 0x00,  0x00, 0x00, 0x00, 0x14,  0x00, 0x01,  0x00, 0x08,  0xFF, 0xD9,  0x00, 0x00,  0, 0, 0, 0,
        0x00, 0x08, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0x00, 0x01,  0x00, 0x09,  0xFF, 0xDF,  0x00, 0x00,  0, 0, 0, 0
    };
    
    for (i=0; i<(0x14*2); i++)
        pset(((uint32_t)p) + sizeof(struct kernel_info) + i, 1, dummy[i]);
    
    // FIXME: *really* implement drive queue
}

void init_macintosh_globals()
{
    uint8_t buf[0x1000];
    
    memset(buf, 0xBB, 0x1000);
    
    uint32_t i;
    for (i=0; i<0x1000; i++)
        pset(i, 1, buf[i]);
    
    
    #define hwCbSCSI (1<<15)
    #define hwCbClock (1<<14)
    #define hwCbExPRAM (1<<13)
    #define hwCbFPU (1<<12)
    #define hwCbMMU (1<<11)
    #define hwCbADB (1<<10)
    #define hwCbAUX (1<<9)
    // uint16_t HWCfgFlags = hwCbSCSI | hwCbClock | hwCbFPU | hwCbMMU | hwCbADB | hwCbAUX;
    
    uint16_t HWCfgFlags = hwCbSCSI | hwCbClock | hwCbFPU | hwCbMMU | hwCbADB;
    
    pset(0xb22, 2, HWCfgFlags); // HWCfgFlags
    
    pset(0x12f, 1, 0x02);       // CPUFlag = 0x02 (MC68020)
    pset(0x31a, 4, 0x00ffffff); // Lo3Bytes (always 0x00ffffff)
    
    pset(0x28e, 2, 0x3fff); // ROM85 (always 0x3fff, I think?)
    
    
    pset(0xdd8, 4, 0); // universal info ptr. is allowed to be null on Mac II, (I THINK)
    pset(0x1d4, 4, 0x50000000); // VIA (via1 ptr)
    pset(0x1d8, 4, 0x50004000); // SCC
    
    /*pset(0x8ce, 1, 1); // CrsrNew (set to 1?)
    pset(0x8cd, 1, 0); // CrsrBusy (0 -> Not presently busy)
    pset(0x8d0, 2, 0); // CrsrState
    pset(0x8cf, 1, 0xff); // CrsrCouple ??
    pset(0x8d6, 4, 0xffffffff); // MouseMask
    pset(0x8da, 4, 0); // MouseOffset
    pset(0x8d3, 1, 6); // CrsrScale
    */
    
    // 21e -> KbdType
    // dd8 -> UnivInfoPtr
    // d00 -> TimeDBRA
    // d02 -> TimeSCCDB
    // FIXME: add 0x21e, 0xdd8, 0x28e, 0xd00, 0xd02
    
    // Do this after setting lomem mac stuff, because this structure is more important.
    init_kernel_info();
}

uint32_t init_state (coff_file *coff, uint8_t *rom, uint32_t romsize)
{
    uint32_t i, j, pc = 0xffffffff;
    
    fpu_setup_jump_table();
    
    shoe.coff = coff;
    shoe.launch = coff_parser("priv/launch");
    
    shoe.physical_mem_size = 32 * 1024 * 1024;
    shoe.physical_rom_size = romsize;
    
    shoe.physical_mem_base = valloc(shoe.physical_mem_size);
    shoe.physical_rom_base = valloc(romsize);
    
    memset(shoe.physical_mem_base, 0, shoe.physical_mem_size);
    memcpy(shoe.physical_rom_base, rom, romsize);
    
    for (i=0; i<16; i++) {
        shoe.slots[i].slotnum = i;
        shoe.slots[i].connected = 0;
        shoe.slots[i].glut_window_id = -1;
    }
    
    // Install TFB at slot B
    /*{
        shoe.slots[0xb].connected = 1;
        shoe.slots[0xb].read_func = nubus_tfb_read_func;
        shoe.slots[0xb].write_func = nubus_tfb_write_func;
        nubus_tfb_init(0xb);
    }*/
    
    {
        shoe.slots[0xa].connected = 1;
        shoe.slots[0xa].read_func = nubus_video_read_func;
        shoe.slots[0xa].write_func = nubus_video_write_func;
        // nubus_video_init(0xa, 1440, 900);
        nubus_video_init(0xa, 800, 600);
    }
    
    
    // Initialize relevant Mac globals
    // (Do this before copying COFF segments, in case they overwrite these globals)
    init_macintosh_globals();
    
    /* Copy COFF segments into memory */
    for (i = 0; i < coff->num_sections; i++) {
        coff_section *s = &coff->sections[i];
        
        // Don't load a "copy" segment
        if (s->flags & coff_copy)
            continue;
        
        if ((s->flags & coff_text) || (s->flags & coff_data)) {
            /* copy text or data section */
            
            for (j = 0; j < s->sz; j++)
                pset(s->p_addr+j, 1, s->data[j]);
            
            if (strcmp(s->name, "pstart") == 0)
                pc = s->p_addr;
        }
        else if (s->flags & coff_bss) {
            /* Create an empty .bss segment */
            
            for (j = 0; j < s->sz; j++)
                pset(s->p_addr+j, 1, 0);
        }
    }
    
    if (pc == 0xffffffff) {
        printf("init_state: this unix doesn't contain a pstart segment\n");
        return 0;
    }
    
    set_sr(0x2000);
    shoe.pc = pc;
    
    // Start the VIA clocks
    gettimeofday(&shoe.start_time, NULL);
    shoe.total_ticks = 0;
    
    // Put the adb chip in state 3 (idle)
    shoe.adb.state = 3;
    pthread_mutex_init(&shoe.adb.lock, NULL);
    
    for (i=0; i<8; i++) {
        shoe.scsi_devices[i].scsi_id = i;
        shoe.scsi_devices[i].block_size = 0;
        shoe.scsi_devices[i].num_blocks = 0;
        shoe.scsi_devices[i].image_path = "dummy";
        shoe.scsi_devices[i].f = NULL;
    }
    
    // Hacky load scsi disk at id 0
    
    /*shoe.scsi_devices[0].scsi_id = 0;
    shoe.scsi_devices[0].block_size = 512;
    shoe.scsi_devices[0].num_blocks = 656544;
    shoe.scsi_devices[0].image_path = "priv/Apple_UNIX_3.iso";
    shoe.scsi_devices[0].f = fopen(shoe.scsi_devices[0].image_path, "r+");
    assert(shoe.scsi_devices[0].f);*/
    
    shoe.scsi_devices[0].scsi_id = 0;
    shoe.scsi_devices[0].block_size = 512;
    shoe.scsi_devices[0].num_blocks = 195912; //205561;
    shoe.scsi_devices[0].image_path = "priv/aux2.img";
    shoe.scsi_devices[0].f = fopen(shoe.scsi_devices[0].image_path, "r+");
    assert(shoe.scsi_devices[0].f);
    
    shoe.scsi_devices[1].scsi_id = 1;
    shoe.scsi_devices[1].block_size = 512;
    shoe.scsi_devices[1].num_blocks = 1048576;
    shoe.scsi_devices[1].image_path = "priv/blank.img";
    shoe.scsi_devices[1].f = fopen(shoe.scsi_devices[1].image_path, "r+");
    assert(shoe.scsi_devices[1].f);
    
    /*shoe.scsi_devices[6].scsi_id = 6;
    shoe.scsi_devices[6].block_size = 512;
    shoe.scsi_devices[6].num_blocks = 1280032; //205561;
    shoe.scsi_devices[6].image_path = "priv/macii_fs.img";
    shoe.scsi_devices[6].f = fopen(shoe.scsi_devices[6].image_path, "r+");
    assert(shoe.scsi_devices[6].f);*/
    
    /* In the future, we'll need to setup that booter struct for A/UX. */
    
    // HACK HACK HACK
    // Not sure why, but it seems that on non-RBV systems, A/UX version >= 1.1.1 does memcpy(0x0, 0x50000, 0x4000), blowing away all the lomem stuff
    // So copy lomem stuff to 0x50000
    
    for (i=0; i<0x4000; i++) {
        uint8_t c = pget(i, 1);
        pset(0x50000 + i, 1, c);
    }
    
    shoe.a[0] += 0x50000; // A/UX thinks kernel_info should live at 0x50000 + (normal kernel_info addr).
    
    return 1;
}


#define KEY_SHIFT 1

const struct {
    uint8_t code;
    char c;
    uint8_t modifiers;
} key_codes[] = {
    {0x0, 'A', 0},
    {0x1, 'S', 0},
    {2, 'D', 0},
    {3, 'F', 0},
    {4, 'H', 0},
    {5, 'G', 0},
    {6, 'Z', 0},
    {7, 'X', 0},
    {8, 'C', 0},
    {9, 'V', 0},
    // {0xa ??
    {0xb, 'B', 0},
    {0xc, 'Q', 0},
    {0xd, 'W', 0},
    {0xe, 'E', 0},
    {0xf, 'R', 0},
    {0x10, 'Y', 0},
    {0x11, 'T', 0},
    
    {0x12, '1', 0},
    {0x12, '!', KEY_SHIFT},
    
    
    {0x13, '2', 0},
    {0x13, '@', KEY_SHIFT},
    
    {0x14, '3', 0},
    {0x14, '#', KEY_SHIFT},
    
    {0x15, '4', 0},
    {0x15, '$', KEY_SHIFT},
    
    {0x16, '6', 0},
    {0x16, '^', KEY_SHIFT},
    
    {0x17, '5', 0},
    {0x17, '%', KEY_SHIFT},
    
    {0x18, '=', 0},
    {0x18, '+', KEY_SHIFT},
    
    {0x19, '9', 0},
    {0x19, '(', KEY_SHIFT},
    
    {0x1a, '7', 0},
    {0x1a, '&', KEY_SHIFT},
    
    {0x1b, '-', 0},
    {0x1b, '_', KEY_SHIFT},
    
    {0x1c, '8', 0},
    {0x1c, '*', KEY_SHIFT},
    
    {0x1d, '0', 0},
    {0x1d, ')', KEY_SHIFT},
    
    {0x1e, ']', 0},
    {0x1e, '}', KEY_SHIFT},
    
    {0x1f, 'O', 0},
    {0x20, 'U', 0},
    
    {0x21, '[', 0},
    {0x21, '{', KEY_SHIFT},
    
    {0x22, 'I', 0},
    {0x23, 'P', 0},
    
    {0x24, '\n', 0},
    {0x24, '\r', 0},
    
    {0x25, 'L', 0},
    {0x26, 'J', 0},
    
    {0x27, '"', KEY_SHIFT},
    {0x27, '\'', 0},
    
    {0x28, 'K', 0},
    
    {0x29, ';', 0},
    {0x29, ':', KEY_SHIFT},
    
    {0x2a, '\\', 0},
    {0x2a, '|', KEY_SHIFT},
    
    {0x2b, ',', 0},
    {0x2b, '<', KEY_SHIFT},
    
    {0x2c, '/', 0},
    {0x2c, '?', 0},
    
    {0x2d, 'N', 0},
    {0x2e, 'M', 0},
    
    {0x2f, '.', 0},
    {0x2f, '>', KEY_SHIFT},
    
    {0x30, '\t', 0},
    {0x31, ' ', 0},
    
    {0x32, '`', 0},
    {0x32, '~', KEY_SHIFT},
    
    {0x33, '\b', 0},
    {0x33, 0x7f, 0},
    // {0x34, ??
    // {0x35 // escape char
    // 0x36 // ctrl
    // 0x37 // command
    // 0x38 // shift
    // 0x39 // caps lock
    // 0x3a // option
    // 0x3b // left arrow
    // 0x3c // right arrow
    // 0x3d // down arrow
    // 0x3e // up arrow
    
    {0, 0, 0},
};

static uint8_t lookup_key(char c)
{
    uint32_t i;
    uint8_t upper=toupper(c);
    
    for (i=0; key_codes[i].c; i++) {
        if (key_codes[i].c == upper)
            return key_codes[i].code;
        
    }
    
    return 0xff;
}

static uint8_t lookup_special(int special)
{
    switch (special) {
        case GLUT_KEY_UP: return 0x3e;
        case GLUT_KEY_DOWN: return 0x3d;
        case GLUT_KEY_LEFT: return 0x3b;
        case GLUT_KEY_RIGHT: return 0x3c;
        default: return 0xff;
    }
}

static void keyboard_add_entry(uint8_t code, uint8_t up)
{
    uint8_t up_mask = up ? 0x80 : 0;
    uint32_t i;
    int modifiers = glutGetModifiers();
    
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    if ((shoe.key.key_i+1) < KEYBOARD_STATE_MAX_KEYS) {
        if (modifiers & GLUT_ACTIVE_SHIFT) {
            shoe.key.keys[shoe.key.key_i].code_a = 0x38;
            shoe.key.keys[shoe.key.key_i].code_b = 0xff;
            shoe.key.key_i++;
        }
        else if (shoe.key.down_modifiers & GLUT_ACTIVE_SHIFT) {
            shoe.key.keys[shoe.key.key_i].code_a = 0x80 | 0x38;
            shoe.key.keys[shoe.key.key_i].code_b = 0xff;
            shoe.key.key_i++;
        }
        shoe.key.keys[shoe.key.key_i].code_a = code | up_mask;
        shoe.key.keys[shoe.key.key_i].code_b = 0xff;
        shoe.key.key_i++;
    }
    
    shoe.key.down_modifiers = modifiers;
    
    adb_request_service_request(2);

    pthread_mutex_unlock(&shoe.adb.lock);
}

void global_mouse_func (int button, int state, int x, int y)
{
    //if (button != GLUT_LEFT_BUTTON)
        // return ;
    
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    shoe.mouse.button_down = (state == GLUT_DOWN);
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);

    pthread_mutex_unlock(&shoe.adb.lock);
    
    // printf("mouse_func: setting service request\n");
}

static void move_mouse (int x, int y, uint8_t button_down)
{
    printf("%s: lock\n", __func__); fflush(stdout);
    assert(pthread_mutex_lock(&shoe.adb.lock) == 0);
    
    int32_t delta_x = x - shoe.mouse.old_x;
    int32_t delta_y = y - shoe.mouse.old_y;
    
    shoe.mouse.old_x = x;
    shoe.mouse.old_y = y;
    
    shoe.mouse.delta_x += delta_x;
    shoe.mouse.delta_y += delta_y;
    shoe.mouse.button_down = button_down;
    shoe.mouse.changed = 1;
    
    adb_request_service_request(3);
    printf("%s: unlock\n", __func__); fflush(stdout);
    pthread_mutex_unlock(&shoe.adb.lock);
    
    // printf("move_mouse: setting service request\n");
}

void global_motion_func (int x, int y)
{
    move_mouse(x, y, 1);
}

void global_passive_motion_func (int x, int y)
{
    move_mouse(x, y, 0);
}

void global_keyboard_up_func (unsigned char c, int x, int y)
{
    uint8_t code = lookup_key(c);
    if (code != 0xff)
        keyboard_add_entry(code, 1);
}

void global_keyboard_down_func (unsigned char c, int x, int y)
{
    uint8_t code = lookup_key(c);
    if (code != 0xff)
        keyboard_add_entry(code, 0);
}

void global_special_up_func (int special, int x, int y)
{
    const uint8_t code = lookup_special(special);
    if (code != 0xff)
        keyboard_add_entry(code, 1);
}

void global_special_down_func (int special, int x, int y)
{
    const uint8_t code = lookup_special(special);
    if (code != 0xff)
        keyboard_add_entry(code, 0);
}

void timer_func (int arg)
{
    glutTimerFunc(15, timer_func, 0); // 66.67hz is the usual refresh interval (right?)
    
    uint32_t slotnum;
    for (slotnum = 0; slotnum < 16; slotnum++) {
        if (shoe.slots[slotnum].glut_window_id == -1)
            continue;
        
        glutSetWindow(shoe.slots[slotnum].glut_window_id);
        glutPostRedisplay();
    }
}

//void _macii_load_video_rom(const char *path);
int main (int argc, char **argv)
{
    coff_file *coff;
    uint32_t romsize;
    uint8_t *rom;
    pthread_t pid;
    
    //_macii_load_video_rom("tfb_rom");
    
    rom = loadrom("priv/macii.rom", &romsize);
    if (rom == NULL)
        return 0;
    //printf(")
    
    /*if (romsize != (256 * 1024)) {
        printf("Bogus rom size (%u bytes)\n", romsize);
        return 0;
    }*/
    
    coff = coff_parser("priv/unix");
    if (coff == NULL) {
        printf("main: coff_parser() failed to load \"unix\"\n");
        return 0;
    }
    
    int dummyargc = 1;
    glutInit(&dummyargc, argv);
    
    if (!init_state(coff, rom, romsize))
        return 0;
    free(rom);
    
    pthread_create(&pid, NULL, ui_thread, NULL);
    
    glutTimerFunc(15, timer_func, 0);
    glutMainLoop();
    while (1) sleep(1);
    return 0;
}


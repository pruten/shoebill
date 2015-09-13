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
#include <histedit.h>
#include "../core/shoebill.h"

rb_tree *keymap;

struct dbg_state_t {
    EditLine *el;
    uint8_t running;
    uint64_t breakpoint_counter;
    dbg_breakpoint_t *breakpoints;
    _Bool trace;
    uint32_t slow_factor;
    
    char *ring;
    uint32_t ring_i, ring_len;
    
    uint64_t op_count[0x10000];

};

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
           _tc_enable(), _tc_sre(), tc_fcl(), _tc_ps(), _tc_is(), tc_tia(), tc_tib(), tc_tic(), tc_tid());
    
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
            name = "";
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


void verb_stepi_handler (const char *line)
{
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
    dbg_state.trace = !dbg_state.trace;
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


void stepper()
{
    dbg_breakpoint_t *cur;
    
    if (shoe.cpu_thread_notifications) {
        
        // If there's an interrupt pending
        if (shoe.cpu_thread_notifications & 0xff) {
            // process_pending_interrupt() may clear SHOEBILL_STATE_STOPPED
            process_pending_interrupt();
        }
        
        if (shoe.cpu_thread_notifications & SHOEBILL_STATE_STOPPED) {
            // I think it's safe to ignore STOP instructions...
        }
    }
    
    cpu_step();
    
    
    
    if (dbg_state.trace) {
        print_pc();
        printregs();
    }
    
    for (cur = dbg_state.breakpoints; cur != NULL; cur = cur->next) {
        if (shoe.pc == cur->addr) {
            printf("Hit breakpoint %llu *0x%08x\n", cur->num, shoe.pc);
            dbg_state.running = 0;
            return ;
        }
    }
}

void verb_continue_handler (const char *line)
{
    dbg_state.running = 1;
    while (dbg_state.running) {
        if (dbg_state.slow_factor)
            usleep(dbg_state.slow_factor);
        stepper();
    }
    print_pc();
}

void verb_quit_handler (const char *line)
{
    printf("Quitting\n");
    fflush(stdout);
    exit(0);
}

void verb_reset_handler (const char *line)
{
    p_free_pool(shoe.pool);
    shoe.pool = NULL;
}

void verb_slow_handler (const char *line)
{
    const uint64_t usecs = strtoul(line, NULL, 0);
    printf("Slow factor %u -> %u\n", dbg_state.slow_factor, (uint32_t)usecs);
    dbg_state.slow_factor = usecs;
}

struct verb_handler_table_t {
    const char *name;
    void (*func)(const char *);
} verb_handler_table[] =
{
    {"quit", verb_quit_handler},
    {"continue", verb_continue_handler},
    {"help", verb_help_handler},
    {"registers", verb_registers_handler},
    {"stepi", verb_stepi_handler},
    {"backtrace", verb_backtrace_handler},
    {"bt", verb_backtrace_handler},
    {"break", verb_break_handler},
    {"delete", verb_delete_handler},
    {"lookup", verb_lookup_handler},
    {"trace", verb_trace_toggle_handler},
    {"x", verb_examine_handler},
    {"reset", verb_reset_handler},
    {"slow", verb_slow_handler},
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
	for (remainder = line; *remainder && !isspace(*remainder); remainder++)
        ;
	
	// Skip past the space between the verb and the arguments
	for (; *remainder && isspace(*remainder); remainder++)
        ;

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

static uint32_t _get_modifiers (void)
{
    int glut_modifiers = glutGetModifiers();
    uint32_t result = 0;
    
    result |= (glut_modifiers & GLUT_ACTIVE_SHIFT) ? (1 << 17) : 0;
    result |= (glut_modifiers & GLUT_ACTIVE_CTRL) ? (1 << 18) : 0;
    result |= (glut_modifiers & GLUT_ACTIVE_ALT) ? (1 << 19) : 0;
    
    return result;
}

void global_mouse_func (int button, int state, int x, int y)
{
    shoebill_mouse_click(state == GLUT_DOWN);
    shoebill_mouse_move(x, y);
}

void global_motion_func (int x, int y)
{
    shoebill_mouse_click(1);
    shoebill_mouse_move(x, y);
}

void global_passive_motion_func (int x, int y)
{
    shoebill_mouse_click(0);
    shoebill_mouse_move(x, y);
}

void global_keyboard_up_func (unsigned char c, int x, int y)
{
    uint16_t value;
    if (rb_find(keymap, c, &value)) {
        shoebill_key_modifier((value >> 8) | (_get_modifiers() >> 16));
        shoebill_key(0, value & 0xff);
    }
}

void global_keyboard_down_func (unsigned char c, int x, int y)
{
    uint16_t value;
    if (rb_find(keymap, c, &value)) {
        shoebill_key_modifier((value >> 8) | (_get_modifiers() >> 16));
        shoebill_key(1, value & 0xff);
    }
}

void global_special_up_func (int special, int x, int y)
{
    const uint8_t code = lookup_special(special);
    if (code != 0xff) {
        shoebill_key_modifier(_get_modifiers() >> 16);
        shoebill_key(0, code);
    }
}

void global_special_down_func (int special, int x, int y)
{
    const uint8_t code = lookup_special(special);
    if (code != 0xff) {
        shoebill_key_modifier(_get_modifiers() >> 16);
        shoebill_key(1, code);
    }
}

void timer_func (int arg)
{
    glutTimerFunc(15, timer_func, 0); // 15ms = 66.67hz
    glutPostRedisplay();
}


void _display_func (void)
{
    shoebill_video_frame_info_t frame = shoebill_get_video_frame(9, 0);
    
    shoebill_send_vbl_interrupt(9);
    
    glDrawBuffer(GL_BACK);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glClearColor(0.0, 0.0, 0.0, 0.0);
    
    glViewport(0, 0, frame.width, frame.height);
    glRasterPos2i(0, frame.height);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    glPixelZoom(1.0, -1.0);
    
    glDrawPixels(frame.width,
                 frame.height,
                 GL_RGBA,
                 GL_UNSIGNED_BYTE,
                 frame.buf);
    
    glutSwapBuffers();
}

static void _init_keyboard_map (void)
{
    #define mapkeymod(u, a, m) do { \
        assert((a >> 7) == 0); \
        uint16_t value = ((m) << 8)| (a); \
        rb_insert(keymap, u, &value, NULL); \
    } while (0)
    
    #define mapkey(_u, a) mapkeymod(_u, a, 0)

    keymap = rb_new(p_new_pool(NULL), sizeof(uint16_t));
    
    // Letters
    mapkey('a', 0x00);
    mapkey('b', 0x0b);
    mapkey('c', 0x08);
    mapkey('d', 0x02);
    mapkey('e', 0x0e);
    mapkey('f', 0x03);
    mapkey('g', 0x05);
    mapkey('h', 0x04);
    mapkey('i', 0x22);
    mapkey('j', 0x26);
    mapkey('k', 0x28);
    mapkey('l', 0x25);
    mapkey('m', 0x2e);
    mapkey('n', 0x2d);
    mapkey('o', 0x1f);
    mapkey('p', 0x23);
    mapkey('q', 0x0c);
    mapkey('r', 0x0f);
    mapkey('s', 0x01);
    mapkey('t', 0x11);
    mapkey('u', 0x20);
    mapkey('v', 0x09);
    mapkey('w', 0x0d);
    mapkey('x', 0x07);
    mapkey('y', 0x10);
    mapkey('z', 0x06);
    
    // Numbers
    mapkey('0', 0x1d);
    mapkey('1', 0x12);
    mapkey('2', 0x13);
    mapkey('3', 0x14);
    mapkey('4', 0x15);
    mapkey('5', 0x17);
    mapkey('6', 0x16);
    mapkey('7', 0x1a);
    mapkey('8', 0x1c);
    mapkey('9', 0x19);
    
    // Top row symbols
    mapkeymod(')', 0x1d, modShift);
    mapkeymod('!', 0x12, modShift);
    mapkeymod('@', 0x13, modShift);
    mapkeymod('#', 0x14, modShift);
    mapkeymod('$', 0x15, modShift);
    mapkeymod('%', 0x17, modShift);
    mapkeymod('^', 0x16, modShift);
    mapkeymod('&', 0x1a, modShift);
    mapkeymod('*', 0x1c, modShift);
    mapkeymod('(', 0x19, modShift);
    
    // Other symbols (no shift)
    mapkeymod('`', 0x32, 0);
    mapkeymod('-', 0x1b, 0);
    mapkeymod('=', 0x18, 0);
    mapkeymod('[', 0x21, 0);
    mapkeymod(']', 0x1e, 0);
    mapkeymod('\\', 0x2a, 0);
    mapkeymod(';', 0x29, 0);
    mapkeymod('\'', 0x27, 0);
    mapkeymod(',', 0x2b, 0);
    mapkeymod('.', 0x2f, 0);
    mapkeymod('/', 0x2c, 0);
    
    // Other symbols (with shift)
    mapkeymod('~', 0x32, modShift);
    mapkeymod('_', 0x1b, modShift);
    mapkeymod('+', 0x18, modShift);
    mapkeymod('{', 0x21, modShift);
    mapkeymod('}', 0x1e, modShift);
    mapkeymod('|', 0x2a, modShift);
    mapkeymod(':', 0x29, modShift);
    mapkeymod('"', 0x27, modShift);
    mapkeymod('<', 0x2b, modShift);
    mapkeymod('>', 0x2f, modShift);
    mapkeymod('?', 0x2c, modShift);
    
    // Function keys
    /*mapkey(NSF1FunctionKey, 0x7a);
    mapkey(NSF2FunctionKey, 0x78);
    mapkey(NSF3FunctionKey, 0x63);
    mapkey(NSF4FunctionKey, 0x76);
    mapkey(NSF5FunctionKey, 0x60);
    mapkey(NSF6FunctionKey, 0x61);
    mapkey(NSF7FunctionKey, 0x62);
    mapkey(NSF8FunctionKey, 0x64);
    mapkey(NSF9FunctionKey, 0x65);
    mapkey(NSF10FunctionKey, 0x6d);
    mapkey(NSF11FunctionKey, 0x67);
    mapkey(NSF12FunctionKey, 0x6f);
    mapkey(NSF13FunctionKey, 0x69);
    mapkey(NSF14FunctionKey, 0x6b);
    mapkey(NSF15FunctionKey, 0x71);*/
    
    // Arrows
    /*mapkey(NSUpArrowFunctionKey, 0x3e);
    mapkey(NSDownArrowFunctionKey, 0x3d);
    mapkey(NSRightArrowFunctionKey, 0x3c);
    mapkey(NSLeftArrowFunctionKey, 0x3b);*/
    
    // Delete
    //mapkey(NSDeleteFunctionKey, 0x75);
    mapkey(0x08, 0x33);
    mapkey(0x7f, 0x33);
    
    // Enter, NL, CR
    mapkey('\r', 0x24);
    mapkey('\n', 0x24);
    mapkey(0x03, 0x24);
    
    // Other keys
    mapkey(0x1b, 0x35); // escape
    mapkey(' ', 0x31); // space
    mapkey('\t', 0x30); // tab
}

static void _init_glut_video (void)
{
    shoebill_video_frame_info_t frame = shoebill_get_video_frame(9, 1);
    
    glutInitWindowSize(frame.width, frame.height);
    glutCreateWindow("Shoebill");
    glutDisplayFunc(_display_func);
    glutIgnoreKeyRepeat(1);
    
    
    glutKeyboardFunc(global_keyboard_down_func);
    glutKeyboardUpFunc(global_keyboard_up_func);
    
    glutSpecialFunc(global_special_down_func);
    glutSpecialUpFunc(global_special_up_func);
    
    glutMouseFunc(global_mouse_func);
    glutMotionFunc(global_motion_func);
    glutPassiveMotionFunc(global_passive_motion_func);
    
    glutInitDisplayMode (GLUT_DOUBLE);
    
    glShadeModel(GL_FLAT);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearColor(0.1, 1.0, 0.1, 1.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, frame.width, 0, frame.height, -1.0, 1.0);
    
    glViewport(0, 0,  frame.width, frame.height);
    
}

int main (int argc, char **argv)
{
    shoebill_config_t config;
    pthread_t pid;
    
    
    bzero(&config, sizeof(shoebill_config_t));
    
    /*
     * A variety of hacky things happen in debug mode.
     * shoebill_start() will not create a new thread to run
     * the CPU loop. We'll create a CPU thread here, bypass
     * core_api, and directly manipulate the emulator guts.
     *
     * This is not a great example of how to write a GUI 
     * for shoebill...
     */
    config.debug_mode = 1;
     
    config.aux_verbose = 0;
    config.ram_size = 16 * 1024 * 1024;
    config.aux_kernel_path = "/unix";
    config.rom_path = "../../../shoebill_priv/macii.rom";
    

    config.scsi_devices[0].path = "../../../shoebill_priv/root3.img";
    //config.scsi_devices[1].path = "../priv/marathon.img";
    
    /*dbg_state.ring_len = 256 * 1024 * 1024;
    dbg_state.ring = malloc(dbg_state.ring_len);
    dbg_state.ring_i = 0;*/
    
    shoebill_validate_or_zap_pram(config.pram, 1);
    
    if (!shoebill_initialize(&config)) {
        printf("%s\n", config.error_msg);
        return 0;
    }
    
    _init_keyboard_map();
    
    shoebill_install_video_card(&config,
                                9, // slotnum
                                640, // 1024,
                                480); // 768,
    
    // uint8_t ethernet_addr[6] = {0x22, 0x33, 0x55, 0x77, 0xbb, 0xdd};
    // shoebill_install_ethernet_card(&config, 13, ethernet_addr);
    
    // Start the VIA timer thread
    shoebill_start();
    
    // Create a new thread to drive the CPU & debugger UI
    pthread_create(&pid, NULL, ui_thread, NULL);
    
    int dummyargc = 1;
    glutInit(&dummyargc, argv);
    
    // Create/configure the screen
    _init_glut_video();
    
    // Set a GLUT timer to update the screen
    glutTimerFunc(15, timer_func, 0);
    
    glutMainLoop();
    
    return 0;
}



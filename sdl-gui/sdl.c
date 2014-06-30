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
#include <assert.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include "../core/shoebill.h"

static void _print_vers(void)
{
    printf("Shoebill v0.0.4 - http://github.com/pruten/shoebill - Peter Rutenbar (c) 2014\n\n");
}

rb_tree *keymap;
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
    mapkey(SDLK_F1, 0x7a);
    mapkey(SDLK_F2, 0x78);
    mapkey(SDLK_F3, 0x63);
    mapkey(SDLK_F4, 0x76);
    mapkey(SDLK_F5, 0x60);
    mapkey(SDLK_F6, 0x61);
    mapkey(SDLK_F7, 0x62);
    mapkey(SDLK_F8, 0x64);
    mapkey(SDLK_F9, 0x65);
    mapkey(SDLK_F10, 0x6d);
    mapkey(SDLK_F11, 0x67);
    mapkey(SDLK_F12, 0x6f);
    mapkey(SDLK_F13, 0x69);
    mapkey(SDLK_F14, 0x6b);
    mapkey(SDLK_F15, 0x71);
    
    // Arrows
    mapkey(SDLK_UP, 0x3e);
    mapkey(SDLK_DOWN, 0x3d);
    mapkey(SDLK_RIGHT, 0x3c);
    mapkey(SDLK_LEFT, 0x3b);
    
    // Delete
    mapkey(SDLK_DELETE, 0x75);
    mapkey(SDLK_BACKSPACE, 0x33);
    mapkey(SDLK_BACKSPACE, 0x33);
    
    // Enter, NL, CR
    mapkey(SDLK_RETURN2, 0x24);
    mapkey(SDLK_RETURN, 0x24);
    // mapkey(0x03, 0x24);
    
    // Other keys
    mapkey(SDLK_ESCAPE, 0x35); // escape
    mapkey(SDLK_SPACE, 0x31); // space
    mapkey(SDLK_TAB, 0x30); // tab
}

static void _display_frame (SDL_Window *win)
{
    shoebill_video_frame_info_t frame = shoebill_get_video_frame(9, 0);
    
    shoebill_send_vbl_interrupt(9);
    
    glDrawBuffer(GL_BACK);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glClearColor(0, 0, 0, 1.0);
    
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
    
    SDL_GL_SwapWindow(win);
}

struct shoe_app_pram_data_t {
    uint8_t pram[256];
    FILE *f;
    pthread_t threadid;
    volatile _Bool updated, tear_down_thread;
};

struct {
    const char *scsi_path[8];
    const char *rom_path;
    const char *relative_unix_path;
    const char *pram_path;
    
    uint32_t height, width;
    uint32_t ram_megabytes;
    _Bool verbose, use_tfb;
    
    struct shoe_app_pram_data_t pram_data;
} user_params;

#define equals_arg(name) keylen = strlen(name); value = argv[i]+keylen; if (strncmp((name), argv[i], keylen) == 0)

#if !((defined WIN32) || (defined _WIN64))
#include <sys/types.h>
#include <pwd.h>
#include <uuid/uuid.h>
#endif

static char* _get_home_dir (const char *terminal_element)
{
    char *result = NULL;
    
#if (defined WIN32) || (defined _WIN64)
    if (getenv("USERPROFILE") != NULL) {
        result = malloc(strlen(getenv("USERPROFILE")) + strlen(terminal_element) + 32);
        sprintf(result, "%s\\%s", getenv("USERPROFILE"), terminal_element);
        goto done;
    }
    
    if (getenv("HOMEDRIVE") && getenv("HOMEPATH")) {
        result = malloc(strlen(getenv("HOMEDRIVE")) + strlen(getenv("HOMEPATH")) + strlen(terminal_element) + 32);
        sprintf(result, "%s\\%s\\%s", getenv("HOMEDRIVE"), getenv("HOMEPATH"), terminal_element);
        goto done;
    }
    
#else
    
    if (getenv("HOME") != NULL)  {
        result = malloc(strlen(getenv("HOME")) + strlen(terminal_element) + 32);
        sprintf(result, "%s/%s", getenv("HOME"), terminal_element);
        goto done;
    }
    
    struct passwd *pwd = getpwuid(getuid());
    if (pwd) {
        result = malloc(strlen(pwd->pw_dir) + strlen(terminal_element) + 32);
        sprintf(result, "%s/%s", pwd->pw_dir, terminal_element);
        goto done;
    }
    
#endif
    
done:
    
    // printf("_get_home_dir: debug: %s\n", result);
    
    return result;
}

static void _print_help (void)
{
    printf("Arguments have the form name=value.\n");
    printf("\n");
    printf("rom=<path to Mac II ROM>\n");
    printf("Specifies the path to a Macintosh II ROM.\n");
    printf("E.g. rom=/home/foo/macii.rom\n");
    printf("\n");
    printf("disk0..disk6=<path to disk image>\n");
    printf("Specifies the path to a disk image for the given SCSI ID. Shoebill will always boot from disk0, so make sure disk0 points to a bootable A/UX image.\n");
    printf("E.g. disk0=/home/foo/aux3.img disk1=/blah.img\n");
    printf("\n");
    printf("ram=<megabytes of memory>\n");
    printf("E.g. ram=16\n");
    printf("\n");
    printf("height=<num pixels>\n");
    printf("Specifies the height of the screen in pixels.\n");
    printf("\n");
    printf("width=<num pixels>\n");
    printf("Specifies the width of the screen in pixels.\n");
    printf("\n");
    printf("pram-path=<path to PRAM file>\n");
    printf("Defaults to ~/.shoebill_pram\n");
    printf("\n");
    printf("verbose=<1 or 0>\n");
    printf("Whether to boot A/UX in verbose mode. Best to leave it at default (1).\n");
    printf("\n");
    printf("unix-path=<path to kernel on disk0>\n");
    printf("Path to the kernel file on the root disk image. Best to leave it at default (/unix).\n");
    printf("\n");
    printf("\n");
    printf("Examples:\n");
    printf("\n");
    printf("shoebill.exe disk0=C:\\aux3.img rom=C:\\macii.rom width=1024 height=768 ram=64\n");
    printf("\n");
    printf("./shoebill disk0=/aux3.img rom=/macii.rom width=1024 height=768 ram=64\n");
    printf("\n");
}

static void _init_user_params (int argc, char **argv)
{
    char *key;
    uint32_t i;
    for (i=0; i<8; i++)
        user_params.scsi_path[i] = NULL;
    
    user_params.rom_path = "macii.rom";
    user_params.relative_unix_path = "/unix";
    
    user_params.height = 640;
    user_params.width = 800;
    user_params.ram_megabytes = 16;
    user_params.verbose = 1;
    user_params.use_tfb = 0;
    
    user_params.pram_path = _get_home_dir(".shoebill_pram");
    
    if (argc < 2) {
        _print_help();
        exit(0);
    }
    
    for (i=1; i<argc; i++) {
        key = "-h";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            _print_help();
            exit(0);
        }
        
        key = "help";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            _print_help();
            exit(0);
        }
        
        key = "toby"; // Whether to use the "toby frame buffer" card, instead of the regular shoebill video card
        if(strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.use_tfb = 1;
            continue;
        }
        
        key = "ram=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.ram_megabytes = strtoul(argv[i]+strlen(key), NULL, 10);
            continue;
        }
        
        key = "height=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.height = strtoul(argv[i]+strlen(key), NULL, 10);
            continue;
        }
        
        key = "width=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.width = strtoul(argv[i]+strlen(key), NULL, 10);
            continue;
        }
        
        key = "verbose=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.verbose = strtoul(argv[i]+strlen(key), NULL, 10);
            continue;
        }
        
        key = "rom=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.rom_path = argv[i] + strlen(key);
            continue;
        }
        
        key = "unix-path=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.relative_unix_path = argv[i] + strlen(key);
            continue;
        }
        
        key = "pram-path=";
        if (strncmp(key, argv[i], strlen(key)) == 0) {
            user_params.pram_path = argv[i] + strlen(key);
            continue;
        }
        
        if ((strncmp("disk", argv[i], 4) == 0) && (isdigit(argv[i][4])) && (argv[i][5] == '=')) {
            uint8_t scsi_num = argv[i][4] - '0';
            if (scsi_num < 7) {
                user_params.scsi_path[scsi_num] = &argv[i][6];
                continue;
            }
        }
    }
    
}

void _pram_callback (void *param, const uint8_t addr, const uint8_t byte)
{
    struct shoe_app_pram_data_t *pram_data = (struct shoe_app_pram_data_t*)param;
    pram_data->pram[addr] = byte;
    pram_data->updated = 1;
}


void* _pram_writer_thread (void *param)
{
    struct shoe_app_pram_data_t *pram_data = (struct shoe_app_pram_data_t*)param;
    while (!pram_data->tear_down_thread) {
        if (pram_data->updated) {
            pram_data->updated = 0;
            rewind(pram_data->f);
            assert(fwrite(pram_data->pram, 256, 1, pram_data->f) == 1);
            fflush(stdout);
            pram_data->tear_down_thread = 0;
        }
        sleep(1);
    }
    
    return NULL;
}


static _Bool _setup_shoebill (void)
{
    uint32_t i;
    shoebill_config_t config;
    
    memset(&config, 0, sizeof(shoebill_config_t));
    
    config.aux_verbose = user_params.verbose;
    config.ram_size = user_params.ram_megabytes * 1024 * 1024;
    config.aux_kernel_path = user_params.relative_unix_path;
    config.rom_path = user_params.rom_path;
    config.pram_callback = _pram_callback;
    config.pram_callback_param = (void*)&user_params.pram_data;
    memcpy(config.pram, user_params.pram_data.pram, 256);
    
    for (i=0; i<7; i++)
        config.scsi_devices[i].path = user_params.scsi_path[i];
    
    if (!shoebill_initialize(&config)) {
        printf("%s\n", config.error_msg);
        return 0;
    }
    
    if (user_params.use_tfb) {
        shoebill_install_tfb_card(&config, 9);
    }
    else {
        shoebill_install_video_card(&config,
                                    9, // slotnum
                                    user_params.width,
                                    user_params.height);
    }
    

    shoebill_start();
    return 1;
}

static void _handle_key_event (SDL_Event *event)
{
    const SDL_Keycode sym = event->key.keysym.sym;
    const _Bool key_down = (event->type == SDL_KEYDOWN);
    const SDL_Keymod sdl_mod = SDL_GetModState();
    uint16_t adb_mod = 0;
    uint16_t value;
    
    if (sdl_mod & KMOD_SHIFT) adb_mod |= modShift;
    if (sdl_mod & KMOD_CTRL) adb_mod |= modControl;
    if (sdl_mod & KMOD_ALT) adb_mod |= modOption;
    if (sdl_mod & KMOD_GUI) adb_mod |= modCommand;
    if (sdl_mod & KMOD_CAPS) adb_mod |= modCapsLock;
    
    if (rb_find(keymap, sym, &value)) {
        shoebill_key_modifier((value >> 8) | adb_mod);
        shoebill_key(key_down, value & 0xff);
    }
}

static _Bool _init_pram (void)
{
    FILE *f = fopen(user_params.pram_path, "r+b");
    memset(&user_params.pram_data, 0, sizeof(struct shoe_app_pram_data_t));
    
    if ((f == NULL) || (fread(user_params.pram_data.pram, 256, 1, f) != 1)) {
        if (f == NULL)
            f = fopen(user_params.pram_path, "w+b");
        if (f == NULL) {
            printf("Can't open pram_path! [%s] [errno=%s]\n",
                   user_params.pram_path,
                   sys_errlist[errno]);
            return 0;
        }
        rewind(f);
        shoebill_validate_or_zap_pram(user_params.pram_data.pram, 1);
        
        assert(fwrite(user_params.pram_data.pram, 256, 1, f) == 1);
        fflush(f);
    }
    
    user_params.pram_data.f = f;
    shoebill_validate_or_zap_pram(user_params.pram_data.pram, 0);
    
    pthread_create(&user_params.pram_data.threadid,
                   NULL,
                   _pram_writer_thread,
                   &user_params.pram_data);
    
    return 1;
}

int main (int argc, char **argv)
{
    const uint32_t frame_ticks = 1000 / 60;
    uint32_t last_frame_ticks;
    _Bool capture_cursor;
    
    _print_vers();
    
    _init_keyboard_map();
    _init_user_params(argc, argv);
    if (!_init_pram())
        return 0;
    else if (!_setup_shoebill())
        return 0;
    
    
    
    shoebill_video_frame_info_t frame = shoebill_get_video_frame(9, 1);
    
    SDL_Init(SDL_INIT_VIDEO);
    
    SDL_Window *win = SDL_CreateWindow("Shoebill",
                                      SDL_WINDOWPOS_UNDEFINED,
                                      SDL_WINDOWPOS_UNDEFINED,
                                      frame.width, frame.height,
                                      SDL_WINDOW_OPENGL);
    
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GLContext glctx = SDL_GL_CreateContext(win);
    
    glShadeModel(GL_FLAT);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearColor(0.5, 0.5, 0.5, 1.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, frame.width, 0, frame.height, -1.0, 1.0);
    
    capture_cursor = 1;
    SDL_ShowCursor(0);
    SDL_SetRelativeMouseMode(1);
    
    SDL_GL_SetSwapInterval(1);
    
    last_frame_ticks = SDL_GetTicks();
    while (1) {
        const uint32_t now = SDL_GetTicks();
        uint32_t ticks_to_next_frame;
        SDL_Event event;
        
        if ((now - last_frame_ticks) >= frame_ticks) {
            _display_frame(win);
            last_frame_ticks = now;
            ticks_to_next_frame = frame_ticks;
        }
        else
            ticks_to_next_frame = frame_ticks - (now - last_frame_ticks);
        
        event.type = SDL_USEREVENT;
        SDL_WaitEventTimeout(&event, ticks_to_next_frame);
        
        switch (event.type) {
            case SDL_QUIT:
                goto quit;
                
            case SDL_MOUSEBUTTONDOWN: {
                if ((event.button.button == SDL_BUTTON_LEFT) && capture_cursor)
                    shoebill_mouse_click(1);
                break;
            }
                
            case SDL_MOUSEBUTTONUP: {
                // If the cursor isn't captured, then any click will capture it
                if (!capture_cursor)
                    capture_cursor = 1;
                // If the cursor is captured, then left clicks get sent to the OS
                else if (event.button.button == SDL_BUTTON_LEFT)
                    shoebill_mouse_click(0);
                // If the cursor is captured, then right clicks will uncapture it
                else if ((event.button.button == SDL_BUTTON_RIGHT) && capture_cursor)
                    capture_cursor = 0;
                break;
            }
                
            case SDL_MOUSEMOTION: {
                if (capture_cursor) {
                    _Bool down = event.motion.state & SDL_BUTTON(SDL_BUTTON_LEFT);
                    shoebill_mouse_click(down);
                    shoebill_mouse_move_delta(event.motion.xrel, event.motion.yrel);
                }
                break;
            }
                
            case SDL_KEYDOWN:
            case SDL_KEYUP:
                if (!event.key.repeat)
                    _handle_key_event(&event);
                break;
        }
        
        if (capture_cursor) {
            SDL_ShowCursor(0);
            SDL_SetRelativeMouseMode(1);
        }
        else {
            SDL_ShowCursor(1);
            SDL_SetRelativeMouseMode(0);
        }
    }
    
quit:
    
    // FIXME: tear down the pram thread and flush pram
    
    return 0;
}

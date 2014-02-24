#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include <GLUT/glut.h>

#include "core/shoebill.h"
#include "core/core_api.h"

shoebill_control_t control;
shoebill_card_video_t *video_card = NULL;

void glut_display_func (void)
{
    uint32_t myw = glutGet(GLUT_WINDOW_WIDTH);
    uint32_t myh = glutGet(GLUT_WINDOW_HEIGHT);
    uint32_t slotnum, i;
    shoebill_card_video_t *ctx = video_card;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, myw, 0, myh, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glColor3f(0.1, 0.1, 0.8);
    
    uint32_t gli = 0;

    switch (ctx->depth) {
        case 1: {
            for (i=0; i < ctx->pixels/8; i++) {
                const uint8_t byte = ctx->indexed_buf[i];
                ctx->direct_buf[i * 8 + 0] = ctx->clut[(byte >> 7) & 1];
                ctx->direct_buf[i * 8 + 1] = ctx->clut[(byte >> 6) & 1];
                ctx->direct_buf[i * 8 + 2] = ctx->clut[(byte >> 5) & 1];
                ctx->direct_buf[i * 8 + 3] = ctx->clut[(byte >> 4) & 1];
                ctx->direct_buf[i * 8 + 4] = ctx->clut[(byte >> 3) & 1];
                ctx->direct_buf[i * 8 + 5] = ctx->clut[(byte >> 2) & 1];
                ctx->direct_buf[i * 8 + 6] = ctx->clut[(byte >> 1) & 1];
                ctx->direct_buf[i * 8 + 7] = ctx->clut[(byte >> 0) & 1];
            }
            break;
        }
        case 2: {
            for (i=0; i < ctx->pixels/4; i++) {
                const uint8_t byte = ctx->indexed_buf[i];
                ctx->direct_buf[i * 4 + 0] = ctx->clut[(byte >> 6) & 3];
                ctx->direct_buf[i * 4 + 1] = ctx->clut[(byte >> 4) & 3];
                ctx->direct_buf[i * 4 + 2] = ctx->clut[(byte >> 2) & 3];
                ctx->direct_buf[i * 4 + 3] = ctx->clut[(byte >> 0) & 3];
            }
            break;
        }
        case 4: {
            for (i=0; i < ctx->pixels/2; i++) {
                const uint8_t byte = ctx->indexed_buf[i];
                ctx->direct_buf[i * 2 + 0] = ctx->clut[(byte >> 4) & 0xf];
                ctx->direct_buf[i * 2 + 1] = ctx->clut[(byte >> 0) & 0xf];
            }
            break;
        }
        case 8:
            for (i=0; i < ctx->pixels; i++)
                ctx->direct_buf[i] = ctx->clut[ctx->indexed_buf[i]];
            break;
            
        default:
            assert(!"unknown depth");
    }
    
    glViewport(0, 0, myw, myh);
    glRasterPos2i(0, myh);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    glPixelZoom(1.0, -1.0);
    
    glDrawPixels(myw, myh, GL_RGBA, GL_UNSIGNED_BYTE, ctx->direct_buf);
    
    glFlush();
}

/*void vbl_callback (shoebill_control_t *ctrl, uint8_t slotnum)
{
    
    
}

void video_depth_calback (sheobill_control_t *ctrl, uint8_t slotnum)
{
    
    
}*/

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
    glutPostRedisplay();
}

int main (int argc, char **argv)
{
    bzero(&control, sizeof(shoebill_control_t));
    
    control.aux_verbose = 1;
    control.ram_size = 1024*1024*1024;
    control.aux_kernel_path = "priv/unix2";
    control.rom_path = "priv/macii.rom";
    
    control.scsi_devices[0].path = "priv/aux2.img";
    
    uint32_t result = shoebill_initialize(&control);
    if (!result) {
        printf("fail: %s\n", control.error_msg);
        return 0;
    }
    else
        printf("success!\n");
    
    shoebill_install_video_card(&control, 10, 800, 600,
                                (2.0/3.0) * 100.0); // 66.67hz
    video_card = &control.slots[10].card.video;
    
    
    int dummyargc = 1;
    glutInit(&dummyargc, argv);
    glutInitWindowSize(video_card->scanline_width, video_card->height);
    glutCreateWindow("");
    glutDisplayFunc(glut_display_func);
    glShadeModel(GL_FLAT);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearColor(0.1, 1.0, 0.1, 1.0);
    
    shoebill_start();
    
    glutTimerFunc(15, timer_func, 0);
    glutMainLoop();
    
    return 0;
}

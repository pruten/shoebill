#include <assert.h>
#include "../core/shoebill.h"


static uint8_t sound_dma_read_byte(uint16_t addr)
{
    if (addr == 0x804)
        return 0xff;
    return 0;
}

static void sound_dma_write_byte(uint16_t addr, uint8_t data)
{
    if (addr >= 0x800) {
        // registers
    }
    else if (addr >= 0x400) {
        // Buf B
    }
    else {
        // Buf A
        /*FILE *f = fopen("buf_a.dmp", "ab");
        if (f) {
            fwrite(&data, 1, 1, f);
            fclose(f);
        }*/
    }
}


void sound_dma_write_raw(uint16_t addr, uint8_t sz, uint32_t data)
{
    int32_t i;
    slog("sound_dma_write: addr=0x%04x sz=%u dat=0x%x\n", addr, sz, data);
    
    for (i = (sz-1) * 8; i >= 0; i -= 8)
        sound_dma_write_byte(addr, (data >> i) & 0xff);
}

uint32_t sound_dma_read_raw(uint16_t addr, uint8_t sz)
{
    uint32_t i, result = 0;
    slog("sound_dma_read: addr=0x%04x sz=%u\n", addr, sz);
    
    for (i=0; i<sz; i++) {
        result <<= 8;
        result |= sound_dma_read_byte(addr + i);
    }
    
    return result;
}
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

int main (int argc, char **argv)
{
    FILE *in, *out;
    uint8_t *rom = malloc(4096);
    uint32_t i;
    
    if (argc != 3) {
        printf("usage ./rom_to_c video_rom.bin video_rom.c\n");
        return 0;
    }
    
    assert(in = fopen(argv[1], "r"));
    assert(out = fopen(argv[2], "w"));
    
    assert(fread(rom, 4096, 1, in) == 1);
    
    fprintf(out, "uint8_t _video_rom[4096] = {\n\t");
    for (i=0; i<4095; i++) {
        fprintf(out, "0x%02x, ", rom[i]);
        if ((i % 8) == 7)
            fprintf(out, "\n\t");
    }
    fprintf(out, "0x%02x\n};\n", rom[i]);
    fclose(out);
    
    return 0;
}


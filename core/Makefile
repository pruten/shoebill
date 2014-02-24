
CC = clang
CFLAGS = -O4 -ggdb -arch x86_64 -Wno-deprecated-declarations


DEPS = core_api.h coff.h mc68851.h redblack.h shoebill.h Makefile macro.pl
NEED_DECODER = cpu dis
NEED_PREPROCESSING = adb fpu mc68851 mem via
NEED_NOTHING = atrap_tab coff exception floppy macii_symbols redblack scsi toby_frame_buffer video core_api 

OBJ_NEED_NOTHING = $(patsubst %,$(TEMP)/%.o,$(NEED_NOTHING))
OBJ_NEED_PREPROCESSING = $(patsubst %,$(TEMP)/%.o,$(NEED_PREPROCESSING))
OBJ_NEED_DECODER = $(patsubst %,$(TEMP)/%.o,$(NEED_DECODER))

# Files that NEED_DECODER also NEED_PREPROCESSING
POST_PREPROCESSING = $(patsubst %,$(TEMP)/%.post.c,$(NEED_PREPROCESSING)) $(patsubst %,$(TEMP)/%.post.c,$(NEED_DECODER))

MACRO = perl macro.pl

TEMP = ../intermediates


all: $(TEMP)/libshoebill_core.a 

$(TEMP)/libshoebill_core.a: $(TEMP) $(DEPS) $(OBJ_NEED_NOTHING) $(OBJ_NEED_PREPROCESSING) $(OBJ_NEED_DECODER)
	libtool -static -o $(TEMP)/libshoebill_core.a $(OBJ_NEED_NOTHING) $(OBJ_NEED_PREPROCESSING) $(OBJ_NEED_DECODER)

# Build object files
$(OBJ_NEED_NOTHING): $(TEMP)/%.o: %.c $(DEPS)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_NEED_PREPROCESSING): $(TEMP)/%.o: $(TEMP)/%.post.c $(DEPS)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_NEED_DECODER): $(TEMP)/%.o: $(TEMP)/%.post.c $(DEPS) $(TEMP)/dis_decoder_guts.c $(TEMP)/inst_decoder_guts.c
	$(CC) -c $(CFLAGS) $< -o $@

# Preprocess C files 
$(POST_PREPROCESSING): $(TEMP)/%.post.c: %.c $(DEPS)
	$(MACRO) $< $@

# Generate instruction decoders
$(TEMP)/inst_decoder_guts.c: $(TEMP)/decoder_gen $(DEPS)
	$(TEMP)/decoder_gen inst $(TEMP)/
$(TEMP)/dis_decoder_guts.c: $(TEMP)/decoder_gen $(DEPS)
	$(TEMP)/decoder_gen dis $(TEMP)/

# Compile the decoder generator
$(TEMP)/decoder_gen: decoder_gen.c $(DEPS)
	$(CC) $(CFLAGS) decoder_gen.c -o $(TEMP)/decoder_gen


$(TEMP):
	mkdir -p $(TEMP)

clean:
	rm -rf $(TEMP)



CC = clang
CFLAGS = -O3 -flto -ggdb -Wno-deprecated-declarations
# CFLAGS = -O0 -ggdb -Wno-deprecated-declarations


DEPS = core_api.h coff.h mc68851.h redblack.h shoebill.h Makefile macro.pl
NEED_DECODER = cpu dis
NEED_PREPROCESSING = adb fpu mc68851 mem via
NEED_NOTHING = atrap_tab coff exception floppy macii_symbols redblack scsi toby_frame_buffer video core_api filesystem debug_server alloc_pool

# Object files that can be compiled directly from the source
OBJ_NEED_NOTHING = $(patsubst %,$(TEMP)/%.o,$(NEED_NOTHING))

# Object files than need preprocessing with macro.pl
OBJ_NEED_PREPROCESSING = $(patsubst %,$(TEMP)/%.o,$(NEED_PREPROCESSING))

# Object files that depend on the instruction decoder
OBJ_NEED_DECODER = $(patsubst %,$(TEMP)/%.o,$(NEED_DECODER))

# Files that NEED_DECODER also NEED_PREPROCESSING
POST_PREPROCESSING = $(patsubst %,$(TEMP)/%.post.c,$(NEED_PREPROCESSING)) $(patsubst %,$(TEMP)/%.post.c,$(NEED_DECODER))



# All the object files compiled for x86_64
OBJ_x86_64 = $(OBJ_NEED_NOTHING) $(OBJ_NEED_PREPROCESSING) $(OBJ_NEED_DECODER)

# The object files compiled for i386 (the same as x86_64 files, but with .i386 appended)
OBJ_i386 = $(patsubst %,%.i386,$(OBJ_x86_64))


MACRO = perl macro.pl

TEMP = ../intermediates


all: $(TEMP)/libshoebill_core.a 

$(TEMP)/libshoebill_core.a: $(TEMP) $(DEPS) $(OBJ_x86_64)
	libtool -static -v -o $(TEMP)/libshoebill_core.a.x86_64 $(OBJ_x86_64)
	libtool -static -v -o $(TEMP)/libshoebill_core.a.i386 $(OBJ_i386)
	lipo -create -output $(TEMP)/libshoebill_core.a $(TEMP)/libshoebill_core.a.x86_64 $(TEMP)/libshoebill_core.a.i386


# Split object files into i386/x86_64 versions, since it seems that libtool is unable to 
# link a static universal library for -O4 object files.
# x86_64 object files have the form "intermediates/<file_name>.o
# i386 object files have the form "intermediates/<file_name>.o.i386

# Build object files
$(OBJ_NEED_NOTHING): $(TEMP)/%.o: %.c $(DEPS)
	$(CC) -c -arch x86_64 $(CFLAGS) $< -o $@
	$(CC) -c -arch i386 $(CFLAGS) $< -o $@.i386

$(OBJ_NEED_PREPROCESSING): $(TEMP)/%.o: $(TEMP)/%.post.c $(DEPS)
	$(CC) -c -arch x86_64 $(CFLAGS) $< -o $@
	$(CC) -c -arch i386 $(CFLAGS) $< -o $@.i386

$(OBJ_NEED_DECODER): $(TEMP)/%.o: $(TEMP)/%.post.c $(DEPS) $(TEMP)/dis_decoder_guts.c $(TEMP)/inst_decoder_guts.c
	$(CC) -c -arch x86_64 $(CFLAGS) $< -o $@
	$(CC) -c -arch i386 $(CFLAGS) $< -o $@.i386

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


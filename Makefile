
CC = clang
CFLAGS = -O4 -arch x86_64 -Wno-deprecated-declarations
LFLAGS = -framework OpenGL -framework GLUT

all: shoebill

shoebill: make_core test.c
	$(CC) $(LFLAGS) $(CFLAGS) -L intermediates -l shoebill_core test.c -o shoebill

make_core:
	$(MAKE) -C core -j 4

clean:
	rm -rf intermediates
	rm -f shoebill

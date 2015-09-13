
CC = clang
CFLAGS = -O3 -ggdb -flto -Wno-deprecated-declarations
LFLAGS = -L ../intermediates -lshoebill_core -framework GLUT -framework OpenGL -ledit

all: debugger

debugger: Makefile debugger.c ../intermediates/libshoebill_core.a
	$(CC) $(CFLAGS) $(LFLAGS) debugger.c -o debugger

clean:
	rm -rf debugger

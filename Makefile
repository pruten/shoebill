
all: shoebill

shoebill: make_core

make_core:
	$(MAKE) -C core -j 4

clean:
	rm -rf intermediates

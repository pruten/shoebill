
all: shoebill

shoebill: make_gui

make_gui: make_core
	xcodebuild -project gui/Shoebill.xcodeproj SYMROOT=build

make_core:
	$(MAKE) -C core -j 4

clean:
	rm -rf intermediates gui/build

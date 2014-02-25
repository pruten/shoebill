
Shoebill - a Macintosh II emulator that runs A/UX
 (except A/UX 3.x.x currently)


See the wiki on https://github.com/pruten/shoebill for better
documentation on building and running Shoebill.

*** KEEP IN MIND ***

* Shoebill v.first-terrible-code-drop (a.k.a. version 0.0.1)
  * ONLY RUNS A/UX
    * BUT NOT 3.x.x (I’m working on it)
    * Only 1.x.x and 2.x.x
* Shoebill has broken, ultra-minimalist support for
  * 68020 (CPU) + 68851 (MMU) + 68881 (FPU)
    * Some instructions for ‘020 and most for the MMU and FPU are unimplemented
  * SCSI, ADB, and VIA
* Shoebill does not support
  * The floppy controller, IWM/SWIM
  * Serial
  * Ethernet
  * PRAM
  * Most other things  

*** RUNNING ***

You will need
* OS X and a 64-bit Intel Macintosh
  (32-bit builds are possible by twiddling the makefiles)
* A Macintosh II or IIx ROM
* A disk image with A/UX 1.x.x or 2.x.x installed
  * If you happen to have an installation CD image for A/UX, that will work
* The kernel on that image (/unix). Shoebill can’t read
  SVFS or UFS file sytems yet to load the kernel directly
  from the disk image.


To boot A/UX
* Backup your disk images!!
  When Shoebill inevitably crashes, your A/UX boot image
  will very likely be corrupted - sometimes so severely 
  that A/UX can’t even boot enough to run fsck.
* Open Shoebill.app and select Preferences menu item
  * Set the paths for your ROM, kernel, and disk image(s).
  * Do use SCSI ID #0 for your A/UX boot image.
  * Press “Apply and Run”  


*** BUILDING ***

1) cd to shoebill/
2) make # to build shoebill_core
3) xcodebuild -project gui/Shoebill.xcodeproj # to build the Cocoa GUI



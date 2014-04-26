
Shoebill - a Macintosh II emulator that runs A/UX

See the wiki on https://github.com/pruten/shoebill for better
documentation on building and running Shoebill.

*** KEEP IN MIND ***

* Shoebill v.0.0.2
  * ONLY RUNS A/UX
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
* OS X 10.8 or 10.9
* A Macintosh II, IIx, or IIcx ROM
* A disk image with A/UX 1.x.x or 2.x.x, or 3.0.0 installed
  * Note: 3.0.1 and 3.1.x do not work!
  * If you happen to have an installation CD image for A/UX, that will work


To boot A/UX
* Backup your disk images!!
  When Shoebill inevitably crashes, your A/UX boot image
  will very likely be corrupted - sometimes so severely 
  that A/UX can’t even boot enough to run fsck.
* Open Shoebill.app and select Preferences menu item
  * Set the paths for your ROM and disk image(s).
  * Do use SCSI ID #0 for your A/UX boot image.
  * Press “Apply and Run” 
* Note: As of 0.0.2, you no longer need to provide your own kernel file


*** BUILDING ***

1) cd to shoebill/
2) make
3) The resulting app will be in gui/build



*** ETC. ***
Props to Jared Falter for technical and emotional support!



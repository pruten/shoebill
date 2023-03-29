<h1><img align=right src="https://github.com/pruten/pruten.github.io/raw/master/web/stork_tiny_head3.jpg"/>Shoebill</h1>

A Macintosh II emulator that runs A/UX (and A/UX only). 

Shoebill is an all-new, BSD-licensed Macintosh II emulator designed from the ground up with the singular goal of running A/UX. 

Shoebill requires a Macintosh II, IIx or IIcx ROM, and a disk image with A/UX installed.

[Download the latest release], and then see the [getting started] wiki.  
Also check out [screenshots].

__Update (March 29, 2023): About issues/pull requests__

__I just wanted to say that I appreciate some folks are still using Shoebill and submitting issues and pull requests. I wish I could continue working on this project, but there's a likely conflict of interest, and so I've mostly avoided pushing changes. I apologize for being unable to address the many, many bugs in this repo. (Also for anyone unaware, [Qemu is now able] now to run A/UX 3.x on its emulated Quadra 800.)__

__Update (Sept 13, 2015): [Shoebill 0.0.5 is available]__

__This will probably be the last release. I won't be able to work on Shoebill going forward (by contractual obligation), so I wanted to race out one last release. Only an OS X binary is available, sorry, and it's very unpolished. But the SDL GUI should still build on linux/windows.__


#### Supports
* A/UX 1.1.1 through 3.1 (and 3.1.1 a little)

#### Currently Implements
* 68020 CPU (mostly)
* 68881 FPU (mostly)
* 68851 PMMU (just enough to boot A/UX)
* SCSI
* ADB
* PRAM
* Ethernet (via emulated Apple EtherTalk/DP8390 card)
* A NuBus video card with 24-bit depth. 

#### Does not implement (yet)
* Sound
* Floppy
* Serial ports

    
[Download the latest release]:https://github.com/pruten/Shoebill/releases
[getting started]:https://github.com/pruten/Shoebill/wiki/Getting-Started
[screenshots]:https://github.com/pruten/Shoebill/wiki/Screenshots
[Shoebill 0.0.5 is available]:https://github.com/pruten/Shoebill/releases
[The thread on emaculation.com]:http://www.emaculation.com/forum/viewtopic.php?f=7&t=8288
[Qemu is now able]:https://virtuallyfun.com/2021/09/02/qemus-macintosh-quadra-in-alpha-usability-runs-a-ux/


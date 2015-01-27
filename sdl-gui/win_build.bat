for %%i in (adb fpu mc68851 mem via floppy core_api cpu dis) do (
	perl ..\core\macro.pl ..\core\%%i.c %%i.post.c
)

gcc -O1 ..\core\decoder_gen.c -o decoder_gen
decoder_gen inst .
decoder_gen dis .

gcc -O3 -flto -mno-ms-bitfields sdl.c adb.post.c fpu.post.c mc68851.post.c mem.post.c via.post.c floppy.post.c core_api.post.c cpu.post.c dis.post.c ..\core\atrap_tab.c ..\core\coff.c ..\core\exception.c ..\core\macii_symbols.c ..\core\redblack.c ..\core\scsi.c ..\core\video.c ..\core\filesystem.c ..\core\alloc_pool.c ..\core\toby_frame_buffer.c ..\core\ethernet.c ..\core\sound.c ..\core\SoftFloat\softfloat.c -lmingw32 -lopengl32 -lsdl2main -lsdl2 -o shoebill

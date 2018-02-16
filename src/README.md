The port is made of multiple files with the following roles

hn70ap_boot.c:
   - Contains the entry point to initialize all board related stuff.
     This function is called early in the boot and can ONLY initialize
     low level stuff like GPIO lines. NO DEVICE should be registered here
     since the OS has not been initialized yet.

hn70ap_spi.c:
   - Initialize CS lines
   - Define callbacks for peripheral selection


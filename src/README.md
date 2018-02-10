The port is made of multiple files with the following roles

stm32_boot.c:
   - Contains the entry point to initialize all board related stuff.
     This function is called early in the boot and can ONLY initialize
     low level stuff like GPIO lines. NO DEVICE should be registered here
     since the OS has not been initialized yet.

stm32_spi.c:
   - Initialize CS lines
   - Define callbacks for peripheral selection


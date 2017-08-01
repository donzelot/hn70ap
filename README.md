# hn70ap
Development board for modern packet radio on UHF

This repository contains sources for a Kicad PCB and a NuttX board support package for the board.

The schematic and board design is released under the CERN OHL license.

Prerequisites
=============
 * linux (in 2017 there are many ways to get that)
 * a micro-usb cable to see the debug uart output
 * a jtag/swd adapter
 * a working openocd installation
 * an arm-none-eabi toolchain: https://launchpad.net/gcc-arm-embedded
 * to change the default configs: install kconfig-frontends
 
kconfig-frontends
=================
TODO, ./configure && make && make install

openocd
=======
TODO apt-get or ./configure && make && make install

Basic Build
===========
You need a working NuttX development environment:

```
$ mkdir nuttx
$ cd nuttx
$ git clone https://bitbucket.org/nuttx/nuttx
$ git clone https://bitbucket.org/nuttx/apps
$ cd nuttx
```
Then you can clone the board config (This is NOT a submodule!)
```
$ cd configs
$ git clone https://github.com/f4grx/hn70ap
$ cd ..
```
Then build the binary image for this board:
```
$ tools/configure.sh hn70ap/nsh
$ make
```
This should end up with an ELF binary 'nuttx', and binary images nuttx.bin and nuttx.hex

You then have to flash this binary on your board, probably with OpenOCD.

This will give you a basic shell, type help to show basic commands. This can be used to validate the behaviour of the board's CPU.

Flashing without JTAG
=====================

This will be possible later, when the bootloader is made ready. The hardware is ready to support that.
A python tool will build an update image from the nuttx elf output, and you will be able to send that image via ethernet.

Advanced build
==============
More configurations will be added later that can be used to test ethernet, spi/i2c memories, etc

To modify the nuttx options type make menuconfig and mess around.

To get useful behaviour from this board you will need to add some applications that are not developed yet.

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
 
Mandatory: Install toolchain
============================
The arm-none-eabi toolchain provided by your distro probably does not work OK. You may be lucky but this is not widespread enough. So we're installing a really working toolchain.

The hardest way: compile a toolchain
------------------------------------
Hey this is a beginner guide. you dont need my guide if you are doing that. Help yourself!

The hard way: download and install
----------------------------------

Open a bash shell console, then type commands ONE BY ONE (adapt them if you know what you are doing):
```
$ cd $HOME
$ mkdir toolchains
$ cd toolchains
$ wget -O toolchain.tar.bz2 https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q3-update/+download/gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2
$ tar jxvf toolchain.tar.bz2
$ cd $HOME
$ export PATH=$PATH:$HOME/toolchains/gcc-arm-none-eabi-5_4-2016q3/bin
```
The last line makes the ARM compiler available in your shell environment. You can make that automatic, search the web. Otherwise you have to execute this last line in each new shell window.

The easy way for ubuntu based distros
-------------------------------------

There is a ppa
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```

Toolchain test
--------------

You can test that the toolchain is functional by typing this command and expecting this answer:
```
$ arm-none-eabi-gcc
arm-none-eabi-gcc: fatal error: no input files
compilation terminated.
```

If the command is not found, then the toolchain is not correctly installed and the following will NOT work.

Optional: kconfig-frontends
===========================
Skip for now. TODO, ./configure && make && make install

openocd
=======
Skip for now. TODO apt-get or ./configure && make && make install

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

#!/usr/bin/env python3
# hn70ap_mkupdate.py - Builds OS update images for the hn70ap board
# Copyright (C) 2018 Sebastien Lorquet
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Boston, MA  02110-1301  USA

# This program transforms an ELF program for arm into an update image that can
# be used to update a hn70ap board using the default bootloader.
#
# Note that this tool is not a part of NuttX and has a different licence than
# the NuttX RTOS.

import sys, struct

#This class can be used to read an ELF file
class elfreader:

  #-----------------------------------------------------------------------------
  def __init__(self):
    self.source = None

  #-----------------------------------------------------------------------------
  def open(self, fname):
    """Open an ELF file given its name"""
    self.source = open(fname, 'rb')
    #read the program header
    buf = self.source.read(6)

    #check signature and format
    if buf[0] != 0x7F or buf[1:4] != b'ELF':
      raise Exception('bad ELF signature')

    if buf[4] != 1:
      raise Exception('bad ELF format')

    #read endianess
    self.endian = "<" if (buf[5] == 1) else ">"

    #check this is an executable
    self.source.seek(16)
    buf = self.source.read(4)
    typ, iset = struct.unpack(self.endian+"HH", buf)
    if typ != 2:
      raise Exception('not an ELF executable')
    if iset != 40:
      print("WARNING not ARM instruction set!")

  #-----------------------------------------------------------------------------
  def close(self):
    """Close the current ELF file"""
    self.source.close()

  #-----------------------------------------------------------------------------
  def segments(self):
    """Return a list representing the list of all ELF memory segments"""

    #get location of program header
    self.source.seek(28)
    buf = self.source.read(4)
    ph  = struct.unpack(self.endian+"L", buf)[0]

    #get count of program headers
    self.source.seek(42)
    buf = self.source.read(4)
    phsz, phcnt = struct.unpack(self.endian+"HH", buf)

    segs = []
    for i in range(phcnt):
      #read program header
      self.source.seek(ph)
      buf = self.source.read(phsz)
      typ,off,vaddr,paddr,filesz,memsz,flags,align = struct.unpack(self.endian+"LLLLLLLL", buf)
      #read associated memory
      self.source.seek(off)
      buf = self.source.read(filesz)
      seg = {'type':typ, 'vaddr':vaddr, 'paddr':paddr, 'filesz':filesz, 'memsz':memsz, 'flags':flags, 'align':align, 'mem':buf}
      segs.append(seg)
      ph += phsz

    return segs

#===============================================================================

#manage arguments

#default value is main nuttx binary
source = "nuttx"

if len(sys.argv) == 2:
  source = sys.argv[1]

print("hn70ap_mkupdate starting, source=", source)
elf = elfreader()
elf.open(source)
segs = elf.segments()
elf.close()

if segs == None:
  print("ELF file was not parsed correctly")
  sys.exit(1)

#stm32 memory is split in two parts:
#  - the bootloader in the first flash sector (16k)
#  - the OS image (remaining of memory)
#build the binary image
binsize = 0
binbase = 0x08000000
binend  = binbase
for seg in segs:
  print("%d bytes at %08X" % (seg['filesz'], seg['paddr']) )
  end = seg['paddr'] + seg['filesz']
  if end > binend:
    binend = end

print("binary image in range %08X - %08X" % (binbase,binend))
binimg = bytearray(binend-binbase)

#copy the program segments into the binary
for seg in segs:
  binoff = seg['paddr'] - binbase
  binimg[binoff:binoff+seg['filesz']] = seg['mem']

with open('output','wb') as out:
  out.write(binimg)


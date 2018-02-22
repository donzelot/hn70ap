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
  def __init__(self):
    self.source = None

  def open(self, fname):
    """Open an ELF file given its name"""
    self.source = open(fname)
    #read the program header


  def close(self):
    """Close the current ELF file"""
    self.source.close()

  def segments(self):
    """Return a list representing the list of all ELF memory segments"""
    #read the section headers and the section contents

    return None

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
for seg in segs:
  print("seg: ", seg["name"])


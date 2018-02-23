#!/usr/bin/env python3
# hn70ap_update_serial.py - Sends an update to the hn70ap board via serial port.
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

# This program sends a firmware update image to a hn70ap port via a serial port.
#
# Note that this tool is not a part of NuttX and has a different licence than
# the NuttX RTOS.

import pyserial


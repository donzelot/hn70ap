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

import sys, serial, os

INST_WRITE = (0x00).to_bytes(1, byteorder='big')

FDELIMB  = 0x7E
FDELIM   = FDELIMB.to_bytes(1, byteorder='big')
FESCB    = 0x7D
FESC     = FESCB.to_bytes(1, byteorder='big')
CRC_INIT = 0xFFFF
CRC_GOOD = 0xF0B8

STATE_NOSYNC = 0
STATE_SYNC   = 1
STATE_DATA   = 2
STATE_ESC    = 3

#-------------------------------------------------------------------------------
def crc16(crc, val):

  crc ^= val & 0xFF
  crc ^= (crc<<4) & 0xFF
  crc  = (crc>>8) ^ ((crc&0xFF)<<8) ^ ((crc&0xFF)<<3) ^ ((crc&0xFF)>>4)

  return crc

#-------------------------------------------------------------------------------
def frame_send(port, frame):

  def write_esc(port,data):
    if data == FDELIMB or data == FESCB:
      port.write(FESC)
      data ^= 0x20
    port.write(data.to_bytes(1, byteorder='big'))

  #print('out dat=',frame.hex())
  crc = CRC_INIT
  port.write(FDELIM)
  for i in range(len(frame)):
    b = frame[i]
    crc = crc16(crc, b)
    write_esc(port, b)

  crc ^= 0xFFFF;
  write_esc(port,crc&0xFF)
  write_esc(port,crc>>8)
  #print('out crc=',crc.to_bytes(2,byteorder='little').hex())
  port.write(FDELIM)

#-------------------------------------------------------------------------------
def frame_receive(port, maxlen):
  state = STATE_NOSYNC
  packet = bytearray()
  crc = CRC_INIT
  maxlen += 2 #include the length required to receive the CRC after the user bytes

  while True:
    ret = port.read(1)
    if len(ret)==0:
      raise Exception("Receive timeout!")

    #print("state",state,"char",ret.hex(),"'",ret,"'", "len",len(packet))
    b = ret[0]
    if state == STATE_NOSYNC:
      if ret == FDELIM:
        state = STATE_SYNC
      else:
        print("\x1B[32m", chr(ret[0]), "\x1B[0m", sep='', end='', flush=True)

    elif state == STATE_SYNC:
      if ret != FDELIM:
        packet.append(b)
        crc = crc16(crc, b)
        state = STATE_DATA
        if len(packet) > maxlen:
          print("packet too long")
          break

    elif state == STATE_DATA:
      if ret == FESC:
        state = STATE_ESC
      elif ret == FDELIM:
        break
      else:
        packet.append(b)
        crc = crc16(crc, b)
        if len(packet) > maxlen:
          print("packet too long")
          break

    elif state == STATE_ESC:
      b ^= 0x20;
      packet.append(b)
      crc = crc16(crc, b)
      if len(packet) > maxlen:
        print("packet too long")
        break
      state = STATE_DATA

  #check crc
  #print("rx crc=%04X"%crc)
  if crc != CRC_GOOD:
    raise Exception("bad CRC in rx frame")

  #done
  return packet[0:len(packet)-2]

#-------------------------------------------------------------------------------
if len(sys.argv) != 3:
  print("hn70ap_serial_update.py <port> <updateimage>")
  sys.exit(1)

up = open(sys.argv[2], 'rb')

#a short timeout will not work since erase needs time
port = serial.Serial(port=sys.argv[1], baudrate=230400, timeout=100)
done = False
while not done:
  print("Sending prompt...")
  port.reset_input_buffer()
  port.reset_output_buffer()
  port.write(b"\r\n\r\nupdate serial\r\n")
  while True:
    resp = port.readline()
    if len(resp) == 0:
      break
    #print(">",resp)
    if b"hn70ap serial update waiting..." in resp:
      print("Good response")
      done = True
      break
  if not done: print("Unexpected response, trying again") 

uplen = os.fstat(up.fileno()).st_size
print("upload start")

BLOCKSIZE = 256

done = 0
seq = 0
while True:
  buf = up.read(BLOCKSIZE)
  l = len(buf)
  s = seq.to_bytes(2,byteorder='big')
  #print("seq=", s.hex(), "len=", l, buf.hex())
  frame_send(port, INST_WRITE+s+buf)
  rx = frame_receive(port,1+2+BLOCKSIZE)
  #print(rx.hex())
  status = rx[3];
  #print("status:",status);
  if status == 3:
    print("send complete")
    break

  done += l
  print("sent bytes",done,"of",uplen, end='\r')
  if l < BLOCKSIZE:
    break
  seq = (seq + 1) & 0xFFFF

print("\nupload complete")

port.close()
up.close()


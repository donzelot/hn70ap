The hn70ap bootloader
=====================

The firmware on the hn70ap board can be directly flashed via JTAG. It can also
be updated using an update process.

How it works
------------

The hn70ap board features an on-board 64 Mb (8 Mbytes) flash memory that is
logically split into two partitions:

 * a 2 MB firmware update partition,
 * a 6 MB a storage partition.

(Note: The next revision will hold two flash chip, and a kind of "RAID0" driver
will be used to aggregate the storage partition and the memory on a secondary
flash.)

Some firmware tools are available to download a firmware update image and store
it in this external flash. The download process can happen via Ethernet, serial,
or serial in the bootloader mode (failsafe recovery mode).

The internal STM32 flash memory is also logically split into two parts:
 * the bootloader, occupies the first sector (16kB) of the internal STM32 flash
(address range 0x08000000 - 0x08004000).
 * the user software, in the other sectors (starting at 0x08004000)

What this means is that, instead of directly starting to execute the user
software, the CPU always starts to execute inside the bootloader. The bootloader
runs some update process described later, then remaps the CPU vector table to
the beginning of the user software using the ARM VTOR register, setups the
stack pointer defined in the zeroth vector, and jumps to the address stored in
the first vector, as would be done by the CPU startup procedure.

In the original NuttX firmware, this address map is ensured by the project's
linker script, that justs makes the linker believe that the main .text section
starts at this 0x08004000 address instead of 0x08000000. No other change is
required in the NuttX RTOS. If you want to use a custom firmware, you have to
make sure that you skip the first 16 kbytes of the flash.

Once the firmware update image has landed in the external flash, the bootloader
can check that an update is present by looking at the image header in the first
256 bytes of the external flash. The first 4 bytes are the CRC32 of the next
252 bytes. If the image header is found valid, the update stored in the
following pages is programmed in the internal STM32 flash. If not valid, the
update partition is wiped.

The bootloader itself writes protects itself during its startup process, so it
cannot be erased inadvertently. If a bug is found during the life of the
product, the bootloader will be forced to update via a specific process.

Firmware update image format
----------------------------

The following paragraph describes the layout of the firmware update image.

The image has two parts:
 * a 16k header (that replaces the bootloader contents, which is not stored in
the firmware update image)
 * a variable length memory image

Header
------

The first 256 bytes of the update image contains critical information related
to the update. The first 4 bytes are the CRC-32 (ZIP or GZIP algorithm) of the
next 252 bytes. If the CRC is not valid, the update is not applied by the
bootloader.
The next 252 bytes contain a sequence of TLV elements. TLV elements are a
nestable binary data structure similar to BER or DER, but simpler. TLVs have
3 fields:
 * a TAG: actually a bit field. It indicates a tag number, a tag class, a bit
indicating if the tag data field is raw or contains more tag, and a bit that
indicates wether more bytes are used to complete the tag.
* a LENGTH: this one describes how many bytes are in the payload (value).
* a VALUE: can be raw bytes or nested tags according to the TAG.

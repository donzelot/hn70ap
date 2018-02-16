BUILD CONFIGURATIONS

This directory contain canned build configurations for the hn70ap board. Each of
them starts with a number that represents a test order.

0_boot: This configuration does nothing more than a hello world. It will blink
the HEARTBEAT LED and stop here. NuttX shell will not start. This can be used
to check that the CPU is booting using the external 20 MHz XTAL. This
configuration does not contain a bootloader, so it can only be installed via
JTAG, and it cannot be upgraded.

1_memories: This configuration enables the external EEPROM and FLASH memories.
Then it tries to read the SST26 unique identifier and the MAC address stored
in the EEPROM. This configuration does not contain a bootloader and can only
be installed via JTAG, and it cannot be upgraded.

2_oled: This configuration contains initialization code and drivers for a small
OLED screen on the i2c bus. It contains a "fb" example application that will
draw some rectangles on the screen. It also initializes the external memories.

3_updates: This configuration contains the full bootloader and an application
that can receive a software update from the serial console, then write it to
the bootloader partition, then reboot. After the reboot, the bootloader will
check the validity of the contents in the external flash, and if an update is
present, it will copy it into the internal STM32 flash memory, then transfer
control to it. You can use the hn70ap_makeupdate utility to build an updater
image to be sent to the board. You can use the hn70ap_serialupdate utility
to transfer the update to the board.

4_ethernet: This configuration will attempt to initialize the external PHY and
internal MAC. Then a daemon is started in the background to manage link up/down
events, and start a DHCP request when the link is up.

5_radios: This configuration will attempt to detect and initialize radio modules
connected to the board. It requires a specific branch of the NuttX code that
contains the proper radio driver. More informations about that will be added in
the future.

Other configurations assume a fully functional board, and are made for real
hn70ap applications, eg router, access point, freedv hotspot.
They will be defined later.

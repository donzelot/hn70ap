Circuit board for hn70ap
========================

This folder contains the schematic of the KiCAD board used by the hn70ap
project.

It is released under the CERN OHL.

The project is completely standalone, it does not depend on ANY external
schematic library.

To open it you need kicad version 4.0.7.

Enclosure
=========
Any enclosure that can host a 80x100mm PCBs with connectors on both 100mm
sides is usable, for example a slotted extrusion box. I am using a
Multicomp MCREBS80 box from Farnell, which is in fact a Box Enclosures B2-080.
https://www.boxenclosures.com/product-category/bex-series-anodized-extruded-aluminum-enclosures/bex-series-2/


Power supply
============
 * Input via Anderson PowerPole for standard 13.8V ham
 * LTC3646 Creates a 5V 1A rail
 * Each component (radios, CPU, ethernet) has a ferrite bead then big storage
capacitor followed bu 3V3 LDO. Radios can reach 100mA each, Ethernet is max
40 mA, CPU is much lower.
 * When using the RF4463 1W module instead of the RFM26, we need a clean 5V
regulated with a LDO, so the LTC is configured for 5.5V instead.
 
Clocks
======
 * Ethernet PHY has a 25 MHx XTAL that gets doubled in the PHY to generate the
RMII reference clock
 * SI4463 has a 30 MHz XTAL for its own use, but a footprint is available for a
5x3.2mm TCXO via an UFL, which can also be used to feed an external reference.
 * CPU HSE is connected to a 20 MHz xtal to reach its max clock speed of 180 MHz
with correct accuracy
 * CPU 32 kHz clock is connected to a cylindrical watch xtal

Another design that I know has the 25 MHz xtal on the cpu, then feeds a 25 MHz
MCO line to the PHY, which doubles it and feeds it back to the RMII clock input.
Weird CPU delay loop calculations, long RF lines, not good.

CPU peripheral connections
==========================

Ethernet
--------

The STM32 Ethernet Peripheral is connected to a KSZ8081RNA PHY via RMII. 50 MHz
RMII Clocking is provided via a 25 MHz XTAL doubled by the PHY. Current status:
soldering failed, waiting for a new device to install.

```
RX0    PC4/33
RX1    PC5/34
CRSDV  PA7/32
TX0    PB12/51
TX1    PB13/52
TXEN   PB11/48
REFCLK PA1/24
IRQ    PE12/43
MDIO   PA2/25
MDC    PC1/16
```

Some GPIOs are used to indicate the link status and reset the PHY:

```
MAC_LINK PD8/55 Open Drain output for a LED
MAC_RST  PB0/35
```

Debug UART
----------
This uart is used to display the NuttX console. It is connected to UART4 (not an
USART). It runs at 230400 bauds, 8N1. The FTDI chip is powered via USB only, so
UART LEDs are not powered when USB is not connected. Current status: Hardware
working, but current PCB footprint has an issue.

```
TXD4  PC10/78 (AF8)
RXD4  PC11/79 (AF8)
```

SPI Flash
---------
This memory is used to store the firmware update to be programmed by the
bootloader, and a flash filesystem for file storage. It is connected to bus
SPI2. Current status: Validated in hardware and software.

NuttX creates two partitions on it:
* a 2 MB partition for firmware updates
* a 6 MB partition for data storage

```
MOSI2  PB15/54 (AF5)
MISO2  PB14/53 (AF5)
SCLK2  PB10/47 (AF5) - via pin PD10/57 to help routing
CS     PA9/68
```

I2C EEPROM
----------
This memory stores the Ethernet MAC address and generic non volatile parameters
that must survive reset. The device is connected to I2C3. The I2C bus is also
available on a pin header, can be used to add some daugterboards. The pinout is
compatible with standard I2C OLED screens based on the SSD1306 controller. Some
configurations enable support for this screen so debug information can be
displayed. Current status: Validated in hardware and software.

```
SDA3  PC9/66 (AF4)
SCL3  PA8/67 (AF4)
```

Main radio transceiver
----------------------
The main radio transceiver is a si4463 with separate TX and RX signals along
with commutation signals in the form of 5V DC bias on the RF lines. It can be
connected to external RF hardware (LNA/PA/Switch). Using BOM components, it is
RF matched for the 430-440 MHz frequency band, but you can tweak components for
any frequency of your choice (have a look at Silabs app notes). The device is
connected to SPI4.

```
MOSI4  PE6/5 (AF5)
MISO4  PE5/4 (AF5)
SCLK4  PE2/1 (AF5)
CS     PE4/3
IRQ    PC13/7
SDN    PE3/2 Used to reset BOTH radio chips in a clean way
```

Auxiliary radio transceiver
---------------------------
The auxiliary radio transceiver is either an off-the-shelf RFM26W from HopeRF
(20 dBm output power) or a RF4463F30 (high power, 30 dBm). These are also based
on the Silabs si4463. The matching is on-board and there are several frequency
bands available. The device shares the same SPI4 bus as the main transceiver.
SDN line is shared with the main transceiver, IRQs are separate.

```
CS    PA3/26
IRQ   PA6/31
```

Power supply
------------
The switching regulator provides a PGOOD output that is asserted while the
output voltage is within a 5% tolerance. This can be used to prevent writes to
the external SP flash if the power is about to fail.

```
PGOOD  PC0/15
```

LEDs
----
Informative LEDs are available. They are connected to the supply voltage and
their GPIO pins are configured as open drain, except for the bicolor LED.
Signification of external LEDs may change in the future.

```
D301 (Panel)    PB9/96 + PE0/97 (Bicolor Green/Red - Status)
D302 (Panel)    PB8/95  (Blue - Clients connected)
D303 (Panel)    PB7/93  (Orange - Transmit)
D304 (Panel)    PB6/92  (Green - Receive)
D305 (Internal) PD15/62 (Ay color - Heartbeat)
D306 (Internal) PD11/58 (Any color - CPU Activity)
J201 (Ethernet) PD8/55  (Ethernet link status)
```

These LEDs are not driven by the CPU:

```
D602 (Power)    ---     (Blue)
D401 (Internal) ---     (Any color - FTDI TX)
D402 (Internal) ---     (Any color - FTDI RX)
J201 (Ethernet) ---     (Ethernet activity)
```

Button
------
A push button with a 100nF debounce to ground and a pull-up is available for misc functions.
If the button is pressed at power up, the bootloader will enter the serial download mode without
even looking at the external flash contents.

```
BUTTON PE11/42
```

eof

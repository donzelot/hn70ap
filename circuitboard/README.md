Circuit board for hn70ap
========================

This folder contains the KiCAD board used by the hn70ap project.

It is released under the CERN OHL.

The project is completely standalone, it does not depend on ANY external schematic or PCB library.

To open it you need a very recent kicad version.

Power supply
============
 * Input via Anderson PowerPole for standard 13.8V ham
 * LTC3646 Creates a 5V 1A rail
 * Each component (radios, CPU, ethernet) has a ferrite bead then big storage capacitor followed bu 3V3 LDO. Radios can reach 100mA each, Ethernet is max 40 mA, CPU is much lower.
 
Clocks
======
 * Ethernet PHY has a 25 MHx XTAL that gets doubled to generate the RMII reference clock
 * SI4463 has a 30 MHz XTAL for its own use
 * CPU has no xtal yet, could use a 20 MHz xtal to reach its max clock speed of 180 MHz.

Another design that I know has the 25 MHz xtal on the cpu, then feeds a 25 MHz MCO line to the PHY, which doubles it and feeds it back to the RMII clock input. Weird delay loop calculations, long RF lines, not good. 20 MHz provides easier delay calculations.

CPU peripheral connections
==========================

Ethernet
--------

The STM32 Ethernet Peripheral is connected to a KSZ8081RNA PHY via RMII. 50 MHz RMII Clocking is provided via a 25 MHz XTAL doubled by the PHY.
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

Some GPIOs are used to indicate the link status and reset the PHY
```
MAC_LINK PD8/55
MAC_RST  PB0/35
```

Debug UART
----------
This uart is used to display the NuttX console. It is connected to UART4 (not an USART). It runs at 230400 bauds.

```
TXD4  PC10/78
RXD4  PC11/79
```

SPI Flash
---------
This memory is used to store the firmware update to be programmed by the bootloader, and a flash filesystem for file storage. It is connected to bus SPI2.
```
MOSI2  PB15/54
MISO2  PB14/53
SCLK2  PB10/47 via pin PD10/57 to help routing
CS     PA9/68
```

I2C EEPROM
----------
This memory stores the Ethernet MAC address and generic non volatile parameters that must survive reset. The device is connected to I2C3.
```
SDA3  PC9/66
SCL3  PA8/67
```

Main radio transceiver
----------------------
The main radio transceiver is a si4463 with separate TX and RX signals along with a PTT. It can be connected to external RF hardware (LNA/PA/Switch). It is RF matched for the 430-440 MHz frequency band. The device is connected to SPI4.
```
MOSI4  PE6/5
MISO4  PE5/4
SCLK4  PE2/1
CS     Pxx TODO
IRQ    Pxx TODO
```

Secondary radio transceiver
---------------------------
The secondary radio transceiver is an off-the-shelf RFM26W from HopeRF. It is also based on the Silabs si4463. The matching is on-board and there are several frequency bands available. The device shares the same SPI4 bus as the main transceiver
```
CS    Pxx TODO
IRQ   Pxx TODO
```
Power supply
------------
The switching regulator provides a PGOOD output that is asserted while the output voltage is within a 5% tolerance.
```
PGOOD  PC0/15
```

LEDs
----
Some LEDs will be added later. They are connected to the supply voltage so outputs can be configured as open drain.
```
LED1  Pxx TODO
```

eof

EESchema Schematic File Version 2
LIBS:apdep
LIBS:ap-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 6
Title "Radio gateway (FSK radio)"
Date ""
Rev "1"
Comp "F4GRX"
Comment1 "CERN OHL"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2250 2350 0    45   Input ~ 0
RADIO_CS
Text HLabel 2250 2250 0    45   Input ~ 0
RADIO_MOSI
Text HLabel 2250 2150 0    45   Input ~ 0
RADIO_MISO
Text HLabel 2250 2050 0    45   Input ~ 0
RADIO_SCLK
Text HLabel 2250 1950 0    45   Input ~ 0
RADIO_IRQ
Text HLabel 3650 2750 3    45   Input ~ 0
RADIO_SDN
$Comp
L GND #PWR072
U 1 1 591F9FB0
P 2950 2750
F 0 "#PWR072" H 2950 2750 30  0001 C CNN
F 1 "GND" H 2950 2680 30  0001 C CNN
F 2 "" H 2950 2750 60  0000 C CNN
F 3 "" H 2950 2750 60  0000 C CNN
	1    2950 2750
	1    0    0    -1  
$EndComp
NoConn ~ 3550 1950
$Comp
L XTAL X501
U 1 1 591FA494
P 2200 3350
F 0 "X501" H 2200 3670 45  0000 C CNN
F 1 "30 MHz" H 2200 3586 45  0000 C CNN
F 2 "apdep:XTAL_SMD_3.2x2.5" H 2200 3350 60  0001 C CNN
F 3 "" H 2200 3350 60  0000 C CNN
	1    2200 3350
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C503
U 1 1 591FA68E
P 2400 1300
F 0 "C503" H 2478 1330 30  0000 L CNN
F 1 "100p" H 2478 1270 30  0000 L CNN
F 2 "apdep:0603_m1608" H 2400 1300 60  0001 C CNN
F 3 "" H 2400 1300 60  0000 C CNN
	1    2400 1300
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C502
U 1 1 591FA6EA
P 2100 1300
F 0 "C502" H 2178 1330 30  0000 L CNN
F 1 "100n" H 2178 1270 30  0000 L CNN
F 2 "apdep:0603_m1608" H 2100 1300 60  0001 C CNN
F 3 "" H 2100 1300 60  0000 C CNN
	1    2100 1300
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C501
U 1 1 591FA86A
P 1800 1300
F 0 "C501" H 1878 1330 30  0000 L CNN
F 1 "1u" H 1878 1270 30  0000 L CNN
F 2 "apdep:0805_m2012" H 1800 1300 60  0001 C CNN
F 3 "" H 1800 1300 60  0000 C CNN
	1    1800 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR073
U 1 1 591FA8BD
P 1800 1400
F 0 "#PWR073" H 1800 1400 30  0001 C CNN
F 1 "GND" H 1800 1330 30  0001 C CNN
F 2 "" H 1800 1400 60  0000 C CNN
F 3 "" H 1800 1400 60  0000 C CNN
	1    1800 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR074
U 1 1 591FA8DE
P 2100 1400
F 0 "#PWR074" H 2100 1400 30  0001 C CNN
F 1 "GND" H 2100 1330 30  0001 C CNN
F 2 "" H 2100 1400 60  0000 C CNN
F 3 "" H 2100 1400 60  0000 C CNN
	1    2100 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR075
U 1 1 591FA8F8
P 2400 1400
F 0 "#PWR075" H 2400 1400 30  0001 C CNN
F 1 "GND" H 2400 1330 30  0001 C CNN
F 2 "" H 2400 1400 60  0000 C CNN
F 3 "" H 2400 1400 60  0000 C CNN
	1    2400 1400
	1    0    0    -1  
$EndComp
$Comp
L LSMALL L501
U 1 1 591FAA98
P 3700 1550
F 0 "L501" V 3730 1465 30  0000 R CNN
F 1 "LSMALL" V 3670 1465 30  0000 R CNN
F 2 "apdep:0402_m1005" H 3700 1550 60  0001 C CNN
F 3 "" H 3700 1550 60  0000 C CNN
	1    3700 1550
	0    -1   -1   0   
$EndComp
$Comp
L CSMALL C505
U 1 1 591FACE1
P 3950 1800
F 0 "C505" V 3782 1800 30  0000 C CNN
F 1 "100p" V 3842 1800 30  0000 C CNN
F 2 "apdep:0402_m1005" H 3950 1800 60  0001 C CNN
F 3 "" H 3950 1800 60  0000 C CNN
	1    3950 1800
	0    1    1    0   
$EndComp
$Comp
L LSMALL L503
U 1 1 591FAD6B
P 4300 1800
F 0 "L503" H 4300 1625 30  0000 C CNN
F 1 "LSMALL" H 4300 1685 30  0000 C CNN
F 2 "apdep:0402_m1005" H 4300 1800 60  0001 C CNN
F 3 "" H 4300 1800 60  0000 C CNN
	1    4300 1800
	-1   0    0    1   
$EndComp
$Comp
L CSMALL C507
U 1 1 591FADEF
P 4550 2000
F 0 "C507" H 4628 2030 30  0000 L CNN
F 1 "100p" H 4628 1970 30  0000 L CNN
F 2 "apdep:0402_m1005" H 4628 1940 60  0001 L CNN
F 3 "" H 4550 2000 60  0000 C CNN
	1    4550 2000
	1    0    0    -1  
$EndComp
$Comp
L LSMALL L504
U 1 1 591FAE6B
P 4800 1800
F 0 "L504" H 4800 1625 30  0000 C CNN
F 1 "LSMALL" H 4800 1685 30  0000 C CNN
F 2 "apdep:0402_m1005" H 4800 1800 60  0001 C CNN
F 3 "" H 4800 1800 60  0000 C CNN
	1    4800 1800
	-1   0    0    1   
$EndComp
$Comp
L CSMALL C509
U 1 1 591FAEFA
P 5050 2000
F 0 "C509" H 5128 2030 30  0000 L CNN
F 1 "100p" H 5128 1970 30  0000 L CNN
F 2 "apdep:0402_m1005" H 5050 2000 60  0001 C CNN
F 3 "" H 5050 2000 60  0000 C CNN
	1    5050 2000
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C510
U 1 1 591FAF61
P 5250 1800
F 0 "C510" V 5082 1800 30  0000 C CNN
F 1 "100p" V 5142 1800 30  0000 C CNN
F 2 "apdep:0402_m1005" H 5250 1800 60  0001 C CNN
F 3 "" H 5250 1800 60  0000 C CNN
	1    5250 1800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR076
U 1 1 591FAFE4
P 4550 2100
F 0 "#PWR076" H 4550 2100 30  0001 C CNN
F 1 "GND" H 4550 2030 30  0001 C CNN
F 2 "" H 4550 2100 60  0000 C CNN
F 3 "" H 4550 2100 60  0000 C CNN
	1    4550 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR077
U 1 1 591FB013
P 5050 2100
F 0 "#PWR077" H 5050 2100 30  0001 C CNN
F 1 "GND" H 5050 2030 30  0001 C CNN
F 2 "" H 5050 2100 60  0000 C CNN
F 3 "" H 5050 2100 60  0000 C CNN
	1    5050 2100
	1    0    0    -1  
$EndComp
$Comp
L CSMALL CR1
U 1 1 591FB248
P 3950 3250
F 0 "CR1" H 4028 3280 30  0000 L CNN
F 1 "4p7" H 4028 3220 30  0000 L CNN
F 2 "apdep:0603_m1608" H 4028 3190 60  0001 L CNN
F 3 "" H 3950 3250 60  0000 C CNN
	1    3950 3250
	1    0    0    -1  
$EndComp
$Comp
L LSMALL LR1
U 1 1 591FB2F5
P 4150 2700
F 0 "LR1" H 4150 2525 30  0000 C CNN
F 1 "62n" H 4150 2585 30  0000 C CNN
F 2 "apdep:0603_m1608" H 4150 2700 60  0001 C CNN
F 3 "" H 4150 2700 60  0000 C CNN
	1    4150 2700
	-1   0    0    1   
$EndComp
$Comp
L CSMALL CR2
U 1 1 591FB4E7
P 4550 2700
F 0 "CR2" V 4382 2700 30  0000 C CNN
F 1 "2p2" V 4442 2700 30  0000 C CNN
F 2 "apdep:0603_m1608" H 4419 2700 60  0001 C CNN
F 3 "" H 4550 2700 60  0000 C CNN
	1    4550 2700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR078
U 1 1 591FB596
P 3950 3350
F 0 "#PWR078" H 3950 3350 30  0001 C CNN
F 1 "GND" H 3950 3280 30  0001 C CNN
F 2 "" H 3950 3350 60  0000 C CNN
F 3 "" H 3950 3350 60  0000 C CNN
	1    3950 3350
	1    0    0    -1  
$EndComp
Text Label 5450 2700 0    45   ~ 0
RFRX
Text Label 5450 1800 0    45   ~ 0
RFTX
$Comp
L SMA J501
U 1 1 591FBF42
P 5900 1800
F 0 "J501" H 5999 1796 45  0000 L CNN
F 1 "SMA" H 5999 1712 45  0000 L CNN
F 2 "apdep:SMA_PINS" H 5900 1800 60  0001 C CNN
F 3 "" H 5900 1800 60  0000 C CNN
	1    5900 1800
	1    0    0    -1  
$EndComp
$Comp
L SMA J502
U 1 1 591FC039
P 5900 2700
F 0 "J502" H 5999 2696 45  0000 L CNN
F 1 "SMA" H 5999 2612 45  0000 L CNN
F 2 "apdep:SMA_PINS" H 5900 2700 60  0001 C CNN
F 3 "" H 5900 2700 60  0000 C CNN
	1    5900 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR079
U 1 1 591FC0DA
P 5900 1950
F 0 "#PWR079" H 5900 1950 30  0001 C CNN
F 1 "GND" H 5900 1880 30  0001 C CNN
F 2 "" H 5900 1950 60  0000 C CNN
F 3 "" H 5900 1950 60  0000 C CNN
	1    5900 1950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR080
U 1 1 591FC118
P 5900 2850
F 0 "#PWR080" H 5900 2850 30  0001 C CNN
F 1 "GND" H 5900 2780 30  0001 C CNN
F 2 "" H 5900 2850 60  0000 C CNN
F 3 "" H 5900 2850 60  0000 C CNN
	1    5900 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR081
U 1 1 591FC6C6
P 2450 3450
F 0 "#PWR081" H 2450 3450 30  0001 C CNN
F 1 "GND" H 2450 3380 30  0001 C CNN
F 2 "" H 2450 3450 60  0000 C CNN
F 3 "" H 2450 3450 60  0000 C CNN
	1    2450 3450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR082
U 1 1 591FC737
P 1950 3250
F 0 "#PWR082" H 1950 3250 30  0001 C CNN
F 1 "GND" H 1950 3180 30  0001 C CNN
F 2 "" H 1950 3250 60  0000 C CNN
F 3 "" H 1950 3250 60  0000 C CNN
	1    1950 3250
	0    1    1    0   
$EndComp
Text Label 1400 1100 0    45   ~ 0
VRADIO
Text Label 3600 2050 0    45   ~ 0
TX
Text Label 3600 2150 0    45   ~ 0
RXN
Text Label 3600 2250 0    45   ~ 0
RXP
Text Label 2550 3350 0    45   ~ 0
XI
Text Label 2550 2950 0    45   ~ 0
XO
Text HLabel 3050 3250 3    45   Input ~ 0
RADIO_GP0
Text HLabel 3150 3250 3    45   Input ~ 0
RADIO_GP1
$Comp
L RFM26W U501
U 1 1 593E4461
P 2350 5400
F 0 "U501" H 2325 6003 60  0000 C CNN
F 1 "RFM26W" H 2325 5897 60  0000 C CNN
F 2 "apdep:RFM26W" H 2350 5550 60  0001 C CNN
F 3 "" H 2350 5550 60  0000 C CNN
	1    2350 5400
	1    0    0    -1  
$EndComp
Text Notes 600  7650 0    300  ~ 60
FSK Radio
Text Notes 7050 7050 0    45   ~ 0
Two options:\n- simple RFM26 module\n- direct use of si4463
Text HLabel 1750 5450 0    45   Input ~ 0
RADIO_CS2
Text HLabel 1750 5250 0    45   Input ~ 0
RADIO_MOSI
Text HLabel 1750 5150 0    45   Input ~ 0
RADIO_MISO
Text HLabel 1750 5350 0    45   Input ~ 0
RADIO_SCLK
Text HLabel 1750 5550 0    45   Input ~ 0
RADIO_IRQ
Text Label 3600 5350 0    45   ~ 0
VRFM
$Comp
L GND #PWR083
U 1 1 593E5C21
P 2800 5650
F 0 "#PWR083" H 2800 5650 30  0001 C CNN
F 1 "GND" H 2800 5580 30  0001 C CNN
F 2 "" H 2800 5650 60  0000 C CNN
F 3 "" H 2800 5650 60  0000 C CNN
	1    2800 5650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR084
U 1 1 593E5E5C
P 1850 5750
F 0 "#PWR084" H 1850 5750 30  0001 C CNN
F 1 "GND" H 1850 5680 30  0001 C CNN
F 2 "" H 1850 5750 60  0000 C CNN
F 3 "" H 1850 5750 60  0000 C CNN
	1    1850 5750
	0    1    1    0   
$EndComp
$Comp
L SMA J503
U 1 1 593E5F82
P 4550 5750
F 0 "J503" H 4649 5746 45  0000 L CNN
F 1 "SMA" H 4649 5662 45  0000 L CNN
F 2 "apdep:SMA_PINS" H 4550 5750 60  0001 C CNN
F 3 "" H 4550 5750 60  0000 C CNN
	1    4550 5750
	1    0    0    -1  
$EndComp
Text Label 3700 5750 0    45   ~ 0
RFIO
$Comp
L GND #PWR085
U 1 1 593E62D4
P 4550 5900
F 0 "#PWR085" H 4550 5900 30  0001 C CNN
F 1 "GND" H 4550 5830 30  0001 C CNN
F 2 "" H 4550 5900 60  0000 C CNN
F 3 "" H 4550 5900 60  0000 C CNN
	1    4550 5900
	1    0    0    -1  
$EndComp
Text HLabel 1750 5050 0    45   Input ~ 0
RADIO_GP1
Text HLabel 2900 5050 2    45   Input ~ 0
RADIO_GP0
$Comp
L CSMALL C504
U 1 1 593E69D2
P 3350 5500
F 0 "C504" H 3428 5530 30  0000 L CNN
F 1 "100n" H 3428 5470 30  0000 L CNN
F 2 "apdep:0603_m1608" H 3428 5440 60  0001 L CNN
F 3 "" H 3350 5500 60  0000 C CNN
	1    3350 5500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR086
U 1 1 593E6B05
P 3350 5600
F 0 "#PWR086" H 3350 5600 30  0001 C CNN
F 1 "GND" H 3350 5530 30  0001 C CNN
F 2 "" H 3350 5600 60  0000 C CNN
F 3 "" H 3350 5600 60  0000 C CNN
	1    3350 5600
	1    0    0    -1  
$EndComp
Text HLabel 1750 5650 0    45   Input ~ 0
RADIO_SDN
$Comp
L CSMALL C511
U 1 1 594172C4
P 7400 900
F 0 "C511" V 7232 900 30  0000 C CNN
F 1 "1n" V 7292 900 30  0000 C CNN
F 2 "apdep:0402_m1005" H 7269 900 60  0001 C CNN
F 3 "" H 7400 900 60  0000 C CNN
	1    7400 900 
	-1   0    0    1   
$EndComp
$Comp
L CSMALL C512
U 1 1 5941734A
P 7400 1400
F 0 "C512" V 7232 1400 30  0000 C CNN
F 1 "1n" V 7292 1400 30  0000 C CNN
F 2 "apdep:0402_m1005" H 7269 1400 60  0001 C CNN
F 3 "" H 7400 1400 60  0000 C CNN
	1    7400 1400
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR087
U 1 1 59417555
P 7400 800
F 0 "#PWR087" H 7400 800 30  0001 C CNN
F 1 "GND" H 7400 730 30  0001 C CNN
F 2 "" H 7400 800 60  0000 C CNN
F 3 "" H 7400 800 60  0000 C CNN
	1    7400 800 
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR088
U 1 1 594175D8
P 7400 1500
F 0 "#PWR088" H 7400 1500 30  0001 C CNN
F 1 "GND" H 7400 1430 30  0001 C CNN
F 2 "" H 7400 1500 60  0000 C CNN
F 3 "" H 7400 1500 60  0000 C CNN
	1    7400 1500
	1    0    0    -1  
$EndComp
$Comp
L SI4463 U502
U 1 1 59424AE8
P 2950 2150
F 0 "U502" H 3900 1700 60  0000 C CNN
F 1 "SI4463" H 3850 1600 60  0000 C CNN
F 2 "" H 2950 2300 60  0000 C CNN
F 3 "" H 2950 2300 60  0000 C CNN
	1    2950 2150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR089
U 1 1 59424BBE
P 2350 1750
F 0 "#PWR089" H 2350 1750 30  0001 C CNN
F 1 "GND" H 2350 1680 30  0001 C CNN
F 2 "" H 2350 1750 60  0000 C CNN
F 3 "" H 2350 1750 60  0000 C CNN
	1    2350 1750
	0    1    1    0   
$EndComp
Text Label 2800 5450 0    45   ~ 0
TXANT
Text Label 2800 5550 0    45   ~ 0
RXANT
$Comp
L CONN_01X04 P501
U 1 1 594418CB
P 10400 1400
F 0 "P501" H 10478 1438 50  0000 L CNN
F 1 "CONN_01X04" H 10478 1347 50  0000 L CNN
F 2 "apdep:PINS-2.54-1x4" H 10400 1400 60  0001 C CNN
F 3 "" H 10400 1400 60  0000 C CNN
	1    10400 1400
	1    0    0    -1  
$EndComp
$Comp
L MOSN_SMALL T501
U 1 1 59441AFF
P 9300 1050
F 0 "T501" H 9493 1092 45  0000 L CNN
F 1 "BS170" H 9493 1008 45  0000 L CNN
F 2 "apdep:SOT23GDS" H 9300 1050 60  0001 C CNN
F 3 "" H 9300 1050 60  0000 C CNN
	1    9300 1050
	1    0    0    -1  
$EndComp
$Comp
L MOSN_SMALL T502
U 1 1 5944202D
P 9300 1700
F 0 "T502" H 9493 1742 45  0000 L CNN
F 1 "BS170" H 9493 1658 45  0000 L CNN
F 2 "apdep:SOT23GDS" H 9300 1700 60  0001 C CNN
F 3 "" H 9300 1700 60  0000 C CNN
	1    9300 1700
	1    0    0    -1  
$EndComp
$Comp
L RSMALL R501
U 1 1 594420A4
P 8100 1000
F 0 "R501" H 8100 1050 30  0000 C CNN
F 1 "100k" H 8100 1000 30  0000 C CNN
F 2 "apdep:0402_m1005" H 8100 1000 60  0001 C CNN
F 3 "" H 8100 1000 60  0000 C CNN
	1    8100 1000
	1    0    0    -1  
$EndComp
$Comp
L RSMALL R502
U 1 1 594425C7
P 8100 1300
F 0 "R502" H 8100 1350 30  0000 C CNN
F 1 "100k" H 8100 1300 30  0000 C CNN
F 2 "apdep:0402_m1005" H 8100 1300 60  0001 C CNN
F 3 "" H 8100 1300 60  0000 C CNN
	1    8100 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR090
U 1 1 5944270E
P 8250 1300
F 0 "#PWR090" H 8250 1300 30  0001 C CNN
F 1 "GND" H 8250 1230 30  0001 C CNN
F 2 "" H 8250 1300 60  0000 C CNN
F 3 "" H 8250 1300 60  0000 C CNN
	1    8250 1300
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR091
U 1 1 594427BB
P 8250 1000
F 0 "#PWR091" H 8250 1000 30  0001 C CNN
F 1 "GND" H 8250 930 30  0001 C CNN
F 2 "" H 8250 1000 60  0000 C CNN
F 3 "" H 8250 1000 60  0000 C CNN
	1    8250 1000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR092
U 1 1 59442A80
P 9400 1200
F 0 "#PWR092" H 9400 1200 30  0001 C CNN
F 1 "GND" H 9400 1130 30  0001 C CNN
F 2 "" H 9400 1200 60  0000 C CNN
F 3 "" H 9400 1200 60  0000 C CNN
	1    9400 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR093
U 1 1 59442AF7
P 9400 1850
F 0 "#PWR093" H 9400 1850 30  0001 C CNN
F 1 "GND" H 9400 1780 30  0001 C CNN
F 2 "" H 9400 1850 60  0000 C CNN
F 3 "" H 9400 1850 60  0000 C CNN
	1    9400 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR094
U 1 1 59442CD5
P 10200 1550
F 0 "#PWR094" H 10200 1550 30  0001 C CNN
F 1 "GND" H 10200 1480 30  0001 C CNN
F 2 "" H 10200 1550 60  0000 C CNN
F 3 "" H 10200 1550 60  0000 C CNN
	1    10200 1550
	0    1    1    0   
$EndComp
Text Label 10100 1050 1    45   ~ 0
VRADIO
Text Label 9750 800  0    45   ~ 0
PASW
Text Label 9750 1450 0    45   ~ 0
LNASW
Text Label 6250 1100 0    45   ~ 0
PAEN
Text Label 6250 1200 0    45   ~ 0
LNAEN
$Comp
L LSMALL LR2
U 1 1 595C21CE
P 4150 3050
F 0 "LR2" H 4150 2875 30  0000 C CNN
F 1 "56n" H 4150 2935 30  0000 C CNN
F 2 "apdep:0603_m1608" H 4150 3050 60  0001 C CNN
F 3 "" H 4150 3050 60  0000 C CNN
	1    4150 3050
	-1   0    0    1   
$EndComp
Text HLabel 6650 3000 0    45   Input ~ 0
5VIN
$Comp
L ADP151 U503
U 1 1 595FBBC6
P 7750 3050
F 0 "U503" H 7750 3304 45  0000 C CNN
F 1 "ADP151" H 7750 3220 45  0000 C CNN
F 2 "apdep:SOT23-6" H 7750 3050 60  0001 C CNN
F 3 "" H 7750 3050 60  0000 C CNN
	1    7750 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR095
U 1 1 597A8ADE
P 7750 3250
F 0 "#PWR095" H 7750 3250 30  0001 C CNN
F 1 "GND" H 7750 3180 30  0001 C CNN
F 2 "" H 7750 3250 60  0000 C CNN
F 3 "" H 7750 3250 60  0000 C CNN
	1    7750 3250
	1    0    0    -1  
$EndComp
Text Label 8300 3000 0    45   ~ 0
VRADIO
$Comp
L CSMALL C506
U 1 1 597A8DB9
P 8150 3300
F 0 "C506" H 8228 3330 30  0000 L CNN
F 1 "10n" H 8228 3270 30  0000 L CNN
F 2 "apdep:0603_m1608" H 8228 3240 60  0001 L CNN
F 3 "" H 8150 3300 60  0000 C CNN
	1    8150 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR096
U 1 1 597A9032
P 8150 3400
F 0 "#PWR096" H 8150 3400 30  0001 C CNN
F 1 "GND" H 8150 3330 30  0001 C CNN
F 2 "" H 8150 3400 60  0000 C CNN
F 3 "" H 8150 3400 60  0000 C CNN
	1    8150 3400
	1    0    0    -1  
$EndComp
$Comp
L FB FB501
U 1 1 597E667D
P 6950 3000
F 0 "FB501" H 6950 3168 30  0000 C CNN
F 1 "FB" H 6950 3108 30  0000 C CNN
F 2 "apdep:0805_m2012" H 6950 3000 60  0001 C CNN
F 3 "" H 6950 3000 60  0000 C CNN
	1    6950 3000
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C513
U 1 1 597E6ACE
P 7350 3300
F 0 "C513" H 7428 3330 30  0000 L CNN
F 1 "1u" H 7428 3270 30  0000 L CNN
F 2 "apdep:0805_m2012" H 7350 3300 60  0001 C CNN
F 3 "" H 7350 3300 60  0000 C CNN
	1    7350 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2350 3550 2350
Wire Wire Line
	2250 2350 2350 2350
Wire Wire Line
	2250 2250 2350 2250
Wire Wire Line
	2250 1950 2350 1950
Wire Wire Line
	2250 2050 2350 2050
Wire Wire Line
	2250 2150 2350 2150
Wire Wire Line
	2950 1100 2950 1550
Wire Wire Line
	1400 1100 3700 1100
Wire Wire Line
	3150 1100 3150 1550
Connection ~ 3150 1100
Wire Wire Line
	2400 1100 2400 1200
Connection ~ 2950 1100
Wire Wire Line
	2100 1200 2100 1100
Connection ~ 2400 1100
Wire Wire Line
	1800 1100 1800 1200
Connection ~ 2100 1100
Wire Wire Line
	3700 1100 3700 1400
Wire Wire Line
	3700 2050 3550 2050
Wire Wire Line
	3700 1700 3700 2050
Wire Wire Line
	3850 1800 3700 1800
Connection ~ 3700 1800
Wire Wire Line
	4150 1800 4050 1800
Wire Wire Line
	4550 1900 4550 1800
Wire Wire Line
	4450 1800 4650 1800
Connection ~ 4550 1800
Wire Wire Line
	5050 1900 5050 1800
Wire Wire Line
	4950 1800 5150 1800
Connection ~ 5050 1800
Wire Wire Line
	3550 2150 4350 2150
Wire Wire Line
	3550 2250 3950 2250
Wire Wire Line
	3950 2250 3950 3150
Wire Wire Line
	4000 2700 3950 2700
Connection ~ 3950 2700
Wire Wire Line
	4350 2150 4350 2700
Wire Wire Line
	3650 2750 3650 2350
Wire Wire Line
	4300 2700 4450 2700
Connection ~ 4350 2700
Wire Wire Line
	4650 2700 5700 2700
Wire Wire Line
	5350 1800 5700 1800
Wire Wire Line
	3150 2750 3150 3250
Wire Wire Line
	3050 2750 3050 3250
Wire Wire Line
	5700 1100 9150 1100
Wire Wire Line
	5600 1200 8700 1200
Wire Wire Line
	2850 2750 2850 3350
Wire Wire Line
	2850 3350 2450 3350
Wire Wire Line
	2750 2750 2750 2950
Wire Wire Line
	2750 2950 1850 2950
Wire Wire Line
	1850 2950 1850 3350
Wire Wire Line
	1850 3350 1950 3350
Connection ~ 1800 1100
Wire Wire Line
	2800 5550 3100 5550
Wire Wire Line
	2900 5050 2800 5050
Wire Wire Line
	1750 5550 1850 5550
Wire Wire Line
	1750 5350 1850 5350
Wire Wire Line
	1750 5250 1850 5250
Wire Wire Line
	1750 5150 1850 5150
Wire Wire Line
	1850 5050 1750 5050
Wire Wire Line
	3000 5450 2800 5450
Wire Wire Line
	2800 5350 3600 5350
Wire Wire Line
	2800 5750 4350 5750
Wire Wire Line
	3350 5400 3350 5350
Connection ~ 3350 5350
Wire Wire Line
	1850 5650 1750 5650
Wire Wire Line
	7400 1200 7400 1300
Wire Wire Line
	7400 1100 7400 1000
Connection ~ 7400 1100
Connection ~ 7400 1200
Wire Wire Line
	3000 5450 3000 5250
Wire Wire Line
	3000 5250 2800 5250
Wire Wire Line
	3100 5550 3100 5150
Wire Wire Line
	3100 5150 2800 5150
Wire Wire Line
	7950 1000 7900 1000
Wire Wire Line
	7900 1000 7900 1100
Connection ~ 7900 1100
Wire Wire Line
	7950 1300 7900 1300
Wire Wire Line
	7900 1300 7900 1200
Connection ~ 7900 1200
Wire Wire Line
	8700 1200 8700 1750
Wire Wire Line
	8700 1750 9150 1750
Wire Wire Line
	9400 1550 9400 1450
Wire Wire Line
	9400 1450 10200 1450
Wire Wire Line
	9400 900  9400 800 
Wire Wire Line
	9400 800  10000 800 
Wire Wire Line
	10000 800  10000 1350
Wire Wire Line
	10000 1350 10200 1350
Wire Wire Line
	10200 1250 10100 1250
Wire Wire Line
	10100 1250 10100 1050
Wire Wire Line
	2850 1550 2850 950 
Wire Wire Line
	2850 950  5600 950 
Wire Wire Line
	2750 1550 2750 850 
Wire Wire Line
	2750 850  5700 850 
Wire Wire Line
	5700 850  5700 1100
Wire Wire Line
	5600 950  5600 1200
Wire Wire Line
	4000 3050 3950 3050
Connection ~ 3950 3050
Wire Wire Line
	4300 3050 4750 3050
Wire Wire Line
	4750 3050 4750 2700
Connection ~ 4750 2700
Wire Wire Line
	7050 3000 7450 3000
Wire Wire Line
	8050 3000 8300 3000
Wire Wire Line
	8150 3200 8150 3100
Wire Wire Line
	8150 3100 8050 3100
Wire Wire Line
	1750 5450 1850 5450
Wire Wire Line
	6650 3000 6850 3000
Wire Wire Line
	7450 3100 7350 3100
Wire Wire Line
	7350 3000 7350 3200
Connection ~ 7350 3000
Connection ~ 7350 3100
$Comp
L GND #PWR097
U 1 1 597E6DCA
P 7350 3400
F 0 "#PWR097" H 7350 3400 30  0001 C CNN
F 1 "GND" H 7350 3330 30  0001 C CNN
F 2 "" H 7350 3400 60  0000 C CNN
F 3 "" H 7350 3400 60  0000 C CNN
	1    7350 3400
	1    0    0    -1  
$EndComp
$Comp
L ADP151 U504
U 1 1 597E72B7
P 7750 5350
F 0 "U504" H 7750 5604 45  0000 C CNN
F 1 "ADP151" H 7750 5520 45  0000 C CNN
F 2 "apdep:SOT23-6" H 7750 5350 60  0001 C CNN
F 3 "" H 7750 5350 60  0000 C CNN
	1    7750 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR098
U 1 1 597E72BD
P 7750 5550
F 0 "#PWR098" H 7750 5550 30  0001 C CNN
F 1 "GND" H 7750 5480 30  0001 C CNN
F 2 "" H 7750 5550 60  0000 C CNN
F 3 "" H 7750 5550 60  0000 C CNN
	1    7750 5550
	1    0    0    -1  
$EndComp
Text Label 8300 5300 0    45   ~ 0
VRFM
$Comp
L CSMALL C508
U 1 1 597E72C4
P 8150 5600
F 0 "C508" H 8228 5630 30  0000 L CNN
F 1 "10n" H 8228 5570 30  0000 L CNN
F 2 "apdep:0603_m1608" H 8228 5540 60  0001 L CNN
F 3 "" H 8150 5600 60  0000 C CNN
	1    8150 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR099
U 1 1 597E72CA
P 8150 5700
F 0 "#PWR099" H 8150 5700 30  0001 C CNN
F 1 "GND" H 8150 5630 30  0001 C CNN
F 2 "" H 8150 5700 60  0000 C CNN
F 3 "" H 8150 5700 60  0000 C CNN
	1    8150 5700
	1    0    0    -1  
$EndComp
$Comp
L FB FB502
U 1 1 597E72D0
P 6950 5300
F 0 "FB502" H 6950 5468 30  0000 C CNN
F 1 "FB" H 6950 5408 30  0000 C CNN
F 2 "apdep:0805_m2012" H 6950 5300 60  0001 C CNN
F 3 "" H 6950 5300 60  0000 C CNN
	1    6950 5300
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C514
U 1 1 597E72D6
P 7350 5600
F 0 "C514" H 7428 5630 30  0000 L CNN
F 1 "1u" H 7428 5570 30  0000 L CNN
F 2 "apdep:0805_m2012" H 7350 5600 60  0001 C CNN
F 3 "" H 7350 5600 60  0000 C CNN
	1    7350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5300 7450 5300
Wire Wire Line
	8050 5300 8300 5300
Wire Wire Line
	8150 5500 8150 5400
Wire Wire Line
	8150 5400 8050 5400
Wire Wire Line
	6750 5300 6850 5300
Wire Wire Line
	7450 5400 7350 5400
Wire Wire Line
	7350 5300 7350 5500
Connection ~ 7350 5300
Connection ~ 7350 5400
$Comp
L GND #PWR0100
U 1 1 597E72E5
P 7350 5700
F 0 "#PWR0100" H 7350 5700 30  0001 C CNN
F 1 "GND" H 7350 5630 30  0001 C CNN
F 2 "" H 7350 5700 60  0000 C CNN
F 3 "" H 7350 5700 60  0000 C CNN
	1    7350 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3000 6750 5300
Connection ~ 6750 3000
Text Label 7050 3000 0    45   ~ 0
5VRADIO
Text Label 7050 5300 0    45   ~ 0
5VRFM
Text Notes 8600 5350 0    45   ~ 0
13 mA RX\n85 mA TX
Text Notes 8600 3050 0    45   ~ 0
13 mA RX\n85 mA TX
$EndSCHEMATC

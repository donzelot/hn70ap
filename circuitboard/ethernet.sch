EESchema Schematic File Version 2
LIBS:apdep
LIBS:ap-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
Title "Radio gateway (Ethernet interface)"
Date "2017-05-19"
Rev "1"
Comp "F4GRX"
Comment1 "CERN OHL"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 600  7700 0    300  ~ 60
Ethernet interface
$Comp
L KSZ8081RN U201
U 1 1 591C7F20
P 5450 3200
F 0 "U201" H 6200 3750 60  0000 C CNN
F 1 "KSZ8081RNA" H 6400 3650 60  0000 C CNN
F 2 "apdep:QFN-24-1EP_4x4mm_Pitch0.5mm" H 5300 3450 60  0001 C CNN
F 3 "" H 5300 3450 60  0000 C CNN
	1    5450 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 591C7F75
P 6200 3750
F 0 "#PWR01" H 6200 3750 30  0001 C CNN
F 1 "GND" H 6200 3680 30  0001 C CNN
F 2 "" H 6200 3750 60  0000 C CNN
F 3 "" H 6200 3750 60  0000 C CNN
	1    6200 3750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR02
U 1 1 591C7F90
P 5400 2500
F 0 "#PWR02" H 5400 2500 30  0001 C CNN
F 1 "GND" H 5400 2430 30  0001 C CNN
F 2 "" H 5400 2500 60  0000 C CNN
F 3 "" H 5400 2500 60  0000 C CNN
	1    5400 2500
	-1   0    0    1   
$EndComp
$Comp
L XTAL X201
U 1 1 591C801B
P 4450 4550
F 0 "X201" H 4450 4870 45  0000 C CNN
F 1 "25 MHz" H 4450 4786 45  0000 C CNN
F 2 "apdep:XTAL_SMD_3.2x2.5" H 4450 4550 60  0001 C CNN
F 3 "" H 4450 4550 60  0000 C CNN
	1    4450 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 591C806E
P 4700 4650
F 0 "#PWR03" H 4700 4650 30  0001 C CNN
F 1 "GND" H 4700 4580 30  0001 C CNN
F 2 "" H 4700 4650 60  0000 C CNN
F 3 "" H 4700 4650 60  0000 C CNN
	1    4700 4650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR04
U 1 1 591C807F
P 4200 4450
F 0 "#PWR04" H 4200 4450 30  0001 C CNN
F 1 "GND" H 4200 4380 30  0001 C CNN
F 2 "" H 4200 4450 60  0000 C CNN
F 3 "" H 4200 4450 60  0000 C CNN
	1    4200 4450
	0    1    1    0   
$EndComp
$Comp
L CSMALL C204
U 1 1 591C80D7
P 4100 4850
F 0 "C204" H 4178 4880 30  0000 L CNN
F 1 "22p" H 4178 4820 30  0000 L CNN
F 2 "apdep:0603_m1608" H 4100 4850 60  0001 C CNN
F 3 "" H 4100 4850 60  0000 C CNN
	1    4100 4850
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C2
U 1 1 591C8104
P 5000 4850
F 0 "C2" H 5078 4880 30  0000 L CNN
F 1 "22p" H 5078 4820 30  0000 L CNN
F 2 "apdep:0603_m1608" H 5000 4850 60  0001 C CNN
F 3 "" H 5000 4850 60  0000 C CNN
	1    5000 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 591C816B
P 4100 4950
F 0 "#PWR05" H 4100 4950 30  0001 C CNN
F 1 "GND" H 4100 4880 30  0001 C CNN
F 2 "" H 4100 4950 60  0000 C CNN
F 3 "" H 4100 4950 60  0000 C CNN
	1    4100 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 591C818E
P 5000 4950
F 0 "#PWR06" H 5000 4950 30  0001 C CNN
F 1 "GND" H 5000 4880 30  0001 C CNN
F 2 "" H 5000 4950 60  0000 C CNN
F 3 "" H 5000 4950 60  0000 C CNN
	1    5000 4950
	1    0    0    -1  
$EndComp
Text HLabel 7450 4350 2    45   Input ~ 0
MAC_MDIO
Text HLabel 7450 4250 2    45   Input ~ 0
MAC_MDC
Text HLabel 7450 4050 2    45   Input ~ 0
MAC_RXD1
Text HLabel 7450 3450 2    45   Input ~ 0
MAC_RXD0
$Comp
L RSMALL R203
U 1 1 591C83C1
P 5400 4750
F 0 "R203" V 5370 4803 30  0000 L CNN
F 1 "6.49k" V 5430 4803 30  0000 L CNN
F 2 "apdep:0603_m1608" H 5400 4750 60  0001 C CNN
F 3 "" H 5400 4750 60  0000 C CNN
	1    5400 4750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR07
U 1 1 591C84D3
P 5400 4900
F 0 "#PWR07" H 5400 4900 30  0001 C CNN
F 1 "GND" H 5400 4830 30  0001 C CNN
F 2 "" H 5400 4900 60  0000 C CNN
F 3 "" H 5400 4900 60  0000 C CNN
	1    5400 4900
	1    0    0    -1  
$EndComp
Text HLabel 7450 3250 2    45   Input ~ 0
MAC_CRSDV
Text HLabel 7450 3150 2    45   Input ~ 0
MAC_REFCLK
Text HLabel 7450 2950 2    45   Input ~ 0
MAC_IRQ
Text HLabel 7450 2400 2    45   Input ~ 0
MAC_TXEN
Text HLabel 7450 2300 2    45   Input ~ 0
MAC_TXD0
Text HLabel 7450 2200 2    45   Input ~ 0
MAC_TXD1
Text HLabel 7450 2000 2    45   Input ~ 0
MAC_RST
$Comp
L CSMALL C205
U 1 1 591C89C3
P 4200 2700
F 0 "C205" H 4278 2730 30  0000 L CNN
F 1 "100n" H 4278 2670 30  0000 L CNN
F 2 "apdep:0603_m1608" H 4200 2700 60  0001 C CNN
F 3 "" H 4200 2700 60  0000 C CNN
	1    4200 2700
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C203
U 1 1 591C8A03
P 4050 2700
F 0 "C203" H 3973 2670 30  0000 R CNN
F 1 "2u2" H 3973 2730 30  0000 R CNN
F 2 "apdep:1206_m3216" H 4050 2700 60  0001 C CNN
F 3 "" H 4050 2700 60  0000 C CNN
	1    4050 2700
	1    0    0    1   
$EndComp
$Comp
L GND #PWR08
U 1 1 591C8AA9
P 4200 2600
F 0 "#PWR08" H 4200 2600 30  0001 C CNN
F 1 "GND" H 4200 2530 30  0001 C CNN
F 2 "" H 4200 2600 60  0000 C CNN
F 3 "" H 4200 2600 60  0000 C CNN
	1    4200 2600
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR09
U 1 1 591C8AD7
P 4050 2600
F 0 "#PWR09" H 4050 2600 30  0001 C CNN
F 1 "GND" H 4050 2530 30  0001 C CNN
F 2 "" H 4050 2600 60  0000 C CNN
F 3 "" H 4050 2600 60  0000 C CNN
	1    4050 2600
	-1   0    0    1   
$EndComp
$Comp
L RSMALL R206
U 1 1 591C8D8C
P 6400 4650
F 0 "R206" V 6370 4703 30  0000 L CNN
F 1 "1k" V 6430 4703 30  0000 L CNN
F 2 "apdep:0603_m1608" H 6400 4650 60  0001 C CNN
F 3 "" H 6400 4650 60  0000 C CNN
	1    6400 4650
	0    1    1    0   
$EndComp
$Comp
L GND #PWR010
U 1 1 591C8E28
P 6400 4800
F 0 "#PWR010" H 6400 4800 30  0001 C CNN
F 1 "GND" H 6400 4730 30  0001 C CNN
F 2 "" H 6400 4800 60  0000 C CNN
F 3 "" H 6400 4800 60  0000 C CNN
	1    6400 4800
	1    0    0    -1  
$EndComp
Text Notes 6200 5050 0    45   ~ 0
Strap-in Pull-down\nPHYAD=0
$Comp
L CSMALL C202
U 1 1 591D3780
P 3650 2850
F 0 "C202" H 3728 2880 30  0000 L CNN
F 1 "100n" H 3728 2820 30  0000 L CNN
F 2 "apdep:0603_m1608" H 3650 2850 60  0001 C CNN
F 3 "" H 3650 2850 60  0000 C CNN
	1    3650 2850
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C201
U 1 1 591D3786
P 3500 2850
F 0 "C201" H 3422 2820 30  0000 R CNN
F 1 "22" H 3422 2880 30  0000 R CNN
F 2 "apdep:1206_m3216" H 3500 2850 60  0001 C CNN
F 3 "" H 3500 2850 60  0000 C CNN
	1    3500 2850
	1    0    0    1   
$EndComp
$Comp
L GND #PWR011
U 1 1 591D378C
P 3650 2750
F 0 "#PWR011" H 3650 2750 30  0001 C CNN
F 1 "GND" H 3650 2680 30  0001 C CNN
F 2 "" H 3650 2750 60  0000 C CNN
F 3 "" H 3650 2750 60  0000 C CNN
	1    3650 2750
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR012
U 1 1 591D3792
P 3500 2750
F 0 "#PWR012" H 3500 2750 30  0001 C CNN
F 1 "GND" H 3500 2680 30  0001 C CNN
F 2 "" H 3500 2750 60  0000 C CNN
F 3 "" H 3500 2750 60  0000 C CNN
	1    3500 2750
	-1   0    0    1   
$EndComp
$Comp
L CSMALL C207
U 1 1 591D3F1F
P 6800 3700
F 0 "C207" H 6878 3730 30  0000 L CNN
F 1 "100n" H 6878 3670 30  0000 L CNN
F 2 "apdep:0603_m1608" H 6800 3700 60  0001 C CNN
F 3 "" H 6800 3700 60  0000 C CNN
	1    6800 3700
	-1   0    0    1   
$EndComp
$Comp
L CSMALL C208
U 1 1 591D3F25
P 6950 3700
F 0 "C208" H 6872 3670 30  0000 R CNN
F 1 "22" H 6872 3730 30  0000 R CNN
F 2 "apdep:1206_m3216" H 6950 3700 60  0001 C CNN
F 3 "" H 6950 3700 60  0000 C CNN
	1    6950 3700
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 591D3F2B
P 6800 3800
F 0 "#PWR013" H 6800 3800 30  0001 C CNN
F 1 "GND" H 6800 3730 30  0001 C CNN
F 2 "" H 6800 3800 60  0000 C CNN
F 3 "" H 6800 3800 60  0000 C CNN
	1    6800 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 591D3F31
P 6950 3800
F 0 "#PWR014" H 6950 3800 30  0001 C CNN
F 1 "GND" H 6950 3730 30  0001 C CNN
F 2 "" H 6950 3800 60  0000 C CNN
F 3 "" H 6950 3800 60  0000 C CNN
	1    6950 3800
	1    0    0    -1  
$EndComp
Text Notes 3500 2350 0    45   ~ 0
POWER BUDGET:\nMax at 100BASE-TX FD\nVETH: 34 mA (VDDA) + 13 mA (VDDIO)
$Comp
L RSMALL R201
U 1 1 591D4293
P 2750 4850
F 0 "R201" H 2750 4993 30  0000 C CNN
F 1 "220" H 2750 4933 30  0000 C CNN
F 2 "apdep:0603_m1608" H 2750 4850 60  0001 C CNN
F 3 "" H 2750 4850 60  0000 C CNN
	1    2750 4850
	1    0    0    -1  
$EndComp
$Comp
L RSMALL R204
U 1 1 591D42F6
P 4850 1950
F 0 "R204" H 4850 2093 30  0000 C CNN
F 1 "1k" H 4850 2033 30  0000 C CNN
F 2 "apdep:0603_m1608" H 4850 1950 60  0001 C CNN
F 3 "" H 4850 1950 60  0000 C CNN
	1    4850 1950
	0    1    1    0   
$EndComp
$Comp
L RSMALL R202
U 1 1 591D435C
P 4850 1500
F 0 "R202" H 4850 1643 30  0000 C CNN
F 1 "4k7" H 4850 1583 30  0000 C CNN
F 2 "apdep:0603_m1608" H 4850 1500 60  0001 C CNN
F 3 "" H 4850 1500 60  0000 C CNN
	1    4850 1500
	0    1    1    0   
$EndComp
$Comp
L DIODE D203
U 1 1 591D4892
P 7100 2000
F 0 "D203" H 7100 2192 40  0000 C CNN
F 1 "DIODE" H 7100 2116 40  0000 C CNN
F 2 "apdep:SOD323" H 7100 2000 60  0001 C CNN
F 3 "" H 7100 2000 60  0000 C CNN
	1    7100 2000
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C206
U 1 1 591D4942
P 6650 1800
F 0 "C206" H 6728 1830 30  0000 L CNN
F 1 "1u" H 6728 1770 30  0000 L CNN
F 2 "apdep:0603_m1608" H 6650 1800 60  0001 C CNN
F 3 "" H 6650 1800 60  0000 C CNN
	1    6650 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 591D4A64
P 6650 1700
F 0 "#PWR015" H 6650 1700 30  0001 C CNN
F 1 "GND" H 6650 1630 30  0001 C CNN
F 2 "" H 6650 1700 60  0000 C CNN
F 3 "" H 6650 1700 60  0000 C CNN
	1    6650 1700
	-1   0    0    1   
$EndComp
$Comp
L RSMALL R205
U 1 1 591D4AE1
P 6400 1750
F 0 "R205" V 6370 1803 30  0000 L CNN
F 1 "10k" V 6430 1803 30  0000 L CNN
F 2 "apdep:0603_m1608" H 6400 1750 60  0001 C CNN
F 3 "" H 6400 1750 60  0000 C CNN
	1    6400 1750
	0    1    1    0   
$EndComp
$Comp
L DIODE D202
U 1 1 591D4C53
P 6050 1700
F 0 "D202" V 6088 1622 40  0000 R CNN
F 1 "DIODE" V 6012 1622 40  0000 R CNN
F 2 "apdep:SOD323" H 6050 1700 60  0001 C CNN
F 3 "" H 6050 1700 60  0000 C CNN
	1    6050 1700
	0    -1   -1   0   
$EndComp
Text Notes 8100 3150 0    60   ~ 0
RMII to STM32
$Comp
L RSMALL R207
U 1 1 591DE549
P 6550 3050
F 0 "R207" H 6550 3000 30  0000 C CNN
F 1 "1k" H 6550 3050 30  0000 C CNN
F 2 "apdep:0603_m1608" H 6550 3050 60  0001 C CNN
F 3 "" H 6550 3050 60  0000 C CNN
	1    6550 3050
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR016
U 1 1 591DE641
P 6700 3050
F 0 "#PWR016" H 6700 3050 30  0001 C CNN
F 1 "GND" H 6700 2980 30  0001 C CNN
F 2 "" H 6700 3050 60  0000 C CNN
F 3 "" H 6700 3050 60  0000 C CNN
	1    6700 3050
	0    -1   -1   0   
$EndComp
Text Notes 6800 3050 0    45   ~ 0
RXER not required
Text HLabel 1000 1100 0    45   Input ~ 0
5V
Text Label 4350 2950 0    45   ~ 0
VDD12
Text Label 4350 3050 0    45   ~ 0
VDDA
Text Label 4400 4000 0    45   ~ 0
XO
Text Label 4750 4550 0    45   ~ 0
XI
Text Label 5400 4450 1    45   ~ 0
REXT
Text Label 6200 3050 0    45   ~ 0
RXER
Text Label 5300 1750 2    45   ~ 0
LED
Text Label 5200 2300 1    45   ~ 0
MACRST
Text Label 6350 2950 0    45   ~ 0
MACIRQ
Text Label 2300 3150 0    45   ~ 0
TXP
Text Label 2300 3550 0    45   ~ 0
TXM
Text Label 2300 3850 0    45   ~ 0
RXP
Text Label 2300 4250 0    45   ~ 0
RXM
$Comp
L WE_74990111217 J201
U 1 1 5961FAB7
P 2200 2950
F 0 "J201" H 3600 700 45  0000 L CNN
F 1 "WE_74990111217" H 3150 600 45  0000 L CNN
F 2 "apdep:WE_74990111217" H 2200 2950 45  0001 C CNN
F 3 "" H 2200 2950 45  0001 C CNN
	1    2200 2950
	-1   0    0    -1  
$EndComp
$Comp
L CSMALL C209
U 1 1 5961FB9E
P 2300 3350
F 0 "C209" V 2200 3300 30  0000 C CNN
F 1 "100n" V 2200 3450 30  0000 C CNN
F 2 "apdep:0805_m2012" H 2300 3350 60  0001 C CNN
F 3 "" H 2300 3350 60  0000 C CNN
	1    2300 3350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR017
U 1 1 5961FF29
P 2400 3350
F 0 "#PWR017" H 2400 3350 30  0001 C CNN
F 1 "GND" H 2400 3280 30  0001 C CNN
F 2 "" H 2400 3350 60  0000 C CNN
F 3 "" H 2400 3350 60  0000 C CNN
	1    2400 3350
	0    -1   -1   0   
$EndComp
$Comp
L CSMALL C210
U 1 1 596200B6
P 2300 4050
F 0 "C210" V 2200 4000 30  0000 C CNN
F 1 "100n" V 2200 4150 30  0000 C CNN
F 2 "apdep:0805_m2012" H 2300 4050 60  0001 C CNN
F 3 "" H 2300 4050 60  0000 C CNN
	1    2300 4050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR018
U 1 1 59620127
P 2400 4050
F 0 "#PWR018" H 2400 4050 30  0001 C CNN
F 1 "GND" H 2400 3980 30  0001 C CNN
F 2 "" H 2400 4050 60  0000 C CNN
F 3 "" H 2400 4050 60  0000 C CNN
	1    2400 4050
	0    -1   -1   0   
$EndComp
Text Notes 800  2950 0    45   ~ 0
WARNING pairs crossed for better layout\nAuto MDI/MDIX is required.
Text Label 2450 4950 0    45   ~ 0
LED
$Comp
L RSMALL R208
U 1 1 5963B88D
P 2750 5150
F 0 "R208" H 2750 5293 30  0000 C CNN
F 1 "220" H 2750 5233 30  0000 C CNN
F 2 "apdep:0603_m1608" H 2750 5150 60  0001 C CNN
F 3 "" H 2750 5150 60  0000 C CNN
	1    2750 5150
	1    0    0    -1  
$EndComp
Text HLabel 2450 5250 2    45   Input ~ 0
MAC_LINK
Text Notes 4650 1000 0    45   ~ 0
Select R202 or R204 for led strapping
$Comp
L GND #PWR019
U 1 1 5963C2E0
P 4850 2100
F 0 "#PWR019" H 4850 2100 30  0001 C CNN
F 1 "GND" H 4850 2030 30  0001 C CNN
F 2 "" H 4850 2100 60  0000 C CNN
F 3 "" H 4850 2100 60  0000 C CNN
	1    4850 2100
	1    0    0    -1  
$EndComp
$Comp
L FB FB202
U 1 1 5964004D
P 1200 1100
F 0 "FB202" H 1200 932 30  0000 C CNN
F 1 "FB" H 1200 992 30  0000 C CNN
F 2 "apdep:0805_m2012" H 1200 1100 60  0001 C CNN
F 3 "" H 1200 1100 60  0000 C CNN
	1    1200 1100
	-1   0    0    1   
$EndComp
$Comp
L VETH #PWR020
U 1 1 597AEAF9
P 3000 1100
F 0 "#PWR020" H 3000 1200 30  0001 C CNN
F 1 "VETH" V 3014 1207 30  0000 L CNN
F 2 "" H 3000 1100 60  0000 C CNN
F 3 "" H 3000 1100 60  0000 C CNN
	1    3000 1100
	0    1    1    0   
$EndComp
$Comp
L VETH #PWR021
U 1 1 597AEC66
P 4850 1350
F 0 "#PWR021" H 4850 1450 30  0001 C CNN
F 1 "VETH" H 4864 1488 30  0000 C CNN
F 2 "" H 4850 1350 60  0000 C CNN
F 3 "" H 4850 1350 60  0000 C CNN
	1    4850 1350
	1    0    0    -1  
$EndComp
$Comp
L VETH #PWR022
U 1 1 597AEE4D
P 6050 1300
F 0 "#PWR022" H 6050 1400 30  0001 C CNN
F 1 "VETH" H 6064 1438 30  0000 C CNN
F 2 "" H 6050 1300 60  0000 C CNN
F 3 "" H 6050 1300 60  0000 C CNN
	1    6050 1300
	1    0    0    -1  
$EndComp
$Comp
L VETH #PWR023
U 1 1 597AF173
P 7050 3350
F 0 "#PWR023" H 7050 3450 30  0001 C CNN
F 1 "VETH" V 7064 3457 30  0000 L CNN
F 2 "" H 7050 3350 60  0000 C CNN
F 3 "" H 7050 3350 60  0000 C CNN
	1    7050 3350
	0    1    1    0   
$EndComp
$Comp
L VETH #PWR024
U 1 1 597AF4A0
P 3200 2550
F 0 "#PWR024" H 3200 2650 30  0001 C CNN
F 1 "VETH" H 3214 2688 30  0000 C CNN
F 2 "" H 3200 2550 60  0000 C CNN
F 3 "" H 3200 2550 60  0000 C CNN
	1    3200 2550
	1    0    0    -1  
$EndComp
$Comp
L VETH #PWR025
U 1 1 597AF7CE
P 2900 4850
F 0 "#PWR025" H 2900 4950 30  0001 C CNN
F 1 "VETH" V 2914 4957 30  0000 L CNN
F 2 "" H 2900 4850 60  0000 C CNN
F 3 "" H 2900 4850 60  0000 C CNN
	1    2900 4850
	0    1    1    0   
$EndComp
$Comp
L VETH #PWR026
U 1 1 597AF848
P 2900 5150
F 0 "#PWR026" H 2900 5250 30  0001 C CNN
F 1 "VETH" V 2914 5257 30  0000 L CNN
F 2 "" H 2900 5150 60  0000 C CNN
F 3 "" H 2900 5150 60  0000 C CNN
	1    2900 5150
	0    1    1    0   
$EndComp
$Comp
L ADP151 U202
U 1 1 597E3D74
P 2050 1150
F 0 "U202" H 2050 1404 45  0000 C CNN
F 1 "ADP151" H 2050 1320 45  0000 C CNN
F 2 "apdep:SOT23-6" H 2050 1150 60  0001 C CNN
F 3 "" H 2050 1150 60  0000 C CNN
	1    2050 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 597E3E73
P 2050 1350
F 0 "#PWR027" H 2050 1350 30  0001 C CNN
F 1 "GND" H 2050 1280 30  0001 C CNN
F 2 "" H 2050 1350 60  0000 C CNN
F 3 "" H 2050 1350 60  0000 C CNN
	1    2050 1350
	1    0    0    -1  
$EndComp
$Comp
L CSMALL C211
U 1 1 597E3EBD
P 2450 1400
F 0 "C211" H 2528 1430 30  0000 L CNN
F 1 "10n" H 2528 1370 30  0000 L CNN
F 2 "apdep:0603_m1608" H 2450 1400 60  0001 C CNN
F 3 "" H 2450 1400 60  0000 C CNN
	1    2450 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 4550 5300 4550
Wire Wire Line
	5000 4550 5000 4750
Wire Wire Line
	5200 3900 5200 4000
Wire Wire Line
	5200 4000 4100 4000
Wire Wire Line
	4100 4000 4100 4750
Wire Wire Line
	4100 4550 4200 4550
Connection ~ 5000 4550
Connection ~ 4100 4550
Wire Wire Line
	5600 3900 5600 4250
Wire Wire Line
	5600 4250 7450 4250
Wire Wire Line
	5500 3900 5500 4350
Wire Wire Line
	5500 4350 7450 4350
Wire Wire Line
	5700 3900 5700 4050
Wire Wire Line
	5700 4050 7450 4050
Wire Wire Line
	6150 3450 7450 3450
Wire Wire Line
	5300 4550 5300 3900
Wire Wire Line
	5400 4600 5400 3900
Wire Wire Line
	6150 3150 7450 3150
Wire Wire Line
	6150 3250 7450 3250
Wire Wire Line
	6150 2950 7450 2950
Wire Wire Line
	5700 2500 5700 2400
Wire Wire Line
	5700 2400 7450 2400
Wire Wire Line
	5600 2500 5600 2300
Wire Wire Line
	5600 2300 7450 2300
Wire Wire Line
	5500 2500 5500 2200
Wire Wire Line
	5500 2200 7450 2200
Wire Wire Line
	5200 2500 5200 2000
Wire Wire Line
	4050 2950 4750 2950
Wire Wire Line
	4200 2950 4200 2800
Wire Wire Line
	4050 2950 4050 2800
Connection ~ 4200 2950
Wire Wire Line
	2800 3150 4750 3150
Wire Wire Line
	2700 3250 4750 3250
Wire Wire Line
	4750 3350 3150 3350
Wire Wire Line
	3050 3450 4750 3450
Wire Wire Line
	3200 3050 4750 3050
Wire Wire Line
	6400 3250 6400 4500
Connection ~ 6400 3250
Wire Wire Line
	3650 3050 3650 2950
Wire Wire Line
	3500 3050 3500 2950
Connection ~ 3650 3050
Connection ~ 3500 3050
Wire Wire Line
	6150 3350 7050 3350
Wire Wire Line
	6800 3350 6800 3600
Wire Wire Line
	6950 3350 6950 3600
Connection ~ 6800 3350
Connection ~ 6950 3350
Wire Wire Line
	5300 1750 5300 2500
Wire Wire Line
	7450 2000 7300 2000
Wire Wire Line
	5200 2000 6900 2000
Wire Wire Line
	6650 2000 6650 1900
Wire Wire Line
	6400 2000 6400 1900
Connection ~ 6650 2000
Wire Wire Line
	6050 2000 6050 1900
Connection ~ 6400 2000
Connection ~ 6050 2000
Wire Wire Line
	6400 1600 6400 1400
Wire Wire Line
	6400 1400 6050 1400
Wire Wire Line
	6050 1300 6050 1500
Connection ~ 6050 1400
Wire Notes Line
	8000 2000 8000 4350
Wire Wire Line
	6400 3050 6150 3050
Wire Wire Line
	1100 1100 1000 1100
Wire Wire Line
	2200 4550 2350 4550
Wire Wire Line
	2350 4550 2350 5550
Wire Wire Line
	2350 5550 1750 5550
Wire Wire Line
	1750 5550 1750 5450
Wire Wire Line
	2800 3150 2800 3550
Wire Wire Line
	2800 3550 2200 3550
Wire Wire Line
	2700 3250 2700 3150
Wire Wire Line
	2700 3150 2200 3150
Wire Wire Line
	3150 3350 3150 4250
Wire Wire Line
	3150 4250 2200 4250
Wire Wire Line
	3050 3450 3050 3850
Wire Wire Line
	3050 3850 2200 3850
Wire Wire Line
	2450 4950 2200 4950
Wire Wire Line
	2200 4850 2600 4850
Wire Wire Line
	2600 5150 2200 5150
Wire Wire Line
	2200 5250 2450 5250
Wire Wire Line
	4850 1650 4850 1800
Wire Wire Line
	4850 1750 5300 1750
Connection ~ 4850 1750
Wire Wire Line
	1300 1100 1750 1100
Wire Wire Line
	2350 1100 3000 1100
Wire Wire Line
	2350 1200 2450 1200
Wire Wire Line
	2450 1200 2450 1300
$Comp
L GND #PWR028
U 1 1 597E40A2
P 2450 1500
F 0 "#PWR028" H 2450 1500 30  0001 C CNN
F 1 "GND" H 2450 1430 30  0001 C CNN
F 2 "" H 2450 1500 60  0000 C CNN
F 3 "" H 2450 1500 60  0000 C CNN
	1    2450 1500
	1    0    0    -1  
$EndComp
Text Label 1350 1100 0    45   ~ 0
5VETH
Wire Wire Line
	1750 1200 1650 1200
Wire Wire Line
	1650 1200 1650 1100
Connection ~ 1650 1100
$Comp
L RSMALL R209
U 1 1 59CCC21C
P 7000 2800
F 0 "R209" V 6970 2853 30  0000 L CNN
F 1 "10k" V 7030 2853 30  0000 L CNN
F 2 "apdep:0603_m1608" H 7000 2800 60  0001 C CNN
F 3 "" H 7000 2800 60  0000 C CNN
	1    7000 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2800 6750 2800
Wire Wire Line
	6750 2800 6750 2950
Connection ~ 6750 2950
$Comp
L VETH #PWR029
U 1 1 59CCC327
P 7150 2800
F 0 "#PWR029" H 7150 2900 30  0001 C CNN
F 1 "VETH" V 7164 2907 30  0000 L CNN
F 2 "" H 7150 2800 60  0000 C CNN
F 3 "" H 7150 2800 60  0000 C CNN
	1    7150 2800
	0    1    1    0   
$EndComp
$Comp
L RSMALL R210
U 1 1 59CCC730
P 7000 4450
F 0 "R210" V 6970 4503 30  0000 L CNN
F 1 "1k" V 7030 4503 30  0000 L CNN
F 2 "apdep:0603_m1608" H 7000 4450 60  0001 C CNN
F 3 "" H 7000 4450 60  0000 C CNN
	1    7000 4450
	-1   0    0    1   
$EndComp
Wire Wire Line
	6850 4450 6750 4450
Wire Wire Line
	6750 4450 6750 4350
Connection ~ 6750 4350
$Comp
L VETH #PWR030
U 1 1 59CCC8B4
P 7150 4450
F 0 "#PWR030" H 7150 4550 30  0001 C CNN
F 1 "VETH" V 7164 4557 30  0000 L CNN
F 2 "" H 7150 4450 60  0000 C CNN
F 3 "" H 7150 4450 60  0000 C CNN
	1    7150 4450
	0    1    1    0   
$EndComp
$Comp
L FB FB201
U 1 1 59CCEBB3
P 3200 2650
F 0 "FB201" H 3200 2482 30  0000 C CNN
F 1 "FB" H 3200 2542 30  0000 C CNN
F 2 "apdep:0805_m2012" H 3200 2650 60  0001 C CNN
F 3 "" H 3200 2650 60  0000 C CNN
	1    3200 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 2750 3200 3050
$EndSCHEMATC

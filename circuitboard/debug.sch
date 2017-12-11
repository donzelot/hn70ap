EESchema Schematic File Version 2
LIBS:apdep
LIBS:f4grx_conn
LIBS:f4grx_rf
LIBS:ap-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 6
Title "Radio gateway (Debug UART)"
Date "2017-05-19"
Rev "1"
Comp "F4GRX"
Comment1 "CERN OHL"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_MICROB U402
U 1 1 591E0B17
P 6850 3850
F 0 "U402" V 6853 3993 60  0000 L CNN
F 1 "USB_MICROB" V 6959 3993 60  0000 L CNN
F 2 "apdep:USB_MICROB_MOLEX" H 6850 3850 60  0001 C CNN
F 3 "" H 6850 3850 60  0000 C CNN
	1    6850 3850
	0    1    1    0   
$EndComp
$Comp
L FT232R_SSOP U401
U 1 1 591E0B72
P 4900 3200
F 0 "U401" H 4825 4103 60  0000 C CNN
F 1 "FT232R_SSOP" H 4825 3997 60  0000 C CNN
F 2 "apdep:TSSOP28" H 4900 3200 60  0001 C CNN
F 3 "" H 4900 3200 60  0000 C CNN
	1    4900 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3750 6600 3750
Wire Wire Line
	5300 3850 6600 3850
$Comp
L GND #PWR410
U 1 1 591E0C6F
P 6900 4350
F 0 "#PWR410" H 6900 4350 30  0001 C CNN
F 1 "GND" H 6900 4280 30  0001 C CNN
F 2 "" H 6900 4350 60  0000 C CNN
F 3 "" H 6900 4350 60  0000 C CNN
	1    6900 4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR409
U 1 1 591E0C87
P 6600 4050
F 0 "#PWR409" H 6600 4050 30  0001 C CNN
F 1 "GND" H 6600 3980 30  0001 C CNN
F 2 "" H 6600 4050 60  0000 C CNN
F 3 "" H 6600 4050 60  0000 C CNN
	1    6600 4050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR406
U 1 1 591E0DE1
P 5300 3550
F 0 "#PWR406" H 5300 3550 30  0001 C CNN
F 1 "GND" H 5300 3480 30  0001 C CNN
F 2 "" H 5300 3550 60  0000 C CNN
F 3 "" H 5300 3550 60  0000 C CNN
	1    5300 3550
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR402
U 1 1 591E0DF8
P 4350 3150
F 0 "#PWR402" H 4350 3150 30  0001 C CNN
F 1 "GND" H 4350 3080 30  0001 C CNN
F 2 "" H 4350 3150 60  0000 C CNN
F 3 "" H 4350 3150 60  0000 C CNN
	1    4350 3150
	0    1    1    0   
$EndComp
NoConn ~ 5300 2950
NoConn ~ 4350 3250
NoConn ~ 4350 3350
NoConn ~ 4350 3450
NoConn ~ 4350 3550
$Comp
L GND #PWR405
U 1 1 591E0E21
P 5300 3250
F 0 "#PWR405" H 5300 3250 30  0001 C CNN
F 1 "GND" H 5300 3180 30  0001 C CNN
F 2 "" H 5300 3250 60  0000 C CNN
F 3 "" H 5300 3250 60  0000 C CNN
	1    5300 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 2550 3850 2550
Wire Wire Line
	4350 2950 3850 2950
Text HLabel 3850 2550 0    45   Input ~ 0
DBG_PCTX_CPURX
Text HLabel 3850 2950 0    45   Input ~ 0
DBG_PCRX_CPUTX
NoConn ~ 4350 3050
NoConn ~ 4350 2750
NoConn ~ 4350 2650
$Comp
L GND #PWR404
U 1 1 591E0FA2
P 5300 2850
F 0 "#PWR404" H 5300 2850 30  0001 C CNN
F 1 "GND" H 5300 2780 30  0001 C CNN
F 2 "" H 5300 2850 60  0000 C CNN
F 3 "" H 5300 2850 60  0000 C CNN
	1    5300 2850
	0    -1   -1   0   
$EndComp
Text Notes 600  7700 0    300  ~ 60
DEBUG
$Comp
L LED D401
U 1 1 593ECEE2
P 6150 3000
F 0 "D401" H 6150 3215 50  0000 C CNN
F 1 "LED" H 6150 3124 50  0000 C CNN
F 2 "apdep:0805_m2012" H 6150 3000 60  0001 C CNN
F 3 "" H 6150 3000 60  0000 C CNN
	1    6150 3000
	-1   0    0    -1  
$EndComp
$Comp
L LED D402
U 1 1 593ECF0B
P 6150 3200
F 0 "D402" H 6150 3000 50  0000 C CNN
F 1 "LED" H 6150 3100 50  0000 C CNN
F 2 "apdep:0805_m2012" H 6150 3200 60  0001 C CNN
F 3 "" H 6150 3200 60  0000 C CNN
	1    6150 3200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5950 3200 5700 3200
Wire Wire Line
	5700 3200 5700 3150
Wire Wire Line
	5700 3150 5300 3150
Wire Wire Line
	5300 3050 5700 3050
Wire Wire Line
	5700 3050 5700 3000
Wire Wire Line
	5700 3000 5950 3000
$Comp
L RSMALL R401
U 1 1 593ED42C
P 6600 3000
F 0 "R401" H 6600 3050 30  0000 C CNN
F 1 "330" H 6600 3000 30  0000 C CNN
F 2 "apdep:0603_m1608" H 6600 3000 60  0001 C CNN
F 3 "" H 6600 3000 60  0000 C CNN
	1    6600 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3000 6350 3000
Wire Wire Line
	6450 3200 6350 3200
Wire Wire Line
	6750 3200 6850 3200
Wire Wire Line
	6850 3200 6850 3000
Wire Wire Line
	6750 3000 7700 3000
Connection ~ 6850 3000
Wire Wire Line
	5700 3350 5700 3550
Wire Wire Line
	5700 3350 5300 3350
$Comp
L CSMALL C401
U 1 1 593ED6F0
P 5800 3350
F 0 "C401" V 5900 3350 30  0000 C CNN
F 1 "100n" V 5700 3350 30  0000 C CNN
F 2 "apdep:0603_m1608" H 5800 3350 60  0001 C CNN
F 3 "" H 5800 3350 60  0000 C CNN
	1    5800 3350
	0    -1   -1   0   
$EndComp
Connection ~ 5700 3350
$Comp
L GND #PWR408
U 1 1 593ED79B
P 5900 3350
F 0 "#PWR408" H 5900 3350 30  0001 C CNN
F 1 "GND" H 5900 3280 30  0001 C CNN
F 2 "" H 5900 3350 60  0000 C CNN
F 3 "" H 5900 3350 60  0000 C CNN
	1    5900 3350
	0    -1   -1   0   
$EndComp
NoConn ~ 6600 3950
$Comp
L GND #PWR403
U 1 1 593EDF98
P 5300 2750
F 0 "#PWR403" H 5300 2750 30  0001 C CNN
F 1 "GND" H 5300 2680 30  0001 C CNN
F 2 "" H 5300 2750 60  0000 C CNN
F 3 "" H 5300 2750 60  0000 C CNN
	1    5300 2750
	0    -1   -1   0   
$EndComp
NoConn ~ 5300 2650
NoConn ~ 5300 2550
NoConn ~ 4350 3850
NoConn ~ 4350 3750
NoConn ~ 4350 3650
NoConn ~ 5300 3450
Wire Wire Line
	2550 2850 4350 2850
$Comp
L RSMALL R402
U 1 1 593EE799
P 6600 3200
F 0 "R402" H 6600 3250 30  0000 C CNN
F 1 "330" H 6600 3200 30  0000 C CNN
F 2 "apdep:0603_m1608" H 6600 3200 60  0001 C CNN
F 3 "" H 6600 3200 60  0000 C CNN
	1    6600 3200
	1    0    0    -1  
$EndComp
Text HLabel 2550 2850 0    45   Input ~ 0
3V3DBG
Text Label 3900 2850 0    45   ~ 0
3V3DBG
Text Label 7000 3000 0    45   ~ 0
3V3USB
$Comp
L CSMALL C402
U 1 1 595FEFEF
P 2800 3050
F 0 "C402" V 2968 3050 30  0000 C CNN
F 1 "100n" V 2908 3050 30  0000 C CNN
F 2 "apdep:0603_m1608" H 2800 3050 60  0001 C CNN
F 3 "" H 2800 3050 60  0000 C CNN
	1    2800 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR401
U 1 1 595FF076
P 2800 3150
F 0 "#PWR401" H 2800 3150 30  0001 C CNN
F 1 "GND" H 2800 3080 30  0001 C CNN
F 2 "" H 2800 3150 60  0000 C CNN
F 3 "" H 2800 3150 60  0000 C CNN
	1    2800 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 2950 2800 2850
Connection ~ 2800 2850
Text Label 6150 3750 0    45   ~ 0
DM
Text Label 6150 3850 0    45   ~ 0
DP
Text Label 6400 3650 0    45   ~ 0
VUSB
Wire Wire Line
	5300 3650 5400 3650
Wire Wire Line
	5400 3650 5400 4650
$Comp
L CSMALL C403
U 1 1 59C4BCE8
P 5400 4750
F 0 "C403" V 5568 4750 30  0000 C CNN
F 1 "100n" V 5508 4750 30  0000 C CNN
F 2 "apdep:0603_m1608" H 5400 4750 60  0001 C CNN
F 3 "" H 5400 4750 60  0000 C CNN
	1    5400 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR407
U 1 1 59C4BD35
P 5400 4850
F 0 "#PWR407" H 5400 4850 30  0001 C CNN
F 1 "GND" H 5400 4780 30  0001 C CNN
F 2 "" H 5400 4850 60  0000 C CNN
F 3 "" H 5400 4850 60  0000 C CNN
	1    5400 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4500 7700 4500
Wire Wire Line
	7700 4500 7700 3000
Connection ~ 5400 4500
$Comp
L FB FB401
U 1 1 59D5EBEC
P 5900 3550
F 0 "FB401" H 5800 3450 30  0000 C CNN
F 1 "FB" H 5950 3450 30  0000 C CNN
F 2 "apdep:0805_m2012" H 5900 3550 60  0001 C CNN
F 3 "" H 5900 3550 60  0000 C CNN
	1    5900 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3550 5800 3550
Wire Wire Line
	6000 3550 6150 3550
Wire Wire Line
	6150 3550 6150 3650
Wire Wire Line
	6150 3650 6600 3650
Text Label 5400 3350 0    45   ~ 0
VCCUSB
Text Label 5750 3000 0    45   ~ 0
TXLED
Text Label 5750 3200 0    45   ~ 0
RXLED
Text Notes 6250 3300 0    45   ~ 0
CPU->USB
Text Notes 6250 2850 0    45   ~ 0
USB->CPU
$EndSCHEMATC

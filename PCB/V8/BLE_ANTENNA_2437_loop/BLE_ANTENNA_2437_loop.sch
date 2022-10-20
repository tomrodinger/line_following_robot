EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R1
U 1 1 62DC735B
P 5100 2500
F 0 "R1" H 5170 2546 50  0000 L CNN
F 1 "3k0" H 5170 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5030 2500 50  0001 C CNN
F 3 "~" H 5100 2500 50  0001 C CNN
	1    5100 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 62DC7DB9
P 5350 2500
F 0 "C2" H 5465 2546 50  0000 L CNN
F 1 "0.6pf NP0" H 5465 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5388 2350 50  0001 C CNN
F 3 "~" H 5350 2500 50  0001 C CNN
	1    5350 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 62DC8413
P 4500 2550
F 0 "C1" H 4615 2596 50  0000 L CNN
F 1 "4.8pf NP0" H 4615 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4538 2400 50  0001 C CNN
F 3 "~" H 4500 2550 50  0001 C CNN
	1    4500 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 62DC891D
P 4750 2300
F 0 "L1" V 4940 2300 50  0000 C CNN
F 1 "2.5nH" V 4849 2300 50  0000 C CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 4750 2300 50  0001 C CNN
F 3 "~" H 4750 2300 50  0001 C CNN
	1    4750 2300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 62DCE64B
P 5350 2800
F 0 "#PWR0101" H 5350 2550 50  0001 C CNN
F 1 "GND" H 5355 2627 50  0000 C CNN
F 2 "" H 5350 2800 50  0001 C CNN
F 3 "" H 5350 2800 50  0001 C CNN
	1    5350 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 62DD06AC
P 5100 2800
F 0 "#PWR0102" H 5100 2550 50  0001 C CNN
F 1 "GND" H 5105 2627 50  0000 C CNN
F 2 "" H 5100 2800 50  0001 C CNN
F 3 "" H 5100 2800 50  0001 C CNN
	1    5100 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 62DD0A62
P 4500 2850
F 0 "#PWR0103" H 4500 2600 50  0001 C CNN
F 1 "GND" H 4505 2677 50  0000 C CNN
F 2 "" H 4500 2850 50  0001 C CNN
F 3 "" H 4500 2850 50  0001 C CNN
	1    4500 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 62DD0E41
P 5650 2800
F 0 "#PWR0104" H 5650 2550 50  0001 C CNN
F 1 "GND" H 5655 2627 50  0000 C CNN
F 2 "" H 5650 2800 50  0001 C CNN
F 3 "" H 5650 2800 50  0001 C CNN
	1    5650 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2850 4500 2700
Wire Wire Line
	5100 2800 5100 2650
Wire Wire Line
	5350 2800 5350 2650
Wire Wire Line
	5350 2300 5350 2350
Wire Wire Line
	5350 2300 5100 2300
Wire Wire Line
	5100 2300 5100 2350
Connection ~ 5350 2300
Wire Wire Line
	5100 2300 4900 2300
Connection ~ 5100 2300
Wire Wire Line
	4500 2400 4500 2300
Wire Wire Line
	4500 2300 4600 2300
Wire Wire Line
	4500 2300 4350 2300
Connection ~ 4500 2300
$Comp
L Device:Antenna_Loop AE1
U 1 1 62DD8491
P 6000 2300
F 0 "AE1" V 6004 2480 50  0000 L CNN
F 1 "Antenna_Loop" V 6095 2480 50  0001 L CNN
F 2 "BLE_ANTENNA_2437_loop:BLE_2437_LOOP_2" H 6000 2300 50  0000 C CNN
F 3 "~" H 6000 2300 50  0001 C CNN
	1    6000 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 2300 5800 2300
Wire Wire Line
	5800 2400 5650 2400
Wire Wire Line
	5650 2400 5650 2800
Text Notes 3900 2250 0    50   ~ 0
Zin =25-5j
Text Notes 4400 3600 0    50   ~ 0
Matching Pi Zin=25-5j\nC1=4,8pF +-0.1pF NP0\nL1=2,5nH +-0.1nH\nC2=0.6pf+-0.05pF NP0\nR1=3k0 1%
Text Notes 4400 4500 0    50   ~ 0
Matching Pi Zin=50 Ohm\nC1=3,8pF +-0.1pF NP0\nL1=2,7nH +-0.1nH\nC2=0.6pf+-0.05pF NP0\nR1=3k3 1%
Text Notes 4450 1350 0    50   ~ 0
BLE unidirectional loop antenna Fo=2437Mhz
Text Notes 7050 6850 0    118  ~ 0
BLE unidirectional loop antenna Fo=2437Mhz
$EndSCHEMATC

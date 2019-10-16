EESchema Schematic File Version 4
LIBS:Bluetrackr-transmitter-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Bluetrackr-transmitter"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32F1:STM32F103C8Tx U2
U 1 1 5DA2677B
P 5550 3950
F 0 "U2" H 6000 2500 50  0000 C CNN
F 1 "STM32F103C8Tx" H 6000 2400 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 4950 2550 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 5550 3950 50  0001 C CNN
	1    5550 3950
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0101
U 1 1 5DA27C6F
P 5550 2250
F 0 "#PWR0101" H 5550 2100 50  0001 C CNN
F 1 "+3V3" H 5565 2423 50  0000 C CNN
F 2 "" H 5550 2250 50  0001 C CNN
F 3 "" H 5550 2250 50  0001 C CNN
	1    5550 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2250 5550 2350
Wire Wire Line
	5650 2450 5650 2350
Wire Wire Line
	5650 2350 5550 2350
Connection ~ 5550 2350
Wire Wire Line
	5550 2350 5550 2450
Wire Wire Line
	5750 2450 5750 2350
Wire Wire Line
	5750 2350 5650 2350
Connection ~ 5650 2350
Wire Wire Line
	5450 2450 5450 2350
Wire Wire Line
	5450 2350 5550 2350
Wire Wire Line
	5350 2450 5350 2350
Wire Wire Line
	5350 2350 5450 2350
Connection ~ 5450 2350
$Comp
L power:GND #PWR0102
U 1 1 5DA29114
P 5350 5650
F 0 "#PWR0102" H 5350 5400 50  0001 C CNN
F 1 "GND" H 5355 5477 50  0000 C CNN
F 2 "" H 5350 5650 50  0001 C CNN
F 3 "" H 5350 5650 50  0001 C CNN
	1    5350 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 5450 5350 5550
Wire Wire Line
	5450 5450 5450 5550
Wire Wire Line
	5450 5550 5350 5550
Connection ~ 5350 5550
Wire Wire Line
	5350 5550 5350 5650
Wire Wire Line
	5550 5450 5550 5550
Wire Wire Line
	5550 5550 5450 5550
Connection ~ 5450 5550
Wire Wire Line
	5650 5450 5650 5550
Wire Wire Line
	5650 5550 5550 5550
Connection ~ 5550 5550
$Comp
L power:GND #PWR0103
U 1 1 5DA2BECA
P 4600 3350
F 0 "#PWR0103" H 4600 3100 50  0001 C CNN
F 1 "GND" H 4605 3177 50  0000 C CNN
F 2 "" H 4600 3350 50  0001 C CNN
F 3 "" H 4600 3350 50  0001 C CNN
	1    4600 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5DA2CF17
P 4200 2950
F 0 "C7" H 4315 2996 50  0000 L CNN
F 1 "100n" H 4315 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4238 2800 50  0001 C CNN
F 3 "~" H 4200 2950 50  0001 C CNN
	1    4200 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3350 4600 3250
$Comp
L power:GND #PWR0104
U 1 1 5DA2E033
P 4200 3350
F 0 "#PWR0104" H 4200 3100 50  0001 C CNN
F 1 "GND" H 4205 3177 50  0000 C CNN
F 2 "" H 4200 3350 50  0001 C CNN
F 3 "" H 4200 3350 50  0001 C CNN
	1    4200 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5DA2B497
P 4600 3100
F 0 "R6" H 4670 3146 50  0000 L CNN
F 1 "10k" H 4670 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4530 3100 50  0001 C CNN
F 3 "~" H 4600 3100 50  0001 C CNN
	1    4600 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2950 4600 2850
Wire Wire Line
	4600 2850 4850 2850
Text GLabel 6150 5150 2    50   Input ~ 0
SWCLK
Text GLabel 6150 5050 2    50   Input ~ 0
SWDIO
Text GLabel 9700 1700 0    50   Input ~ 0
MPU_INT
Text GLabel 9700 1800 0    50   Input ~ 0
MPU_CS
NoConn ~ 9700 1400
NoConn ~ 9700 1500
$Comp
L power:GND #PWR0105
U 1 1 5DA52C7F
P 9150 1200
F 0 "#PWR0105" H 9150 950 50  0001 C CNN
F 1 "GND" H 9155 1027 50  0000 C CNN
F 2 "" H 9150 1200 50  0001 C CNN
F 3 "" H 9150 1200 50  0001 C CNN
	1    9150 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0106
U 1 1 5DA53EA5
P 9150 900
F 0 "#PWR0106" H 9150 750 50  0001 C CNN
F 1 "+3V3" H 9165 1073 50  0000 C CNN
F 2 "" H 9150 900 50  0001 C CNN
F 3 "" H 9150 900 50  0001 C CNN
	1    9150 900 
	1    0    0    -1  
$EndComp
Text GLabel 4850 4850 0    50   Input ~ 0
MPU_INT
Text GLabel 4850 3750 0    50   Input ~ 0
MPU_CS
$Comp
L Regulator_Linear:MIC5205-3.3YM5 U1
U 1 1 5DA6326B
P 1400 1400
F 0 "U1" H 1400 1742 50  0000 C CNN
F 1 "MIC5205-3.3YM5" H 1400 1651 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 1400 1725 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005785A.pdf" H 1400 1400 50  0001 C CNN
	1    1400 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0107
U 1 1 5DA65CBF
P 1800 1200
F 0 "#PWR0107" H 1800 1050 50  0001 C CNN
F 1 "+3V3" H 1815 1373 50  0000 C CNN
F 2 "" H 1800 1200 50  0001 C CNN
F 3 "" H 1800 1200 50  0001 C CNN
	1    1800 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5DA66B94
P 1400 2000
F 0 "#PWR0108" H 1400 1750 50  0001 C CNN
F 1 "GND" H 1405 1827 50  0000 C CNN
F 2 "" H 1400 2000 50  0001 C CNN
F 3 "" H 1400 2000 50  0001 C CNN
	1    1400 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1700 1400 1900
Wire Wire Line
	1000 1200 1000 1300
Wire Wire Line
	1000 1300 1100 1300
Wire Wire Line
	1100 1400 1000 1400
Wire Wire Line
	1000 1400 1000 1300
Connection ~ 1000 1300
Wire Wire Line
	1700 1300 1800 1300
Wire Wire Line
	1800 1300 1800 1200
$Comp
L Device:C_Small C1
U 1 1 5DA6A7E4
P 1200 1900
F 0 "C1" V 1429 1900 50  0000 C CNN
F 1 "10u" V 1338 1900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1200 1900 50  0001 C CNN
F 3 "~" H 1200 1900 50  0001 C CNN
	1    1200 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5DA6B416
P 1600 1900
F 0 "C2" V 1829 1900 50  0000 C CNN
F 1 "10u" V 1738 1900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1600 1900 50  0001 C CNN
F 3 "~" H 1600 1900 50  0001 C CNN
	1    1600 1900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 1900 1300 1900
Wire Wire Line
	1400 2000 1400 1900
Connection ~ 1400 1900
Wire Wire Line
	1500 1900 1400 1900
Wire Wire Line
	1100 1900 1000 1900
Wire Wire Line
	1000 1900 1000 1400
Connection ~ 1000 1400
Wire Wire Line
	1700 1900 1800 1900
Wire Wire Line
	1800 1900 1800 1300
Connection ~ 1800 1300
$Comp
L Device:C_Small C3
U 1 1 5DA6FEB8
P 1950 1650
F 0 "C3" H 2042 1696 50  0000 L CNN
F 1 "470p" H 2042 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1950 1650 50  0001 C CNN
F 3 "~" H 1950 1650 50  0001 C CNN
	1    1950 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5DA706EB
P 1950 1850
F 0 "#PWR0109" H 1950 1600 50  0001 C CNN
F 1 "GND" H 1955 1677 50  0000 C CNN
F 2 "" H 1950 1850 50  0001 C CNN
F 3 "" H 1950 1850 50  0001 C CNN
	1    1950 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1850 1950 1750
Wire Wire Line
	1950 1550 1950 1400
Wire Wire Line
	1950 1400 1700 1400
Text Notes 750  900  0    118  ~ 0
3.3V REGULATOR
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 5DA77594
P 10250 2550
F 0 "J2" H 10300 2550 50  0000 L CNN
F 1 "Battery_Connector" H 10300 2450 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 10250 2550 50  0001 C CNN
F 3 "~" H 10250 2550 50  0001 C CNN
	1    10250 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0110
U 1 1 5DA78455
P 9950 2550
F 0 "#PWR0110" H 9950 2400 50  0001 C CNN
F 1 "+BATT" V 9965 2677 50  0000 L CNN
F 2 "" H 9950 2550 50  0001 C CNN
F 3 "" H 9950 2550 50  0001 C CNN
	1    9950 2550
	0    -1   -1   0   
$EndComp
$Comp
L power:-BATT #PWR0111
U 1 1 5DA79547
P 9950 2650
F 0 "#PWR0111" H 9950 2500 50  0001 C CNN
F 1 "-BATT" V 9965 2777 50  0000 L CNN
F 2 "" H 9950 2650 50  0001 C CNN
F 3 "" H 9950 2650 50  0001 C CNN
	1    9950 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9950 2550 10000 2550
Wire Wire Line
	10050 2650 9950 2650
$Comp
L Device:R R3
U 1 1 5DA9BCC0
P 3400 1200
F 0 "R3" V 3193 1200 50  0000 C CNN
F 1 "47K" V 3284 1200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3330 1200 50  0001 C CNN
F 3 "~" H 3400 1200 50  0001 C CNN
	1    3400 1200
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5DA9C2B5
P 3650 1450
F 0 "R4" H 3580 1404 50  0000 R CNN
F 1 "150k" H 3580 1495 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3580 1450 50  0001 C CNN
F 3 "~" H 3650 1450 50  0001 C CNN
	1    3650 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 1200 3650 1200
Wire Wire Line
	3650 1200 3650 1300
$Comp
L power:GND #PWR0114
U 1 1 5DAA0979
P 3650 1700
F 0 "#PWR0114" H 3650 1450 50  0001 C CNN
F 1 "GND" H 3655 1527 50  0000 C CNN
F 2 "" H 3650 1700 50  0001 C CNN
F 3 "" H 3650 1700 50  0001 C CNN
	1    3650 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 1200 3250 1200
Wire Wire Line
	3650 1600 3650 1700
$Comp
L Device:C C4
U 1 1 5DAA5180
P 3200 7150
F 0 "C4" H 3315 7196 50  0000 L CNN
F 1 "100n" H 3315 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3238 7000 50  0001 C CNN
F 3 "~" H 3200 7150 50  0001 C CNN
	1    3200 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5DAA58BE
P 3600 7150
F 0 "C5" H 3715 7196 50  0000 L CNN
F 1 "100n" H 3715 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3638 7000 50  0001 C CNN
F 3 "~" H 3600 7150 50  0001 C CNN
	1    3600 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5DAA5C0E
P 4000 7150
F 0 "C6" H 4115 7196 50  0000 L CNN
F 1 "4.7u" H 4115 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4038 7000 50  0001 C CNN
F 3 "~" H 4000 7150 50  0001 C CNN
	1    4000 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5DAA60CE
P 3200 7400
F 0 "#PWR0115" H 3200 7150 50  0001 C CNN
F 1 "GND" H 3205 7227 50  0000 C CNN
F 2 "" H 3200 7400 50  0001 C CNN
F 3 "" H 3200 7400 50  0001 C CNN
	1    3200 7400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5DAA6EFC
P 3600 7400
F 0 "#PWR0116" H 3600 7150 50  0001 C CNN
F 1 "GND" H 3605 7227 50  0000 C CNN
F 2 "" H 3600 7400 50  0001 C CNN
F 3 "" H 3600 7400 50  0001 C CNN
	1    3600 7400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5DAA71B0
P 4000 7400
F 0 "#PWR0117" H 4000 7150 50  0001 C CNN
F 1 "GND" H 4005 7227 50  0000 C CNN
F 2 "" H 4000 7400 50  0001 C CNN
F 3 "" H 4000 7400 50  0001 C CNN
	1    4000 7400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0118
U 1 1 5DAA7541
P 3200 6900
F 0 "#PWR0118" H 3200 6750 50  0001 C CNN
F 1 "+3V3" H 3215 7073 50  0000 C CNN
F 2 "" H 3200 6900 50  0001 C CNN
F 3 "" H 3200 6900 50  0001 C CNN
	1    3200 6900
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0119
U 1 1 5DAA7E8F
P 3600 6900
F 0 "#PWR0119" H 3600 6750 50  0001 C CNN
F 1 "+3V3" H 3615 7073 50  0000 C CNN
F 2 "" H 3600 6900 50  0001 C CNN
F 3 "" H 3600 6900 50  0001 C CNN
	1    3600 6900
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0120
U 1 1 5DAA824B
P 4000 6900
F 0 "#PWR0120" H 4000 6750 50  0001 C CNN
F 1 "+3V3" H 4015 7073 50  0000 C CNN
F 2 "" H 4000 6900 50  0001 C CNN
F 3 "" H 4000 6900 50  0001 C CNN
	1    4000 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6900 3200 7000
Wire Wire Line
	3600 6900 3600 7000
Wire Wire Line
	4000 6900 4000 7000
Wire Wire Line
	4000 7300 4000 7400
Wire Wire Line
	3600 7300 3600 7400
Wire Wire Line
	3200 7300 3200 7400
$Comp
L power:GND #PWR0121
U 1 1 5DAC1DC0
P 10300 3850
F 0 "#PWR0121" H 10300 3600 50  0001 C CNN
F 1 "GND" H 10305 3677 50  0000 C CNN
F 2 "" H 10300 3850 50  0001 C CNN
F 3 "" H 10300 3850 50  0001 C CNN
	1    10300 3850
	1    0    0    -1  
$EndComp
Text GLabel 3650 1200 2    50   Input ~ 0
VBAT_SENSE
Text GLabel 6150 3750 2    50   Input ~ 0
VBAT_SENSE
$Comp
L power:+BATT #PWR0122
U 1 1 5DAC02A5
P 10300 3450
F 0 "#PWR0122" H 10300 3300 50  0001 C CNN
F 1 "+BATT" V 10315 3577 50  0000 L CNN
F 2 "" H 10300 3450 50  0001 C CNN
F 3 "" H 10300 3450 50  0001 C CNN
	1    10300 3450
	0    1    1    0   
$EndComp
$Comp
L power:-BATT #PWR0123
U 1 1 5DAC8CFB
P 10300 3650
F 0 "#PWR0123" H 10300 3500 50  0001 C CNN
F 1 "-BATT" V 10315 3777 50  0000 L CNN
F 2 "" H 10300 3650 50  0001 C CNN
F 3 "" H 10300 3650 50  0001 C CNN
	1    10300 3650
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR0124
U 1 1 5DACA8B5
P 10300 3250
F 0 "#PWR0124" H 10300 3100 50  0001 C CNN
F 1 "VDD" H 10317 3423 50  0000 C CNN
F 2 "" H 10300 3250 50  0001 C CNN
F 3 "" H 10300 3250 50  0001 C CNN
	1    10300 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0125
U 1 1 5DACF007
P 3150 1200
F 0 "#PWR0125" H 3150 1050 50  0001 C CNN
F 1 "+BATT" V 3165 1327 50  0000 L CNN
F 2 "" H 3150 1200 50  0001 C CNN
F 3 "" H 3150 1200 50  0001 C CNN
	1    3150 1200
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5DAD06E4
P 5250 1200
F 0 "SW2" H 5250 1485 50  0000 C CNN
F 1 "SW_Pair" H 5250 1394 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H9.5mm" H 5250 1400 50  0001 C CNN
F 3 "~" H 5250 1400 50  0001 C CNN
	1    5250 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0126
U 1 1 5DAD2A53
P 5550 1100
F 0 "#PWR0126" H 5550 950 50  0001 C CNN
F 1 "+3V3" H 5565 1273 50  0000 C CNN
F 2 "" H 5550 1100 50  0001 C CNN
F 3 "" H 5550 1100 50  0001 C CNN
	1    5550 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 1200 5550 1200
Text GLabel 4950 1200 0    50   Input ~ 0
BTN_PAIR
Wire Wire Line
	5550 1100 5550 1200
Text GLabel 4850 4550 0    50   Input ~ 0
BTN_PAIR
$Comp
L RF:NRF24L01_Breakout U3
U 1 1 5DAF7EB6
P 9900 5350
F 0 "U3" H 10280 5396 50  0000 L CNN
F 1 "NRF24L01_Breakout" H 10280 5305 50  0000 L CNN
F 2 "RF_Module:nRF24L01_Breakout" H 10050 5950 50  0001 L CIN
F 3 "http://www.nordicsemi.com/eng/content/download/2730/34105/file/nRF24L01_Product_Specification_v2_0.pdf" H 9900 5250 50  0001 C CNN
F 4 "https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/" H 9900 5350 50  0001 C CNN "Doc"
	1    9900 5350
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0127
U 1 1 5DAFBC10
P 9900 4550
F 0 "#PWR0127" H 9900 4400 50  0001 C CNN
F 1 "+3V3" H 9915 4723 50  0000 C CNN
F 2 "" H 9900 4550 50  0001 C CNN
F 3 "" H 9900 4550 50  0001 C CNN
	1    9900 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5DAFC602
P 9900 6150
F 0 "#PWR0128" H 9900 5900 50  0001 C CNN
F 1 "GND" H 9905 5977 50  0000 C CNN
F 2 "" H 9900 6150 50  0001 C CNN
F 3 "" H 9900 6150 50  0001 C CNN
	1    9900 6150
	1    0    0    -1  
$EndComp
Text GLabel 9400 5350 0    50   Input ~ 0
NRF_CS
Text GLabel 9400 5550 0    50   Input ~ 0
NRF_CE
Text GLabel 4850 3850 0    50   Input ~ 0
NRF_CS
Text GLabel 4850 4250 0    50   Input ~ 0
NRF_CE
Text GLabel 4850 4750 0    50   Input ~ 0
NRF_INT
Text GLabel 6150 4450 2    50   Input ~ 0
SPI_MOSI
Text GLabel 6150 4350 2    50   Input ~ 0
SPI_MISO
Text GLabel 6150 4250 2    50   Input ~ 0
SPI_SCK
Text GLabel 9400 5050 0    50   Input ~ 0
SPI_MOSI
Text GLabel 9400 5150 0    50   Input ~ 0
SPI_MISO
Text GLabel 9400 5250 0    50   Input ~ 0
SPI_SCK
Text GLabel 9700 1200 0    50   Input ~ 0
SPI_SCK
Text GLabel 9700 1300 0    50   Input ~ 0
SPI_MOSI
Text GLabel 9700 1600 0    50   Input ~ 0
SPI_MISO
Text GLabel 9400 5650 0    50   Input ~ 0
NRF_INT
$Comp
L Device:C C8
U 1 1 5DA70C62
P 8650 5350
F 0 "C8" H 8765 5396 50  0000 L CNN
F 1 "10u" H 8765 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8688 5200 50  0001 C CNN
F 3 "~" H 8650 5350 50  0001 C CNN
	1    8650 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 6150 9900 6050
Wire Wire Line
	9900 4750 9900 4650
Wire Wire Line
	8650 5200 8650 4650
Wire Wire Line
	8650 4650 9900 4650
Connection ~ 9900 4650
Wire Wire Line
	9900 4650 9900 4550
Wire Wire Line
	8650 5500 8650 6050
Wire Wire Line
	8650 6050 9900 6050
Connection ~ 9900 6050
Wire Wire Line
	9900 6050 9900 5950
$Comp
L Modules:TP4056 M1
U 1 1 5DAA50C7
P 9650 3550
F 0 "M1" H 9650 4000 50  0000 C CNN
F 1 "TP4056" H 9650 3900 50  0000 C CNN
F 2 "Modules:TP4056" H 9650 3550 118 0001 C CNN
F 3 "https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Prototyping/TP4056.pdf" H 9650 3550 118 0001 C CNN
	1    9650 3550
	1    0    0    -1  
$EndComp
NoConn ~ 9100 3350
NoConn ~ 9100 3750
Wire Wire Line
	10200 3350 10300 3350
Wire Wire Line
	10300 3350 10300 3250
Wire Wire Line
	10200 3450 10300 3450
Wire Wire Line
	10200 3650 10300 3650
Wire Wire Line
	10200 3750 10300 3750
Wire Wire Line
	10300 3750 10300 3850
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5DACC230
P 6800 1200
F 0 "J1" H 6850 1550 50  0000 C CNN
F 1 "Programming_Connector" H 6850 1450 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6800 1200 50  0001 C CNN
F 3 "~" H 6800 1200 50  0001 C CNN
	1    6800 1200
	1    0    0    -1  
$EndComp
Text GLabel 7000 1300 2    50   Input ~ 0
SWCLK
Text GLabel 7000 1100 2    50   Input ~ 0
SWDIO
$Comp
L power:GND #PWR0129
U 1 1 5DAD06ED
P 7400 1300
F 0 "#PWR0129" H 7400 1050 50  0001 C CNN
F 1 "GND" H 7405 1127 50  0000 C CNN
F 2 "" H 7400 1300 50  0001 C CNN
F 3 "" H 7400 1300 50  0001 C CNN
	1    7400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 1200 7400 1200
Wire Wire Line
	7400 1200 7400 1300
NoConn ~ 7000 1400
Text Notes 6700 1600 0    50   ~ 0
no power from\nprogramming
$Comp
L Mechanical:MountingHole H4
U 1 1 5DA718A1
P 800 7450
F 0 "H4" H 900 7496 50  0000 L CNN
F 1 "MountingHole" H 900 7405 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 800 7450 50  0001 C CNN
F 3 "~" H 800 7450 50  0001 C CNN
	1    800  7450
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5DA73216
P 800 7250
F 0 "H3" H 900 7296 50  0000 L CNN
F 1 "MountingHole" H 900 7205 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 800 7250 50  0001 C CNN
F 3 "~" H 800 7250 50  0001 C CNN
	1    800  7250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5DA7349F
P 800 7050
F 0 "H2" H 900 7096 50  0000 L CNN
F 1 "MountingHole" H 900 7005 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 800 7050 50  0001 C CNN
F 3 "~" H 800 7050 50  0001 C CNN
	1    800  7050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 5DA7374E
P 800 6850
F 0 "H1" H 900 6896 50  0000 L CNN
F 1 "MountingHole" H 900 6805 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 800 6850 50  0001 C CNN
F 3 "~" H 800 6850 50  0001 C CNN
	1    800  6850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5DA78845
P 1400 3650
F 0 "D1" H 1393 3866 50  0000 C CNN
F 1 "STATUS_LED" H 1393 3775 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1400 3650 50  0001 C CNN
F 3 "~" H 1400 3650 50  0001 C CNN
	1    1400 3650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5DA7D020
P 1400 3250
F 0 "R1" H 1470 3296 50  0000 L CNN
F 1 "2k" H 1470 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1330 3250 50  0001 C CNN
F 3 "~" H 1400 3250 50  0001 C CNN
	1    1400 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3400 1400 3500
$Comp
L power:GND #PWR0130
U 1 1 5DA82ACA
P 1400 3900
F 0 "#PWR0130" H 1400 3650 50  0001 C CNN
F 1 "GND" H 1405 3727 50  0000 C CNN
F 2 "" H 1400 3900 50  0001 C CNN
F 3 "" H 1400 3900 50  0001 C CNN
	1    1400 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3800 1400 3900
Text GLabel 1400 3000 1    50   Input ~ 0
LED_STATUS
Text GLabel 1800 3000 1    50   Input ~ 0
LED_POWER
$Comp
L Device:R R2
U 1 1 5DA85E55
P 1800 3250
F 0 "R2" H 1870 3296 50  0000 L CNN
F 1 "2k" H 1870 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1730 3250 50  0001 C CNN
F 3 "~" H 1800 3250 50  0001 C CNN
	1    1800 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5DA8600A
P 1800 3650
F 0 "D2" H 1793 3866 50  0000 C CNN
F 1 "POWER_LED" H 1793 3775 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1800 3650 50  0001 C CNN
F 3 "~" H 1800 3650 50  0001 C CNN
	1    1800 3650
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 5DA8626D
P 1800 3900
F 0 "#PWR0131" H 1800 3650 50  0001 C CNN
F 1 "GND" H 1805 3727 50  0000 C CNN
F 2 "" H 1800 3900 50  0001 C CNN
F 3 "" H 1800 3900 50  0001 C CNN
	1    1800 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 3900 1800 3800
Wire Wire Line
	1800 3500 1800 3400
Wire Wire Line
	1800 3100 1800 3000
Wire Wire Line
	1400 3100 1400 3000
Text GLabel 6150 3950 2    50   Input ~ 0
LED_STATUS
Text GLabel 6150 4050 2    50   Input ~ 0
LED_POWER
$Comp
L power:GND #PWR0132
U 1 1 5DA98822
P 9600 2000
F 0 "#PWR0132" H 9600 1750 50  0001 C CNN
F 1 "GND" H 9605 1827 50  0000 C CNN
F 2 "" H 9600 2000 50  0001 C CNN
F 3 "" H 9600 2000 50  0001 C CNN
	1    9600 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 2000 9600 1900
Wire Wire Line
	9600 1900 9700 1900
Wire Wire Line
	4950 1200 5050 1200
$Comp
L Modules:MPU-9250 M2
U 1 1 5DAB5746
P 10250 1450
F 0 "M2" H 10250 2200 50  0000 C CNN
F 1 "MPU-9250" H 10250 2100 50  0000 C CNN
F 2 "Modules:Module_10pin_15x26mm" H 11900 2300 50  0001 C CNN
F 3 "" H 11900 2300 50  0001 C CNN
	1    10250 1450
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0133
U 1 1 5DAEE703
P 1000 1200
F 0 "#PWR0133" H 1000 1050 50  0001 C CNN
F 1 "VDD" H 1017 1373 50  0000 C CNN
F 2 "" H 1000 1200 50  0001 C CNN
F 3 "" H 1000 1200 50  0001 C CNN
	1    1000 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5DB02C74
P 4600 2400
F 0 "R5" H 4670 2446 50  0000 L CNN
F 1 "10k" H 4670 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4530 2400 50  0001 C CNN
F 3 "~" H 4600 2400 50  0001 C CNN
	1    4600 2400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0134
U 1 1 5DB04998
P 4600 2150
F 0 "#PWR0134" H 4600 2000 50  0001 C CNN
F 1 "+3V3" H 4615 2323 50  0000 C CNN
F 2 "" H 4600 2150 50  0001 C CNN
F 3 "" H 4600 2150 50  0001 C CNN
	1    4600 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2150 4600 2250
Wire Wire Line
	4600 2550 4600 2650
Wire Wire Line
	4600 2650 4850 2650
$Comp
L Switch:SW_Push SW1
U 1 1 5DB0C86A
P 3450 2950
F 0 "SW1" V 3404 3098 50  0000 L CNN
F 1 "Button_Reset" V 3495 3098 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H4.3mm" H 3450 3150 50  0001 C CNN
F 3 "~" H 3450 3150 50  0001 C CNN
	1    3450 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 3150 3450 3250
Wire Wire Line
	3450 3250 4200 3250
Wire Wire Line
	4200 3250 4200 3100
Wire Wire Line
	4600 2650 4200 2650
Wire Wire Line
	3450 2650 3450 2750
Connection ~ 4600 2650
Wire Wire Line
	4200 2800 4200 2650
Connection ~ 4200 2650
Wire Wire Line
	4200 2650 3450 2650
Wire Wire Line
	4200 3350 4200 3250
Connection ~ 4200 3250
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DB266A6
P 10000 2550
F 0 "#FLG0101" H 10000 2625 50  0001 C CNN
F 1 "PWR_FLAG" H 10000 2723 50  0000 C CNN
F 2 "" H 10000 2550 50  0001 C CNN
F 3 "~" H 10000 2550 50  0001 C CNN
	1    10000 2550
	1    0    0    -1  
$EndComp
Connection ~ 10000 2550
Wire Wire Line
	10000 2550 10050 2550
NoConn ~ 6150 5250
NoConn ~ 6150 4950
NoConn ~ 6150 4850
NoConn ~ 6150 4750
NoConn ~ 6150 4650
NoConn ~ 6150 4550
NoConn ~ 4850 5250
NoConn ~ 4850 5150
NoConn ~ 4850 5050
NoConn ~ 4850 4950
NoConn ~ 4850 4650
NoConn ~ 4850 4450
NoConn ~ 4850 4350
NoConn ~ 4850 4150
NoConn ~ 4850 4050
NoConn ~ 4850 3950
NoConn ~ 4850 3550
NoConn ~ 4850 3450
NoConn ~ 4850 3350
NoConn ~ 4850 3050
NoConn ~ 4850 3150
NoConn ~ 6150 3850
NoConn ~ 6150 4150
Wire Wire Line
	9150 900  9150 1000
Wire Wire Line
	9150 1000 9700 1000
Wire Wire Line
	9150 1200 9150 1100
Wire Wire Line
	9150 1100 9700 1100
$EndSCHEMATC

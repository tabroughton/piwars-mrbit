"""
File: mrBit_qtr_8c.py
Author: Tom Broughton
Date: 19/02/2017

This script contols the Pololu qtr-8c line IR sensor array

As this is via the RPi GPIO we are running at 3V3 so ensure the pads 
for 3V3 are soldered on the IR module 

VCC -> 3V3
GND -> GND
LEDON -> GPIO 8 (This is the Pi's only PWM port

The LEDON can be toggled on and off inbetween readings to conserve power
LEDON can also be poweredd by a PWM output - reducing power consumption even further
The module generally pulls around 100mA when using all sensors at 5v
This script, based on 3V3, PWM and turning off LEDs inbetween readings aims to 
reduce current to around 10mA (so the Pi can reserve power for other sensors and actuators).

"""



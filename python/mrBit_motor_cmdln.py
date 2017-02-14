"""
    implementation of mrBit Motor Driver via Command line text
    Date: 14/02/2107
    Atuhor: Tom Broughton

    To test the motor control from command line

    Hardware:
    * GrovePi Hat on Raspberry Pi 3 (or other with Bluetooth)
    * GrovePi connected to Sparkfun Monster Moto Shield
    * 2x motors connected to motor driver
"""

import grovepi
import mrBit_moto_driver

#main loop
while 1:
    try:
        #store any data from the command line input
        btData = raw_input("Enter Motor Command: ")
        if btData: mrBit_moto_driver.motorCMD(btData)

    except KeyboardInterrupt:
        mrBit_moto_driver.motorsOff();
        break

    except Exception as e:
        print str(e)
    	# Turn motors off
        mrBit_moto_driver.motorsOff()
        break

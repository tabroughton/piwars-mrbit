"""
    implementation of mrBit Moto Driver over bluetooth
    Date: 14/02/2107
    Atuhor: Tom Broughton

    Hardware:
    * GrovePi Hat on Raspberry Pi 3 (or other with Bluetooth)
    * GrovePi connected to Sparkfun Monster Moto Shield
    * 2x motors connected to motor driver
    * Bluetooth from Android running App Inventor 2

    BT Commands:
    F - drive both motors forwards
    B - drive both motors backwards
    R - Turn motor A CW and B CCW
    L - Turn motor A CCW and B CW
    S - stop motors
    PWM-[val] where 0 <= [val] <= 255
"""

import grovepi
import bluetooth
import mrBit_moto_driver

#set up bluetooth server

server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
port = 0x1101 #make sure you've set up an RFCOMM channel=1 using sdptool
server_sock.bind(("",port))
server_sock.listen(1)

try:
    client_sock,address = server_sock.accept()
    print "Accepted connection from ",address
    
except Exception as e:
    print str(e)
    server_sock.close()

while 1:
    try:
        #store any data from the bluetooth socket and print it
        btData = client_sock.recv(1024)
        if btData:  mrBit_moto_driver.motorCMD(btData)

    except KeyboardInterrupt:
        mrBit_moto_driver.motorsOff();
        break

    except Exception as e:
        print str(e)
        mrBit_moto_driver.motorsOff()
        break

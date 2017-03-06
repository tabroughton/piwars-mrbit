import grovepi as gp
import mrBit_moto_driver as moto
from time import sleep

IR1_PIN = 16 #A2
IR2_PIN = 17 #A3
THRESHOLD = 200

#SETUP
gp.pinMode(IR1_PIN, "INPUT")
gp.pinMode(IR2_PIN, "INPUT")
print("off we go")
#LOOP
while True:
    try:
        val1 = gp.analogRead(IR1_PIN)
        val2 = gp.analogRead(IR2_PIN)
        print ("IR 1: %d, IR2: %d" % (val1, val2))

        if val1 > THRESHOLD:
            moto.motorStop(moto.MOT_1)
        else:
            moto.motorGo(moto.MOT_1, moto.MOTOR_CW, 255)

        if val2 > THRESHOLD:
            moto.motorStop(moto.MOT_2)
        else:
            moto.motorGo(moto.MOT_2, moto.MOTOR_CW, 255)

        sleep(.1)

    except Exception as e:
        moto.motorsOff()
        print str(e)
        break

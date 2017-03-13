"""
    Control Motors via Grove Shield on Pi using Command line text
    Date: 14/02/2017
    Atuhor: Tom Broughton

    Intention of this script is to test the motor control
    the main function to call is motorCMD(mCMD) with below text commands

    Hardware:
    * GrovePi Hat on Raspberry Pi 3 (or other with Bluetooth)
    * GrovePi connected to Sparkfun Monster Moto Shield
    * 2x motors connected to motor driver

    Text Commands:
    F - drive both motors forwards
    B - drive both motors backwards
    R - Turn motor A CW and B CCW
    L - Turn motor A CCW and B CW
    S - stop motors
    PWM-[val] where 0 <= [val] <= 255
"""

import grovepi
from time import sleep

#motor control constants
MOTOR_CW = 1
MOTOR_CCW = 0
MOT_1 = 0
MOT_2 = 1

#pin val constants
HIGH = 1
LOW = 0
PWM_MAX = 255

#set up motor driver pins
mot1Pins = [7, 8]
mot2Pins = [3, 4]
motorPins = [mot1Pins, mot2Pins]
motPWMPins = [5, 6]

#Store the values of the current motor direction and speed
motDirection = [MOTOR_CW, MOTOR_CW]
motPWM = [PWM_MAX, PWM_MAX]

#setup pinmodes
for mot in range(0, 2):
    for pin in range(0, 2): grovepi.pinMode(motorPins[mot][pin],"OUTPUT")
    grovepi.pinMode(motPWMPins[mot],"OUTPUT")

#define motor functions
def motorGo(motor, direction, pwm = PWM_MAX):
    grovepi.digitalWrite(motorPins[motor][direction], HIGH)
    grovepi.digitalWrite(motorPins[motor][not direction], LOW)
    grovepi.analogWrite(motPWMPins[motor], pwm)

def motorStop(motor):
    for pin in range(0, 2):
        grovepi.digitalWrite(motorPins[motor][pin], LOW)
    grovepi.analogWrite(motPWMPins[motor], LOW)

def motorsStopDead(ReverseTime=0.1):
    for mot in range(0, 2):
        motDirection[mot] = MOTOR_CCW if motDirection[mot] == MOTOR_CW else MOTOR_CW
    moveMotorsAandB()
    sleep(ReverseTime)
    motorsOff()

def motorsOff():
    for mot in range(0, 2): motorStop(mot)
    print ("stopped")

def motorsForward():
    motDirection[MOT_1] = MOTOR_CW
    motDirection[MOT_2] = MOTOR_CW
    moveMotorsAandB()
    print ("moving fowards")

def motorsBackwards():
    #test backwards both
    motDirection[MOT_1] = MOTOR_CCW
    motDirection[MOT_2] = MOTOR_CCW
    moveMotorsAandB()
    print ("moving backwards")

def motorsTurnRight():
    #test right
    motDirection[MOT_1] = MOTOR_CW
    motDirection[MOT_2] = MOTOR_CCW
    moveMotorsAandB()
    print ("turning right")

def motorsTurnLeft():
    #test left
    motDirection[MOT_1] = MOTOR_CCW
    motDirection[MOT_2] = MOTOR_CW
    moveMotorsAandB()
    print ("turning left")

def changeSpeedAandB(aVal):
    if aVal.isdigit():
        pwmVal = int(aVal)
        if pwmVal >= 0 and pwmVal <= PWM_MAX:
            motPWM[MOT_1] = pwmVal
            motPWM[MOT_2] = pwmVal
            moveMotorsAandB()
            print ("speed = %d" % pwmVal)

def setMotorSpeed(motor, pwmVal):
    if pwmVal > MAX_PWM or pwmVal < 0: return
    motPWM[motor] = pwmVal

def setMotorDirection(motor, direction):
    if direction != MOTOR_CW and direction < != MOTOR_CCW: return
    motDirection[motor] = direction

def moveMotorsAandB():
    motorGo(MOT_1, motDirection[MOT_1], motPWM[MOT_1])
    motorGo(MOT_2, motDirection[MOT_2], motPWM[MOT_2])

def motorInvalidCMD():
    print("Invalid command")

def motorAction(action):
    return{
        'F' : motorsForward,
        'B' : motorsBackwards,
        'R' : motorsTurnRight,
        'L' : motorsTurnLeft,
        'S' : motorsOff,
        'PWM' : changeSpeedAandB,
    }.get(action, motorInvalidCMD)

def motorCMD(mCMD):
    print "received [%s]" % mCMD
    action = mCMD.split("-")
    motorAction(action[0])() if action[0] != "PWM" else motorAction(action[0])(action[1])

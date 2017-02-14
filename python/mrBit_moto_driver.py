"""
    Control Motors via Grove Shield on Pi using Command line text
    Date: 14/02/2107
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

#motor control constants
MOTOR_CW = 0
MOTOR_CCW = 1
MOT_A = 0
MOT_B = 1

#pin val constants
HIGH = 1
LOW = 0
PWM_MAX = 255

#set up motor driver pins
motAPins = [7, 8]
motBPins = [4, 3]
motorPins = [motAPins, motBPins] #motor A is 0, motor B is 1
motPWMPins = [5, 6]

#Store the values of the current motor direction and speed
motDirection = [MOTOR_CW, MOTOR_CW]
motPWM = [PWM_MAX, PWM_MAX]

#setup pinmodes
for mot in range(0, 2):
    for pin in range(0, 2): grovepi.pinMode(motorPins[mot][pin],"OUTPUT")
    grovepi.pinMode(motPWMPins[mot],"OUTPUT")

#define motor functions
def motorGo(motor, direction, pwm = 0):
    #when we set the direction of the motor top HIGH we have to turn the alternative direction LOW
    alt_direction = 1 if direction == 0 else 0
    grovepi.digitalWrite(motorPins[motor][direction], HIGH)
    grovepi.digitalWrite(motorPins[motor][alt_direction], LOW)
    grovepi.analogWrite(motPWMPins[motor], pwm)

def motorStop(motor):
    grovepi.digitalWrite(motorPins[motor][0], LOW)
    grovepi.digitalWrite(motorPins[motor][1], LOW)
    grovepi.analogWrite(motPWMPins[motor], LOW)

def motorsOff():
    for mot in range(0, 2): motorStop(mot)
    print ("stopped")

def motorsForward():
    motDirection[MOT_A] = MOTOR_CW
    motDirection[MOT_B] = MOTOR_CW
    moveMotorsAandB()
    print ("moving fowards")

def motorsBackwards():
    #test backwards both
    motDirection[MOT_A] = MOTOR_CCW
    motDirection[MOT_B] = MOTOR_CCW
    moveMotorsAandB()
    print ("moving backwards")

def motorsTurnRight():
    #test right
    motDirection[MOT_A] = MOTOR_CW
    motDirection[MOT_B] = MOTOR_CCW
    moveMotorsAandB()
    print ("turning right")

def motorsTurnLeft():
    #test left
    motDirection[MOT_A] = MOTOR_CCW
    motDirection[MOT_B] = MOTOR_CW
    moveMotorsAandB()
    print ("turning left")

def motorsSpeed(aVal):
    if aVal.isdigit():
        pwmVal = int(aVal)
        if pwmVal >= 0 and pwmVal <= PWM_MAX:
            motPWM[MOT_A] = pwmVal
            motPWM[MOT_B] = pwmVal
            moveMotorsAandB()
            print ("speed = %d" % pwmVal)

def moveMotorsAandB():
    motorGo(MOT_A, motDirection[MOT_A], motPWM[MOT_A])
    motorGo(MOT_B, motDirection[MOT_B], motPWM[MOT_B])

def motorInvalidCMD():
    print("Invalid command")

def motorAction(action):
    return{
        'F' : motorsForward,
        'B' : motorsBackwards,
        'R' : motorsTurnRight,
        'L' : motorsTurnLeft,
        'S' : motorsOff,
        'PWM' : motorsSpeed,
    }.get(action, motorInvalidCMD)

def motorCMD(mCMD):
    print "received [%s]" % mCMD
    action = mCMD.split("-")
    motorAction(action[0])() if action[0] != "PWM" else motorAction(action[0])(action[1])

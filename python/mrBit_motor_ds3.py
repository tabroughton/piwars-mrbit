"""
    Control Motors via Grove Shield on Pi using PS3 sixaxis dualshock3 controller
    Date: 04/03/2017
    Atuhor: Tom Broughton

    This script is dependent on Tom Oinn's approxeng.input package http://pythonhosted.org/approxeng.input/index.html
    Install the package with pip insall approxeng.input
    You will also need to have installed sixpair http://www.pabr.org/sixlinux/sixlinux.en.html

    Hardware:
    * GrovePi Hat on Raspberry Pi 3 (or other with Bluetooth)
    * GrovePi connected to Sparkfun Monster Moto Shield
    * 2x motors connected to motor driver
    * PS3 siaxis dualshock3 controller

    When moved vertically the thumbstricks will operate the motors controlling speed forwards and backwards
    Pressing the square button on the controller will calibrate the thumbsticks to centre (0)
"""
import mrBit_moto_driver as moto
from approxeng.input.dualshock3 import DualShock3, CONTROLLER_NAMES
from approxeng.input.asyncorebinder import ControllerResource
from time import sleep

MIN_PWM = 55
MAX_PWM = 255

pwmOffset = MAX_PWM - MIN_PWM
lastLAxis = 0
lastRAxis = 0

with ControllerResource(controller=DualShock3(dead_zone=0.1, hot_zone=0.2), device_name=CONTROLLER_NAMES) as joystick:
    # Bind the square button to call the set_axis_centres function
    joystick.buttons.register_button_handler(joystick.axes.set_axis_centres, joystick.BUTTON_SQUARE)

    while 1:
        sleep(0.1)
        try:
            lAxis = joystick.AXIS_LEFT_VERTICAL.corrected_value()
            rAxis = joystick.AXIS_RIGHT_VERTICAL.corrected_value()

            if lAxis != lastLAxis:
                if lAxis == 0:
                    moto.motorStop(moto.MOT_1)
                    print("Left Reading: %f => stop left" % lAxis)
                else:
                    lPWM = int(round(lAxis * pwmOffset))
                    if lPWM > 0:
                        lDir = moto.MOTOR_CW
                        lPWM += MIN_PWM
                    else:
                        lDir = moto.MOTOR_CCW
                        lPWM -= MIN_PWM
                        lPWM *= -1

                    print("Left Reading: %f => Motor: 1, Direction: %d, PWM: %d" % (lAxis, lDir, lPWM))
                    moto.motorGo(moto.MOT_1, lDir, lPWM)
                lastLAxis = lAxis

            if rAxis != lastRAxis:
                if rAxis == 0:
                    moto.motorStop(moto.MOT_2)
                    print("Right Reading: %f => stop right" % rAxis)
                else:
                    rPWM = int(round(rAxis * pwmOffset))
                    if rPWM > 0:
                        rDir = moto.MOTOR_CW
                        rPWM += MIN_PWM
                    else:
                        rDir = moto.MOTOR_CCW
                        rPWM -= MIN_PWM
                        rPWM *= -1

                    print("Right Reading: %f => Motor: 2, Direction: %d, PWM: %d" % (rAxis, rDir, rPWM))
                    moto.motorGo(moto.MOT_2, rDir, rPWM)
                lastRAxis = rAxis

        except KeyboardInterrupt:
            mrBit_moto_driver.motorsOff();
            break

        except Exception as e:
            print str(e)
            mrBit_moto_driver.motorsOff()
            break

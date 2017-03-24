#   Mr Bit Controller Package -
#   Atuhor: Tom Broughton (@dpolymath)
#   Date: 23/03/2017
#
#   Controlls motors for manaul and automatic challenges for PiWars 2017
#
# #############################################################################
#   MIT License
#
#   Copyright (c) 2017 Tom Broughton https://github.com/tabroughton/
#
#   Permission is hereby granted, free vof charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.
###############################################################################

import grovepi as gp
import mrBit_pid as pid
import mrBit_qtr_8rc as qtr

class MrBit_Motor:
    """ deals with a single motor (or a number of motors connected together)"""

    HIGH = 1
    LOW = 0

    def __init__(self, CWPin, CCWPin, PWMPin, Speed=0, Direction="CW"):
        self.cwPin = CWPin
        self.ccwPin = CCWPin
        self.pwmPin = PWMPin
        self.maxSpeed = 255
        self.minSpeed = -255
        self.lastSpeed = 0
        self.speed = 0

        gp.pinMode(self.cwPin, "OUTPUT")
        gp.pinMode(self.ccwPin, "OUTPUT")
        gp.pinMode(self.pwmPin, "OUTPUT")

        self.set_speed(Speed)
        self.set_direction(Direction)

    def set_speed(self, Speed):
        if Speed == self.lastSpeed: return
        self.lastSpeed = Speed
        if Speed > self.maxSpeed:
            self.speed = self.maxSpeed
        elif Speed < self.minSpeed:
            self.speed = self.minSpeed
        else:
            self.speed = Speed

        if self.speed < 0:
            self.set_direction("CCW")
            self.speed = -self.speed
        else:
            self.set_direction("CW")

    def set_direction(self, Direction):
        if Direction != "CW" and Direction != "CCW": return
        self.direction = Direction

    def on(self):
        if self.direction == "CW":
            gp.digitalWrite(self.cwPin, MrBit_Motor.HIGH)
            gp.digitalWrite(self.ccwPin, MrBit_Motor.LOW)
        else:
            gp.digitalWrite(self.ccwPin, MrBit_Motor.HIGH)
            gp.digitalWrite(self.cwPin, MrBit_Motor.LOW)
        gp.analogWrite(self.pwmPin, int(self.speed))

    def off(self):
        print("turning motor off")
        gp.digitalWrite(self.cwPin, MrBit_Motor.LOW)
        gp.digitalWrite(self.ccwPin, MrBit_Motor.LOW)
        gp.analogWrite(self.pwmPin, MrBit_Motor.LOW)

    def get_speed(self): return self.speed
    def get_direction(self): return self.direction

class MrBit_Motor_Driver(object):

    def __init__(self):
        self.driving = False
        self.motorLeft = MrBit_Motor(8, 7, 5)
        self.motorRight = MrBit_Motor(4, 3, 6)

    def drive(self, SpeedLeft, SpeedRight):
        print("Speeds %d, %d" % (SpeedLeft, SpeedRight))
        self.motorLeft.set_speed(SpeedLeft)
        self.motorRight.set_speed(SpeedRight)
        self.motorLeft.on()
        self.motorRight.on()

    def sharp_left(self, Speed):
        self.drive(-Speed, Speed)

    def sharp_right(self, Speed):
        self.drive(Speed, -Speed)

    def stop(self):
        print("driver off")
        self.motorLeft.off()
        self.motorRight.off()
        self.driving = False

    def start(self):
        print("driver on")
        self.driving = True

    def __del__(self):
        self.stop()

class MrBit_DS3_Driver(MrBit_Motor_Driver):

    def __init__(self):
        self.pwmOffset = 255 - 55
        super(MrBit_DS3_Driver, self).__init__()

    def drive(self, leftSpeed, rightSpeed):
        speeds = (int(leftSpeed * self.pwmOffset), int(rightSpeed * self.pwmOffset))
        super(MrBit_DS3_Driver, self).drive(*speeds)

class MrBit_Auto_Driver(MrBit_Motor_Driver):
    """ Abstract class for auto driver challenges """

    def __init__(self, Setpoint,  Kp,  Ki,  Kd, OutputLeft, OutputRight, ControllerDirectionLeft, ControllerDirectionRight):
        self.leftPid = pid.MrBit_PID(Setpoint,  Kp,  Ki,  Kd, OutputLeft, ControllerDirectionLeft)
        self.rightPid = pid.MrBit_PID(Setpoint,  Kp,  Ki,  Kd, OutputRight, ControllerDirectionRight)
        super(MrBit_Auto_Driver, self).__init__()

    def drive(self, withPid=True):
        self.set_position()
        self.leftPid.compute(self.position)
        self.rightPid.compute(self.position)
        leftSpeed = self.leftPid.get_output()
        rightSpeed = self.rightPid.get_output()
        super(MrBit_Auto_Driver, self).drive(leftSpeed, rightSpeed)

    def stop(self):
        self.leftPid.set_mode(self.leftPid.MANUAL)
        self.rightPid.set_mode(self.rightPid.MANUAL)
        super(MrBit_Auto_Driver, self).stop()

    def start(self):
        self.set_position()
        self.leftPid.set_output(self.baseSpeed)
        self.rightPid.set_output(self.baseSpeed)
        self.leftPid.set_mode(self.leftPid.AUTOMATIC)
        self.rightPid.set_mode(self.rightPid.AUTOMATIC)
        super(MrBit_Auto_Driver, self).start()

    def set_constants(self):
        Kp = input("Enter a Kp val: ")
        Ki = input("Enter a Ki val: ")
        Kd = input("Enter a Kd val: ")
        self.leftPid.set_tunings(Kp, Ki, Kd)
        self.rightPid.set_tunings(Kp, Ki, Kd)

    def set_position(self):
        raise NotImplementedError

class MrBit_Straight_Line(MrBit_Auto_Driver):

    def __init__(self):
        self.kp = 1.4
        self.ki = 0.5
        self.kd = 0.3
        self.baseSpeed = 250
        self.setpoint = 0
        self.leftIRSensor = MrBit_IR_Distance_Sensor(16)
        self.rightIRSensor = MrBit_IR_Distance_Sensor(17)
        super(MrBit_Straight_Line, self).__init__(self.setpoint,  self.kp,  self.ki, self.kd, self.baseSpeed, self.baseSpeed, pid.MrBit_PID.REVERSE, pid.MrBit_PID.DIRECT)

    def set_position(self):
        leftIRReading = self.leftIRSensor.getReading()
        rightIRReading = self.rightIRSensor.getReading()
        print("readings L: %d, R: %d" %(leftIRReading, rightIRReading))
        self.position = leftIRReading - rightIRReading


class MrBit_Minimal_Maze(MrBit_Auto_Driver):

    def __init__(self, Side="LEFT"):
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.baseSpeed = 75
        self.setpoint = 150
        if Side == "RIGHT":
            self.wallIRSensor = MrBit_IR_Distance_Sensor(17)
            self.frontIRSensor = MrBit_IR_Distance_Sensor(16)
            leftDirection = pid.MrBit_PID.DIRECT
            rightDirection = pid.MrBit_PID.REVERSE
        else:
            self.wallIRSensor = MrBit_IR_Distance_Sensor(16)
            self.frontIRSensor = MrBit_IR_Distance_Sensor(17)
            leftDirection = pid.MrBit_PID.REVERSE
            rightDirection = pid.MrBit_PID.DIRECT
        super(MrBit_Minimal_Maze, self).__init__(self.setpoint,  self.kp,  self.ki, self.kd, self.baseSpeed, self.baseSpeed, leftDirection, rightDirection)
        self.leftPid.set_output_limits(-255, 255)
        self.rightPid.set_output_limits(-255, 255)

    def set_position(self):
        wallReading = self.wallIRSensor.getReading()
        frontReading = self.frontIRSensor.getReading()
        self.position = wallReading
        if frontReading > 150: self.position += frontReading
        print("W: %d, F: %d, Pos: %d" % (wallReading, frontReading, self.position))


class MrBit_Line_Following(MrBit_Auto_Driver):

    def __init__(self):
        self.kp = 0.08
        self.ki = 0
        self.kd = 0
        self.baseSpeed = 75
        self.qtr8rc = qtr.MrBit_QTR_8RC()
        self.setpoint = 3500
        self.calibrate()
        super(MrBit_Line_Following, self).__init__(self.setpoint,  self.kp,  self.ki,  self.kd, self.baseSpeed, self.baseSpeed, pid.MrBit_PID.DIRECT, pid.MrBit_PID.REVERSE)
        self.leftPid.set_output_limits(-255, 255)
        self.rightPid.set_output_limits(-255, 255)

    def set_position(self):
        self.qtr8rc.emitters_on()
        self.position = self.qtr8rc.read_line()
        self.qtr8rc.emitters_off()
        print(self.position)

    def calibrate(self):
        """calibration of sensors, rotates robot several times over sensors
            and takes a reading.  Readings displayed and need to be accepted
            by user before continuing"""
        manualDriver = MrBit_Motor_Driver()
        approveCal = False
        while not approveCal:
            print("calibrating")
            self.qtr8rc.initialise_calibration()
            self.qtr8rc.emitters_on()
            for t in range(0, 4):
                for i in range(0, 10):
                    if t % 2 ==  0:
                        manualDriver.drive(100, 100)
                    else:
                        manualDriver.drive(-100, -100)
                    self.qtr8rc.calibrate_sensors()
                    self.qtr8rc.wp.delay(15)
            self.qtr8rc.emitters_off
            manualDriver.stop()

            print("calibration complete")
            print("max vals")
            self.qtr8rc.print_sensor_values(self.qtr8rc.calibratedMax)
            print("calibration complete")
            print("min vals")
            self.qtr8rc.print_sensor_values(self.qtr8rc.calibratedMin)
            approved = raw_input("happy with calibrtion (Y/n)? ")
            if approved == "Y": approveCal = True
        del manualDriver

    def stop(self):
        self.qtr8rc.emitters_off()
        super(MrBit_Line_Following, self).stop()


class MrBit_IR_Distance_Sensor:
    """ simple class for a single analog sensor, makes use of a global grovepi library"""

    def __init__(self, Pin):
        self.irPin = Pin
        gp.pinMode(self.irPin, "INPUT")

    def getPin(self): return self.irPin

    def getReading(self):
        return gp.analogRead(self.irPin)

class MrBit_Controller:
    """controller class to toggle between modes and select challenges if in Auto mode"""

    AUTOMATIC = 0
    MANUAL = 1

    def __init__(self, Mode=1):
        self.drivers = dict(MD=MrBit_DS3_Driver, SL=MrBit_Straight_Line, MM=MrBit_Minimal_Maze, LF=MrBit_Line_Following)
        self.set_mode(Mode)

    def toggle_mode(self):
        mode = MrBit_Controller.AUTOMATIC if self.mode == MrBit_Controller.MANUAL else MrBit_Controller.MANUAL
        print("mode: %d" % self.mode)
        self.set_mode(mode)

    def get_mode(self):
        return self.mode

    def set_mode(self, Mode):
        self.driverReady = False
        if Mode != MrBit_Controller.MANUAL and Mode != MrBit_Controller.AUTOMATIC: return
        self.mode = Mode
        if self.mode == MrBit_Controller.MANUAL:
            self.change_driver("MD")
        elif self.mode == MrBit_Controller.AUTOMATIC:
            self.set_challenge()

    def set_challenge(self):
        validChallenge = False
        while not validChallenge:
            Challenge = raw_input("Set challenge: ")
            if Challenge in self.drivers: validChallenge = True
        self.change_driver(Challenge)

    def change_driver(self, Driver):
        if Driver not in self.drivers: return
        if hasattr(self, 'driver'):
            del self.driver
        print("changing driver to %s:" % Driver)
        self.driver = self.drivers[Driver]()
        self.driverReady = True
        self.driver.start()

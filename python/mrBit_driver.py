#   Mr Bit Controller Package -
#   Atuhor: Tom Broughton (@dpolymath)
#   Date: 23/03/2017
#
#   Controls motors for manaul and automatic challenges for PiWars 2017
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
    """ Deals with single motor output on the sparkfun monster moto shield.
    """

    HIGH = 1
    LOW = 0

    def __init__(self, CWPin, CCWPin, PWMPin, Speed=0, Direction="CW"):
        """ Creates an instance of a motor.

            :param CWPin:
                The pin driving clockwise motoion

            :param CCWPin:
                The pin on the motor driver driving counterclockwise motoion

            :param PWMPin:
                The pin varying the speed of the motor

            :Speed:
                The initial speed to set the motor - note a negative value
                will result in a counterclockwise direction.
        """
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
        """ Sets the speed of the motor.

            :param Speed:
                A value (between 255 and -255) mapped to the voltage of the motor.
                A negative value will result in counterclockwise direction of motor.
        """
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
        """ Sets the direction of the motor, either clockwise or
            counterclockwise.

            params: Direction can be "CW" or "CCW"
        """
        if Direction != "CW" and Direction != "CCW": return
        self.direction = Direction

    def on(self):
        """ Turns the motor on.
        """
        if self.direction == "CW":
            gp.digitalWrite(self.cwPin, MrBit_Motor.HIGH)
            gp.digitalWrite(self.ccwPin, MrBit_Motor.LOW)
        else:
            gp.digitalWrite(self.ccwPin, MrBit_Motor.HIGH)
            gp.digitalWrite(self.cwPin, MrBit_Motor.LOW)
        gp.analogWrite(self.pwmPin, int(self.speed))

    def off(self):
        """ Turns the motor off.
        """
        print("turning motor off")
        gp.digitalWrite(self.cwPin, MrBit_Motor.LOW)
        gp.digitalWrite(self.ccwPin, MrBit_Motor.LOW)
        gp.analogWrite(self.pwmPin, MrBit_Motor.LOW)

    def get_speed(self): return self.speed
    def get_direction(self): return self.direction

class MrBit_Motor_Driver(object):
    """ Super class for all motor driver classes, drives the motors attached
        to the monster moto shield.
    """

    def __init__(self):
        """ Creates an instance of the motor driver and two instances of motor.
        """
        self.driving = False
        self.motorLeft = MrBit_Motor(8, 7, 5)
        self.motorRight = MrBit_Motor(4, 3, 6)

    def drive(self, SpeedLeft, SpeedRight):
        """ Called to drive both left and right motors.

            :param SpeedLeft:
                The speed at which the left motor should turn.

            :param SpeedRight:
                The speed at which the right motor should turn.
        """
        print("Speeds %d, %d" % (SpeedLeft, SpeedRight))
        self.motorLeft.set_speed(SpeedLeft)
        self.motorRight.set_speed(SpeedRight)
        self.motorLeft.on()
        self.motorRight.on()

    def stop(self):
        """ Turns off both motors to stop the robot.
        """
        print("driver off")
        self.motorLeft.off()
        self.motorRight.off()
        self.driving = False

    def start(self):
        """ Called to start the driver, simply sets the driving to true so that
            this attribute can be returned later to determin if driver is in
            action.
        """
        print("driver on")
        self.driving = True

    def __del__(self):
        """ Deconstructor ensure we stop the motors if script ends or class no
            longer needed."""
        self.stop()

class MrBit_DS3_Driver(MrBit_Motor_Driver):
    """ Interfaced with DS3 controller (approxeng.input) to drive Mr. Bit
        from a PS3 game pad.
    """
    def __init__(self):
        self.pwmOffset = 255 - 55
        super(MrBit_DS3_Driver, self).__init__()

    def drive(self, leftSpeed, rightSpeed):
        speeds = (int(leftSpeed * self.pwmOffset), int(rightSpeed * self.pwmOffset))
        super(MrBit_DS3_Driver, self).drive(*speeds)

class MrBit_Auto_Driver(MrBit_Motor_Driver):
    """ Super class for auto driver challenges - auto challenges use MrBit_PID"""

    def __init__(self, Setpoint,  Kp,  Ki,  Kd, Input, OutputLeft, OutputRight, ControllerDirectionLeft, ControllerDirectionRight):
        """ Creates an instance of Auto Driver which creates two instances of the
            PID controllers, one for each motor

            :param Setpoint:
                Is the position where Mr Bit should be moved to.

            :param Kp:
                The proportional constant used by the PID controller.

            :param Ki:
                The integral constant used by the PID controller.

            :param Kd:
                The derivitive constant used by the PID controller.

            :param Input:
                The current position to initialise the pid

            :param OutputLeft:
                The inital and desired output from the PID controller for the
                left motor.

            :param OutputRight:
                The inital and desired output from the PID controller for the
                right motor.

            :param ControllerDirectionLeft:
                The direction of the pid controller for the left motor (not the
                direction of the motor, see MrBit_PID).

            :param ControllerDirectionRight:
                The direction of the pid controller for the right motor (not the
                direction of the motor, see MrBit_PID).
        """
        self.leftPid = pid.MrBit_PID(Setpoint,  Kp,  Ki,  Kd, Input, OutputLeft, ControllerDirectionLeft)
        self.rightPid = pid.MrBit_PID(Setpoint,  Kp,  Ki,  Kd, Input, OutputRight, ControllerDirectionRight)
        super(MrBit_Auto_Driver, self).__init__()

    def drive(self):
        """ Called to drive Mr Bit through the automated challenges.
        """
        self.set_position()
        self.leftPid.compute(self.position)
        self.rightPid.compute(self.position)
        leftSpeed = self.leftPid.get_output()
        rightSpeed = self.rightPid.get_output()
        super(MrBit_Auto_Driver, self).drive(leftSpeed, rightSpeed)

    def stop(self):
        """ Called to stop the PID and the motors of Mr. Bit, typically used
            when switching from automatic driving to manual.
        """
        self.leftPid.set_mode(self.leftPid.MANUAL)
        self.rightPid.set_mode(self.rightPid.MANUAL)
        super(MrBit_Auto_Driver, self).stop()

    def start(self):
        """ Starts the auto driver, typically used when switching from manual
            driving to automatic."""
        self.set_position()
        self.leftPid.set_output(self.baseSpeed)
        self.rightPid.set_output(self.baseSpeed)
        self.leftPid.set_mode(self.leftPid.AUTOMATIC)
        self.rightPid.set_mode(self.rightPid.AUTOMATIC)
        super(MrBit_Auto_Driver, self).start()

    def set_constants(self):
        """ Alters the constants of the PID controllers, the user is required to
            enter in new values for each constant."""
        Kp = input("Enter a Kp val: ")
        Ki = input("Enter a Ki val: ")
        Kd = input("Enter a Kd val: ")
        self.leftPid.set_tunings(Kp, Ki, Kd)
        self.rightPid.set_tunings(Kp, Ki, Kd)

    def set_position(self):
        raise NotImplementedError

class MrBit_Straight_Line(MrBit_Auto_Driver):
    """ Class for the straight line challenge for PiWars."""

    def __init__(self):
        """ Creates an instance of Straight Line, sets the constants for the PID
            controllers and creates instances of required sensors.
        """
        self.kp = 0.6
        self.ki = 0.05
        self.kd = 0.1
        self.baseSpeed = 200
        self.setpoint = 0
        self.leftIRSensor = MrBit_IR_Distance_Sensor(17)
        self.rightIRSensor = MrBit_IR_Distance_Sensor(16)
        self.set_position()
        super(MrBit_Straight_Line, self).__init__(self.setpoint,  self.kp,  self.ki, self.kd, self.position, self.baseSpeed, self.baseSpeed, pid.MrBit_PID.REVERSE, pid.MrBit_PID.DIRECT)

    def set_position(self):
        """ Determins current position of Mr. Bit to send to the PID controllers.
        """
        leftIRReading = self.leftIRSensor.getReading()
        rightIRReading = self.rightIRSensor.getReading()
        #print("readings L: %d, R: %d" %(leftIRReading, rightIRReading))
        self.position = leftIRReading - rightIRReading


class MrBit_Minimal_Maze(MrBit_Auto_Driver):
    """ Class for the Minimal Maze challenge for PiWars."""

    def __init__(self):
        """ Creates an instance of Minimal Maze, sets the constants for the PID
            controllers and creates instances of required sensors.
        """
        #0.5 0 0.005, frontReading*2
        self.kp = 0.55
        self.ki = 0.0
        self.kd = 0.001
        self.baseSpeed = 50
        self.setpoint = 250
        self.wallIRSensor = MrBit_IR_Distance_Sensor(17)
        self.frontIRSensor = MrBit_IR_Distance_Sensor(16)
        self.set_position()
        leftDirection = pid.MrBit_PID.REVERSE
        rightDirection = pid.MrBit_PID.DIRECT
        super(MrBit_Minimal_Maze, self).__init__(self.setpoint,  self.kp,  self.ki, self.kd, self.position, self.baseSpeed, self.baseSpeed, leftDirection, rightDirection)
        self.leftPid.set_output_limits(-75, 125)
        self.rightPid.set_output_limits(-75, 125)

    def set_position(self):
        """ Determins current position of Mr. Bit to send to the PID controllers.
        """
        wallReading = self.wallIRSensor.getReading()
        frontReading = self.frontIRSensor.getReading()
        self.position = wallReading
        if frontReading > 200 and wallReading > 125: self.position += frontReading*2
        print("W: %d, F: %d, Pos: %d" % (wallReading, frontReading, self.position))


class MrBit_Line_Following(MrBit_Auto_Driver):
    """ Class for the Line Following challenge for PiWars."""

    def __init__(self):
        """ Creates an instance of Line Following, sets the constants for the PID
            controllers and creates instances of required sensors.
        """
        self.kp = 0.1
        self.ki = 0
        self.kd = 0
        self.baseSpeed = 50
        self.qtr8rc = qtr.MrBit_QTR_8RC()
        self.setpoint = 3500
        self.calibrate()
        self.set_position()
        super(MrBit_Line_Following, self).__init__(self.setpoint,  self.kp,  self.ki,  self.kd, self.position, self.baseSpeed, self.baseSpeed, pid.MrBit_PID.DIRECT, pid.MrBit_PID.REVERSE)
        self.leftPid.set_output_limits(-255, 255)
        self.rightPid.set_output_limits(-255, 255)

    def set_position(self):
        """ Determins current position of Mr. Bit to send to the PID controllers.
        """
        self.qtr8rc.emitters_on()
        self.position = self.qtr8rc.read_line()
        self.qtr8rc.emitters_off()
        print(self.position)

    def calibrate(self):
        """ Calibration of sensors, rotates robot several times over sensors
            and takes a reading.  Readings displayed and need to be accepted
            by user before continuing, if not accepted Mr Bit repeats calibration.
        """
        manualDriver = MrBit_Motor_Driver()
        approveCal = False
        while not approveCal:
            print("calibrating")
            self.qtr8rc.initialise_calibration()
            self.qtr8rc.emitters_on()
            for t in range(0, 4):
                for i in range(0, 10):
                    if t % 2 ==  0:
                        manualDriver.drive(100, -100)
                    else:
                        manualDriver.drive(-100, 100)
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
        """ Ensures the QTR emitters are turned off.
        """
        self.qtr8rc.emitters_off()
        super(MrBit_Line_Following, self).stop()


class MrBit_IR_Distance_Sensor:
    """ Class for a single analog Sharp IR distance sensor,
        makes use of a global grovepi library.
    """

    def __init__(self, Pin):
        """ Creates an instance of IR Distance sensor.

            :param Pin:
                Is the pin which the sensor is connected to.
        """
        self.irPin = Pin
        gp.pinMode(self.irPin, "INPUT")

    def getPin(self): return self.irPin

    def getReading(self):
        """ Returns the current reading of the IR sensor.
        """
        return gp.analogRead(self.irPin)

class MrBit_Controller:
    """ Main driver factory to create different drivers for the different
        piWars challenges.
    """

    AUTOMATIC = 0
    MANUAL = 1

    def __init__(self, Mode=1):
        """ Creates an instance of Controller and defines the different drivers.

            :param Mode:
                Sets the mode Mr Bit should start up in, either manual or auto.
        """
        self.drivers = dict(MD=MrBit_DS3_Driver, SL=MrBit_Straight_Line, MM=MrBit_Minimal_Maze, LF=MrBit_Line_Following)
        self.set_mode(Mode)

    def toggle_mode(self):
        """ Toggles between automatic and manual so that Mr. Bit can be driven
            manually once after completing an automated challenge and vice-versa.
        """
        mode = MrBit_Controller.AUTOMATIC if self.mode == MrBit_Controller.MANUAL else MrBit_Controller.MANUAL
        print("mode: %d" % self.mode)
        self.set_mode(mode)

    def get_mode(self):
        """ Returns Mr Bit's current mode (Automatic or Manual).
        """
        return self.mode

    def set_mode(self, Mode):
        """ sets the mode for Mr Bit, if the Mode is Manual then the Manual Driver
            is loaded, if it is automatice then a challenge needs to be set.

            :param Mode:
                Sets the mode Mr Bit should start up in, either manual or auto.
        """
        self.driverReady = False
        if Mode != MrBit_Controller.MANUAL and Mode != MrBit_Controller.AUTOMATIC: return
        self.mode = Mode
        if self.mode == MrBit_Controller.MANUAL:
            self.change_driver("MD")
        elif self.mode == MrBit_Controller.AUTOMATIC:
            self.set_challenge()

    def set_challenge(self):
        """ Prompts the user for a challenge, if the challenge is valid then
            the corresponding driver will be loaded.
        """
        validChallenge = False
        while not validChallenge:
            Challenge = raw_input("Set challenge: ")
            if Challenge in self.drivers: validChallenge = True
        self.change_driver(Challenge)

    def change_driver(self, Driver):
        """ Loads the driver for either manual driving or one of the automated
            challenges.

            :param Driver:
                Needs to be of a value set up in the constructor corresponding to
                a challenge (or manual).
        """
        if Driver not in self.drivers: return
        if hasattr(self, 'driver'):
            del self.driver
        print("changing driver to %s:" % Driver)
        self.driver = self.drivers[Driver]()
        self.driverReady = True
        self.driver.start()

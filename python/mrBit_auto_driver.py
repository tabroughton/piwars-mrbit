import mrBit_pid as pid
import mrBit_moto_driver as driver
import grovepi as gp
from time import sleep

class mrBit_auto_driver(object):

    def __init__(self, Setpoint, Kp, Ki, Kd, Input, BasePWM, SampleTime):

        self.motLeft = driver.MOT_1
        self.motRight = driver.MOT_2
        self.mrBitActive = False
        self.basePWM = BasePWM
        self.setpoint = Setpoint
        self.inval = Input
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sampleTime = SampleTime

        self.lPid = pid.mrBit_pid(self.setpoint, self.Kp, self.Ki, self.Kd, self.inval, self.basePWM, 1, self.sampleTime)
        self.rPid = pid.mrBit_pid(self.setpoint, self.Kp, self.Ki, self.Kd, -self.inval, self.basePWM, 1, self.sampleTime)


    def setOutput(self):
        self.lPid.SetOutput(self.basePWM)
        self.rPid.SetOutput(self.basePWM)

    def drive(self, Input):
        if self.mrBitActive == False: return
        self.inval = Input
        self.lPid.Compute(self.inval)
        self.rPid.Compute(-self.inval)
        pwmLeft = self.lPid.GetOutput()
        pwmRight = self.rPid.GetOutput()
        print("error left: %d => left pwm: %d, right pwm: %d" % (self.inval, pwmLeft, pwmRight))
        driver.motorGo(driver.MOT_1, driver.MOTOR_CW, int(pwmLeft))
        driver.motorGo(driver.MOT_2, driver.MOTOR_CW, int(pwmRight))

    def autoDriveActivate(self):
        if self.mrBitActive: return
        print("robot straight line go")
        self.setOutput()
        self.lPid.SetMode(self.lPid.AUTOMATIC)
        self.rPid.SetMode(self.rPid.AUTOMATIC)
        self.mrBitActive = True

    def autoDriveStop(self):
        if self.mrBitActive == False: return
        print("robot straight line stop")
        self.lPid.SetMode(self.lPid.MANUAL)
        self.rPid.SetMode(self.rPid.MANUAL)
        driver.motorsStopDead()
        self.mrBitActive = False

    def editPIDConstants(self):
        self.autoDriveStop()
        #ToDo: checks on input vals
        Kp = input("Enter a Kp val: ")
        self.Kp = Kp
        Ki = input("Enter a Ki val: ")
        self.Ki = Ki
        Kd = input("Enter a Kd val: ")
        self.Kd = Kd
        self.lPid.SetTunings(self.Kp, self.Ki, self.Kd)
        self.rPid.SetTunings(self.Kp, self.Ki, self.Kd)
        print("new constants set Kp: %f Ki: %f Kd: %f" % (self.Kp, self.Ki, self.Kd))

class mrBit_straight_line(mrBit_auto_driver):

    def __init__(self, SampleTime):
        self.basePWM = 50
        self.irLeft = 0
        self.irRight = 1
        self.irPins = (16,17) #(A2,A3)
        self.setpoint = 0
        self.Kp = 2
        self.Ki = 0
        self.Kd = 0.3
        self.sampleTime = SampleTime

        gp.pinMode(self.irLeft, "INPUT")
        gp.pinMode(self.irRight, "INPUT")
        self.setInputForPID()

        super(mrBit_straight_line, self).__init__(self.setpoint, self.Kp, self.Kd, self.Ki, self.inval, self.basePWM, self.sampleTime)

    def setInputForPID(self):
        leftReading = gp.analogRead(self.irPins[self.irLeft])
        rightReading = gp.analogRead(self.irPins[self.irRight])
        #print ("IR LEFT: %d, IR RIGHT: %d" % (leftReading, rightReading))
        self.inval = leftReading - rightReading

    def drive(self):
        self.setInputForPID()
        super(mrBit_straight_line, self).drive(self.inval)


class mrBit_wall_follower(mrBit_auto_driver):

    def __init__(self, SampleTime):

        self.basePWM = 50
        self.irLeft = 0
        self.irFront = 1
        self.irPins = (16,17) #(A2,A3)
        self.setpoint = 0
        self.Kp = 2
        self.Ki = 0
        self.Kd = 0.05
        self.sampleTime = SampleTime

        gp.pinMode(self.irLeft, "INPUT")
        gp.pinMode(self.irFront, "INPUT")
        self.setInputForPID()

        super(mrBit_wall_follower, self).__init__(self.setpoint, self.Kp, self.Kd, self.Ki, self.inval, self.basePWM, self.sampleTime)

    def setInputForPID(self):
        leftReading = gp.analogRead(self.irPins[self.irLeft])
        reading = leftReading
        #print(frontReading)
        self.inval = reading -100

    def drive(self):
        frontReading = gp.analogRead(self.irPins[self.irFront])
        #print("front: %d" % frontReading)
        if frontReading > 200:
            self.autoDriveStop()
        else:
            self.setInputForPID()
            super(mrBit_wall_follower, self).drive(self.inval)

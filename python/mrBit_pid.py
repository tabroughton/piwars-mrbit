#   Python PID library
#   Atuhor: Tom Broughton (@dpolymath)
#   Date: 10/03/2017
#
#   A library to control output via PID,
#   inspired tutorials and C++ code by Brett Beauregard brettbeauregard.com
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
import time

class MrBit_PID:
    """A library to control output via PID"""
    AUTOMATIC = 1
    MANUAL = 0
    DIRECT = 0
    REVERSE = 1

    @classmethod
    def millis(cls):
        """ returns time now in milliseconds """
        return int(round(time.time() * 1000))


    def __init__(self, Setpoint,  Kp,  Ki,  Kd, Input, Output,
                    ControllerDirection=0,
                    SampleTime=30,
                    MinOutput=0,
                    MaxOutput=255):
        """ Creates an instance of the PID.

            :param Setpoint:
                Is the position where Mr Bit should be moved to.

            :param Kp:
                The proportional constant used by the PID controller.

            :param Ki:
                The integral constant used by the PID controller.

            :param Kd:
                The derivitive constant used by the PID controller.

            :param Input:
                The inital input for the PID controller.

            :param Output:
                The inital and desired output from the PID controller.

            :param ControllerDirection:
                Can be either DIRECT (a +ve/-ve input results in +ve/-ve output)
                or REVERSE (a +ve/-ve input results in -ve/+ve output).

            :param SampleTime:
                The time in milliseconds a new output value should be calculated.

            :param MinOutput:
                The minimum output from the PID.

            :param MaxOutput:
                The maximimum output from the PID.

        """
        self.inAuto = True
        self.mySetpoint = Setpoint
        self.myInput = Input
        self.lastInput = 0
        self.set_output(Output)
        self.iTerm = self.myOutput
        self.sampleTime = SampleTime
        self.controller_direction = ControllerDirection
        self.set_tunings(Kp, Ki, Kd)
        self.set_output_limits(MinOutput, MaxOutput)
        self.lastTime = self.millis() - self.sampleTime


    def compute(self, Input):
        """ In the words of Brett Beauregard "This, as they say, is where the
            magic happens.""  This function should be called
            every time a loop of the process executes - for exaple each time we
            loop around a main processing application controlling a robot or other
            machine.  The function will decide for itself whether a new
            pid Output needs to be computed based on current input constants set,
            previous states and whether the sample time has been reached.
            Parameter Input = the current position from sensor readings.
            Returns true when the output is computed false when nothing has been
            done. Accessing new values from the pid comes from myOuput - Use
            get_output().

            :param Input:
                The current position.
        """
        self.myInput = Input
        if not self.inAuto:
            print("not in auto")
            return False
        now = self.millis()
        timeChange = (now - self.lastTime)
        if timeChange >= self.sampleTime:
            # Compute all the working error variables
            error = self.mySetpoint - self.myInput
            self.iTerm += (self.ki * error)
            if self.iTerm > self.outMax:
                self.iTerm = self.outMax
            elif self.iTerm < self.outMin:
                self.iTerm = self.outMin
            dInput = self.myInput - self.lastInput

            #compute PID Output
            output = (self.kp * error) + self.iTerm - (self.kd * dInput)
            if output > self.outMax:
                output = self.outMax
            elif output < self.outMin:
                output = self.outMin
            self.myOutput = output
            #print("input: %d, error: %d, output: %d" % (self.myInput, error, self.myOutput))

            #remember these for next PID loop
            self.lastInput = self.myInput
            self.lastTime = now
            return True
        else:
           return False


    def set_tunings(self, Kp, Ki, Kd):
        """ This function allows the controller's dynamic performance to be
            adjusted. It's called automatically from the constructor, but tunings
            can also be adjusted on the fly during normal operation (for example)
            when detecting cliff edges on sensor readings.

            :param Kp:
                The proportional constant used by the PID controller.

            :param Ki:
                The integral constant used by the PID controller.

            :param Kd:
                The derivitive constant used by the PID controller.

        """
        if Kp<0 or Ki<0 or Kd<0: return
        self.dispKp = Kp
        self.dispKi = Ki
        self.dispKd = Kd

        SampleTimeInSec = self.sampleTime / 1000.0
        self.kp = Kp
        self.ki = Ki * SampleTimeInSec
        self.kd = Kd / SampleTimeInSec

        if self.controller_direction == MrBit_PID.REVERSE:
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd
        print("Constants Set Kp: %f, Ki: %f, Kd: %f" % (self.kp, self.ki, self.kd))


    def update_controller_direction(ControllerDirection):
        """ The PID will either be connected to a DIRECT acting process
            (+Output leads to +Input) or a REVERSE acting process (+Output leads
            to -Input.)  we need to know which one, because otherwise we may
            increase the output when we should be decreasing.  This is managed
            via constructor and set tunings but when updating the direction we
            need to make sure the right constants are updated.

            :param ControllerDirection:
                The new controller direction, can be DIRECT or REVERSE.
        """
        if self.inAuto and self.controllerDirection != ControllerDirection:
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd
        self.controllerDirection = ControllerDirection


    def update_sample_time(self, NewSampleTime):
        """ updates the period, in Milliseconds, at which the calculation is
            performed only used if sampleTime is already set and not needing
            resetting.

            :param NewSampleTime:
                The sample time to update to in milliseconds.
        """
        if NewSampleTime > 0:
            ratio  = NewSampleTime / self.sampleTime
            self.ki *= ratio
            self.kd /= ratio
            self.sampleTime = NewSampleTime


    def set_output(self, Output):
        """ Manually set the output if needing to change (eg. speed of a motor)
            during operation.

            :param Output:
                The new output for the PID.
        """
        self.myOutput = Output
        #self.reinitialse()


    def set_output_limits(self, MinLimit, MaxLimit):
        """ Output limits can be set to ensure that the PID doesn't return values
            outside that which can be used or are desirable.  Default limits set
            in the Library are 255 max and 0, to match that of the PWM we are
            typically using (in Arduino and GrovePi+) cases for Mr Bit change.
            If using wiringpi you may want to set these limits to 1023.

            :param MinLimit:
                The value for the minimum output from the PID controller.

            :param MaxLimit
                The value for the maximimum output from thr PID controller.
        """
        if MinLimit >= MaxLimit: return
        self.outMin = MinLimit
        self.outMax = MaxLimit

        if self.inAuto:
            if self.myOutput > self.outMax:
                self.myOutput = self.outMax
            elif self.myOutput < self.outMin:
                self.myOutput = self.outMin

            if self.iTerm > self.outMax:
                self.iTerm = self.outMax
            elif self.iTerm < self.outMin:
                vself.outMin


    def set_mode(self, Mode):
        """ Allows the controller Mode to be set to manual (0) or Automatic
            (non-zero) - when the transition from manual to auto occurs,
            the controller is automatically initialized.

            :param Mode:
                Can be AUTOMATIC or MANUAL
        """
        if Mode == MrBit_PID.AUTOMATIC:
            self.inAuto = True
            #self.reinitialse()
        elif Mode == MrBit_PID.MANUAL:
            self.inAuto = False


    def reinitialse(self):
        """ Does all the things that need to happen to ensure a bumpless transfer
            from manual to automatic mode or when manually altering the output.
        """
        self.iTerm = self.myOutput
        self.lastInput = self.myInput
        if self.iTerm > self.outMax:
            self.iTerm = self.outMax
        elif self.iTerm < self.outMin:
            self.iTerm = self.outMin


    def GetKp(self): return self.dispKp
    def GetKi(self): return self.dispKi
    def GetKd(self): return self.dispKd
    def GetMode(self): return self.inAuto
    def GetDirection(self): return self.controllerDirection
    def get_output(self): return self.myOutput

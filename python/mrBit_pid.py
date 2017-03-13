"""
/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/
"""
import time

class mrBit_pid:
    AUTOMATIC = 1
    MANUAL = 0
    DIRECT = 0
    REVERSE = 1

    """
    millis returns milliseconds
    """
    def millis(self):
        return int(round(time.time() * 1000))

    """
    /*Constructor (...)*********************************************************
     *    The parameters specified here are those for for which we can't set up
     *    reliable defaults, so we need to have the user set them.
     ***************************************************************************/
    """

    def __init__(self, Setpoint,  Kp,  Ki,  Kd, Input=0, Output=0, ControllerDirection=1, SampleTime=100):

        self.myInput = Input
        self.myOutput = Output
        self.mySetpoint = Setpoint
        self.inAuto = False

        self.SetOutputLimits(0,255)			#default output limit corresponds to
        									    #the arduino pwm limits

        self.sampleTime = SampleTime
        print(self.sampleTime)					#default Controller Sample Time is 0.1 seconds

        self.SetControllerDirection(ControllerDirection)
        self.SetTunings(Kp, Ki, Kd)

        self.lastTime = self.millis() - self.sampleTime


    """
    /* Compute() **********************************************************************
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     **********************************************************************************/
    """
    def Compute(self, Input):
        if not self.inAuto: return False;
        now = self.millis()
        timeChange = (now - self.lastTime)
        if timeChange >= self.sampleTime:

            # Compute all the working error variables
            self.Input = Input
            error = self.mySetpoint - self.Input
            self.ITerm += (self.ki * error)
            if self.ITerm > self.outMax:
                self.ITerm = self.outMax
            elif self.ITerm < self.outMin:
                self.ITerm = self.outMin

            dInput = self.Input - self.lastInput

            #compute PID Output
            output = (self.kp * error) + self.ITerm - (self.kd * dInput)

            if output > self.outMax:
                output = self.outMax
            elif output < self.outMin:
                output = self.outMin

            self.myOutput = output

            #Remember some variables for next time
            self.lastInput = self.Input
            self.lastTime = now
            return True
        else:
           return False

    """
    /* SetTunings(...)*************************************************************
     * This function allows the controller's dynamic performance to be adjusted.
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation
     ******************************************************************************/
     """
    def SetTunings(self, Kp, Ki, Kd):

        if Kp<0 or Ki<0 or Kd<0: return

        self.dispKp = Kp
        self.dispKi = Ki
        self.dispKd = Kd

        SampleTimeInSec = self.sampleTime / 1000.0
        self.kp = Kp
        self.ki = Ki * SampleTimeInSec
        self.kd = Kd / SampleTimeInSec

        if self.controllerDirection == mrBit_pid.REVERSE:
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd

    """
    /* setOutput() **********************************
    * manually set the  Output.
    * ***********************************************
    """
    def SetOutput(self, Output):
        self.myOutput = Output

    """
    /* SetSampleTime(...) *********************************************************
     * sets the period, in Milliseconds, at which the calculation is performed
     ******************************************************************************/
    """
    def SetSampleTime(self,newSampleTime):
        if newSampleTime > 0:
            ratio  = newSampleTime / self.sampleTime
            self.ki *= ratio
            self.kd /= ratio
            self.sampleTime = newSampleTime


    """
    /* SetOutputLimits(...)****************************************************
     *     This function will be used far more often than SetInputLimits.  while
     *  the input to the controller will generally be in the 0-1023 range (which is
     *  the default already,)  the output will be a little different.  maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     **************************************************************************/
     """
    def SetOutputLimits(self, MinLimit, MaxLimit):
        if MinLimit >= MaxLimit: return
        self.outMin = MinLimit
        self.outMax = MaxLimit

        if self.inAuto:
           if self.myOutput > self.outMax:
               self.myOutput = self.outMax
           elif self.myOutput < self.outMin:
               self.myOutput = self.outMin

           if self.ITerm > self.outMax:
               self.ITerm = self.outMax
           elif self.ITerm < self.outMin:
               self.ITerm = self.outMin

    """
    /* SetMode(...)****************************************************************
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     ******************************************************************************/
    """
    def SetMode(self, mode):
        if mode == mrBit_pid.AUTOMATIC and self.inAuto == False:
            #we just went from manual to auto
            self.Initialize()
        self.inAuto = mode

    """
    /* Initialize()****************************************************************
     *	does all the things that need to happen to ensure a bumpless transfer
     *  from manual to automatic mode.
     ******************************************************************************/
    """
    def Initialize(self):
        self.ITerm = self.myOutput
        self.lastInput = self.myInput
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin

    """
    /* SetControllerDirection(...)*************************************************
     * The PID will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     ******************************************************************************/
     """
    def SetControllerDirection(self,Direction):
        if self.inAuto and Direction != self.controllerDirection:
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd
        self.controllerDirection = Direction;

    """
    /* Status Funcions*************************************************************
     * Just because you set the Kp=-1 doesn't mean it actually happened.  these
     * functions query the internal state of the PID.  they're here for display
     * purposes.  this are the functions the PID Front-end uses for example
     ******************************************************************************/
    """
    def GetKp(self): return self.dispKp
    def GetKi(self): return self.dispKi
    def GetKd(self): return self.dispKd
    def GetMode(self): return self.inAuto
    def GetDirection(self): return self.controllerDirection
    def GetOutput(self): return self.myOutput

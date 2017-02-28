"""
    Read the Pololu QTR-8RC IR sensor array via the pi
    Date: 26/02/2017
    Atuhor: Tom Broughton

    NOTE - THIS VERSION DOES NOT USE PWM FOR LEDON

    Hardware:
    * LEDON pin connected to GPIO 21
    * 3V3 to rail and GND
    * Pins 22 - 29 to each IR LED/phototransitor pair

    Higher values = darker surfaces
"""

import wiringpi
wiringpi.wiringPiSetup()

#Set up the constants
LEDON_PIN = 21
SENSOR_PINS = [22, 26, 23, 27, 24, 28, 25, 29]
NUM_SENSORS = len(SENSOR_PINS)
CHARGE_TIME = 10 #us to charge the capacitors
READINGS_PER_SECOND = 1
READING_TIMEOUT = 1000 #us if it takes longer, assume line is black

#list to store sensor values in
sensorValues = []
calibratedMax = []
calibratedMin = []
lastValue = 0

"""
function: initPins
-----------------
sets up the GPIO pins and also ensures the correct number of items in sensors values list
"""
def initPins():
    for pin in SENSOR_PINS:
        sensorValues.append(0)
        calibratedMax.append(0)
        calibratedMin.append(0)
        wiringpi.pullUpDnControl(pin, wiringpi.PUD_DOWN) #ensure when low these GPIO pins are pulled down
    wiringpi.pinMode(LEDON_PIN, wiringpi.OUTPUT)

"""
function: emittersOn
-----------------
turns the LEDON pin on so that the IR LEDs can be turned on.
note: if there is nothing wired to LEDON emitters will always be on
"""
def emittersOn():
    print("emitters on")
    wiringpi.digitalWrite(LEDON_PIN, wiringpi.HIGH)
    wiringpi.delayMicroseconds(200)

"""
function: emittersOff
-----------------
turns the LEDON pin off so that IR LEDs can't be turned on
"""
def emittersOff():
    print("emitters off")
    wiringpi.digitalWrite(LEDON_PIN, wiringpi.LOW)
    wiringpi.delayMicroseconds(200)

"""
function: printSensorValues
-----------------
Params: values - a list of sensor values to print
prints out the sensor and it's current recorded reading

"""
def printSensorValues(values):
    for i in range(0, NUM_SENSORS):
        print("sensor %d, reading %d" % (i, values[i]))

"""
function: resetCalibrationVals
-----------------
resets max and min (inverse)thresholds prior to calibration
"""
def resetCalibrationVals():
    for i in range(0, NUM_SENSORS):
        calibratedMax[i] = 0
        calibratedMin[i] = READING_TIMEOUT

"""
function: calibrateSensors
-----------------
Takes readings across all sensors and sets max and min readings
typical use of this function is to call several times with delay such that
a total of 5 seconds pass (i.e. 250 calls, with 20ms delays)
"""
def calibrateSensors():
    for j in range(0, 10):
        readSensors()
        for i in range(0, NUM_SENSORS):
            if calibratedMax[i] < sensorValues[i]:
                calibratedMax[i] = sensorValues[i]
            if calibratedMin[i] > sensorValues[i]:
                calibratedMin[i] = sensorValues[i]

def readLine():
    global lastValue
    readCalibrated()
    avg = 0
    summ = 0
    online = False
    for i in range(0, NUM_SENSORS):
        val = sensorValues[i]
        if val > 200: online = True
        if val > 50:
            avg += val * (i * 1000)
            summ +=  val

    if online == False:
        if lastValue < (NUM_SENSORS-1)*1000/2:
            return 0
        else:
            return (NUM_SENSORS-1)*1000

    lastValue = avg/summ
    return lastValue


def readCalibrated():
    readSensors()
    print("uncalibrated readings")
    printSensorValues(sensorValues)
    print("calibrated readings")
    for i in range(0, NUM_SENSORS):
        denominator = calibratedMax[i] - calibratedMin[i]
        val = 0
        if denominator != 0:
            val = (sensorValues[i] - calibratedMin[i]) * 1000 / denominator
        if val < 0: val = 0
        if val > 1000: val = 1000
        sensorValues[i] = val

    print("calibrated readings")
    printSensorValues(sensorValues)

"""
function: readSensors
-----------------
Follows the Pololu guidance and cpp implementation of reading sensors:
1. Set the I/O line to an output and drive it high.
2. Allow at least 10 us for the sensor output to rise.
3. Make the I/O line an input (high impedance).
4. Measure the time for the voltage to decay by waiting for the I/O line to go low.
"""
def readSensors():
    for i in range(0, NUM_SENSORS): sensorValues[i] = READING_TIMEOUT

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, wiringpi.OUTPUT)
        wiringpi.digitalWrite(sensorPin, wiringpi.HIGH)

    wiringpi.delayMicroseconds(CHARGE_TIME)

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, wiringpi.INPUT)
        wiringpi.digitalWrite(sensorPin, wiringpi.LOW) #important: ensure pins are pulled down

    startTime = wiringpi.micros()
    while wiringpi.micros() - startTime < READING_TIMEOUT:
        time = wiringpi.micros() - startTime
        for i in range(0, NUM_SENSORS):
            if wiringpi.digitalRead(SENSOR_PINS[i]) == 0 and time < sensorValues[i]:
                sensorValues[i] = time

initPins()

try:
    print("calibrating")
    resetCalibrationVals()
    emittersOn()
    for i in range(0, 250):
        calibrateSensors()
        wiringpi.delay(20)
    emittersOff
except KeyboardInterrupt:
    emittersOff()

print("calibration complete")
print("max vals")
printSensorValues(calibratedMax)
print("calibration complete")
print("min vals")
printSensorValues(calibratedMin)

try:
    while 1:
        emittersOn()
        print("read line")
        print(readLine())
        emittersOff()
        wiringpi.delay(1000/READINGS_PER_SECOND)

except KeyboardInterrupt:
    emittersOff()

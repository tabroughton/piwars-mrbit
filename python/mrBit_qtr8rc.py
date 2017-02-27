"""
    Read the Pololu QTR-8RC IR sensor array via the pi
    Date: 26/02/2017
    Atuhor: Tom Broughton

    Hardware:
    * LEDON pin connected to GPIO 1(BCM 18)
    * 3V3 to rail and GND
    * Pins 22 - 29 to each IR LED/phototransitor pair

    Higher values = darker surfaces
"""

import wiringpi
wiringpi.wiringPiSetup()

#Set up the constants
LEDON_PIN = 1 #attached to pwm pin on the pi
SENSOR_PINS = [22, 26, 23, 27, 24, 28, 25, 29]
NUM_SENSORS = len(SENSOR_PINS)

#can tinker with the following to obtain optimum results:
CHARGE_TIME = 10 #us to charge the capacitors
LEDON_PWM = 512 #reduce current over time, this val currently gives same readings as 1024
READINGS_PER_SECOND = 5
READING_TIMEOUT = 1000 #us if it takes longer, assume line is black

#list to store sensor values in
sensorValues = []

"""
function: initPins
-----------------
sets up the GPIO pins and also ensures the correct number of items in sensors values list
"""
def initPins():
    for pins in range(0, NUM_SENSORS):
        sensorValues.append(0)
        wiringpi.pullUpDnControl(sensorPin, wiringpi.PUD_DOWN) #ensure when low these GPIO pins are pulled down
    wiringpi.pinMode(LEDON_PIN, wiringpi.PWM_OUTPUT)
    wiringpi.pwmSetMode(wiringpi.PWM_MODE_BAL)

"""
function: emittersOn
-----------------
turns the LEDON pin on so that the IR LEDs can be turned on.
note: if there is nothing wired to LEDON emitters will always be on
"""
def emittersOn():
    print("emitters on")
    wiringpi.pwmWrite(LEDON_PIN, LEDON_PWM)
    wiringpi.delayMicroseconds(200)

"""
function: emittersOff
-----------------
turns the LEDON pin off so that IR LEDs can't be turned on
"""
def emittersOff():
    print("emitters off")
    wiringpi.pwmWrite(LEDON_PIN, wiringpi.LOW)
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
function: readSensors
-----------------
Follows the Pololu guidance and cpp implementation of reading sensors:
1. Set the I/O line to an output and drive it high.
2. Allow at least 10 μs for the sensor output to rise.
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
    while 1:
        emittersOn()
        readSensors()
        emittersOff() #TODO: submit query as to why to Pololu
        printSensorValues(sensorValues)
        wiringpi.delay(1000/READINGS_PER_SECOND)

except KeyboardInterrupt:
    emittersOff()

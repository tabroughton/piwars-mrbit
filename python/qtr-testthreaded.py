import wiringpi
import threading
from multiprocessing.dummy import Pool as ThreadPool

SENSOR_PINS = [22, 26, 23, 27, 24, 28, 25, 29]
readings = []
readingsThreaded = []

for pins in SENSOR_PINS:
    readings.append(0)
    readingsThreaded.append(0)

def readLED(sensor):
    wiringpi.pinMode(sensor, 1)
    wiringpi.digitalWrite(sensor, 1)

    wiringpi.delayMicroseconds(10)

    wiringpi.pinMode(sensor, 0)
    wiringpi.digitalWrite(sensor, 0)

    startTime = wiringpi.micros()
    reading = 0
    while wiringpi.digitalRead(sensor) == 1:
         reading = wiringpi.micros() - startTime

    return reading

def readArray():
    for sensor in range(0, len(SENSOR_PINS)): readings[sensor] = 0

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, 1)
        wiringpi.digitalWrite(sensorPin, 1)

    wiringpi.delayMicroseconds(10)

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, 0)
        wiringpi.digitalWrite(sensorPin, 0)

    startTime = wiringpi.micros()
    while wiringpi.micros() - startTime < 1000:
        for sensor in range(0, len(SENSOR_PINS)):
            if wiringpi.digitalRead(SENSOR_PINS[sensor]) == 1:
                readings[sensor] = wiringpi.micros() - startTime

def readArrayThreaded(sensorPin):
    readingsThreaded[SENSOR_PINS.index(sensorPin)] = readLED(sensorPin)


try:

    wiringpi.wiringPiSetup()
    while 1:
        wiringpi.pinMode(1, 2)
        wiringpi.pwmWrite(1, 255)

        timeStart = wiringpi.micros()

        print "individual"
        for sensor in range(0, len(SENSOR_PINS)):
            print("sensor %d, reading %d" % (sensor, readLED(SENSOR_PINS[sensor])))
        timeToUpdate = wiringpi.micros() - timeStart
        print("time to update %d" % timeToUpdate)

        timeStart = wiringpi.micros()
        print "array"
        readArray()
        for sensor in range(0, len(SENSOR_PINS)):
            print("sensor %d, reading %d" % (sensor, readings[sensor]))
        timeToUpdate = wiringpi.micros() - timeStart
        print("time to update %d" % timeToUpdate)


        timeStart = wiringpi.micros()
        print "multithreading array"
        # Make the Pool of workers
        pool = ThreadPool(8)
        # Open the urls in their own threads
        # and return the results
        results = pool.map(readArrayThreaded, SENSOR_PINS)
        #close the pool and wait for the work to finish
        pool.close()
        pool.join()
        for sensor in range(0, len(SENSOR_PINS)):
            print("sensor %d, reading %d" % (sensor, readingsThreaded[sensor]))

        timeToUpdate = wiringpi.micros() - timeStart
        print("time to update %d" % timeToUpdate)


        wiringpi.pwmWrite(1, 0)
        wiringpi.delay(500)

except KeyboardInterrupt:
    wiringpi.pwmWrite(1, 0)

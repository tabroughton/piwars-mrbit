import wiringpi
import thread
import threading
from multiprocessing.dummy import Pool as ThreadPool

SENSOR_PINS = [22, 26, 23, 27, 24, 28, 25, 29]
readings = []
readingLoops = []

CHARGE_TIME = 10
LEDON_PWM = 255
READINGS_PER_SECOND = 20
READING_TIMEOUT = 1000

for pins in SENSOR_PINS:
    readings.append(0)
    readingLoops.append(0)

def resetResults():
    for pins in SENSOR_PINS:
        readings[SENSOR_PINS.index(pins)] = 0
        readingLoops[SENSOR_PINS.index(pins)] = 0

def readLED(sensor):
    wiringpi.pinMode(sensor, 1)
    wiringpi.digitalWrite(sensor, 1)

    wiringpi.delayMicroseconds(CHARGE_TIME)

    wiringpi.pinMode(sensor, 0)
    wiringpi.digitalWrite(sensor, 0)

    startTime = wiringpi.micros()
    reading = 0
    loops = 0
    while wiringpi.digitalRead(sensor) == 1:
         readings[SENSOR_PINS.index(sensor)] = wiringpi.micros() - startTime
         readingLoops[SENSOR_PINS.index(sensor)] = readingLoops[SENSOR_PINS.index(sensor)] + 1



def readArray():
    for sensor in range(0, len(SENSOR_PINS)): readings[sensor] = 0

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, 1)
        wiringpi.digitalWrite(sensorPin, 1)

    wiringpi.delayMicroseconds(CHARGE_TIME)

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, 0)
        wiringpi.digitalWrite(sensorPin, 0)

    startTime = wiringpi.micros()
    while wiringpi.micros() - startTime < READING_TIMEOUT:
        for sensor in range(0, len(SENSOR_PINS)):
            if wiringpi.digitalRead(SENSOR_PINS[sensor]) == 1:
                readingLoops[sensor] = readingLoops[sensor] + 1
                readings[sensor] = wiringpi.micros() - startTime


def spawnThreads2():
    try:
       for pin in SENSOR_PINS:
           t = threading.Thread(target=readLED, args = (pin,))
           t.daemon = True
           t.start()
    except Exception as e:
       print str(e)

def printReadings():
    for sensor in range(0, len(SENSOR_PINS)):
        print("sensor %d, reading %d, loops %d" % (sensor, readings[sensor], readingLoops[sensor]))

def readIndividual():
    timeStart = wiringpi.micros()
    resetResults()
    print "individual"
    for sensor in range(0, len(SENSOR_PINS)):
        readLED(SENSOR_PINS[sensor])
    printReadings()
    timeToUpdate = wiringpi.micros() - timeStart
    print("time to update %d" % timeToUpdate)

def readParallel():
    timeStart = wiringpi.micros()
    resetResults()
    print "Parallel"
    readArray()
    printReadings()
    timeToUpdate = wiringpi.micros() - timeStart
    print("time to update %d" % timeToUpdate)

def readMapping():
    timeStart = wiringpi.micros()
    resetResults()
    print "mapping"
    # Make the Pool of workers
    pool = ThreadPool(8)
    # Open the urls in their own threads
    # and return the results
    results = pool.map(readLED, SENSOR_PINS)
    #close the pool and wait for the work to finish
    pool.close()
    pool.join()
    printReadings()
    timeToUpdate = wiringpi.micros() - timeStart
    print("time to update %d" % timeToUpdate)

def readThreading():
    timeStart = wiringpi.micros()
    resetResults()
    print "threading"
    spawnThreads2()
    printReadings()
    timeToUpdate = wiringpi.micros() - timeStart
    print("time to update %d" % timeToUpdate)


try:

    wiringpi.wiringPiSetup()
    while 1:
        wiringpi.pinMode(1, 2)
        wiringpi.pwmWrite(1, LEDON_PWM)
        #readIndividual()
        #readParallel()
        readIndividual()
        readParallel()
        readMapping()
        readThreading()

        wiringpi.pwmWrite(1, 0)
        wiringpi.delay(1000/READINGS_PER_SECOND)

except KeyboardInterrupt:
    wiringpi.pwmWrite(1, 0)

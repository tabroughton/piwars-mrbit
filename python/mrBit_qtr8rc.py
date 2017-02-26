import wiringpi
wiringpi.wiringPiSetup()

#Set up the constants
LEDON_PIN = 1 #attached to pwm pin on the pi
SENSOR_PINS = [22, 26, 23, 27, 24, 28, 25, 29]
NUM_SENSORS = len(SENSOR_PINS)
CHARGE_TIME = 10 #us to charge the capacitors
LEDON_PWM = 255 #reduce current over time, this val currently gives same readings as 1024
READINGS_PER_SECOND = 5 #1kHz at 1024 => 100mA, 0.1kHz at 1024 should => 10mA (assume with pwm at 512 this may even be lower)
READING_TIMEOUT = 2000 #us if it takes longer, assume line is black


sensorValues = []
for pins in range(0, NUM_SENSORS): sensorValues.append(0)

def emittersOn():
    print("emitters on")
    wiringpi.pinMode(LEDON_PIN, wiringpi.PWM_OUTPUT)
    wiringpi.pwmSetMode(wiringpi.PWM_MODE_BAL)
    wiringpi.pwmWrite(LEDON_PIN, LEDON_PWM)
    wiringpi.delayMicroseconds(200)

def emittersOff():
    print("emitters off")
    wiringpi.pwmWrite(LEDON_PIN, wiringpi.LOW)
    wiringpi.delayMicroseconds(200)

def printSensorValues(values):
    for i in range(0, NUM_SENSORS):
        print("sensor %d, reading %d" % (i, values[i]))

def readSensors():
    for i in range(0, NUM_SENSORS): sensorValues[i] = READING_TIMEOUT

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, wiringpi.OUTPUT)
        wiringpi.digitalWrite(sensorPin, wiringpi.HIGH)

    wiringpi.delayMicroseconds(CHARGE_TIME)

    for sensorPin in SENSOR_PINS:
        wiringpi.pinMode(sensorPin, wiringpi.INPUT)
        wiringpi.digitalWrite(sensorPin, wiringpi.LOW)
        wiringpi.pullUpDnControl(sensorPin, wiringpi.PUD_DOWN) #important: ensure pins are pulled down

    startTime = wiringpi.micros()
    while wiringpi.micros() - startTime < READING_TIMEOUT:
        time = wiringpi.micros() - startTime
        for i in range(0, NUM_SENSORS):
            if wiringpi.digitalRead(SENSOR_PINS[i]) == 0 and time < sensorValues[i]:
                sensorValues[i] = time


try:
    while 1:
        emittersOn()
        readSensors()
        readSensors()
        emittersOff() #TODO: submit query as to why to Pololu
        printSensorValues(sensorValues)
        wiringpi.delay(1000/READINGS_PER_SECOND)

except KeyboardInterrupt:
    emittersOff()

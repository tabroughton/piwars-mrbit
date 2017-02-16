import grovepi
import time

ledPin = 3
grovepi.pinMode(ledpin, "OUTPUT")

def blink(num=1):
    for i in range(0, num):
        grovepi.digitalWrite(ledPin, HIGH)
        sleep(1)
        grovepi.digitalWrite(ledPin, LOW)

def fade(num=1):
    for i in range(0, num):
        for pwm in range(0, 255): grovepi.analogWrite(ledPin, pwm)

try:
    blink(3)
    fade(3)
    blink(3)
except Exception as e:
    print str(e)

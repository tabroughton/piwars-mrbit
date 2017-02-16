import mrBit_moto_driver
import time

try:
    #test motors individually forwards and backwards.
    for mot in range(0, 2):
        for direct in range(0, 2):
            mrBit_moto_driver.motorGo(mot, direct)
            time.sleep(1)
            mrBit_moto_driver.motorGo(mot, direct, 0)

    #test motors together forwards and backwards.
    for direct in range(0, 2):
        mrBit_moto_driver.motorGo(0, direct)
        mrBit_moto_driver.motorGo(1, direct)
        time.sleep(1)
        mrBit_moto_driver.motorGo(0, direct, 0)
        mrBit_moto_driver.motorGo(1, direct, 0)

    #test PWM up and down, forwards and backwards.
    for direct in range(0, 2):
        for pwm in range(0, mrBit_moto_driver.PWM_MAX):
            mrBit_moto_driver.motorGo(0, direct, pwm)
            mrBit_moto_driver.motorGo(1, direct, pwm)
            time.sleep(0.01)
        for pwm in range(mrBit_moto_driver.PWM_MAX, 0, -1):
            mrBit_moto_driver.motorGo(0, direct, pwm)
            mrBit_moto_driver.motorGo(1, direct, pwm)
            time.sleep(0.01)

    mrBit_moto_driver.motorsOff()

except KeyboardInterrupt:
    mrBit_moto_driver.motorsOff()

import mrBit_driver
from time import sleep

controller = MrBit_Driver.MrBit_Controller(0)
while True:
    try:
        controller.driver.drive()
        sleep(0.02)
    except KeyboardInterrupt:
        controller.driver.stop()
        break
    except Exception as e:
        controller.driver.stop()
        print str(e)
        break

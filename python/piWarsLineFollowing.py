import mrBit_driver
from time import sleep
import traceback

controller = mrBit_driver.MrBit_Controller(0)
while True:
    try:
        controller.driver.drive()
        sleep(0.03)
    except KeyboardInterrupt:
        controller.driver.stop()
        break
    except Exception as e:
        controller.driver.stop()
        traceback.print_exc()
        break

import mrBit_auto_driver2
from approxeng.input.dualshock3 import DualShock3, CONTROLLER_NAMES
from approxeng.input.asyncorebinder import ControllerResource
from time import sleep

sl = mrBit_auto_driver2.mrBit_straight_line(0, 0, 0)
running = True
setNewConstants = False

def slActivate(button):
    sl.autoDriveActivate()

def slDeactivate(button):
    sl.autoDriveStop()

def setK(button):
    global setNewConstants
    setNewConstants = True

def stopScript(button):
    global running
    running = False


with ControllerResource(controller=DualShock3(dead_zone=0.1, hot_zone=0.2), device_name=CONTROLLER_NAMES) as joystick:

    # Bind the square button to call the set_axis_centres function
    joystick.buttons.register_button_handler(slActivate, joystick.BUTTON_TRIANGLE)
    joystick.buttons.register_button_handler(slDeactivate, joystick.BUTTON_SQUARE)
    joystick.buttons.register_button_handler(stopScript, joystick.BUTTON_CIRCLE)
    joystick.buttons.register_button_handler(setK, joystick.BUTTON_CROSS)

    #LOOP
    while running:
        try:
            if setNewConstants:
                sl.editPIDConstants()
                setNewConstants = False

            sl.drive()
            sleep(.05)

        except KeyboardInterrupt:
            sl.autoDriveStop()
            break

        except Exception as e:
            print str(e)
        	# Turn motors off
            sl.autoDriveStop()
            break
    sl.autoDriveStop()

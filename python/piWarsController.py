import mrBit_driver
from approxeng.input.dualshock3 import DualShock3, CONTROLLER_NAMES
from approxeng.input.asyncorebinder import ControllerResource
from time import sleep
import traceback
import mrBit_gripper_pipper

controller = mrBit_driver.MrBit_Controller(1)
gripperPipper = mrBit_gripper_pipper.MrBit_Gripper_Pipper()
running = True
autoStart = False
lastLAxis = 0
lastRAxis = 0


def toggle_mode(button):
    try:
        controller.toggle_mode()
    except:
        traceback.print_exc()

def set_constants(button):
    try:
        if controller.get_mode() == controller.AUTOMATIC: controller.driver.set_constants()
    except:
        traceback.print_exc()

def auto_start_stop(button):
    global autoStart
    if controller.get_mode() == controller.AUTOMATIC:
        if autoStart:
            controller.driver.stop()
        else:
            controller.driver.start()
        autoStart = not autoStart

def stop_script(button):
    global running
    running = False

with ControllerResource(controller=DualShock3(dead_zone=0.1, hot_zone=0.2), device_name=CONTROLLER_NAMES) as joystick:

    joystick.buttons.register_button_handler(toggle_mode, joystick.BUTTON_SELECT)
    joystick.buttons.register_button_handler(set_constants, joystick.BUTTON_CIRCLE)
    joystick.buttons.register_button_handler(auto_start_stop, joystick.BUTTON_TRIANGLE)
    #joystick.buttons.register_button_handler(stop_script, joystick.BUTTON_CROSS)

    while running:
        try:
            if controller.driverReady:
                mode = controller.get_mode()
                if mode == controller.MANUAL:
                    lAxis = joystick.AXIS_LEFT_VERTICAL.corrected_value()
                    rAxis = joystick.AXIS_RIGHT_VERTICAL.corrected_value()
                    controller.driver.drive(lAxis, rAxis)
                    gripTime = joystick.buttons.is_held(joystick.BUTTON_R1)
                    releaseTime = joystick.buttons.is_held(joystick.BUTTON_L1)
                    if gripTime is not None and releaseTime is None:
                        gripperPipper.grip()
                    elif releaseTime is not None and gripTime is None:
                        gripperPipper.release()
                elif mode == controller.AUTOMATIC:
                    if autoStart: controller.driver.drive()
                    sleep(0.05)
        except KeyboardInterrupt:
            #driver.stop()
            break

        except:
            traceback.print_exc()
            break
    if hasattr(controller, "driver"): controller.driver.stop()

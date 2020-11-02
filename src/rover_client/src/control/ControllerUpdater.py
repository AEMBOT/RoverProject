import rospy
import asyncio

import evdev
from evdev import InputDevice, ecodes

import threading

from rover_main.msg import controllerMap

STOPPING_THREAD = False

# Local dictionary of the state of the controller
CONTROLLER_DICT = {

    # Boolean Switches
    "Key_X" : False,
    "Key_Square" : False,
    "Key_Circle" : False,
    "Key_Triangle" : False,
    "Key_Select" : False,
    "Key_Start" : False,
    "Key_Dpad_Left" : False,
    "Key_Dpad_Right" : False,
    "Key_Dpad_Up" : False,
    "Key_Dpad_Down" : False,
    "Key_Bumper_Left" : False,
    "Key_Bumper_Right" : False,
    "Key_Joystick_Left" : False,
    "Key_Joystick_Right" : False,

    # Analog values
    "Joystick_LeftX" : 0.0,
    "Joystick_LeftY" : 0.0,
    "Joystick_RightX" : 0.0,
    "Joystick_RightY" : 0.0,
    "Trigger_Left" : 0.0,
    "Trigger_Right" : 0.0,

}

def threaded_loop(controller, loop):
    """Threaded async loop"""
    asyncio.set_event_loop(loop)
    print(controller)
    loop.run_until_complete(get_controller_states(controller))

def start_controller_monitor():
    """Start the asynced thread to register controller inputs"""
    
    print("Staring Controller Thread...")
    primary_controller = InputDevice(evdev.list_devices()[0])
    loop = asyncio.get_event_loop()
    thread = threading.Thread(target=threaded_loop, args=(primary_controller,loop,))
    thread.start()

@asyncio.coroutine
def get_controller_states(controller):

    global CONTROLLER_DICT
    global STOPPING_THREAD

    # Events are called when the button is pushed and when it is released
    for event in controller.read_loop():

        # If ROS was shutdown break out of the loop and exit
        if rospy.is_shutdown():
            break

        # If a normal button is pressed
        if event.type == ecodes.EV_KEY:

            # X / A
            if event.code == 304:
                CONTROLLER_DICT["Key_X"] = not CONTROLLER_DICT["Key_X"]

            # Circle / B
            if event.code == 305:
                CONTROLLER_DICT["Key_Circle"] = not CONTROLLER_DICT["Key_Circle"]

            # Triangle / X
            if event.code == 307:
                CONTROLLER_DICT["Key_Triangle"] = not CONTROLLER_DICT["Key_Triangle"]

            # Square / Y
            if event.code == 308:
                CONTROLLER_DICT["Key_Square"] = not CONTROLLER_DICT["Key_Square"]
            
            # Left Bumper
            if event.code == 310:
                CONTROLLER_DICT["Key_Bumper_Left"] = not CONTROLLER_DICT["Key_Bumper_Left"]

            # Right Bumper
            if event.code == 311:
                CONTROLLER_DICT["Key_Bumper_Right"] = not CONTROLLER_DICT["Key_Bumper_Right"]

            # Select
            if event.code == 314:
                CONTROLLER_DICT["Key_Select"] = not CONTROLLER_DICT["Key_Select"]

            # Start
            if event.code == 315:
                CONTROLLER_DICT["Key_Start"] = not CONTROLLER_DICT["Key_Start"]

            # Joystick Left Button
            if event.code == 317:
                CONTROLLER_DICT["Key_Joystick_Left"] = not CONTROLLER_DICT["Key_Joystick_Left"]

            # Joystick Right Button
            if event.code == 317:
                CONTROLLER_DICT["Key_Joystick_Right"] = not CONTROLLER_DICT["Key_Joystick_Right"]

        # If a joystick is moved / dpad is used
        if event.type == ecodes.EV_ABS:

            # Left Joystick X
            if event.code == 0:
                CONTROLLER_DICT["Joystick_LeftX"] = convert_joystick(event.value)

            # Left Joystick Y
            if event.code == 1:
                CONTROLLER_DICT["Joystick_LeftY"] = -convert_joystick(event.value)

            # Right Joystick X
            if event.code == 3:
                CONTROLLER_DICT["Joystick_RightX"] = convert_joystick(event.value)

            # Right Joystick Y
            if event.code == 4:
                CONTROLLER_DICT["Joystick_RightY"] = -convert_joystick(event.value)

            # Right Trigger
            if event.code == 9:
                CONTROLLER_DICT["Trigger_Right"] = convert_trigger(event.value)

            # Left Trigger
            if event.code == 10:
                CONTROLLER_DICT["Trigger_Left"] = convert_trigger(event.value)

            # X Dpad axis
            if event.code == 16:

                # Convert the analog values to digital
                if event.value < -0.2:
                    CONTROLLER_DICT["Key_Dpad_Left"] = False
                    CONTROLLER_DICT["Key_Dpad_Right"] = True
                elif event.value > 0.2:
                    CONTROLLER_DICT["Key_Dpad_Right"] = False
                    CONTROLLER_DICT["Key_Dpad_Left"] = True
                else:
                    CONTROLLER_DICT["Key_Dpad_Left"] = False
                    CONTROLLER_DICT["Key_Dpad_Right"] = False
            
            # Y Dpad axis
            if event.code == 17:

                # Convert the analog values to digital
                if event.value < -0.2:
                    CONTROLLER_DICT["Key_Dpad_Down"] = False
                    CONTROLLER_DICT["Key_Dpad_Up"] = True
                elif event.value > 0.2:
                    CONTROLLER_DICT["Key_Dpad_Up"] = False
                    CONTROLLER_DICT["Key_Dpad_Down"] = True
                else:
                    CONTROLLER_DICT["Key_Dpad_Down"] = False
                    CONTROLLER_DICT["Key_Dpad_Up"] = False
            

def getCurrentControllerState():
    controller_state = controllerMap()

    # Keys
    controller_state.Key_X = CONTROLLER_DICT["Key_X"]
    controller_state.Key_Square = CONTROLLER_DICT["Key_Square"]
    controller_state.Key_Circle = CONTROLLER_DICT["Key_Circle"]
    controller_state.Key_Triangle = CONTROLLER_DICT["Key_Triangle"]

    # Modifier Keys
    controller_state.Key_Select = CONTROLLER_DICT["Key_Select"]
    controller_state.Key_Start = CONTROLLER_DICT["Key_Start"]

    # Dpad
    controller_state.Dpad_Up = CONTROLLER_DICT["Key_Dpad_Up"]
    controller_state.Dpad_Down = CONTROLLER_DICT["Key_Dpad_Down"]
    controller_state.Dpad_Left = CONTROLLER_DICT["Key_Dpad_Left"]
    controller_state.Dpad_Right = CONTROLLER_DICT["Key_Dpad_Right"]

    # Bumpers
    controller_state.Bumper_Left = CONTROLLER_DICT["Key_Bumper_Left"]
    controller_state.Bumper_Right = CONTROLLER_DICT["Key_Bumper_Right"]

    # Joystick Buttons
    controller_state.Key_LeftJoystick = CONTROLLER_DICT["Key_Joystick_Left"]
    controller_state.Key_RightJoystick = CONTROLLER_DICT["Key_Joystick_Right"]

    # Analog
    controller_state.Joystick_LeftX = CONTROLLER_DICT["Joystick_LeftX"]
    controller_state.Joystick_LeftY = CONTROLLER_DICT["Joystick_LeftY"]
    controller_state.Joystick_RightX = CONTROLLER_DICT["Joystick_RightX"]
    controller_state.Joystick_RightY = CONTROLLER_DICT["Joystick_RightY"]

    # Triggers
    controller_state.Trigger_Left = CONTROLLER_DICT["Trigger_Left"]
    controller_state.Trigger_Right = CONTROLLER_DICT["Trigger_Right"]

    return controller_state

def convert_joystick(value):
    """Convert the joystick to ranges from -1 to 1"""
    return (value/32_768)

def convert_trigger(value):
    """Convert the trigger to ranges from 0 to 1"""
    return (value/255)
import rospy

import pygame

import threading

from rover_main.msg import controllerMap

CONTROLLER_TYPE = ""

# Local dictionary of the state of the controller
CONTROLLER_DICT = {

    # Boolean Switches
    "Key_X" : False,
    "Key_Square" : False,
    "Key_Circle" : False,
    "Key_Triangle" : False,
    "Key_Dpad_Left" : False,
    "Key_Dpad_Right" : False,
    "Key_Dpad_Up" : False,
    "Key_Dpad_Down" : False,
    "Key_Bumper_Left" : False,
    "Key_Bumper_Right" : False,

    # Analog values
    "Joystick_LeftX" : 0.0,
    "Joystick_LeftY" : 0.0,
    "Joystick_RightX" : 0.0,
    "Joystick_RightY" : 0.0,
    "Trigger_Left" : 0.0,
    "Trigger_Right" : 0.0,

}


def start_controller_monitor():
    """Start the asynced thread to register controller inputs"""
    
    rospy.loginfo("Staring Controller Thread...")

    # The launch file contains a parameter to set the correct controller type
    global CONTROLLER_TYPE
    CONTROLLER_TYPE = rospy.get_param("controller_type").strip()
       
    # Create and start the asynico thread
    thread = threading.Thread(target=get_controller_states, args=())
    thread.start()

def get_controller_states():

    # Create the pygame backend to manage the controller
    pygame.display.init()
    pygame.joystick.init()

    try:
        # Get the first controller in the InputDevices
        controller = pygame.joystick.Joystick(0)
    except Exception as e:
        rospy.loginfo("No Controller Detected: %s"%e)
        return

    # Refernce the global CONTROLLER_DICT to be used locally
    global CONTROLLER_DICT

    # Use the XBOX controller layout
    if CONTROLLER_TYPE == "xbox":
        
        # Events are called when the button is pushed and when it is released
        while not rospy.is_shutdown():

            # X / A
            CONTROLLER_DICT["Key_X"] = bool(controller.get_button(0))

            # Circle / B
            CONTROLLER_DICT["Key_Circle"] = bool(controller.get_button(1))

            # Triangle / X
            CONTROLLER_DICT["Key_Triangle"] = bool(controller.get_button(3))

            # Square / Y
            CONTROLLER_DICT["Key_Square"] = bool(controller.get_button(2))
            
            # Left Bumper
            CONTROLLER_DICT["Key_Bumper_Left"] = bool(controller.get_button(4))

            # Right Bumper
            CONTROLLER_DICT["Key_Bumper_Right"] = bool(controller.get_button(5))

            # Dpad-Parser (Left / Right)
            if controller.get_hat(0)[0] > 0.2:
                CONTROLLER_DICT["Key_Dpad_Right"] = True
                CONTROLLER_DICT["Key_Dpad_Left"] = False
            elif controller.get_hat(0)[0] < -0.2:
                CONTROLLER_DICT["Key_Dpad_Right"] = False
                CONTROLLER_DICT["Key_Dpad_Left"] = True
            else:
                CONTROLLER_DICT["Key_Dpad_Right"] = False
                CONTROLLER_DICT["Key_Dpad_Left"] = False

            # Dpad-Parser (Up / Down)
            if controller.get_hat(0)[1] > 0.2:
                CONTROLLER_DICT["Key_Dpad_Up"] = True
                CONTROLLER_DICT["Key_Dpad_Down"] = False
            elif controller.get_hat(0)[1] < -0.2:
                CONTROLLER_DICT["Key_Dpad_Up"] = False
                CONTROLLER_DICT["Key_Dpad_Down"] = True
            else:
                CONTROLLER_DICT["Key_Dpad_Up"] = False
                CONTROLLER_DICT["Key_Dpad_Down"] = False



            # Left Joystick X
            CONTROLLER_DICT["Joystick_LeftX"] = controller.get_axis(0)
            

            # Left Joystick Y
            CONTROLLER_DICT["Joystick_LeftY"] = controller.get_axis(1)

            # Right Joystick X
            CONTROLLER_DICT["Joystick_RightX"] = controller.get_axis(3)

            # Right Joystick Y
            CONTROLLER_DICT["Joystick_RightY"] = controller.get_axis(4)

            # Right Trigger
            CONTROLLER_DICT["Trigger_Right"] = convert_trigger(controller.get_axis(5))

            # Left Trigger
            CONTROLLER_DICT["Trigger_Left"] = convert_trigger(controller.get_axis(2))

            # Register new events
            pygame.event.pump()
            

def getCurrentControllerState():
    """Takes the values from the dictionary and assigns them to a new controllerMap object"""

    controller_state = controllerMap()

    # Keys
    controller_state.Key_X = CONTROLLER_DICT["Key_X"]
    controller_state.Key_Square = CONTROLLER_DICT["Key_Square"]
    controller_state.Key_Circle = CONTROLLER_DICT["Key_Circle"]
    controller_state.Key_Triangle = CONTROLLER_DICT["Key_Triangle"]

    # Dpad
    controller_state.Dpad_Up = CONTROLLER_DICT["Key_Dpad_Up"]
    controller_state.Dpad_Down = CONTROLLER_DICT["Key_Dpad_Down"]
    controller_state.Dpad_Left = CONTROLLER_DICT["Key_Dpad_Left"]
    controller_state.Dpad_Right = CONTROLLER_DICT["Key_Dpad_Right"]

    # Bumpers
    controller_state.Bumper_Left = CONTROLLER_DICT["Key_Bumper_Left"]
    controller_state.Bumper_Right = CONTROLLER_DICT["Key_Bumper_Right"]

    # Analog
    controller_state.Joystick_LeftX = CONTROLLER_DICT["Joystick_LeftX"]
    controller_state.Joystick_LeftY = CONTROLLER_DICT["Joystick_LeftY"]
    controller_state.Joystick_RightX = CONTROLLER_DICT["Joystick_RightX"]
    controller_state.Joystick_RightY = CONTROLLER_DICT["Joystick_RightY"]

    # Triggers
    controller_state.Trigger_Left = CONTROLLER_DICT["Trigger_Left"]
    controller_state.Trigger_Right = CONTROLLER_DICT["Trigger_Right"]

    return controller_state

def convert_trigger(value):
    """Convert the trigger to ranges from 0 to 1"""
    if value < 0:
        value = 0
    return (value)
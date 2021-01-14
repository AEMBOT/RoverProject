import rospy

import pygame

import threading

from rover_main.msg import controllerMap

from control.controllers.Xbox import Xbox
from control.controllers.SimulatedXbox import SimulatedXbox


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
    
    # Select the correct controller layout
    if CONTROLLER_TYPE == "xbox":
        joystick = Xbox(controller)

    elif CONTROLLER_TYPE == "sim_xbox":
        joystick = SimulatedXbox(controller)

    # Refernce the global CONTROLLER_DICT to be used locally
    global CONTROLLER_DICT

    # Events are called when the button is pushed and when it is released
    while not rospy.is_shutdown():

        # Update the current joystick inputs
        joystick.update_controller(controller)

        # X / A
        CONTROLLER_DICT["Key_X"] = joystick.get_button_A()

        # Circle / B
        CONTROLLER_DICT["Key_Circle"] = joystick.get_button_B()

        # Triangle / X
        CONTROLLER_DICT["Key_Triangle"] = joystick.get_button_X()

        # Square / Y
        CONTROLLER_DICT["Key_Square"] = joystick.get_button_Y()
        
        # Left Bumper
        CONTROLLER_DICT["Key_Bumper_Left"] = joystick.get_left_bumper()

        # Right Bumper
        CONTROLLER_DICT["Key_Bumper_Right"] = joystick.get_right_bumper()

        # Dpad 

        # Right Dpad
        CONTROLLER_DICT["Key_Dpad_Right"] = joystick.get_dpad_right()

        # Left Dpad
        CONTROLLER_DICT["Key_Dpad_Left"] = joystick.get_dpad_left()
        
        # Up Dpad
        CONTROLLER_DICT["Key_Dpad_Up"] = joystick.get_dpad_up()

        # Down Dpad
        CONTROLLER_DICT["Key_Dpad_Down"] = joystick.get_dpad_down()


        # Left Joystick X
        CONTROLLER_DICT["Joystick_LeftX"] = joystick.get_joystick_left_x()
        

        # Left Joystick Y
        CONTROLLER_DICT["Joystick_LeftY"] = joystick.get_joystick_left_y()

        # Right Joystick X
        CONTROLLER_DICT["Joystick_RightX"] = joystick.get_joystick_right_x()

        # Right Joystick Y
        CONTROLLER_DICT["Joystick_RightY"] = joystick.get_joystick_right_y()

        # Right Trigger
        CONTROLLER_DICT["Trigger_Right"] = joystick.get_right_trigger()

        # Left Trigger
        CONTROLLER_DICT["Trigger_Left"] = joystick.get_left_trigger()

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

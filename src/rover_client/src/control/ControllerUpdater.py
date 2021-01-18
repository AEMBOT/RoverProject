import rospy

import pygame

import threading

from rover_main.msg import controllerMap

from control.controllers.Xbox import Xbox
from control.controllers.SimulatedXbox import SimulatedXbox

from time import sleep


CONTROLLER_TYPE = ""
CONTROLLER_CONNECTED = False

# Holds the current state of the controller
CONTROLLER_STATE = controllerMap()

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

    global CONTROLLER_CONNECTED

    # If no controller was found on start idle until a controller is connected
    while not CONTROLLER_CONNECTED and not rospy.is_shutdown():            
        rospy.loginfo("Searching For Controller...")
        try:
            # Get the first controller in the InputDevices
            controller = pygame.joystick.Joystick(0)
            rospy.loginfo("Controller Found!")
            CONTROLLER_CONNECTED = True
        except Exception as e:
            rospy.logerr("No Controller Detected: %s, Retrying in 2 seconds..."%e)
            sleep(2)

            # Update pygame
            pygame.event.pump()

    # Select the correct controller layout
    if CONTROLLER_TYPE == "xbox":
        joystick = Xbox(controller)

    elif CONTROLLER_TYPE == "sim_xbox":
        joystick = SimulatedXbox(controller)

    # Get a global reference of the CONTROLLER_STATE variable
    global CONTROLLER_STATE

    # Events are called when the button is pushed and when it is released
    while not rospy.is_shutdown():

        # Check if the remote roscore is still running
        if not is_roscore_running():
            rospy.logfatal("Failed to connect to roscore! Exiting...")
            rospy.signal_shutdown("Lost Connection to ros master")

        # Update the current joystick inputs
        joystick.update_controller(controller)

        # X / A
        CONTROLLER_STATE.Key_X = joystick.get_button_A()

        # Circle / B
        CONTROLLER_STATE.Key_Square = joystick.get_button_B()

        # Triangle / X
        CONTROLLER_STATE.Key_Circle = joystick.get_button_X()

        # Square / Y
        CONTROLLER_STATE.Key_Triangle = joystick.get_button_Y()
        
        # Left Bumper
        CONTROLLER_STATE.Bumper_Left = joystick.get_left_bumper()

        # Right Bumper
        CONTROLLER_STATE.Bumper_Right = joystick.get_right_bumper()

        # Dpad 

        # Right Dpad
        CONTROLLER_STATE.Dpad_Right = joystick.get_dpad_right()

        # Left Dpad
        CONTROLLER_STATE.Dpad_Left = joystick.get_dpad_left()
        
        # Up Dpad
        CONTROLLER_STATE.Dpad_Up = joystick.get_dpad_up()

        # Down Dpad
        CONTROLLER_STATE.Dpad_Down = joystick.get_dpad_down()


        # Left Joystick X
        CONTROLLER_STATE.Joystick_LeftX = joystick.get_joystick_left_x()
        

        # Left Joystick Y
        CONTROLLER_STATE.Joystick_LeftY = joystick.get_joystick_left_y()

        # Right Joystick X
        CONTROLLER_STATE.Joystick_RightX = joystick.get_joystick_right_x()

        # Right Joystick Y
        CONTROLLER_STATE.Joystick_RightY = joystick.get_joystick_right_y()

        # Right Trigger
        CONTROLLER_STATE.Trigger_Right = joystick.get_right_trigger()

        # Left Trigger
        CONTROLLER_STATE.Trigger_Left = joystick.get_left_trigger()

        # Register new events
        pygame.event.pump()
            

def getCurrentControllerState():
    """Easy getter for the current controller state"""

    global CONTROLLER_STATE
    return CONTROLLER_STATE

def is_roscore_running():
    """Returns wether or not the roscore is running"""
    roscore_running = True
    try:
        # Get the proc. ID of the remote roscore, if it fails it means the remote core is dead
        rospy.get_master().getPid()
    except:
        roscore_running = False
    return roscore_running

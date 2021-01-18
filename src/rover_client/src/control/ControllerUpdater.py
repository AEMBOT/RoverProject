import rospy

import pygame

import threading

from rover_main.msg import controllerMap

from control.controllers.Xbox import Xbox
from control.controllers.SimulatedXbox import SimulatedXbox

from time import sleep

class ControllerUpdater:

    def __init__(self):

        # Type of controller supplied by the launch file
        self.controller_type = ""

        # Whether or not the controller is currently connected
        self.controller_connected = False

        # Holds the current state of the controller
        self.controller_state = controllerMap()

    def start_controller_monitor(self):
        """Start the asynced thread to register controller inputs"""
        
        rospy.loginfo("Staring Controller Thread...")

        # The launch file contains a parameter to set the correct controller type
        self.controller_type = rospy.get_param("controller_type").strip()
        
        # Create and start the asynico thread
        thread = threading.Thread(target=self.get_controller_states, args=())
        thread.start()

    def get_controller_states(self):

        # Create the pygame backend to manage the controller
        pygame.display.init()
        pygame.joystick.init()


        # If no controller was found on start idle until a controller is connected
        while not self.controller_connected and not rospy.is_shutdown():            
            rospy.loginfo("Searching For Controller...")
            try:
                # Get the first controller in the InputDevices
                controller = pygame.joystick.Joystick(0)
                rospy.loginfo("Controller Found!")
                self.controller_connected = True
            except Exception as e:
                rospy.logerr("No Controller Detected: %s, Retrying in 2 seconds..."%e)
                sleep(2)

                # Update pygame
                pygame.event.pump()

        # Select the correct controller layout
        if self.controller_type == "xbox":
            joystick = Xbox(controller)

        elif self.controller_type == "sim_xbox":
            joystick = SimulatedXbox(controller)


        # Events are called when the button is pushed and when it is released
        while not rospy.is_shutdown():

            # Check if the remote roscore is still running
            if not self.is_roscore_running():
                rospy.logfatal("Failed to connect to roscore! Exiting...")
                rospy.signal_shutdown("Lost Connection to ros master")

            # X / A
            self.controller_state.Key_X = joystick.get_button_A()

            # Circle / B
            self.controller_state.Key_Square = joystick.get_button_B()

            # Triangle / X
            self.controller_state.Key_Circle = joystick.get_button_X()

            # Square / Y
            self.controller_state.Key_Triangle = joystick.get_button_Y()
            
            # Left Bumper
            self.controller_state.Bumper_Left = joystick.get_left_bumper()

            # Right Bumper
            self.controller_state.Bumper_Right = joystick.get_right_bumper()

            # Dpad 

            # Right Dpad
            self.controller_state.Dpad_Right = joystick.get_dpad_right()

            # Left Dpad
            self.controller_state.Dpad_Left = joystick.get_dpad_left()
            
            # Up Dpad
            self.controller_state.Dpad_Up = joystick.get_dpad_up()

            # Down Dpad
            self.controller_state.Dpad_Down = joystick.get_dpad_down()


            # Left Joystick X
            self.controller_state.Joystick_LeftX = joystick.get_joystick_left_x()
            

            # Left Joystick Y
            self.controller_state.Joystick_LeftY = joystick.get_joystick_left_y()

            # Right Joystick X
            self.controller_state.Joystick_RightX = joystick.get_joystick_right_x()

            # Right Joystick Y
            self.controller_state.Joystick_RightY = joystick.get_joystick_right_y()

            # Right Trigger
            self.controller_state.Trigger_Right = joystick.get_right_trigger()

            # Left Trigger
            self.controller_state.Trigger_Left = joystick.get_left_trigger()

            # Register new events
            pygame.event.pump()
                

    def get_current_controller_state(self):
        """Easy getter for the current controller state"""

        return self.controller_state

    def is_roscore_running(self):
        """Returns wether or not the roscore is running"""
        roscore_running = True
        try:
            # Get the proc. ID of the remote roscore, if it fails it means the remote core is dead
            rospy.get_master().getPid()
        except:
            roscore_running = False
        return roscore_running

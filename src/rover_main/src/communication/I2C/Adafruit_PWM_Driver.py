# In reference to: https://www.adafruit.com/product/815

# Used to get IO information form the board
import busio

import numpy as np

# Unit test mock to allow the service to run even off of a compatible board
from unittest.mock import Mock

# Response format
from rover_main.srv import i2c_commsResponse

# Rospy interface
import rospy

# Basic python math
import math

# Lower level board driver
from adafruit_pca9685 import PCA9685

# Adds the functionality to control servos if need be
from adafruit_motor import servo

# Created bus representing bus 0
try:
    # Used to create an actual I2C bus
    from board import SCL, SDA
    I2C_BUS = busio.I2C(SCL, SDA)
except NotImplementedError:

    # Tells the user that an incomatable board is in use but then mock the I2C bus to allow the program to continue
    rospy.logfatal_once("Invalid Board, You are likely running this on a developement computer, Mocking I2C bus...")
    I2C_BUS = Mock(busio.I2C)

# Duty cycle constants
FULL_DUTY_CYCLE = 0xFFFF
HALF_DUTY_CYCLE = 0x7FFF

class AdafruitDriver:
    """Driver for the 16 Channel Adafruit PWM Board"""

    def __init__(self):

        # Driver board instance
        self.pwm_driver = PCA9685(i2c_bus=I2C_BUS)

        # Maps servo objects to their channels so they can easily be changed
        # Channel # , Servo
        self.servo_dict: dict = {}

    def interact(self, command):
        """Interaction with the device that calls the rest of the methods from the parsed results"""

        # Split the passed command to an array
        spilt_command = command.strip().split(" ")

        try:

            # Get the second item in the command as that is the action we are taking
            # Set the board frequency
            if spilt_command[1] == "set_freq":
                self.set_board_freq(frequency=int(spilt_command[2]))
                return i2c_commsResponse("ok")
                
            # Set the PWM range for servos
            elif spilt_command[1] == "set_range":
                self.set_pwm_range(channel=int(spilt_command[2]),
                                   min_pulse=int(spilt_command[3]),
                                   max_pulse=int(spilt_command[4]))
                return i2c_commsResponse("ok")

            # Set the angle of a servo
            elif spilt_command[1] == "set_angle":
                self.set_angle(channel=int(spilt_command[2]),
                               angle=float(spilt_command[3]))
                return i2c_commsResponse("ok")

            # Set the range of the servo in degrees
            elif spilt_command[1] == "set_angle_range":
                self.set_angle_range(channel=int(spilt_command[2]),
                                     angle=float(spilt_command[3]))
                return i2c_commsResponse("ok")

            # Set the power of a motor ranging from 1 to -1
            elif spilt_command[1] == "set_power":
                self.set_power(channel=int(spilt_command[2]),
                               power=float(spilt_command[3]))
                return i2c_commsResponse("ok")

        # If an exception occurred return that as the response, if not send 'ok'
        except Exception as e:
            return i2c_commsResponse("Exception Caught: " + str(e))

    def set_board_freq(self, frequency):
        """Set the acting frequency of the board"""

        self.pwm_driver.frequency = frequency

    def set_pwm_range(self, channel, min_pulse, max_pulse):
        """Set the PWM range for a servo"""

        # Create or set the value of the current channels servo
        self.servo_dict[channel] = servo.Servo(self.pwm_driver.channels[channel], min_pulse=min_pulse, max_pulse=max_pulse)

    def set_angle(self, channel, angle):
        """Set the angle of a servo"""

        # If that servo doesn't exist create one
        if not channel in self.servo_dict:
            self.servo_dict[channel] = servo.Servo(self.pwm_driver.channels[channel])

        # However, if it does just set the actuation range
        else:
            self.servo_dict[channel].angle = angle

    def set_angle_range(self, channel, angle):
        """Set the angle range of a servo"""

        # If that servo doesn't exist create one
        if not channel in self.servo_dict:
            self.servo_dict[channel] = servo.Servo(self.pwm_driver.channels[channel])

        # However, if it does just set the actuation range
        else:
            self.servo_dict[channel].actuation_range = angle
    
    def set_power(self, channel, power):
        """
        Set the duty cycle of the current channel
        
        Current setup assumes if the duty cycle is < half then we are running in reverse and vice versa
        """

        self.pwm_driver.channels[channel].duty_cycle = int((HALF_DUTY_CYCLE+(HALF_DUTY_CYCLE*power)))

   
   

   

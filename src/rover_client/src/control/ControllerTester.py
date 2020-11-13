#!/usr/bin/python3

"""Used to test different joystick inputs"""

import pygame

# Create the pygame backend to manage the controller
pygame.display.init()
pygame.joystick.init()

try:
    # Get the first controller in the InputDevices
    controller = pygame.joystick.Joystick(0)
except Exception as e:
    print("No Controller Detected: %s"%e)
    exit(-1)

while True:
    print("Button: " + str((controller.get_axis(4)+1)/2))
    pygame.event.pump()
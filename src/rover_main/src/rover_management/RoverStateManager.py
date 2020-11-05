"""Used to manage the current state of the rover, this class should not be an OOP class. All methods here should remain static"""

# Global variables to easily access and change
ROVER_ENABLED = False

class RoverControlState:
    """Manage the current control state of the rover, i.e enabled or what not"""

    @staticmethod
    def set_enabled_state(state: bool):
        """Globally set the status of the robot"""
        global ROVER_ENABLED
        ROVER_ENABLED = state
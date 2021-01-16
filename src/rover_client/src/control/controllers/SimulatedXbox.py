from control.controllers.Xbox import Xbox

class SimulatedXbox(Xbox):

    def __init__(self, controller):
        """
        controller: Reference to the previously created controller object to read data from
        """
        super().__init__(controller)


    # Analog / Joystick Inputs

    def get_joystick_left_x(self):
        return  super().get_controller().get_axis(0)

    def get_joystick_left_y(self):
        return -super().get_controller().get_axis(1)
    
    def get_joystick_right_x(self):
        return super().get_controller().get_axis(2)
    
    def get_joystick_right_y(self):
        return -super().get_controller().get_axis(3)

    def get_right_trigger(self):
         return super().remap(super().get_controller().get_axis(4), (-1, 1), (0, 1)) 
    
    def get_left_trigger(self):
        return super().remap(super().get_controller().get_axis(5), (-1, 1), (0, 1)) 

 
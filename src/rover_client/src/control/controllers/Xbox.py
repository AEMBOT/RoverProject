class Xbox:

    def __init__(self, controller):
        """
        controller: Reference to the previously created controller object to read data from
        """
        self.controller = controller

    # Button Inputs

    def get_button_A(self):
        return self.controller.get_button(0)
    
    def get_button_B(self):
        return self.controller.get_button(1)
    
    def get_button_X(self):
        return self.controller.get_button(3)
    
    def get_button_Y(self):
        return self.controller.get_button(2)
    
    def get_left_bumper(self):
        return self.controller.get_button(4)
    
    def get_right_bumper(self):
        return self.controller.get_button(5)

    
    # Dpad inputs

    def get_dpad_up(self):
        return self.controller.get_hat(0)[1] > 0.2
    
    def get_dpad_down(self):
        return self.controller.get_hat(0)[1] < -0.2

    def get_dpad_right(self):
        return self.controller.get_hat(0)[0] > 0.2
    
    def get_dpad_left(self):
        return self.controller.get_hat(0)[0] < -0.2
    
    def get_joystick_left_x(self):
        return self.controller.get_axis(0)

    def get_joystick_left_y(self):
        return self.controller.get_axis(1)
    
    def get_joystick_right_x(self):
        return self.controller.get_axis(3)
    
    def get_joystick_right_y(self):
        return self.controller.get_axis(4)

    def get_right_trigger(self):
        return self.remap(self.controller.get_axis(5), (-1, 1), (0, 1))
    
    def get_left_trigger(self):
        return self.remap(self.controller.get_axis(2), (-1, 1), (0, 1))
    
    def remap(self, old_val, old_range: tuple, new_range: tuple):
        """Remaps a range of numbers to fit that of a new range"""
        return (new_range[1] - new_range[0])*(old_val - old_range[0]) / (old_range[1] - old_range[0]) + new_range[0]

 